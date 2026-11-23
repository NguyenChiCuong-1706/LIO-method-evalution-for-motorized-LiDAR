"""Analyze saved motor logs and optional shaft-video measurements offline.

This module only reads files. It never opens the Waveshare adapter or a CAN bus.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import sys

from speed_analysis import EXPERIMENT_HARD_SPEED_RPM, classify_speed


FEEDBACK = re.compile(r"^feedback: .*?elapsed=([\d.]+)s .*?speed_raw=(-?\d+) "
                      r"encoder_raw=(\d+)")
REQUEST = re.compile(r"\brequested=([+-]?[\d.]+) RPM duration=([\d.]+)s")
RX_SPEED = re.compile(r"RX 0x141: (9C(?: [0-9A-F]{2}){7})$")


def analyze_log(contents: str) -> dict:
    """Read legacy or Phase 6 JSON lines without inferring physical speed."""
    requested = None
    duration = None
    samples = []
    structured = []
    last_raw_speed_frame = None
    motion_started = False
    in_motion_angle_frames = []
    for line in contents.splitlines():
        match = REQUEST.search(line)
        if match:
            requested, duration = map(float, match.groups())
        if " TX 0x141: A2 " in line:
            motion_started = True
        if " TX 0x141: 81 " in line:
            motion_started = False
        if motion_started and " RX 0x141: 92 " in line:
            in_motion_angle_frames.append(line.rsplit(": ", 1)[-1])
        match = RX_SPEED.search(line)
        if match:
            last_raw_speed_frame = match.group(1)
        if line.startswith("monitor_sample_json: "):
            structured.append(json.loads(line.split(": ", 1)[1]))
        match = FEEDBACK.match(line)
        if match:
            elapsed, raw, encoder = match.groups()
            samples.append({"elapsed_s_rounded": float(elapsed),
                            "speed_raw_dps": int(raw), "encoder_raw": int(encoder),
                            "raw_9c_hex": last_raw_speed_frame})
    if structured:
        samples = [{"elapsed_s": item["elapsed_since_speed_command_s"],
                    "speed_raw_dps": item["reported_speed_raw_dps"],
                    "encoder_raw": item["encoder_raw"],
                    "raw_9c_hex": item["feedback_can_data"],
                    "classification": item["category"],
                    "feedback_rtt_s": item.get("feedback_rtt_s")}
                   for item in structured]
    elif requested is not None:
        count = 0
        for item in samples:
            analysis = classify_speed(item["speed_raw_dps"], requested,
                                      item["elapsed_s_rounded"], count)
            count = analysis["consecutive_tracking_deviations"]
            item["classification"] = analysis["category"]
    for item in samples:
        item["speed_rpm_provisional"] = item["speed_raw_dps"] / 6
    return {
        "requested_rpm": requested, "requested_duration_s": duration,
        "hard_speed_limit_rpm_unchanged": EXPERIMENT_HARD_SPEED_RPM,
        "samples": samples,
        "peak_abs_speed_rpm_provisional": max((abs(item["speed_rpm_provisional"])
                                               for item in samples), default=None),
        "in_motion_0x92_raw_frames": in_motion_angle_frames,
        "angle_derived_speed_for_event": None,
        "angle_derived_speed_reason": "no timed bracketing 0x92 samples during motion"
        if len(in_motion_angle_frames) < 2 else
        "raw 0x92 frames lack monotonic timing; use speed-angle-diagnostic separately",
        "stop_acknowledged": "stop: acknowledged" in contents,
        "disable_acknowledged": "disable: acknowledged" in contents,
        "zero_speed_confirmed_by_can_only": "zero_speed_confirmed=True" in contents,
        "protection_state": "UNKNOWN",
        "physical_speed_verified": False,
        "note": "V2.35 speed scaling provisional for MHF9025-V3; two CAN fields are not independent physical proof",
    }


def analyze_video_csv(path: Path, *, fps: float, a2_frame: int | None,
                      angle_uncertainty_deg: float, sync_uncertainty_s: float | None,
                      can_samples: list[dict]) -> dict:
    """Compute independent average RPM from manually annotated output-shaft frames."""
    if (not math.isfinite(fps) or fps <= 0 or
            not math.isfinite(angle_uncertainty_deg) or angle_uncertainty_deg <= 0 or
            a2_frame is not None and a2_frame < 0 or
            sync_uncertainty_s is not None and
            (not math.isfinite(sync_uncertainty_s) or sync_uncertainty_s <= 0) or
            sync_uncertainty_s is not None and a2_frame is None):
        raise ValueError("video requires positive fps/angle uncertainty; sync uncertainty requires A2 frame")
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise ValueError("video CSV needs at least two frame,angle_deg rows")
    points = []
    for row in rows:
        frame = int(row["frame"])
        angle = float(row["angle_deg"])
        if frame < 0 or not math.isfinite(angle) or points and frame <= points[-1][0]:
            raise ValueError("video frames must increase strictly; angles must be finite")
        points.append((frame, angle))
    intervals = []
    for (first_frame, first_angle), (last_frame, last_angle) in zip(points, points[1:]):
        dt = (last_frame - first_frame) / fps
        dt_uncertainty = 2 / fps  # Conservative ±1 frame at each endpoint.
        delta = last_angle - first_angle
        rpm = delta / 360 / dt * 60
        bounds = None
        if dt > dt_uncertainty:
            possible = [angle / 360 / time_s * 60
                        for angle in (delta - 2 * angle_uncertainty_deg,
                                      delta + 2 * angle_uncertainty_deg)
                        for time_s in (dt - dt_uncertainty, dt + dt_uncertainty)]
            bounds = [min(possible), max(possible)]
        t0, t1 = ((first_frame - a2_frame) / fps, (last_frame - a2_frame) / fps) \
            if a2_frame is not None else (None, None)
        nearby = ([item for item in can_samples
                   if t0 - sync_uncertainty_s <= item.get("elapsed_s", item.get("elapsed_s_rounded", -1))
                   <= t1 + sync_uncertainty_s]
                  if a2_frame is not None and sync_uncertainty_s is not None else None)
        comparisons = ([{"can_elapsed_s": item.get("elapsed_s", item.get("elapsed_s_rounded")),
                         "can_reported_rpm_provisional": item["speed_raw_dps"] / 6,
                         "can_minus_video_average_rpm": item["speed_raw_dps"] / 6 - rpm,
                         "interpretation": "descriptive only: CAN instantaneous/controller value versus video interval average"}
                        for item in nearby] if nearby is not None else None)
        intervals.append({
            "first_frame": first_frame, "last_frame": last_frame,
            "angle_change_deg_from_video": delta,
            "duration_s": dt, "duration_uncertainty_s_conservative": dt_uncertainty,
            "average_output_shaft_rpm": rpm,
            "rpm_bounds_from_frame_and_angle_uncertainty": bounds,
            "time_from_visible_a2_line_s": [t0, t1],
            "can_samples_possibly_overlapping": nearby,
            "can_vs_video_descriptive_comparisons": comparisons,
            "comparison_note": "CAN samples are instantaneous/controller reported; video RPM averages this interval",
        })
    return {
        "source": str(path), "fps": fps, "visible_a2_line_frame": a2_frame,
        "angle_marking_uncertainty_deg_each_endpoint": angle_uncertainty_deg,
        "video_to_can_sync_uncertainty_s": sync_uncertainty_s,
        "a2_sync_note": "terminal TX line appears after host write; display and controller latency remain unknown"
        if a2_frame is not None else "no A2 video anchor; no time-aligned CAN comparison",
        "intervals": intervals,
        "independence": "visible output-shaft marker is independent of controller angle/speed values",
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--video-csv", type=Path)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--a2-frame", type=int)
    parser.add_argument("--angle-uncertainty-deg", type=float)
    parser.add_argument("--sync-uncertainty-s", type=float)
    args = parser.parse_args(argv)
    if args.video_csv and any(value is None for value in
                              (args.fps, args.angle_uncertainty_deg)):
        parser.error("--video-csv requires --fps and --angle-uncertainty-deg")
    if not args.video_csv and any(value is not None for value in
                                  (args.fps, args.a2_frame, args.angle_uncertainty_deg,
                                   args.sync_uncertainty_s)):
        parser.error("video parameters require --video-csv")
    try:
        report = analyze_log(args.log.read_text(encoding="utf-8"))
        if args.video_csv:
            report["video"] = analyze_video_csv(
                args.video_csv, fps=args.fps, a2_frame=args.a2_frame,
                angle_uncertainty_deg=args.angle_uncertainty_deg,
                sync_uncertainty_s=args.sync_uncertainty_s,
                can_samples=report["samples"])
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
