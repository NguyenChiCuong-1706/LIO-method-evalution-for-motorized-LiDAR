"""Timestamp-aware display estimators and offline motor CSV analysis.

Only rows carrying a fresh 0x9C or 0x92 response update an estimator.  Other
rows contain cached values for convenient GUI display and are deliberately not
counted as independent observations.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics


DISPLAY_FILTER_TAU_S = 0.25


@dataclass
class DisplaySpeedEstimator:
    """Build visualization-only speed estimates from fresh controller replies."""

    filter_tau_s: float = DISPLAY_FILTER_TAU_S
    filtered_rpm: float | None = None
    angle_rpm: float | None = None
    angle_interval_s: float | None = None
    angle_time_uncertainty_s: float | None = None
    _speed_time: float | None = None
    _angle_time: float | None = None
    _angle_degrees: float | None = None
    _angle_half_rtt: float = 0.0

    def update_speed(self, rpm: float, received_s: float) -> float:
        if self.filtered_rpm is None or self._speed_time is None or received_s <= self._speed_time:
            self.filtered_rpm = rpm
        else:
            dt = received_s - self._speed_time
            alpha = 1.0 - math.exp(-dt / self.filter_tau_s)
            self.filtered_rpm += alpha * (rpm - self.filtered_rpm)
        self._speed_time = received_s
        return self.filtered_rpm

    def update_angle(self, angle_degrees: float, received_s: float,
                     response_rtt_s: float = 0.0) -> float | None:
        # The exchange could have completed anywhere within its measured RTT.
        # Midpoint timestamps reduce systematic delay; the half-RTTs bound only
        # host request/response timing, not undocumented controller latency.
        half_rtt = max(0.0, response_rtt_s) / 2.0
        sample_time = received_s - half_rtt
        if self._angle_time is not None and sample_time > self._angle_time:
            self.angle_interval_s = sample_time - self._angle_time
            self.angle_time_uncertainty_s = self._angle_half_rtt + half_rtt
            self.angle_rpm = ((angle_degrees - self._angle_degrees) / 6.0 /
                              self.angle_interval_s)
        self._angle_time = sample_time
        self._angle_degrees = angle_degrees
        self._angle_half_rtt = half_rtt
        return self.angle_rpm


def _stats(values: list[float], target: float) -> dict:
    return {
        "samples": len(values),
        "mean_rpm": statistics.fmean(values),
        "mean_absolute_error_rpm": statistics.fmean(abs(v - target) for v in values),
        "standard_deviation_rpm": statistics.stdev(values) if len(values) > 1 else 0.0,
        "minimum_rpm": min(values),
        "maximum_rpm": max(values),
    }


def analyze_csv(path: Path, *, startup_exclusion_s: float = 0.5,
                filter_tau_s: float = DISPLAY_FILTER_TAU_S) -> dict:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    running = [row for row in rows if row.get("motor_state") == "RUNNING"]
    if not running:
        raise ValueError("CSV contains no RUNNING telemetry")
    targets = {float(row["requested_rpm"]) for row in running if row.get("requested_rpm")}
    if len(targets) != 1:
        raise ValueError("analysis requires one target RPM")
    target = targets.pop()
    estimator = DisplaySpeedEstimator(filter_tau_s)
    raw_values: list[float] = []
    filtered_values: list[float] = []
    angle_values: list[float] = []
    angle_intervals: list[float] = []
    fresh_9c = fresh_92 = 0
    for row in running:
        frame = row.get("raw_can_response", "")
        elapsed = float(row["elapsed_session_s"])
        timestamp = float(row["monotonic_timestamp"])
        if frame.startswith("9C "):
            fresh_9c += 1
            raw = float(row["actual_rpm_provisional"])
            filtered = estimator.update_speed(raw, timestamp)
            if elapsed >= startup_exclusion_s:
                raw_values.append(raw)
                filtered_values.append(filtered)
        elif frame.startswith("92 "):
            fresh_92 += 1
            angle = float(row["angle_degrees_provisional"])
            derived = estimator.update_angle(angle, timestamp)
            # Assign an interval estimate to its midpoint for startup exclusion.
            if (derived is not None and estimator.angle_interval_s is not None and
                    elapsed - estimator.angle_interval_s / 2 >= startup_exclusion_s):
                angle_values.append(derived)
                angle_intervals.append(estimator.angle_interval_s)
    if not raw_values or not angle_values:
        raise ValueError("CSV has insufficient fresh 0x9C/0x92 samples")
    final = rows[-1]
    return {
        "file": str(path.resolve()),
        "target_rpm": target,
        "startup_exclusion_s": startup_exclusion_s,
        "display_filter_tau_s": filter_tau_s,
        "all_csv_rows": len(rows),
        "running_rows": len(running),
        "fresh_0x9c_rows": fresh_9c,
        "fresh_0x92_rows": fresh_92,
        "raw_0x9c": _stats(raw_values, target),
        "filtered_0x9c_visualization_only": _stats(filtered_values, target),
        "angle_derived_provisional": {
            **_stats(angle_values, target),
            "mean_interval_s": statistics.fmean(angle_intervals),
            "minimum_interval_s": min(angle_intervals),
            "maximum_interval_s": max(angle_intervals),
            "timestamp_uncertainty": (
                "legacy CSV has receive timestamps but no exchange RTT; controller latency is unbounded"
            ),
        },
        "final_state": final.get("motor_state"),
        "final_stop_reason": final.get("event_stop_reason"),
        "interpretation": (
            "0x9C and 0x92 come from the same controller and are not independent physical verification"
        ),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--startup-exclusion", type=float, default=0.5)
    parser.add_argument("--filter-tau", type=float, default=DISPLAY_FILTER_TAU_S)
    args = parser.parse_args(argv)
    if args.startup_exclusion < 0 or args.filter_tau <= 0:
        parser.error("startup exclusion must be nonnegative and filter tau must be positive")
    print(json.dumps(analyze_csv(args.csv, startup_exclusion_s=args.startup_exclusion,
                                 filter_tau_s=args.filter_tau), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
