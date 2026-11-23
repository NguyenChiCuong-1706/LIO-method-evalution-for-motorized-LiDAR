"""Read-only MHF9025-V3 absolute-encoder diagnostic.

The default Phase 1 mode sends exactly one request for each of 0x90, 0x94,
0x92, and 0x9C. Phase 3 adds operator-driven absolute-recovery and multi-turn
tracking modes. The utility never enables, moves, stops, configures, or writes
the motor. V2.35 field interpretations are reported as provisional for V3.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Callable

from motor_can import CommunicationError, ResponseTimeout, decode_feedback
from waveshare_motor import WaveshareMotor


READ_COMMANDS = (
    (0x90, "encoder_information"),
    (0x94, "single_turn_angle"),
    (0x92, "multi_turn_angle"),
    (0x9C, "motor_speed_and_state"),
)
REQUEST_PAYLOAD = b"\x00" * 7
FORBIDDEN_COMMANDS = frozenset((0x19, 0x91, 0x31, 0x32, 0x80, 0x81, 0x88,
                                0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6))
CSV_FIELDS = (
    "timestamp_utc", "can_id", "command", "name", "status", "tx_hex", "rx_hex",
    "decoded_json", "compatibility", "uncertainty", "error",
)
COMPATIBILITY = "LK-TECH CAN V2.35 family layout; unverified for MHF9025-V3"
RECOVERY_ROTATED_CONFIRMATION = "POWER_OFF_ROTATION_COMPLETE"
RECOVERY_POWER_CONFIRMATION = "POWER_RESTORED"
TRACKING_CONFIRMATION = "MOTOR_DISABLED_POWERED_SAFE_TO_ROTATE"
PHASE3_READ_COMMANDS = frozenset((0x90, 0x94, 0x92, 0x9C))
PHASE3_CSV_FIELDS = (
    "sample_index", "request_timestamp_utc", "response_timestamp_utc",
    "request_elapsed_s", "response_elapsed_s", "can_id", "command", "name",
    "status", "tx_hex", "rx_hex", "decoded_json", "uncertainty", "error",
)


@dataclass(frozen=True)
class DecodedRead:
    values: dict
    uncertainty: str


def _validate_reply(command: int, reply: bytes) -> None:
    if len(reply) != 8:
        raise CommunicationError(
            f"malformed 0x{command:02X} response: expected DLC 8, received {len(reply)}")
    if reply[0] != command:
        raise CommunicationError(
            f"malformed 0x{command:02X} response: command byte is 0x{reply[0]:02X}")


def decode_encoder_90(reply: bytes) -> DecodedRead:
    _validate_reply(0x90, reply)
    u16 = lambda start: int.from_bytes(reply[start:start + 2], "little", signed=False)
    return DecodedRead(
        {
            "encoder_position_raw_u16": u16(2),
            "encoder_original_raw_u16": u16(4),
            "encoder_offset_raw_u16": u16(6),
            "reserved_byte_1": reply[1],
        },
        "Three uint16 fields follow V2.35 bytes 2-3, 4-5, and 6-7. "
        "Counts-per-revolution and offset persistence are not verified for MHF9025-V3.",
    )


def decode_single_turn_94(reply: bytes) -> DecodedRead:
    _validate_reply(0x94, reply)
    # MHF9025-V3 hardware captures put a uint32 value in bytes 4-7.  An older
    # V2.35 family layout puts uint16 in bytes 6-7.  Decode both and select only
    # when the observed layout is distinguishable; retain both raw candidates.
    v3_raw = int.from_bytes(reply[4:8], "little", signed=False)
    legacy_raw = int.from_bytes(reply[6:8], "little", signed=False)
    if v3_raw <= 35_999:
        raw = v3_raw
        layout = "MHF9025-V3 observed uint32 bytes 4-7"
        reserved = reply[1:4]
    else:
        raw = legacy_raw
        layout = "older V2.35 uint16 bytes 6-7 fallback"
        reserved = reply[1:6]
    warnings = []
    if any(reserved):
        warnings.append("reserved bytes for selected layout are nonzero")
    if raw > 35_999:
        warnings.append("value is outside the V2.35 0..35999 single-turn range")
    detail = "; ".join(warnings) if warnings else "reserved bytes and V2.35 range are consistent"
    return DecodedRead(
        {
            "single_turn_angle_raw": raw,
            "single_turn_angle_degrees_provisional": raw / 100.0,
            "selected_layout": layout,
            "mfh9025_v3_bytes_4_7_raw_u32": v3_raw,
            "older_v2_35_bytes_6_7_raw_u16": legacy_raw,
            "v2_35_range_check": detail,
        },
        "MHF9025-V3 captures use bytes 4-7 as uint32 little-endian at 0.01 "
        "degree/LSB; the older V2.35 bytes 6-7 uint16 candidate is retained. "
        "The value wraps at one turn; direction and absolute accuracy remain unverified.",
    )


def decode_multi_turn_92(reply: bytes) -> DecodedRead:
    _validate_reply(0x92, reply)
    raw = int.from_bytes(reply[1:8], "little", signed=True)
    return DecodedRead(
        {
            "multi_turn_angle_raw_s56": raw,
            "multi_turn_angle_degrees_provisional": raw / 100.0,
        },
        "V2.35 describes a signed accumulated angle at 0.01 degree/LSB, but the "
        "8-byte CAN frame exposes only seven angle bytes. This tool sign-extends bytes "
        "1-7 as signed 56-bit; MHF9025-V3 compatibility is unverified.",
    )


def decode_state_9c(reply: bytes) -> DecodedRead:
    _validate_reply(0x9C, reply)
    feedback = decode_feedback(reply)
    return DecodedRead(
        {
            "temperature_c_s8": feedback.temperature_c,
            "torque_current_raw_s16": feedback.torque_current_raw,
            "speed_raw_dps_s16": feedback.speed_dps,
            "speed_rpm_provisional": feedback.speed_rpm,
            "encoder_raw_u16": feedback.encoder_raw,
        },
        "V2.35 uses int8 temperature, int16 torque-current, int16 speed at "
        "1 degree/s per LSB, and uint16 encoder in bytes 6-7. Current scaling, "
        "encoder meaning, and full MHF9025-V3 state mapping are unverified.",
    )


DECODERS = {
    0x90: decode_encoder_90,
    0x94: decode_single_turn_94,
    0x92: decode_multi_turn_92,
    0x9C: decode_state_9c,
}


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def run_diagnostic(motor, *, output_dir: Path,
                   now: Callable[[], datetime] = _utc_now) -> tuple[list[dict], Path]:
    """Execute four bounded, single-attempt reads and save one CSV report."""
    if motor.motor_id != 1 or motor.can_bitrate != 500_000:
        raise ValueError("Phase 1 diagnostic requires standard CAN ID 0x141 at 500000 bit/s")
    commands = [command for command, _ in READ_COMMANDS]
    if len(commands) != len(set(commands)) or set(commands) & FORBIDDEN_COMMANDS:
        raise RuntimeError("read command allowlist contains a duplicate or forbidden command")

    output_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    stamp = now().strftime("%Y%m%dT%H%M%S%fZ")
    csv_path = output_dir / f"absolute_encoder_{stamp}.csv"
    results = []
    with csv_path.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for command, name in READ_COMMANDS:
            timestamp = now().isoformat()
            request = bytes((command,)) + REQUEST_PAYLOAD
            result = {
                "timestamp_utc": timestamp,
                "can_id": "0x141",
                "command": f"0x{command:02X}",
                "name": name,
                "status": "",
                "tx_hex": request.hex(" ").upper(),
                "rx_hex": "",
                "decoded_json": "",
                "compatibility": COMPATIBILITY,
                "uncertainty": "",
                "error": "",
            }
            print(f"{timestamp} REQUEST standard ID=0x141 DLC=8 DATA={result['tx_hex']}")
            try:
                # One exchange only. WaveshareMotor applies the configured bounded timeout.
                reply = motor.exchange(command, REQUEST_PAYLOAD)
                result["rx_hex"] = reply.hex(" ").upper()
                print(f"{timestamp} RESPONSE standard ID=0x141 DLC={len(reply)} "
                      f"DATA={result['rx_hex']}")
                decoded = DECODERS[command](reply)
                result["decoded_json"] = json.dumps(decoded.values, sort_keys=True)
                result["uncertainty"] = decoded.uncertainty
                result["status"] = "ok"
            except ResponseTimeout as exc:
                result["status"] = "unsupported_or_timeout"
                result["error"] = str(exc)
                result["uncertainty"] = (
                    "No reply: command may be unsupported by MHF9025-V3, or communication failed."
                )
            except CommunicationError as exc:
                result["status"] = "malformed_response"
                result["error"] = str(exc)
                result["uncertainty"] = "A reply was received but does not match the V2.35 layout."
            writer.writerow(result)
            handle.flush()
            results.append(result)
            print(json.dumps(result, indent=2))
    return results, csv_path


def wrapped_angle_difference(final_deg: float, initial_deg: float) -> float:
    """Return the signed shortest circular difference in [-180, 180)."""
    return (final_deg - initial_deg + 180.0) % 360.0 - 180.0


def unwrap_single_turn(angles_deg: list[float]) -> list[float]:
    if not angles_deg:
        return []
    unwrapped = [angles_deg[0]]
    for angle in angles_deg[1:]:
        unwrapped.append(unwrapped[-1] + wrapped_angle_difference(angle, angles_deg[len(unwrapped) - 1]))
    return unwrapped


def _check_phase3_motor(motor) -> None:
    if motor.motor_id != 1 or motor.can_bitrate != 500_000:
        raise ValueError("Phase 3 requires standard CAN ID 0x141 at 500000 bit/s")


def _read_record(motor, command: int, sample_index: int, *,
                 start_monotonic: float | None = None,
                 now: Callable[[], datetime] = _utc_now,
                 clock: Callable[[], float] = time.monotonic) -> tuple[dict, DecodedRead]:
    if command not in PHASE3_READ_COMMANDS or command in FORBIDDEN_COMMANDS:
        raise RuntimeError(f"command 0x{command:02X} is not allowed in Phase 3")
    name = dict(READ_COMMANDS)[command]
    request = bytes((command,)) + REQUEST_PAYLOAD
    if start_monotonic is None:
        start_monotonic = clock()
    request_elapsed = clock() - start_monotonic
    request_timestamp = now().isoformat()
    record = {
        "sample_index": sample_index,
        "request_timestamp_utc": request_timestamp,
        "response_timestamp_utc": "",
        "request_elapsed_s": f"{request_elapsed:.9f}",
        "response_elapsed_s": "",
        "can_id": "0x141",
        "command": f"0x{command:02X}",
        "name": name,
        "status": "",
        "tx_hex": request.hex(" ").upper(),
        "rx_hex": "",
        "decoded_json": "",
        "uncertainty": "",
        "error": "",
    }
    print(f"{request_timestamp} TX 0x141: {record['tx_hex']}")
    try:
        reply = motor.exchange(command, REQUEST_PAYLOAD)
    except (OSError, CommunicationError) as exc:
        record["response_elapsed_s"] = f"{clock() - start_monotonic:.9f}"
        record["response_timestamp_utc"] = now().isoformat()
        record["status"] = "error"
        record["uncertainty"] = "No automatic retry was attempted."
        record["error"] = f"{type(exc).__name__}: {exc}"
        exc.phase3_record = record
        raise
    record["response_elapsed_s"] = f"{clock() - start_monotonic:.9f}"
    record["response_timestamp_utc"] = now().isoformat()
    record["rx_hex"] = reply.hex(" ").upper()
    print(f"{record['response_timestamp_utc']} RX 0x141: {record['rx_hex']}")
    decoded = DECODERS[command](reply)
    record["decoded_json"] = json.dumps(decoded.values, sort_keys=True)
    record["uncertainty"] = decoded.uncertainty
    record["status"] = "ok"
    return record, decoded


def _write_phase3_csv(path: Path, records: list[dict]) -> None:
    with path.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=PHASE3_CSV_FIELDS)
        writer.writeheader()
        writer.writerows(records)


def _require_operator_phrase(authorize: Callable[[str], str], prompt: str,
                             phrase: str) -> None:
    if authorize(f"{prompt}\nType exactly: {phrase}\n> ") != phrase:
        raise CommunicationError(f"operator confirmation {phrase} was not provided")


def _write_recovery_report(path: Path, report: dict) -> None:
    initial = report["initial"]
    final = report["final"]
    lines = [
        "# Test B — Absolute Position Recovery",
        "",
        f"- Status: **{report['status']}**",
        f"- Initial 0x94: {initial['single_turn_deg']:.2f}°",
        f"- Final 0x94: {final['single_turn_deg']:.2f}°",
        f"- Wrapped 0x94 change: {report['wrapped_single_turn_change_deg']:+.2f}°",
        f"- Initial 0x92: {initial['multi_turn_deg']:.2f}°",
        f"- Final 0x92: {final['multi_turn_deg']:.2f}°",
        f"- 0x92 change: {report['multi_turn_change_deg']:+.2f}°",
        f"- Approximate physical request: {report['physical_rotation_request']}",
        f"- Direction convention assessment: {report['direction_assessment']}",
        "",
        "The physical rotation is an operator estimate, not independent angular metrology.",
        "The 0x92 value is reported for comparison and is not assumed to survive power loss.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_absolute_recovery(motor_factory: Callable[[], object], *, output_dir: Path,
                          authorize: Callable[[str], str] = input,
                          now: Callable[[], datetime] = _utc_now) -> tuple[dict, Path, Path]:
    """Run operator-driven Test B without transmitting any control command."""
    output_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    stamp = now().strftime("%Y%m%dT%H%M%S%fZ")
    csv_path = output_dir / f"absolute_recovery_{stamp}.csv"
    report_path = output_dir / f"absolute_recovery_{stamp}.md"
    records: list[dict] = []
    phase_start = time.monotonic()

    def read_set(label_index: int) -> dict:
        decoded_by_command = {}
        with motor_factory() as motor:
            _check_phase3_motor(motor)
            for command in (0x90, 0x94, 0x92):
                record, decoded = _read_record(
                    motor, command, label_index, start_monotonic=phase_start, now=now)
                records.append(record)
                decoded_by_command[command] = decoded.values
        return {
            "encoder": decoded_by_command[0x90],
            "single_turn_deg": decoded_by_command[0x94]["single_turn_angle_degrees_provisional"],
            "multi_turn_deg": decoded_by_command[0x92]["multi_turn_angle_degrees_provisional"],
        }

    initial = read_set(0)
    print("Initial readings saved. The CAN serial connection is now closed.")
    _require_operator_phrase(
        authorize,
        "Disconnect motor power, use the physical mark to rotate approximately +90° "
        "without forcing the mechanism, and leave the shaft at that position.",
        RECOVERY_ROTATED_CONFIRMATION,
    )
    _require_operator_phrase(
        authorize,
        "Restore motor power and wait for the controller to finish starting.",
        RECOVERY_POWER_CONFIRMATION,
    )
    final = read_set(1)
    wrapped_delta = wrapped_angle_difference(final["single_turn_deg"],
                                             initial["single_turn_deg"])
    multi_delta = final["multi_turn_deg"] - initial["multi_turn_deg"]
    # Phase 1 physical +90-degree test produced a controller-negative change.
    direction_ok = wrapped_delta < 0
    magnitude_ok = abs(abs(wrapped_delta) - 90.0) <= 20.0
    report = {
        "test": "B_absolute_position_recovery",
        "status": "CONSISTENT_WITH_APPROXIMATE_POSITIVE_90" if direction_ok and magnitude_ok
                  else "REVIEW_REQUIRED",
        "initial": initial,
        "final": final,
        "wrapped_single_turn_change_deg": wrapped_delta,
        "multi_turn_change_deg": multi_delta,
        "physical_rotation_request": "approximately +90 degrees using a physical mark",
        "direction_assessment": (
            "matches Phase 1 convention: physical positive maps to controller-negative"
            if direction_ok else
            "does not match the Phase 1 direction convention"
        ),
        "magnitude_tolerance_deg": 20.0,
        "multi_turn_persistence_assumed": False,
        "stored_home_reference_assessment": (
            "SUPPORTED_BY_APPROXIMATE_PHYSICAL_ROTATION"
            if direction_ok and magnitude_ok else "REVIEW_REQUIRED"
        ),
        "mechanical_accuracy_independently_verified": False,
        "commands_sent": ["0x90", "0x94", "0x92"],
    }
    _write_phase3_csv(csv_path, records)
    _write_recovery_report(report_path, report)
    return report, csv_path, report_path


def analyze_tracking_samples(samples: list[dict]) -> dict:
    if len(samples) < 2:
        return {"status": "INSUFFICIENT_DATA", "sample_count": len(samples)}
    single = [sample["single_turn_deg"] for sample in samples]
    multi = [sample["multi_turn_deg"] for sample in samples]
    unwrapped = unwrap_single_turn(single)
    multi_relative = [value - multi[0] for value in multi]
    single_relative = [value - unwrapped[0] for value in unwrapped]
    multi_delta = multi_relative[-1]
    single_delta = single_relative[-1]
    wraps = sum(abs(single[i] - single[i - 1]) > 180.0 for i in range(1, len(single)))
    max_step = max(abs(single_relative[i] - single_relative[i - 1])
                   for i in range(1, len(single_relative)))
    max_abs_multi = max(abs(value) for value in multi_relative)
    pair_skews = [abs(sample["single_elapsed_s"] - sample["multi_elapsed_s"])
                  for sample in samples]
    maximum_speed = max(abs(sample["speed_dps"]) for sample in samples)
    max_pair_skew = max(pair_skews)
    return {
        "status": "TWO_TURNS_OBSERVED" if max_abs_multi >= 720.0 else "LESS_THAN_TWO_TURNS",
        "sample_count": len(samples),
        "duration_s": samples[-1]["elapsed_s"] - samples[0]["elapsed_s"],
        "multi_turn_displacement_deg": multi_delta,
        "single_turn_unwrapped_displacement_deg": single_delta,
        "agreement_error_deg": single_delta - multi_delta,
        "estimated_revolutions": multi_delta / 360.0,
        "maximum_absolute_multi_turn_excursion_deg": max_abs_multi,
        "exceeded_one_turn": max_abs_multi >= 360.0,
        "reached_two_turns": max_abs_multi >= 720.0,
        "single_turn_wrap_count": wraps,
        "maximum_unwrapped_step_deg": max_step,
        "aliasing_warning": max_step >= 150.0,
        "maximum_0x92_0x94_timestamp_skew_s": max_pair_skew,
        "timing_motion_bound_deg_provisional": max_pair_skew * maximum_speed,
        "multi_relative_deg": multi_relative,
        "single_unwrapped_relative_deg": single_relative,
    }


def _svg_plot(path: Path, samples: list[dict], analysis: dict) -> None:
    width, height, margin = 1000, 560, 70
    xs = [sample["elapsed_s"] for sample in samples]
    series = [analysis["multi_relative_deg"], analysis["single_unwrapped_relative_deg"]]
    x_min, x_max = min(xs), max(xs)
    y_min = min(min(values) for values in series)
    y_max = max(max(values) for values in series)
    if x_max == x_min:
        x_max += 1.0
    if y_max == y_min:
        y_min -= 1.0
        y_max += 1.0
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min, y_max = y_min - y_pad, y_max + y_pad

    def px(value):
        return margin + (value - x_min) / (x_max - x_min) * (width - 2 * margin)

    def py(value):
        return height - margin - (value - y_min) / (y_max - y_min) * (height - 2 * margin)

    colors = ("#1565c0", "#ef6c00")
    labels = ("0x92 relative accumulated angle", "unwrapped 0x94 relative angle")
    polylines = []
    for values, color in zip(series, colors):
        points = " ".join(f"{px(x):.2f},{py(y):.2f}" for x, y in zip(xs, values))
        polylines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{points}"/>')
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="black"/>',
        f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="black"/>',
        *polylines,
        f'<text x="{width/2}" y="{height-15}" text-anchor="middle" font-family="sans-serif">Elapsed time (s)</text>',
        f'<text x="20" y="{height/2}" transform="rotate(-90 20 {height/2})" text-anchor="middle" font-family="sans-serif">Relative angle (deg)</text>',
        f'<text x="{margin}" y="30" font-family="sans-serif" font-size="18">MHF9025-V3 Test D angle tracking</text>',
    ]
    for index, (label, color) in enumerate(zip(labels, colors)):
        y = 50 + index * 22
        svg.extend((
            f'<line x1="{width-390}" y1="{y}" x2="{width-350}" y2="{y}" stroke="{color}" stroke-width="3"/>',
            f'<text x="{width-340}" y="{y+5}" font-family="sans-serif">{label}</text>',
        ))
    svg.append('</svg>')
    path.write_text("\n".join(svg) + "\n", encoding="utf-8")


def _write_tracking_report(path: Path, analysis: dict) -> None:
    if analysis["status"] == "INSUFFICIENT_DATA":
        path.write_text(
            "# Test D — Multi-turn Angle Tracking\n\n"
            f"- Status: **INSUFFICIENT_DATA**\n"
            f"- Fresh complete samples: {analysis['sample_count']}\n\n"
            "The run ended before two complete fresh samples were collected.\n",
            encoding="utf-8",
        )
        return
    lines = [
        "# Test D — Multi-turn Angle Tracking",
        "",
        f"- Status: **{analysis['status']}**",
        f"- Fresh complete samples: {analysis['sample_count']}",
        f"- Duration: {analysis['duration_s']:.3f} s",
        f"- 0x92 displacement: {analysis['multi_turn_displacement_deg']:+.2f}°",
        f"- Unwrapped 0x94 displacement: {analysis['single_turn_unwrapped_displacement_deg']:+.2f}°",
        f"- End-to-end disagreement: {analysis['agreement_error_deg']:+.2f}°",
        f"- Estimated revolutions: {analysis['estimated_revolutions']:+.4f}",
        f"- 0x94 observed wrap transitions: {analysis['single_turn_wrap_count']}",
        f"- Exceeded 360°: {analysis['exceeded_one_turn']}",
        f"- Reached 720°: {analysis['reached_two_turns']}",
        f"- Sampling alias warning: {analysis['aliasing_warning']}",
        f"- Maximum 0x92/0x94 timestamp skew: "
        f"{analysis['maximum_0x92_0x94_timestamp_skew_s']:.6f} s",
        f"- Provisional motion during that skew: "
        f"≤ {analysis['timing_motion_bound_deg_provisional']:.3f}° using peak 0x9C speed",
        "",
        "Both angle streams originate in the same controller and are not independent physical metrology.",
        "A sampling alias warning means a single-turn step approached 180°, so unwrapping may be ambiguous.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_multi_turn_tracking(motor, *, output_dir: Path, duration_s: float,
                            sample_interval_s: float,
                            authorize: Callable[[str], str] = input,
                            now: Callable[[], datetime] = _utc_now,
                            clock: Callable[[], float] = time.monotonic,
                            sleeper: Callable[[float], None] = time.sleep,
                            sample_limit: int | None = None) -> tuple[dict, Path, Path, Path]:
    """Run read-only Test D; operator supplies all physical shaft movement."""
    _check_phase3_motor(motor)
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("tracking duration must be positive and finite")
    if not math.isfinite(sample_interval_s) or sample_interval_s <= 0:
        raise ValueError("sample interval must be positive and finite")
    _require_operator_phrase(
        authorize,
        "Confirm the motor is safely disabled but powered, the mechanism permits manual "
        "rotation, and you will stop rather than force the shaft.",
        TRACKING_CONFIRMATION,
    )
    output_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    stamp = now().strftime("%Y%m%dT%H%M%S%fZ")
    csv_path = output_dir / f"multi_turn_tracking_{stamp}.csv"
    report_path = output_dir / f"multi_turn_tracking_{stamp}.md"
    plot_path = output_dir / f"multi_turn_tracking_{stamp}.svg"
    records: list[dict] = []
    samples: list[dict] = []
    start = clock()
    index = 0
    with csv_path.open("x", newline="", encoding="utf-8") as csv_handle:
        csv_writer = csv.DictWriter(csv_handle, fieldnames=PHASE3_CSV_FIELDS)
        csv_writer.writeheader()

        def retain(record: dict) -> None:
            records.append(record)
            csv_writer.writerow(record)
            csv_handle.flush()

        try:
            while True:
                elapsed = clock() - start
                if samples and elapsed > duration_s:
                    break
                decoded_by_command = {}
                record_by_command = {}
                try:
                    for command in (0x92, 0x94, 0x9C):
                        record, decoded = _read_record(
                            motor, command, index, start_monotonic=start,
                            now=now, clock=clock)
                        retain(record)
                        record_by_command[command] = record
                        decoded_by_command[command] = decoded.values
                except (OSError, CommunicationError) as exc:
                    retain(getattr(exc, "phase3_record", {
                        "sample_index": index,
                        "request_timestamp_utc": now().isoformat(),
                        "response_timestamp_utc": "",
                        "request_elapsed_s": f"{elapsed:.9f}",
                        "response_elapsed_s": "", "can_id": "0x141",
                        "command": "incomplete_sample", "name": "tracking_abort",
                        "status": "error", "tx_hex": "", "rx_hex": "",
                        "decoded_json": "",
                        "uncertainty": "No automatic retry was attempted.",
                        "error": f"{type(exc).__name__}: {exc}",
                    }))
                    break
                samples.append({
                    "sample_index": index,
                    "elapsed_s": float(record_by_command[0x94]["response_elapsed_s"]),
                    "multi_elapsed_s": float(record_by_command[0x92]["response_elapsed_s"]),
                    "single_elapsed_s": float(record_by_command[0x94]["response_elapsed_s"]),
                    "single_turn_deg": decoded_by_command[0x94]["single_turn_angle_degrees_provisional"],
                    "multi_turn_deg": decoded_by_command[0x92]["multi_turn_angle_degrees_provisional"],
                    "speed_dps": decoded_by_command[0x9C]["speed_raw_dps_s16"],
                    "encoder_raw": decoded_by_command[0x9C]["encoder_raw_u16"],
                })
                index += 1
                if sample_limit is not None and index >= sample_limit:
                    break
                if elapsed >= duration_s:
                    break
                sleeper(min(sample_interval_s, max(0.0, duration_s - elapsed)))
        except KeyboardInterrupt:
            retain({
                "sample_index": index, "request_timestamp_utc": now().isoformat(),
                "response_timestamp_utc": "", "request_elapsed_s":
                f"{clock() - start:.9f}", "response_elapsed_s": "", "can_id": "0x141",
                "command": "operator_interrupt", "name": "tracking_abort",
                "status": "interrupted", "tx_hex": "", "rx_hex": "",
                "decoded_json": "", "uncertainty": "Partial read-only log retained.",
                "error": "KeyboardInterrupt",
            })

    analysis = analyze_tracking_samples(samples)
    analysis["commands_sent"] = ["0x92", "0x94", "0x9C"]
    analysis["physical_rotation_independently_measured"] = False
    _write_tracking_report(report_path, analysis)
    if analysis["status"] != "INSUFFICIENT_DATA":
        _svg_plot(plot_path, samples, analysis)
    else:
        plot_path.write_text("Insufficient complete samples for a plot.\n", encoding="utf-8")
    return analysis, csv_path, report_path, plot_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--can-bitrate", type=int, default=500_000)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--serial-baud", type=int, default=2_000_000)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--mode", choices=("diagnostic", "absolute-recovery",
                                            "multi-turn-tracking"),
                        default="diagnostic")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Test D logging duration in seconds")
    parser.add_argument("--sample-interval", type=float, default=0.1,
                        help="Test D delay between complete 0x92/0x94/0x9C samples")
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "absolute_encoder_logs")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.device != "/dev/ttyUSB0":
        raise SystemExit("Phase 1 hardware configuration requires --device /dev/ttyUSB0")
    if args.can_bitrate != 500_000 or args.motor_id != 1:
        raise SystemExit("Phase 1 requires --can-bitrate 500000 and --motor-id 1 (CAN ID 0x141)")
    if args.serial_baud != 2_000_000:
        raise SystemExit("Waveshare serial transport requires --serial-baud 2000000")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise SystemExit("--timeout must be positive and finite")
    if not math.isfinite(args.duration) or args.duration <= 0:
        raise SystemExit("--duration must be positive and finite")
    if not math.isfinite(args.sample_interval) or args.sample_interval <= 0:
        raise SystemExit("--sample-interval must be positive and finite")

    try:
        factory = lambda: WaveshareMotor(args.device, args.motor_id, args.can_bitrate,
                                         args.serial_baud, args.timeout)
        if args.mode == "absolute-recovery":
            report, csv_path, report_path = run_absolute_recovery(
                factory, output_dir=args.output_dir)
            print(json.dumps(report, indent=2))
            print(f"CSV: {csv_path}")
            print(f"REPORT: {report_path}")
            return 0 if report["status"] != "REVIEW_REQUIRED" else 1
        with factory() as motor:
            if args.mode == "multi-turn-tracking":
                analysis, csv_path, report_path, plot_path = run_multi_turn_tracking(
                    motor, output_dir=args.output_dir, duration_s=args.duration,
                    sample_interval_s=args.sample_interval)
                print(json.dumps({key: value for key, value in analysis.items()
                                  if not isinstance(value, list)}, indent=2))
                print(f"CSV: {csv_path}")
                print(f"REPORT: {report_path}")
                print(f"PLOT: {plot_path}")
                return 0 if analysis["status"] == "TWO_TURNS_OBSERVED" else 1
            results, csv_path = run_diagnostic(motor, output_dir=args.output_dir)
    except (OSError, CommunicationError) as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(f"CSV: {csv_path}")
    return 0 if all(item["status"] == "ok" for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
