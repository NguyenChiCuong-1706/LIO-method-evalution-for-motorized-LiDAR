"""Guarded persistent HOME calibration for MHF9025-V3 (Phase 2).

The default ``inspect`` and ``prepare`` modes are read-only.  Command 0x19 is
reachable only through ``write`` with a previously prepared session, four
interactive confirmations, and a second fresh stationary check.  A write is
attempted once per session and is never retried automatically.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from enum import Enum
import fcntl
import json
import math
from pathlib import Path
import sys
import time
from typing import Callable
from uuid import uuid4

from absolute_encoder_diagnostic import (
    REQUEST_PAYLOAD, decode_encoder_90, decode_multi_turn_92,
    decode_single_turn_94, decode_state_9c,
)
from motor_can import CommunicationError, ResponseTimeout
from waveshare_motor import WaveshareMotor, _encoder_delta


CAN_ID = 0x141
HOME_COMMAND = 0x19
HOME_FRAME = bytes((HOME_COMMAND,)) + REQUEST_PAYLOAD
STATIONARY_SAMPLES = 5
STATIONARY_INTERVAL_S = 0.1
STATIONARY_SPEED_LIMIT_DPS = 4
STATIONARY_ENCODER_LIMIT_COUNTS = 2
MAX_SPEED_FEEDBACK_AGE_S = 0.25
FINAL_CONFIRMATION = "WRITE_ABSOLUTE_HOME"
ALIGNMENT_CONFIRMATION = "MECHANICAL_HOME_ALIGNED"
SAFETY_CONFIRMATION = "ROTATING_ASSEMBLY_SAFE"
NO_MOTION_CONFIRMATION = "NO_ACTIVE_MOTION_COMMAND"
POWER_CYCLE_CONFIRMATION = "POWER_CYCLE_COMPLETED_WITHOUT_SHAFT_MOVEMENT"
MECHANICAL_HOME_CONFIRMATION = "MECHANICAL_HOME_POSITION_CONFIRMED"
FORBIDDEN_ZERO_COMMANDS = frozenset((0x91, 0x95))
ANGLE_STABILITY_SAMPLES = 5
ANGLE_STABILITY_INTERVAL_S = 0.05
ANGLE_STABILITY_LIMIT_DEG = 0.20


class CalibrationState(str, Enum):
    NOT_CALIBRATED = "NOT_CALIBRATED"
    READY_TO_WRITE = "READY_TO_WRITE"
    WRITE_SENT_UNVERIFIED = "WRITE_SENT_UNVERIFIED"
    WRITE_ACKNOWLEDGED_UNVERIFIED = "WRITE_ACKNOWLEDGED_UNVERIFIED"
    WRITE_RESULT_UNKNOWN = "WRITE_RESULT_UNKNOWN"
    VERIFIED_AFTER_POWER_CYCLE = "VERIFIED_AFTER_POWER_CYCLE"
    FAILED = "FAILED"


class AuthorizationError(RuntimeError):
    pass


class DuplicateWriteError(RuntimeError):
    pass


@dataclass(frozen=True)
class StationaryResult:
    stationary: bool
    samples: list[dict]
    last_response_monotonic: float
    detail: str


@dataclass(frozen=True)
class AngleStabilityResult:
    stable: bool
    samples: list[dict]
    maximum_circular_delta_degrees: float
    detail: str


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _frame(command: int, reply: bytes, decoded: dict, timestamp: str) -> dict:
    return {
        "timestamp_utc": timestamp,
        "command": f"0x{command:02X}",
        "tx_hex": (bytes((command,)) + REQUEST_PAYLOAD).hex(" ").upper(),
        "rx_hex": reply.hex(" ").upper(),
        "decoded": decoded,
    }


def _read(motor, command: int, decoder, now: Callable[[], datetime]) -> dict:
    if command in FORBIDDEN_ZERO_COMMANDS or command == HOME_COMMAND:
        raise RuntimeError("write/zero command used in read helper")
    reply = motor.exchange(command, REQUEST_PAYLOAD)
    decoded = decoder(reply).values
    return _frame(command, reply, decoded, now().isoformat())


def stationary_check(motor, *, clock: Callable[[], float] = time.monotonic,
                     now: Callable[[], datetime] = utc_now,
                     sleeper: Callable[[float], None] = time.sleep) -> StationaryResult:
    """Use five fresh 0x9C replies; never infer stationarity from one sample."""
    samples = []
    for index in range(STATIONARY_SAMPLES):
        reply = motor.exchange(0x9C, REQUEST_PAYLOAD)
        received = clock()
        decoded = decode_state_9c(reply).values
        samples.append(_frame(0x9C, reply, decoded, now().isoformat()) |
                       {"received_monotonic": received})
        if index + 1 < STATIONARY_SAMPLES:
            sleeper(STATIONARY_INTERVAL_S)
    first_encoder = samples[0]["decoded"]["encoder_raw_u16"]
    speeds_ok = all(abs(sample["decoded"]["speed_raw_dps_s16"]) <=
                    STATIONARY_SPEED_LIMIT_DPS for sample in samples)
    encoders_ok = all(abs(_encoder_delta(first_encoder,
                                         sample["decoded"]["encoder_raw_u16"])) <=
                      STATIONARY_ENCODER_LIMIT_COUNTS for sample in samples)
    stationary = speeds_ok and encoders_ok
    detail = (
        f"{STATIONARY_SAMPLES} fresh samples: speed within "
        f"±{STATIONARY_SPEED_LIMIT_DPS} deg/s={speeds_ok}, encoder within "
        f"±{STATIONARY_ENCODER_LIMIT_COUNTS} counts={encoders_ok}; "
        "this does not prove that another CAN controller has no queued command"
    )
    return StationaryResult(stationary, samples,
                            samples[-1]["received_monotonic"], detail)


def angle_stability_check(motor, *, now: Callable[[], datetime] = utc_now,
                          sleeper: Callable[[float], None] = time.sleep) -> AngleStabilityResult:
    """Require five fresh, circularly stable V3 0x94 measurements."""
    samples = []
    for index in range(ANGLE_STABILITY_SAMPLES):
        samples.append(_read(motor, 0x94, decode_single_turn_94, now))
        if index + 1 < ANGLE_STABILITY_SAMPLES:
            sleeper(ANGLE_STABILITY_INTERVAL_S)
    first = samples[0]["decoded"]["single_turn_angle_degrees_provisional"]
    deltas = []
    for sample in samples:
        value = sample["decoded"]["single_turn_angle_degrees_provisional"]
        deltas.append(abs((value - first + 180.0) % 360.0 - 180.0))
    maximum = max(deltas)
    stable = maximum <= ANGLE_STABILITY_LIMIT_DEG
    detail = (
        f"{ANGLE_STABILITY_SAMPLES} fresh 0x94 samples; maximum circular change "
        f"{maximum:.3f} deg; limit {ANGLE_STABILITY_LIMIT_DEG:.3f} deg"
    )
    return AngleStabilityResult(stable, samples, maximum, detail)


def collect_preflight(motor, *, clock: Callable[[], float] = time.monotonic,
                      now: Callable[[], datetime] = utc_now,
                      sleeper: Callable[[float], None] = time.sleep) -> dict:
    """Read angles/encoder then establish stationarity with fresh telemetry."""
    encoder = _read(motor, 0x90, decode_encoder_90, now)
    single = _read(motor, 0x94, decode_single_turn_94, now)
    multi = _read(motor, 0x92, decode_multi_turn_92, now)
    stationary = stationary_check(motor, clock=clock, now=now, sleeper=sleeper)
    age = clock() - stationary.last_response_monotonic
    result = {
        "encoder_0x90": encoder,
        "single_turn_0x94": single,
        "multi_turn_0x92": multi,
        "stationary_check": asdict(stationary),
        "speed_feedback_age_s": age,
        "communication_status": "OK",
        "motor_state": "STATIONARY_OBSERVED" if stationary.stationary else "MOVING_OR_UNCERTAIN",
        "motion_command_status": (
            "tool sent no motion command; no motion observed; external command state is not "
            "independently readable from verified MHF9025-V3 fields"
        ),
    }
    if not stationary.stationary:
        exc = CommunicationError("fresh telemetry does not confirm a stationary motor")
        exc.preflight = result
        raise exc
    if age < 0 or age > MAX_SPEED_FEEDBACK_AGE_S:
        exc = CommunicationError(
            f"speed feedback is stale ({age:.3f}s > {MAX_SPEED_FEEDBACK_AGE_S:.3f}s)")
        exc.preflight = result
        raise exc
    return result


def new_session(*, now: Callable[[], datetime] = utc_now) -> dict:
    timestamp = now().isoformat()
    return {
        "schema_version": 1,
        "session_id": str(uuid4()),
        "created_utc": timestamp,
        "updated_utc": timestamp,
        "state": CalibrationState.NOT_CALIBRATED.value,
        "verification_status": "NOT_PERFORMED",
        "hardware": {
            "motor": "MHF9025-V3", "transport": "Waveshare USB-CAN serial",
            "device": "/dev/ttyUSB0", "can_bitrate": 500000, "can_id": "0x141",
        },
        "proposed_write": {
            "command": "0x19", "frame_hex": HOME_FRAME.hex(" ").upper(),
            "purpose": "write current encoder position as zero into ROM; V3 compatibility provisional",
        },
        "preflight": None,
        "write": {
            "attempted": False, "timestamp_utc": None, "tx_hex": None,
            "rx_hex": None, "acknowledgement_status": "NOT_SENT", "error": None,
        },
        "verification": None,
        "history": [{"timestamp_utc": timestamp,
                     "state": CalibrationState.NOT_CALIBRATED.value}],
    }


def _transition(session: dict, state: CalibrationState, *,
                now: Callable[[], datetime] = utc_now, detail: str = "") -> None:
    timestamp = now().isoformat()
    session["state"] = state.value
    session["updated_utc"] = timestamp
    event = {"timestamp_utc": timestamp, "state": state.value}
    if detail:
        event["detail"] = detail
    session["history"].append(event)


def _write_json_handle(handle, session: dict) -> None:
    handle.seek(0)
    json.dump(session, handle, indent=2, sort_keys=True)
    handle.write("\n")
    handle.truncate()
    handle.flush()


def create_session_file(output_dir: Path, *, now: Callable[[], datetime] = utc_now) -> tuple[dict, Path]:
    output_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    session = new_session(now=now)
    stamp = now().strftime("%Y%m%dT%H%M%S%fZ")
    path = output_dir / f"home_calibration_{stamp}_{session['session_id'][:8]}.json"
    with path.open("x", encoding="utf-8") as handle:
        json.dump(session, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return session, path


def prepare_session(motor, session: dict, *, clock=time.monotonic, now=utc_now,
                    sleeper=time.sleep) -> dict:
    if session["state"] != CalibrationState.NOT_CALIBRATED.value:
        raise DuplicateWriteError("calibration session is not new")
    try:
        session["preflight"] = collect_preflight(
            motor, clock=clock, now=now, sleeper=sleeper)
        stability = angle_stability_check(motor, now=now, sleeper=sleeper)
        session["angle_stability_check"] = asdict(stability)
        if not stability.stable:
            raise CommunicationError("fresh absolute-angle readings are not stable")
    except BaseException as exc:
        if hasattr(exc, "preflight"):
            session["preflight"] = exc.preflight
        _transition(session, CalibrationState.FAILED, now=now, detail=str(exc))
        raise
    _transition(session, CalibrationState.READY_TO_WRITE, now=now,
                detail="read-only preflight passed; no ROM command sent")
    return session


def validate_home_ack(reply: bytes) -> dict:
    """Decode documented V2.35 and observed MHF9025-V3 0x19 replies.

    The V3 layout is based on one physical capture and therefore remains
    provisional.  Either valid layout only proves that a reply was received;
    persistence still requires the separate power-cycle verification.
    """
    if len(reply) != 8 or reply[0] != HOME_COMMAND:
        raise CommunicationError("malformed 0x19 reply; expected command 0x19 and DLC 8")
    if not any(reply[1:6]):
        return {
            "layout": "LK-TECH CAN V2.35",
            "encoder_offset_raw_u16": int.from_bytes(reply[6:8], "little"),
            "compatibility": "documented family layout; MHF9025-V3 support provisional",
        }
    if not any(reply[1:4]) and not any(reply[6:8]):
        return {
            "layout": "MHF9025-V3 observed bytes 4-5",
            "encoder_value_raw_u16": int.from_bytes(reply[4:6], "little"),
            "compatibility": "observed once on MHF9025-V3; field meaning unverified",
        }
    raise CommunicationError(
        "malformed 0x19 reply; matches neither documented V2.35 nor observed MHF9025-V3 layout")


def _require_phrase(authorize: Callable[[str], str], prompt: str, phrase: str) -> None:
    if authorize(f"{prompt}\nType exactly: {phrase}\n> ") != phrase:
        raise AuthorizationError(f"required confirmation {phrase} was not provided")


def write_home_once(motor, session: dict, persist: Callable[[dict], None], *,
                    authorize: Callable[[str], str] = input,
                    clock=time.monotonic, now=utc_now, sleeper=time.sleep) -> dict:
    """Perform the only 0x19 call site after all interlocks pass."""
    if session["state"] != CalibrationState.READY_TO_WRITE.value:
        raise DuplicateWriteError(
            f"session state {session['state']} cannot issue a HOME write")
    if session["write"]["attempted"]:
        raise DuplicateWriteError("this calibration session already attempted 0x19")

    # Re-read before asking for authorization; a prepared session may be old.
    try:
        session["pre_write_preflight"] = collect_preflight(
            motor, clock=clock, now=now, sleeper=sleeper)
        print("WARNING: this operation modifies the motor encoder zero in ROM.")
        print("Proposed standard CAN frame: ID=0x141 DLC=8 DATA=" +
              HOME_FRAME.hex(" ").upper())
        _require_phrase(authorize, "Confirm LiDAR is physically aligned to mechanical HOME.",
                        ALIGNMENT_CONFIRMATION)
        _require_phrase(authorize, "Confirm the rotating assembly is secured and safe.",
                        SAFETY_CONFIRMATION)
        _require_phrase(authorize,
                        "Confirm no controller is executing or able to issue a movement command.",
                        NO_MOTION_CONFIRMATION)
        _require_phrase(authorize,
                        "Final persistent ROM-write authorization. Power-cycle verification is required.",
                        FINAL_CONFIRMATION)
        # Confirm again after the human interaction so telemetry cannot become stale.
        final_stationary = stationary_check(
            motor, clock=clock, now=now, sleeper=sleeper)
        session["final_stationary_check"] = asdict(final_stationary)
        age = clock() - final_stationary.last_response_monotonic
        session["final_speed_feedback_age_s"] = age
        if not final_stationary.stationary:
            raise CommunicationError("motor moved or became uncertain after authorization")
        if age < 0 or age > MAX_SPEED_FEEDBACK_AGE_S:
            raise CommunicationError(
                f"final speed feedback is stale ({age:.3f}s > {MAX_SPEED_FEEDBACK_AGE_S:.3f}s)")
    except BaseException as exc:
        if hasattr(exc, "preflight"):
            session["pre_write_preflight"] = exc.preflight
        _transition(session, CalibrationState.FAILED, now=now, detail=str(exc))
        persist(session)
        raise

    session["write"].update({
        "attempted": True, "timestamp_utc": now().isoformat(),
        "tx_hex": HOME_FRAME.hex(" ").upper(), "rx_hex": None,
        "acknowledgement_status": "SENT_AWAITING_REPLY", "error": None,
    })
    _transition(session, CalibrationState.WRITE_SENT_UNVERIFIED, now=now,
                detail="0x19 attempt committed; automatic retry prohibited")
    persist(session)  # Commit the one-shot guard before transmitting.
    try:
        reply = motor.exchange(HOME_COMMAND, REQUEST_PAYLOAD)
        session["write"]["rx_hex"] = reply.hex(" ").upper()
        ack_decoded = validate_home_ack(reply)
    except (OSError, CommunicationError) as exc:
        session["write"]["acknowledgement_status"] = "UNKNOWN"
        session["write"]["error"] = f"{type(exc).__name__}: {exc}"
        session["verification_status"] = "POWER_CYCLE_REQUIRED_RESULT_UNKNOWN"
        _transition(session, CalibrationState.WRITE_RESULT_UNKNOWN, now=now, detail=str(exc))
        persist(session)
        return session

    session["write"]["acknowledgement_status"] = "ACKNOWLEDGED_UNVERIFIED"
    session["write"]["ack_decoded"] = ack_decoded
    if "encoder_offset_raw_u16" in ack_decoded:
        session["write"]["encoder_offset_raw_u16"] = ack_decoded["encoder_offset_raw_u16"]
    session["verification_status"] = "POWER_CYCLE_REQUIRED"
    _transition(session, CalibrationState.WRITE_ACKNOWLEDGED_UNVERIFIED, now=now,
                detail="valid 0x19 reply received; persistence is not yet proven")
    persist(session)
    return session


def verify_after_power_cycle(motor, session: dict, persist: Callable[[dict], None], *,
                             authorize: Callable[[str], str] = input,
                             mechanical_authorize: Callable[[str], str] | None = None,
                             tolerance_deg: float = 1.0, clock=time.monotonic,
                             now=utc_now, sleeper=time.sleep) -> dict:
    allowed = {CalibrationState.WRITE_ACKNOWLEDGED_UNVERIFIED.value,
               CalibrationState.WRITE_RESULT_UNKNOWN.value}
    if session["state"] not in allowed or not session["write"]["attempted"]:
        raise DuplicateWriteError("session has no unverified HOME-write attempt")
    _require_phrase(authorize,
                    "Confirm motor power was cycled without intentional shaft movement.",
                    POWER_CYCLE_CONFIRMATION)
    mechanical_confirmed = False
    if mechanical_authorize is not None:
        _require_phrase(
            mechanical_authorize,
            "Confirm the shaft mark is still physically aligned with the selected mechanical HOME.",
            MECHANICAL_HOME_CONFIRMATION,
        )
        mechanical_confirmed = True
    verification = collect_preflight(motor, clock=clock, now=now, sleeper=sleeper)
    single = verification["single_turn_0x94"]["decoded"][
        "single_turn_angle_degrees_provisional"]
    multi = verification["multi_turn_0x92"]["decoded"][
        "multi_turn_angle_degrees_provisional"]
    circular_zero_error = min(single, 360.0 - single)
    # 0x92 is recorded for context but is not a persistence gate: its
    # accumulated count is not assumed to survive a controller power cycle.
    passed = circular_zero_error <= tolerance_deg
    session["verification"] = {
        "operator_power_cycle_confirmation": True,
        "operator_mechanical_home_confirmation": mechanical_confirmed,
        "tolerance_degrees": tolerance_deg,
        "single_turn_zero_error_degrees": circular_zero_error,
        "multi_turn_zero_error_degrees": abs(multi),
        "multi_turn_used_for_pass": False,
        "measurements": verification,
        "passed": passed,
        "physical_home_alignment_independently_measured": False,
        "mechanical_home_position_operator_attested": mechanical_confirmed,
    }
    if passed:
        session["verification_status"] = "VERIFIED_BY_CONTROLLER_AFTER_OPERATOR_ATTESTED_POWER_CYCLE"
        _transition(session, CalibrationState.VERIFIED_AFTER_POWER_CYCLE, now=now,
                    detail="controller angles are within configured zero tolerance")
    else:
        session["verification_status"] = "FAILED_ZERO_TOLERANCE"
        _transition(session, CalibrationState.FAILED, now=now,
                    detail="post-power-cycle controller angles are outside tolerance")
    persist(session)
    return session


def recheck_verified_home(motor, session: dict, persist: Callable[[dict], None], *,
                          mechanical_authorize: Callable[[str], str],
                          tolerance_deg: float = 1.0, clock=time.monotonic,
                          now=utc_now, sleeper=time.sleep) -> dict:
    """Check the current physical HOME mark without repeating a ROM write.

    The historical power-cycle verification remains in ``verification``. A
    fresh check is appended so a later failed check cannot erase that record.
    """
    if (session.get("state") != CalibrationState.VERIFIED_AFTER_POWER_CYCLE.value or
            not session.get("write", {}).get("attempted") or
            not (session.get("verification") or {}).get("passed")):
        raise DuplicateWriteError("session has no verified HOME write to recheck")
    if not math.isfinite(tolerance_deg) or not 0 < tolerance_deg <= 5:
        raise ValueError("HOME verification tolerance must be above 0 and at most 5 degrees")
    _require_phrase(
        mechanical_authorize,
        "Confirm the shaft mark is physically aligned with the selected mechanical HOME.",
        MECHANICAL_HOME_CONFIRMATION,
    )
    measurements = collect_preflight(motor, clock=clock, now=now, sleeper=sleeper)
    single = measurements["single_turn_0x94"]["decoded"][
        "single_turn_angle_degrees_provisional"]
    circular_zero_error = min(single % 360.0, 360.0 - (single % 360.0))
    check = {
        "timestamp_utc": now().isoformat(),
        "tolerance_degrees": tolerance_deg,
        "single_turn_zero_error_degrees": circular_zero_error,
        "mechanical_home_position_operator_attested": True,
        "physical_home_alignment_independently_measured": False,
        "measurements": measurements,
        "passed": circular_zero_error <= tolerance_deg,
    }
    session.setdefault("current_home_checks", []).append(check)
    session["updated_utc"] = check["timestamp_utc"]
    persist(session)
    return check


def _open_locked_session(path: Path):
    handle = path.open("r+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException:
        handle.close()
        raise
    return handle


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("inspect", "prepare", "write", "verify"),
                        default="inspect")
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--can-bitrate", type=int, default=500_000)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--serial-baud", type=int, default=2_000_000)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--log-dir", type=Path,
                        default=Path(__file__).resolve().parent / "absolute_home_logs")
    parser.add_argument("--session", type=Path,
                        help="prepared session JSON; required for write and verify")
    parser.add_argument("--verification-tolerance-deg", type=float, default=1.0)
    return parser


def _validate_cli(args) -> None:
    if args.device != "/dev/ttyUSB0" or args.can_bitrate != 500_000 or args.motor_id != 1:
        raise SystemExit("Phase 2 requires /dev/ttyUSB0, 500000 bit/s, motor ID 1 (0x141)")
    if args.serial_baud != 2_000_000 or not math.isfinite(args.timeout) or args.timeout <= 0:
        raise SystemExit("serial baud must be 2000000 and timeout positive and finite")
    if args.mode in ("write", "verify") and args.session is None:
        raise SystemExit(f"--session is required for --mode {args.mode}")
    if args.mode in ("inspect", "prepare") and args.session is not None:
        raise SystemExit("--session is accepted only by write and verify modes")
    if not math.isfinite(args.verification_tolerance_deg) or not (
            0 < args.verification_tolerance_deg <= 5):
        raise SystemExit("verification tolerance must be above 0 and at most 5 degrees")
    if args.mode in ("write", "verify") and not sys.stdin.isatty():
        raise SystemExit("write and verify modes require an interactive terminal")


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    _validate_cli(args)
    session_handle = None
    try:
        with WaveshareMotor(args.device, args.motor_id, args.can_bitrate,
                            args.serial_baud, args.timeout) as motor:
            if args.mode in ("inspect", "prepare"):
                session, path = create_session_file(args.log_dir)
                if args.mode == "inspect":
                    try:
                        session["preflight"] = collect_preflight(motor)
                    except BaseException as exc:
                        if hasattr(exc, "preflight"):
                            session["preflight"] = exc.preflight
                        _transition(session, CalibrationState.FAILED, detail=str(exc))
                        with path.open("w", encoding="utf-8") as handle:
                            json.dump(session, handle, indent=2, sort_keys=True); handle.write("\n")
                        raise
                else:
                    try:
                        prepare_session(motor, session)
                    finally:
                        with path.open("w", encoding="utf-8") as handle:
                            json.dump(session, handle, indent=2, sort_keys=True); handle.write("\n")
                if args.mode == "inspect":
                    with path.open("w", encoding="utf-8") as handle:
                        json.dump(session, handle, indent=2, sort_keys=True); handle.write("\n")
                print(json.dumps(session, indent=2, sort_keys=True))
                print(f"SESSION_FILE={path}")
                if args.mode == "prepare":
                    print("PREPARED ONLY: 0x19 was not sent.")
                    print("Proposed frame: ID=0x141 DLC=8 DATA=" + HOME_FRAME.hex(" ").upper())
                return 0

            session_handle = _open_locked_session(args.session)
            session = json.load(session_handle)
            persist = lambda value: _write_json_handle(session_handle, value)
            if args.mode == "write":
                write_home_once(motor, session, persist)
            else:
                verify_after_power_cycle(
                    motor, session, persist,
                    tolerance_deg=args.verification_tolerance_deg)
            print(json.dumps(session, indent=2, sort_keys=True))
            print(f"SESSION_FILE={args.session}")
            return 0 if session["state"] != CalibrationState.FAILED.value else 1
    except (OSError, ValueError, CommunicationError, AuthorizationError,
            DuplicateWriteError, BlockingIOError) as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    finally:
        if session_handle is not None:
            session_handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
