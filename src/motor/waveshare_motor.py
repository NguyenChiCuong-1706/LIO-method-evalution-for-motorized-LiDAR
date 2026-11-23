"""Bounded MF9025-family speed test via Waveshare USB-CAN-A serial bridge.

Normal motion is refused while MHF9025-V3 protection state remains UNKNOWN.
Experimental motion requires separate authorization and interactive confirmation.
Physical positive/negative direction must be mapped from the chosen viewing side.
"""

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
from enum import Enum
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time

from motor_can import (CommunicationError, ControllerFault, ProtectionState, ResponseTimeout,
                       assess_mhf9025_v3_protection, decode_feedback,
                       decode_mhf9025_v3_status1, speed_payload)
from speed_analysis import (EXPERIMENT_HARD_SPEED_RPM, STARTUP_LABEL_WINDOW_S,
                            TRACKING_BAND_DPS, TimedRawSample, classify_speed,
                            compare_speed_with_angle)
from waveshare_probe import SERIAL_SPEED, CAN_SPEED_CODE, extract_frames, settings_frame


class MotionState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    STATUS_VERIFIED = "STATUS_VERIFIED"
    READY = "READY"
    ENABLE = "ENABLE"
    SPEED_COMMAND = "SPEED_COMMAND"
    MONITOR_FEEDBACK = "MONITOR_FEEDBACK"
    STOP = "STOP"
    VERIFY_STOP = "VERIFY_STOP"
    DISABLE = "DISABLE"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"


class MotionStateMachine:
    ORDER = (MotionState.DISCONNECTED, MotionState.CONNECTED,
             MotionState.STATUS_VERIFIED, MotionState.READY, MotionState.ENABLE,
             MotionState.SPEED_COMMAND, MotionState.MONITOR_FEEDBACK,
             MotionState.STOP, MotionState.VERIFY_STOP, MotionState.DISABLE,
             MotionState.COMPLETE)

    def __init__(self):
        self.state = MotionState.DISCONNECTED

    def transition(self, target: MotionState):
        current = self.state
        next_normal = (self.ORDER[self.ORDER.index(current) + 1]
                       if current in self.ORDER[:-1] else None)
        if target != next_normal and not (
                target == MotionState.ERROR and current not in (MotionState.COMPLETE, MotionState.ERROR)) and not (
                target == MotionState.DISABLE and current == MotionState.ERROR):
            raise RuntimeError(f"invalid motion-state transition {current.value} -> {target.value}")
        self.state = target
        print(f"state: {target.value}", flush=True)


def require_protection_clear(assessment):
    if assessment.state == ProtectionState.UNKNOWN:
        raise CommunicationError("protection state UNKNOWN: " + assessment.evidence +
                                 "; motion blocked before enable")
    if assessment.state == ProtectionState.VERIFIED_FAULT:
        raise ControllerFault("verified motor protection fault: " + assessment.evidence)
    if assessment.state != ProtectionState.VERIFIED_NORMAL:
        raise CommunicationError("unrecognized protection classification; motion blocked")


def serial_can_frame(motor_id: int, command: int, payload: bytes = b"\x00" * 7) -> bytes:
    if not 1 <= motor_id <= 32 or not 0 <= command <= 255 or len(payload) != 7:
        raise ValueError("invalid motor ID, command, or payload")
    can_id = 0x140 + motor_id
    return bytes((0xAA, 0xC8, can_id & 0xFF, can_id >> 8, command)) + payload + b"\x55"


def write_with_timeout(fd: int, payload: bytes, timeout: float) -> None:
    """Write a complete bridge packet without an unbounded serial drain."""
    deadline = time.monotonic() + timeout
    remaining_data = memoryview(payload)
    while remaining_data:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise CommunicationError("USB serial write timed out")
        try:
            count = os.write(fd, remaining_data)
        except BlockingIOError:
            continue
        if count <= 0:
            raise CommunicationError("USB serial write made no progress")
        remaining_data = remaining_data[count:]


class WaveshareMotor:
    def __init__(self, device: str, motor_id: int, can_bitrate: int = 500_000,
                 serial_baud: int = 2_000_000, timeout: float = 0.5):
        self.device = device
        self.motor_id = motor_id
        self.can_bitrate = can_bitrate
        self.serial_baud = serial_baud
        self.timeout = timeout
        self.fd = None
        self.original = None
        self.buffer = bytearray()

    def __enter__(self):
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise CommunicationError(
                    f"{self.device} is already owned by another Waveshare process; "
                    "close the GUI or joint-state node first") from exc
            self.original = termios.tcgetattr(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[0:4] = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0]
            attrs[4:6] = [SERIAL_SPEED[self.serial_baud]] * 2
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIFLUSH)
            write_with_timeout(self.fd, settings_frame(self.can_bitrate), self.timeout)
            time.sleep(0.1)
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        if self.fd is not None:
            try:
                if self.original is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            finally:
                os.close(self.fd)
                self.fd = None

    def exchange(self, command: int, payload: bytes = b"\x00" * 7,
                 *, timeout: float | None = None) -> bytes:
        if self.fd is None:
            raise CommunicationError("serial CAN bridge is not open")
        wait = self.timeout if timeout is None else min(self.timeout, timeout)
        if not math.isfinite(wait) or wait <= 0:
            raise ValueError("CAN exchange timeout must be positive and finite")
        deadline = time.monotonic() + wait
        request = serial_can_frame(self.motor_id, command, payload)
        # Discard stale replies so the response belongs to this request.
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.buffer.clear()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ResponseTimeout(f"0x{command:02X} exchange timed out before write")
        write_with_timeout(self.fd, request, remaining)
        print(f"{time.time():.6f} TX 0x{0x140 + self.motor_id:03X}: "
              f"{request[4:-1].hex(' ').upper()}", flush=True)
        while (remaining := deadline - time.monotonic()) > 0:
            if not select.select([self.fd], [], [], remaining)[0]:
                break
            try:
                self.buffer.extend(os.read(self.fd, 512))
            except BlockingIOError:
                continue
            for kind, can_id, data in extract_frames(self.buffer):
                if kind != "standard" or can_id != 0x140 + self.motor_id or len(data) != 8:
                    continue
                print(f"{time.time():.6f} RX 0x{can_id:03X}: {data.hex(' ').upper()}", flush=True)
                if data[0] == command:
                    return data
        raise ResponseTimeout(f"no 0x{command:02X} response within {wait:g}s")

    def stop_and_disable(self) -> bool:
        ok = True
        for command, label in ((0x81, "stop"), (0x80, "disable")):
            try:
                reply = self.exchange(command)
                if reply != bytes((command,)) + b"\x00" * 7:
                    raise CommunicationError(f"{label} response payload mismatch")
                print(f"{label}: acknowledged", flush=True)
            except (OSError, CommunicationError, KeyboardInterrupt, InterruptedError) as exc:
                print(f"{label}: NOT confirmed: {exc}", file=sys.stderr, flush=True)
                ok = False
        return ok


def check_status(motor: WaveshareMotor, require_motion_ready: bool = True):
    raw1 = motor.exchange(0x9A)
    raw2 = motor.exchange(0x9C)
    status = decode_mhf9025_v3_status1(raw1, can_id=0x140 + motor.motor_id,
                                       expected_can_id=0x140 + motor.motor_id)
    feedback = decode_feedback(raw2)
    assessment = assess_mhf9025_v3_protection(status)
    print(f"status: raw={raw1.hex(' ').upper()}, temperature={status.temperature_c} C, "
          f"voltage={status.voltage_v:.2f} V (experimental), "
          f"protection={assessment.state.value}, byte6=0x{raw1[6]:02X}, byte7=0x{raw1[7]:02X}; "
          f"speed_raw={feedback.speed_dps}, provisional_speed={feedback.speed_rpm:.2f} RPM")
    print(f"protection evidence: {assessment.evidence}")
    if not (
            -40 <= status.temperature_c <= 125 and
            5 <= status.voltage_v <= 60 and
            abs(status.temperature_c - feedback.temperature_c) <= 20):
        raise CommunicationError("implausible status values; motion blocked")
    if require_motion_ready:
        require_protection_clear(assessment)
        if abs(feedback.speed_dps) > 1:
            raise CommunicationError("motor not confirmed stopped before enable")
    return status, feedback, assessment


def build_fault_report(raw1: bytes, raw2: bytes, can_id: int,
                       can_bitrate: int = 500_000) -> dict:
    """Compare fault-field hypotheses without assigning unverified bit meanings."""
    status = decode_mhf9025_v3_status1(raw1, can_id=can_id,
                                       expected_can_id=can_id, dlc=len(raw1))
    feedback = decode_feedback(raw2)
    protection = assess_mhf9025_v3_protection(status)
    byte6 = raw1[6]
    byte7 = raw1[7]
    raw16 = int.from_bytes(raw1[6:8], "little")
    return {
        "transport": "Waveshare USB serial CAN",
        "can_bitrate": can_bitrate,
        "can_id": f"0x{can_id:03X}",
        "requests": ["0x9A", "0x9C"],
        "motor_configuration_changed": False,
        "status_9a": {
            "frame_type": "standard", "dlc": len(raw1),
            "raw_hex": raw1.hex(" ").upper(),
            "bytes": [f"0x{value:02X}" for value in raw1],
            "temperature_c": status.temperature_c,
            "voltage_v": status.voltage_v,
            "voltage_basis": "bytes 2-3 LE / 100; one matching independent measurement",
        },
        "status_9c": {
            "frame_type": "standard", "dlc": len(raw2),
            "raw_hex": raw2.hex(" ").upper(),
            "bytes": [f"0x{value:02X}" for value in raw2],
            "temperature_c": feedback.temperature_c,
            "current_raw": feedback.torque_current_raw,
            "speed_raw": feedback.speed_dps,
            "speed_rpm_provisional": feedback.speed_rpm,
            "encoder_raw": feedback.encoder_raw,
        },
        "fault_field_hypotheses": [
            {
                "hypothesis": "A: byte 7 is an 8-bit error field",
                "raw_value": f"0x{byte7:02X}", "assessment": "UNKNOWN",
                "support": "older MF9025 V2.35 puts an 8-bit error field in byte 7",
                "contradiction": "older voltage layout conflicts with measured MHF9025-V3 response; "
                                 "byte 6 meaning remains unexplained",
            },
            {
                "hypothesis": "B: bytes 6-7 are a 16-bit LE error field",
                "raw_value": f"0x{raw16:04X}", "assessment": "UNKNOWN",
                "support": "a related RMD-X V3.x reference uses a 16-bit field at bytes 6-7",
                "contradiction": "that reference is for a different family; no MHF9025-V3 "
                                 "fault-bit mapping is verified",
            },
            {
                "hypothesis": "C: byte 6 is operating-state or configuration data",
                "raw_value": f"0x{byte6:02X}", "assessment": "UNKNOWN",
                "support": "read-only project captures contain byte 6 values 0x30 and 0x20 "
                           "with the same decoded 28.66 V supply; the field changes",
                "contradiction": "a changing field could encode state, fault bits, or other feedback; "
                                 "no controller-specific definition is available",
            },
            {
                "hypothesis": "D: the full MHF9025-V3 status layout differs from both references",
                "raw_value": raw1.hex(" ").upper(), "assessment": "PLAUSIBLE",
                "support": "measured voltage matches bytes 2-3 /100, not older bytes 3-4 /10",
                "contradiction": "complete field map, including protection, remains unavailable",
            },
        ],
        "protection": {"state": protection.state.value, "evidence": protection.evidence},
        "motion_ready": False,
    }


def run_fault_diagnostic(motor: WaveshareMotor) -> dict:
    """Send exactly one 0x9A and one 0x9C request; never actuate the motor."""
    raw1 = motor.exchange(0x9A)
    raw2 = motor.exchange(0x9C)
    report = build_fault_report(raw1, raw2, 0x140 + motor.motor_id,
                                motor.can_bitrate)
    print(json.dumps(report, indent=2))
    return report


LED_LOG_DIR = Path(__file__).resolve().parent / "led_logs"
LED_HARDWARE_CONFIRMATION = "MOTOR SECURED; POWER CUTOFF READY"
LED_ENABLE_CONFIRMATION = "ENABLE FOR LED TEST"


def _led_prompt(message: str, timeout: float) -> str:
    """Read a complete terminal line within a finite observation window."""
    print(message, end=" ", flush=True)
    if not select.select([sys.stdin], [], [], timeout)[0]:
        print(f"input timed out after {timeout:g}s", flush=True)
        raise CommunicationError(f"LED observation/confirmation timed out after {timeout:g}s")
    answer = sys.stdin.readline()
    if not answer:
        raise CommunicationError("LED observation/confirmation input ended")
    return answer.rstrip("\r\n")


def run_led_state_test(motor: WaveshareMotor, prompt=None) -> dict:
    """Observe disabled and enabled LEDs; never issue a motion command."""
    if motor.motor_id != 1 or motor.can_bitrate != 500_000:
        raise ValueError("LED test requires CAN ID 0x141 at 500000 bit/s")
    prompt = prompt or _led_prompt
    result = {"protection_state": ProtectionState.UNKNOWN.value}
    cleanup_ok = False
    try:
        status, feedback, assessment = check_status(motor, require_motion_ready=False)
        result["protection_state"] = assessment.state.value
        if assessment.state == ProtectionState.VERIFIED_FAULT or status.fault_state is True:
            raise ControllerFault("known active protection fault; ENABLE blocked")
        if assessment.state not in (ProtectionState.UNKNOWN, ProtectionState.VERIFIED_NORMAL):
            raise CommunicationError("unrecognized protection classification; ENABLE blocked")
        # These are only legacy V2.35 candidate bits, not a V3 fault decode.
        if status.raw[7] & 0x09:
            raise ControllerFault("0x9A byte7 has legacy V2.35 fault candidate bits; "
                                  "MHF9025-V3 mapping UNKNOWN; ENABLE blocked")
        if feedback.speed_dps != 0:
            raise CommunicationError("0x9C speed is not zero; ENABLE blocked")
        print("preflight: speed_raw=0; protection=UNKNOWN (not confirmed fault-free)", flush=True)
        if not motor.stop_and_disable():
            raise CommunicationError("initial STOP/DISABLE not confirmed; ENABLE blocked")
        before = prompt("Observe LED after STOP/DISABLE; describe color, blink interval, "
                        "and pattern, then press Enter:", 60)
        if not before.strip():
            raise CommunicationError("disabled-state LED observation is required")
        result["disabled_led_observation"] = before
        print(f"{time.time():.6f} user disabled LED observation: {before}", flush=True)
        answer = prompt(f'Type exactly "{LED_ENABLE_CONFIRMATION}" to send ENABLE only:', 60)
        print(f"{time.time():.6f} ENABLE confirmation received: {answer!r}", flush=True)
        if answer != LED_ENABLE_CONFIRMATION:
            raise CommunicationError("ENABLE confirmation declined")
        print("ENABLE will be sent without any speed, torque, or position command", flush=True)
        reply = motor.exchange(0x88)
        if reply != b"\x88" + b"\x00" * 7:
            raise CommunicationError("ENABLE response payload mismatch")
        print("enable: acknowledged; protection remains UNKNOWN", flush=True)
        after = prompt("Observe LED after ENABLE; describe color, blink interval, "
                       "and pattern within 20 seconds, then press Enter:", 20)
        if not after.strip():
            raise CommunicationError("enabled-state LED observation is required")
        result["enabled_led_observation"] = after
        print(f"{time.time():.6f} user enabled LED observation: {after}", flush=True)
    finally:
        print("LED test cleanup: attempting STOP and DISABLE", flush=True)
        cleanup_ok = motor.stop_and_disable()
        print(f"LED test cleanup: stop_disable_acknowledged={cleanup_ok}; "
              "software cannot guarantee physical stopping", flush=True)
    if not cleanup_ok:
        raise CommunicationError("final STOP/DISABLE not confirmed")
    return result


def run_logged_led_state_test(args) -> dict:
    """Record every bridge frame and terminal observation in one session log."""
    LED_LOG_DIR.mkdir(mode=0o700, parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    path = LED_LOG_DIR / f"led_state_{stamp}.log"
    with path.open("x", encoding="utf-8") as log, \
            redirect_stdout(Tee(sys.stdout, log)), redirect_stderr(Tee(sys.stderr, log)):
        print(f"LED test log: {path}", flush=True)
        print(f"start_utc={datetime.now(timezone.utc).isoformat()} "
              f"CAN_ID=0x141 bitrate=500000 device={args.device}", flush=True)
        try:
            answer = _led_prompt(
                f'Type exactly "{LED_HARDWARE_CONFIRMATION}" after securing the motor '
                "and preparing an independent power cutoff:", 60)
            print(f"{time.time():.6f} hardware confirmation received: {answer!r}", flush=True)
            if answer != LED_HARDWARE_CONFIRMATION:
                raise CommunicationError("motor securing/power cutoff confirmation declined")
            with WaveshareMotor(args.device, args.motor_id, args.can_bitrate,
                                args.serial_baud, args.timeout) as motor:
                return run_led_state_test(motor)
        except BaseException as exc:
            print(f"LED test failed: {type(exc).__name__}: {exc}; "
                  "software cannot guarantee physical stopping", file=sys.stderr, flush=True)
            raise
        finally:
            print(f"end_utc={datetime.now(timezone.utc).isoformat()}", flush=True)


# MF9025-family CAN V2.35 read commands. The MHF9025-V3 reply layouts and
# physical units below have not been verified on this controller.
EXTENDED_READ_COMMANDS = (
    (0x30, "PID parameters"),
    (0x33, "acceleration"),
    (0x90, "encoder"),
    (0x92, "multi-turn angle"),
    (0x9D, "three-phase current"),
)
EXTENDED_READ_ONLY_NOTE = "MF9025-family V2.35 interpretation; unverified for MHF9025-V3"


def decode_extended_read(command: int, reply: bytes) -> dict:
    """Decode one V2.35 read response without claiming V3 compatibility."""
    if command not in dict(EXTENDED_READ_COMMANDS):
        raise ValueError(f"unsupported extended read command 0x{command:02X}")
    if len(reply) != 8 or reply[0] != command:
        raise CommunicationError(f"malformed 0x{command:02X} reply: expected DLC 8 and matching command")
    u16 = lambda start: int.from_bytes(reply[start:start + 2], "little")
    s16 = lambda start: int.from_bytes(reply[start:start + 2], "little", signed=True)
    if command == 0x30:
        return dict(zip(("position_kp_raw", "position_ki_raw", "speed_kp_raw",
                         "speed_ki_raw", "torque_kp_raw", "torque_ki_raw"), reply[2:8]))
    if command == 0x33:
        value = int.from_bytes(reply[4:8], "little", signed=True)
        return {"acceleration_raw": value, "acceleration_dps2_provisional": value}
    if command == 0x90:
        return {"encoder_position_raw": u16(2), "encoder_original_raw": u16(4),
                "encoder_offset_raw": u16(6)}
    if command == 0x92:
        # The CAN reply carries only seven of the reference's int64 bytes.
        # Interpret those seven bytes as a signed little-endian 56-bit value.
        value = int.from_bytes(reply[1:8], "little", signed=True)
        return {"angle_raw_signed_56bit": value, "angle_degrees_provisional": value / 100}
    return {"temperature_c_provisional": int.from_bytes(reply[1:2], "little", signed=True),
            "phase_a_current_raw": s16(2), "phase_b_current_raw": s16(4),
            "phase_c_current_raw": s16(6),
            "phase_a_current_a_provisional": s16(2) / 64,
            "phase_b_current_a_provisional": s16(4) / 64,
            "phase_c_current_a_provisional": s16(6) / 64}


def run_extended_read_diagnostic(motor: WaveshareMotor) -> list[dict]:
    """Send each of the five fixed read requests once; never issue control commands."""
    if motor.motor_id != 1 or motor.can_bitrate != 500_000:
        raise ValueError("extended read diagnostic requires CAN ID 0x141 at 500000 bit/s")
    results = []
    for command, name in EXTENDED_READ_COMMANDS:
        request = bytes((command,)) + b"\x00" * 7
        result = {"command": f"0x{command:02X}", "name": name, "can_id": "0x141",
                  "frame_type": "standard", "dlc": 8,
                  "tx_data_hex": request.hex(" ").upper(),
                  "interpretation": EXTENDED_READ_ONLY_NOTE}
        try:
            # exchange logs the raw TX/RX frames, matches ID/command, and has
            # one bounded wait with no retry.
            reply = motor.exchange(command, b"\x00" * 7)
            result["rx_data_hex"] = reply.hex(" ").upper()
            result["decoded"] = decode_extended_read(command, reply)
            result["status"] = "ok"
        except ResponseTimeout as exc:
            result["status"] = "timeout"
            result["error"] = str(exc)
        results.append(result)
        print(json.dumps(result, indent=2), flush=True)
    return results


def run_speed_angle_diagnostic(motor: WaveshareMotor) -> dict:
    """Read-only timed 0x92/0x9C comparison; no motion or control commands."""
    if motor.motor_id != 1 or motor.can_bitrate != 500_000:
        raise ValueError("speed-angle diagnostic requires CAN ID 0x141 at 500000 bit/s")

    def sample(command: int, decoder):
        request_mono = time.monotonic()
        raw = motor.exchange(command)
        received_mono = time.monotonic()
        value = decoder(raw)
        print("timed_read_json: " + json.dumps({
            "command": f"0x{command:02X}", "can_data": raw.hex(" ").upper(),
            "request_started_mono": request_mono, "received_mono": received_mono,
            "round_trip_s": received_mono - request_mono, "raw_value": value,
        }), flush=True)
        return TimedRawSample(value, request_mono, received_mono)

    before = sample(0x92, lambda raw: decode_extended_read(0x92, raw)["angle_raw_signed_56bit"])
    time.sleep(0.2)
    speed = sample(0x9C, lambda raw: decode_feedback(raw).speed_dps)
    time.sleep(0.2)
    after = sample(0x92, lambda raw: decode_extended_read(0x92, raw)["angle_raw_signed_56bit"])
    result = compare_speed_with_angle(before, speed, after)
    result["interpretation"] = EXTENDED_READ_ONLY_NOTE
    print("speed_angle_comparison_json: " + json.dumps(result), flush=True)
    return result


def _encoder_delta(first: int, second: int) -> int:
    """Smallest signed change between two 16-bit raw encoder values."""
    return (second - first + 32768) % 65536 - 32768


def confirm_experimental_stationary(motor: WaveshareMotor, initial_feedback) -> None:
    """Check five read-only samples so one transient cannot decide readiness."""
    samples = [initial_feedback]
    print(f"stationary preflight: sample=1/5 speed_raw={initial_feedback.speed_dps} "
          f"encoder_raw={initial_feedback.encoder_raw}", flush=True)
    for attempt in range(4):
        time.sleep(0.1)
        sample = decode_feedback(motor.exchange(0x9C))
        samples.append(sample)
        delta = _encoder_delta(samples[0].encoder_raw, sample.encoder_raw)
        print(f"stationary preflight: sample={attempt + 2}/5 "
              f"speed_raw={sample.speed_dps} encoder_raw={sample.encoder_raw} "
              f"encoder_delta={delta}", flush=True)
    if any(abs(_encoder_delta(samples[0].encoder_raw, item.encoder_raw)) > 2
           for item in samples) or any(abs(item.speed_dps) > 3 for item in samples[1:]):
        raise CommunicationError("motor not confirmed stopped before enable: "
                                 "speed or encoder changed beyond stationary tolerance")
    print("stationary preflight: four consecutive near-zero speed samples and "
          "stable encoder across five samples (experimental tolerance ±3 deg/s, "
          "±2 raw encoder counts)", flush=True)


def confirm_stopped(motor: WaveshareMotor, *, experimental: bool = False) -> None:
    """Require bounded near-zero feedback after stop."""
    if experimental:
        recent = []
        for attempt in range(4):
            feedback = decode_feedback(motor.exchange(0x9C))
            recent.append(feedback)
            recent = recent[-3:]
            print(f"stop feedback: timestamp={time.time():.6f} "
                  f"sample={attempt + 1}/4 speed_raw={feedback.speed_dps} "
                  f"encoder_raw={feedback.encoder_raw} "
                  f"provisional_speed={feedback.speed_rpm:.2f} RPM", flush=True)
            if len(recent) == 3 and all(abs(item.speed_dps) <= 3 and
                   abs(_encoder_delta(recent[0].encoder_raw, item.encoder_raw)) <= 2
                   for item in recent):
                print("stop telemetry settled: three near-zero speed readings and "
                      "stable raw encoder; physical shaft stop not independently verified",
                      flush=True)
                return
            if attempt < 3:
                time.sleep(0.2)
        raise CommunicationError("stop acknowledged but near-zero speed and stable encoder "
                                 "were not confirmed")
    consecutive_zero = 0
    for attempt in range(4):
        feedback = decode_feedback(motor.exchange(0x9C))
        print(f"stop feedback: timestamp={time.time():.6f} "
              f"sample={attempt + 1}/4 speed_raw={feedback.speed_dps} "
              f"provisional_speed={feedback.speed_rpm:.2f} RPM")
        consecutive_zero = consecutive_zero + 1 if abs(feedback.speed_dps) <= 1 else 0
        if consecutive_zero >= 2:
            print("stopped: two consecutive speed readings within ±1 deg/s")
            return
        if attempt < 3:
            time.sleep(0.2)
    raise CommunicationError("stop acknowledged but two consecutive near-zero "
                             "speed readings were not confirmed")


EXPERIMENT_CONFIRMATION = "I ACCEPT EXPERIMENTAL MOTION"
EXPERIMENT_LOG_DIR = Path(__file__).resolve().parent / "experiment_logs"
# Local test window around the independently measured 28.66 V supply.
# These are experiment limits, not an asserted manufacturer rating.
EXPERIMENT_VOLTAGE_RANGE = (24.0, 32.0)
EXPERIMENT_TEMPERATURE_RANGE = (0, 70)


class Tee:
    def __init__(self, terminal, log):
        self.terminal = terminal
        self.log = log

    def write(self, value):
        self.terminal.write(value)
        self.log.write(value)
        self.log.flush()

    def flush(self):
        self.terminal.flush()
        self.log.flush()

    def isatty(self):
        return self.terminal.isatty()


def confirm_experimental_motion() -> None:
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise CommunicationError("experimental motion requires an interactive terminal")
    print("EXPERIMENTAL MOTION: protection state is UNKNOWN; speed command and physical "
          "direction are provisional; requested RPM may differ from actual RPM. "
          "Confirm that the motor is secured, the shaft is "
          "unobstructed, and an independent power disconnect is within reach.", flush=True)
    try:
        answer = input(f'Type exactly "{EXPERIMENT_CONFIRMATION}" to enable motion: ')
    except EOFError as exc:
        raise CommunicationError("experimental motion confirmation unavailable") from exc
    if answer != EXPERIMENT_CONFIRMATION:
        raise CommunicationError("experimental motion confirmation declined")
    print(f"{time.time():.6f} experimental confirmation accepted", flush=True)


def _run_bounded_motion(motor: WaveshareMotor, rpm: float, duration: float,
                        *, experimental: bool = False):
    flow = MotionStateMachine()
    flow.transition(MotionState.CONNECTED)
    enabled = False
    completed = False
    stopped_confirmed = False
    try:
        status, initial_feedback, assessment = check_status(
            motor, require_motion_ready=False)
        flow.transition(MotionState.STATUS_VERIFIED)
        if experimental:
            if assessment.state == ProtectionState.VERIFIED_FAULT:
                raise ControllerFault("verified motor protection fault: " + assessment.evidence)
            if assessment.state not in (ProtectionState.UNKNOWN, ProtectionState.VERIFIED_NORMAL):
                raise CommunicationError("unrecognized protection classification; motion blocked")
            raw_status = getattr(status, "raw", b"")
            if isinstance(raw_status, (bytes, bytearray)) and len(raw_status) == 8 and raw_status[7] & 0x09:
                raise ControllerFault("0x9A byte7 has legacy V2.35 fault candidate bits; "
                                      "MHF9025-V3 mapping UNKNOWN; motion blocked")
            print(f"experimental preflight limits: voltage={EXPERIMENT_VOLTAGE_RANGE[0]:g}–"
                  f"{EXPERIMENT_VOLTAGE_RANGE[1]:g} V, temperature="
                  f"{EXPERIMENT_TEMPERATURE_RANGE[0]}–{EXPERIMENT_TEMPERATURE_RANGE[1]} C "
                  "(local test limits; manufacturer limits unverified)", flush=True)
            if not EXPERIMENT_VOLTAGE_RANGE[0] <= status.voltage_v <= EXPERIMENT_VOLTAGE_RANGE[1]:
                raise CommunicationError("supply outside configured experimental voltage limits")
            if not EXPERIMENT_TEMPERATURE_RANGE[0] <= status.temperature_c <= EXPERIMENT_TEMPERATURE_RANGE[1]:
                raise CommunicationError("temperature outside configured experimental limits")
            print(f"WARNING: experimental protection={assessment.state.value}; "
                  f"{assessment.evidence}; UNKNOWN does not mean fault-free", flush=True)
        else:
            require_protection_clear(assessment)
        if experimental:
            confirm_experimental_stationary(motor, initial_feedback)
        elif abs(initial_feedback.speed_dps) > 1:
            raise CommunicationError("motor not confirmed stopped before enable")
        flow.transition(MotionState.READY)
        if experimental:
            confirm_experimental_motion()
        enabled = True  # Attempt cleanup even if enable acknowledgement is lost.
        flow.transition(MotionState.ENABLE)
        if motor.exchange(0x88) != b"\x88" + b"\x00" * 7:
            raise CommunicationError("enable response payload mismatch")
        print("enable: acknowledged")
        flow.transition(MotionState.SPEED_COMMAND)
        command_data = speed_payload(rpm)
        print("monitor_policy_json: " + json.dumps({
            "requested_rpm": rpm, "requested_dps": rpm * 6,
            "speed_command_raw_int32": int.from_bytes(command_data[3:], "little", signed=True),
            "speed_command_can_data": (b"\xA2" + command_data).hex(" ").upper(),
            "hard_speed_limit_rpm": EXPERIMENT_HARD_SPEED_RPM if experimental else 10.5,
            "tracking_band_dps": TRACKING_BAND_DPS,
            "startup_label_window_s": STARTUP_LABEL_WINDOW_S,
            "protection_state": assessment.state.value,
            "v3_speed_scaling_verified": False,
            "encoder_to_output_angle_verified": False,
        }), flush=True)
        started = time.monotonic()  # Just before submitting 0xA2; USB TX time remains uncertain.
        deadline = started + duration
        command_remaining = deadline - time.monotonic()
        if command_remaining <= 0:
            raise CommunicationError("experiment duration elapsed before speed command")
        response = motor.exchange(0xA2, command_data, timeout=command_remaining)
        command_feedback = decode_feedback(response, 0xA2)
        print(f"speed command: requested={rpm:+.2f} RPM, acknowledged; "
              f"response_speed_raw={command_feedback.speed_dps}")
        flow.transition(MotionState.MONITOR_FEEDBACK)
        observed = False
        deviation_count = 0
        previous_sample_mono = None
        while (remaining := deadline - time.monotonic()) > 0:
            request_mono = time.monotonic()
            raw_feedback = motor.exchange(0x9C, timeout=remaining)
            received_mono = time.monotonic()
            feedback = decode_feedback(raw_feedback)
            elapsed = received_mono - started
            analysis = classify_speed(feedback.speed_dps, rpm, elapsed,
                                      deviation_count, experimental=experimental)
            deviation_count = analysis["consecutive_tracking_deviations"]
            analysis.update({
                "elapsed_since_speed_command_s": elapsed,
                "feedback_request_started_mono": request_mono,
                "feedback_received_mono": received_mono,
                "feedback_rtt_s": received_mono - request_mono,
                "feedback_interval_s": None if previous_sample_mono is None
                else received_mono - previous_sample_mono,
                "feedback_can_data": raw_feedback.hex(" ").upper(),
                "encoder_raw": feedback.encoder_raw,
                "temperature_c": feedback.temperature_c,
                "torque_current_raw": feedback.torque_current_raw,
                "protection_state": assessment.state.value,
                "angle_crosscheck": "unavailable during motion; no extra CAN reads in timed loop",
                "phase_current": "unavailable during motion",
            })
            previous_sample_mono = received_mono
            print("monitor_sample_json: " + json.dumps(analysis), flush=True)
            print(f"feedback: timestamp={time.time():.6f} state={flow.state.value} "
                  f"elapsed={elapsed:.2f}s "
                  f"requested={rpm:+.2f} RPM motor_reported={feedback.speed_rpm:+.2f} RPM "
                  f"speed_raw={feedback.speed_dps} encoder_raw={feedback.encoder_raw} "
                  f"temperature={feedback.temperature_c} C "
                  f"supply={status.voltage_v:.2f} V (preflight) communication=OK "
                  f"protection_state={assessment.state.value} "
                  f"classification={analysis['category']}")
            if analysis["category"] == "critical_failure":
                print(f"stop_trigger_json: {json.dumps(analysis)}", flush=True)
                raise CommunicationError("abnormal speed feedback: " + analysis["reason"] +
                                         "; attempting immediate stop")
            observed_speed_dps = max(10, 3 * abs(rpm)) if experimental else 2
            if feedback.speed_dps * rpm >= observed_speed_dps * abs(rpm):
                observed = True
            time.sleep(min(0.2, max(0, deadline - time.monotonic())))
        if experimental:
            print("motion telemetry: " +
                  ("motor-reported speed crossed the threshold" if observed else
                   "motor-reported speed threshold not reached") +
                  "; physical shaft rotation NOT VERIFIED", flush=True)
        else:
            print("rotation: " + ("observed in requested protocol direction" if observed else
                                  "not confirmed in requested protocol direction"))
        print("stop_trigger_json: " + json.dumps({
            "reason": "duration_elapsed", "elapsed_since_speed_command_s":
            time.monotonic() - started, "duration_limit_s": duration,
        }), flush=True)
        flow.transition(MotionState.STOP)
        if motor.exchange(0x81) != b"\x81" + b"\x00" * 7:
            raise CommunicationError("stop response payload mismatch")
        print("stop: acknowledged")
        flow.transition(MotionState.VERIFY_STOP)
        confirm_stopped(motor, experimental=experimental)
        stopped_confirmed = True
        if not observed:
            raise CommunicationError("motor-reported speed threshold not confirmed"
                                     if experimental else
                                     "rotation in requested protocol direction was not confirmed")
        completed = True
    except BaseException as exc:
        print("stop_trigger_json: " + json.dumps({
            "reason": "exception", "exception_type": type(exc).__name__,
            "detail": str(exc),
        }), flush=True)
        flow.transition(MotionState.ERROR)
        raise
    finally:
        if enabled or experimental and flow.state == MotionState.ERROR:
            flow.transition(MotionState.DISABLE)
            cleanup_ok = motor.stop_and_disable()
            if experimental and not stopped_confirmed:
                try:
                    confirm_stopped(motor, experimental=True)
                    stopped_confirmed = True
                except BaseException as exc:
                    print(f"post-abort zero-speed confirmation unavailable: {exc}",
                          file=sys.stderr, flush=True)
            if experimental and not completed:
                try:
                    raw_after = motor.exchange(0x9A)
                    status_after = decode_mhf9025_v3_status1(
                        raw_after, can_id=0x140 + motor.motor_id,
                        expected_can_id=0x140 + motor.motor_id)
                    assessment_after = assess_mhf9025_v3_protection(status_after)
                    print(f"post-abort status: raw={raw_after.hex(' ').upper()} "
                          f"voltage={status_after.voltage_v:.2f} V "
                          f"temperature={status_after.temperature_c} C "
                          f"protection={assessment_after.state.value}; "
                          f"{assessment_after.evidence}", flush=True)
                except BaseException as exc:
                    print(f"post-abort status unavailable: {exc}",
                          file=sys.stderr, flush=True)
            if experimental and cleanup_ok and stopped_confirmed:
                for command, label in ((0x92, "post_stop_angle"),
                                       (0x9D, "post_stop_phase_current")):
                    try:
                        raw_extra = motor.exchange(command)
                        print(label + "_json: " + json.dumps({
                            "raw_can_data": raw_extra.hex(" ").upper(),
                            "decoded_provisional": decode_extended_read(command, raw_extra),
                            "note": "post-stop only; V3 scaling and physical motion unverified",
                        }), flush=True)
                    except Exception as exc:
                        print(f"{label} unavailable: {exc}", file=sys.stderr, flush=True)
            print(f"cleanup: stop_disable_acknowledged={cleanup_ok} "
                  f"zero_speed_confirmed={stopped_confirmed} "
                  f"(feedback only), physical_stop_verified=False", flush=True)
            if completed and not cleanup_ok:
                raise CommunicationError("motion ended, but stop/disable was not confirmed")
            if completed:
                flow.transition(MotionState.COMPLETE)


def run_motion(motor: WaveshareMotor, rpm: float, duration: float):
    _run_bounded_motion(motor, rpm, duration)


def run_experimental_motion(motor: WaveshareMotor, rpm: float, duration: float):
    if not math.isfinite(rpm) or not 0 < rpm <= 5:
        raise ValueError("experimental speed must be positive and at most 5 RPM")
    if not math.isfinite(duration) or not 0 < duration <= 2:
        raise ValueError("experimental duration must be at most 2 seconds")
    _run_bounded_motion(motor, rpm, duration, experimental=True)


def _terminate(_signum, _frame):
    raise InterruptedError("SIGTERM received")


def run_logged_experiment(args) -> None:
    EXPERIMENT_LOG_DIR.mkdir(mode=0o700, parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    path = EXPERIMENT_LOG_DIR / f"experimental_{stamp}.log"
    with path.open("x", encoding="utf-8") as log, \
            redirect_stdout(Tee(sys.stdout, log)), redirect_stderr(Tee(sys.stderr, log)):
        start = time.monotonic()
        print(f"experiment log: {path}", flush=True)
        print(f"start_utc={datetime.now(timezone.utc).isoformat()} "
              f"CAN_ID=0x{0x140 + args.motor_id:03X} bitrate={args.can_bitrate} "
              f"requested={args.speed:+g} RPM duration={args.duration:g}s", flush=True)
        try:
            with WaveshareMotor(args.device, args.motor_id, args.can_bitrate,
                                args.serial_baud, args.timeout) as motor:
                run_experimental_motion(motor, args.speed, args.duration)
            print(f"experiment CAN sequence completed: elapsed={time.monotonic() - start:.3f}s; "
                  "physical shaft rotation NOT VERIFIED",
                  flush=True)
        except BaseException as exc:
            print(f"{time.time():.6f} experiment failed: {type(exc).__name__}: {exc}; "
                  "physical stop is not guaranteed by software", file=sys.stderr, flush=True)
            raise
        finally:
            print(f"end_utc={datetime.now(timezone.utc).isoformat()} "
                  f"total_elapsed={time.monotonic() - start:.3f}s", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("status", "fault-diagnostic", "extended-read-diagnostic",
                                           "speed-angle-diagnostic", "led-state-test", "positive", "negative",
                                           "experimental-motion", "stop", "dry-run"),
                        default="status")
    parser.add_argument("--device", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--can-bitrate", type=int, choices=tuple(CAN_SPEED_CODE), default=500_000)
    parser.add_argument("--serial-baud", type=int, choices=tuple(SERIAL_SPEED), default=2_000_000)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds; default 1 for experimental motion, 2 for other modes")
    parser.add_argument("--timeout", type=float, default=0.5,
                        help="response and serial-write timeout in seconds (0 < timeout <= 2)")
    parser.add_argument("--dry-run-direction", choices=("positive", "negative", "both"),
                        default="both", help="speed frame(s) printed in dry-run mode")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--ack-protocol-compatibility", action="store_true")
    parser.add_argument("--allow-experimental-motion", action="store_true")
    args = parser.parse_args(argv)
    if args.duration is None:
        args.duration = 1.0 if args.mode == "experimental-motion" else 2.0
    if not 1 <= args.motor_id <= 32:
        parser.error("--motor-id must be 1–32")
    if not math.isfinite(args.speed) or not 0 < args.speed <= 10:
        parser.error("--speed must be greater than 0 and at most 10 RPM")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 5:
        parser.error("--duration must be greater than 0 and at most 5 seconds")
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 2:
        parser.error("--timeout must be greater than 0 and at most 2 seconds")
    if args.mode in ("extended-read-diagnostic", "speed-angle-diagnostic") and (
            args.motor_id != 1 or args.can_bitrate != 500_000):
        parser.error("read-only diagnostic requires --motor-id 1 and --can-bitrate 500000")
    if args.mode == "led-state-test":
        if args.motor_id != 1 or args.can_bitrate != 500_000:
            parser.error("LED test requires --motor-id 1 and --can-bitrate 500000")
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            parser.error("LED test requires an interactive terminal")
    if args.mode in ("positive", "negative") and not (
            args.allow_motion and args.ack_protocol_compatibility):
        parser.error("motion requires --allow-motion and --ack-protocol-compatibility")
    if args.mode == "experimental-motion":
        if not args.allow_experimental_motion:
            parser.error("experimental motion requires --allow-experimental-motion")
        if args.speed > 5 or args.duration > 2:
            parser.error("experimental motion is limited to +5 RPM and 2 seconds")
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            parser.error("experimental motion requires an interactive terminal")
    if args.mode == "dry-run":
        directions = ([args.speed, -args.speed] if args.dry_run_direction == "both" else
                      [args.speed if args.dry_run_direction == "positive" else -args.speed])
        print(f"CAN ID=0x{0x140 + args.motor_id:03X}; "
              f"{', '.join(f'{rpm:+g}' for rpm in directions)} RPM for "
              f"{args.duration:g}s each; physical CW sign unverified; no transmission")
        sequence = [(0x9A, b"\x00" * 7), (0x9C, b"\x00" * 7),
                    (0x88, b"\x00" * 7)]
        for rpm in directions:
            sequence.extend(((0xA2, speed_payload(rpm)), (0x81, b"\x00" * 7)))
        sequence.append((0x80, b"\x00" * 7))
        for command, payload in sequence:
            print(serial_can_frame(args.motor_id, command, payload).hex(' ').upper())
        return 0
    old_handler = signal.signal(signal.SIGTERM, _terminate)
    try:
        if args.mode == "experimental-motion":
            run_logged_experiment(args)
            return 0
        if args.mode == "led-state-test":
            run_logged_led_state_test(args)
            return 0
        with WaveshareMotor(args.device, args.motor_id, args.can_bitrate,
                           args.serial_baud, args.timeout) as motor:
            if args.mode == "stop":
                return 0 if motor.stop_and_disable() else 1
            if args.mode == "status":
                check_status(motor, require_motion_ready=False)
            elif args.mode == "fault-diagnostic":
                run_fault_diagnostic(motor)
            elif args.mode == "extended-read-diagnostic":
                results = run_extended_read_diagnostic(motor)
                return 1 if any(result["status"] != "ok" for result in results) else 0
            elif args.mode == "speed-angle-diagnostic":
                run_speed_angle_diagnostic(motor)
            else:
                rpm = args.speed if args.mode == "positive" else -args.speed
                run_motion(motor, rpm, args.duration)
        return 0
    except (OSError, ValueError, CommunicationError, ControllerFault, KeyboardInterrupt,
            InterruptedError, termios.error) as exc:
        suffix = ("; physical stop is not guaranteed by software"
                  if args.mode in ("positive", "negative", "experimental-motion", "stop",
                                   "led-state-test") else "")
        print(f"ERROR: {exc}{suffix}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_handler)


if __name__ == "__main__":
    sys.exit(main())
