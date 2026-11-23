"""Thread-agnostic control core for the MHF9025-V3 desktop GUI.

The controller owns exactly one transport.  It is intended to run in the GUI
worker thread; :meth:`request_stop` is the only method designed to be called
from another thread while a session is active.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import struct
import threading
import time
from typing import Callable

import absolute_home_calibration as home_calibration
from absolute_encoder_diagnostic import (decode_encoder_90, decode_multi_turn_92,
                                         decode_single_turn_94)
from motor_can import (CommunicationError, ControllerFault, ProtectionState,
                       assess_mhf9025_v3_protection, decode_feedback,
                       decode_mhf9025_v3_status1, position_hold_payload,
                       speed_payload)
from speed_analysis import classify_speed
from telemetry_analysis import DisplaySpeedEstimator
from waveshare_motor import (EXPERIMENT_TEMPERATURE_RANGE, EXPERIMENT_VOLTAGE_RANGE,
                             WaveshareMotor, _encoder_delta)


MAX_GUI_RPM = 100.0
MAX_GUI_DURATION_S = 20.0
GUI_HARD_SPEED_RPM = 100.0
GUI_CONFIRMATION = "OK"
HOLD_MAX_DRIFT_DEG = 1.0
# 0x9C has shown 0/5 dps quantization around a commanded 0.5 RPM while
# 0x92 changes smoothly. Keep a separate stricter reverse guard during approach.
HOLD_MAX_SPEED_DPS = 6
HOLD_APPROACH_REVERSE_DPS = 3
HOLD_APPROACH_RPM = 0.5
HOLD_SETTLE_MAX_SAMPLES = 30
HOLD_SETTLE_REQUIRED_SAMPLES = 8
AUTO_HOME_DEFAULT_SPEED_RPM = 5.0
AUTO_HOME_FINE_SPEED_RPM = 0.5
AUTO_HOME_FINE_WINDOW_DEG = 10.0
AUTO_HOME_MAX_TIMEOUT_S = 130.0
STOP_SETTLE_MAX_SAMPLES = 10
STOP_SETTLE_REQUIRED_SAMPLES = 3
PID_FIELD_NAMES = ("angle_kp", "angle_ki", "speed_kp", "speed_ki",
                   "torque_kp", "torque_ki")
CSV_FIELDS = (
    "utc_timestamp", "monotonic_timestamp", "elapsed_session_s", "requested_rpm",
    "actual_rpm_provisional", "raw_speed_dps", "torque_current_raw", "angle_degrees_provisional",
    "raw_angle", "encoder_raw", "temperature_c", "voltage_v", "motor_state",
    "communication_status", "protection_state", "raw_can_response",
    "event_stop_reason", "feedback_age_s", "speed_sample_hz", "angle_sample_hz",
    "sample_source", "response_rtt_s", "filtered_speed_rpm_visual",
    "angle_derived_rpm_provisional", "angle_interval_s",
    "angle_time_uncertainty_s", "raw_tracking_error_rpm",
    "filtered_tracking_error_rpm_visual", "angle_tracking_error_rpm_provisional",
    "stop_acknowledged", "disable_acknowledged",
    "zero_speed_feedback_confirmed", "physical_stop_verified",
    "stop_confirmation_detail",
)


@dataclass
class TelemetrySample:
    utc_timestamp: str
    monotonic_timestamp: float
    elapsed_session_s: float | None = None
    requested_rpm: float | None = None
    actual_rpm_provisional: float | None = None
    raw_speed_dps: int | None = None
    torque_current_raw: int | None = None
    angle_degrees_provisional: float | None = None
    raw_angle: int | None = None
    encoder_raw: int | None = None
    temperature_c: int | None = None
    voltage_v: float | None = None
    motor_state: str = "CONNECTED"
    communication_status: str = "OK"
    protection_state: str = "UNKNOWN"
    raw_can_response: str = ""
    event_stop_reason: str = ""
    feedback_age_s: float | None = None
    speed_sample_hz: float | None = None
    angle_sample_hz: float | None = None
    sample_source: str = ""
    response_rtt_s: float | None = None
    filtered_speed_rpm_visual: float | None = None
    angle_derived_rpm_provisional: float | None = None
    angle_interval_s: float | None = None
    angle_time_uncertainty_s: float | None = None
    raw_tracking_error_rpm: float | None = None
    filtered_tracking_error_rpm_visual: float | None = None
    angle_tracking_error_rpm_provisional: float | None = None
    stop_acknowledged: bool | None = None
    disable_acknowledged: bool | None = None
    zero_speed_feedback_confirmed: bool | None = None
    physical_stop_verified: bool = False
    stop_confirmation_detail: str = "not evaluated"


def validate_motion_request(rpm: float, duration_s: float) -> None:
    """Validate the currently authorized positive-direction experiment."""
    if not math.isfinite(rpm) or not 0 < rpm <= MAX_GUI_RPM:
        raise ValueError("hardware motion is limited to positive speed above 0 and at most 100 RPM")
    if not math.isfinite(duration_s) or not 0 < duration_s <= MAX_GUI_DURATION_S:
        raise ValueError("motion duration must be above 0 and at most 20 seconds")


def validate_auto_home_request(tolerance_deg: float, speed_rpm: float) -> None:
    if not math.isfinite(tolerance_deg) or not 0.1 <= tolerance_deg <= 5.0:
        raise ValueError("auto HOME tolerance must be between 0.1 and 5 degrees")
    if not math.isfinite(speed_rpm) or not 0.5 <= speed_rpm <= 5.0:
        raise ValueError("auto HOME coarse speed must be between 0.5 and 5 RPM")


def positive_distance_to_home(single_turn_degrees: float) -> float:
    """Return positive-protocol-direction travel to encoder zero in [0, 360)."""
    angle = single_turn_degrees % 360.0
    return 0.0 if math.isclose(angle, 0.0, abs_tol=1e-9) else 360.0 - angle


def distance_from_home(single_turn_degrees: float) -> float:
    angle = single_turn_degrees % 360.0
    return min(angle, 360.0 - angle)


def decode_angle_92(data: bytes) -> tuple[int, float]:
    if len(data) != 8 or data[0] != 0x92:
        raise CommunicationError("invalid 0x92 accumulated-angle response")
    raw = int.from_bytes(data[1:8], "little", signed=False)
    if raw & (1 << 55):
        raw -= 1 << 56
    return raw, raw / 100.0


def decode_pid_30(data: bytes) -> dict[str, int]:
    """Decode the six raw V2.35 PID bytes; compatibility with V3 is provisional."""
    if len(data) != 8 or data[0] != 0x30 or data[1] != 0:
        raise CommunicationError("invalid 0x30 PID response")
    return dict(zip(PID_FIELD_NAMES, data[2:8]))


class SimulatedWaveshareMotor:
    """Deterministic software transport; it never opens a device."""

    def __init__(self, device: str = "SIMULATED", motor_id: int = 1,
                 can_bitrate: int = 500_000, serial_baud: int = 2_000_000,
                 timeout: float = 0.2):
        self.device = device
        self.motor_id = motor_id
        self.can_bitrate = can_bitrate
        self.serial_baud = serial_baud
        self.timeout = timeout
        self.enabled = False
        self.commanded_rpm = 0.0
        self.angle_raw = 0
        self.encoder_raw = 20_000
        self.last_update = time.monotonic()
        self.pid = {
            "angle_kp": 100, "angle_ki": 100,
            "speed_kp": 100, "speed_ki": 50,
            "torque_kp": 100, "torque_ki": 50,
        }

    def __enter__(self):
        self.last_update = time.monotonic()
        return self

    def __exit__(self, *_):
        self.enabled = False
        self.commanded_rpm = 0.0

    def _advance(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self.last_update)
        self.last_update = now
        rpm = self.commanded_rpm if self.enabled else 0.0
        delta_deg = rpm * 6.0 * dt
        self.angle_raw += round(delta_deg * 100)
        self.encoder_raw = (self.encoder_raw + round(delta_deg * 100)) & 0xFFFF

    def exchange(self, command: int, payload: bytes = b"\x00" * 7,
                 *, timeout: float | None = None) -> bytes:
        del timeout
        self._advance()
        if command == 0x9A:
            return bytes.fromhex("9A 20 30 0B 00 00 20 00")
        if command == 0x9C:
            raw_speed = round((self.commanded_rpm if self.enabled else 0.0) * 6)
            return struct.pack("<BbhhH", 0x9C, 32, 0, raw_speed, self.encoder_raw)
        if command == 0x92:
            raw = self.angle_raw & ((1 << 56) - 1)
            return bytes((0x92,)) + raw.to_bytes(7, "little")
        if command == 0x94:
            raw = self.angle_raw % 36_000
            return bytes((0x94, 0, 0, 0)) + raw.to_bytes(4, "little")
        if command == 0x90:
            return (bytes((0x90, 0, 0, 0)) +
                    self.encoder_raw.to_bytes(2, "little") + b"\x00\x00")
        if command == 0x19:
            previous = self.encoder_raw
            self.angle_raw = 0
            self.encoder_raw = 0
            return bytes((0x19, 0, 0, 0)) + previous.to_bytes(2, "little") + b"\x00\x00"
        if command == 0x30:
            return bytes((0x30, 0, *(self.pid[name] for name in PID_FIELD_NAMES)))
        if command == 0x88:
            self.enabled = True
            return bytes((0x88,)) + b"\x00" * 7
        if command == 0xA2:
            self.commanded_rpm = int.from_bytes(payload[3:7], "little", signed=True) / 600.0
            return struct.pack("<BbhhH", 0xA2, 32, 0, 0, self.encoder_raw)
        if command == 0xA4:
            self.commanded_rpm = 0.0
            return struct.pack("<BbhhH", 0xA4, 32, 0, 0, self.encoder_raw)
        if command == 0x81:
            self.commanded_rpm = 0.0
            return bytes((0x81,)) + b"\x00" * 7
        if command == 0x80:
            self.commanded_rpm = 0.0
            self.enabled = False
            return bytes((0x80,)) + b"\x00" * 7
        raise CommunicationError(f"simulator does not implement 0x{command:02X}")

    def stop_and_disable(self) -> bool:
        return (self.exchange(0x81) == bytes((0x81,)) + b"\x00" * 7 and
                self.exchange(0x80) == bytes((0x80,)) + b"\x00" * 7)


class MotorSessionController:
    """Serialized connection, polling, bounded motion, and CSV logging."""

    def __init__(self, *, transport_factory=WaveshareMotor,
                 event_callback: Callable[[str, str], None] | None = None,
                 telemetry_callback: Callable[[dict], None] | None = None,
                 state_callback: Callable[[str], None] | None = None,
                 log_dir: Path | None = None,
                 home_log_dir: Path | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 wall_clock: Callable[[], datetime] | None = None,
                 sleep: Callable[[float], None] = time.sleep):
        self.transport_factory = transport_factory
        self.event_callback = event_callback or (lambda _level, _message: None)
        self.telemetry_callback = telemetry_callback or (lambda _sample: None)
        self.state_callback = state_callback or (lambda _state: None)
        self.log_dir = log_dir or Path(__file__).resolve().parent / "motor_logs"
        self.home_log_dir = (home_log_dir or
                             Path(__file__).resolve().parent / "absolute_home_logs")
        self.clock = clock
        self.wall_clock = wall_clock or (lambda: datetime.now(timezone.utc))
        self.sleep = sleep
        self.motor = None
        self.connected = False
        self.motion_active = False
        self.enable_uncertain = False
        self.motor_enabled = False
        self.stop_only_active = False
        self.position_hold_active = False
        self.position_hold_target_deg: float | None = None
        self.stop_requested = threading.Event()
        self.last_sample: TelemetrySample | None = None
        self.last_csv_path: Path | None = None
        self._session_start: float | None = None
        self._requested_rpm: float | None = None
        self._latest_status = None
        self._latest_assessment = None
        self._latest_speed = None
        self._latest_angle = None
        self._last_response_mono = None
        self._last_speed_mono = None
        self._last_angle_mono = None
        self._speed_hz = None
        self._angle_hz = None
        self._csv_file = None
        self._csv_writer = None
        self._pid_readback = None
        self._home_session = None
        self._home_session_path: Path | None = None
        self._home_current_check: dict | None = None
        self._display_estimator = DisplaySpeedEstimator()
        self._stop_status = {
            "stop_acknowledged": None,
            "disable_acknowledged": None,
            "zero_speed_feedback_confirmed": None,
            "physical_stop_verified": False,
            "stop_confirmation_detail": "not evaluated",
        }

    def _event(self, message: str, level: str = "INFO") -> None:
        self.event_callback(level, message)

    def _state(self, state: str) -> None:
        self.state_callback(state)
        self._event(f"Motor state: {state}")

    def connect(self, device: str, bitrate: int, motor_id: int,
                serial_baud: int = 2_000_000, timeout: float = 0.2) -> None:
        if self.connected:
            raise CommunicationError("a motor connection is already open")
        if not 1 <= motor_id <= 32 or bitrate not in (500_000, 1_000_000):
            raise ValueError("GUI requires motor ID 1-32 and CAN bitrate 500000 or 1000000")
        motor = self.transport_factory(device, motor_id, bitrate, serial_baud, timeout)
        try:
            self.motor = motor.__enter__()
            self.connected = True
            self._state("CONNECTED")
            self._event(f"Connected: {device}, CAN ID 0x{0x140 + motor_id:03X}")
        except BaseException:
            self.motor = None
            raise

    def request_stop(self, reason: str = "operator_stop") -> None:
        self._stop_reason = reason
        self.stop_requested.set()

    def _exchange(self, command: int, payload: bytes = b"\x00" * 7) -> tuple[bytes, float]:
        if not self.connected or self.motor is None:
            raise CommunicationError("motor is disconnected")
        self._event(f"TX 0x{0x140 + self.motor.motor_id:03X}: " +
                    (bytes((command,)) + payload).hex(" ").upper(), "CAN")
        started = self.clock()
        raw = self.motor.exchange(command, payload)
        received = self.clock()
        self._last_response_mono = received
        self._event(f"RX 0x{0x140 + self.motor.motor_id:03X}: {raw.hex(' ').upper()}", "CAN")
        return raw, received - started

    def _sample(self, raw: bytes, *, state: str, stop_reason: str = "",
                source: str = "", response_rtt_s: float | None = None) -> TelemetrySample:
        now = self.clock()
        sample = TelemetrySample(
            utc_timestamp=self.wall_clock().isoformat(), monotonic_timestamp=now,
            elapsed_session_s=None if self._session_start is None else now - self._session_start,
            requested_rpm=self._requested_rpm,
            actual_rpm_provisional=None if self._latest_speed is None else self._latest_speed.speed_rpm,
            raw_speed_dps=None if self._latest_speed is None else self._latest_speed.speed_dps,
            # Only a fresh 0x9C read owns this value. Other telemetry rows may
            # carry cached speed/angle fields, but must not impersonate current.
            torque_current_raw=(self._latest_speed.torque_current_raw
                                if source == "0x9C" and self._latest_speed is not None else None),
            angle_degrees_provisional=None if self._latest_angle is None else self._latest_angle[1],
            raw_angle=None if self._latest_angle is None else self._latest_angle[0],
            encoder_raw=None if self._latest_speed is None else self._latest_speed.encoder_raw,
            temperature_c=(self._latest_status.temperature_c if self._latest_status is not None else
                           self._latest_speed.temperature_c if self._latest_speed is not None else None),
            voltage_v=None if self._latest_status is None else self._latest_status.voltage_v,
            motor_state=state, communication_status="OK",
            protection_state=(ProtectionState.UNKNOWN.value if self._latest_assessment is None
                              else self._latest_assessment.state.value),
            raw_can_response=raw.hex(" ").upper(), event_stop_reason=stop_reason,
            feedback_age_s=None if self._last_response_mono is None else now - self._last_response_mono,
            speed_sample_hz=self._speed_hz,
            angle_sample_hz=self._angle_hz,
            sample_source=source,
            response_rtt_s=response_rtt_s,
            filtered_speed_rpm_visual=self._display_estimator.filtered_rpm,
            angle_derived_rpm_provisional=self._display_estimator.angle_rpm,
            angle_interval_s=self._display_estimator.angle_interval_s,
            angle_time_uncertainty_s=self._display_estimator.angle_time_uncertainty_s,
            raw_tracking_error_rpm=(None if self._requested_rpm is None or self._latest_speed is None
                                    else self._latest_speed.speed_rpm - self._requested_rpm),
            filtered_tracking_error_rpm_visual=(
                None if self._requested_rpm is None or self._display_estimator.filtered_rpm is None
                else self._display_estimator.filtered_rpm - self._requested_rpm),
            angle_tracking_error_rpm_provisional=(
                None if self._requested_rpm is None or self._display_estimator.angle_rpm is None
                else self._display_estimator.angle_rpm - self._requested_rpm),
            **self._stop_status,
        )
        self.last_sample = sample
        data = asdict(sample)
        self.telemetry_callback(data)
        if self._csv_writer is not None:
            self._csv_writer.writerow(data)
            self._csv_file.flush()
        return sample

    @staticmethod
    def _rate(now: float, previous: float | None) -> float | None:
        return None if previous is None or now <= previous else 1.0 / (now - previous)

    def read_status(self, state: str = "CONNECTED") -> TelemetrySample:
        raw, rtt = self._exchange(0x9A)
        self._latest_status = decode_mhf9025_v3_status1(
            raw, can_id=0x140 + self.motor.motor_id,
            expected_can_id=0x140 + self.motor.motor_id)
        self._latest_assessment = assess_mhf9025_v3_protection(self._latest_status)
        self._event("Protection state UNKNOWN; UNKNOWN does not mean fault-free", "WARNING")
        return self._sample(raw, state=state, source="0x9A", response_rtt_s=rtt)

    def read_speed(self, state: str = "CONNECTED") -> TelemetrySample:
        raw, rtt = self._exchange(0x9C)
        now = self.clock()
        previous = self._last_speed_mono
        self._latest_speed = decode_feedback(raw)
        self._display_estimator.update_speed(self._latest_speed.speed_rpm, now)
        self._speed_hz = self._rate(now, previous)
        self._last_speed_mono = now
        sample = self._sample(raw, state=state, source="0x9C", response_rtt_s=rtt)
        return sample

    def read_angle(self, state: str = "CONNECTED") -> TelemetrySample:
        raw, rtt = self._exchange(0x92)
        now = self.clock()
        previous = self._last_angle_mono
        self._latest_angle = decode_angle_92(raw)
        self._display_estimator.update_angle(self._latest_angle[1], now, rtt)
        self._angle_hz = self._rate(now, previous)
        self._last_angle_mono = now
        sample = self._sample(raw, state=state, source="0x92", response_rtt_s=rtt)
        return sample

    def poll_idle_once(self) -> None:
        if not self.connected or self.motion_active:
            return
        state = ("POSITION_HOLD" if self.position_hold_active else
                 "STOPPED_ENABLED" if self.motor_enabled else "CONNECTED")
        self.read_speed(state)
        self.read_angle(state)
        now = self.clock()
        if self._latest_status is None or self._last_status_poll is None or now - self._last_status_poll >= 1:
            self.read_status(state)
            self._last_status_poll = now
        if self.position_hold_active:
            drift = abs(self._latest_angle[1] - self.position_hold_target_deg)
            if (drift > HOLD_MAX_DRIFT_DEG or
                    abs(self._latest_speed.speed_dps) > HOLD_MAX_SPEED_DPS or
                    not EXPERIMENT_TEMPERATURE_RANGE[0] <= self._latest_status.temperature_c <=
                    EXPERIMENT_TEMPERATURE_RANGE[1] or
                    not EXPERIMENT_VOLTAGE_RANGE[0] <= self._latest_status.voltage_v <=
                    EXPERIMENT_VOLTAGE_RANGE[1] or
                    self._latest_status.raw[7] & 0x09):
                self._cleanup("position_hold_feedback_out_of_bounds", disable=True)
                raise CommunicationError(
                    f"position hold feedback out of bounds: angle drift={drift:.2f} deg; "
                    "STOP/DISABLE attempted")

    def read_pid(self) -> dict[str, int]:
        if not self.connected or self.motion_active:
            raise CommunicationError("PID read requires a connected, idle motor")
        raw, _ = self._exchange(0x30)
        values = decode_pid_30(raw)
        self._pid_readback = values.copy()
        self._event("PID read succeeded using provisional V2.35 byte layout; MHF9025-V3 units unverified")
        return values

    class _HomeMotorAdapter:
        """Route the reused calibration backend through the logged GUI transport."""

        def __init__(self, controller):
            self.controller = controller

        def exchange(self, command: int, payload: bytes = b"\x00" * 7) -> bytes:
            return self.controller._exchange(command, payload)[0]

    def _home_adapter(self):
        return self._HomeMotorAdapter(self)

    def _persist_home_session(self, session: dict) -> None:
        if self._home_session_path is None:
            raise RuntimeError("HOME calibration session path is unavailable")
        with self._home_session_path.open("w", encoding="utf-8") as handle:
            import json
            json.dump(session, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()

    def _require_home_idle_and_healthy(self, *, require_disabled: bool) -> None:
        if not self.connected or self.motor is None:
            raise CommunicationError("HOME calibration requires a connected motor")
        if self.motor.motor_id != 1 or self.motor.can_bitrate not in (500_000, 1_000_000):
            raise CommunicationError(
                "Absolute HOME calibration requires motor ID 1 (CAN 0x141) "
                "at 500000 or 1000000 bit/s")
        if self.motion_active:
            raise CommunicationError("HOME calibration is blocked while motor state is RUNNING")
        if require_disabled and not self._stop_status.get("disable_acknowledged"):
            raise CommunicationError(
                "HOME calibration requires an acknowledged DISABLE; press DISABLE first")
        self.read_status("HOME_CHECK")
        if (self._latest_assessment.state == ProtectionState.VERIFIED_FAULT or
                self._latest_status.fault_state is True):
            raise ControllerFault("known active protection fault blocks HOME calibration")
        if self._latest_status.raw[7] & 0x09:
            raise ControllerFault(
                "legacy V2.35 fault-candidate bits are active; V3 mapping remains UNKNOWN")

    def read_home_angles(self) -> dict:
        if not self.connected or self.motion_active:
            raise CommunicationError("absolute-angle read requires a connected, idle motor")
        raw94, _ = self._exchange(0x94)
        raw92, _ = self._exchange(0x92)
        raw90, _ = self._exchange(0x90)
        single = decode_single_turn_94(raw94).values
        multi = decode_multi_turn_92(raw92).values
        encoder = decode_encoder_90(raw90).values
        return {
            "single_turn_degrees": single["single_turn_angle_degrees_provisional"],
            "single_turn_raw": single["single_turn_angle_raw"],
            "multi_turn_degrees": multi["multi_turn_angle_degrees_provisional"],
            "multi_turn_raw": multi["multi_turn_angle_raw_s56"],
            "encoder": encoder,
            "raw_frames": {
                "0x94": raw94.hex(" ").upper(),
                "0x92": raw92.hex(" ").upper(),
                "0x90": raw90.hex(" ").upper(),
            },
        }

    def load_saved_home_session(self) -> dict | None:
        """Restore the newest recorded HOME write without accessing the motor."""
        if self.motor is not None and self.motor.motor_id != 1:
            return None
        if self._home_session is not None:
            return self._home_result(read_angles=False)
        candidates = []
        for path in self.home_log_dir.glob("home_calibration_*.json"):
            if path.is_symlink() or not path.is_file():
                continue
            try:
                if path.stat().st_size > 2_000_000:
                    continue
                with path.open(encoding="utf-8") as handle:
                    session = json.load(handle)
                hardware = session.get("hardware") or {}
                if (session.get("schema_version") != 1 or
                        not isinstance(session.get("session_id"), str) or
                        not isinstance(session.get("created_utc"), str) or
                        session.get("state") not in {state.value for state in
                                                     home_calibration.CalibrationState} or
                        not (session.get("write") or {}).get("attempted") or
                        hardware.get("can_id") != "0x141" or
                        hardware.get("can_bitrate") not in (500_000, 1_000_000)):
                    continue
                candidates.append((session.get("created_utc", ""), path, session))
            except (OSError, ValueError, TypeError, AttributeError):
                continue
        if not candidates:
            return None
        _, self._home_session_path, self._home_session = max(candidates, key=lambda item: item[0])
        self._home_current_check = None
        self._event(f"Restored saved HOME session {self._home_session['session_id']} "
                    f"({self._home_session['state']})")
        return self._home_result(read_angles=False)

    def _home_result(self, *, tolerate_angle_error: bool = False,
                     read_angles: bool = True) -> dict:
        session = self._home_session
        angle_error = None
        try:
            angles = (self.read_home_angles() if read_angles and self.connected and
                      not self.motion_active else None)
        except (OSError, CommunicationError) as exc:
            if not tolerate_angle_error:
                raise
            angles = None
            angle_error = f"{type(exc).__name__}: {exc}"
            self._event("Post-write angle read unavailable: " + angle_error, "ERROR")
        if (angles is not None and self._home_current_check is not None and
                self._home_current_check.get("passed") and
                distance_from_home(angles["single_turn_degrees"]) >
                self._home_current_check.get("tolerance_degrees", 1.0)):
            self._home_current_check = None
        return {
            "session_id": None if session is None else session["session_id"],
            "session_path": None if self._home_session_path is None else str(self._home_session_path),
            "state": home_calibration.CalibrationState.NOT_CALIBRATED.value
                     if session is None else session["state"],
            "verification_status": "NOT_PERFORMED" if session is None
                                   else session["verification_status"],
            "write": None if session is None else session["write"],
            "verification": None if session is None else session["verification"],
            "angles": angles,
            "angle_read_error": angle_error,
            "current_home_check": self._home_current_check,
        }

    def set_absolute_home(self, authorize: Callable[[dict], str]) -> dict:
        """Prepare, authorize, and make at most one 0x19 attempt."""
        self._require_home_idle_and_healthy(require_disabled=True)
        self.stop_requested.clear()
        session, path = home_calibration.create_session_file(
            self.home_log_dir, now=self.wall_clock)
        session["hardware"]["device"] = self.motor.device
        session["hardware"]["can_bitrate"] = self.motor.can_bitrate
        self._home_session = session
        self._home_session_path = path
        self._home_current_check = None
        self._persist_home_session(session)
        adapter = self._home_adapter()
        try:
            home_calibration.prepare_session(
                adapter, session, clock=self.clock, now=self.wall_clock, sleeper=self.sleep)
            preflight = session["preflight"]
            session["previous_home_information"] = {
                "single_turn_0x94": preflight["single_turn_0x94"],
                "multi_turn_0x92": preflight["multi_turn_0x92"],
                "encoder_0x90": preflight["encoder_0x90"],
            }
            self._persist_home_session(session)
            summary = {
                "session_id": session["session_id"],
                "session_path": str(path),
                "single_turn_degrees": preflight["single_turn_0x94"]["decoded"][
                    "single_turn_angle_degrees_provisional"],
                "multi_turn_degrees": preflight["multi_turn_0x92"]["decoded"][
                    "multi_turn_angle_degrees_provisional"],
                "frame_hex": home_calibration.HOME_FRAME.hex(" ").upper(),
                "angle_stability": session["angle_stability_check"],
            }
            phrase = authorize(summary)
            accepted = phrase == home_calibration.FINAL_CONFIRMATION
            session["operator_confirmation"] = {
                "timestamp_utc": self.wall_clock().isoformat(),
                "mechanical_home_aligned": accepted,
                "rotating_assembly_safe": accepted,
                "no_active_motion_command": accepted,
                "exact_phrase_accepted": accepted,
            }
            self._persist_home_session(session)
            confirmations = iter((
                home_calibration.ALIGNMENT_CONFIRMATION,
                home_calibration.SAFETY_CONFIRMATION,
                home_calibration.NO_MOTION_CONFIRMATION,
                home_calibration.FINAL_CONFIRMATION,
            ))

            def backend_authorize(_prompt: str) -> str:
                return next(confirmations) if accepted else ""

            home_calibration.write_home_once(
                adapter, session, self._persist_home_session,
                authorize=backend_authorize, clock=self.clock,
                now=self.wall_clock, sleeper=self.sleep)
        except BaseException:
            self._persist_home_session(session)
            raise
        self._event(
            f"HOME write state: {session['state']}; power-cycle verification required",
            "WARNING")
        return self._home_result(tolerate_angle_error=True)

    def verify_absolute_home(self, tolerance_deg: float,
                             authorize: Callable[[dict], str]) -> dict:
        if not math.isfinite(tolerance_deg) or not 0 < tolerance_deg <= 5:
            raise ValueError("HOME verification tolerance must be above 0 and at most 5 degrees")
        self._require_home_idle_and_healthy(require_disabled=False)
        self.stop_requested.clear()
        if self._home_session is None and self.load_saved_home_session() is None:
            raise CommunicationError(
                "no saved HOME write session is available; READ ANGLE can still show "
                "the current encoder angle")
        summary = {
            "session_id": self._home_session["session_id"],
            "tolerance_degrees": tolerance_deg,
            "state": self._home_session["state"],
        }
        phrase = authorize(summary)
        accepted = phrase == home_calibration.MECHANICAL_HOME_CONFIRMATION
        power_authorize = lambda _prompt: (
            home_calibration.POWER_CYCLE_CONFIRMATION if accepted else "")
        mechanical_authorize = lambda _prompt: (
            home_calibration.MECHANICAL_HOME_CONFIRMATION if accepted else "")
        if self._home_session["state"] == home_calibration.CalibrationState.VERIFIED_AFTER_POWER_CYCLE.value:
            self._home_current_check = home_calibration.recheck_verified_home(
                self._home_adapter(), self._home_session, self._persist_home_session,
                mechanical_authorize=mechanical_authorize, tolerance_deg=tolerance_deg,
                clock=self.clock, now=self.wall_clock, sleeper=self.sleep)
            self._event(
                "Current HOME position check " +
                ("passed" if self._home_current_check["passed"] else "failed"),
                "INFO" if self._home_current_check["passed"] else "ERROR")
        else:
            home_calibration.verify_after_power_cycle(
                self._home_adapter(), self._home_session, self._persist_home_session,
                authorize=power_authorize, mechanical_authorize=mechanical_authorize,
                tolerance_deg=tolerance_deg, clock=self.clock,
                now=self.wall_clock, sleeper=self.sleep)
            self._home_current_check = {
                "passed": self._home_session["verification"]["passed"],
                "mechanical_home_position_operator_attested": accepted,
                "tolerance_degrees": tolerance_deg,
            }
        return self._home_result()

    _last_status_poll = None

    def _preflight(self) -> None:
        self._state("PREFLIGHT")
        self.read_status("PREFLIGHT")
        self.read_speed("PREFLIGHT")
        status = self._latest_status
        assessment = self._latest_assessment
        if assessment.state == ProtectionState.VERIFIED_FAULT or status.fault_state is True:
            raise ControllerFault("known active protection fault")
        if status.raw[7] & 0x09:
            raise ControllerFault("legacy V2.35 fault-candidate bits are active; V3 mapping UNKNOWN")
        if not EXPERIMENT_VOLTAGE_RANGE[0] <= status.voltage_v <= EXPERIMENT_VOLTAGE_RANGE[1]:
            raise CommunicationError("supply voltage outside the experimental 24-32 V window")
        if not EXPERIMENT_TEMPERATURE_RANGE[0] <= status.temperature_c <= EXPERIMENT_TEMPERATURE_RANGE[1]:
            raise CommunicationError("temperature outside the experimental 0-70 C window")
        first = self._latest_speed
        for _ in range(4):
            if self.stop_requested.wait(0.1):
                raise InterruptedError(self._stop_reason)
            self.read_speed("PREFLIGHT")
            if (abs(self._latest_speed.speed_dps) > 3 or
                    abs(_encoder_delta(first.encoder_raw, self._latest_speed.encoder_raw)) > 2):
                raise CommunicationError("motor not confirmed stationary during preflight")
        self._event("Stationary preflight passed using provisional controller telemetry")

    def _read_single_turn_angle(self, state: str) -> float:
        raw, rtt = self._exchange(0x94)
        decoded = decode_single_turn_94(raw).values
        degrees = decoded["single_turn_angle_degrees_provisional"] % 360.0
        # Preserve the fresh frame in the session log even though main telemetry
        # uses accumulated 0x92 for its angle field.
        self._sample(raw, state=state, source="0x94", response_rtt_s=rtt)
        return degrees

    def _auto_home(self, tolerance_deg: float, coarse_speed_rpm: float, *,
                   position_hold: bool = False) -> None:
        """Move to encoder zero after measuring the angle direction of positive 0xA2."""
        validate_auto_home_request(tolerance_deg, coarse_speed_rpm)
        self._state("AUTO_HOMING")
        initial_angle = self._read_single_turn_angle("AUTO_HOMING")
        if distance_from_home(initial_angle) <= tolerance_deg:
            self._event(
                f"Auto HOME already within tolerance: angle={initial_angle:.2f} deg, "
                f"tolerance={tolerance_deg:.2f} deg")
            if position_hold:
                self._hold_at_home(tolerance_deg)
                for index in range(5):
                    if self.stop_requested.is_set():
                        raise InterruptedError(self._stop_reason)
                    checked = self._read_single_turn_angle("AUTO_HOME_VERIFY")
                    if distance_from_home(checked) > tolerance_deg:
                        raise CommunicationError("HOME angle left tolerance after position command")
                    if index < 4:
                        self.sleep(0.05)
            return

        self._event(
            f"Auto HOME starting from {initial_angle:.2f} deg; measuring encoder "
            f"direction at {AUTO_HOME_FINE_SPEED_RPM:.2f} RPM before approach", "WARNING")
        self.position_hold_active = False
        self.position_hold_target_deg = None
        self.enable_uncertain = True
        raw, _ = self._exchange(0x88)
        if raw != bytes((0x88,)) + b"\x00" * 7:
            raise CommunicationError("auto HOME ENABLE acknowledgement mismatch")
        self.motor_enabled = True

        commanded = AUTO_HOME_FINE_SPEED_RPM
        raw, _ = self._exchange(0xA2, speed_payload(commanded))
        decode_feedback(raw, 0xA2)
        self._requested_rpm = commanded
        started = self.clock()
        timeout_s = 1.5  # bounded low-speed direction observation
        direction = None
        remaining = None
        previous_remaining = None
        next_speed = started
        reached = False
        # Recent hardware logs show about 0.20 deg of position change between
        # first entering a 1 deg band and settling after STOP. Approach
        # deeper into the operator-selected band, then retain the original
        # tolerance for the post-stop acceptance check.
        capture_tolerance_deg = min(0.4, tolerance_deg * 0.4)
        self._event(
            f"Auto HOME capture threshold={capture_tolerance_deg:.2f} deg; "
            f"final tolerance={tolerance_deg:.2f} deg")
        while self.clock() - started < timeout_s:
            if self.stop_requested.is_set():
                raise InterruptedError(self._stop_reason)
            angle = self._read_single_turn_angle("AUTO_HOMING")
            if distance_from_home(angle) <= capture_tolerance_deg:
                reached = True
                self._event(
                    f"Auto HOME angle entered tolerance: {angle:.2f} deg "
                    f"(distance {distance_from_home(angle):.2f} deg)")
                break
            observed_change = ((angle - initial_angle + 180.0) % 360.0) - 180.0
            if direction is None and abs(observed_change) >= 0.25:
                direction = 1 if observed_change > 0 else -1
                remaining = positive_distance_to_home(angle) if direction > 0 else angle
                previous_remaining = remaining
                coarse_degrees = max(0.0, remaining - AUTO_HOME_FINE_WINDOW_DEG)
                fine_degrees = min(remaining, AUTO_HOME_FINE_WINDOW_DEG)
                expected_s = (coarse_degrees / (coarse_speed_rpm * 6.0) +
                              fine_degrees / (AUTO_HOME_FINE_SPEED_RPM * 6.0))
                timeout_s = min(AUTO_HOME_MAX_TIMEOUT_S,
                                max(10.0, expected_s * 1.75 + 5.0))
                self._event(
                    f"Auto HOME observed encoder direction={direction:+d}; "
                    f"remaining={remaining:.2f} deg", "WARNING")
            elif direction is not None:
                remaining = positive_distance_to_home(angle) if direction > 0 else angle
                if remaining > previous_remaining + 0.75:
                    raise CommunicationError(
                        "auto HOME angle moved away from zero or feedback jumped")
                previous_remaining = remaining
            if direction is not None:
                desired = (AUTO_HOME_FINE_SPEED_RPM
                           if remaining <= AUTO_HOME_FINE_WINDOW_DEG else coarse_speed_rpm)
            else:
                desired = AUTO_HOME_FINE_SPEED_RPM
            if commanded != desired:
                commanded = desired
                raw, _ = self._exchange(0xA2, speed_payload(commanded))
                decode_feedback(raw, 0xA2)
                self._requested_rpm = commanded
                self._event(
                    f"Auto HOME approach: {commanded:.2f} RPM, "
                    f"remaining={remaining:.2f} deg")
            now = self.clock()
            if now >= next_speed:
                self.read_speed("AUTO_HOMING")
                actual = self._latest_speed.speed_rpm
                if actual < -0.5:
                    raise CommunicationError("unexpected reverse motion during auto HOME")
                limit = max(6.0, coarse_speed_rpm * 2.0)
                if actual > limit:
                    raise CommunicationError(
                        f"auto HOME speed {actual:.2f} RPM exceeds {limit:.2f} RPM limit")
                next_speed = now + 0.1
            self.stop_requested.wait(0.02)
        if not reached:
            raise CommunicationError(
                "auto HOME could not confirm angle direction within 1.5 seconds"
                if direction is None else
                f"auto HOME did not reach tolerance within {timeout_s:.1f} seconds")

        if position_hold:
            self._hold_at_home(tolerance_deg)
        else:
            self._state("AUTO_HOME_STOPPING")
            if not self._cleanup("auto_home_target_reached", disable=False):
                raise CommunicationError("auto HOME STOP was not confirmed")
        self._requested_rpm = None

        final_angles = []
        for index in range(5):
            if self.stop_requested.is_set():
                raise InterruptedError(self._stop_reason)
            final_angles.append(self._read_single_turn_angle("AUTO_HOME_VERIFY"))
            if index < 4:
                self.sleep(0.05)
        final_distances = [distance_from_home(value) for value in final_angles]
        if max(final_distances) > tolerance_deg:
            raise CommunicationError(
                "auto HOME final angle is outside tolerance after STOP: "
                + ", ".join(f"{value:.2f}" for value in final_angles) + " deg")
        self._event(
            f"Auto HOME complete: five fresh 0x94 samples within ±{tolerance_deg:.2f} deg; "
            "controller angle does not independently prove mechanical alignment")

    def _open_csv(self) -> None:
        self.log_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        stamp = self.wall_clock().strftime("%Y%m%dT%H%M%S%fZ")
        self.last_csv_path = self.log_dir / f"motor_session_{stamp}.csv"
        self._csv_file = self.last_csv_path.open("x", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._csv_writer.writeheader()

    def _close_csv(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
        self._csv_file = self._csv_writer = None

    def _cleanup(self, reason: str, *, disable: bool = True) -> bool:
        if not self.connected or self.motor is None:
            return False
        self.position_hold_active = False
        self.position_hold_target_deg = None
        ok = True
        self._stop_status = {
            "stop_acknowledged": False,
            "disable_acknowledged": False if disable else None,
            "zero_speed_feedback_confirmed": False, "physical_stop_verified": False,
            "stop_confirmation_detail": "cleanup in progress",
        }
        commands = ((0x81, "STOP"), (0x80, "DISABLE")) if disable else ((0x81, "STOP"),)
        for command, name in commands:
            try:
                raw, _ = self._exchange(command)
                if raw != bytes((command,)) + b"\x00" * 7:
                    raise CommunicationError(f"{name} acknowledgement mismatch")
                self._event(f"{name} acknowledged")
                self._stop_status["stop_acknowledged" if command == 0x81 else
                                  "disable_acknowledged"] = True
                if command == 0x80:
                    self.motor_enabled = False
                    self.stop_only_active = False
            except BaseException as exc:
                ok = False
                self._event(f"{name} NOT confirmed: {exc}", "ERROR")
        zero_confirmed = False
        recent = []
        settled_streak = 0
        previous_encoder = None
        try:
            for attempt in range(STOP_SETTLE_MAX_SAMPLES):
                self.read_speed("VERIFY_STOP")
                recent.append(self._latest_speed)
                speed_ok = abs(self._latest_speed.speed_dps) <= 3
                encoder_ok = (previous_encoder is None or
                              abs(_encoder_delta(previous_encoder,
                                                 self._latest_speed.encoder_raw)) <= 2)
                settled_streak = settled_streak + 1 if speed_ok and encoder_ok else 0
                self._event(
                    f"Stop verification {attempt + 1}/{STOP_SETTLE_MAX_SAMPLES}: "
                    f"speed={self._latest_speed.speed_dps} deg/s, "
                    f"encoder={self._latest_speed.encoder_raw}, "
                    f"settled_streak={settled_streak}/{STOP_SETTLE_REQUIRED_SAMPLES}")
                previous_encoder = self._latest_speed.encoder_raw
                if settled_streak >= STOP_SETTLE_REQUIRED_SAMPLES:
                    zero_confirmed = True
                    break
                if attempt < STOP_SETTLE_MAX_SAMPLES - 1:
                    self.sleep(0.05)
        except BaseException as exc:
            self._event(f"Zero-speed feedback not confirmed: {exc}", "ERROR")
        if zero_confirmed:
            self._event("Zero speed confirmed by controller telemetry only; physical stop is not verified")
        else:
            ok = False
            self._event("Zero-speed feedback not confirmed; hardware cutoff required", "ERROR")
        self._stop_status["zero_speed_feedback_confirmed"] = zero_confirmed
        ack_ok = (self._stop_status["stop_acknowledged"] and
                  (not disable or self._stop_status["disable_acknowledged"]))
        self._stop_status["stop_confirmation_detail"] = (
            ("STOP and DISABLE acknowledged" if disable else
             "STOP acknowledged; DISABLE not sent") +
            "; three consecutive controller speed/encoder "
            "samples settled within a bounded ten-sample window; "
            "physical stop remains unverified" if ok else
            f"STOP ack={self._stop_status['stop_acknowledged']}, "
            f"DISABLE ack={self._stop_status['disable_acknowledged']}, "
            f"controller zero-speed feedback={zero_confirmed}")
        ok = bool(ok and ack_ok and zero_confirmed)
        if not ok and not disable:
            self._event("STOP-only confirmation failed; attempting STOP/DISABLE", "ERROR")
            self._cleanup(reason, disable=True)
            return False
        self.enable_uncertain = not ok
        if ok and not disable:
            self.stop_only_active = True
        state = ("DISABLED" if disable else
                 "STOPPED_ENABLED" if self.motor_enabled else "IDLE") if ok else "STOP_UNCONFIRMED"
        self._sample(b"", state=state, stop_reason=reason,
                     source="cleanup_result")
        if not ok:
            self._event("Hardware cutoff required; software cannot guarantee physical stopping", "ERROR")
        return ok

    def _decelerate_for_position_hold(self) -> None:
        """Keep speed control active while reaching a bounded low speed."""
        self._state("HOLD_APPROACH")
        raw, _ = self._exchange(0xA2, speed_payload(HOLD_APPROACH_RPM))
        decode_feedback(raw, 0xA2)
        self._requested_rpm = HOLD_APPROACH_RPM
        settled = 0
        previous_angle = None
        previous_angle_time = None
        deadline = self.clock() + 1.5
        for index in range(HOLD_SETTLE_MAX_SAMPLES):
            if self.stop_requested.is_set():
                raise InterruptedError(self._stop_reason)
            if self.clock() >= deadline:
                break
            self.read_speed("HOLD_APPROACH")
            self.read_angle("HOLD_APPROACH")
            if index % 5 == 0:
                self.read_status("HOLD_APPROACH")
                status = self._latest_status
                if (self._latest_assessment.state == ProtectionState.VERIFIED_FAULT or
                        status.fault_state is True or status.raw[7] & 0x09 or
                        not EXPERIMENT_VOLTAGE_RANGE[0] <= status.voltage_v <=
                        EXPERIMENT_VOLTAGE_RANGE[1] or
                        not EXPERIMENT_TEMPERATURE_RANGE[0] <= status.temperature_c <=
                        EXPERIMENT_TEMPERATURE_RANGE[1]):
                    raise ControllerFault("fault or operating limit during hold approach")
            speed = self._latest_speed.speed_dps
            angle = self._latest_angle[1]
            angle_time = self._last_angle_mono
            if speed < -HOLD_APPROACH_REVERSE_DPS or abs(speed) > GUI_HARD_SPEED_RPM * 6:
                raise CommunicationError("unsafe speed or direction during hold approach")
            interval = (None if previous_angle_time is None else
                        angle_time - previous_angle_time)
            angle_rate = (None if interval is None or interval < 0.02 else
                          (angle - previous_angle) / interval)
            if (abs(speed) <= HOLD_MAX_SPEED_DPS and angle_rate is not None and
                    -HOLD_APPROACH_REVERSE_DPS <= angle_rate <= HOLD_MAX_SPEED_DPS):
                settled += 1
            else:
                settled = 0
            if settled >= HOLD_SETTLE_REQUIRED_SAMPLES:
                self._event("Low-speed hold approach confirmed by fresh 0x9C/0x92 samples")
                return
            previous_angle = angle
            previous_angle_time = angle_time
            if index < HOLD_SETTLE_MAX_SAMPLES - 1:
                if self.stop_requested.wait(0.05):
                    raise InterruptedError(self._stop_reason)
        raise CommunicationError("low-speed hold approach did not settle within bounded window")

    def _hold_at_home(self, tolerance_deg: float) -> None:
        """Use matching fresh 0x94/0x92 angles to target the nearest HOME turn."""
        single = self._read_single_turn_angle("POSITION_HOLD_PREPARE")
        if distance_from_home(single) > tolerance_deg:
            raise CommunicationError("HOME angle left tolerance before position command")
        self.read_angle("POSITION_HOLD_PREPARE")
        current_raw, current_deg = self._latest_angle
        discrepancy = ((current_deg - single + 180.0) % 360.0) - 180.0
        if abs(discrepancy) > 1.0:
            raise CommunicationError("0x92 and 0x94 HOME angles disagree; position command blocked")
        home_delta = ((-single + 180.0) % 360.0) - 180.0
        target_raw = current_raw + round(home_delta * 100)
        self._engage_position_hold(target_raw)

    def _engage_position_hold(self, target_raw: int | None = None) -> None:
        """Switch an enabled motor directly to experimental 0xA4 position mode."""
        self.read_status("POSITION_HOLD_PREPARE")
        status = self._latest_status
        if (self._latest_assessment.state == ProtectionState.VERIFIED_FAULT or
                status.fault_state is True or status.raw[7] & 0x09):
            raise ControllerFault("fault indication during position-hold transition")
        if (not EXPERIMENT_VOLTAGE_RANGE[0] <= status.voltage_v <= EXPERIMENT_VOLTAGE_RANGE[1]
                or not EXPERIMENT_TEMPERATURE_RANGE[0] <= status.temperature_c <=
                EXPERIMENT_TEMPERATURE_RANGE[1]):
            raise CommunicationError("voltage or temperature outside experimental hold limits")
        self.read_angle("POSITION_HOLD_PREPARE")
        current_raw = self._latest_angle[0]
        if target_raw is None:
            target_raw = current_raw
        if abs(target_raw - current_raw) > 100:
            raise CommunicationError("hold target is more than 1 degree from fresh position")
        payload = position_hold_payload(target_raw)
        target_deg = target_raw / 100.0
        self._event(
            f"EXPERIMENTAL direct position transition: target={target_deg:.2f} deg, "
            "0xA4 speed limit=1 deg/s; V3 behavior under load unverified", "WARNING")
        if not self.motor_enabled:
            self.enable_uncertain = True
            raw, _ = self._exchange(0x88)
            if raw != bytes((0x88,)) + b"\x00" * 7:
                raise CommunicationError("position hold ENABLE acknowledgement mismatch")
            self.motor_enabled = True
        self.enable_uncertain = True
        raw, _ = self._exchange(0xA4, payload)
        decode_feedback(raw, 0xA4)
        for index in range(5):
            if self.stop_requested.is_set():
                raise InterruptedError(self._stop_reason)
            self.read_speed("POSITION_HOLD_VERIFY")
            self.read_angle("POSITION_HOLD_VERIFY")
            drift = abs(self._latest_angle[1] - target_deg)
            if drift > HOLD_MAX_DRIFT_DEG or abs(self._latest_speed.speed_dps) > HOLD_MAX_SPEED_DPS:
                raise CommunicationError(
                    f"position hold verification failed: angle drift={drift:.2f} deg, "
                    f"speed={self._latest_speed.speed_dps} deg/s")
            if index < 4:
                if self.stop_requested.wait(0.1):
                    raise InterruptedError(self._stop_reason)
        self.position_hold_target_deg = target_deg
        self.position_hold_active = True
        self.enable_uncertain = False
        self._event("Experimental position hold accepted by controller feedback only; physical hold unverified")

    def home_only(self, tolerance_deg: float, speed_rpm: float, *,
                  authorization: str | None = None,
                  authorization_callback: Callable[[], str] | None = None,
                  position_hold: bool = False) -> None:
        """Run the existing bounded homing sequence without a speed session."""
        validate_auto_home_request(tolerance_deg, speed_rpm)
        if not self.connected or self.motion_active:
            raise CommunicationError("motor must be connected and idle")
        self.stop_requested.clear()
        self._home_current_check = None
        self._stop_reason = "operator_stop"
        self.motion_active = True
        self._requested_rpm = None
        self._session_start = None
        self._open_csv()
        failure = None
        try:
            self._preflight()
            response = authorization_callback() if authorization_callback is not None else authorization
            if response != GUI_CONFIRMATION:
                raise PermissionError("HOME motion authorization phrase was not accepted")
            self._event("Operator confirmed motor secured and independent power cutoff ready", "WARNING")
            self._auto_home(tolerance_deg, speed_rpm, position_hold=position_hold)
            if self.stop_requested.is_set():
                raise InterruptedError(self._stop_reason)
            self._event("GO HOME complete; target speed and duration were not commanded")
        except BaseException as exc:
            failure = exc
            self._event(f"GO HOME stopped: {type(exc).__name__}: {exc}", "ERROR")
        finally:
            # A failed transition gets a bounded STOP/DISABLE cleanup.
            cleanup_ok = True
            if failure is not None or self.enable_uncertain:
                self._state("STOPPING")
                cleanup_ok = self._cleanup(
                    "home_only_complete" if failure is None else
                    f"{type(failure).__name__}: {failure}",
                    disable=not (isinstance(failure, InterruptedError) and
                                 self._stop_reason == "operator_stop"))
            self.motion_active = False
            self._requested_rpm = None
            self._session_start = None
            self.stop_requested.clear()
            self._close_csv()
            self._state(("POSITION_HOLD" if self.position_hold_active else
                         "STOPPED_ENABLED" if self.motor_enabled else "IDLE")
                        if cleanup_ok else "STOP_UNCONFIRMED")
        if failure is not None:
            raise failure
        if not cleanup_ok:
            raise CommunicationError("GO HOME stop was not confirmed")

    def start_motion(self, rpm: float, duration_s: float, *, authorization: str | None = None,
                     authorization_callback: Callable[[], str] | None = None,
                     auto_home: bool = True, home_tolerance_deg: float = 1.0,
                     home_speed_rpm: float = AUTO_HOME_DEFAULT_SPEED_RPM,
                     position_hold: bool = False) -> None:
        validate_motion_request(rpm, duration_s)
        if auto_home:
            validate_auto_home_request(home_tolerance_deg, home_speed_rpm)
        if not self.connected or self.motion_active:
            raise CommunicationError("motor must be connected and idle")
        self.stop_requested.clear()
        self._home_current_check = None
        self._stop_reason = "duration_elapsed"
        self.motion_active = True
        self._requested_rpm = rpm
        self._session_start = None
        self._open_csv()
        failure = None
        hold_completed = False
        try:
            self._preflight()
            response = authorization_callback() if authorization_callback is not None else authorization
            if response != GUI_CONFIRMATION:
                raise PermissionError("motion authorization phrase was not accepted")
            self._event("Operator confirmed motor secured and independent power cutoff ready", "WARNING")
            if auto_home:
                self._auto_home(home_tolerance_deg, home_speed_rpm,
                                position_hold=position_hold)
                if not position_hold:
                    self._preflight()
            self.position_hold_active = False
            self.position_hold_target_deg = None
            self._requested_rpm = rpm
            # Start visualization estimators at the motion boundary so an idle
            # angle interval is never interpreted as a running-speed interval.
            self._display_estimator = DisplaySpeedEstimator()
            self.enable_uncertain = True
            raw, _ = self._exchange(0x88)
            if raw != bytes((0x88,)) + b"\x00" * 7:
                raise CommunicationError("ENABLE acknowledgement mismatch")
            self._event("ENABLE acknowledged")
            self.motor_enabled = True
            self._session_start = self.clock()  # Duration starts immediately before 0xA2.
            raw, _ = self._exchange(0xA2, speed_payload(rpm))
            decode_feedback(raw, 0xA2)
            self._event(f"Speed command acknowledged: {rpm:+.2f} RPM (provisional)")
            self._state("RUNNING")
            deadline = self._session_start + duration_s
            next_speed = next_angle = next_status = self.clock()
            deviation_count = 0
            while self.clock() < deadline:
                if self.stop_requested.is_set():
                    raise InterruptedError(self._stop_reason)
                now = self.clock()
                if now >= next_speed:
                    sample = self.read_speed("RUNNING")
                    analysis = classify_speed(self._latest_speed.speed_dps, rpm,
                                              sample.elapsed_session_s or 0,
                                              deviation_count, experimental=True,
                                              hard_limit_rpm=GUI_HARD_SPEED_RPM)
                    deviation_count = analysis["consecutive_tracking_deviations"]
                    if analysis["category"] == "critical_failure":
                        self._event("Overspeed/direction protection: " + analysis["reason"], "ERROR")
                        raise CommunicationError(analysis["reason"])
                    next_speed = now + 0.125
                if self.stop_requested.is_set():
                    raise InterruptedError(self._stop_reason)
                now = self.clock()
                if now >= next_angle:
                    self.read_angle("RUNNING")
                    next_angle = now + 0.1
                if now >= next_status:
                    self.read_status("RUNNING")
                    next_status = now + 1.0
                self.stop_requested.wait(min(0.01, max(0.0, deadline - self.clock())))
            if position_hold:
                self._decelerate_for_position_hold()
                self._engage_position_hold()
                hold_completed = True
        except BaseException as exc:
            failure = exc
            if isinstance(exc, InterruptedError):
                self._event(f"STOP requested: {exc}", "WARNING")
            else:
                self._event(f"Motion stopped: {type(exc).__name__}: {exc}", "ERROR")
        finally:
            reason = self._stop_reason if isinstance(failure, InterruptedError) else (
                "duration_elapsed" if failure is None else f"{type(failure).__name__}: {failure}")
            if not hold_completed:
                self._state("STOPPING")
            stop_only = (
                (failure is None or isinstance(failure, InterruptedError)) and
                self._stop_reason in ("duration_elapsed", "operator_stop")
            )
            cleanup_ok = True if hold_completed else self._cleanup(
                reason, disable=not stop_only)
            self.motion_active = False
            self._requested_rpm = None
            self._session_start = None
            self.stop_requested.clear()
            self._close_csv()
            self._state(("POSITION_HOLD" if self.position_hold_active else
                         "STOPPED_ENABLED" if self.motor_enabled else "IDLE")
                        if cleanup_ok else "STOP_UNCONFIRMED")
        if failure is not None and not isinstance(failure, InterruptedError):
            raise failure
        if not cleanup_ok:
            raise CommunicationError(
                "normal STOP was not confirmed; STOP/DISABLE fallback attempted; "
                "physical stopping is not guaranteed")

    def disable(self, reason: str = "operator_disable") -> bool:
        self.request_stop(reason)
        if self.motion_active:
            return True
        ok = self._cleanup(reason)
        self._state("IDLE" if ok else "STOP_UNCONFIRMED")
        return ok

    def stop_position_hold(self) -> bool:
        """End an idle experimental hold with STOP; retain the enabled state."""
        if not self.connected or self.motion_active or not self.position_hold_active:
            return False
        ok = self._cleanup("operator_stop_position_hold", disable=False)
        self._state("STOPPED_ENABLED" if ok else "STOP_UNCONFIRMED")
        return ok

    def disconnect(self) -> None:
        if self.motor is None:
            return
        if self.motion_active or self.enable_uncertain or self.motor_enabled or self.stop_only_active:
            self.request_stop("disconnect")
            self._cleanup("disconnect")
        self.motor.__exit__(None, None, None)
        self.motor = None
        self.connected = False
        self.motion_active = False
        self.motor_enabled = False
        self.stop_only_active = False
        self._pid_readback = None
        self._state("DISCONNECTED")

