"""Small, deliberately bounded SocketCAN client for the MF9025-family protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import re
import struct
import subprocess
import time

try:
    import can
except ImportError:  # Allow software-only frame generation without python-can.
    can = None


MAX_RPM = 100.0
RESPONSE_TIMEOUT = 0.5


class MotorError(Exception):
    """Base error for motor communication and validation."""


class CommunicationError(MotorError):
    """The CAN interface or the expected reply was unavailable."""


class ResponseTimeout(CommunicationError):
    """No matching response arrived before the deadline."""


class ControllerFault(MotorError):
    """The controller reported protection or unknown error flags."""


class ProtectionState(str, Enum):
    VERIFIED_NORMAL = "VERIFIED_NORMAL"
    VERIFIED_FAULT = "VERIFIED_FAULT"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class ProtectionAssessment:
    state: ProtectionState
    evidence: str


@dataclass(frozen=True)
class Status1:
    temperature_c: int
    voltage_v: float
    error: int
    raw: bytes = b""

    @property
    def layout_valid(self) -> bool:
        """Whether reserved bytes match the supplied MF9025-family layout."""
        return not self.raw or (len(self.raw) == 8 and
                                self.raw[2] == self.raw[5] == self.raw[6] == 0)

    @property
    def undervoltage(self) -> bool:
        return bool(self.error & 0x01)

    @property
    def overtemperature(self) -> bool:
        return bool(self.error & 0x08)

    @property
    def unknown_error_bits(self) -> int:
        return self.error & ~0x09


@dataclass(frozen=True)
class MHF9025V3Status1:
    """Model-specific 0x9A status; only voltage has experimental support."""
    raw: bytes
    temperature_c: int
    voltage_v: float
    fault_state: bool | None = None  # No verified fault-field mapping for MHF9025-V3.


def assess_mhf9025_v3_protection(status: MHF9025V3Status1) -> ProtectionAssessment:
    """Classify only what the available MHF9025-V3 evidence actually supports."""
    fields = (f"0x9A byte6=0x{status.raw[6]:02X}, byte7=0x{status.raw[7]:02X}"
              if len(status.raw) == 8 else "0x9A raw payload unavailable")
    return ProtectionAssessment(
        ProtectionState.UNKNOWN,
        f"{fields}; "
        "neither field has a verified MHF9025-V3 protection definition; "
        "0x9C provides speed/current/encoder, not independent fault status",
    )


def decode_mhf9025_v3_status1(data: bytes, *, can_id: int,
                              expected_can_id: int = 0x141,
                              is_extended: bool = False,
                              dlc: int | None = None) -> MHF9025V3Status1:
    """Decode voltage from bytes 2-3 at 0.01 V/LSB, supported by one measurement."""
    if (can_id != expected_can_id or is_extended or len(data) != 8 or
            (dlc is not None and dlc != 8) or data[0] != 0x9A):
        raise CommunicationError("invalid MHF9025-V3 status 1 CAN frame")
    raw = bytes(data)
    return MHF9025V3Status1(raw=raw,
                            temperature_c=struct.unpack_from("<b", raw, 1)[0],
                            voltage_v=int.from_bytes(raw[2:4], "little") / 100.0)


def decode_mhf9025_v3_voltage(data: bytes, *, can_id: int = 0x141) -> float:
    """Convenience decoder for the experimentally supported voltage field."""
    return decode_mhf9025_v3_status1(data, can_id=can_id).voltage_v


@dataclass(frozen=True)
class Feedback:
    temperature_c: int
    torque_current_raw: int
    speed_dps: int
    encoder_raw: int

    @property
    def speed_rpm(self) -> float:
        return self.speed_dps / 6.0


def validate_speed(rpm: float) -> None:
    if not math.isfinite(rpm) or abs(rpm) > MAX_RPM:
        raise ValueError(f"speed must be finite and within ±{MAX_RPM:g} RPM")


def speed_payload(rpm: float) -> bytes:
    validate_speed(rpm)
    return b"\x00\x00\x00" + struct.pack("<i", round(rpm * 600))


def position_hold_payload(angle_raw_centidegrees: int, speed_limit_dps: int = 1) -> bytes:
    """V2.36 0xA4 payload: unsigned degree/s limit and signed 0.01-degree target."""
    if (not isinstance(angle_raw_centidegrees, int) or isinstance(angle_raw_centidegrees, bool)
            or not -(1 << 31) <= angle_raw_centidegrees < (1 << 31)):
        raise ValueError("position hold angle must fit signed 32-bit centidegrees")
    if (not isinstance(speed_limit_dps, int) or isinstance(speed_limit_dps, bool)
            or not 1 <= speed_limit_dps <= 0xffff):
        raise ValueError("position speed limit must be an integer in 1..65535 deg/s")
    return b"\x00" + struct.pack("<H", speed_limit_dps) + struct.pack("<i", angle_raw_centidegrees)


def decode_status1(data: bytes) -> Status1:
    if len(data) != 8 or data[0] != 0x9A:
        raise CommunicationError("invalid status 1 response")
    temperature = struct.unpack_from("<b", data, 1)[0]
    voltage = struct.unpack_from("<H", data, 3)[0] / 10.0
    return Status1(temperature, voltage, data[7], bytes(data))


def decode_feedback(data: bytes, command: int = 0x9C) -> Feedback:
    if len(data) != 8 or data[0] != command or command not in (0x9C, 0xA2, 0xA4):
        raise CommunicationError("invalid feedback response")
    return Feedback(struct.unpack_from("<b", data, 1)[0],
                    struct.unpack_from("<h", data, 2)[0],
                    struct.unpack_from("<h", data, 4)[0],
                    struct.unpack_from("<H", data, 6)[0])


def check_socketcan(channel: str, expected_bitrate: int = 1_000_000) -> str:
    """Check live link state and bitrate; fail closed if they cannot be read."""
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", channel):
        raise ValueError("invalid CAN interface name")
    try:
        result = subprocess.run(["ip", "-details", "link", "show", "dev", channel],
                                check=True, capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise CommunicationError(f"cannot inspect {channel}: {exc}") from exc
    output = result.stdout
    if not re.search(r"\bstate UP\b", output) and not re.search(r"<[^>]*\bUP\b[^>]*>", output):
        raise CommunicationError(f"{channel} is not UP")
    if re.search(r"\bcan\s+<[^>]*\bLISTEN-ONLY\b[^>]*>", output, re.I):
        raise CommunicationError(f"{channel} is in LISTEN-ONLY mode and cannot transmit")
    state_match = re.search(r"\bcan state\s+([A-Z-]+)\b", output, re.I)
    state = state_match.group(1).upper() if state_match else "UNKNOWN"
    if state in ("BUS-OFF", "ERROR-PASSIVE", "ERROR-WARNING"):
        raise CommunicationError(f"{channel} CAN controller state is {state}; inspect error counters")
    match = re.search(r"\bbitrate\s+(\d+)\b", output)
    if not match:
        raise CommunicationError(f"cannot determine {channel} bitrate")
    if int(match.group(1)) != expected_bitrate:
        raise CommunicationError(f"{channel} bitrate is {match.group(1)}, expected {expected_bitrate}")
    return output


class MotorCAN:
    def __init__(self, channel: str = "can0", motor_id: int = 1, verbose: bool = False,
                 bus=None):
        if not 1 <= motor_id <= 32:
            raise ValueError("motor ID must be 1–32")
        self.channel = channel
        self.motor_id = motor_id
        self.arbitration_id = 0x140 + motor_id
        self.verbose = verbose
        self.bus = bus

    def connect(self) -> None:
        if self.bus is not None:
            return
        if can is None:
            raise CommunicationError("python-can is missing; install requirements.txt")
        try:
            self.bus = can.Bus(interface="socketcan", channel=self.channel)
        except (can.CanError, OSError) as exc:
            raise CommunicationError(f"cannot open {self.channel}: {exc}") from exc

    def disconnect(self) -> None:
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def _log(self, direction: str, message) -> None:
        if self.verbose:
            stamp = getattr(message, "timestamp", None) if direction == "RX" else None
            stamp = stamp if stamp else time.time()
            print(f"{stamp:.6f} {direction} {message.arbitration_id:03X} "
                  f"{bytes(message.data).hex(' ').upper()}", flush=True)

    def send_frame(self, command: int, payload: bytes = b"\x00" * 7) -> None:
        if not 0 <= command <= 255 or len(payload) != 7:
            raise ValueError("command must be a byte and payload exactly seven bytes")
        if self.bus is None:
            raise CommunicationError("CAN bus is not connected")
        if can is None:
            raise CommunicationError("python-can is missing")
        message = can.Message(arbitration_id=self.arbitration_id,
                              data=bytes([command]) + payload, is_extended_id=False,
                              is_fd=False)
        try:
            self.bus.send(message, timeout=RESPONSE_TIMEOUT)
        except (can.CanError, OSError) as exc:
            raise CommunicationError(f"CAN transmit failed: {exc}") from exc
        self._log("TX", message)

    def receive_response(self, expected_command: int, timeout: float = RESPONSE_TIMEOUT) -> bytes:
        if self.bus is None:
            raise CommunicationError("CAN bus is not connected")
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            try:
                message = self.bus.recv(timeout=remaining)
            except ((OSError, can.CanError) if can is not None else OSError) as exc:
                raise CommunicationError(f"CAN receive failed: {exc}") from exc
            if message is None:
                break
            if (message.arbitration_id != self.arbitration_id or
                    message.is_extended_id or message.is_remote_frame or
                    message.is_error_frame or getattr(message, "is_fd", False) or
                    message.dlc != 8 or len(message.data) != 8 or
                    message.data[0] != expected_command):
                continue
            self._log("RX", message)
            return bytes(message.data)
        raise ResponseTimeout(f"no 0x{expected_command:02X} reply from 0x{self.arbitration_id:03X} "
                              f"within {timeout:g}s")

    def _exchange(self, command: int, payload: bytes = b"\x00" * 7) -> bytes:
        self.send_frame(command, payload)
        return self.receive_response(command)

    def read_status1(self) -> Status1:
        return decode_status1(self._exchange(0x9A))

    def read_status2(self) -> Feedback:
        return decode_feedback(self._exchange(0x9C))

    def enable(self) -> None:
        if self._exchange(0x88) != bytes([0x88]) + b"\x00" * 7:
            raise CommunicationError("enable acknowledgement payload mismatch")

    def stop(self) -> None:
        if self._exchange(0x81) != bytes([0x81]) + b"\x00" * 7:
            raise CommunicationError("stop acknowledgement payload mismatch")

    def disable(self) -> None:
        if self._exchange(0x80) != bytes([0x80]) + b"\x00" * 7:
            raise CommunicationError("disable acknowledgement payload mismatch")

    def set_speed_rpm(self, rpm: float) -> Feedback:
        return decode_feedback(self._exchange(0xA2, speed_payload(rpm)), 0xA2)

    def get_feedback(self) -> Feedback:
        return self.read_status2()
