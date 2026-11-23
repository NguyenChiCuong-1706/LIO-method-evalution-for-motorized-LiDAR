"""Convert fresh GUI CAN angle samples to ROS JointState without opening CAN.

The GUI worker remains the only serial transport owner.  This module consumes
its emitted telemetry on the GUI thread.  Timestamps approximate acquisition
at the midpoint of the request/response interval; the controller supplies no
independent timestamp, so LiDAR clock alignment must be checked in recorded bags.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math


JOINT_NAME = "lidar_joint"
MAX_SAMPLE_INTERVAL_S = 0.25  # Keep angle travel below pi at the GUI's 100 RPM limit.
MIN_SAMPLE_INTERVAL_S = 0.01
MAX_RESPONSE_RTT_S = 0.1
MAX_PLAUSIBLE_RPM = 120.0  # Data-quality gate, not a motor safety limit.


@dataclass(frozen=True)
class JointMeasurement:
    stamp_ns: int
    position_rad: float
    velocity_rad_s: float
    angle_degrees: float
    source: str = "fresh 0x92"


def _stamp_ns(utc_timestamp: str, response_rtt_s: float) -> int:
    timestamp = datetime.fromisoformat(utc_timestamp.replace("Z", "+00:00"))
    if timestamp.tzinfo is None:
        raise ValueError("angle timestamp must include a UTC offset")
    delta = timestamp.astimezone(timezone.utc) - datetime(1970, 1, 1, tzinfo=timezone.utc)
    received_ns = ((delta.days * 86_400 + delta.seconds) * 1_000_000_000 +
                   delta.microseconds * 1_000)
    return received_ns - round(response_rtt_s * 500_000_000)


class JointStateEstimator:
    """Derive theta1 and its signed velocity from fresh accumulated-angle frames."""

    def __init__(self, direction_sign: int = 1):
        if direction_sign not in (-1, 1):
            raise ValueError("direction_sign must be +1 or -1")
        self.direction_sign = direction_sign
        self.reference_offset_deg: float | None = None
        self.single_turn_reference_deg: float | None = None
        self._previous: tuple[float, float, int] | None = None
        self.last_rejection_reason: str | None = None

    def clear(self) -> None:
        self.reference_offset_deg = None
        self.single_turn_reference_deg = None
        self._previous = None
        self.last_rejection_reason = None

    def set_reference(self, single_turn_deg: float, multi_turn_deg: float) -> float:
        if not (math.isfinite(single_turn_deg) and math.isfinite(multi_turn_deg)):
            raise ValueError("HOME reference angles must be finite")
        # 0x92 may restart at a different turn count after a power cycle.
        # Correct only the within-turn difference while preserving its live
        # multi-turn progression for this connection.
        offset = ((single_turn_deg - multi_turn_deg + 180.0) % 360.0) - 180.0
        self.clear()
        self.reference_offset_deg = offset
        self.single_turn_reference_deg = single_turn_deg % 360.0
        return offset

    def consume(self, sample: dict) -> JointMeasurement | None:
        self.last_rejection_reason = None
        if self.reference_offset_deg is None or sample.get("sample_source") != "0x92":
            self.last_rejection_reason = "HOME reference or 0x92 source missing"
            return None
        try:
            raw = sample["raw_can_response"].split()
            if len(raw) != 8 or raw[0].upper() != "92":
                raise ValueError("not a complete 0x92 response")
            if sample["communication_status"] != "OK":
                raise ValueError("communication is not OK")
            angle_deg = float(sample["angle_degrees_provisional"])
            received_mono = float(sample["monotonic_timestamp"])
            rtt = float(sample["response_rtt_s"])
            if not all(math.isfinite(v) for v in (angle_deg, received_mono, rtt)):
                raise ValueError("nonfinite angle sample")
            if not 0 <= rtt <= MAX_RESPONSE_RTT_S:
                raise ValueError("angle response RTT is outside the usable range")
            midpoint_mono = received_mono - rtt / 2.0
            stamp_ns = _stamp_ns(sample["utc_timestamp"], rtt)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            self._previous = None
            self.last_rejection_reason = "malformed 0x92 sample or timestamp"
            return None

        position_deg = self.direction_sign * (angle_deg + self.reference_offset_deg)
        current = (midpoint_mono, position_deg, stamp_ns)
        previous = self._previous
        self._previous = current
        if previous is None:
            self.last_rejection_reason = "first fresh 0x92 sample; velocity needs a second sample"
            return None  # A second fresh frame is needed for signed velocity.
        interval_s = midpoint_mono - previous[0]
        if not MIN_SAMPLE_INTERVAL_S <= interval_s <= MAX_SAMPLE_INTERVAL_S:
            self.last_rejection_reason = (
                f"0x92 sample interval {interval_s:.3f}s outside "
                f"{MIN_SAMPLE_INTERVAL_S:g}–{MAX_SAMPLE_INTERVAL_S:g}s freshness window")
            return None
        if stamp_ns <= previous[2]:
            self.last_rejection_reason = "non-increasing 0x92 UTC timestamp"
            return None
        velocity_rad_s = math.radians((position_deg - previous[1]) / interval_s)
        if abs(velocity_rad_s) > MAX_PLAUSIBLE_RPM * 2 * math.pi / 60:
            self.last_rejection_reason = (
                f"angle-derived speed {math.degrees(velocity_rad_s) / 6:.1f} RPM "
                f"exceeds {MAX_PLAUSIBLE_RPM:g} RPM data-quality limit")
            return None
        return JointMeasurement(stamp_ns, math.radians(position_deg),
                                velocity_rad_s, position_deg)


class RclpyJointStatePublisher:
    """ROS publisher only; it does not create a motor transport or command motion."""

    def __init__(self, *, topic: str = "/joint_states", node_name: str = "mhf9025_gui_joint_state",
                 rclpy_module=None, joint_state_type=None):
        if rclpy_module is None or joint_state_type is None:
            try:
                import rclpy
                from sensor_msgs.msg import JointState
            except ImportError as exc:
                raise RuntimeError(
                    f"ROS 2 Python import failed ({exc}). Source /opt/ros/kilted/setup.bash "
                    "and install requirements.txt; if this virtual environment lacks PyYAML, "
                    "also add /usr/lib/python3/dist-packages to PYTHONPATH") from exc
            rclpy_module = rclpy if rclpy_module is None else rclpy_module
            joint_state_type = JointState if joint_state_type is None else joint_state_type
        self._rclpy = rclpy_module
        self._joint_state_type = joint_state_type
        self._owns_context = not self._rclpy.ok()
        if self._owns_context:
            self._rclpy.init(args=None)
        try:
            self.node = self._rclpy.create_node(node_name)
            self.publisher = self.node.create_publisher(joint_state_type, topic, 10)
        except BaseException:
            if self._owns_context:
                self._rclpy.shutdown()
            raise

    def publish(self, measurement: JointMeasurement) -> None:
        message = self._joint_state_type()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            measurement.stamp_ns, 1_000_000_000)
        message.name = [JOINT_NAME]
        message.position = [measurement.position_rad]
        message.velocity = [measurement.velocity_rad_s]
        message.effort = []
        self.publisher.publish(message)

    def running(self) -> bool:
        return self._rclpy.ok()

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
