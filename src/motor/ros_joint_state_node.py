"""Read-only ROS 2 /joint_states node for one MHF9025-V3 Waveshare adapter.

Run this instead of the GUI; both programs use the same serial device. Only
0x94 and 0x92 read requests are sent. No motor control command is issued.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import sys
import time

from absolute_encoder_diagnostic import decode_multi_turn_92, decode_single_turn_94
from joint_state_publisher import JointStateEstimator, RclpyJointStatePublisher
from motor_can import MotorError
from waveshare_motor import WaveshareMotor


class ReadOnlyJointStateSource:
    """Use a caller-owned transport and the same estimator as the GUI."""

    def __init__(self, motor, publisher, *, direction_sign=1,
                 monotonic_clock=time.monotonic,
                 utc_clock=lambda: datetime.now(timezone.utc)):
        self.motor = motor
        self.publisher = publisher
        self.estimator = JointStateEstimator(direction_sign)
        self.monotonic_clock = monotonic_clock
        self.utc_clock = utc_clock
        self.last_measurement = None
        self.last_angle_degrees = None
        self.last_sample_monotonic = None

    def initialize(self) -> float:
        decoded_single = decode_single_turn_94(self.motor.exchange(0x94)).values
        if decoded_single["selected_layout"] != "MHF9025-V3 observed uint32 bytes 4-7":
            raise ValueError("0x94 response does not match the observed MHF9025-V3 layout")
        single = decoded_single["single_turn_angle_degrees_provisional"]
        multi = decode_multi_turn_92(self.motor.exchange(0x92)).values[
            "multi_turn_angle_degrees_provisional"]
        return self.estimator.set_reference(single, multi)

    def poll_once(self) -> bool:
        started = self.monotonic_clock()
        raw = self.motor.exchange(0x92)
        received = self.monotonic_clock()
        self.last_sample_monotonic = received
        stamp = self.utc_clock()
        angle = decode_multi_turn_92(raw).values[
            "multi_turn_angle_degrees_provisional"]
        self.last_angle_degrees = angle
        sample = {
            "sample_source": "0x92",
            "raw_can_response": raw.hex(" ").upper(),
            "communication_status": "OK",
            "angle_degrees_provisional": angle,
            "monotonic_timestamp": received,
            "utc_timestamp": stamp.isoformat(),
            "response_rtt_s": received - started,
        }
        measurement = self.estimator.consume(sample)
        self.last_measurement = measurement
        if measurement is None:
            return False
        self.publisher.publish(measurement)
        return True


def run(*, device="/dev/ttyUSB0", can_bitrate=500_000, motor_id=1,
        direction_sign=1, period=0.125, timeout=0.2,
        motor_factory=WaveshareMotor,
        publisher_factory=RclpyJointStatePublisher,
        monotonic_clock=time.monotonic, sleep=time.sleep) -> int:
    if can_bitrate != 500_000 or motor_id != 1:
        raise ValueError("this joint is CAN ID 0x141 at 500000 bit/s")
    if direction_sign not in (-1, 1):
        raise ValueError("direction sign must be +1 or -1")
    if not 0.05 <= period <= 0.2 or not 0.05 <= timeout <= 0.5:
        raise ValueError("period must be 0.05..0.2 s and timeout 0.05..0.5 s")

    publisher = publisher_factory(node_name="mhf9025_joint_state")
    try:
        with motor_factory(device, motor_id, can_bitrate, 2_000_000, timeout) as motor:
            source = ReadOnlyJointStateSource(motor, publisher,
                                              direction_sign=direction_sign,
                                              monotonic_clock=monotonic_clock)
            offset = source.initialize()
            print(f"Read-only /joint_states active: 0x94 HOME alignment "
                  f"offset={offset:+.2f} deg; direction_sign={direction_sign:+d}",
                  flush=True)
            next_poll = monotonic_clock()
            while publisher.running():
                source.poll_once()
                next_poll += period
                sleep(max(0.0, next_poll - monotonic_clock()))
    except KeyboardInterrupt:
        return 0
    finally:
        publisher.close()
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--can-bitrate", type=int, default=500_000)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--direction-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--period", type=float, default=0.125, help="0x92 poll interval in seconds")
    parser.add_argument("--timeout", type=float, default=0.2, help="single CAN exchange timeout")
    cli_args = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in cli_args:
        # ros2 launch appends node remapping arguments. Keep them in sys.argv
        # for rclpy.init(), while parsing only this executable's options here.
        from rclpy.utilities import remove_ros_args
        cli_args = remove_ros_args(["joint_state_node", *cli_args])[1:]
    args = parser.parse_args(cli_args)
    try:
        return run(**vars(args))
    except (MotorError, OSError, RuntimeError, ValueError) as exc:
        print(f"Joint-state node stopped: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
