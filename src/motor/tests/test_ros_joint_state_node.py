"""Standalone ROS publisher tests: all CAN traffic uses fake or pseudo devices."""

from contextlib import redirect_stdout
from datetime import datetime, timedelta, timezone
import io
import math
import os
import pathlib
import pty
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from motor_can import CommunicationError, ResponseTimeout
from ros_joint_state_node import ReadOnlyJointStateSource, run
from waveshare_motor import WaveshareMotor


def frame92(degrees):
    raw = int(round(degrees * 100))
    return b"\x92" + raw.to_bytes(7, "little", signed=True)


def frame94(degrees):
    return b"\x94\x00\x00\x00" + int(round(degrees * 100)).to_bytes(4, "little")


class FakeMotor:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.commands = []
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True

    def exchange(self, command):
        self.commands.append(command)
        reply = next(self.replies)
        if isinstance(reply, BaseException):
            raise reply
        return reply


class FakePublisher:
    def __init__(self, *, node_name=""):
        self.node_name = node_name
        self.measurements = []
        self.closed = False
        self.loops = 0

    def publish(self, measurement):
        self.measurements.append(measurement)

    def running(self):
        self.loops += 1
        return self.loops <= 2

    def close(self):
        self.closed = True


class ReadOnlyNodeTests(unittest.TestCase):
    def test_fresh_92_produces_home_relative_position_and_signed_velocity(self):
        motor = FakeMotor([frame94(359.5), frame92(359.5),
                           frame92(359.5), frame92(358.5)])
        publisher = FakePublisher()
        ticks = iter([1.0, 1.004, 1.1, 1.104])
        wall = iter([datetime(2026, 11, 2, tzinfo=timezone.utc),
                     datetime(2026, 11, 2, tzinfo=timezone.utc) +
                     timedelta(milliseconds=100)])
        source = ReadOnlyJointStateSource(
            motor, publisher, monotonic_clock=lambda: next(ticks),
            utc_clock=lambda: next(wall))
        self.assertEqual(source.initialize(), 0)
        self.assertFalse(source.poll_once())
        self.assertTrue(source.poll_once())
        self.assertEqual(motor.commands, [0x94, 0x92, 0x92, 0x92])
        self.assertAlmostEqual(publisher.measurements[0].position_rad,
                               math.radians(358.5))
        self.assertAlmostEqual(publisher.measurements[0].velocity_rad_s,
                               math.radians(-10))
        self.assertEqual(publisher.measurements[0].stamp_ns % 1_000_000_000,
                         98_000_000)

    def test_timeout_or_malformed_frame_never_triggers_retry_or_motion(self):
        for failure in (ResponseTimeout("0x92 timeout"), b"\x92\x00"):
            with self.subTest(failure=failure):
                motor = FakeMotor([frame94(0), frame92(0), failure])
                publisher = FakePublisher()
                source = ReadOnlyJointStateSource(motor, publisher)
                source.initialize()
                with self.assertRaises(CommunicationError):
                    source.poll_once()
                self.assertEqual(motor.commands, [0x94, 0x92, 0x92])
                self.assertEqual(publisher.measurements, [])

    def test_unrecognized_94_layout_blocks_publication(self):
        motor = FakeMotor([frame94(400), frame92(0)])
        source = ReadOnlyJointStateSource(motor, FakePublisher())
        with self.assertRaisesRegex(ValueError, "does not match"):
            source.initialize()
        self.assertEqual(motor.commands, [0x94])

    def test_run_closes_transport_and_ros_publisher_on_failure(self):
        motor = FakeMotor([frame94(0), frame92(0), ResponseTimeout("no reply")])
        publisher = FakePublisher()
        with self.assertRaises(ResponseTimeout):
            run(motor_factory=lambda *_args: motor,
                publisher_factory=lambda **_kwargs: publisher,
                sleep=lambda _seconds: None)
        self.assertTrue(motor.closed)
        self.assertTrue(publisher.closed)
        self.assertEqual(motor.commands, [0x94, 0x92, 0x92])

    def test_invalid_interface_settings_rejected_before_open(self):
        for settings in ({"motor_id": 2}, {"can_bitrate": 1_000_000},
                         {"period": 0.3}, {"direction_sign": 0}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                run(**settings, motor_factory=lambda *_args: self.fail("opened motor"),
                    publisher_factory=lambda **_kwargs: self.fail("opened ROS"))

    def test_shared_waveshare_transport_excludes_second_process_owner(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        first = WaveshareMotor(path, 1, timeout=0.2)
        second = WaveshareMotor(path, 1, timeout=0.2)
        try:
            with redirect_stdout(io.StringIO()), first:
                with self.assertRaisesRegex(CommunicationError, "already owned"):
                    second.__enter__()
                self.assertIsNone(second.fd)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
