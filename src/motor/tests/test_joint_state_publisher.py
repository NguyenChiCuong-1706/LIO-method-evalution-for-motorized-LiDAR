import math
import pathlib
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from joint_state_publisher import (JointMeasurement, JointStateEstimator,
                                   RclpyJointStatePublisher)


def angle_sample(angle, mono, stamp="2026-11-02T10:00:00+00:00", *, rtt=0.004):
    raw = int(round(angle * 100)) & ((1 << 56) - 1)
    return {
        "sample_source": "0x92",
        "raw_can_response": "92 " + raw.to_bytes(7, "little").hex(" ").upper(),
        "communication_status": "OK",
        "angle_degrees_provisional": angle,
        "monotonic_timestamp": mono,
        "utc_timestamp": stamp,
        "response_rtt_s": rtt,
    }


class JointStateEstimatorTests(unittest.TestCase):
    def test_home_reference_aligns_continuous_angle_after_power_cycle(self):
        estimator = JointStateEstimator(1)
        self.assertEqual(estimator.set_reference(270.0, 0.0), -90.0)
        self.assertIsNone(estimator.consume(angle_sample(0, 1.0)))
        result = estimator.consume(angle_sample(
            1, 1.1, "2026-11-02T10:00:00.100000+00:00"))
        self.assertAlmostEqual(result.position_rad, math.radians(-89))
        self.assertAlmostEqual(result.velocity_rad_s, math.radians(10))
        self.assertEqual(result.stamp_ns % 1_000_000_000, 98_000_000)

    def test_signed_velocity_uses_fresh_0x92_changes_not_cached_0x9c(self):
        estimator = JointStateEstimator(1)
        self.assertEqual(estimator.set_reference(270, -90), 0)
        self.assertIsNone(estimator.consume(angle_sample(-90, 1.0)))
        cached = angle_sample(-90, 1.05)
        cached["sample_source"] = "0x9C"
        self.assertIsNone(estimator.consume(cached))
        result = estimator.consume(angle_sample(
            -93, 1.1, "2026-11-02T10:00:00.100000+00:00"))
        self.assertAlmostEqual(result.position_rad, math.radians(-93))
        self.assertAlmostEqual(result.velocity_rad_s, math.radians(-30))

    def test_alternate_joint_sign_inverts_position_and_velocity(self):
        estimator = JointStateEstimator(-1)
        estimator.set_reference(270, -90)
        self.assertIsNone(estimator.consume(angle_sample(-90, 1.0)))
        result = estimator.consume(angle_sample(
            -93, 1.1, "2026-11-02T10:00:00.100000+00:00"))
        self.assertAlmostEqual(result.position_rad, math.radians(93))
        self.assertAlmostEqual(result.velocity_rad_s, math.radians(30))

    def test_duplicate_gap_bad_response_and_nonfinite_data_are_not_published(self):
        estimator = JointStateEstimator()
        estimator.set_reference(0, 0)
        self.assertIsNone(estimator.consume(angle_sample(0, 1.0)))
        self.assertIsNone(estimator.consume(angle_sample(1, 1.0)))
        self.assertIsNone(estimator.consume(angle_sample(2, 2.0)))
        malformed = angle_sample(3, 2.1)
        malformed["raw_can_response"] = "92 00"
        self.assertIsNone(estimator.consume(malformed))
        nonfinite = angle_sample(3, 2.2)
        nonfinite["angle_degrees_provisional"] = float("nan")
        self.assertIsNone(estimator.consume(nonfinite))
        self.assertIsNone(estimator.consume(angle_sample(4, 2.3)))
        result = estimator.consume(angle_sample(
            5, 2.4, "2026-11-02T10:00:00.100000+00:00"))
        self.assertIsNotNone(result)

    def test_long_polling_gap_reports_freshness_failure(self):
        estimator = JointStateEstimator()
        estimator.set_reference(0, 0)
        self.assertIsNone(estimator.consume(angle_sample(0, 1.0)))
        self.assertIsNone(estimator.consume(angle_sample(
            108, 1.6, "2026-11-02T10:00:00.600000+00:00")))
        self.assertIn("0.600s", estimator.last_rejection_reason)
        self.assertIn("freshness window", estimator.last_rejection_reason)

    def test_large_angle_jump_and_backward_wall_clock_are_rejected(self):
        estimator = JointStateEstimator()
        estimator.set_reference(0, 0)
        self.assertIsNone(estimator.consume(angle_sample(0, 1.0)))
        self.assertIsNone(estimator.consume(angle_sample(
            100, 1.1, "2026-11-02T10:00:00.100000+00:00")))
        self.assertIsNone(estimator.consume(angle_sample(
            101, 1.2, "2026-11-02T10:00:00.050000+00:00")))

    def test_reference_rejects_invalid_sign_and_nonfinite_angles(self):
        with self.assertRaises(ValueError):
            JointStateEstimator(0)
        estimator = JointStateEstimator()
        with self.assertRaises(ValueError):
            estimator.set_reference(float("inf"), 0)

    def test_ros_adapter_message_fields_and_lifecycle_with_fake_ros(self):
        messages = []

        class FakeJointState:
            def __init__(self):
                self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
                self.name = self.position = self.velocity = self.effort = None

        class FakeNode:
            def create_publisher(self, _message_type, topic, _depth):
                self.topic = topic
                return SimpleNamespace(publish=messages.append)

            def destroy_node(self):
                self.destroyed = True

        node = FakeNode()
        ros = SimpleNamespace(
            ok=lambda: False, init=lambda args=None: None,
            create_node=lambda _name: node, shutdown=lambda: None)
        publisher = RclpyJointStatePublisher(
            rclpy_module=ros, joint_state_type=FakeJointState)
        publisher.publish(JointMeasurement(1_234_567_890, 1.2, -0.3, 68.75))
        self.assertEqual(node.topic, "/joint_states")
        self.assertEqual((messages[0].header.stamp.sec,
                          messages[0].header.stamp.nanosec), (1, 234_567_890))
        self.assertEqual(messages[0].name, ["lidar_joint"])
        self.assertEqual(messages[0].position, [1.2])
        self.assertEqual(messages[0].velocity, [-0.3])
        publisher.close()
        self.assertTrue(node.destroyed)


if __name__ == "__main__":
    unittest.main()
