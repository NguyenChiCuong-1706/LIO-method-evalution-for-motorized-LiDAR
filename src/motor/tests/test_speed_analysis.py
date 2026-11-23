import pathlib
import sys
import unittest
from unittest.mock import Mock, patch
from contextlib import redirect_stdout
import io

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import speed_analysis
import waveshare_motor
from motor_can import CommunicationError, ResponseTimeout, decode_feedback


class SpeedAnalysisTests(unittest.TestCase):
    def test_command_feedback_resolution_and_low_speed_quantization(self):
        self.assertEqual(waveshare_motor.speed_payload(5).hex(" "),
                         "00 00 00 b8 0b 00 00")
        for raw, expected in ((29, 29 / 6), (30, 5), (31, 31 / 6), (35, 35 / 6)):
            frame = bytes.fromhex(f"9C 20 00 00 {raw:02X} 00 00 00")
            self.assertAlmostEqual(decode_feedback(frame).speed_rpm, expected)
        with self.assertRaises(CommunicationError):
            decode_feedback(bytes.fromhex("9C 20 00 00 1E 00 00"))

    def test_tracking_categories_keep_existing_hard_stop(self):
        for raw in (29, 30, 31):
            self.assertEqual(speed_analysis.classify_speed(raw, 5, 0.1)["category"],
                             "normal_tracking")
        first = speed_analysis.classify_speed(32, 5, 0.1)
        self.assertEqual(first["category"], "startup_transient_candidate")
        second = speed_analysis.classify_speed(32, 5, 0.3,
                                               first["consecutive_tracking_deviations"])
        self.assertEqual(second["category"], "tracking_deviation")
        self.assertEqual(speed_analysis.classify_speed(32, 5, 0.8)["category"],
                         "tracking_deviation")
        critical = speed_analysis.classify_speed(35, 5, 0.21)
        self.assertEqual(critical["category"], "critical_failure")
        self.assertEqual(critical["reason"], "hard_overspeed")
        self.assertEqual(critical["hard_speed_limit_rpm"], 5.5)
        self.assertAlmostEqual(critical["absolute_tracking_error_rpm"], 5 / 6)
        self.assertAlmostEqual(critical["relative_tracking_error_fraction"], 1 / 6)
        self.assertEqual(speed_analysis.classify_speed(-3, 5, 0.1)["reason"],
                         "unexpected_reverse_speed")

    def test_angle_comparison_signed_and_timing_aware(self):
        sample = speed_analysis.TimedRawSample
        for before_raw, after_raw, speed_raw in ((-4000, -1000, 30),
                                                  (1000, -2000, -30)):
            comparison = speed_analysis.compare_speed_with_angle(
                sample(before_raw, 0, 0.01),
                sample(speed_raw, 0.5, 0.51),
                sample(after_raw, 1, 1.01))
            self.assertEqual(comparison["quality"], "timing_acceptable_provisional")
            self.assertAlmostEqual(comparison["angle_interval_s"], 1)
            self.assertAlmostEqual(comparison["speed_from_angle_rpm_provisional"],
                                   speed_raw / 6)
            self.assertAlmostEqual(comparison["discrepancy_rpm"], 0)
            self.assertAlmostEqual(comparison["angle_interval_uncertainty_s"], 0.01)
            self.assertLessEqual(comparison["speed_from_angle_timing_bounds_rpm"][0],
                                 speed_raw / 6)
            self.assertGreaterEqual(comparison["speed_from_angle_timing_bounds_rpm"][1],
                                    speed_raw / 6)
            self.assertEqual(comparison["consistency"],
                             "within_one_feedback_count_provisional")

    def test_angle_comparison_rejects_bad_or_delayed_timestamps(self):
        sample = speed_analysis.TimedRawSample
        with self.assertRaisesRegex(ValueError, "monotonic"):
            speed_analysis.compare_speed_with_angle(sample(0, 1, 0),
                                                    sample(0, 1, 1.01),
                                                    sample(0, 2, 2.01))
        delayed = speed_analysis.compare_speed_with_angle(
            sample(0, 0, 0.01), sample(30, 0.2, 0.5), sample(3000, 1, 1.01))
        self.assertEqual(delayed["quality"], "unusable_timing")
        self.assertIsNone(delayed["speed_from_angle_rpm_provisional"])
        unbracketed = speed_analysis.compare_speed_with_angle(
            sample(0, 0, 0.01), sample(30, 2, 2.01), sample(3000, 1, 1.01))
        self.assertEqual(unbracketed["quality"], "unusable_timing")

    def test_encoder_wrap_is_raw_16bit_only(self):
        self.assertEqual(waveshare_motor._encoder_delta(65534, 2), 4)
        self.assertEqual(waveshare_motor._encoder_delta(2, 65534), -4)

    def test_read_only_angle_mode_never_sends_control(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [bytes.fromhex("92 00 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 20 00 00 1E 00 00 00"),
                                      bytes.fromhex("92 B8 0B 00 00 00 00 00")]
        ticks = iter((0, 0.003, 0.2, 0.203, 0.4, 0.403))
        with patch.object(waveshare_motor.time, "monotonic", side_effect=lambda: next(ticks)), \
             patch.object(waveshare_motor.time, "sleep"), redirect_stdout(io.StringIO()):
            result = waveshare_motor.run_speed_angle_diagnostic(motor)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x92, 0x9C, 0x92])
        self.assertEqual(result["quality"], "timing_acceptable_provisional")
        self.assertAlmostEqual(result["speed_from_angle_rpm_provisional"], 12.5)
        self.assertEqual(result["consistency"],
                         "difference_above_one_feedback_count_unresolved")
        motor.stop_and_disable.assert_not_called()

    def test_read_only_angle_mode_does_not_retry_timeout(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = ResponseTimeout("no 0x92 reply")
        with self.assertRaises(ResponseTimeout):
            waveshare_motor.run_speed_angle_diagnostic(motor)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list], [0x92])
