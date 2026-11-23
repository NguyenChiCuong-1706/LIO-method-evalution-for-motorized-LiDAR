import io
import json
import pathlib
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import offline_speed_report


MOTOR_DIR = pathlib.Path(__file__).resolve().parents[1]
LEGACY_LOG = MOTOR_DIR / "experiment_logs/experimental_20261030T072632630471Z.log"


class OfflineSpeedReportTests(unittest.TestCase):
    @unittest.skipUnless(LEGACY_LOG.is_file(), "historical experiment log fixture is absent")
    def test_actual_log_identifies_abort_without_inventing_angle_speed(self):
        report = offline_speed_report.analyze_log(LEGACY_LOG.read_text())
        self.assertEqual(report["requested_rpm"], 5)
        self.assertEqual(report["requested_duration_s"], 1)
        self.assertEqual([sample["speed_raw_dps"] for sample in report["samples"]],
                         [5, 35])
        self.assertEqual(report["samples"][1]["classification"], "critical_failure")
        self.assertEqual(report["hard_speed_limit_rpm_unchanged"], 5.5)
        self.assertEqual(report["in_motion_0x92_raw_frames"], [])
        self.assertIsNone(report["angle_derived_speed_for_event"])
        self.assertTrue(report["stop_acknowledged"])
        self.assertTrue(report["disable_acknowledged"])
        self.assertTrue(report["zero_speed_confirmed_by_can_only"])
        self.assertFalse(report["physical_speed_verified"])
        self.assertEqual(report["protection_state"], "UNKNOWN")

    def test_structured_samples_take_precedence_over_legacy_line(self):
        structured = {"elapsed_since_speed_command_s": 0.203,
                      "reported_speed_raw_dps": 35, "encoder_raw": 59460,
                      "feedback_can_data": "9C 20 0B 00 23 00 44 E8",
                      "category": "critical_failure", "feedback_rtt_s": 0.003}
        report = offline_speed_report.analyze_log(
            "start_utc=x requested=+5 RPM duration=1s\n"
            "feedback: elapsed=0.20s speed_raw=35 encoder_raw=59460\n"
            "monitor_sample_json: " + json.dumps(structured))
        self.assertEqual(len(report["samples"]), 1)
        self.assertEqual(report["samples"][0]["elapsed_s"], 0.203)

    def test_video_uses_output_angle_and_explicit_uncertainties(self):
        with tempfile.TemporaryDirectory() as directory:
            video = pathlib.Path(directory) / "measurements.csv"
            video.write_text("frame,angle_deg\n0,0\n60,30\n120,60\n")
            report = offline_speed_report.analyze_video_csv(
                video, fps=60, a2_frame=0, angle_uncertainty_deg=1,
                sync_uncertainty_s=0.1,
                can_samples=[{"elapsed_s": 0.21, "speed_raw_dps": 35}])
        first = report["intervals"][0]
        self.assertEqual(first["average_output_shaft_rpm"], 5)
        self.assertLess(first["rpm_bounds_from_frame_and_angle_uncertainty"][0], 5)
        self.assertGreater(first["rpm_bounds_from_frame_and_angle_uncertainty"][1], 5)
        self.assertEqual(len(first["can_samples_possibly_overlapping"]), 1)
        self.assertAlmostEqual(first["can_vs_video_descriptive_comparisons"][0]
                               ["can_minus_video_average_rpm"], 5 / 6)
        self.assertIn("instantaneous", first["comparison_note"])

    def test_video_without_sync_does_not_claim_time_alignment(self):
        with tempfile.TemporaryDirectory() as directory:
            video = pathlib.Path(directory) / "measurements.csv"
            video.write_text("frame,angle_deg\n10,-10\n70,-40\n")
            report = offline_speed_report.analyze_video_csv(
                video, fps=60, a2_frame=None, angle_uncertainty_deg=1,
                sync_uncertainty_s=None, can_samples=[])
        self.assertEqual(report["intervals"][0]["average_output_shaft_rpm"], -5)
        self.assertIsNone(report["intervals"][0]["can_samples_possibly_overlapping"])
        self.assertIsNone(report["intervals"][0]["can_vs_video_descriptive_comparisons"])

    def test_video_rejects_duplicate_frames_and_unbounded_sync(self):
        with tempfile.TemporaryDirectory() as directory:
            video = pathlib.Path(directory) / "measurements.csv"
            video.write_text("frame,angle_deg\n10,0\n10,30\n")
            with self.assertRaises(ValueError):
                offline_speed_report.analyze_video_csv(
                    video, fps=60, a2_frame=None, angle_uncertainty_deg=1,
                    sync_uncertainty_s=None, can_samples=[])
            with self.assertRaises(ValueError):
                offline_speed_report.analyze_video_csv(
                    video, fps=60, a2_frame=None, angle_uncertainty_deg=1,
                    sync_uncertainty_s=0.1, can_samples=[])

    @unittest.skipUnless(LEGACY_LOG.is_file(), "historical experiment log fixture is absent")
    def test_cli_analyzes_file_only(self):
        with redirect_stdout(io.StringIO()) as output:
            result = offline_speed_report.main(["--log", str(LEGACY_LOG)])
        self.assertEqual(result, 0)
        self.assertIn('"physical_speed_verified": false', output.getvalue())
