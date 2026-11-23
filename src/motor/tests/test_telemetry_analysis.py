import csv
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from telemetry_analysis import DisplaySpeedEstimator, analyze_csv


class TelemetryAnalysisTests(unittest.TestCase):
    def test_filter_and_angle_estimator_are_timestamp_aware(self):
        estimator = DisplaySpeedEstimator(filter_tau_s=0.25)
        self.assertEqual(estimator.update_speed(80, 1.0), 80)
        filtered = estimator.update_speed(100, 1.25)
        self.assertGreater(filtered, 80)
        self.assertLess(filtered, 100)
        self.assertIsNone(estimator.update_angle(0, 2.0, 0.02))
        angle_rpm = estimator.update_angle(9, 3.0, 0.02)
        self.assertAlmostEqual(angle_rpm, 1.5)
        self.assertAlmostEqual(estimator.angle_interval_s, 1.0)
        self.assertAlmostEqual(estimator.angle_time_uncertainty_s, 0.02)

    def test_offline_analysis_ignores_cached_rows(self):
        fields = ["monotonic_timestamp", "elapsed_session_s", "requested_rpm",
                  "actual_rpm_provisional", "angle_degrees_provisional",
                  "motor_state", "raw_can_response", "event_stop_reason"]
        rows = [
            (0.0, 0.0, 90, 80, 0, "RUNNING", "9C 00", ""),
            (0.1, 0.1, 90, 80, 0, "RUNNING", "9A 00", ""),  # cached, ignored
            (0.2, 0.2, 90, 90, 0, "RUNNING", "9C 00", ""),
            (0.3, 0.3, 90, 90, 27, "RUNNING", "92 00", ""),
            (0.4, 0.4, 90, 90, 27, "RUNNING", "9A 00", ""),  # cached, ignored
            (0.5, 0.5, 90, 90, 81, "RUNNING", "92 00", ""),
            (0.6, 0.6, 90, 90, 81, "STOP_UNCONFIRMED", "", "duration_elapsed"),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "sample.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle); writer.writerow(fields); writer.writerows(rows)
            report = analyze_csv(path, startup_exclusion_s=0)
        self.assertEqual(report["fresh_0x9c_rows"], 2)
        self.assertEqual(report["fresh_0x92_rows"], 2)
        self.assertEqual(report["raw_0x9c"]["samples"], 2)
        self.assertEqual(report["angle_derived_provisional"]["samples"], 1)
        self.assertAlmostEqual(report["angle_derived_provisional"]["mean_rpm"], 45)
        self.assertEqual(report["final_state"], "STOP_UNCONFIRMED")


if __name__ == "__main__":
    unittest.main()
