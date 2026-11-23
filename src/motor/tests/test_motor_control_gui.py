import os
import pathlib
import sys
import time
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:
    from PySide6 import QtCore, QtWidgets
    import motor_control_gui
except (ImportError, SystemExit):
    QtCore = QtWidgets = motor_control_gui = None


@unittest.skipIf(QtCore is None, "PySide6/pyqtgraph not installed")
class MotorControlGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def wait_until(self, predicate, timeout_s=2):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return True
            QtCore.QThread.msleep(10)
        return False

    def test_x11_runtime_check_reports_exact_ubuntu_package(self):
        with patch.dict(os.environ, {"DISPLAY": ":1", "QT_QPA_PLATFORM": ""}, clear=False), \
             patch.object(motor_control_gui.ctypes.util, "find_library", return_value=None), \
             self.assertRaisesRegex(SystemExit, "sudo apt install -y libxcb-cursor0"):
            motor_control_gui.require_qt_platform_runtime()

    def test_gui_starts_disconnected_without_adapter(self):
        window = motor_control_gui.MotorControlWindow(simulate=True)
        try:
            self.assertEqual(motor_control_gui.GUI_CONFIRMATION, "OK")
            self.assertFalse(window.connected)
            self.assertFalse(window.start_button.isEnabled())
            self.assertFalse(window.go_home_button.isEnabled())
            self.assertEqual(window.connection_label.text(), "DISCONNECTED")
            self.assertEqual(window.device.text(), "SIMULATED")
            self.assertEqual([window.bitrate.itemData(i) for i in range(window.bitrate.count())],
                             [500_000, 1_000_000])
            self.assertEqual(window.bitrate.currentData(), 1_000_000)
            self.assertEqual(window.speed.maximum(), 100)
            self.assertEqual(window.duration.maximum(), 20)
            self.assertEqual(window.limit_high.value(), 100)
            self.assertFalse(window.pid_read_button.isEnabled())
            self.assertFalse(hasattr(window, "pid_apply_button"))
            self.assertTrue(window.show_filtered.isChecked())
            self.assertEqual(window.home_direction_sign.currentData(), 1)
            self.assertFalse(window.home_set_button.isEnabled())
            self.assertFalse(window.home_go_button.isEnabled())
            self.assertIn("ROM WRITE", window.home_set_button.text())
            self.assertFalse(window.home_read_button.isEnabled())
            self.assertFalse(hasattr(window, "home_verify_button"))
            self.assertFalse(hasattr(window.worker, "request_verify_home"))
            self.assertIn("zero angle alone", window.home_status.text())
            self.assertTrue(window.auto_home.isChecked())
            self.assertEqual(window.auto_home_speed.value(), 5.0)
        finally:
            window.worker.request_shutdown()
            self.assertTrue(window.thread.wait(3000))
            window.deleteLater()

    def test_torque_display_uses_only_fresh_9c_and_clears_on_disconnect(self):
        window = motor_control_gui.MotorControlWindow(simulate=True)
        try:
            window._connection(True, "0x141 (SIMULATED)")
            sample = {"sample_source": "0x9C", "torque_current_raw": -7,
                      "monotonic_timestamp": time.monotonic() - 0.02}
            window._torque_feedback(sample)
            self.assertEqual(window.values["torque_current"][0].text(), "-7 raw counts")
            window._torque_feedback({**sample, "sample_source": "0x92",
                                     "torque_current_raw": 999})
            self.assertEqual(window.values["torque_current"][0].text(), "-7 raw counts")
            window._flush_telemetry()
            self.assertIn(" s", window.values["torque_age"][0].text())
            window._last_torque_feedback_mono = time.monotonic() - 1.0
            window._flush_telemetry()
            self.assertEqual(window.values["torque_current"][0].text(), "—")
            window._connection(False, "DISCONNECTED")
            self.assertEqual(window.values["torque_current"][0].text(), "—")
            self.assertEqual(window.values["torque_age"][0].text(), "—")
        finally:
            window.worker.request_shutdown()
            self.assertTrue(window.thread.wait(3000))
            window.deleteLater()

    def test_ros_joint_publisher_reuses_gui_worker_and_every_fresh_angle_sample(self):
        class FakeRosPublisher:
            def __init__(self):
                self.measurements = []
                self.closed = False

            def publish(self, measurement):
                self.measurements.append(measurement)

            def close(self):
                self.closed = True

        fake = FakeRosPublisher()
        window = motor_control_gui.MotorControlWindow(
            simulate=True, publish_joint_states=True,
            ros_publisher_factory=lambda: fake)
        samples = []
        window.worker.telemetry.connect(samples.append)
        try:
            window._connect()
            self.assertTrue(self.wait_until(lambda: window.connected))
            self.assertTrue(self.wait_until(lambda:
                window.joint_estimator.reference_offset_deg is not None))
            self.assertFalse(window.home_direction_sign.isEnabled())
            self.assertTrue(self.wait_until(lambda: len(fake.measurements) >= 2, 3))
            self.assertTrue(all(item.source == "fresh 0x92" for item in fake.measurements))
            self.assertTrue(all(item.stamp_ns > 0 for item in fake.measurements))
            self.assertGreater(len([item for item in samples
                                    if item["sample_source"] == "0x92"]), 2)
            self.assertTrue(window.close())
            self.assertTrue(fake.closed)
        finally:
            if window.thread.isRunning():
                window.worker.request_shutdown()
                self.assertTrue(window.thread.wait(3000))
            window.deleteLater()

    def test_ros_publication_pauses_after_home_write_until_reconnect(self):
        class FakeRosPublisher:
            def publish(self, _measurement):
                pass

            def close(self):
                pass

        window = motor_control_gui.MotorControlWindow(
            simulate=True, publish_joint_states=True,
            ros_publisher_factory=FakeRosPublisher)
        try:
            window._connection(True, "0x141 (SIMULATED)")
            fresh = {"angles": {"single_turn_degrees": 0.0,
                                "multi_turn_degrees": 0.0}, "state": "NOT_CALIBRATED"}
            window._home_received(fresh)
            self.assertEqual(window.joint_estimator.reference_offset_deg, 0.0)
            window._home_received({
                **fresh, "state": "WRITE_RESULT_UNKNOWN",
                "write_attempted_this_connection": True})
            self.assertIsNone(window.joint_estimator.reference_offset_deg)
            window._home_received(fresh)
            self.assertIsNone(window.joint_estimator.reference_offset_deg)
            window._connection(False, "DISCONNECTED")
            window._connection(True, "0x141 (SIMULATED)")
            window._home_received(fresh)
            self.assertEqual(window.joint_estimator.reference_offset_deg, 0.0)
        finally:
            window.worker.request_shutdown()
            self.assertTrue(window.thread.wait(3000))
            window.deleteLater()

    def test_go_home_button_uses_approach_settings_and_not_target_run(self):
        window = motor_control_gui.MotorControlWindow(simulate=True)
        try:
            window._connection(True, "0x141 (SIMULATED)")
            window.auto_home.setChecked(False)
            self.assertTrue(window.auto_home_speed.isEnabled())
            window.home_tolerance.setValue(1.5)
            window.auto_home_speed.setValue(2.0)
            self.assertTrue(window.go_home_button.isEnabled())
            with patch.object(window.worker, "request_home_motion") as request_home, \
                 patch.object(window.worker, "start_motion") as request_start:
                window.go_home_button.click()
            request_home.assert_called_once_with(1.5, 2.0, False)
            request_start.assert_not_called()
            self.assertTrue(window.running)
            self.assertFalse(window.go_home_button.isEnabled())
            self.assertTrue(window.stop_button.isEnabled())
            window._session_finished(True, "test complete")
            self.assertTrue(window.home_go_button.isEnabled())
            with patch.object(window.worker, "request_home_motion") as request_home_panel, \
                 patch.object(window.worker, "request_set_home") as request_write:
                window.home_go_button.click()
            request_home_panel.assert_called_once_with(1.5, 2.0, False)
            request_write.assert_not_called()
        finally:
            window.worker.request_shutdown()
            self.assertTrue(window.thread.wait(3000))
            window.deleteLater()

    def test_experimental_hold_requires_opt_in_and_is_forwarded_to_worker(self):
        window = motor_control_gui.MotorControlWindow(simulate=True)
        try:
            self.assertFalse(window.position_hold.isChecked())
            window._connection(True, "0x141 (SIMULATED)")
            window.position_hold.setChecked(True)
            with patch.object(window.worker, "start_motion") as start:
                window.start_button.click()
            self.assertTrue(start.call_args.args[-1])
        finally:
            window.worker.request_shutdown()
            self.assertTrue(window.thread.wait(3000))
            window.deleteLater()

    def test_long_stop_status_and_csv_path_do_not_resize_window(self):
        window = motor_control_gui.MotorControlWindow(simulate=True)
        try:
            window.show()
            self.app.processEvents()
            original_size = window.size()
            original_minimum_width = window.minimumSizeHint().width()
            status = (
                "STOP and DISABLE acknowledged; three consecutive controller "
                "speed/encoder samples settled within a bounded ten-sample "
                "window; physical stop remains unverified")
            window._set_value("stop_confirmation", status)
            window._csv_ready(
                "/home/cuong/lm_calibr/src/motor/motor_logs/"
                "motor_session_20261102T031335353466Z.csv")
            self.app.processEvents()
            self.assertEqual(window.size(), original_size)
            self.assertEqual(window.minimumSizeHint().width(), original_minimum_width)
            self.assertEqual(window.values["stop_confirmation"][0].toolTip(), status)
            self.assertEqual(window.csv_label.toolTip(), window.last_csv_path)
        finally:
            window.worker.request_shutdown()
            self.assertTrue(window.thread.wait(3000))
            window.close()
            window.deleteLater()

    def test_simulated_worker_connects_polls_and_emits_qt_signals(self):
        thread = QtCore.QThread()
        worker = motor_control_gui.MotorControlWorker(simulate=True)
        worker.moveToThread(thread)
        states, samples, connections, pid_results = [], [], [], []
        worker.state_changed.connect(states.append)
        worker.telemetry.connect(samples.append)
        worker.connection_changed.connect(lambda ok, label: connections.append((ok, label)))
        worker.pid_received.connect(pid_results.append)
        worker.finished.connect(thread.quit)
        thread.started.connect(worker.run)
        thread.start()
        try:
            worker.connect_motor("SIMULATED", 500_000, 1)
            self.assertTrue(self.wait_until(lambda: bool(connections)))
            self.assertTrue(connections[-1][0])
            self.assertTrue(self.wait_until(lambda: len(samples) >= 3))
            self.assertIn("CONNECTED", states)
            self.assertTrue(any(sample["protection_state"] == "UNKNOWN" for sample in samples))
            worker.request_pid_read()
            self.assertTrue(self.wait_until(lambda: len(pid_results) == 1))
            self.assertEqual(pid_results[-1]["speed_kp"], 100)
            self.assertFalse(hasattr(worker, "request_pid_ram_write"))
        finally:
            worker.request_shutdown()
            self.assertTrue(thread.wait(3000))

    def test_simulated_worker_requires_post_preflight_authorization_and_runs_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            thread = QtCore.QThread()
            worker = motor_control_gui.MotorControlWorker(
                simulate=True, log_dir=pathlib.Path(directory))
            worker.moveToThread(thread)
            authorizations, finished, csv_paths = [], [], []
            worker.authorization_required.connect(authorizations.append)
            worker.session_finished.connect(lambda ok, text: finished.append((ok, text)))
            worker.csv_ready.connect(csv_paths.append)
            worker.finished.connect(thread.quit)
            thread.started.connect(worker.run)
            thread.start()
            try:
                worker.connect_motor("SIMULATED", 500_000, 1)
                self.assertTrue(self.wait_until(lambda: worker._controller is not None and
                                               worker._controller.connected))
                worker.start_motion(5, 0.15)
                self.assertTrue(self.wait_until(lambda: bool(authorizations), 2))
                self.assertFalse(worker._controller.motor.enabled)
                worker.submit_authorization(motor_control_gui.GUI_CONFIRMATION)
                self.assertTrue(self.wait_until(lambda: bool(finished), 3))
                self.assertTrue(finished[-1][0])
                self.assertTrue(self.wait_until(lambda: bool(csv_paths)))
                self.assertTrue(pathlib.Path(csv_paths[-1]).is_file())
            finally:
                worker.request_shutdown()
                self.assertTrue(thread.wait(3000))

    def test_worker_shutdown_during_motion_stops_disables_and_releases_thread(self):
        with tempfile.TemporaryDirectory() as directory:
            thread = QtCore.QThread()
            worker = motor_control_gui.MotorControlWorker(
                simulate=True, log_dir=pathlib.Path(directory))
            worker.moveToThread(thread)
            authorization_requested = []
            worker.authorization_required.connect(authorization_requested.append)
            thread.started.connect(worker.run)
            thread.start()
            motor = None
            try:
                worker.connect_motor("SIMULATED", 500_000, 1)
                self.assertTrue(self.wait_until(lambda: worker._controller is not None and
                                               worker._controller.connected))
                worker.start_motion(5, 2)
                self.assertTrue(self.wait_until(lambda: bool(authorization_requested), 2))
                worker.submit_authorization(motor_control_gui.GUI_CONFIRMATION)
                self.assertTrue(self.wait_until(lambda: worker._controller.motor.enabled, 2))
                motor = worker._controller.motor
                worker.request_shutdown()
                self.assertTrue(thread.wait(3000))
                self.assertFalse(motor.enabled)
                self.assertEqual(motor.commanded_rpm, 0)
            finally:
                if thread.isRunning():
                    worker.request_shutdown()
                    self.assertTrue(thread.wait(3000))

    def test_gui_home_status_and_signed_single_turn_match_observed_joint_direction(self):
        window = motor_control_gui.MotorControlWindow(simulate=True)
        try:
            base = {
                "session_id": "test", "session_path": "/tmp/test.json",
                "state": "WRITE_ACKNOWLEDGED_UNVERIFIED",
                "verification_status": "POWER_CYCLE_REQUIRED",
                "write": {}, "verification": None,
                "angles": {"single_turn_degrees": 270.0, "multi_turn_degrees": -90.0},
            }
            window._home_received(base)
            self.assertIn("persistence not checked after power cycle",
                          window.home_status.text())
            self.assertIn("GO HOME and START remain available", window.home_status.text())
            self.assertEqual(window.home_theta_deg.text(), "-90.00 deg")
            window.running = True
            window._update_theta()
            self.assertEqual(window.home_theta_deg.text(), "-90.00 deg")
            self.assertEqual(window.home_theta_source.text(), "0x92 continuous")
            window.running = False
            window.home_direction_sign.setCurrentIndex(1)
            self.assertEqual(window.home_theta_deg.text(), "90.00 deg")
            window.home_direction_sign.setCurrentIndex(0)
            zero_without_mark = dict(base)
            zero_without_mark.update({
                "state": "VERIFIED_AFTER_POWER_CYCLE",
                "verification": {"passed": True,
                                 "mechanical_home_position_operator_attested": False},
                "angles": {"single_turn_degrees": 0.0, "multi_turn_degrees": 0.0},
            })
            window._home_received(zero_without_mark)
            self.assertNotIn("HOME VERIFIED", window.home_status.text())
            with_mark = dict(zero_without_mark)
            with_mark["verification"] = {
                "passed": True, "mechanical_home_position_operator_attested": True}
            with_mark["current_home_check"] = {
                "passed": True, "mechanical_home_position_operator_attested": True}
            window._home_received(with_mark)
            self.assertIn("HOME VERIFIED", window.home_status.text())
            restored = dict(with_mark, current_home_check=None)
            window._home_received(restored)
            self.assertIn("previous power-cycle check",
                          window.home_status.text())
        finally:
            window.worker.request_shutdown()
            self.assertTrue(window.thread.wait(3000))
            window.deleteLater()

    def test_simulated_worker_home_write_confirms_disable_and_exact_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            thread = QtCore.QThread()
            worker = motor_control_gui.MotorControlWorker(
                simulate=True, log_dir=pathlib.Path(directory) / "motor",
                home_log_dir=pathlib.Path(directory) / "home")
            worker.moveToThread(thread)
            authorizations, results, errors = [], [], []
            worker.home_authorization_required.connect(authorizations.append)
            worker.home_result.connect(results.append)
            worker.home_error.connect(errors.append)
            worker.finished.connect(thread.quit)
            thread.started.connect(worker.run)
            thread.start()
            try:
                worker.connect_motor("SIMULATED", 500_000, 1)
                self.assertTrue(self.wait_until(lambda: worker._controller is not None and
                                               worker._controller.connected))
                worker.request_set_home()
                self.assertTrue(self.wait_until(lambda: bool(authorizations), 3))
                self.assertFalse(errors)
                self.assertTrue(worker._controller._stop_status["stop_acknowledged"])
                self.assertTrue(worker._controller._stop_status["disable_acknowledged"])
                self.assertTrue(worker._controller._stop_status["zero_speed_feedback_confirmed"])
                self.assertFalse(worker._controller.motor.enabled)
                self.assertEqual(authorizations[-1]["frame_hex"],
                                 "19 00 00 00 00 00 00 00")
                worker.submit_home_authorization(motor_control_gui.FINAL_CONFIRMATION)
                self.assertTrue(self.wait_until(lambda: bool(results), 3))
                self.assertEqual(results[-1]["state"], "WRITE_ACKNOWLEDGED_UNVERIFIED")
                self.assertTrue(pathlib.Path(results[-1]["session_path"]).is_file())
            finally:
                worker.request_shutdown()
                self.assertTrue(thread.wait(3000))

    def test_set_home_blocks_rom_write_when_disable_acknowledgement_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            thread = QtCore.QThread()
            worker = motor_control_gui.MotorControlWorker(
                simulate=True, log_dir=pathlib.Path(directory) / "motor",
                home_log_dir=pathlib.Path(directory) / "home")
            worker.moveToThread(thread)
            authorizations, errors, events = [], [], []
            worker.home_authorization_required.connect(authorizations.append)
            worker.home_error.connect(errors.append)
            worker.event.connect(lambda _level, message: events.append(message))
            worker.finished.connect(thread.quit)
            thread.started.connect(worker.run)
            thread.start()
            try:
                worker.connect_motor("SIMULATED", 500_000, 1)
                self.assertTrue(self.wait_until(lambda: worker._controller is not None and
                                               worker._controller.connected))
                motor = worker._controller.motor
                original_exchange = motor.exchange

                def malformed_disable(command, payload=b"\x00" * 7):
                    if command == 0x80:
                        return bytes((0x80, 0x01)) + b"\x00" * 6
                    return original_exchange(command, payload)

                with patch.object(motor, "exchange", side_effect=malformed_disable):
                    worker.request_set_home()
                    self.assertTrue(self.wait_until(lambda: bool(errors), 3))
                self.assertIn("STOP/DISABLE", errors[-1])
                self.assertFalse(authorizations)
                self.assertFalse(any(message.startswith("TX 0x141: 19 ") for message in events))
            finally:
                worker.request_shutdown()
                self.assertTrue(thread.wait(3000))


if __name__ == "__main__":
    unittest.main()
