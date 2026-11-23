import csv
import pathlib
import sys
import tempfile
import threading
import time
import unittest
import json
import struct
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from motor_can import CommunicationError, ControllerFault, ResponseTimeout
from motor_gui_core import (GUI_CONFIRMATION, MotorSessionController,
                            SimulatedWaveshareMotor, decode_angle_92, decode_pid_30,
                            distance_from_home, positive_distance_to_home,
                            validate_auto_home_request, validate_motion_request)
import absolute_home_calibration as home


class RecordingSimulatedMotor(SimulatedWaveshareMotor):
    instances = []

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.commands = []
        self.sent_payloads = []
        type(self).instances.append(self)

    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        self.commands.append(command)
        self.sent_payloads.append((command, payload))
        return super().exchange(command, payload, timeout=timeout)


class CurrentFeedbackMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        if command == 0x9C:
            return raw[:2] + struct.pack("<h", -7) + raw[4:]
        return raw


class OverspeedMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        if command == 0x9C and self.commanded_rpm:
            return bytes.fromhex("9C 20 00 00 59 02 00 00")  # 601 deg/s = 100.17 RPM
        return raw


class TimeoutMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        if command == 0x92 and self.commanded_rpm:
            self.commands.append(command)
            raise ResponseTimeout("simulated angle timeout")
        return super().exchange(command, payload, timeout=timeout)


class MalformedStopAckMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        return bytes.fromhex("81 01 00 00 00 00 00 00") if command == 0x81 else raw


class MalformedHoldAckMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        return bytes.fromhex("A4 00 00 00 00 00 00") if command == 0xA4 else raw


class FaultCandidateMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        return bytes.fromhex("9A 20 30 0B 00 00 20 08") if command == 0x9A else raw


class UnstableHomeAngleMotor(RecordingSimulatedMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.home_angle_reads = 0

    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        if command == 0x94:
            self.home_angle_reads += 1
            value = (self.home_angle_reads * 100) % 36_000
            return bytes((0x94, 0, 0, 0)) + value.to_bytes(4, "little")
        return raw


class MalformedHomeAckMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        return bytes.fromhex("19 01 02 03 04 05 06 07") if command == 0x19 else raw


class HomeWriteTimeoutMotor(RecordingSimulatedMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        if command == 0x19:
            self.commands.append(command)
            raise ResponseTimeout("simulated 0x19 timeout")
        return super().exchange(command, payload, timeout=timeout)


class StepAutoHomeMotor(RecordingSimulatedMotor):
    """Advance deterministically so auto-HOME tests do not depend on wall time."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.angle_raw = 35_000

    def _advance(self):
        self.last_update = time.monotonic()

    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        if command == 0x94 and self.enabled and self.commanded_rpm:
            self.angle_raw = (self.angle_raw + 20) % 36_000
        if command == 0x81:
            # Model the measured roughly 0.2 degree post-STOP position change.
            self.angle_raw = (self.angle_raw - 20) % 36_000
        return super().exchange(command, payload, timeout=timeout)


class NegativeStepAutoHomeMotor(StepAutoHomeMotor):
    """Positive speed decreases encoder angle, as observed on the mounted motor."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.angle_raw = 3_000

    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        if command == 0x94 and self.enabled and self.commanded_rpm:
            self.angle_raw = (self.angle_raw - 100) % 36_000
        if command == 0x81:
            self.angle_raw = (self.angle_raw - 20) % 36_000
        return RecordingSimulatedMotor.exchange(self, command, payload, timeout=timeout)


class StaticAwayFromHomeMotor(RecordingSimulatedMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.angle_raw = 18_000

    def _advance(self):
        # Report the commanded speed but deliberately keep position fixed.
        self.last_update = time.monotonic()


class MismatchedHomeAnglesMotor(StepAutoHomeMotor):
    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        if command == 0x92:
            shifted = (self.angle_raw + 2000) & ((1 << 56) - 1)
            return bytes((0x92,)) + shifted.to_bytes(7, "little")
        return raw


class ReverseDuringHoldApproachMotor(RecordingSimulatedMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.hold_approach = False

    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        if command == 0xA2 and int.from_bytes(payload[3:7], "little", signed=True) == 300:
            self.hold_approach = True
        if command == 0x9C and self.hold_approach:
            return struct.pack("<BbhhH", 0x9C, 32, 0, -100, self.encoder_raw)
        return raw


class QuantizedLowSpeedMotor(RecordingSimulatedMotor):
    """Model the observed 0/5 dps 0x9C readings near commanded 0.5 RPM."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.low_speed_reads = 0

    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        raw = super().exchange(command, payload, timeout=timeout)
        if command == 0x9C and self.commanded_rpm == 0.5:
            self.low_speed_reads += 1
            raw_speed = 5 if self.low_speed_reads % 2 else 0
            return struct.pack("<BbhhH", 0x9C, 32, 40, raw_speed, self.encoder_raw)
        return raw


class SettlingStopMotor(RecordingSimulatedMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stop_feedback = []

    def exchange(self, command, payload=b"\x00" * 7, *, timeout=None):
        if command == 0x81:
            raw = super().exchange(command, payload, timeout=timeout)
            self.stop_feedback = [(-8, 100), (0, 54), (0, 58),
                                  (0, 58), (0, 58), (0, 58)]
            return raw
        if command == 0x9C and self.stop_feedback:
            self.commands.append(command)
            speed, encoder = self.stop_feedback.pop(0)
            return struct.pack("<BbhhH", 0x9C, 32, 0, speed, encoder)
        return super().exchange(command, payload, timeout=timeout)


class MotorGuiCoreTests(unittest.TestCase):
    def setUp(self):
        RecordingSimulatedMotor.instances.clear()

    def controller(self, directory, factory=RecordingSimulatedMotor):
        self.events = []
        self.samples = []
        self.states = []
        return MotorSessionController(
            transport_factory=factory,
            event_callback=lambda level, text: self.events.append((level, text)),
            telemetry_callback=self.samples.append,
            state_callback=self.states.append,
            log_dir=pathlib.Path(directory),
            home_log_dir=pathlib.Path(directory) / "home",
        )

    def disabled_controller(self, directory, factory=RecordingSimulatedMotor):
        controller = self.controller(directory, factory)
        controller.sleep = lambda _seconds: None
        controller.connect("SIMULATED", 500_000, 1)
        self.assertTrue(controller.disable("home_test_disable"))
        return controller

    def test_request_validation_accepts_positive_100_rpm_20_second_limits(self):
        validate_motion_request(100, 20)
        for rpm, duration in ((0, 1), (-1, 1), (100.01, 1), (1, 0), (1, 20.01),
                              (float("nan"), 1)):
            with self.subTest(rpm=rpm, duration=duration), self.assertRaises(ValueError):
                validate_motion_request(rpm, duration)

    def test_auto_home_geometry_and_bounds(self):
        self.assertEqual(positive_distance_to_home(0), 0)
        self.assertAlmostEqual(positive_distance_to_home(350), 10)
        self.assertAlmostEqual(distance_from_home(359.5), 0.5)
        self.assertAlmostEqual(distance_from_home(0.5), 0.5)
        validate_auto_home_request(1.0, 5.0)
        for tolerance, speed in ((0.09, 1), (5.01, 1), (1, 0.49), (1, 5.01)):
            with self.subTest(tolerance=tolerance, speed=speed), self.assertRaises(ValueError):
                validate_auto_home_request(tolerance, speed)

    def test_start_auto_homes_then_stops_only_and_runs_without_writing_home(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, StepAutoHomeMotor)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(
                5, 0.05, authorization=GUI_CONFIRMATION,
                auto_home=True, home_tolerance_deg=1.0, home_speed_rpm=5.0)
            motor = StepAutoHomeMotor.instances[-1]
            controls = [command for command in motor.commands
                        if command in (0x19, 0x88, 0xA2, 0x81, 0x80)]
            self.assertNotIn(0x19, controls)
            self.assertGreaterEqual(controls.count(0xA2), 2)
            first_a2 = controls.index(0xA2)
            second_a2 = controls.index(0xA2, first_a2 + 1)
            self.assertEqual(controls[first_a2 + 1:first_a2 + 3], [0x81, 0x88])
            self.assertGreater(second_a2, first_a2 + 1)
            self.assertNotIn(0x80, controls)
            self.assertIn("AUTO_HOMING", self.states)
            self.assertIn("AUTO_HOME_VERIFY", {
                sample["motor_state"] for sample in self.samples})
            self.assertEqual(self.states[-1], "STOPPED_ENABLED")
            controller.disconnect()

    def test_experimental_hold_after_start_uses_fresh_current_angle(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION,
                                    auto_home=False, position_hold=True)
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertEqual(motor.commands.count(0xA4), 1)
            self.assertNotIn(0x81, motor.commands)
            final_speed = [payload for command, payload in motor.sent_payloads
                           if command == 0xA2][-1]
            self.assertEqual(int.from_bytes(final_speed[3:7], "little", signed=True), 300)
            self.assertTrue(controller.position_hold_active)
            self.assertEqual(self.states[-1], "POSITION_HOLD")
            self.assertNotIn(0x80, motor.commands)
            controller.poll_idle_once()
            self.assertEqual(controller.last_sample.motor_state, "POSITION_HOLD")
            controller.disconnect()
            self.assertEqual([cmd for cmd in motor.commands if cmd in (0x81, 0x80)][-2:],
                             [0x81, 0x80])

    def test_experimental_hold_bridges_auto_home_and_final_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, StepAutoHomeMotor)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION,
                                    auto_home=True, position_hold=True)
            motor = StepAutoHomeMotor.instances[-1]
            controls = [cmd for cmd in motor.commands if cmd in (0xA2, 0xA4, 0x81, 0x80)]
            self.assertEqual(controls.count(0xA4), 2)
            self.assertIn(0xA2, controls[controls.index(0xA4) + 1:])
            self.assertEqual(controls[-1], 0xA4)
            self.assertNotIn(0x81, controls)
            self.assertNotIn(0x80, controls)
            controller.disconnect()

    def test_experimental_hold_after_go_home(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, StepAutoHomeMotor)
            controller.connect("SIMULATED", 500_000, 1)
            controller.home_only(1.0, 5.0, authorization=GUI_CONFIRMATION,
                                 position_hold=True)
            motor = StepAutoHomeMotor.instances[-1]
            self.assertEqual(motor.commands.count(0xA4), 1)
            self.assertNotIn(0x81, motor.commands)
            home_payload = next(payload for command, payload in motor.sent_payloads
                                if command == 0xA4)
            self.assertEqual(int.from_bytes(home_payload[3:7], "little", signed=True),
                             36_000)
            self.assertEqual(self.states[-1], "POSITION_HOLD")
            self.assertTrue(controller.position_hold_active)
            controller.disconnect()

    def test_operator_stop_uses_immediate_stop_instead_of_position_transition(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            errors = []
            thread = threading.Thread(
                target=lambda: self._capture(
                    errors, controller.start_motion, 5, 2,
                    authorization=GUI_CONFIRMATION, auto_home=False,
                    position_hold=True))
            thread.start()
            motor = RecordingSimulatedMotor.instances[-1]
            deadline = time.monotonic() + 1.5
            while 0xA2 not in motor.commands and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertIn(0xA2, motor.commands)
            controller.request_stop("operator_stop")
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(motor.commands.count(0xA4), 0)
            self.assertEqual(motor.commands.count(0x81), 1)
            self.assertEqual(self.states[-1], "STOPPED_ENABLED")
            controller.disconnect()

    def test_stop_button_ends_idle_position_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION,
                                    auto_home=False, position_hold=True)
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertTrue(controller.stop_position_hold())
            self.assertFalse(controller.position_hold_active)
            self.assertTrue(motor.enabled)
            self.assertEqual(self.states[-1], "STOPPED_ENABLED")
            self.assertEqual(motor.commands.count(0x81), 1)
            controller.disconnect()

    def test_bad_position_hold_ack_disables_and_reports_error(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, MalformedHoldAckMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(CommunicationError, "invalid feedback response"):
                controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION,
                                        auto_home=False, position_hold=True)
            motor = MalformedHoldAckMotor.instances[-1]
            self.assertIn(0x80, motor.commands)
            self.assertFalse(motor.enabled)
            self.assertFalse(controller.position_hold_active)
            controller.disconnect()

    def test_home_angle_disagreement_blocks_direct_position_command(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, MismatchedHomeAnglesMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(CommunicationError, "HOME angles disagree"):
                controller.home_only(1.0, 5.0, authorization=GUI_CONFIRMATION,
                                     position_hold=True)
            motor = MismatchedHomeAnglesMotor.instances[-1]
            self.assertNotIn(0xA4, motor.commands)
            self.assertIn(0x80, motor.commands)
            controller.disconnect()

    def test_reverse_during_hold_approach_stops_and_disables(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, ReverseDuringHoldApproachMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(CommunicationError, "unsafe speed or direction"):
                controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION,
                                        auto_home=False, position_hold=True)
            motor = ReverseDuringHoldApproachMotor.instances[-1]
            self.assertNotIn(0xA4, motor.commands)
            self.assertEqual([cmd for cmd in motor.commands if cmd in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            controller.disconnect()

    def test_quantized_low_speed_feedback_can_settle_without_stopping(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, QuantizedLowSpeedMotor)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(10, 0.05, authorization=GUI_CONFIRMATION,
                                    auto_home=False, position_hold=True)
            motor = QuantizedLowSpeedMotor.instances[-1]
            self.assertGreaterEqual(motor.low_speed_reads, 8)
            self.assertEqual(motor.commands.count(0xA4), 1)
            self.assertNotIn(0x81, motor.commands)
            self.assertTrue(controller.position_hold_active)
            controller.disconnect()

    def test_position_hold_drift_triggers_stop_disable(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION,
                                    auto_home=False, position_hold=True)
            motor = RecordingSimulatedMotor.instances[-1]
            motor.angle_raw += 200  # 2 degrees away from the captured hold target.
            with self.assertRaisesRegex(CommunicationError, "position hold feedback out of bounds"):
                controller.poll_idle_once()
            self.assertFalse(controller.position_hold_active)
            self.assertFalse(motor.enabled)
            self.assertEqual([cmd for cmd in motor.commands if cmd in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            controller.disconnect()

    def test_go_home_only_stops_and_never_starts_target_run_or_writes_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, StepAutoHomeMotor)
            controller.connect("SIMULATED", 500_000, 1)
            controller.home_only(1.0, 5.0, authorization=GUI_CONFIRMATION)
            motor = StepAutoHomeMotor.instances[-1]
            controls = [command for command in motor.commands
                        if command in (0x19, 0x88, 0xA2, 0x81, 0x80)]
            self.assertEqual(controls[0], 0x88)
            self.assertEqual(controls[-1], 0x81)
            self.assertNotIn(0x80, controls)
            self.assertNotIn(0x19, controls)
            self.assertEqual(controls.count(0x88), 1)
            self.assertTrue(all(sample["requested_rpm"] in (None, 5.0, 0.5)
                                for sample in self.samples))
            self.assertNotIn("RUNNING", self.states)
            self.assertEqual(self.states[-1], "STOPPED_ENABLED")
            with controller.last_csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(any(row["motor_state"] == "AUTO_HOME_VERIFY" for row in rows))
            controller.disconnect()

    def test_go_home_adapts_to_negative_encoder_direction(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, NegativeStepAutoHomeMotor)
            controller.connect("SIMULATED", 500_000, 1)
            controller.home_only(1.0, 5.0, authorization=GUI_CONFIRMATION)
            motor = NegativeStepAutoHomeMotor.instances[-1]
            self.assertIn("AUTO_HOME_VERIFY", {sample["motor_state"] for sample in self.samples})
            self.assertTrue(any("encoder direction=-1" in text
                                for _level, text in self.events))
            self.assertLessEqual(distance_from_home(motor.angle_raw / 100.0), 1.0)
            self.assertNotIn(0x19, motor.commands)
            self.assertEqual(self.states[-1], "STOPPED_ENABLED")
            controller.disconnect()

    def test_go_home_declined_authorization_never_enables(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, StepAutoHomeMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaises(PermissionError):
                controller.home_only(1.0, 5.0, authorization="")
            commands = StepAutoHomeMotor.instances[-1].commands
            self.assertNotIn(0x88, commands)
            self.assertNotIn(0xA2, commands)
            self.assertNotIn(0x19, commands)
            self.assertEqual([command for command in commands if command in (0x81, 0x80)],
                             [0x81, 0x80])
            controller.disconnect()

    def test_operator_stop_during_auto_home_cleans_up_and_never_starts_requested_run(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, StaticAwayFromHomeMotor)
            controller.connect("SIMULATED", 500_000, 1)
            thread = threading.Thread(
                target=lambda: controller.start_motion(
                    20, 1, authorization=GUI_CONFIRMATION,
                    auto_home=True, home_tolerance_deg=1.0, home_speed_rpm=5.0))
            thread.start()
            motor = StaticAwayFromHomeMotor.instances[-1]
            deadline = time.monotonic() + 2
            while motor.commands.count(0xA2) < 1 and time.monotonic() < deadline:
                time.sleep(0.005)
            controller.request_stop("test_stop_during_home")
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(motor.commands.count(0xA2), 1)
            self.assertEqual([command for command in motor.commands if command in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            self.assertNotIn(0x19, motor.commands)
            self.assertEqual(self.states[-1], "IDLE")
            controller.disconnect()

    def test_cleanup_accepts_only_three_consecutive_settled_samples_after_deceleration(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, SettlingStopMotor)
            controller.connect("SIMULATED", 500_000, 1)
            self.assertTrue(controller._cleanup("settling_test"))
            motor = SettlingStopMotor.instances[-1]
            self.assertEqual(motor.commands.count(0x9C), 6)
            self.assertTrue(controller._stop_status["zero_speed_feedback_confirmed"])
            self.assertIn("bounded ten-sample window",
                          controller._stop_status["stop_confirmation_detail"])
            controller.disconnect()

    def test_connect_does_not_enable_and_prevents_second_owner(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertEqual(motor.commands, [])
            with self.assertRaisesRegex(CommunicationError, "already open"):
                controller.connect("SIMULATED", 500_000, 1)
            controller.disconnect()

    def test_one_megabit_connection_reads_status_without_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 1_000_000, 1)
            controller.read_status("BITRATE_TEST")
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertEqual(motor.can_bitrate, 1_000_000)
            self.assertIn(0x9A, motor.commands)
            self.assertFalse(set(motor.commands) & {0x19, 0x88, 0xA2, 0xA4})
            controller.disconnect()

    def test_unsupported_gui_bitrate_is_rejected_before_transport(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            with self.assertRaisesRegex(ValueError, "500000 or 1000000"):
                controller.connect("SIMULATED", 250_000, 1)
            self.assertEqual(RecordingSimulatedMotor.instances, [])

    def test_authorization_is_required_before_any_motion_command(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaises(PermissionError):
                controller.start_motion(5, 0.1, authorization="yes")
            self.assertNotIn(0x88, RecordingSimulatedMotor.instances[-1].commands)
            self.assertNotIn(0xA2, RecordingSimulatedMotor.instances[-1].commands)
            controller.disconnect()

    def test_bounded_session_polls_speed_angle_status_and_writes_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(100, 0.3, authorization=GUI_CONFIRMATION)
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertIn(0x88, motor.commands)
            self.assertIn(0xA2, motor.commands)
            self.assertGreaterEqual(motor.commands.count(0x9C), 6)
            self.assertGreaterEqual(motor.commands.count(0x92), 2)
            self.assertEqual([item for item in motor.commands if item in (0x81, 0x80)],
                             [0x81])
            self.assertTrue(motor.enabled)
            self.assertIn("RUNNING", self.states)
            self.assertEqual(self.states[-1], "STOPPED_ENABLED")
            controller.poll_idle_once()
            self.assertEqual(controller.last_sample.motor_state, "STOPPED_ENABLED")
            path = controller.last_csv_path
            self.assertTrue(path.is_file())
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(any(row["raw_speed_dps"] == "600" for row in rows))
            self.assertTrue(any(row["raw_angle"] for row in rows))
            self.assertEqual(rows[-1]["event_stop_reason"], "duration_elapsed")
            self.assertTrue(all(row["protection_state"] == "UNKNOWN" for row in rows))
            controller.disconnect()

    def test_torque_current_is_logged_only_from_fresh_9c_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, CurrentFeedbackMotor)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(5, 0.3, authorization=GUI_CONFIRMATION)
            with controller.last_csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            fresh = [row for row in rows if row["sample_source"] == "0x9C"
                     and row["motor_state"] == "RUNNING"]
            self.assertTrue(fresh)
            self.assertTrue(all(row["torque_current_raw"] == "-7" for row in fresh))
            self.assertTrue(all(row["torque_current_raw"] == "" for row in rows
                                if row["sample_source"] != "0x9C"))
            self.assertEqual(controller.last_sample.torque_current_raw, None)
            controller.disconnect()

    def test_hard_overspeed_is_not_relaxed_and_cleanup_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, OverspeedMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(CommunicationError, "hard_overspeed"):
                controller.start_motion(5, 0.2, authorization=GUI_CONFIRMATION)
            motor = OverspeedMotor.instances[-1]
            self.assertEqual([item for item in motor.commands if item in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            self.assertTrue(any("Overspeed" in text for _, text in self.events))
            controller.disconnect()

    def test_telemetry_timeout_causes_stop_disable(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, TimeoutMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(ResponseTimeout, "angle timeout"):
                controller.start_motion(5, 0.2, authorization=GUI_CONFIRMATION)
            commands = TimeoutMotor.instances[-1].commands
            self.assertEqual([item for item in commands if item in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            controller.disconnect()

    def test_known_legacy_fault_candidate_blocks_enable(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, FaultCandidateMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(ControllerFault, "fault-candidate"):
                controller.start_motion(5, 0.2, authorization=GUI_CONFIRMATION)
            commands = FaultCandidateMotor.instances[-1].commands
            self.assertNotIn(0x88, commands)
            self.assertNotIn(0xA2, commands)
            self.assertEqual([item for item in commands if item in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            controller.disconnect()

    def test_operator_stop_interrupts_session_and_shutdown_cleanup_is_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            errors = []
            thread = threading.Thread(
                target=lambda: self._capture(errors, controller.start_motion, 5, 2,
                                             authorization=GUI_CONFIRMATION))
            thread.start()
            motor = RecordingSimulatedMotor.instances[-1]
            deadline = time.monotonic() + 1.5
            while 0xA2 not in motor.commands and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertIn(0xA2, motor.commands)
            controller.request_stop("operator_stop")
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            commands = motor.commands
            self.assertEqual([item for item in commands if item in (0x81, 0x80)], [0x81])
            self.assertEqual(self.states[-1], "STOPPED_ENABLED")
            self.assertTrue(motor.enabled)
            controller.disconnect()
            self.assertEqual([item for item in commands if item in (0x81, 0x80)][-2:],
                             [0x81, 0x80])

    def test_unconfirmed_normal_stop_attempts_disable_and_reports_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory, MalformedStopAckMotor)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(CommunicationError, "normal STOP was not confirmed"):
                controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION)
            motor = MalformedStopAckMotor.instances[-1]
            self.assertIn(0x80, motor.commands)
            self.assertFalse(motor.enabled)
            controller.disconnect()

    def test_disable_after_normal_stop_sends_stop_then_disable(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(5, 0.05, authorization=GUI_CONFIRMATION)
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertTrue(motor.enabled)
            self.assertTrue(controller.disable())
            self.assertEqual([item for item in motor.commands if item in (0x81, 0x80)],
                             [0x81, 0x81, 0x80])
            self.assertFalse(motor.enabled)
            self.assertEqual(self.states[-1], "IDLE")
            controller.disconnect()

    def test_disable_request_during_motion_still_disables(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            errors = []
            thread = threading.Thread(
                target=lambda: self._capture(errors, controller.start_motion, 5, 2,
                                             authorization=GUI_CONFIRMATION))
            thread.start()
            motor = RecordingSimulatedMotor.instances[-1]
            deadline = time.monotonic() + 1.5
            while 0xA2 not in motor.commands and time.monotonic() < deadline:
                time.sleep(0.01)
            controller.request_stop("operator_disable")
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual([item for item in motor.commands if item in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            self.assertFalse(motor.enabled)
            controller.disconnect()

    @staticmethod
    def _capture(errors, function, *args, **kwargs):
        try:
            function(*args, **kwargs)
        except BaseException as exc:
            errors.append(exc)

    def test_disable_sends_stop_before_disable(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            self.assertTrue(controller.disable())
            commands = RecordingSimulatedMotor.instances[-1].commands
            self.assertEqual([item for item in commands if item in (0x81, 0x80)][-2:],
                             [0x81, 0x80])
            controller.disconnect()

    def test_idle_polling_reports_provisional_angle_speed_and_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.poll_idle_once()
            self.assertTrue(self.samples)
            self.assertTrue(any(sample["raw_speed_dps"] == 0 for sample in self.samples))
            self.assertTrue(any(sample["raw_angle"] == 0 for sample in self.samples))
            self.assertTrue(all(sample["utc_timestamp"].endswith("+00:00") for sample in self.samples))
            self.assertTrue(any(sample["protection_state"] == "UNKNOWN" for sample in self.samples))
            controller.disconnect()

    def test_signed_56bit_angle_decode(self):
        positive = bytes((0x92,)) + (12345).to_bytes(7, "little")
        negative = bytes((0x92,)) + ((1 << 56) - 12345).to_bytes(7, "little")
        self.assertEqual(decode_angle_92(positive), (12345, 123.45))
        self.assertEqual(decode_angle_92(negative), (-12345, -123.45))

    def test_pid_read_decoder_is_read_only(self):
        values = {"angle_kp": 1, "angle_ki": 2, "speed_kp": 3,
                  "speed_ki": 4, "torque_kp": 5, "torque_ki": 6}
        self.assertEqual(decode_pid_30(bytes.fromhex("30 00 01 02 03 04 05 06")), values)
        with self.assertRaises(CommunicationError):
            decode_pid_30(bytes.fromhex("30 01 01 02 03 04 05 06"))

    def test_pid_ui_core_can_only_read_and_never_sends_write_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            original = controller.read_pid()
            self.assertEqual(original["speed_kp"], 100)
            commands = RecordingSimulatedMotor.instances[-1].commands
            self.assertEqual(commands, [0x30])
            self.assertFalse(hasattr(controller, "write_pid_ram"))
            self.assertNotIn(0x31, commands)
            self.assertNotIn(0x32, commands)
            controller.disconnect()

    def test_fresh_sources_estimators_and_cleanup_evidence_are_logged(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.start_motion(5, 0.25, authorization=GUI_CONFIRMATION)
            with controller.last_csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(any(row["sample_source"] == "0x9C" for row in rows))
            self.assertTrue(any(row["sample_source"] == "0x92" and
                                row["angle_derived_rpm_provisional"] for row in rows))
            self.assertTrue(any(row["filtered_speed_rpm_visual"] for row in rows))
            final = rows[-1]
            self.assertEqual(final["sample_source"], "cleanup_result")
            self.assertEqual(final["stop_acknowledged"], "True")
            self.assertEqual(final["disable_acknowledged"], "")
            self.assertEqual(final["motor_state"], "STOPPED_ENABLED")
            self.assertEqual(final["zero_speed_feedback_confirmed"], "True")
            self.assertEqual(final["physical_stop_verified"], "False")
            controller.disconnect()

    def test_home_read_is_serialized_and_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            result = controller.read_home_angles()
            commands = RecordingSimulatedMotor.instances[-1].commands
            self.assertEqual(commands, [0x94, 0x92, 0x90])
            self.assertEqual(result["single_turn_degrees"], 0.0)
            self.assertEqual(result["multi_turn_degrees"], 0.0)
            self.assertFalse(set(commands) & {0x19, 0x91, 0x88, 0xA2})
            controller.disconnect()

    def test_successful_set_home_logs_preflight_and_one_unverified_write(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.disabled_controller(directory)
            summaries = []
            result = controller.set_absolute_home(
                lambda summary: summaries.append(summary) or home.FINAL_CONFIRMATION)
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertEqual(motor.commands.count(0x19), 1)
            self.assertNotIn(0x91, motor.commands)
            self.assertEqual(result["state"], "WRITE_ACKNOWLEDGED_UNVERIFIED")
            self.assertEqual(result["write"]["acknowledgement_status"],
                             "ACKNOWLEDGED_UNVERIFIED")
            self.assertEqual(summaries[0]["frame_hex"], "19 00 00 00 00 00 00 00")
            log = json.loads(pathlib.Path(result["session_path"]).read_text())
            self.assertIn("previous_home_information", log)
            self.assertEqual(len(log["preflight"]["stationary_check"]["samples"]), 5)
            self.assertEqual(len(log["angle_stability_check"]["samples"]), 5)
            self.assertTrue(log["operator_confirmation"]["exact_phrase_accepted"])
            controller.disconnect()

    def test_home_session_records_selected_one_megabit_bitrate_without_write(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.sleep = lambda _seconds: None
            controller.connect("SIMULATED", 1_000_000, 1)
            self.assertTrue(controller.disable("home_test_disable"))
            with self.assertRaises(home.AuthorizationError):
                controller.set_absolute_home(lambda _summary: "")
            self.assertEqual(controller._home_session["hardware"]["can_bitrate"], 1_000_000)
            self.assertEqual(controller._home_session["hardware"]["device"], "SIMULATED")
            self.assertNotIn(0x19, RecordingSimulatedMotor.instances[-1].commands)
            controller.disconnect()

    def test_home_write_is_blocked_while_running(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            controller.motion_active = True
            with self.assertRaisesRegex(CommunicationError, "RUNNING"):
                controller.set_absolute_home(lambda _summary: home.FINAL_CONFIRMATION)
            self.assertNotIn(0x19, RecordingSimulatedMotor.instances[-1].commands)
            controller.motion_active = False
            controller.disconnect()

    def test_home_write_requires_acknowledged_disable(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.controller(directory)
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaisesRegex(CommunicationError, "press DISABLE"):
                controller.set_absolute_home(lambda _summary: home.FINAL_CONFIRMATION)
            self.assertNotIn(0x19, RecordingSimulatedMotor.instances[-1].commands)
            controller.disconnect()

    def test_home_stale_feedback_and_unstable_angle_block_write(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.disabled_controller(directory)
            with patch.object(home, "collect_preflight",
                              side_effect=CommunicationError("speed feedback is stale")):
                with self.assertRaisesRegex(CommunicationError, "stale"):
                    controller.set_absolute_home(lambda _summary: home.FINAL_CONFIRMATION)
            self.assertNotIn(0x19, RecordingSimulatedMotor.instances[-1].commands)
            controller.disconnect()

        RecordingSimulatedMotor.instances.clear()
        with tempfile.TemporaryDirectory() as directory:
            controller = self.disabled_controller(directory, UnstableHomeAngleMotor)
            with self.assertRaisesRegex(CommunicationError, "not stable"):
                controller.set_absolute_home(lambda _summary: home.FINAL_CONFIRMATION)
            self.assertNotIn(0x19, UnstableHomeAngleMotor.instances[-1].commands)
            controller.disconnect()

    def test_missing_home_confirmation_never_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.disabled_controller(directory)
            with self.assertRaises(home.AuthorizationError):
                controller.set_absolute_home(lambda _summary: "NO")
            motor = RecordingSimulatedMotor.instances[-1]
            self.assertEqual(motor.commands.count(0x19), 0)
            self.assertEqual(controller._home_session["state"], "FAILED")
            controller.disconnect()

    def test_malformed_home_ack_and_timeout_are_unknown_without_retry(self):
        for factory, error_fragment in ((MalformedHomeAckMotor, "malformed"),
                                        (HomeWriteTimeoutMotor, "timeout")):
            RecordingSimulatedMotor.instances.clear()
            with self.subTest(factory=factory.__name__), tempfile.TemporaryDirectory() as directory:
                controller = self.disabled_controller(directory, factory)
                result = controller.set_absolute_home(
                    lambda _summary: home.FINAL_CONFIRMATION)
                motor = factory.instances[-1]
                self.assertEqual(motor.commands.count(0x19), 1)
                self.assertEqual(result["state"], "WRITE_RESULT_UNKNOWN")
                self.assertIn(error_fragment.lower(), result["write"]["error"].lower())
                controller.disconnect()

    def test_power_cycle_verification_requires_physical_home_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.disabled_controller(directory)
            written = controller.set_absolute_home(
                lambda _summary: home.FINAL_CONFIRMATION)
            self.assertEqual(written["state"], "WRITE_ACKNOWLEDGED_UNVERIFIED")
            controller.disconnect()
            controller.connect("SIMULATED", 500_000, 1)
            with self.assertRaises(home.AuthorizationError):
                controller.verify_absolute_home(1.0, lambda _summary: "")
            self.assertEqual(controller._home_session["state"],
                             "WRITE_ACKNOWLEDGED_UNVERIFIED")
            verified = controller.verify_absolute_home(
                1.0, lambda _summary: home.MECHANICAL_HOME_CONFIRMATION)
            self.assertEqual(verified["state"], "VERIFIED_AFTER_POWER_CYCLE")
            self.assertTrue(verified["verification"]["passed"])
            self.assertTrue(verified["verification"][
                "mechanical_home_position_operator_attested"])
            self.assertEqual(RecordingSimulatedMotor.instances[0].commands.count(0x19), 1)
            self.assertEqual(RecordingSimulatedMotor.instances[-1].commands.count(0x19), 0)
            controller.disconnect()

    def test_verify_restores_pending_write_after_gui_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.disabled_controller(directory)
            written = first.set_absolute_home(lambda _summary: home.FINAL_CONFIRMATION)
            path = pathlib.Path(written["session_path"])
            first.disconnect()

            second = self.controller(directory)
            second.sleep = lambda _seconds: None
            second.connect("SIMULATED", 500_000, 1)
            restored = second.load_saved_home_session()
            self.assertEqual(restored["session_path"], str(path))
            self.assertEqual(restored["state"], "WRITE_ACKNOWLEDGED_UNVERIFIED")
            verified = second.verify_absolute_home(
                1.0, lambda _summary: home.MECHANICAL_HOME_CONFIRMATION)
            self.assertEqual(verified["state"], "VERIFIED_AFTER_POWER_CYCLE")
            self.assertTrue(verified["current_home_check"]["passed"])
            commands = RecordingSimulatedMotor.instances[-1].commands
            self.assertFalse(set(commands) & {0x19, 0x88, 0xA2})
            self.assertEqual(json.loads(path.read_text())["state"],
                             "VERIFIED_AFTER_POWER_CYCLE")
            second.disconnect()

    def test_verify_rechecks_saved_verified_write_without_rom_command(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.disabled_controller(directory)
            written = first.set_absolute_home(lambda _summary: home.FINAL_CONFIRMATION)
            first.disconnect()
            first.connect("SIMULATED", 500_000, 1)
            first.verify_absolute_home(
                1.0, lambda _summary: home.MECHANICAL_HOME_CONFIRMATION)
            path = pathlib.Path(written["session_path"])
            original_verification = json.loads(path.read_text())["verification"]
            first.disconnect()

            second = self.controller(directory)
            second.sleep = lambda _seconds: None
            second.connect("SIMULATED", 500_000, 1)
            restored = second.load_saved_home_session()
            self.assertEqual(restored["state"], "VERIFIED_AFTER_POWER_CYCLE")
            self.assertIsNone(restored["current_home_check"])
            result = second.verify_absolute_home(
                1.0, lambda _summary: home.MECHANICAL_HOME_CONFIRMATION)
            self.assertTrue(result["current_home_check"]["passed"])
            saved = json.loads(path.read_text())
            self.assertEqual(saved["verification"], original_verification)
            self.assertEqual(len(saved["current_home_checks"]), 1)
            commands = RecordingSimulatedMotor.instances[-1].commands
            self.assertFalse(set(commands) & {0x19, 0x88, 0xA2})
            second.disconnect()

    def test_saved_verification_remains_historical_when_current_home_check_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.disabled_controller(directory)
            written = first.set_absolute_home(lambda _summary: home.FINAL_CONFIRMATION)
            first.disconnect()
            first.connect("SIMULATED", 500_000, 1)
            first.verify_absolute_home(
                1.0, lambda _summary: home.MECHANICAL_HOME_CONFIRMATION)
            path = pathlib.Path(written["session_path"])
            first.disconnect()

            second = self.controller(directory)
            second.sleep = lambda _seconds: None
            second.connect("SIMULATED", 500_000, 1)
            second.load_saved_home_session()
            motor = RecordingSimulatedMotor.instances[-1]
            motor.angle_raw = 200  # Two degrees away from physical HOME.
            result = second.verify_absolute_home(
                1.0, lambda _summary: home.MECHANICAL_HOME_CONFIRMATION)
            self.assertFalse(result["current_home_check"]["passed"])
            saved = json.loads(path.read_text())
            self.assertEqual(saved["state"], "VERIFIED_AFTER_POWER_CYCLE")
            self.assertTrue(saved["verification"]["passed"])
            self.assertFalse(saved["current_home_checks"][-1]["passed"])
            self.assertFalse(set(motor.commands) & {0x19, 0x88, 0xA2})
            second.disconnect()


if __name__ == "__main__":
    unittest.main()
