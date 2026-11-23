import pathlib
import sys
import unittest
import io
import tempfile
from types import SimpleNamespace
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import waveshare_motor
from motor_can import (CommunicationError, ControllerFault, ProtectionAssessment,
                       ProtectionState, ResponseTimeout, decode_feedback)


MOCK_VERIFIED = ProtectionAssessment(ProtectionState.VERIFIED_NORMAL,
                                     "simulated verified protection for software test")
MOCK_UNKNOWN = ProtectionAssessment(ProtectionState.UNKNOWN, "byte 6/7 mapping unverified")


class WaveshareMotorTests(unittest.TestCase):
    def test_led_state_test_sends_only_status_stop_disable_and_enable(self):
        motor = waveshare_motor.WaveshareMotor("unused", 1)
        motor.exchange = Mock(side_effect=[
            bytes.fromhex("9A 1F 30 0B 00 00 20 00"),
            bytes.fromhex("9C 1F FC FF 00 00 9E 50"),
            b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
            b"\x88" + b"\x00" * 7,
            b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
        ])
        prompt = Mock(side_effect=["green blinking every 2 seconds",
                                   waveshare_motor.LED_ENABLE_CONFIRMATION,
                                   "green solid"])
        with redirect_stdout(io.StringIO()) as output:
            result = waveshare_motor.run_led_state_test(motor, prompt)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x9A, 0x9C, 0x81, 0x80, 0x88, 0x81, 0x80])
        self.assertTrue(all(len(call.args) == 1 for call in motor.exchange.call_args_list))
        self.assertEqual(result["protection_state"], "UNKNOWN")
        self.assertEqual(result["enabled_led_observation"], "green solid")
        self.assertIn("green blinking every 2 seconds", output.getvalue())
        self.assertIn("green solid", output.getvalue())
        self.assertEqual(prompt.call_args_list[-1].args[1], 20)

    def test_led_state_test_blocks_fault_nonzero_speed_and_refused_confirmation(self):
        status = bytes.fromhex("9A 1F 30 0B 00 00 20 00")
        for first, second, prompt_answers, error in (
                (bytes.fromhex("9A 1F 30 0B 00 00 20 08"),
                 bytes.fromhex("9C 1F 00 00 00 00 00 00"), [], ControllerFault),
                (status, bytes.fromhex("9C 1F 00 00 01 00 00 00"),
                 [], CommunicationError),
                (status, bytes.fromhex("9C 1F 00 00 00 00 00 00"),
                 ["green blinking", "no"], CommunicationError)):
            with self.subTest(error=error, answers=prompt_answers):
                motor = waveshare_motor.WaveshareMotor("unused", 1)
                motor.exchange = Mock(side_effect=[first, second] +
                                      [b"\x81" + b"\x00" * 7,
                                       b"\x80" + b"\x00" * 7] *
                                      (2 if prompt_answers else 1))
                prompt = Mock(side_effect=prompt_answers)
                with redirect_stdout(io.StringIO()), self.assertRaises(error):
                    waveshare_motor.run_led_state_test(motor, prompt)
                commands = [call.args[0] for call in motor.exchange.call_args_list]
                self.assertNotIn(0x88, commands)
                self.assertEqual(commands[-2:], [0x81, 0x80])

    def test_led_state_test_timeout_or_interrupt_attempts_cleanup(self):
        for failure in (ResponseTimeout("no ENABLE response"), KeyboardInterrupt()):
            motor = waveshare_motor.WaveshareMotor("unused", 1)
            motor.exchange = Mock(side_effect=[
                bytes.fromhex("9A 1F 30 0B 00 00 20 00"),
                bytes.fromhex("9C 1F 00 00 00 00 00 00"),
                b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
                failure,
                b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
            ])
            prompt = Mock(side_effect=["green blinking", waveshare_motor.LED_ENABLE_CONFIRMATION])
            with redirect_stdout(io.StringIO()), self.assertRaises(type(failure)):
                waveshare_motor.run_led_state_test(motor, prompt)
            self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                             [0x9A, 0x9C, 0x81, 0x80, 0x88, 0x81, 0x80])

    def test_led_observation_timeout_after_enable_attempts_cleanup(self):
        motor = waveshare_motor.WaveshareMotor("unused", 1)
        motor.exchange = Mock(side_effect=[
            bytes.fromhex("9A 1F 30 0B 00 00 20 00"),
            bytes.fromhex("9C 1F 00 00 00 00 00 00"),
            b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
            b"\x88" + b"\x00" * 7,
            b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
        ])
        prompt = Mock(side_effect=["green blinking", waveshare_motor.LED_ENABLE_CONFIRMATION,
                                   CommunicationError("observation timed out")])
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(CommunicationError, "timed out"):
            waveshare_motor.run_led_state_test(motor, prompt)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x9A, 0x9C, 0x81, 0x80, 0x88, 0x81, 0x80])

    def test_led_cli_requires_terminal_before_opening_adapter(self):
        with patch.object(waveshare_motor, "WaveshareMotor",
                          side_effect=AssertionError("opened adapter")), \
             patch.object(waveshare_motor.sys.stdin, "isatty", return_value=False), \
             redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            waveshare_motor.main(["--mode", "led-state-test"])

    def test_led_state_test_verified_fault_blocks_enable(self):
        motor = waveshare_motor.WaveshareMotor("unused", 1)
        motor.exchange = Mock(side_effect=[
            bytes.fromhex("9A 1F 30 0B 00 00 20 00"),
            bytes.fromhex("9C 1F 00 00 00 00 00 00"),
            b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
        ])
        fault = ProtectionAssessment(ProtectionState.VERIFIED_FAULT, "simulated active fault")
        with patch.object(waveshare_motor, "assess_mhf9025_v3_protection", return_value=fault), \
             redirect_stdout(io.StringIO()), self.assertRaises(ControllerFault):
            waveshare_motor.run_led_state_test(motor, Mock())
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x9A, 0x9C, 0x81, 0x80])

    def test_led_log_records_frames_and_observations_without_hardware(self):
        replies = iter([
            bytes.fromhex("9A 1F 30 0B 00 00 20 00"),
            bytes.fromhex("9C 1F 00 00 00 00 00 00"),
            b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
            b"\x88" + b"\x00" * 7,
            b"\x81" + b"\x00" * 7, b"\x80" + b"\x00" * 7,
        ])
        motor = waveshare_motor.WaveshareMotor("unused", 1)

        def fake_exchange(command):
            reply = next(replies)
            print(f"TX 0x141: {command:02X} 00 00 00 00 00 00 00")
            print(f"RX 0x141: {reply.hex(' ').upper()}")
            return reply

        motor.exchange = Mock(side_effect=fake_exchange)
        context = Mock()
        context.__enter__ = Mock(return_value=motor)
        context.__exit__ = Mock(return_value=None)
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(waveshare_motor, "LED_LOG_DIR", pathlib.Path(directory)), \
             patch.object(waveshare_motor, "WaveshareMotor", return_value=context), \
             patch.object(waveshare_motor, "_led_prompt", side_effect=[
                 waveshare_motor.LED_HARDWARE_CONFIRMATION, "green blinking",
                 waveshare_motor.LED_ENABLE_CONFIRMATION, "green solid"]), \
             redirect_stdout(io.StringIO()):
            waveshare_motor.run_logged_led_state_test(SimpleNamespace(
                device="/dev/ttyUSB0", motor_id=1, can_bitrate=500_000,
                serial_baud=2_000_000, timeout=0.5))
            log = next(pathlib.Path(directory).glob("led_state_*.log")).read_text()
        self.assertIn("TX 0x141: 88 00 00 00 00 00 00 00", log)
        self.assertIn("RX 0x141: 88 00 00 00 00 00 00 00", log)
        self.assertIn("user disabled LED observation: green blinking", log)
        self.assertIn("user enabled LED observation: green solid", log)
        self.assertIn("hardware confirmation received", log)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x9A, 0x9C, 0x81, 0x80, 0x88, 0x81, 0x80])

    def test_five_rpm_frame(self):
        self.assertEqual(
            waveshare_motor.serial_can_frame(1, 0xA2,
                                             waveshare_motor.speed_payload(5)).hex(" "),
            "aa c8 41 01 a2 00 00 00 b8 0b 00 00 55")

    def test_live_status_layout_blocks_enable(self):
        motor = Mock()
        motor.motor_id = 1
        motor.exchange.side_effect = [bytes.fromhex("9A 20 32 0B 00 00 30 00"),
                                      bytes.fromhex("9C 20 03 00 00 00 14 32")]
        with self.assertRaisesRegex(CommunicationError, "protection state UNKNOWN"):
            waveshare_motor.run_motion(motor, 5, 5)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x9A, 0x9C])
        motor.stop_and_disable.assert_not_called()

    def test_speed_timeout_attempts_cleanup(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      ResponseTimeout("speed response missing")]
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=24.0), Mock(speed_dps=0), MOCK_VERIFIED)):
            with self.assertRaises(ResponseTimeout):
                waveshare_motor.run_motion(motor, 5, 5)
        motor.stop_and_disable.assert_called_once()

    def test_reverse_speed_feedback_attempts_stop(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      bytes.fromhex("A2 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 E2 FF 00 00")]
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=24.0), Mock(speed_dps=0), MOCK_VERIFIED)):
            with self.assertRaisesRegex(CommunicationError, "abnormal speed"):
                waveshare_motor.run_motion(motor, 5, 5)
        motor.stop_and_disable.assert_called_once()

    def test_unknown_fault_bytes_remain_unknown(self):
        motor = Mock()
        motor.motor_id = 1
        motor.exchange.side_effect = [bytes.fromhex("9A 19 32 0B 00 00 30 08"),
                                      bytes.fromhex("9C 19 00 00 00 00 00 00")]
        with self.assertRaisesRegex(CommunicationError, "protection state UNKNOWN"):
            waveshare_motor.run_motion(motor, 5, 5)
        self.assertEqual(motor.exchange.call_count, 2)
        motor.stop_and_disable.assert_not_called()

    def test_status_only_reports_voltage_without_enabling(self):
        motor = Mock()
        motor.motor_id = 1
        motor.exchange.side_effect = [bytes.fromhex("9A 21 32 0B 00 00 30 00"),
                                      bytes.fromhex("9C 21 FD FF 00 00 34 32")]
        status, feedback, assessment = waveshare_motor.check_status(
            motor, require_motion_ready=False)
        self.assertEqual(status.voltage_v, 28.66)
        self.assertIsNone(status.fault_state)
        self.assertEqual(assessment.state, ProtectionState.UNKNOWN)
        self.assertIn("byte6=0x30", assessment.evidence)
        self.assertEqual(feedback.speed_dps, 0)
        self.assertEqual(motor.exchange.call_count, 2)

    def test_status_only_reports_small_nonzero_speed(self):
        motor = Mock()
        motor.motor_id = 1
        motor.exchange.side_effect = [bytes.fromhex("9A 23 32 0B 00 00 20 00"),
                                      bytes.fromhex("9C 23 04 00 03 00 32 32")]
        _, feedback, assessment = waveshare_motor.check_status(
            motor, require_motion_ready=False)
        self.assertEqual(feedback.speed_dps, 3)
        self.assertEqual(assessment.state, ProtectionState.UNKNOWN)
        self.assertEqual(motor.exchange.call_count, 2)

    def test_nonzero_preflight_speed_blocks_enable_even_if_faults_verified(self):
        motor = Mock()
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.66),
                                        Mock(speed_dps=3), MOCK_VERIFIED)):
            with self.assertRaisesRegex(CommunicationError, "not confirmed stopped"):
                waveshare_motor.run_motion(motor, 5, 2)
        motor.exchange.assert_not_called()

    def test_fault_report_keeps_8bit_and_16bit_hypotheses_unknown(self):
        report = waveshare_motor.build_fault_report(
            bytes.fromhex("9A 23 32 0B 00 00 30 00"),
            bytes.fromhex("9C 23 FE FF 00 00 2D 32"), 0x141)
        self.assertEqual(report["status_9a"]["voltage_v"], 28.66)
        self.assertEqual(report["status_9a"]["bytes"][6:], ["0x30", "0x00"])
        self.assertEqual(report["status_9c"]["speed_raw"], 0)
        hypotheses = report["fault_field_hypotheses"]
        self.assertEqual(hypotheses[0]["raw_value"], "0x00")
        self.assertEqual(hypotheses[1]["raw_value"], "0x0030")
        self.assertEqual(hypotheses[0]["assessment"], "UNKNOWN")
        self.assertEqual(hypotheses[1]["assessment"], "UNKNOWN")
        self.assertEqual(report["protection"]["state"], "UNKNOWN")
        self.assertFalse(report["motion_ready"])

    def test_new_live_byte6_value_remains_unknown(self):
        report = waveshare_motor.build_fault_report(
            bytes.fromhex("9A 23 32 0B 00 00 20 00"),
            bytes.fromhex("9C 23 04 00 03 00 32 32"), 0x141)
        self.assertEqual(report["fault_field_hypotheses"][0]["raw_value"], "0x00")
        self.assertEqual(report["fault_field_hypotheses"][1]["raw_value"], "0x0020")
        self.assertEqual(report["fault_field_hypotheses"][2]["raw_value"], "0x20")
        self.assertEqual(report["status_9c"]["speed_raw"], 3)
        self.assertEqual(report["protection"]["state"], "UNKNOWN")

    def test_fault_diagnostic_sends_only_two_read_requests(self):
        motor = Mock()
        motor.motor_id = 1
        motor.can_bitrate = 500_000
        motor.exchange.side_effect = [bytes.fromhex("9A 23 32 0B 00 00 30 00"),
                                      bytes.fromhex("9C 23 FE FF 00 00 2D 32")]
        with redirect_stdout(io.StringIO()) as output:
            report = waveshare_motor.run_fault_diagnostic(motor)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x9A, 0x9C])
        self.assertIn('"state": "UNKNOWN"', output.getvalue())
        self.assertEqual(report["status_9a"]["raw_hex"], "9A 23 32 0B 00 00 30 00")

    def test_extended_read_frames_and_decoders(self):
        cases = (
            (0x30, "30 00 01 02 03 04 05 06", "torque_ki_raw", 6),
            (0x33, "33 00 00 00 38 FF FF FF", "acceleration_raw", -200),
            (0x90, "90 00 34 12 78 56 BC 9A", "encoder_offset_raw", 0x9ABC),
            (0x92, "92 9C FF FF FF FF FF FF", "angle_degrees_provisional", -1.0),
            (0x9D, "9D E0 40 00 C0 FF 80 00", "phase_b_current_a_provisional", -1.0),
        )
        for command, response_hex, field, expected in cases:
            with self.subTest(command=command):
                self.assertEqual(waveshare_motor.serial_can_frame(1, command),
                                 bytes.fromhex(f"AA C8 41 01 {command:02X} " +
                                               "00 " * 7 + "55"))
                decoded = waveshare_motor.decode_extended_read(
                    command, bytes.fromhex(response_hex))
                self.assertEqual(decoded[field], expected)
        self.assertEqual(waveshare_motor.decode_extended_read(
            0x9D, bytes.fromhex("9D E0 40 00 C0 FF 80 00"))["temperature_c_provisional"], -32)

    def test_extended_read_only_sends_five_fixed_commands_once(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [
            bytes.fromhex("30 00 01 02 03 04 05 06"),
            bytes.fromhex("33 00 00 00 38 FF FF FF"),
            bytes.fromhex("90 00 34 12 78 56 BC 9A"),
            bytes.fromhex("92 9C FF FF FF FF FF FF"),
            bytes.fromhex("9D E0 40 00 C0 FF 80 00"),
        ]
        with redirect_stdout(io.StringIO()) as output:
            results = waveshare_motor.run_extended_read_diagnostic(motor)
        self.assertEqual([call.args for call in motor.exchange.call_args_list],
                         [(command, b"\x00" * 7) for command, _ in
                          waveshare_motor.EXTENDED_READ_COMMANDS])
        self.assertTrue(all(result["status"] == "ok" for result in results))
        self.assertTrue(all("unverified for MHF9025-V3" in result["interpretation"]
                            for result in results))
        self.assertIn('"rx_data_hex": "92 9C FF FF FF FF FF FF"', output.getvalue())
        motor.stop_and_disable.assert_not_called()

    def test_extended_read_timeout_does_not_retry_or_send_control(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [ResponseTimeout("no 0x30 response within 0.5s"),
                                      bytes.fromhex("33 00 00 00 64 00 00 00"),
                                      bytes.fromhex("90 00 01 00 02 00 03 00"),
                                      bytes.fromhex("92 00 00 00 00 00 00 00"),
                                      bytes.fromhex("9D 20 00 00 00 00 00 00")]
        with redirect_stdout(io.StringIO()):
            results = waveshare_motor.run_extended_read_diagnostic(motor)
        self.assertEqual(results[0]["status"], "timeout")
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x30, 0x33, 0x90, 0x92, 0x9D])
        motor.stop_and_disable.assert_not_called()

    def test_extended_read_rejects_malformed_reply_and_wrong_bus(self):
        for command, _ in waveshare_motor.EXTENDED_READ_COMMANDS:
            for reply in (bytes((command,)) + b"\x00" * 6,
                          bytes((command ^ 1,)) + b"\x00" * 7):
                with self.assertRaises(CommunicationError):
                    waveshare_motor.decode_extended_read(command, reply)
        motor = Mock(motor_id=2, can_bitrate=500_000)
        with self.assertRaises(ValueError):
            waveshare_motor.run_extended_read_diagnostic(motor)
        motor.exchange.assert_not_called()

    def test_extended_read_cli_dispatch_and_timeout_exit(self):
        replies = [bytes.fromhex("30 00 01 02 03 04 05 06"),
                   ResponseTimeout("no 0x33 response within 0.5s"),
                   bytes.fromhex("90 00 01 00 02 00 03 00"),
                   bytes.fromhex("92 00 00 00 00 00 00 00"),
                   bytes.fromhex("9D 20 00 00 00 00 00 00")]
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = replies
        context = Mock()
        context.__enter__ = Mock(return_value=motor)
        context.__exit__ = Mock(return_value=None)
        with patch.object(waveshare_motor, "WaveshareMotor", return_value=context) as motor_class:
            with redirect_stdout(io.StringIO()):
                status = waveshare_motor.main([
                    "--mode", "extended-read-diagnostic", "--device", "/dev/ttyUSB0",
                    "--can-bitrate", "500000", "--motor-id", "1", "--timeout", "0.5"])
        self.assertEqual(status, 1)
        motor_class.assert_called_once_with("/dev/ttyUSB0", 1, 500_000, 2_000_000, 0.5)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x30, 0x33, 0x90, 0x92, 0x9D])
        motor.stop_and_disable.assert_not_called()

    def test_fault_report_rejects_malformed_responses(self):
        status = bytes.fromhex("9A 23 32 0B 00 00 30 00")
        speed = bytes.fromhex("9C 23 FE FF 00 00 2D 32")
        for bad_status in (status[:-1], b"\x9C" + status[1:]):
            with self.assertRaises(CommunicationError):
                waveshare_motor.build_fault_report(bad_status, speed, 0x141)
        for bad_speed in (speed[:-1], b"\x9A" + speed[1:]):
            with self.assertRaises(CommunicationError):
                waveshare_motor.build_fault_report(status, bad_speed, 0x141)

    def test_protection_classification_gates_ready(self):
        with self.assertRaisesRegex(CommunicationError, "UNKNOWN"):
            waveshare_motor.require_protection_clear(
                ProtectionAssessment(ProtectionState.UNKNOWN, "no field mapping"))
        with self.assertRaisesRegex(ControllerFault, "verified motor protection fault"):
            waveshare_motor.require_protection_clear(
                ProtectionAssessment(ProtectionState.VERIFIED_FAULT, "simulated fault"))
        waveshare_motor.require_protection_clear(MOCK_VERIFIED)

    def test_verified_fault_prevents_enable_transition(self):
        motor = Mock()
        fault = ProtectionAssessment(ProtectionState.VERIFIED_FAULT,
                                     "simulated verified fault for software test")
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.66), Mock(speed_dps=0), fault)):
            with self.assertRaises(ControllerFault):
                waveshare_motor.run_motion(motor, 5, 2)
        motor.exchange.assert_not_called()
        motor.stop_and_disable.assert_not_called()

    def test_motion_state_machine_order(self):
        flow = waveshare_motor.MotionStateMachine()
        with self.assertRaisesRegex(RuntimeError, "invalid motion-state transition"):
            flow.transition(waveshare_motor.MotionState.ENABLE)
        for state in waveshare_motor.MotionStateMachine.ORDER[1:]:
            flow.transition(state)
        self.assertEqual(flow.state, waveshare_motor.MotionState.COMPLETE)

    def test_interrupt_during_motion_attempts_cleanup(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      KeyboardInterrupt()]
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=24.0), Mock(speed_dps=0), MOCK_VERIFIED)):
            with self.assertRaises(KeyboardInterrupt):
                waveshare_motor.run_motion(motor, 5, 2)
        motor.stop_and_disable.assert_called_once()

    def test_stop_confirmation_needs_two_consecutive_samples(self):
        motor = Mock()
        motor.exchange.side_effect = [bytes.fromhex("9C 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 02 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 00 00 00 00")]
        with patch.object(waveshare_motor.time, "sleep"):
            waveshare_motor.confirm_stopped(motor)
        self.assertEqual(motor.exchange.call_count, 4)

    def test_stop_confirmation_fails_without_two_zeros(self):
        motor = Mock()
        motor.exchange.return_value = bytes.fromhex("9C 19 00 00 02 00 00 00")
        with patch.object(waveshare_motor.time, "sleep"):
            with self.assertRaisesRegex(CommunicationError, "two consecutive"):
                waveshare_motor.confirm_stopped(motor)
        self.assertEqual(motor.exchange.call_count, 4)

    def test_complete_mock_motion_reaches_complete_state(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      bytes.fromhex("A2 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 1E 00 01 00"),
                                      b"\x81" + b"\x00" * 7,
                                      bytes.fromhex("9C 19 00 00 00 00 02 00"),
                                      bytes.fromhex("9C 19 00 00 00 00 02 00")]
        output = io.StringIO()
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.66), Mock(speed_dps=0), MOCK_VERIFIED)), \
             redirect_stdout(output):
            waveshare_motor.run_motion(motor, 5, 0.05)
        self.assertIn("state: COMPLETE", output.getvalue())
        self.assertIn("rotation: observed", output.getvalue())
        self.assertIn("stopped: two consecutive", output.getvalue())
        motor.stop_and_disable.assert_called_once()

    def test_cleanup_attempts_disable_after_stop_timeout(self):
        motor = waveshare_motor.WaveshareMotor("unused", 1)
        motor.exchange = Mock(side_effect=[ResponseTimeout("stop timeout"),
                                           b"\x80" + b"\x00" * 7])
        with redirect_stderr(io.StringIO()):
            self.assertFalse(motor.stop_and_disable())
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x81, 0x80])

    def test_missing_motion_feedback_attempts_cleanup(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      bytes.fromhex("A2 19 00 00 00 00 00 00"),
                                      ResponseTimeout("0x9C timeout")]
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=24.0), Mock(speed_dps=0), MOCK_VERIFIED)):
            with self.assertRaises(ResponseTimeout):
                waveshare_motor.run_motion(motor, 5, 2)
        motor.stop_and_disable.assert_called_once()

    def test_write_timeout_is_bounded(self):
        with patch.object(waveshare_motor.select, "select", return_value=([], [], [])):
            with self.assertRaisesRegex(CommunicationError, "write timed out"):
                waveshare_motor.write_with_timeout(7, b"request", 0.1)

    def test_default_duration_and_dry_run_never_open_adapter(self):
        output = io.StringIO()
        with patch.object(waveshare_motor, "WaveshareMotor",
                          side_effect=AssertionError("opened adapter")), redirect_stdout(output):
            self.assertEqual(waveshare_motor.main(["--mode", "dry-run", "--speed", "5"]), 0)
        self.assertIn("for 2s each", output.getvalue())
        self.assertIn("A2 00 00 00 B8 0B 00 00", output.getvalue())
        self.assertIn("A2 00 00 00 48 F4 FF FF", output.getvalue())

    def test_experimental_requires_flag_without_opening_adapter(self):
        with patch.object(waveshare_motor, "WaveshareMotor",
                          side_effect=AssertionError("opened adapter")), \
             redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            waveshare_motor.main(["--mode", "experimental-motion", "--duration", "1"])

    def test_experimental_stationary_check_accepts_observed_small_jitter(self):
        for first_frame in ("9C 20 FD FF 03 00 51 3B",
                            "9C 20 FD FF FB FF 51 3B"):
            motor = Mock()
            initial = decode_feedback(bytes.fromhex(first_frame))
            motor.exchange.side_effect = [bytes.fromhex("9C 20 FD FF 00 00 51 3B"),
                                          bytes.fromhex("9C 20 FD FF 01 00 52 3B"),
                                          bytes.fromhex("9C 20 FD FF 00 00 52 3B"),
                                          bytes.fromhex("9C 20 FD FF 02 00 52 3B")]
            with patch.object(waveshare_motor.time, "sleep"), redirect_stdout(io.StringIO()) as output:
                waveshare_motor.confirm_experimental_stationary(motor, initial)
            self.assertIn("stable encoder", output.getvalue())
            self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                             [0x9C] * 4)

    def test_experimental_stationary_check_blocks_real_movement_indicators(self):
        for first_frame, next_frame in (
                ("9C 20 FD FF 03 00 51 3B", "9C 20 FD FF 04 00 51 3B"),
                ("9C 20 FD FF 03 00 51 3B", "9C 20 FD FF 00 00 58 3B"),
                ("9C 20 FD FF FB FF 51 3B", "9C 20 FD FF FB FF 51 3B")):
            motor = Mock()
            initial = decode_feedback(bytes.fromhex(first_frame))
            motor.exchange.return_value = bytes.fromhex(next_frame)
            with patch.object(waveshare_motor.time, "sleep"), \
                 self.assertRaisesRegex(CommunicationError, "not confirmed stopped"):
                waveshare_motor.confirm_experimental_stationary(motor, initial)

    def test_experimental_motion_with_observed_three_dps_preflight_reaches_enable(self):
        motor = Mock()
        initial = decode_feedback(bytes.fromhex("9C 20 FD FF 03 00 51 3B"))
        motor.exchange.side_effect = [bytes.fromhex("9C 20 FD FF 00 00 51 3B"),
                                      bytes.fromhex("9C 20 FD FF 01 00 52 3B"),
                                      bytes.fromhex("9C 20 FD FF 00 00 52 3B"),
                                      bytes.fromhex("9C 20 FD FF 02 00 52 3B"),
                                      b"\x88" + b"\x00" * 7,
                                      bytes.fromhex("A2 20 FD FF 00 00 52 3B"),
                                      bytes.fromhex("9C 20 FD FF 1E 00 60 3B"),
                                      b"\x81" + b"\x00" * 7,
                                      bytes.fromhex("9C 20 FD FF 03 00 61 3B"),
                                      bytes.fromhex("9C 20 FD FF 00 00 61 3B"),
                                      bytes.fromhex("9C 20 FD FF 02 00 62 3B"),
                                      bytes.fromhex("92 00 00 00 00 00 00 00"),
                                      bytes.fromhex("9D 20 00 00 00 00 00 00")]
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.66, temperature_c=32),
                                        initial, MOCK_UNKNOWN)), \
             patch.object(waveshare_motor, "confirm_experimental_motion"), \
             redirect_stdout(io.StringIO()):
            waveshare_motor.run_experimental_motion(motor, 5, 0.05)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x9C, 0x9C, 0x9C, 0x9C, 0x88, 0xA2, 0x9C, 0x81,
                          0x9C, 0x9C, 0x9C, 0x92, 0x9D])
        motor.stop_and_disable.assert_called_once()

    def test_experimental_stop_accepts_small_speed_jitter_only_with_stable_encoder(self):
        motor = Mock()
        motor.exchange.side_effect = [bytes.fromhex("9C 20 FD FF 03 00 51 3B"),
                                      bytes.fromhex("9C 20 FD FF 00 00 51 3B"),
                                      bytes.fromhex("9C 20 FD FF 02 00 52 3B")]
        with patch.object(waveshare_motor.time, "sleep"):
            waveshare_motor.confirm_stopped(motor, experimental=True)
        motor = Mock()
        motor.exchange.side_effect = [bytes.fromhex("9C 20 FD FF 03 00 51 3B"),
                                      bytes.fromhex("9C 20 FD FF 00 00 58 3B"),
                                      bytes.fromhex("9C 20 FD FF 02 00 60 3B"),
                                      bytes.fromhex("9C 20 FD FF 01 00 65 3B")]
        with patch.object(waveshare_motor.time, "sleep"), \
             self.assertRaisesRegex(CommunicationError, "stable encoder"):
            waveshare_motor.confirm_stopped(motor, experimental=True)

    def test_experimental_bounds_and_noninteractive_terminal(self):
        for extra in (["--speed", "5.1"], ["--duration", "2.1"]):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                waveshare_motor.main(["--mode", "experimental-motion",
                                      "--allow-experimental-motion", *extra])
        with patch.object(waveshare_motor.sys.stdin, "isatty", return_value=False), \
             redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            waveshare_motor.main(["--mode", "experimental-motion",
                                  "--allow-experimental-motion"])

    def test_interactive_confirmation_requires_exact_phrase(self):
        with patch.object(waveshare_motor.sys.stdin, "isatty", return_value=True), \
             patch.object(waveshare_motor.sys.stdout, "isatty", return_value=True), \
             patch("builtins.input", return_value="no"), \
             self.assertRaisesRegex(CommunicationError, "confirmation declined"):
            waveshare_motor.confirm_experimental_motion()
        with patch.object(waveshare_motor.sys.stdin, "isatty", return_value=True), \
             patch.object(waveshare_motor.sys.stdout, "isatty", return_value=True), \
             patch("builtins.input", return_value=waveshare_motor.EXPERIMENT_CONFIRMATION):
            waveshare_motor.confirm_experimental_motion()

    def test_experimental_unknown_can_reach_bounded_mock_motion(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      bytes.fromhex("A2 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 1E 00 01 00"),
                                      b"\x81" + b"\x00" * 7,
                                      bytes.fromhex("9C 19 00 00 00 00 02 00"),
                                      bytes.fromhex("9C 19 00 00 00 00 02 00"),
                                      bytes.fromhex("9C 19 00 00 00 00 02 00"),
                                      bytes.fromhex("92 00 00 00 00 00 00 00"),
                                      bytes.fromhex("9D 20 00 00 00 00 00 00")]
        output = io.StringIO()
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.66, temperature_c=34),
                                        Mock(speed_dps=0), MOCK_UNKNOWN)), \
             patch.object(waveshare_motor, "confirm_experimental_motion"), \
             patch.object(waveshare_motor, "confirm_experimental_stationary"), \
             redirect_stdout(output):
            waveshare_motor.run_experimental_motion(motor, 5, 0.05)
        commands = [call.args[0] for call in motor.exchange.call_args_list]
        self.assertEqual(commands, [0x88, 0xA2, 0x9C, 0x81, 0x9C, 0x9C, 0x9C,
                                    0x92, 0x9D])
        self.assertEqual(motor.exchange.call_args_list[1].args[1],
                         bytes.fromhex("00 00 00 B8 0B 00 00"))
        self.assertGreater(motor.exchange.call_args_list[1].kwargs["timeout"], 0)
        self.assertLessEqual(motor.exchange.call_args_list[1].kwargs["timeout"], 0.05)
        self.assertGreater(motor.exchange.call_args_list[2].kwargs["timeout"], 0)
        self.assertLessEqual(motor.exchange.call_args_list[2].kwargs["timeout"], 0.05)
        self.assertIn("protection=UNKNOWN", output.getvalue())
        self.assertIn("zero_speed_confirmed=True", output.getvalue())
        motor.stop_and_disable.assert_called_once()

    def test_experimental_preflight_blocks_bad_voltage_temperature_and_fault(self):
        fault = ProtectionAssessment(ProtectionState.VERIFIED_FAULT, "mock active fault")
        for status, assessment, expected in (
                (Mock(voltage_v=12, temperature_c=34), MOCK_UNKNOWN, CommunicationError),
                (Mock(voltage_v=28.66, temperature_c=90), MOCK_UNKNOWN, CommunicationError),
                (Mock(voltage_v=28.66, temperature_c=34), fault, ControllerFault)):
            motor = Mock()
            with patch.object(waveshare_motor, "check_status",
                              return_value=(status, Mock(speed_dps=0), assessment)), \
                 patch.object(waveshare_motor, "confirm_experimental_motion") as confirm, \
                 self.assertRaises(expected):
                waveshare_motor.run_experimental_motion(motor, 5, 1)
            confirm.assert_not_called()
            self.assertNotIn(0x88, [call.args[0] for call in motor.exchange.call_args_list])

    def test_experimental_legacy_fault_candidate_blocks_enable(self):
        motor = Mock()
        motor.motor_id = 1
        motor.exchange.side_effect = [bytes.fromhex("9A 20 30 0B 00 00 20 08"),
                                      bytes.fromhex("9C 20 00 00 00 00 00 00")]
        with redirect_stdout(io.StringIO()), \
             self.assertRaisesRegex(ControllerFault, "fault candidate bits"):
            waveshare_motor.run_experimental_motion(motor, 5, 1)
        self.assertNotIn(0x88, [call.args[0] for call in motor.exchange.call_args_list])

    def test_experimental_cli_defaults_to_one_second_without_hardware(self):
        with patch.object(waveshare_motor.sys.stdin, "isatty", return_value=True), \
             patch.object(waveshare_motor.sys.stdout, "isatty", return_value=True), \
             patch.object(waveshare_motor, "run_logged_experiment") as run:
            self.assertEqual(waveshare_motor.main([
                "--mode", "experimental-motion", "--allow-experimental-motion"]), 0)
        self.assertEqual(run.call_args.args[0].duration, 1.0)

    def test_experimental_preflight_communication_failure_attempts_cleanup(self):
        motor = Mock()
        with patch.object(waveshare_motor, "check_status",
                          side_effect=ResponseTimeout("0x9A missing")), \
             self.assertRaises(ResponseTimeout):
            waveshare_motor.run_experimental_motion(motor, 5, 1)
        motor.stop_and_disable.assert_called_once()

    def test_experimental_monitor_timeout_and_interrupt_attempt_cleanup(self):
        for failure in (ResponseTimeout("0x9C timeout"), KeyboardInterrupt(),
                        InterruptedError("SIGTERM received")):
            motor = Mock()
            motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                          bytes.fromhex("A2 19 00 00 00 00 00 00"), failure]
            with patch.object(waveshare_motor, "check_status",
                              return_value=(Mock(voltage_v=28.66, temperature_c=34),
                                            Mock(speed_dps=0), MOCK_UNKNOWN)), \
                 patch.object(waveshare_motor, "confirm_experimental_motion"), \
                 patch.object(waveshare_motor, "confirm_experimental_stationary"), \
                 self.assertRaises(type(failure)):
                waveshare_motor.run_experimental_motion(motor, 5, 1)
            motor.stop_and_disable.assert_called_once()

    def test_experimental_abnormal_speed_attempts_cleanup(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      bytes.fromhex("A2 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 24 00 00 00")]
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.66, temperature_c=34),
                                        Mock(speed_dps=0), MOCK_UNKNOWN)), \
             patch.object(waveshare_motor, "confirm_experimental_motion"), \
             patch.object(waveshare_motor, "confirm_experimental_stationary"), \
             self.assertRaisesRegex(CommunicationError, "abnormal speed"):
            waveshare_motor.run_experimental_motion(motor, 5, 1)
        motor.stop_and_disable.assert_called_once()

    def test_experimental_abort_checks_zero_speed_and_reads_status_after_stop(self):
        motor = waveshare_motor.WaveshareMotor("unused", 1)
        motor.exchange = Mock(side_effect=[
            b"\x88" + b"\x00" * 7,
            bytes.fromhex("A2 20 05 00 00 00 6A 3B"),
            bytes.fromhex("9C 20 00 00 30 00 96 3F"),
            b"\x81" + b"\x00" * 7,
            b"\x80" + b"\x00" * 7,
            bytes.fromhex("9C 20 00 00 00 00 96 3F"),
            bytes.fromhex("9C 20 00 00 01 00 96 3F"),
            bytes.fromhex("9C 20 00 00 00 00 96 3F"),
            bytes.fromhex("9A 20 30 0B 00 00 30 00"),
            bytes.fromhex("92 49 2C 00 00 00 00 00"),
            bytes.fromhex("9D 1F FE FF 00 00 02 00")])
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.64, temperature_c=32),
                                        Mock(speed_dps=0), MOCK_UNKNOWN)), \
             patch.object(waveshare_motor, "confirm_experimental_stationary"), \
             patch.object(waveshare_motor, "confirm_experimental_motion"), \
             redirect_stdout(io.StringIO()) as output, \
             self.assertRaisesRegex(CommunicationError, "abnormal speed"):
            waveshare_motor.run_experimental_motion(motor, 5, 1)
        self.assertIn("post-abort status: raw=9A 20 30 0B 00 00 30 00", output.getvalue())
        self.assertIn("stop: acknowledged", output.getvalue())
        self.assertIn("disable: acknowledged", output.getvalue())
        self.assertIn("zero_speed_confirmed=True", output.getvalue())
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [0x88, 0xA2, 0x9C, 0x81, 0x80, 0x9C, 0x9C, 0x9C, 0x9A,
                          0x92, 0x9D])

    def test_experimental_timed_stop_does_not_require_observed_rotation(self):
        motor = Mock()
        motor.exchange.side_effect = [b"\x88" + b"\x00" * 7,
                                      bytes.fromhex("A2 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 03 00 00 00"),
                                      b"\x81" + b"\x00" * 7,
                                      bytes.fromhex("9C 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 00 00 00 00"),
                                      bytes.fromhex("9C 19 00 00 00 00 00 00")]
        with patch.object(waveshare_motor, "check_status",
                          return_value=(Mock(voltage_v=28.66, temperature_c=34),
                                        Mock(speed_dps=0), MOCK_UNKNOWN)), \
             patch.object(waveshare_motor, "confirm_experimental_motion"), \
             patch.object(waveshare_motor, "confirm_experimental_stationary"), \
             redirect_stdout(io.StringIO()) as output, \
             self.assertRaisesRegex(CommunicationError, "motor-reported speed threshold not confirmed"):
            waveshare_motor.run_experimental_motion(motor, 5, 0.05)
        self.assertIn(0x81, [call.args[0] for call in motor.exchange.call_args_list])
        self.assertIn("zero_speed_confirmed=True", output.getvalue())
        motor.stop_and_disable.assert_called_once()

    def test_experiment_log_records_run_inside_project_log_directory(self):
        args = SimpleNamespace(motor_id=1, can_bitrate=500000, speed=5,
                               duration=1, device="mock", serial_baud=2000000,
                               timeout=0.5)
        with tempfile.TemporaryDirectory() as temporary, \
             patch.object(waveshare_motor, "EXPERIMENT_LOG_DIR", pathlib.Path(temporary)), \
             patch.object(waveshare_motor, "WaveshareMotor"), \
             patch.object(waveshare_motor, "run_experimental_motion",
                          side_effect=lambda *_: print("123.456 TX 0x141: A2 00 00 00 B8 0B 00 00")), \
             redirect_stdout(io.StringIO()):
            waveshare_motor.run_logged_experiment(args)
            logs = list(pathlib.Path(temporary).glob("experimental_*.log"))
            self.assertEqual(len(logs), 1)
            data = logs[0].read_text()
            self.assertIn("CAN_ID=0x141", data)
            self.assertIn("123.456 TX 0x141", data)
            self.assertIn("experiment CAN sequence completed:", data)
            self.assertIn("physical shaft rotation NOT VERIFIED", data)


if __name__ == "__main__":
    unittest.main()
