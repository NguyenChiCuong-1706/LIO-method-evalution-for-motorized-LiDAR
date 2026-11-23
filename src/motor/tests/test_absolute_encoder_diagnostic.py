import csv
from datetime import datetime, timezone
import io
import pathlib
import sys
import tempfile
import unittest
from contextlib import nullcontext
from contextlib import redirect_stdout
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import absolute_encoder_diagnostic as diagnostic
from motor_can import CommunicationError, ResponseTimeout


class AbsoluteEncoderDiagnosticTests(unittest.TestCase):
    @staticmethod
    def single_frame(degrees):
        raw = round(degrees * 100) % 36_000
        return bytes((0x94, 0, 0, 0)) + raw.to_bytes(4, "little")

    @staticmethod
    def multi_frame(degrees):
        raw = round(degrees * 100)
        return bytes((0x92,)) + raw.to_bytes(7, "little", signed=True)

    @staticmethod
    def state_frame(speed_dps=0, encoder=0):
        return (bytes((0x9C, 25)) + (0).to_bytes(2, "little", signed=True) +
                int(speed_dps).to_bytes(2, "little", signed=True) +
                int(encoder).to_bytes(2, "little", signed=False))

    def test_decoders_signedness_positions_and_scaling(self):
        encoder = diagnostic.decode_encoder_90(bytes.fromhex("90 00 34 12 78 56 BC 9A"))
        self.assertEqual(encoder.values["encoder_position_raw_u16"], 0x1234)
        self.assertEqual(encoder.values["encoder_original_raw_u16"], 0x5678)
        self.assertEqual(encoder.values["encoder_offset_raw_u16"], 0x9ABC)

        single = diagnostic.decode_single_turn_94(
            bytes.fromhex("94 00 00 00 00 00 10 27"))
        self.assertEqual(single.values["single_turn_angle_raw"], 10_000)
        self.assertEqual(single.values["single_turn_angle_degrees_provisional"], 100.0)
        self.assertIn("older V2.35", single.values["selected_layout"])

        observed_v3 = diagnostic.decode_single_turn_94(
            bytes.fromhex("94 00 00 00 8E 02 00 00"))
        self.assertEqual(observed_v3.values["single_turn_angle_raw"], 654)
        self.assertEqual(observed_v3.values["single_turn_angle_degrees_provisional"], 6.54)
        self.assertIn("MHF9025-V3", observed_v3.values["selected_layout"])

        positive = diagnostic.decode_multi_turn_92(
            bytes((0x92,)) + (12_345).to_bytes(7, "little", signed=True))
        negative = diagnostic.decode_multi_turn_92(
            bytes((0x92,)) + (-12_345).to_bytes(7, "little", signed=True))
        self.assertEqual(positive.values["multi_turn_angle_degrees_provisional"], 123.45)
        self.assertEqual(negative.values["multi_turn_angle_degrees_provisional"], -123.45)

        state = diagnostic.decode_state_9c(bytes.fromhex("9C E2 FE FF D4 FE 34 12"))
        self.assertEqual(state.values["temperature_c_s8"], -30)
        self.assertEqual(state.values["torque_current_raw_s16"], -2)
        self.assertEqual(state.values["speed_raw_dps_s16"], -300)
        self.assertEqual(state.values["speed_rpm_provisional"], -50.0)
        self.assertEqual(state.values["encoder_raw_u16"], 0x1234)

    def test_single_turn_flags_reserved_bytes_and_out_of_range(self):
        decoded = diagnostic.decode_single_turn_94(
            bytes.fromhex("94 01 00 00 00 00 A0 8C"))
        self.assertIn("reserved", decoded.values["v2_35_range_check"])
        self.assertIn("outside", decoded.values["v2_35_range_check"])

    def test_malformed_responses_are_rejected(self):
        for command, decoder in diagnostic.DECODERS.items():
            with self.subTest(command=command), self.assertRaises(CommunicationError):
                decoder(bytes((command,)) + b"\x00" * 6)
            with self.subTest(command=command), self.assertRaises(CommunicationError):
                decoder(bytes((command ^ 1,)) + b"\x00" * 7)

    def test_run_uses_four_reads_once_writes_csv_and_never_sends_forbidden_command(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [
            bytes.fromhex("90 00 01 00 02 00 03 00"),
            bytes.fromhex("94 00 00 00 00 00 10 27"),
            bytes.fromhex("92 C7 CF FF FF FF FF FF"),
            bytes.fromhex("9C 20 FE FF 1E 00 34 12"),
        ]
        fixed = datetime(2026, 11, 1, 1, 2, 3, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            results, path = diagnostic.run_diagnostic(
                motor, output_dir=pathlib.Path(directory), now=lambda: fixed)
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        commands = [call.args[0] for call in motor.exchange.call_args_list]
        self.assertEqual(commands, [0x90, 0x94, 0x92, 0x9C])
        self.assertFalse(set(commands) & diagnostic.FORBIDDEN_COMMANDS)
        self.assertEqual(len(results), 4)
        self.assertEqual(len(rows), 4)
        self.assertTrue(all(row["status"] == "ok" for row in rows))
        self.assertTrue(all(call.args[1] == b"\x00" * 7
                            for call in motor.exchange.call_args_list))

    def test_timeout_and_malformed_reply_are_logged_without_retry(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [
            ResponseTimeout("no 0x90 response within 0.5s"),
            bytes.fromhex("94 00 00 00 00 00 10"),
            bytes.fromhex("92 00 00 00 00 00 00 00"),
            bytes.fromhex("9C 20 00 00 00 00 00 00"),
        ]
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            results, _ = diagnostic.run_diagnostic(motor, output_dir=pathlib.Path(directory))
        self.assertEqual(motor.exchange.call_count, 4)
        self.assertEqual(results[0]["status"], "unsupported_or_timeout")
        self.assertEqual(results[1]["status"], "malformed_response")
        self.assertEqual(results[2]["status"], "ok")
        self.assertEqual(results[3]["status"], "ok")

    def test_wrong_bus_is_rejected_before_exchange(self):
        for motor in (Mock(motor_id=2, can_bitrate=500_000),
                      Mock(motor_id=1, can_bitrate=1_000_000)):
            with tempfile.TemporaryDirectory() as directory, self.assertRaises(ValueError):
                diagnostic.run_diagnostic(motor, output_dir=pathlib.Path(directory))
            motor.exchange.assert_not_called()

    def test_cli_uses_existing_transport_and_returns_nonzero_for_unsupported_read(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [
            bytes.fromhex("90 00 01 00 02 00 03 00"),
            ResponseTimeout("0x94 unsupported"),
            bytes.fromhex("92 00 00 00 00 00 00 00"),
            bytes.fromhex("9C 20 00 00 00 00 00 00"),
        ]
        context = Mock()
        context.__enter__ = Mock(return_value=motor)
        context.__exit__ = Mock(return_value=None)
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(diagnostic, "WaveshareMotor", return_value=context) as factory, \
             redirect_stdout(io.StringIO()):
            status = diagnostic.main(["--output-dir", directory])
        self.assertEqual(status, 1)
        factory.assert_called_once_with("/dev/ttyUSB0", 1, 500_000, 2_000_000, 0.5)

    def test_wrapped_difference_and_unwrap_follow_verified_direction(self):
        self.assertAlmostEqual(diagnostic.wrapped_angle_difference(273.94, 6.54), -92.60)
        values = diagnostic.unwrap_single_turn([10.0, 350.0, 250.0, 150.0])
        self.assertEqual(values, [10.0, -10.0, -110.0, -210.0])

    def test_absolute_recovery_reconnects_and_uses_only_three_reads(self):
        initial = Mock(motor_id=1, can_bitrate=500_000)
        initial.exchange.side_effect = [
            bytes.fromhex("90 00 00 00 E8 03 00 00"),
            self.single_frame(10.0), self.multi_frame(10.0),
        ]
        final = Mock(motor_id=1, can_bitrate=500_000)
        final.exchange.side_effect = [
            bytes.fromhex("90 00 00 00 10 6D 00 00"),
            self.single_frame(280.0), self.multi_frame(-80.0),
        ]
        contexts = iter((nullcontext(initial), nullcontext(final)))
        answers = iter((diagnostic.RECOVERY_ROTATED_CONFIRMATION,
                        diagnostic.RECOVERY_POWER_CONFIRMATION))
        fixed = datetime(2026, 11, 1, 4, 0, 0, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            report, csv_path, report_path = diagnostic.run_absolute_recovery(
                lambda: next(contexts), output_dir=pathlib.Path(directory),
                authorize=lambda _prompt: next(answers), now=lambda: fixed)
            with csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            report_text = report_path.read_text(encoding="utf-8")
        self.assertEqual(report["status"], "CONSISTENT_WITH_APPROXIMATE_POSITIVE_90")
        self.assertAlmostEqual(report["wrapped_single_turn_change_deg"], -90.0)
        self.assertEqual(len(rows), 6)
        commands = ([call.args[0] for call in initial.exchange.call_args_list] +
                    [call.args[0] for call in final.exchange.call_args_list])
        self.assertEqual(commands, [0x90, 0x94, 0x92, 0x90, 0x94, 0x92])
        self.assertFalse(set(commands) & diagnostic.FORBIDDEN_COMMANDS)
        self.assertIn("operator estimate", report_text)

    def test_multi_turn_tracking_logs_two_turns_and_generates_svg(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        angles = [0, -120, -240, -360, -480, -600, -720]
        replies = []
        for angle in angles:
            replies.extend((self.multi_frame(angle), self.single_frame(angle),
                            self.state_frame(-120, round(angle) % 65536)))
        motor.exchange.side_effect = replies

        class Clock:
            value = 0.0
            def __call__(self):
                result = self.value
                self.value += 0.001
                return result

        fixed = datetime(2026, 11, 1, 4, 1, 0, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            analysis, csv_path, report_path, plot_path = diagnostic.run_multi_turn_tracking(
                motor, output_dir=pathlib.Path(directory), duration_s=30.0,
                sample_interval_s=0.1,
                authorize=lambda _prompt: diagnostic.TRACKING_CONFIRMATION,
                now=lambda: fixed, clock=Clock(), sleeper=lambda _seconds: None,
                sample_limit=len(angles))
            with csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            plot = plot_path.read_text(encoding="utf-8")
            report = report_path.read_text(encoding="utf-8")
        self.assertEqual(analysis["status"], "TWO_TURNS_OBSERVED")
        self.assertAlmostEqual(analysis["multi_turn_displacement_deg"], -720.0)
        self.assertAlmostEqual(analysis["single_turn_unwrapped_displacement_deg"], -720.0)
        self.assertAlmostEqual(analysis["agreement_error_deg"], 0.0)
        self.assertEqual(analysis["single_turn_wrap_count"], 2)
        self.assertEqual(len(rows), len(angles) * 3)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list],
                         [command for _ in angles for command in (0x92, 0x94, 0x9C)])
        self.assertFalse(set(call.args[0] for call in motor.exchange.call_args_list) &
                         diagnostic.FORBIDDEN_COMMANDS)
        self.assertIn("<svg", plot)
        self.assertIn("Reached 720°: True", report)

    def test_tracking_timeout_is_logged_without_retry(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [
            self.multi_frame(0), self.single_frame(0), self.state_frame(),
            ResponseTimeout("0x92 timeout"),
        ]

        class Clock:
            value = 0.0
            def __call__(self):
                self.value += 0.001
                return self.value

        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            analysis, csv_path, report_path, _ = diagnostic.run_multi_turn_tracking(
                motor, output_dir=pathlib.Path(directory), duration_s=10.0,
                sample_interval_s=0.1,
                authorize=lambda _prompt: diagnostic.TRACKING_CONFIRMATION,
                clock=Clock(), sleeper=lambda _seconds: None)
            with csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            report = report_path.read_text(encoding="utf-8")
        self.assertEqual(motor.exchange.call_count, 4)
        self.assertEqual(analysis["status"], "INSUFFICIENT_DATA")
        self.assertEqual(rows[-1]["status"], "error")
        self.assertIn("INSUFFICIENT_DATA", report)

    def test_tracking_interrupt_during_sample_delay_retains_partial_csv(self):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = [
            self.multi_frame(0), self.single_frame(0), self.state_frame(),
        ]

        class Clock:
            value = 0.0
            def __call__(self):
                self.value += 0.001
                return self.value

        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            analysis, csv_path, report_path, _ = diagnostic.run_multi_turn_tracking(
                motor, output_dir=pathlib.Path(directory), duration_s=10.0,
                sample_interval_s=0.1,
                authorize=lambda _prompt: diagnostic.TRACKING_CONFIRMATION,
                clock=Clock(),
                sleeper=lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt()))
            with csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            report_exists = report_path.exists()
        self.assertEqual(analysis["status"], "INSUFFICIENT_DATA")
        self.assertEqual(len(rows), 4)
        self.assertEqual(rows[-1]["status"], "interrupted")
        self.assertTrue(report_exists)


if __name__ == "__main__":
    unittest.main()
