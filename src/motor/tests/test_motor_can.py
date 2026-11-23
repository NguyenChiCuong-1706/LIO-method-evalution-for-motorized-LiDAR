import io
import pathlib
import sys
import unittest
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import motor_can
import test_motor


class FakeMessage:
    def __init__(self, arbitration_id=0x141, data=b"\x9c\x00\x00\x00\x1e\x00\x34\x12", **kwargs):
        self.arbitration_id = arbitration_id
        self.data = data
        self.dlc = len(data)
        self.is_extended_id = kwargs.get("is_extended_id", False)
        self.is_remote_frame = kwargs.get("is_remote_frame", False)
        self.is_error_frame = kwargs.get("is_error_frame", False)
        self.is_fd = kwargs.get("is_fd", False)


class MotorTests(unittest.TestCase):
    def test_provisional_position_hold_payload_is_bounded_and_signed(self):
        self.assertEqual((b"\xA4" + motor_can.position_hold_payload(0)).hex(" "),
                         "a4 00 01 00 00 00 00 00")
        self.assertEqual(motor_can.position_hold_payload(-123),
                         bytes.fromhex("00 01 00 85 ff ff ff"))
        self.assertEqual(motor_can.position_hold_payload(-123, 90),
                         bytes.fromhex("00 5a 00 85 ff ff ff"))
        for value in (0, -1, 65536, 1.5, True):
            with self.assertRaises(ValueError):
                motor_can.position_hold_payload(0, value)
        for value in (-(1 << 31) - 1, 1 << 31, 1.0, True):
            with self.assertRaises(ValueError):
                motor_can.position_hold_payload(value)

    def test_speed_frames(self):
        expected = {5: "a2 00 00 00 b8 0b 00 00",
                    -5: "a2 00 00 00 48 f4 ff ff",
                    10: "a2 00 00 00 70 17 00 00",
                    -10: "a2 00 00 00 90 e8 ff ff",
                    100: "a2 00 00 00 60 ea 00 00",
                    -100: "a2 00 00 00 a0 15 ff ff"}
        for rpm, frame in expected.items():
            self.assertEqual((b"\xa2" + motor_can.speed_payload(rpm)).hex(" "), frame)
        for rpm in (101, -101, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                motor_can.speed_payload(rpm)

    def test_decode_and_id(self):
        self.assertEqual(motor_can.MotorCAN(motor_id=32).arbitration_id, 0x160)
        with self.assertRaises(ValueError):
            motor_can.MotorCAN(motor_id=0)
        status = motor_can.decode_status1(bytes.fromhex("9a fb 00 f0 00 00 00 09"))
        self.assertEqual((status.temperature_c, status.voltage_v, status.error), (-5, 24.0, 9))
        self.assertTrue(status.layout_valid)
        self.assertFalse(motor_can.decode_status1(
            bytes.fromhex("9A 20 32 0B 00 00 30 00")).layout_valid)
        v3_raw = bytes.fromhex("9A 21 32 0B 00 00 30 00")
        v3 = motor_can.decode_mhf9025_v3_status1(v3_raw, can_id=0x141, dlc=8)
        self.assertEqual(v3.voltage_v, 28.66)
        self.assertEqual(motor_can.decode_mhf9025_v3_voltage(v3_raw), 28.66)
        self.assertEqual(v3.raw, v3_raw)
        self.assertIsNone(v3.fault_state)
        for kwargs in ({"can_id": 0x142}, {"can_id": 0x141, "dlc": 7},
                       {"can_id": 0x141, "is_extended": True}):
            with self.assertRaises(motor_can.CommunicationError):
                motor_can.decode_mhf9025_v3_status1(v3_raw, **kwargs)
        for malformed in (v3_raw[:-1], b"\x9c" + v3_raw[1:]):
            with self.assertRaises(motor_can.CommunicationError):
                motor_can.decode_mhf9025_v3_status1(malformed, can_id=0x141)
        feedback = motor_can.decode_feedback(bytes.fromhex("9c fe ff ff e2 ff 34 12"))
        self.assertEqual((feedback.temperature_c, feedback.torque_current_raw,
                          feedback.speed_dps, feedback.speed_rpm, feedback.encoder_raw),
                         (-2, -1, -30, -5, 0x1234))

    def test_receive_filters_and_timeout(self):
        bus = Mock()
        wrong = [FakeMessage(arbitration_id=0x142), FakeMessage(is_extended_id=True),
                 FakeMessage(is_remote_frame=True), FakeMessage(is_error_frame=True),
                 FakeMessage(is_fd=True), FakeMessage(data=b"\x9c"),
                 FakeMessage(data=b"\x9a" + b"\x00" * 7)]
        good = FakeMessage()
        bus.recv.side_effect = wrong + [good]
        motor = motor_can.MotorCAN(bus=bus)
        self.assertEqual(motor.receive_response(0x9C, 0.1), good.data)
        bus.recv.side_effect = [None]
        with self.assertRaises(motor_can.ResponseTimeout):
            motor.receive_response(0x9C, 0.1)

    def test_send_frame(self):
        bus = Mock()
        motor = motor_can.MotorCAN(bus=bus)
        with patch.object(motor_can, "can", Mock(Message=FakeMessage)):
            motor.send_frame(0x9A)
        sent = bus.send.call_args.args[0]
        self.assertEqual(sent.arbitration_id, 0x141)
        self.assertEqual(sent.data, b"\x9a" + b"\x00" * 7)
        self.assertFalse(sent.is_extended_id)
        with self.assertRaises(ValueError):
            motor.send_frame(0x9A, b"\x00")

    def test_cli_validation(self):
        for args in (["--mode", "both"], ["--speed", "11"], ["--duration", "6"],
                     ["--motor-id", "33"], ["--bitrate", "123456"],
                     ["--mode", "positive", "--allow-motion"]):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                test_motor.parse_args(args)
        self.assertEqual(test_motor.parse_args(["--mode", "both", "--allow-motion",
                                               "--ack-protocol-compatibility"]).mode, "both")

    def test_interface_rejects_listen_only(self):
        output = ("5: can0: <NOARP,UP,LOWER_UP,ECHO> state UP\n"
                  "    can <LISTEN-ONLY> state ERROR-ACTIVE restart-ms 0\n"
                  "    bitrate 500000 sample-point 0.875\n")
        with patch.object(motor_can.subprocess, "run", return_value=Mock(stdout=output)):
            with self.assertRaisesRegex(motor_can.CommunicationError, "LISTEN-ONLY"):
                motor_can.check_socketcan("can0", expected_bitrate=500_000)

    def test_interface_accepts_selected_bitrate(self):
        output = ("5: can0: <NOARP,UP,LOWER_UP,ECHO> state UP\n"
                  "    can state ERROR-ACTIVE restart-ms 0\n"
                  "    bitrate 500000 sample-point 0.875\n")
        with patch.object(motor_can.subprocess, "run", return_value=Mock(stdout=output)):
            self.assertEqual(motor_can.check_socketcan("can0", expected_bitrate=500_000), output)

    def test_cleanup_on_interrupt_and_error(self):
        motor = Mock()
        motor.set_speed_rpm.return_value = motor_can.Feedback(25, 0, 0, 1)
        args = test_motor.parse_args(["--mode", "positive", "--allow-motion",
                                      "--ack-protocol-compatibility"])
        with patch.object(test_motor, "monitor", side_effect=KeyboardInterrupt), redirect_stdout(io.StringIO()):
            with self.assertRaises(KeyboardInterrupt):
                test_motor.run_motion(motor, args)
        motor.stop.assert_called_once()
        motor.disable.assert_called_once()
        motor.reset_mock()
        motor.enable.side_effect = motor_can.ResponseTimeout("enable ack missing")
        with redirect_stdout(io.StringIO()):
            with self.assertRaises(motor_can.ResponseTimeout):
                test_motor.run_motion(motor, args)
        motor.stop.assert_called_once()
        motor.disable.assert_called_once()

    def test_stop_attempts_disable_after_failure(self):
        motor = Mock()
        motor.stop.side_effect = motor_can.ResponseTimeout("no ack")
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertFalse(test_motor.stop_and_disable(motor))
        motor.disable.assert_called_once()

    def test_status_is_read_only_and_fault_blocks_motion(self):
        motor = Mock()
        motor.read_status1.return_value = motor_can.Status1(25, 24.0, 0)
        motor.read_status2.return_value = motor_can.Feedback(25, 0, 0, 123)
        with patch.object(test_motor, "MotorCAN", return_value=motor), \
             patch.object(test_motor, "check_socketcan", return_value="UP bitrate 1000000"), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(test_motor.main(["--mode", "status"]), 0)
        motor.enable.assert_not_called()
        motor.set_speed_rpm.assert_not_called()
        motor.stop.assert_not_called()
        motor.read_status1.return_value = motor_can.Status1(25, 24.0, 0x80)
        with redirect_stdout(io.StringIO()), self.assertRaises(motor_can.ControllerFault):
            test_motor.probe(motor)

    def test_stop_mode_skips_probe(self):
        motor = Mock()
        with patch.object(test_motor, "MotorCAN", return_value=motor), \
             patch.object(test_motor, "check_socketcan", side_effect=AssertionError("link probe")), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(test_motor.main(["--mode", "stop"]), 0)
        motor.read_status1.assert_not_called()
        motor.stop.assert_called_once()
        motor.disable.assert_called_once()

    def test_dry_run_no_bus(self):
        output = io.StringIO()
        with patch.object(test_motor, "MotorCAN", side_effect=AssertionError("opened CAN")):
            with redirect_stdout(output):
                self.assertEqual(test_motor.main(["--mode", "dry-run"]), 0)
        self.assertIn("A2 00 00 00 B8 0B 00 00", output.getvalue())
        self.assertIn("A2 00 00 00 48 F4 FF FF", output.getvalue())


if __name__ == "__main__":
    unittest.main()
