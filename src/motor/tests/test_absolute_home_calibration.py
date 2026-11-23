import copy
from datetime import datetime, timezone
import pathlib
import sys
import tempfile
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import absolute_home_calibration as home
from motor_can import CommunicationError, ResponseTimeout


ENCODER = bytes.fromhex("90 00 00 00 00 10 00 10")
SINGLE_ZERO = bytes.fromhex("94 00 00 00 00 00 00 00")
MULTI_ZERO = bytes.fromhex("92 00 00 00 00 00 00 00")
STOPPED = bytes.fromhex("9C 1A 00 00 00 00 00 10")
MOVING = bytes.fromhex("9C 1A 00 00 3C 00 20 10")
ACK = bytes.fromhex("19 00 00 00 00 00 34 12")
V3_ACK = bytes.fromhex("19 00 00 00 CA 0D 00 00")


def ready_session():
    session = home.new_session(now=lambda: datetime(2026, 11, 1, tzinfo=timezone.utc))
    session["state"] = home.CalibrationState.READY_TO_WRITE.value
    return session


def confirmations():
    values = iter((home.ALIGNMENT_CONFIRMATION, home.SAFETY_CONFIRMATION,
                   home.NO_MOTION_CONFIRMATION, home.FINAL_CONFIRMATION))
    return lambda _prompt: next(values)


class HomeCalibrationTests(unittest.TestCase):
    def motor(self, replies):
        motor = Mock(motor_id=1, can_bitrate=500_000)
        motor.exchange.side_effect = replies
        return motor

    def preflight_replies(self, speed=STOPPED):
        return [ENCODER, SINGLE_ZERO, MULTI_ZERO] + [speed] * home.STATIONARY_SAMPLES

    def test_successful_read_only_preflight_never_sends_write_or_control(self):
        motor = self.motor(self.preflight_replies())
        result = home.collect_preflight(motor, sleeper=lambda _seconds: None)
        commands = [call.args[0] for call in motor.exchange.call_args_list]
        self.assertEqual(commands, [0x90, 0x94, 0x92] + [0x9C] * 5)
        self.assertNotIn(0x19, commands)
        self.assertFalse(set(commands) & {0x91, 0x95, 0x88, 0xA2})
        self.assertEqual(result["motor_state"], "STATIONARY_OBSERVED")
        self.assertEqual(result["communication_status"], "OK")

    def test_moving_motor_blocks_preflight(self):
        motor = self.motor(self.preflight_replies(MOVING))
        with self.assertRaisesRegex(CommunicationError, "stationary"):
            home.collect_preflight(motor, sleeper=lambda _seconds: None)
        self.assertNotIn(0x19, [call.args[0] for call in motor.exchange.call_args_list])

    def test_stale_speed_feedback_blocks_preflight(self):
        motor = self.motor(self.preflight_replies())
        ticks = iter((0.0, 0.1, 0.2, 0.3, 0.4, 1.0))
        with self.assertRaisesRegex(CommunicationError, "stale"):
            home.collect_preflight(motor, clock=lambda: next(ticks),
                                   sleeper=lambda _seconds: None)

    def test_missing_confirmation_fails_without_write(self):
        motor = self.motor(self.preflight_replies())
        session = ready_session()
        persisted = []
        with self.assertRaises(home.AuthorizationError):
            home.write_home_once(
                motor, session, lambda value: persisted.append(copy.deepcopy(value)),
                authorize=lambda _prompt: "NO", sleeper=lambda _seconds: None)
        self.assertEqual(session["state"], home.CalibrationState.FAILED.value)
        self.assertFalse(session["write"]["attempted"])
        self.assertNotIn(0x19, [call.args[0] for call in motor.exchange.call_args_list])
        self.assertTrue(persisted)

    def test_valid_ack_is_recorded_but_remains_unverified(self):
        replies = self.preflight_replies() + [STOPPED] * 5 + [ACK]
        motor = self.motor(replies)
        session = ready_session()
        persisted = []
        home.write_home_once(
            motor, session, lambda value: persisted.append(copy.deepcopy(value)),
            authorize=confirmations(), sleeper=lambda _seconds: None)
        commands = [call.args[0] for call in motor.exchange.call_args_list]
        self.assertEqual(commands.count(0x19), 1)
        self.assertEqual(commands[-1], 0x19)
        self.assertFalse(set(commands) & {0x91, 0x95, 0x88, 0xA2})
        self.assertEqual(session["state"],
                         home.CalibrationState.WRITE_ACKNOWLEDGED_UNVERIFIED.value)
        self.assertEqual(session["write"]["rx_hex"], "19 00 00 00 00 00 34 12")
        self.assertEqual(session["write"]["encoder_offset_raw_u16"], 0x1234)
        self.assertEqual(session["write"]["ack_decoded"]["layout"],
                         "LK-TECH CAN V2.35")
        self.assertEqual(session["verification_status"], "POWER_CYCLE_REQUIRED")
        self.assertTrue(any(item["state"] == "WRITE_SENT_UNVERIFIED"
                            for item in session["history"]))

    def test_observed_v3_ack_is_accepted_but_field_remains_unverified(self):
        replies = self.preflight_replies() + [STOPPED] * 5 + [V3_ACK]
        motor = self.motor(replies)
        session = ready_session()
        home.write_home_once(
            motor, session, lambda _value: None,
            authorize=confirmations(), sleeper=lambda _seconds: None)
        self.assertEqual(session["state"],
                         home.CalibrationState.WRITE_ACKNOWLEDGED_UNVERIFIED.value)
        self.assertEqual(session["write"]["ack_decoded"]["layout"],
                         "MHF9025-V3 observed bytes 4-5")
        self.assertEqual(session["write"]["ack_decoded"]["encoder_value_raw_u16"], 3530)
        self.assertIn("unverified", session["write"]["ack_decoded"]["compatibility"])

    def test_write_timeout_is_unknown_and_never_retried(self):
        replies = (self.preflight_replies() + [STOPPED] * 5 +
                   [ResponseTimeout("no 0x19 reply")])
        motor = self.motor(replies)
        session = ready_session()
        home.write_home_once(motor, session, lambda _value: None,
                             authorize=confirmations(), sleeper=lambda _seconds: None)
        self.assertEqual(session["state"], home.CalibrationState.WRITE_RESULT_UNKNOWN.value)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list].count(0x19), 1)

    def test_malformed_write_reply_is_unknown_and_never_retried(self):
        replies = self.preflight_replies() + [STOPPED] * 5 + [bytes.fromhex(
            "19 01 00 00 00 00 34 12")]
        motor = self.motor(replies)
        session = ready_session()
        home.write_home_once(motor, session, lambda _value: None,
                             authorize=confirmations(), sleeper=lambda _seconds: None)
        self.assertEqual(session["state"], home.CalibrationState.WRITE_RESULT_UNKNOWN.value)
        self.assertEqual([call.args[0] for call in motor.exchange.call_args_list].count(0x19), 1)

    def test_duplicate_calibration_request_is_rejected_without_can(self):
        for state in (home.CalibrationState.WRITE_SENT_UNVERIFIED,
                      home.CalibrationState.WRITE_ACKNOWLEDGED_UNVERIFIED,
                      home.CalibrationState.WRITE_RESULT_UNKNOWN,
                      home.CalibrationState.VERIFIED_AFTER_POWER_CYCLE,
                      home.CalibrationState.FAILED):
            session = ready_session()
            session["state"] = state.value
            session["write"]["attempted"] = True
            motor = self.motor([])
            with self.subTest(state=state), self.assertRaises(home.DuplicateWriteError):
                home.write_home_once(motor, session, lambda _value: None,
                                     authorize=confirmations(), sleeper=lambda _seconds: None)
            motor.exchange.assert_not_called()

    def test_prepare_transitions_to_ready_without_0x19(self):
        motor = self.motor(self.preflight_replies() + [SINGLE_ZERO] * 5)
        session = home.new_session()
        home.prepare_session(motor, session, sleeper=lambda _seconds: None)
        self.assertEqual(session["state"], home.CalibrationState.READY_TO_WRITE.value)
        self.assertFalse(session["write"]["attempted"])
        self.assertNotIn(0x19, [call.args[0] for call in motor.exchange.call_args_list])

    def test_prepare_rejects_unstable_absolute_angle_without_write(self):
        unstable = [
            bytes.fromhex("94 00 00 00 00 00 00 00"),
            bytes.fromhex("94 00 00 00 64 00 00 00"),
            bytes.fromhex("94 00 00 00 C8 00 00 00"),
            bytes.fromhex("94 00 00 00 2C 01 00 00"),
            bytes.fromhex("94 00 00 00 90 01 00 00"),
        ]
        motor = self.motor(self.preflight_replies() + unstable)
        session = home.new_session()
        with self.assertRaisesRegex(CommunicationError, "not stable"):
            home.prepare_session(motor, session, sleeper=lambda _seconds: None)
        self.assertEqual(session["state"], home.CalibrationState.FAILED.value)
        self.assertFalse(session["angle_stability_check"]["stable"])
        self.assertNotIn(0x19, [call.args[0] for call in motor.exchange.call_args_list])

    def test_failed_prepare_preserves_moving_measurements(self):
        motor = self.motor(self.preflight_replies(MOVING))
        session = home.new_session()
        with self.assertRaises(CommunicationError):
            home.prepare_session(motor, session, sleeper=lambda _seconds: None)
        self.assertEqual(session["state"], home.CalibrationState.FAILED.value)
        self.assertEqual(session["preflight"]["motor_state"], "MOVING_OR_UNCERTAIN")
        self.assertEqual(len(session["preflight"]["stationary_check"]["samples"]), 5)

    def test_power_cycle_verification_requires_confirmation_and_zero_angles(self):
        session = ready_session()
        session["state"] = home.CalibrationState.WRITE_ACKNOWLEDGED_UNVERIFIED.value
        session["write"]["attempted"] = True
        motor = self.motor(self.preflight_replies())
        home.verify_after_power_cycle(
            motor, session, lambda _value: None,
            authorize=lambda _prompt: home.POWER_CYCLE_CONFIRMATION,
            sleeper=lambda _seconds: None)
        self.assertEqual(session["state"],
                         home.CalibrationState.VERIFIED_AFTER_POWER_CYCLE.value)
        self.assertFalse(session["verification"]["physical_home_alignment_independently_measured"])
        self.assertNotIn(0x19, [call.args[0] for call in motor.exchange.call_args_list])

    def test_power_cycle_verification_does_not_assume_multi_turn_persistence(self):
        session = ready_session()
        session["state"] = home.CalibrationState.WRITE_ACKNOWLEDGED_UNVERIFIED.value
        session["write"]["attempted"] = True
        nonzero_multi = bytes((0x92,)) + (12_345).to_bytes(7, "little", signed=True)
        replies = [ENCODER, SINGLE_ZERO, nonzero_multi] + [STOPPED] * 5
        motor = self.motor(replies)
        home.verify_after_power_cycle(
            motor, session, lambda _value: None,
            authorize=lambda _prompt: home.POWER_CYCLE_CONFIRMATION,
            mechanical_authorize=lambda _prompt: home.MECHANICAL_HOME_CONFIRMATION,
            sleeper=lambda _seconds: None)
        self.assertEqual(session["state"],
                         home.CalibrationState.VERIFIED_AFTER_POWER_CYCLE.value)
        self.assertEqual(session["verification"]["multi_turn_zero_error_degrees"], 123.45)
        self.assertFalse(session["verification"]["multi_turn_used_for_pass"])
        self.assertTrue(session["verification"][
            "mechanical_home_position_operator_attested"])

    def test_session_log_has_required_identifiers_and_initial_state(self):
        with tempfile.TemporaryDirectory() as directory:
            session, path = home.create_session_file(pathlib.Path(directory))
            loaded = __import__("json").loads(path.read_text())
        self.assertEqual(loaded["session_id"], session["session_id"])
        self.assertEqual(loaded["state"], home.CalibrationState.NOT_CALIBRATED.value)
        self.assertEqual(loaded["proposed_write"]["frame_hex"],
                         "19 00 00 00 00 00 00 00")
        self.assertEqual(loaded["verification_status"], "NOT_PERFORMED")


if __name__ == "__main__":
    unittest.main()
