"""Software-only oscillation tests; the fake motor never opens a serial port."""

from datetime import datetime, timedelta, timezone
import pathlib
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from motor_can import CommunicationError, ResponseTimeout
from ros_oscillating_motor_node import OscillatingMotor, run


class Clock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def utc(self):
        return datetime(2026, 1, 1, tzinfo=timezone.utc) + timedelta(seconds=self.value)

    def sleep(self, seconds):
        self.value += seconds


class FakeMotor:
    def __init__(self, clock, *, angle=0.0, status_timeout=False,
                 fail_during_motion=False, speed_override=None):
        self.clock = clock
        self.angle = angle
        self.target = None
        self.speed_command_rpm = 0.0
        self.enabled = False
        self.commands = []
        self.closed = False
        self.status_timeout = status_timeout
        self.fail_during_motion = fail_during_motion
        self.speed_override = speed_override

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True

    def exchange(self, command, payload=b"\x00" * 7):
        self.clock.value += 0.001
        self.commands.append((command, payload))
        if command == 0x9A:
            if self.status_timeout:
                raise ResponseTimeout("mock status timeout")
            return bytes.fromhex("9A 1D 2E 0B 00 00 30 00")
        if command == 0x9C:
            speed = (round(self.speed_command_rpm * 6) if self.enabled and self.speed_command_rpm else
                     1 if self.enabled and self.target is not None and self.angle != self.target else 0)
            if self.enabled and self.speed_override is not None:
                speed = self.speed_override
            return struct.pack("<BbhhH", 0x9C, 29, 0, speed, round(self.angle * 10) & 0xffff)
        if command == 0x94:
            return b"\x94\x00\x00\x00" + round(self.angle % 360 * 100).to_bytes(4, "little")
        if command == 0x92:
            if self.enabled and self.speed_command_rpm:
                self.angle += self.speed_command_rpm * 6 * 0.025
            elif self.enabled and self.target is not None:
                error = self.target - self.angle
                self.angle += max(-0.025, min(0.025, error))
            return b"\x92" + round(self.angle * 100).to_bytes(7, "little", signed=True)
        if command == 0x88:
            self.enabled = True
            return b"\x88" + b"\x00" * 7
        if command == 0xA4:
            if self.fail_during_motion:
                raise ResponseTimeout("mock A4 timeout")
            self.target = struct.unpack_from("<i", payload, 3)[0] / 100
            return struct.pack("<BbhhH", 0xA4, 29, 0, 0, 0)
        if command == 0xA2:
            self.target = None
            self.speed_command_rpm = struct.unpack_from("<i", payload, 3)[0] / 600
            return struct.pack("<BbhhH", 0xA2, 29, 0, 0, 0)
        if command == 0x81:
            self.enabled = False
            self.target = None
            self.speed_command_rpm = 0.0
            return b"\x81" + b"\x00" * 7
        raise AssertionError(f"unexpected CAN command {command:02X}")

    def stop_and_disable(self):
        self.commands.extend(((0x81, b"\x00" * 7), (0x80, b"\x00" * 7)))
        self.enabled = False
        return True


class FakePublisher:
    def __init__(self, *, node_name=""):
        self.name = node_name
        self.measurements = []
        self.closed = False
        self.loops = 0

    def publish(self, measurement):
        self.measurements.append(measurement)

    def running(self):
        self.loops += 1
        return self.loops <= 4

    def close(self):
        self.closed = True


class OscillatingMotorTests(unittest.TestCase):
    def controller(self, motor, publisher, clock):
        return OscillatingMotor(motor, publisher, monotonic_clock=clock.now,
                                sleep=clock.sleep, utc_clock=clock.utc)

    def test_preflight_home_then_a2_probe(self):
        clock = Clock()
        motor = FakeMotor(clock)
        publisher = FakePublisher()
        controller = self.controller(motor, publisher, clock)
        controller.preflight()
        self.assertEqual([cmd for cmd, _ in motor.commands].count(0x9C), 5)
        self.assertFalse(any(cmd in (0x88, 0xA4) for cmd, _ in motor.commands))
        controller.enable()
        for _ in range(3):
            clock.sleep(0.125)
            controller.step()
        self.assertFalse(controller.homing)
        command, payload = motor.commands[-1]
        self.assertEqual(command, 0xA2)
        self.assertEqual(payload, b"\x00\x00\x00" + struct.pack("<i", 300))
        self.assertIn(0x81, [cmd for cmd, _ in motor.commands])
        self.assertEqual(publisher.measurements[-1].position_rad, 0.0)
        clock.sleep(0.125)
        controller.step()
        self.assertGreater(publisher.measurements[-1].velocity_rad_s, 0)

    def test_default_sweep_commands_ten_rpm(self):
        clock = Clock()
        motor = FakeMotor(clock)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        controller.enable()
        for _ in range(10):
            clock.sleep(0.025)
            controller.step()
        speeds = [struct.unpack_from("<i", payload, 3)[0] / 600
                  for command, payload in motor.commands if command == 0xA2]
        self.assertEqual(speeds[:2], [0.5, 10.0])

    def test_continuous_rotation_crosses_two_turns_without_reversing(self):
        clock = Clock()
        motor = FakeMotor(clock)
        publisher = FakePublisher()
        controller = OscillatingMotor(motor, publisher, continuous=True,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.preflight()
        controller.enable()
        for _ in range(600):
            clock.sleep(0.025)
            controller.step()
            if controller.completed_turns >= 2:
                break
        self.assertGreaterEqual(controller.completed_turns, 2)
        self.assertGreater(motor.angle, 720)
        speeds = [struct.unpack_from("<i", payload, 3)[0] / 600
                  for command, payload in motor.commands if command == 0xA2]
        self.assertEqual(speeds, [0.5, 10.0])
        self.assertNotIn(0x81, [cmd for cmd, _ in motor.commands])
        positions = [sample.position_rad for sample in publisher.measurements]
        self.assertGreater(positions[-1], 4 * 3.14159)

    def test_continuous_fifty_rpm_command_and_angle_samples(self):
        clock = Clock()
        motor = FakeMotor(clock)
        controller = OscillatingMotor(motor, FakePublisher(), continuous=True,
                                      sweep_rpm=50.0, monotonic_clock=clock.now,
                                      sleep=clock.sleep, utc_clock=clock.utc)
        controller.preflight()
        controller.enable()
        for _ in range(15):
            clock.sleep(0.025)
            controller.step()
        speeds = [struct.unpack_from("<i", payload, 3)[0] / 600
                  for command, payload in motor.commands if command == 0xA2]
        self.assertEqual(speeds[:2], [0.5, 50.0])
        self.assertGreater(motor.angle, 20)

    def test_continuous_angle_check_uses_actual_poll_interval(self):
        clock = Clock()
        motor = FakeMotor(clock)
        controller = OscillatingMotor(motor, FakePublisher(), continuous=True,
                                      sweep_rpm=30.0, monotonic_clock=clock.now,
                                      sleep=clock.sleep, utc_clock=clock.utc)
        controller.preflight()
        controller.enable()
        for _ in range(30):
            clock.sleep(0.025)
            controller.step()
            if motor.speed_command_rpm == 30.0:
                break
        self.assertFalse(controller.homing)
        self.assertEqual(motor.speed_command_rpm, 30.0)
        motor.angle += 16.15  # Fake motor adds another 0.75 deg on the 0x92 read.
        clock.sleep(0.092)
        controller.step()  # About 16.9 deg in 92 ms is valid near 30 RPM.
        motor.angle += 13.5
        clock.sleep(0.025)
        with self.assertRaisesRegex(CommunicationError, "discontinuous joint angle feedback"):
            controller.step()  # Similar jump in 25 ms is still rejected.

    def test_continuous_run_reaches_runtime_and_cleans_up(self):
        clock = Clock()
        motor = FakeMotor(clock)
        class AlwaysRunning(FakePublisher):
            def running(self):
                return True
        result = run(continuous=True, max_runtime_s=4,
                     motor_factory=lambda *_args: motor,
                     publisher_factory=lambda **_kwargs: AlwaysRunning(),
                     monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual(result, 0)
        self.assertEqual([command for command, _ in motor.commands][-2:], [0x81, 0x80])
        self.assertTrue(motor.closed)

    def test_unlimited_continuous_run_ignores_elapsed_time_until_shutdown(self):
        clock = Clock()
        motor = FakeMotor(clock)
        class ThreePollPublisher(FakePublisher):
            def running(self):
                self.loops += 1
                return self.loops <= 2
        publisher = ThreePollPublisher()
        with patch.object(OscillatingMotor, "step", side_effect=lambda: clock.sleep(700)):
            result = run(continuous=True, max_runtime_s=0,
                         motor_factory=lambda *_args: motor,
                         publisher_factory=lambda **_kwargs: publisher,
                         monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual(result, 0)
        self.assertGreater(clock.now(), 600)
        self.assertEqual(publisher.loops, 3)
        self.assertEqual([command for command, _ in motor.commands][-2:], [0x81, 0x80])

    def test_sigint_during_continuous_rotation_runs_cleanup(self):
        clock = Clock()
        motor = FakeMotor(clock)
        class AlwaysRunning(FakePublisher):
            def running(self):
                return True
        handlers = {}
        def install(signum, callback):
            previous = handlers.get(signum, lambda *_args: None)
            handlers[signum] = callback
            return previous
        original_exchange = motor.exchange
        def interrupt_after_motion(command, payload=b"\x00" * 7):
            if command == 0x92 and motor.speed_command_rpm >= 10:
                handlers[2](2, None)
            return original_exchange(command, payload)
        motor.exchange = interrupt_after_motion
        with patch("ros_oscillating_motor_node.signal.signal", side_effect=install):
            result = run(continuous=True, max_runtime_s=10,
                         motor_factory=lambda *_args: motor,
                         publisher_factory=lambda **_kwargs: AlwaysRunning(),
                         monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual(result, 0)
        self.assertEqual([command for command, _ in motor.commands][-2:], [0x81, 0x80])

    def test_endpoint_stop_settle_then_reverse_probe(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=40.0)
        publisher = FakePublisher()
        controller = self.controller(motor, publisher, clock)
        controller.source.estimator.set_reference(40.0, 40.0)
        # Set the HOME-aligned estimator offset to zero for this isolated step test.
        controller.source.estimator.reference_offset_deg = 0.0
        controller.home_turn_deg = 0.0
        controller.target_deg = 45.0
        controller.homing = False
        controller.sweep_phase = "sweep"
        controller.commanded_rpm = 10.0
        controller.enabled = True
        motor.enabled = True
        controller.source.poll_once()
        clock.sleep(0.125)
        controller.source.poll_once()
        motor.speed_command_rpm = 10.0
        for _ in range(100):
            clock.sleep(0.025)
            controller.step()
            if controller.legs_completed and controller.sweep_phase == "sweep":
                break
        self.assertEqual(controller.legs_completed, 1)
        self.assertEqual(controller.target_deg, -45.0)
        self.assertEqual(controller.sweep_phase, "sweep")
        self.assertEqual(motor.speed_command_rpm, -10.0)
        commands = [command for command, _ in motor.commands]
        self.assertIn(0xA4, commands)
        self.assertNotIn(0x81, commands)

    def test_start_away_from_home_commands_nearest_home_first(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=20.0)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        self.assertTrue(controller.homing)
        self.assertEqual(controller.target_deg, 0.0)
        self.assertFalse(any(cmd in (0x88, 0xA4) for cmd, _ in motor.commands))
        controller.enable()
        clock.sleep(0.125)
        controller.step()
        self.assertEqual(motor.target, 15.0)
        a4_payload = [payload for command, payload in motor.commands if command == 0xA4][-1]
        self.assertEqual(struct.unpack_from("<H", a4_payload, 1)[0], 30)

    def test_home_slows_to_one_rpm_within_ten_degrees(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=8.0)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        controller.enable()
        clock.sleep(0.125)
        controller.step()
        self.assertEqual(motor.target, 7.0)
        a4_payload = [payload for command, payload in motor.commands if command == 0xA4][-1]
        self.assertEqual(struct.unpack_from("<H", a4_payload, 1)[0], 6)

    def test_one_noisy_preflight_speed_sample_requires_five_new_stable_samples(self):
        clock = Clock()
        motor = FakeMotor(clock)
        original_exchange = motor.exchange
        speed_samples = iter((0, 0, 5, 0, 0, 0, 0, 0))
        def noisy_exchange(command, payload=b"\x00" * 7):
            result = original_exchange(command, payload)
            if command == 0x9C:
                speed = next(speed_samples)
                return result[:4] + struct.pack("<h", speed) + result[6:]
            return result
        motor.exchange = noisy_exchange
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        self.assertEqual([cmd for cmd, _ in motor.commands].count(0x9C), 8)
        self.assertFalse(any(cmd in (0x88, 0xA4) for cmd, _ in motor.commands))

    def test_sustained_preflight_motion_never_enables(self):
        clock = Clock()
        motor = FakeMotor(clock)
        original_exchange = motor.exchange
        def moving_exchange(command, payload=b"\x00" * 7):
            result = original_exchange(command, payload)
            return result[:4] + struct.pack("<h", 5) + result[6:] if command == 0x9C else result
        motor.exchange = moving_exchange
        with self.assertRaisesRegex(CommunicationError, "not confirmed stationary"):
            run(motor_factory=lambda *_args: motor,
                publisher_factory=FakePublisher, monotonic_clock=clock.now,
                sleep=clock.sleep, utc_clock=clock.utc)
        self.assertNotIn(0x88, [cmd for cmd, _ in motor.commands])
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_start_at_home_after_one_turn_uses_current_turn(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=360.0)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        self.assertEqual(controller.home_turn_deg, 360.0)
        self.assertEqual(controller.target_deg, 360.0)
        controller.enable()
        for _ in range(3):
            clock.sleep(0.125)
            controller.step()
        self.assertEqual(controller.target_deg, 405.0)
        self.assertEqual(motor.speed_command_rpm, 0.5)

    def test_speed_mode_reaches_endpoint_and_reverses_after_stop(self):
        clock = Clock()
        motor = FakeMotor(clock)
        controller = OscillatingMotor(motor, FakePublisher(), sweep_rpm=20.0,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.preflight()
        controller.enable()
        for _ in range(250):
            clock.sleep(0.025)
            controller.step()
            if controller.legs_completed:
                break
        self.assertEqual(controller.legs_completed, 1)
        speed_commands = [struct.unpack_from("<i", payload, 3)[0] for command, payload
                          in motor.commands if command == 0xA2]
        self.assertIn(12000, speed_commands)  # 20 RPM uses the GUI's 0xA2 encoding.
        self.assertIn(6000, speed_commands)  # 10 RPM in the endpoint band.
        self.assertIn(-300, speed_commands)  # Reverse begins at 0.5 RPM.
        self.assertEqual([command for command, _ in motor.commands].count(0x81), 1)
        self.assertIn(0xA4, [command for command, _ in motor.commands])
        self.assertLessEqual(max(abs(value) for value in speed_commands), 12000)

    def test_two_legs_follow_20_rpm_center_and_10_rpm_edges(self):
        clock = Clock()
        motor = FakeMotor(clock)
        controller = OscillatingMotor(motor, FakePublisher(), sweep_rpm=20.0,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.preflight()
        controller.enable()
        for _ in range(300):
            clock.sleep(controller.period)
            controller.step()
            if controller.legs_completed == 2:
                break
        self.assertEqual(controller.legs_completed, 2)
        speeds = [struct.unpack_from("<i", payload, 3)[0] / 600
                  for command, payload in motor.commands if command == 0xA2]
        self.assertEqual(speeds, [0.5, 20.0, 10.0, 0.5,
                                  -0.5, -10.0, -20.0, -10.0, -0.5, 0.5])

    def test_negative_joint_sign_reverses_a2_command_sign(self):
        clock = Clock()
        motor = FakeMotor(clock)
        controller = OscillatingMotor(motor, FakePublisher(), direction_sign=-1,
                                      sweep_rpm=20.0,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.preflight()
        controller.enable()
        for _ in range(9):
            clock.sleep(0.125)
            controller.step()
        speed_commands = [struct.unpack_from("<i", payload, 3)[0] for command, payload
                          in motor.commands if command == 0xA2]
        self.assertEqual(speed_commands[:2], [-300, -12000])

    def test_sweep_overspeed_attempts_cleanup(self):
        clock = Clock()
        motor = FakeMotor(clock)
        original_exchange = motor.exchange
        def speeding_exchange(command, payload=b"\x00" * 7):
            result = original_exchange(command, payload)
            if command == 0x9C and motor.speed_command_rpm:
                return result[:4] + struct.pack("<h", 181) + result[6:]
            return result
        motor.exchange = speeding_exchange
        class AlwaysRunning(FakePublisher):
            def running(self):
                return True
        with self.assertRaisesRegex(CommunicationError, "180 deg/s"):
            run(motor_factory=lambda *_args: motor,
                publisher_factory=AlwaysRunning, monotonic_clock=clock.now,
                sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_sweep_allows_recorded_transient_below_new_limit(self):
        clock = Clock()
        motor = FakeMotor(clock, speed_override=145)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.homing = False
        motor.enabled = True
        self.assertEqual(controller._speed_check().speed_dps, 145)

    def test_continuous_single_36_rpm_sample_does_not_stop(self):
        clock = Clock()
        motor = FakeMotor(clock, speed_override=216)
        controller = OscillatingMotor(motor, FakePublisher(), continuous=True,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.homing = False
        motor.enabled = True
        self.assertEqual(controller._speed_check().speed_dps, 216)
        self.assertEqual(controller.continuous_overspeed_samples, 0)
        motor.speed_override = 510
        controller._speed_check()
        self.assertEqual(controller.continuous_overspeed_samples, 1)
        motor.speed_override = 90
        self.assertEqual(controller._speed_check().speed_dps, 90)
        self.assertEqual(controller.continuous_overspeed_samples, 0)

    def test_continuous_eighty_rpm_feedback_is_at_limit(self):
        clock = Clock()
        motor = FakeMotor(clock, speed_override=480)
        controller = OscillatingMotor(motor, FakePublisher(), continuous=True,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.homing = False
        motor.enabled = True
        for _ in range(5):
            self.assertEqual(controller._speed_check().speed_dps, 480)
        self.assertEqual(controller.continuous_overspeed_samples, 0)

    def test_continuous_five_consecutive_overspeed_samples_stop(self):
        clock = Clock()
        motor = FakeMotor(clock, speed_override=510)
        controller = OscillatingMotor(motor, FakePublisher(), continuous=True,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.homing = False
        motor.enabled = True
        for _ in range(4):
            controller._speed_check()
        with self.assertRaisesRegex(CommunicationError, "480 deg/s for 5 consecutive samples"):
            try:
                controller._speed_check()
            finally:
                motor.stop_and_disable()
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_continuous_critical_speed_stops_immediately(self):
        clock = Clock()
        motor = FakeMotor(clock, speed_override=541)
        controller = OscillatingMotor(motor, FakePublisher(), continuous=True,
                                      monotonic_clock=clock.now, sleep=clock.sleep,
                                      utc_clock=clock.utc)
        controller.homing = False
        motor.enabled = True
        with self.assertRaisesRegex(CommunicationError, "critical speed feedback"):
            controller._speed_check()
        self.assertEqual(controller.continuous_overspeed_samples, 0)

    def test_position_hold_drift_triggers_stop_disable_cleanup(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=44.0)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        controller.homing = False
        controller.sweep_phase = "settle"
        controller.home_turn_deg = 0.0
        controller.target_deg = 45.0
        controller.hold_target_deg = 44.0
        clock.sleep(0.125)
        controller.source.poll_once()
        clock.sleep(0.125)
        motor.angle = 45.5
        with self.assertRaisesRegex(CommunicationError, "position hold drift"):
            try:
                controller.step()
            finally:
                motor.stop_and_disable()
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_hold_approach_timeout_never_sends_a4(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=44.0, speed_override=7)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        controller.homing = False
        controller.sweep_phase = "hold_approach"
        controller.home_turn_deg = 0.0
        controller.target_deg = 45.0
        controller.hold_wait_started = clock.now()
        motor.enabled = True
        with self.assertRaisesRegex(CommunicationError, "did not slow"):
            for _ in range(30):
                clock.sleep(0.125)
                controller.step()
        self.assertNotIn(0xA4, [cmd for cmd, _ in motor.commands])

    def test_run_publishes_then_stops_and_disables(self):
        clock = Clock()
        motor = FakeMotor(clock)
        publisher = FakePublisher()
        result = run(motor_factory=lambda *_args: motor,
                     publisher_factory=lambda **_kwargs: publisher,
                     monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual(result, 0)
        self.assertTrue(motor.closed)
        self.assertTrue(publisher.closed)
        self.assertGreater(len(publisher.measurements), 1)
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_communication_failure_attempts_stop_and_disable(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=20.0, fail_during_motion=True)
        publisher = FakePublisher()
        with self.assertRaises(ResponseTimeout):
            run(motor_factory=lambda *_args: motor,
                publisher_factory=lambda **_kwargs: publisher,
                monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])
        self.assertTrue(publisher.closed)

    def test_status_timeout_never_enables_and_attempts_cleanup(self):
        clock = Clock()
        motor = FakeMotor(clock, status_timeout=True)
        with self.assertRaises(ResponseTimeout):
            run(motor_factory=lambda *_args: motor,
                publisher_factory=FakePublisher,
                monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertFalse(any(cmd in (0x88, 0xA4) for cmd, _ in motor.commands))
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_overspeed_feedback_stops_before_next_position_command(self):
        clock = Clock()
        motor = FakeMotor(clock, speed_override=61)
        publisher = FakePublisher()
        with self.assertRaisesRegex(CommunicationError, "speed feedback exceeds"):
            run(motor_factory=lambda *_args: motor,
                publisher_factory=lambda **_kwargs: publisher,
                monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertNotIn(0xA4, [cmd for cmd, _ in motor.commands])
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_home_transient_speed_reduces_lead_then_repeated_excess_stops(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=20.0, speed_override=55)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        controller.enable()
        clock.sleep(0.125)
        controller.step()
        self.assertEqual(motor.target, 19.0)  # Smaller lead after a fast sample.
        clock.sleep(0.125)
        controller.step()
        clock.sleep(0.125)
        with self.assertRaisesRegex(CommunicationError, "repeatedly exceeds 54"):
            controller.step()

    def test_home_feedback_at_fifty_dps_remains_below_new_limits(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=20.0, speed_override=50)
        controller = self.controller(motor, FakePublisher(), clock)
        controller.preflight()
        controller.enable()
        for _ in range(4):
            clock.sleep(0.125)
            controller.step()
        self.assertTrue(controller.homing)
        self.assertEqual(controller.home_soft_overspeed_samples, 0)

    def test_lost_progress_stops_with_cleanup(self):
        clock = Clock()
        motor = FakeMotor(clock)
        # Freeze the reported shaft angle despite successive speed commands.
        original_exchange = motor.exchange
        def stalled_exchange(command, payload=b"\x00" * 7):
            result = original_exchange(command, payload)
            if command == 0x92:
                return b"\x92" + (0).to_bytes(7, "little", signed=True)
            return result
        motor.exchange = stalled_exchange
        class AlwaysRunning(FakePublisher):
            def running(self):
                return True
        with self.assertRaisesRegex(CommunicationError, "sweep direction not confirmed"):
            run(max_runtime_s=10, motor_factory=lambda *_args: motor,
                publisher_factory=AlwaysRunning, monotonic_clock=clock.now,
                sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])

    def test_ctrl_c_during_initial_homing_attempts_stop_and_disable(self):
        clock = Clock()
        motor = FakeMotor(clock, angle=20.0)
        publisher = FakePublisher()
        original_exchange = motor.exchange
        def interrupted_exchange(command, payload=b"\x00" * 7):
            if command == 0xA4:
                raise KeyboardInterrupt
            return original_exchange(command, payload)
        motor.exchange = interrupted_exchange
        result = run(motor_factory=lambda *_args: motor,
                     publisher_factory=lambda **_kwargs: publisher,
                     monotonic_clock=clock.now, sleep=clock.sleep, utc_clock=clock.utc)
        self.assertEqual(result, 0)
        self.assertEqual([cmd for cmd, _ in motor.commands][-2:], [0x81, 0x80])
        self.assertFalse(motor.enabled)
        self.assertTrue(publisher.closed)

    def test_invalid_arguments_do_not_open_motor(self):
        with self.assertRaises(ValueError):
            run(can_bitrate=250_000, motor_factory=lambda *_args: self.fail("opened"))
        with self.assertRaises(ValueError):
            run(sweep_rpm=21, motor_factory=lambda *_args: self.fail("opened"))
        with self.assertRaises(ValueError):
            run(sweep_rpm=51, continuous=True,
                motor_factory=lambda *_args: self.fail("opened"))
        with self.assertRaises(ValueError):
            run(max_runtime_s=0, motor_factory=lambda *_args: self.fail("opened"))


if __name__ == "__main__":
    unittest.main()
