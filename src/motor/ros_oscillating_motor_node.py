"""Bounded MHF9025-V3 ROS 2 position oscillation and /joint_states publisher.

Starting this node first returns to the nearest stored mechanical HOME turn,
then commands motion between theta1 = +45 and -45 degrees about that turn.
It is the sole owner of the Waveshare serial port.
HOME uses slow A4 position mode. Sweeps use the GUI's A2 speed command with a
slow approach to each endpoint. Run only with a secured assembly and an
independent power cutoff. No ROM or PID commands are sent.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
import signal
import sys
import time

from motor_can import (CommunicationError, ControllerFault, MotorError,
                       decode_feedback, decode_mhf9025_v3_status1,
                       position_hold_payload, speed_payload)
from ros_joint_state_node import ReadOnlyJointStateSource
from joint_state_publisher import RclpyJointStatePublisher
from waveshare_motor import WaveshareMotor, _encoder_delta


AMPLITUDE_DEG = 45.0
HOME_SPEED_DPS = 30  # 5 RPM; A4 accepts degrees per second.
HOME_APPROACH_SPEED_DPS = 6
HOME_APPROACH_START_DEG = 10.0
DEFAULT_SWEEP_RPM = 10.0
# Preserve the original ±45-degree sweep cutoff.
SWEEP_HARD_LIMIT_RPM = 30.0
# In continuous A2 mode, five consecutive readings above 80 RPM stop;
# a severe reading above 90 RPM stops at once.
CONTINUOUS_OVERSPEED_LIMIT_RPM = 80.0
CONTINUOUS_OVERSPEED_SAMPLES = 5
CONTINUOUS_CRITICAL_SPEED_DPS = 540  # 90 RPM emergency ceiling.
CONTINUOUS_MAX_COMMAND_RPM = 50.0
CONTINUOUS_ANGLE_TOLERANCE_DEG = 1.0
EDGE_RPM = 10.0
PROBE_RPM = 0.5
EDGE_START_DEG = 40.0
POSITION_LEAD_DEG = 5.0
HOME_APPROACH_LEAD_DEG = 1.0
HOME_TOLERANCE_DEG = 0.5
SWEEP_STOP_MARGIN_DEG = 3.0
HOLD_APPROACH_RPM = 0.5
HOLD_TARGET_TOLERANCE_DEG = 1.0
HOLD_MAX_DRIFT_DEG = 1.0
HOLD_MAX_SPEED_DPS = 6
HOLD_SETTLE_TIMEOUT_S = 3.0
HOME_MAX_SPEED_DPS = 60  # 10 RPM; immediate HOME safety limit.
HOME_SOFT_SPEED_DPS = 54  # 9 RPM; three consecutive HOME samples.
HOME_SOFT_OVERSPEED_SAMPLES = 3
HOME_MAX_ANGLE_STEP_DEG = 2.0
SWEEP_MAX_ANGLE_STEP_DEG = 8.0
PREFLIGHT_SPEED_DPS = 3
PREFLIGHT_REQUIRED_SAMPLES = 5
PREFLIGHT_MAX_SAMPLES = 20


class OscillatingMotor:
    """One CAN owner; all motion decisions use fresh controller feedback."""

    def __init__(self, motor, publisher, *, direction_sign=1, sweep_rpm=DEFAULT_SWEEP_RPM,
                 continuous=False,
                 period=0.025, monotonic_clock=time.monotonic, sleep=time.sleep,
                 utc_clock=lambda: datetime.now(timezone.utc)):
        if direction_sign not in (-1, 1):
            raise ValueError("direction_sign must be +1 or -1")
        max_command_rpm = CONTINUOUS_MAX_COMMAND_RPM if continuous else 20.0
        if not math.isfinite(sweep_rpm) or not EDGE_RPM <= sweep_rpm <= max_command_rpm:
            raise ValueError(f"sweep RPM must be between 10 and {max_command_rpm:g}")
        self.motor = motor
        self.publisher = publisher
        self.period = period
        self.sweep_rpm = sweep_rpm
        self.continuous = continuous
        self.completed_turns = 0
        self.rotation_origin_deg = None
        self.clock = monotonic_clock
        self.sleep = sleep
        self.source = ReadOnlyJointStateSource(
            motor, publisher, direction_sign=direction_sign,
            monotonic_clock=monotonic_clock, utc_clock=utc_clock)
        self.enabled = False
        self.home_turn_deg = None
        self.target_deg = None
        self.homing = True
        self.endpoint_samples = 0
        self.last_theta_deg = None
        self.last_theta_time = None
        self.last_progress_time = None
        self.last_progress_angle = None
        self.legs_completed = 0
        self.sweep_phase = None
        self.commanded_rpm = 0.0
        self.probe_origin = None
        self.probe_started = None
        self.settle_samples = 0
        self.hold_target_deg = None
        self.hold_wait_started = None
        self.home_soft_overspeed_samples = 0
        self.continuous_overspeed_samples = 0

    def _status_check(self) -> None:
        status = decode_mhf9025_v3_status1(
            self.motor.exchange(0x9A), can_id=0x141)
        # V3 protection definition is UNKNOWN. Preserve the GUI's conservative
        # legacy candidate-bit check without claiming fault-free status.
        if status.fault_state is True or status.raw[7] & 0x09:
            raise ControllerFault("0x9A fault candidate active; V3 protection mapping UNKNOWN")
        if not 24.0 <= status.voltage_v <= 32.0:
            raise CommunicationError("voltage outside local experimental 24–32 V window")
        if not 0 <= status.temperature_c <= 70:
            raise CommunicationError("temperature outside local experimental 0–70 C window")

    def _speed_check(self):
        feedback = decode_feedback(self.motor.exchange(0x9C))
        limit = (HOME_MAX_SPEED_DPS if self.homing else
                 round((CONTINUOUS_OVERSPEED_LIMIT_RPM if self.continuous else
                        SWEEP_HARD_LIMIT_RPM) * 6))
        speed = abs(feedback.speed_dps)
        if self.continuous and not self.homing:
            if speed > CONTINUOUS_CRITICAL_SPEED_DPS:
                raise CommunicationError(
                    f"critical speed feedback {feedback.speed_dps} deg/s exceeds "
                    f"{CONTINUOUS_CRITICAL_SPEED_DPS} deg/s emergency limit")
            if speed > limit:
                self.continuous_overspeed_samples += 1
                print(f"overspeed candidate: {feedback.speed_dps} deg/s "
                      f"({self.continuous_overspeed_samples}/"
                      f"{CONTINUOUS_OVERSPEED_SAMPLES} consecutive samples)", flush=True)
                if self.continuous_overspeed_samples >= CONTINUOUS_OVERSPEED_SAMPLES:
                    raise CommunicationError(
                        f"speed feedback exceeds {limit} deg/s for "
                        f"{CONTINUOUS_OVERSPEED_SAMPLES} consecutive samples")
            else:
                self.continuous_overspeed_samples = 0
        elif speed > limit:
            raise CommunicationError(f"speed feedback exceeds {limit} deg/s safety limit")
        if self.homing:
            self.home_soft_overspeed_samples = (
                self.home_soft_overspeed_samples + 1
                if abs(feedback.speed_dps) > HOME_SOFT_SPEED_DPS else 0)
            if self.home_soft_overspeed_samples >= HOME_SOFT_OVERSPEED_SAMPLES:
                raise CommunicationError(
                    f"speed feedback repeatedly exceeds {HOME_SOFT_SPEED_DPS} deg/s "
                    "HOME tolerance")
        return feedback

    def preflight(self) -> None:
        self._status_check()
        first = None
        consecutive = 0
        for index in range(PREFLIGHT_MAX_SAMPLES):
            feedback = self._speed_check()
            if abs(feedback.speed_dps) > PREFLIGHT_SPEED_DPS:
                first = None
                consecutive = 0
            elif first is not None and abs(_encoder_delta(first.encoder_raw, feedback.encoder_raw)) > 2:
                first = feedback
                consecutive = 1
            else:
                if first is None:
                    first = feedback
                consecutive += 1
                if consecutive == PREFLIGHT_REQUIRED_SAMPLES:
                    break
            if index < PREFLIGHT_MAX_SAMPLES - 1:
                self.sleep(0.1)
        else:
            raise CommunicationError("motor not confirmed stationary during bounded preflight")
        offset = self.source.initialize()
        # Align the live 0x92 count to the absolute 0x94 single-turn reference.
        # The closest multiple of 360 is the initial HOME target; never assume
        # the multi-turn count survives a power cycle.
        self.source.poll_once()  # Seed the timestamp-aware velocity estimator.
        self.sleep(self.period)
        if not self.source.poll_once() or self.source.last_measurement is None:
            raise CommunicationError("fresh 0x92 angle/velocity unavailable")
        theta = self.source.last_measurement.angle_degrees
        self.home_turn_deg = round(theta / 360.0) * 360.0
        self.target_deg = self.home_turn_deg
        print(f"Returning to nearest HOME={self.home_turn_deg:+.2f} deg from "
              f"theta1={theta:+.2f} deg; 0x92 offset={offset:+.2f} deg; "
              f"protection UNKNOWN; {HOME_SPEED_DPS / 6:g} RPM A4 HOME, "
              f"slowing to {HOME_APPROACH_SPEED_DPS / 6:g} RPM within "
              f"{HOME_APPROACH_START_DEG:g} deg; "
              f"then {self.sweep_rpm:g} RPM A2 "
              f"{'continuous positive rotation' if self.continuous else 'sweeps'}", flush=True)

    def _continuous_step(self, theta: float) -> None:
        """Keep positive joint rotation across 360-degree boundaries."""
        if self.sweep_phase == "probe":
            progress = theta - self.probe_origin
            if progress < -0.5:
                raise CommunicationError("continuous rotation moved opposite commanded direction")
            if progress >= 0.25:
                self.sweep_phase = "continuous"
                self._command_speed(self.sweep_rpm * self.source.estimator.direction_sign)
            elif self.clock() - self.probe_started > 1.5:
                raise CommunicationError("continuous direction not confirmed within 1.5 seconds")
        elif self.sweep_phase != "continuous":
            raise CommunicationError("invalid continuous rotation phase")
        if self.last_progress_time is None:
            self.last_progress_time, self.last_progress_angle = self.clock(), theta
        elif self.clock() - self.last_progress_time >= 5.0:
            if theta - self.last_progress_angle < 0.5:
                raise CommunicationError("no forward joint progress within 5 seconds")
            self.last_progress_time, self.last_progress_angle = self.clock(), theta
        if theta < self.rotation_origin_deg - 0.5:
            raise CommunicationError("continuous joint rotation reversed")
        turns = max(0, int((theta - self.rotation_origin_deg) // 360.0))
        if turns > self.completed_turns:
            self.completed_turns = turns
            print(f"Continuous rotation: {turns} complete turn(s); "
                  f"theta1={theta:+.2f} deg", flush=True)

    def enable(self) -> None:
        reply = self.motor.exchange(0x88)
        if reply != b"\x88" + b"\x00" * 7:
            raise CommunicationError("ENABLE acknowledgement mismatch")
        self.enabled = True

    def _command_speed(self, rpm: float) -> None:
        reply = self.motor.exchange(0xA2, speed_payload(rpm))
        decode_feedback(reply, 0xA2)
        self.commanded_rpm = rpm

    def _begin_probe(self, theta: float) -> None:
        if not self.enabled:
            self.enable()
        self.sweep_phase = "probe"
        self.probe_origin = theta
        self.probe_started = self.clock()
        self.last_progress_time = None
        self._command_speed(math.copysign(PROBE_RPM, self.target_deg - theta) *
                            self.source.estimator.direction_sign)

    def _profile_speed(self, theta: float) -> float:
        return EDGE_RPM if abs(theta - self.home_turn_deg) >= EDGE_START_DEG else self.sweep_rpm

    def _sweep_step(self, theta: float, raw_angle: float, feedback) -> None:
        error = self.target_deg - theta
        direction = 1 if error > 0 else -1
        if self.sweep_phase in ("probe", "sweep"):
            now = self.clock()
            if self.last_progress_time is None:
                self.last_progress_time, self.last_progress_angle = now, theta
            elif now - self.last_progress_time >= 5.0:
                progress = direction * (theta - self.last_progress_angle)
                if abs(error) > 2.0 and progress < 0.5:
                    raise CommunicationError("no forward joint progress within 5 seconds")
                self.last_progress_time, self.last_progress_angle = now, theta
        joint_command = self.commanded_rpm * self.source.estimator.direction_sign
        if abs(theta - self.target_deg) > 2.0 and self.sweep_phase == "sweep" and \
                direction * joint_command < 0:
            raise CommunicationError("sweep passed endpoint by more than 2 degrees")
        if self.sweep_phase == "probe":
            intended = 1 if joint_command > 0 else -1
            progress = intended * (theta - self.probe_origin)
            if progress < -0.5:
                raise CommunicationError("sweep moved opposite commanded direction")
            if progress >= 0.25:
                self.sweep_phase = "sweep"
                speed = self._profile_speed(theta)
                self._command_speed(intended * self.source.estimator.direction_sign *
                                    speed)
            elif self.clock() - self.probe_started > 1.5:
                raise CommunicationError("sweep direction not confirmed within 1.5 seconds")
            return
        if self.sweep_phase == "sweep":
            if abs(error) <= SWEEP_STOP_MARGIN_DEG:
                # A 20 RPM step can cross the 40-degree boundary between
                # samples. Issue the requested 10 RPM edge command first.
                if abs(self.commanded_rpm) > EDGE_RPM:
                    self._command_speed(math.copysign(EDGE_RPM, self.commanded_rpm))
                    return
                # Keep the controller enabled while slowing down. STOP here
                # allowed the external load to pull the shaft away before A4.
                self._command_speed(math.copysign(HOLD_APPROACH_RPM, self.commanded_rpm))
                self.sweep_phase = "hold_approach"
                self.hold_wait_started = self.clock()
                self.settle_samples = 0
            else:
                desired = math.copysign(self._profile_speed(theta), self.commanded_rpm)
                if desired != self.commanded_rpm:
                    self._command_speed(desired)
            return
        if self.sweep_phase == "hold_approach":
            if abs(error) > SWEEP_STOP_MARGIN_DEG + 0.5:
                raise CommunicationError("endpoint drifted during hold approach")
            if self.clock() - self.hold_wait_started > HOLD_SETTLE_TIMEOUT_S:
                raise CommunicationError("endpoint did not slow before position hold")
            if (abs(error) > HOLD_TARGET_TOLERANCE_DEG or
                    abs(feedback.speed_dps) > HOLD_MAX_SPEED_DPS):
                self.settle_samples = 0
                return
            self.settle_samples += 1
            if self.settle_samples < 3:
                return
            # Match the GUI's direct A2-to-A4 transition at low speed. The
            # endpoint target is at most one degree from fresh 0x92 feedback.
            self._status_check()
            target_raw = (round(raw_angle * 100) +
                          round(self.source.estimator.direction_sign * error * 100))
            reply = self.motor.exchange(0xA4, position_hold_payload(target_raw, 1))
            decode_feedback(reply, 0xA4)
            self.commanded_rpm = 0.0
            self.hold_target_deg = self.target_deg
            self.sweep_phase = "settle"
            self.settle_samples = 0
            print(f"Endpoint position hold active at theta1={theta:+.2f} deg; "
                  "controller feedback only, physical hold unverified", flush=True)
            return
        if self.sweep_phase == "settle":
            if (abs(error) > 2.0 or
                    abs(theta - self.hold_target_deg) > HOLD_MAX_DRIFT_DEG or
                    abs(feedback.speed_dps) > HOLD_MAX_SPEED_DPS):
                raise CommunicationError("position hold drift or speed exceeded limit")
            self.settle_samples += 1
            if self.settle_samples >= 5:
                self.legs_completed += 1
                self.target_deg = (self.home_turn_deg - AMPLITUDE_DEG if
                                   self.target_deg > self.home_turn_deg else
                                   self.home_turn_deg + AMPLITUDE_DEG)
                print(f"Endpoint settled; leg={self.legs_completed}; "
                      f"next target={self.target_deg:+.1f} deg", flush=True)
                self._begin_probe(theta)
            return
        raise CommunicationError("invalid sweep phase")

    def step(self) -> None:
        self._status_check()
        feedback = self._speed_check()
        if not self.source.poll_once():
            reason = self.source.estimator.last_rejection_reason or "unknown reason"
            raise CommunicationError(f"missing or invalid fresh 0x92 joint sample: {reason}")
        measurement = self.source.last_measurement
        theta = measurement.angle_degrees
        raw_angle = self.source.last_angle_degrees
        sample_time = self.source.last_sample_monotonic
        elapsed = sample_time - self.last_theta_time if self.last_theta_time is not None else None
        if self.continuous and not self.homing and elapsed is not None:
            if not math.isfinite(elapsed) or elapsed <= 0:
                raise CommunicationError("non-increasing 0x92 sample time")
            # Bag recording can delay polling. Compare angle travel to the
            # actual sample interval, while 0x9C independently enforces the
            # 80 RPM repeated / 90 RPM immediate speed limits.
            angle_limit = (CONTINUOUS_OVERSPEED_LIMIT_RPM * 6 * elapsed +
                           CONTINUOUS_ANGLE_TOLERANCE_DEG)
        else:
            angle_limit = HOME_MAX_ANGLE_STEP_DEG if self.homing else SWEEP_MAX_ANGLE_STEP_DEG
        if self.last_theta_deg is not None and abs(theta - self.last_theta_deg) > angle_limit:
            raise CommunicationError(
                f"discontinuous joint angle feedback: {theta - self.last_theta_deg:+.2f} "
                f"deg exceeds {angle_limit:.2f} deg over "
                f"{elapsed:.3f} s" if elapsed is not None else
                "discontinuous joint angle feedback")
        self.last_theta_deg = theta
        self.last_theta_time = sample_time
        travel_limit = 181.0 if self.homing else AMPLITUDE_DEG + 1.0
        if (self.homing or not self.continuous) and abs(theta - self.home_turn_deg) > travel_limit:
            raise CommunicationError("joint travelled outside the current HOME sweep limit")
        if not self.homing:
            if self.continuous:
                self._continuous_step(theta)
            else:
                self._sweep_step(theta, raw_angle, feedback)
            return
        if abs(theta - self.target_deg) <= HOME_TOLERANCE_DEG and abs(feedback.speed_dps) <= 3:
            self.endpoint_samples += 1
            if self.endpoint_samples >= 3:
                if self.homing:
                    self.homing = False
                    self.target_deg = self.home_turn_deg + (360.0 if self.continuous else AMPLITUDE_DEG)
                    if self.continuous:
                        self.rotation_origin_deg = theta
                        print("Initial HOME confirmed by controller feedback; "
                              "starting continuous positive joint rotation", flush=True)
                    else:
                        reply = self.motor.exchange(0x81)
                        if reply != b"\x81" + b"\x00" * 7:
                            raise CommunicationError("HOME STOP acknowledgement mismatch")
                        self.enabled = False
                        print("Initial HOME confirmed by controller feedback; "
                              f"starting ±{AMPLITUDE_DEG:g} degree A2 oscillation", flush=True)
                    self._begin_probe(theta)
                    return
                self.endpoint_samples = 0
                self.last_progress_time = None
        else:
            self.endpoint_samples = 0

        error = self.target_deg - theta
        # Detect a stalled or backwards-moving mechanism before prolonged force.
        now = self.clock()
        if self.last_progress_time is None:
            self.last_progress_time, self.last_progress_angle = now, theta
        elif now - self.last_progress_time >= 5.0:
            progress = math.copysign(1.0, error) * (theta - self.last_progress_angle)
            if abs(error) > 2.0 and progress < 0.5:
                raise CommunicationError("no forward joint progress within 5 seconds")
            self.last_progress_time, self.last_progress_angle = now, theta

        approaching = abs(error) <= HOME_APPROACH_START_DEG
        lead_limit = (HOME_APPROACH_LEAD_DEG if approaching or
                      abs(feedback.speed_dps) > HOME_SPEED_DPS else POSITION_LEAD_DEG)
        speed_limit = HOME_APPROACH_SPEED_DPS if approaching else HOME_SPEED_DPS
        lead = max(-lead_limit, min(lead_limit, error))
        target_raw = (round(raw_angle * 100) +
                      round(self.source.estimator.direction_sign * lead * 100))
        reply = self.motor.exchange(0xA4, position_hold_payload(target_raw, speed_limit))
        decode_feedback(reply, 0xA4)


def run(*, device="/dev/ttyUSB0", can_bitrate=1_000_000, motor_id=1,
        direction_sign=1, sweep_rpm=DEFAULT_SWEEP_RPM, continuous=False,
        period=0.025, timeout=0.2, max_runtime_s=300.0,
        motor_factory=WaveshareMotor, publisher_factory=RclpyJointStatePublisher,
        monotonic_clock=time.monotonic, sleep=time.sleep,
        utc_clock=lambda: datetime.now(timezone.utc)) -> int:
    if motor_id != 1 or can_bitrate not in (500_000, 1_000_000):
        raise ValueError("motor must be CAN ID 0x141 at 500000 or 1000000 bit/s")
    if direction_sign not in (-1, 1):
        raise ValueError("direction_sign must be +1 or -1")
    max_command_rpm = CONTINUOUS_MAX_COMMAND_RPM if continuous else 20.0
    if not math.isfinite(sweep_rpm) or not EDGE_RPM <= sweep_rpm <= max_command_rpm:
        raise ValueError(f"sweep RPM must be between 10 and {max_command_rpm:g}")
    if not 0.02 <= period <= 0.05 or not 0.05 <= timeout <= 0.5:
        raise ValueError("period must be 0.02..0.05 s and timeout 0.05..0.5 s")
    if not math.isfinite(max_runtime_s) or not (1 <= max_runtime_s <= 600 or
                                               (continuous and max_runtime_s == 0)):
        raise ValueError("max_runtime_s must be 1..600 seconds, or 0 for continuous rotation")
    publisher = publisher_factory(node_name="mhf9025_oscillating_motor")
    previous_handlers = {}
    shutdown_requested = False
    def _request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        # rclpy may keep its context alive after ROS launch forwards SIGINT.
        # Raise in the main thread so the motor cleanup finally block runs.
        if not shutdown_requested:
            shutdown_requested = True
            raise KeyboardInterrupt
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, _request_shutdown)
    except ValueError:
        # Mocked callers may run in a non-main thread; Python owns its signals.
        pass
    cleanup_ok = False
    try:
        with motor_factory(device, motor_id, can_bitrate, 2_000_000, timeout) as motor:
            controller = OscillatingMotor(motor, publisher, direction_sign=direction_sign,
                                          sweep_rpm=sweep_rpm, continuous=continuous,
                                          period=period, monotonic_clock=monotonic_clock,
                                          sleep=sleep, utc_clock=utc_clock)
            try:
                controller.preflight()
                if publisher.running():
                    controller.enable()
                    started = monotonic_clock()
                    next_poll = started + period
                    while max_runtime_s == 0 or monotonic_clock() - started < max_runtime_s:
                        sleep(max(0.0, next_poll - monotonic_clock()))
                        if not publisher.running():
                            break
                        controller.step()
                        # A delayed iteration must not trigger rapid catch-up
                        # polls: intervals below 10 ms cannot yield reliable
                        # angle-derived velocity and previously stopped motion.
                        next_poll = max(next_poll + period,
                                        monotonic_clock() + period)
            finally:
                cleanup_ok = motor.stop_and_disable()
                if not cleanup_ok:
                    print("STOP/DISABLE not acknowledged; use independent power cutoff. "
                          "Physical stop is not guaranteed by software.", file=sys.stderr)
                else:
                    print("STOP/DISABLE acknowledged; verify physical stopping independently.",
                          flush=True)
    except KeyboardInterrupt:
        return 0 if cleanup_ok else 1
    finally:
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
        publisher.close()
    return 0 if cleanup_ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--can-bitrate", type=int, default=1_000_000)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--direction-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--sweep-rpm", type=float, default=DEFAULT_SWEEP_RPM)
    parser.add_argument("--continuous", action="store_true",
                        help="rotate through successive 360-degree turns after HOME")
    parser.add_argument("--period", type=float, default=0.025)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--max-runtime-s", type=float, default=300.0,
                        help="runtime limit in seconds; 0 means unlimited in continuous mode")
    cli_args = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in cli_args:
        from rclpy.utilities import remove_ros_args
        cli_args = remove_ros_args(["oscillating_motor_node", *cli_args])[1:]
    args = parser.parse_args(cli_args)
    try:
        return run(**vars(args))
    except (MotorError, OSError, RuntimeError, ValueError) as exc:
        print(f"Oscillation stopped: {type(exc).__name__}: {exc}. "
              "Physical stop is not guaranteed by software.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
