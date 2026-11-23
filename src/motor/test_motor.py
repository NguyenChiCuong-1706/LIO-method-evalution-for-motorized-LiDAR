"""Read status and run short, explicitly authorized speed tests."""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from motor_can import (CommunicationError, ControllerFault, MotorCAN, MotorError,
                       check_socketcan, speed_payload)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("status", "positive", "negative", "both", "stop", "dry-run"),
                        default="status")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1_000_000,
                        choices=(100_000, 125_000, 250_000, 500_000, 1_000_000),
                        help="expected live CAN bitrate in bit/s; does not reconfigure the interface")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--speed", type=float, default=5.0, help="magnitude in RPM (0 < speed <= 10)")
    parser.add_argument("--duration", type=float, default=2.0, help="seconds (0 < duration <= 5)")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--ack-protocol-compatibility", action="store_true",
                        help="acknowledge that MHF9025-V3 motion compatibility is provisional")
    parser.add_argument("--dry-run", action="store_true", help="print frames without opening CAN")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    if not 1 <= args.motor_id <= 32:
        parser.error("--motor-id must be 1–32")
    if not math.isfinite(args.speed) or not 0 < args.speed <= 10:
        parser.error("--speed must be greater than 0 and at most 10 RPM")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 5:
        parser.error("--duration must be greater than 0 and at most 5 seconds")
    if args.mode in ("positive", "negative", "both") and not args.dry_run:
        if not args.allow_motion or not args.ack_protocol_compatibility:
            parser.error("motion requires --allow-motion and --ack-protocol-compatibility")
    return args


def print_status(status, feedback):
    if not status.layout_valid or not 5 <= status.voltage_v <= 60:
        print(f"Status1 raw: {status.raw.hex(' ').upper() if status.raw else 'unavailable'}")
        print("Status1: voltage=UNKNOWN fault_state=UNKNOWN "
              "(incompatible or unverified 0x9A layout)")
        print_feedback("Status2", feedback)
        return
    flags = []
    if status.undervoltage:
        flags.append("undervoltage")
    if status.overtemperature:
        flags.append("overtemperature")
    if status.unknown_error_bits:
        flags.append(f"unknown=0x{status.unknown_error_bits:02X}")
    print(f"Status1: temperature={status.temperature_c} C voltage={status.voltage_v:.1f} V "
          f"error=0x{status.error:02X} flags={','.join(flags) or 'none'}")
    print_feedback("Status2", feedback)


def print_feedback(label, feedback):
    print(f"{label}: temperature={feedback.temperature_c} C "
          f"torque_current_raw={feedback.torque_current_raw} "
          f"speed={feedback.speed_dps} deg/s ({feedback.speed_rpm:.2f} RPM) "
          f"encoder_raw={feedback.encoder_raw}")


def probe(motor):
    status = motor.read_status1()
    feedback = motor.read_status2()
    print_status(status, feedback)
    if not (status.layout_valid and
            -40 <= status.temperature_c <= 125 and
            -40 <= feedback.temperature_c <= 125 and
            abs(status.temperature_c - feedback.temperature_c) <= 20 and
            5 <= status.voltage_v <= 60):
        raise CommunicationError("unknown or incompatible 0x9A layout; voltage/fault state "
                                 "cannot be verified; motion blocked")
    print("Communication: OK (read-only 0x9A and 0x9C replies)")
    if status.error:
        raise ControllerFault(f"controller error=0x{status.error:02X}; motion blocked")
    return status, feedback


def dry_run(args):
    ident = 0x140 + args.motor_id
    sequence = [("read status 1", 0x9A, b"\x00" * 7),
                ("read status 2", 0x9C, b"\x00" * 7),
                ("enable", 0x88, b"\x00" * 7),
                ("positive speed", 0xA2, speed_payload(args.speed)),
                ("stop", 0x81, b"\x00" * 7),
                ("negative speed", 0xA2, speed_payload(-args.speed)),
                ("stop", 0x81, b"\x00" * 7),
                ("disable", 0x80, b"\x00" * 7)]
    print(f"DRY RUN: interface={args.channel}, motor ID={args.motor_id}, "
          f"speed=±{args.speed:g} RPM, duration={args.duration:g}s per direction; no CAN bus opened")
    for label, command, payload in sequence:
        print(f"{label}: ID=0x{ident:03X} DATA={(bytes([command]) + payload).hex(' ').upper()}")


def stop_and_disable(motor):
    errors = []
    for label, action in (("stop", motor.stop), ("disable", motor.disable)):
        try:
            action()
            print(f"{label}: acknowledged")
        except (MotorError, OSError, KeyboardInterrupt, InterruptedError) as exc:
            print(f"{label}: NOT confirmed ({exc})", file=sys.stderr)
            errors.append(exc)
    return not errors


def monitor(motor, rpm, duration):
    deadline = time.monotonic() + duration
    observed = False
    while True:
        feedback = motor.get_feedback()
        print_feedback("Motion feedback", feedback)
        if feedback.speed_dps * rpm > 0 and abs(feedback.speed_dps) >= 2:
            observed = True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.2, remaining))
    print(f"Motion feedback: {'direction observed' if observed else 'no confirmed rotation'}")
    return observed


def confirm_stopped(motor, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        feedback = motor.get_feedback()
        print_feedback("Stop feedback", feedback)
        if abs(feedback.speed_dps) <= 1:
            print("Stopped: confirmed by speed feedback")
            return
        time.sleep(0.2)
    raise CommunicationError("stop was acknowledged but zero speed was not confirmed")


def run_motion(motor, args):
    # The guard is set before enable, so every subsequent failure attempts cleanup.
    enable_attempted = False
    completed = False
    try:
        enable_attempted = True
        motor.enable()
        print("enable: acknowledged")
        directions = ([args.speed, -args.speed] if args.mode == "both" else
                      [args.speed if args.mode == "positive" else -args.speed])
        for index, rpm in enumerate(directions):
            feedback = motor.set_speed_rpm(rpm)
            print_feedback(f"Speed command {rpm:+g} RPM response", feedback)
            observed = monitor(motor, rpm, args.duration)
            motor.stop()
            print("stop: acknowledged")
            confirm_stopped(motor)
            if not observed:
                raise CommunicationError("command acknowledged, but matching rotation was not observed")
            if index + 1 < len(directions):
                time.sleep(2.0)
        completed = True
    finally:
        if enable_attempted:
            cleanup_ok = stop_and_disable(motor)
            if completed and not cleanup_ok:
                raise CommunicationError("motion completed but cleanup was not confirmed")


def _terminate(_signum, _frame):
    raise InterruptedError("SIGTERM received")


def main(argv=None):
    args = parse_args(argv)
    if args.mode == "dry-run" or args.dry_run:
        dry_run(args)
        return 0
    motor = MotorCAN(args.channel, args.motor_id, args.verbose)
    old_handler = signal.signal(signal.SIGTERM, _terminate)
    try:
        if args.mode != "stop":
            check_socketcan(args.channel, expected_bitrate=args.bitrate)
            print(f"SocketCAN {args.channel}: UP, {args.bitrate} bit/s")
        motor.connect()
        if args.mode == "stop":
            return 0 if stop_and_disable(motor) else 1
        probe(motor)
        if args.mode != "status":
            run_motion(motor, args)
        return 0
    except (MotorError, ValueError, KeyboardInterrupt, InterruptedError) as exc:
        print(f"ERROR: {exc}; physical stop is not guaranteed by software", file=sys.stderr)
        return 1
    finally:
        try:
            motor.disconnect()
        finally:
            signal.signal(signal.SIGTERM, old_handler)


if __name__ == "__main__":
    sys.exit(main())
