"""One-shot, read-only MHF9025 status probe for the Holtek USB-CAN bridge."""

import argparse
import fcntl
import os
import select
import sys
import termios
import time
import tty


DEFAULT_DEVICE = "/dev/serial/by-id/usb-HOLTEK_USB_TO_CAN_BRIDGE_0000-if00"
BITRATE_COMMAND = {500000: b"S\x07\r", 1000000: b"S\x09\r"}
STATUS_REQUEST = b"t\x01\x41\x08\x9a" + bytes(7) + b"\r"


def find_motor_reply(raw):
    """Find a complete standard-ID 0x141, DLC-8, 0x9A response."""
    prefix = b"t\x01\x41\x08\x9a"
    for start in range(len(raw)):
        if raw.startswith(prefix, start) and len(raw) >= start + 13:
            frame = raw[start : start + 13]
            if frame[-1:] == b"\r":
                return frame
    # Accept ASCII firmware variants only when a whole frame is present.
    return next(
        (line for line in raw.split(b"\r") if line.startswith(b"t14189A") and len(line) == 21),
        None,
    )


def status_hypotheses(raw):
    """Interpret Fxx provisionally using the Bosch C_CAN status bit layout."""
    values = []
    for line in raw.split(b"\r"):
        if len(line) == 3 and line[:1] == b"F":
            try:
                value = int(line[1:].decode("ascii"), 16)
            except ValueError:
                continue
            flags = []
            if value & 0x80:
                flags.append("bus-off")
            if value & 0x40:
                flags.append("error-warning")
            if value & 0x20:
                flags.append("error-passive")
            if value & 7 == 3:
                flags.append("missing CAN ACK")
            values.append((value, flags))
    return values


def receive(fd, timeout):
    deadline = time.monotonic() + timeout
    result = bytearray()
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], max(0, deadline - time.monotonic()))
        if readable:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                break
            result.extend(chunk)
    return bytes(result)


def exchange(fd, label, request, timeout=0.3):
    print(f"TX {label}: {request!r}", flush=True)
    os.write(fd, request)
    reply = receive(fd, timeout)
    print(f"RX {label}: {reply!r}", flush=True)
    return reply


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--can-bitrate", type=int, choices=BITRATE_COMMAND, default=500000)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()
    if not 0.1 <= args.timeout <= 10.0:
        parser.error("--timeout must be between 0.1 and 10 seconds")

    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    old = None
    opened = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        receive(fd, 0.1)

        version = exchange(fd, "adapter version", b"v\r")
        if not any(line.startswith(b"v") for line in version.split(b"\r")):
            print("No Holtek version response; CAN request was not sent.")
            return 2
        print("Note: this firmware also returns CR for an invalid bitrate code; CR is not proof of bitrate configuration.")
        for label, request in (
            ("close CAN for bitrate setup", b"C\r"),
            (f"set CAN {args.can_bitrate}", BITRATE_COMMAND[args.can_bitrate]),
            ("open CAN normal", b"O\r"),
        ):
            if request == b"O\r":
                opened = True
            if exchange(fd, label, request) != b"\r":
                print(f"Adapter did not acknowledge {label}; motor request was not sent.")
                return 2
        reply = exchange(fd, "read-only motor status 0x9A", STATUS_REQUEST, args.timeout)
        motor_reply = find_motor_reply(reply)
        status_reply = exchange(fd, "adapter CAN status", b"F\r")
        for value, flags in status_hypotheses(reply + status_reply):
            print(
                f"F{value:02X} provisional C_CAN interpretation: "
                + (", ".join(flags) if flags else "no decoded error flag")
                + "; Holtek Fxx bit mapping is unverified"
            )
        if motor_reply:
            print(f"Motor responded: {motor_reply.hex(' ').upper()}")
            return 0
        print("No valid 0x9A motor response. Adapter 'z' also occurs for malformed frames in loopback; it does not prove motor receipt.")
        return 1
    finally:
        if opened:
            try:
                exchange(fd, "close CAN", b"C\r")
            except OSError as exc:
                print(f"Could not close adapter CAN channel: {exc}", file=sys.stderr)
        if old is not None:
            termios.tcsetattr(fd, termios.TCSANOW, old)
        os.close(fd)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, KeyboardInterrupt) as exc:
        print(f"Probe stopped: {exc}", file=sys.stderr)
        sys.exit(2)
