"""One read-only MHF9025 status probe through a Waveshare USB-CAN-A adapter.

The USB-CAN-A appears as a CH340 serial port, not a SocketCAN interface.
This program configures only the adapter, sends one CAN status request, and
prints every received serial/CAN frame until a bounded deadline.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time

from motor_can import decode_feedback, decode_mhf9025_v3_status1


CAN_SPEED_CODE = {1_000_000: 0x01, 500_000: 0x03, 250_000: 0x05,
                  125_000: 0x07, 100_000: 0x08}
SERIAL_SPEED = {2_000_000: termios.B2000000, 1_152_000: termios.B1152000,
                115_200: termios.B115200}


def describe_status1(data: bytes, can_id: int = 0x141) -> list[str]:
    """Describe MHF9025-V3 voltage and keep unverified fault fields raw."""
    status = decode_mhf9025_v3_status1(data, can_id=can_id, expected_can_id=can_id)
    lines = [f"MHF9025-V3 experimental decode: temperature={status.temperature_c} C, "
             f"voltage=uint16_LE(bytes 2-3)/100={status.voltage_v:.2f} V "
             "(supported by one independent voltage measurement)",
             f"Fault_state=UNKNOWN; raw byte6=0x{data[6]:02X}, "
             f"raw byte7=0x{data[7]:02X}; protection-bit mapping unverified"]
    for start in (2, 3, 4, 5):
        pair = data[start:start + 2]
        le = int.from_bytes(pair, 'little')
        be = int.from_bytes(pair, 'big')
        lines.append(f"Candidate bytes {start}-{start + 1}: "
                     f"LE={le} BE={be} (raw; units unverified)")
        if start in (2, 3):
            lines.append(f"  Numerical comparisons: LE/10={le / 10:.2f}, "
                         f"LE/100={le / 100:.2f}, BE/10={be / 10:.2f}, "
                         f"BE/100={be / 100:.2f} V")
    lines.append("Legacy MF9025-family bytes 3-4 /10 would yield "
                 f"{int.from_bytes(data[3:5], 'little') / 10:.1f} V; "
                 "this is not the active MHF9025-V3 decoder.")
    return lines


def describe_status2(data: bytes) -> list[str]:
    feedback = decode_feedback(data)
    return [f"0x9C provisional fields: temperature_raw={data[1]} "
            f"torque_current_raw={feedback.torque_current_raw} "
            f"speed_raw={feedback.speed_dps} deg/s "
            f"speed={feedback.speed_rpm:.2f} RPM "
            f"encoder_raw={feedback.encoder_raw}",
            ("Zero raw speed is consistent with a stationary motor; "
             if feedback.speed_dps == 0 else
             "This nonzero raw sample alone does not establish shaft motion; ") +
            "nonzero speed scaling is not yet verified."]


def describe_frame(kind: str, can_id: int, data: bytes, timestamp: float) -> list[str]:
    lines = [f"CAN RX: timestamp={timestamp:.6f} (host receipt time) "
             f"type={kind} ID=0x{can_id:X} DLC={len(data)} "
             f"DATA={data.hex(' ').upper()}"]
    lines.extend(f"  Byte {index}: 0x{value:02X}" for index, value in enumerate(data))
    return lines


def settings_frame(can_bitrate: int) -> bytes:
    """Waveshare canusb.c 20-byte settings command: normal, standard, all IDs."""
    if can_bitrate not in CAN_SPEED_CODE:
        raise ValueError("unsupported CAN bitrate")
    frame = bytearray.fromhex("AA 55 12 00 01")
    frame[3] = CAN_SPEED_CODE[can_bitrate]
    frame.extend(b"\x00" * 8)  # Filter ID and mask ID both zero.
    frame.extend(b"\x00\x01")  # Normal mode; start channel.
    frame.extend(b"\x00" * 4)
    frame.append(sum(frame[2:]) & 0xFF)
    assert len(frame) == 20
    return bytes(frame)


def status_serial_frame(motor_id: int, command: int = 0x9A) -> bytes:
    if not 1 <= motor_id <= 32:
        raise ValueError("motor ID must be 1–32")
    if command not in (0x9A, 0x9C):
        raise ValueError("only read-only status commands 0x9A and 0x9C are allowed")
    can_id = 0x140 + motor_id
    return bytes([0xAA, 0xC8, can_id & 0xFF, can_id >> 8, command]) + b"\x00" * 7 + b"\x55"


def extract_frames(buffer: bytearray):
    """Yield complete Waveshare variable CAN packets; skip config replies/noise."""
    while buffer:
        if buffer[0] != 0xAA:
            del buffer[0]
            continue
        if len(buffer) < 2:
            return
        marker = buffer[1]
        if marker == 0x55:
            length = 20
            if len(buffer) < length:
                return
            packet = bytes(buffer[:length])
            del buffer[:length]
            yield ("adapter", None, packet)
            continue
        if marker & 0xC0 != 0xC0 or marker & 0x0F > 8:
            del buffer[0]
            continue
        extended = bool(marker & 0x20)
        id_length = 4 if extended else 2
        data_length = marker & 0x0F
        length = 2 + id_length + data_length + 1
        if len(buffer) < length:
            return
        packet = bytes(buffer[:length])
        if packet[-1] != 0x55:
            del buffer[0]
            continue
        del buffer[:length]
        can_id = int.from_bytes(packet[2:2 + id_length], "little")
        data = packet[2 + id_length:-1]
        yield ("extended" if extended else "standard", can_id, data)


def probe(device: str, can_bitrate: int, serial_baud: int, motor_id: int, command: int,
          timeout: float) -> bool:
    config = settings_frame(can_bitrate)
    request = status_serial_frame(motor_id, command)
    can_id = 0x140 + motor_id
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    found = False
    received = 0
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = SERIAL_SPEED[serial_baud]
        attrs[5] = SERIAL_SPEED[serial_baud]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        print(f"Adapter: {device} (USB serial bridge; SocketCAN CAN counters unavailable)")
        print("Before probe: CAN controller packet/error statistics unavailable via this adapter")
        print(f"Adapter setup: CAN {can_bitrate} bit/s, serial {serial_baud} bit/s, "
              f"normal mode, standard frames; UART={config.hex(' ').upper()}")
        os.write(fd, config)
        termios.tcdrain(fd)
        time.sleep(0.1)
        print(f"Read-only CAN request: ID=0x{can_id:03X} "
              f"DATA={bytes([command]).hex().upper()} 00 00 00 00 00 00 00; "
              f"UART={request.hex(' ').upper()}")
        os.write(fd, request)
        termios.tcdrain(fd)
        buffer = bytearray()
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            if not select.select([fd], [], [], remaining)[0]:
                break
            chunk = os.read(fd, 512)
            if not chunk:
                continue
            buffer.extend(chunk)
            for kind, incoming_id, data in extract_frames(buffer):
                received += 1
                if kind == "adapter":
                    print(f"Adapter reply: {data.hex(' ').upper()}")
                    continue
                for line in describe_frame(kind, incoming_id, data, time.time()):
                    print(line)
                if kind == "standard" and incoming_id == can_id and len(data) == 8 and data[0] == command:
                    if command == 0x9A:
                        for line in describe_status1(data, incoming_id):
                            print(line)
                    else:
                        for line in describe_status2(data):
                            print(line)
                    found = True
                    break
            if found:
                break
        if buffer:
            print(f"Unparsed serial bytes: {buffer.hex(' ').upper()}")
        print(f"Matching 0x{command:02X} response: {'yes' if found else 'no'}; "
              f"decoded packets={received}")
        print("After probe: CAN controller packet/error statistics unavailable via this adapter")
        return found
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original)
        os.close(fd)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--can-bitrate", type=int, choices=tuple(CAN_SPEED_CODE), default=500_000)
    parser.add_argument("--serial-baud", type=int, choices=tuple(SERIAL_SPEED), default=2_000_000)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--status", type=int, choices=(1, 2), default=1,
                        help="1 sends 0x9A; 2 sends 0x9C")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if not 1 <= args.motor_id <= 32:
        parser.error("--motor-id must be 1–32")
    if not 0 < args.timeout <= 5:
        parser.error("--timeout must be greater than zero and at most 5 seconds")
    if args.dry_run:
        print(f"Adapter setup UART: {settings_frame(args.can_bitrate).hex(' ').upper()}")
        command = 0x9A if args.status == 1 else 0x9C
        print(f"One status request UART: {status_serial_frame(args.motor_id, command).hex(' ').upper()}")
        print("No serial port opened and no CAN frame sent.")
        return 0
    try:
        command = 0x9A if args.status == 1 else 0x9C
        return 0 if probe(args.device, args.can_bitrate, args.serial_baud,
                          args.motor_id, command, args.timeout) else 2
    except (OSError, ValueError, termios.error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
