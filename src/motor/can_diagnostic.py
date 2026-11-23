"""Read-only and single-probe diagnostics for a Linux SocketCAN interface."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
import subprocess
import sys
import time

try:
    import can
except ImportError:
    can = None


STATUS_COMMAND = 0x9A
SUPPORTED_BITRATES = (100_000, 125_000, 250_000, 500_000, 1_000_000)


class DiagnosticError(Exception):
    """A diagnostic could not be completed safely or reliably."""


@dataclass(frozen=True)
class CanSnapshot:
    interface: str
    operstate: str
    flags: tuple[str, ...]
    controller_state: str
    bitrate: int | None
    driver: str
    listen_only_supported: bool
    rx_packets: int
    rx_bytes: int
    rx_errors: int
    rx_dropped: int
    tx_packets: int
    tx_bytes: int
    tx_errors: int
    tx_dropped: int
    restarts: int
    bus_errors: int
    arbitration_lost: int
    error_warning: int
    error_passive: int
    bus_off: int

    @property
    def is_up(self) -> bool:
        return self.operstate.upper() == "UP" and "UP" in self.flags

    @property
    def healthy_for_probe(self) -> bool:
        return self.is_up and self.controller_state.upper() == "ERROR-ACTIVE"


def _nonnegative(mapping, key: str) -> int:
    value = mapping.get(key, 0)
    return int(value) if isinstance(value, (int, float)) and value >= 0 else 0


def parse_ip_json(payload: str, interface: str) -> CanSnapshot:
    try:
        records = json.loads(payload)
        record = records[0]
        linkinfo = record["linkinfo"]
        data = linkinfo["info_data"]
    except (json.JSONDecodeError, IndexError, KeyError, TypeError) as exc:
        raise DiagnosticError("ip did not return usable CAN interface data") from exc
    if record.get("ifname") != interface or linkinfo.get("info_kind") != "can":
        raise DiagnosticError(f"{interface} is not reported as a CAN interface")
    stats = record.get("stats64", {})
    rx = stats.get("rx", {})
    tx = stats.get("tx", {})
    xstats = linkinfo.get("info_xstats", {})
    supported = {str(item).upper() for item in data.get("ctrlmode_supported", [])}
    driver = str(data.get("bittiming_const", {}).get("name", "unknown"))
    bitrate = data.get("bittiming", {}).get("bitrate")
    return CanSnapshot(
        interface=interface,
        operstate=str(record.get("operstate", "UNKNOWN")),
        flags=tuple(str(flag) for flag in record.get("flags", [])),
        controller_state=str(data.get("state", "UNKNOWN")),
        bitrate=int(bitrate) if isinstance(bitrate, (int, float)) else None,
        driver=driver,
        listen_only_supported="LISTEN-ONLY" in supported,
        rx_packets=_nonnegative(rx, "packets"), rx_bytes=_nonnegative(rx, "bytes"),
        rx_errors=_nonnegative(rx, "errors"), rx_dropped=_nonnegative(rx, "dropped"),
        tx_packets=_nonnegative(tx, "packets"), tx_bytes=_nonnegative(tx, "bytes"),
        tx_errors=_nonnegative(tx, "errors"), tx_dropped=_nonnegative(tx, "dropped"),
        restarts=_nonnegative(xstats, "restarts"),
        bus_errors=_nonnegative(xstats, "bus_error"),
        arbitration_lost=_nonnegative(xstats, "arbitration_lost"),
        error_warning=_nonnegative(xstats, "error_warning"),
        error_passive=_nonnegative(xstats, "error_passive"),
        bus_off=_nonnegative(xstats, "bus_off"),
    )


def read_snapshot(interface: str) -> CanSnapshot:
    try:
        result = subprocess.run(
            ["ip", "-details", "-statistics", "-json", "link", "show", "dev", interface],
            check=True, capture_output=True, text=True,
        )
    except FileNotFoundError as exc:
        raise DiagnosticError("the 'ip' command is not installed") from exc
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout or str(exc)).strip()
        raise DiagnosticError(f"cannot inspect {interface}: {detail}") from exc
    return parse_ip_json(result.stdout, interface)


def print_snapshot(snapshot: CanSnapshot, label: str = "Interface") -> None:
    bitrate = str(snapshot.bitrate) if snapshot.bitrate is not None else "unknown"
    print(f"{label}: {snapshot.interface}")
    print(f"  operational state: {snapshot.operstate} (flags: {','.join(snapshot.flags)})")
    print(f"  CAN controller state: {snapshot.controller_state}")
    print(f"  bitrate: {bitrate} bit/s")
    print(f"  driver: {snapshot.driver}")
    print(f"  listen-only supported: {'yes' if snapshot.listen_only_supported else 'no'}")
    print(f"  RX: packets={snapshot.rx_packets} bytes={snapshot.rx_bytes} "
          f"errors={snapshot.rx_errors} dropped={snapshot.rx_dropped}")
    print(f"  TX: packets={snapshot.tx_packets} bytes={snapshot.tx_bytes} "
          f"errors={snapshot.tx_errors} dropped={snapshot.tx_dropped}")
    print(f"  CAN errors: warning={snapshot.error_warning} passive={snapshot.error_passive} "
          f"bus={snapshot.bus_errors} bus-off={snapshot.bus_off} "
          f"arbitration-lost={snapshot.arbitration_lost} restarts={snapshot.restarts}")


def counter_delta(before: CanSnapshot, after: CanSnapshot) -> dict[str, int]:
    names = ("rx_packets", "tx_packets", "rx_errors", "tx_errors", "rx_dropped",
             "tx_dropped", "bus_errors", "error_warning", "error_passive", "bus_off")
    return {name: getattr(after, name) - getattr(before, name) for name in names}


def classify(before: CanSnapshot, after: CanSnapshot | None = None,
             valid_response: bool = False) -> list[str]:
    after = after or before
    delta = counter_delta(before, after)
    findings = []
    state = after.controller_state.upper()
    if valid_response:
        findings.append("A valid status response was received; CAN wiring, bitrate, ID, and this read command worked for this probe.")
        return findings
    if state == "BUS-OFF" or delta["bus_off"] > 0:
        findings.append("The CAN controller is BUS-OFF; repair the bus and manually cycle the interface before another probe.")
    if (state in ("ERROR-WARNING", "ERROR-PASSIVE") or
            delta["error_warning"] > 0 or delta["error_passive"] > 0 or
            delta["bus_errors"] > 0):
        findings.append("The adapter accumulated CAN-layer errors without a response. Inspect error notifications to distinguish protocol errors from missing ACK.")
        findings.append("Conditional causes: motor/controller unpowered, CAN H/L wiring or common-ground fault, inactive transceiver, bitrate mismatch, or termination.")
        findings.append("Motor ID and application protocol do not explain CAN-layer errors; every active CAN node at the correct bitrate acknowledges a valid frame regardless of ID.")
    elif delta["tx_packets"] > 0:
        findings.append("SocketCAN recorded a submitted TX frame but no valid response. Submission alone does not prove that another node acknowledged it.")
        findings.append("With stable ERROR-ACTIVE counters, wrong motor ID or incompatible application protocol remains possible, but the available evidence is not conclusive.")
    elif not after.is_up:
        findings.append("The interface is not UP, so no bus diagnosis can be made.")
    else:
        findings.append("No probe delta is available; current counters alone cannot determine wiring, bitrate, motor ID, or protocol compatibility.")
    return findings


def print_analysis(before: CanSnapshot, after: CanSnapshot | None = None,
                   valid_response: bool = False) -> None:
    if after is not None:
        delta = counter_delta(before, after)
        print("Counter changes:")
        for name, value in delta.items():
            print(f"  {name}: {value:+d}")
    print("Analysis:")
    for finding in classify(before, after, valid_response):
        print(f"  - {finding}")


def format_message(message) -> str:
    if message.is_error_frame:
        frame_type = "CAN error notification"
        details = describe_can_error(message)
        if details:
            frame_type += f" ({', '.join(details)})"
    else:
        frame_type = "extended" if message.is_extended_id else "standard"
    return (f"{message.timestamp:.6f} ID=0x{message.arbitration_id:X} "
            f"DLC={message.dlc} DATA={bytes(message.data).hex(' ').upper()} {frame_type}")


def describe_can_error(message) -> list[str]:
    """Decode Linux CAN error classes and the relevant controller/protocol bits."""
    if not message.is_error_frame:
        return []
    ident = message.arbitration_id
    data = bytes(message.data)
    details = []
    if ident & 0x20:
        details.append("no ACK")
    if ident & 0x40:
        details.append("bus off")
    if ident & 0x04 and len(data) > 1:
        flags = data[1]
        if flags & 0x04:
            details.append("RX warning")
        if flags & 0x08:
            details.append("TX warning")
        if flags & 0x10:
            details.append("RX passive")
        if flags & 0x20:
            details.append("TX passive")
    if ident & 0x08 and len(data) > 2:
        flags = data[2]
        if flags & 0x01:
            details.append("bit error")
        if flags & 0x02:
            details.append("form error")
        if flags & 0x04:
            details.append("bit stuffing error")
    return details


def open_bus(interface: str):
    if can is None:
        raise DiagnosticError("python-can is missing; activate .venv and install requirements.txt")
    try:
        return can.Bus(interface="socketcan", channel=interface)
    except (can.CanError, OSError) as exc:
        raise DiagnosticError(f"cannot open {interface}: {exc}") from exc


def monitor(interface: str, timeout: float, max_display: int = 100) -> int:
    snapshot = read_snapshot(interface)
    print_snapshot(snapshot)
    print(f"Listening passively for {timeout:g}s; no frames will be transmitted.")
    bus = open_bus(interface)
    count = 0
    error_count = 0
    error_details = Counter()
    suppressed_notice = False
    deadline = time.monotonic() + timeout
    try:
        while (remaining := deadline - time.monotonic()) > 0:
            message = bus.recv(timeout=remaining)
            if message is None:
                break
            count += 1
            error_count += int(message.is_error_frame)
            error_details.update(describe_can_error(message))
            if count <= max_display:
                print(format_message(message))
            elif not suppressed_notice:
                print(f"Further frames suppressed after {max_display}; counting continues until timeout.")
                suppressed_notice = True
    except (can.CanError, OSError) as exc:
        raise DiagnosticError(f"CAN receive failed: {exc}") from exc
    finally:
        bus.shutdown()
    print(f"Passive monitor complete: {count} frame(s) observed, including "
          f"{error_count} kernel CAN error notification(s).")
    if error_count:
        print("Kernel CAN error notifications are local bus diagnostics, not messages from the motor.")
        print("Decoded error types: " + ", ".join(
            f"{label}={count}" for label, count in error_details.most_common())
            if error_details else "Decoded error types: unspecified")
    if count == 0:
        print("Silence does not identify the bitrate or prove that the motor is unpowered; many motor controllers transmit only when queried.")
    return count


def status_probe(interface: str, motor_id: int, timeout: float) -> bool:
    before = read_snapshot(interface)
    print_snapshot(before, "Before probe")
    if not before.healthy_for_probe:
        raise DiagnosticError(
            f"refusing to transmit: {interface} is {before.controller_state}, not ERROR-ACTIVE; "
            "repair the bus and manually cycle the interface first"
        )
    arbitration_id = 0x140 + motor_id
    request = bytes([STATUS_COMMAND]) + b"\x00" * 7
    bus = open_bus(interface)
    response = None
    send_succeeded = False
    received_count = 0
    error_count = 0
    error_details = Counter()
    try:
        message = can.Message(arbitration_id=arbitration_id, data=request,
                              is_extended_id=False, is_fd=False)
        bus.send(message, timeout=min(timeout, 0.5))
        send_succeeded = True
        print(f"Send call: succeeded (queued one standard frame ID=0x{arbitration_id:03X} "
              f"DATA={request.hex(' ').upper()})")
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            received = bus.recv(timeout=remaining)
            if received is None:
                break
            received_count += 1
            error_count += int(received.is_error_frame)
            error_details.update(describe_can_error(received))
            if received_count <= 20:
                print(f"RX: {format_message(received)}")
            elif received_count == 21:
                print("RX: further frames suppressed; counting continues until timeout")
            if (not received.is_extended_id and not received.is_error_frame and
                    received.arbitration_id == arbitration_id and
                    received.dlc == 8 and len(received.data) == 8 and
                    received.data[0] == STATUS_COMMAND):
                response = received
                break
    except (can.CanError, OSError) as exc:
        print(f"Send/receive error: {exc}")
    finally:
        bus.shutdown()
    time.sleep(0.1)
    after = read_snapshot(interface)
    print_snapshot(after, "After probe")
    print(f"Send submitted to SocketCAN: {'yes' if send_succeeded else 'no'}")
    print(f"Valid 0x9A response: {'yes' if response is not None else 'no'}")
    print(f"Received while waiting: {received_count} frame(s), including "
          f"{error_count} kernel CAN error notification(s)")
    if error_details:
        print("Decoded error types: " + ", ".join(
            f"{label}={count}" for label, count in error_details.most_common()))
    print("A successful send call means the kernel accepted the frame; it does not prove CAN ACK or motor receipt.")
    print_analysis(before, after, response is not None)
    return response is not None


def print_bitrate_plan(interface: str) -> None:
    snapshot = read_snapshot(interface)
    print_snapshot(snapshot)
    print("No interface settings were changed and no frames were transmitted.")
    print("Test one bitrate at a time manually. Listen-only is supported by this adapter." if snapshot.listen_only_supported
          else "This adapter did not report listen-only support; do not use these listen-only commands.")
    if not snapshot.listen_only_supported:
        print("Listen-only is unavailable, so no reconfiguration commands are emitted.")
        print("Passive silence would not identify a silent motor's bitrate in any case.")
        return
    for bitrate in SUPPORTED_BITRATES:
        print(f"\n# {bitrate} bit/s, passive only")
        print(f"sudo ip link set dev {interface} down")
        print(f"sudo ip link set dev {interface} type can bitrate {bitrate} listen-only on")
        print(f"sudo ip link set dev {interface} up")
        print(f"python can_diagnostic.py --mode monitor --channel {interface} --timeout 10")
    print(f"\n# Restore the initial 1 Mbps active configuration")
    print(f"sudo ip link set dev {interface} down")
    print(f"sudo ip link set dev {interface} type can bitrate 1000000 listen-only off")
    print(f"sudo ip link set dev {interface} up")
    print("A silent motor cannot be assigned a bitrate by passive listening alone. Do not probe every bitrate automatically.")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True,
                        choices=("inspect", "monitor", "probe", "analyze", "bitrates"))
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--max-display", type=int, default=100,
                        help="maximum monitor frames to print while continuing to count")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 60:
        parser.error("--timeout must be greater than 0 and at most 60 seconds")
    if not 1 <= args.motor_id <= 32:
        parser.error("--motor-id must be 1–32")
    if not 0 <= args.max_display <= 1000:
        parser.error("--max-display must be 0–1000")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        if args.mode == "inspect":
            print_snapshot(read_snapshot(args.channel))
        elif args.mode == "monitor":
            monitor(args.channel, args.timeout, args.max_display)
        elif args.mode == "probe":
            return 0 if status_probe(args.channel, args.motor_id, args.timeout) else 2
        elif args.mode == "analyze":
            snapshot = read_snapshot(args.channel)
            print_snapshot(snapshot)
            print_analysis(snapshot)
        else:
            print_bitrate_plan(args.channel)
        return 0
    except DiagnosticError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
