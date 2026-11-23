import io
import json
import pathlib
import sys
import unittest
from contextlib import redirect_stdout, redirect_stderr
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import can_diagnostic as diagnostic


def ip_payload(state="ERROR-ACTIVE", *, tx=0, rx=0, warning=0, passive=0,
               bus_off=0, bus_error=0):
    return json.dumps([{
        "ifname": "can0", "flags": ["NOARP", "UP", "LOWER_UP", "ECHO"],
        "operstate": "UP", "linkinfo": {"info_kind": "can", "info_data": {
            "state": state, "ctrlmode_supported": ["LOOPBACK", "LISTEN-ONLY"],
            "bittiming": {"bitrate": 1_000_000},
            "bittiming_const": {"name": "gs_usb"}},
            "info_xstats": {"restarts": 0, "bus_error": bus_error,
                            "arbitration_lost": 0, "error_warning": warning,
                            "error_passive": passive, "bus_off": bus_off}},
        "stats64": {"rx": {"bytes": rx * 8, "packets": rx, "errors": 0, "dropped": 0},
                    "tx": {"bytes": tx * 8, "packets": tx, "errors": 0, "dropped": 0}}
    }])


def snapshot(**kwargs):
    return diagnostic.parse_ip_json(ip_payload(**kwargs), "can0")


class FakeMessage:
    def __init__(self, arbitration_id=0x141,
                 data=b"\x9a\x19\x00\xf0\x00\x00\x00\x00",
                 is_extended_id=False, is_error_frame=False, timestamp=123.0,
                 is_fd=False):
        self.arbitration_id = arbitration_id
        self.data = data
        self.dlc = len(data)
        self.is_extended_id = is_extended_id
        self.is_error_frame = is_error_frame
        self.is_fd = is_fd
        self.timestamp = timestamp


class DiagnosticTests(unittest.TestCase):
    def test_parse_real_fields(self):
        item = snapshot(state="ERROR-PASSIVE", tx=1, rx=1, warning=4,
                        passive=4054380)
        self.assertEqual(item.driver, "gs_usb")
        self.assertEqual(item.bitrate, 1_000_000)
        self.assertTrue(item.listen_only_supported)
        self.assertFalse(item.healthy_for_probe)
        self.assertEqual((item.tx_packets, item.rx_packets, item.error_warning,
                          item.error_passive), (1, 1, 4, 4054380))

    def test_error_counter_increase_classification(self):
        before = snapshot()
        after = snapshot(state="ERROR-PASSIVE", tx=1, warning=1, passive=10)
        findings = " ".join(diagnostic.classify(before, after))
        self.assertIn("distinguish protocol errors from missing ACK", findings)
        self.assertIn("do not explain CAN-layer errors", findings)

    def test_bus_off_classification(self):
        before = snapshot()
        after = snapshot(state="BUS-OFF", tx=1, bus_off=1)
        self.assertIn("BUS-OFF", diagnostic.classify(before, after)[0])

    def test_valid_response_classification(self):
        before = snapshot()
        after = snapshot(tx=1, rx=1)
        self.assertIn("valid status response", diagnostic.classify(before, after, True)[0])

    def test_probe_refuses_unhealthy_interface_without_opening_bus(self):
        with patch.object(diagnostic, "read_snapshot", return_value=snapshot(state="ERROR-PASSIVE")), \
             patch.object(diagnostic, "open_bus", side_effect=AssertionError("opened")), \
             redirect_stdout(io.StringIO()), self.assertRaises(diagnostic.DiagnosticError):
            diagnostic.status_probe("can0", 1, 0.1)

    def test_mocked_status_response(self):
        before = snapshot()
        after = snapshot(tx=1, rx=1)
        bus = Mock()
        bus.recv.return_value = FakeMessage()
        fake_can = SimpleNamespace(Message=FakeMessage, CanError=RuntimeError)
        output = io.StringIO()
        with patch.object(diagnostic, "read_snapshot", side_effect=[before, after]), \
             patch.object(diagnostic, "open_bus", return_value=bus), \
             patch.object(diagnostic, "can", fake_can), \
             patch.object(diagnostic.time, "sleep"), redirect_stdout(output):
            self.assertTrue(diagnostic.status_probe("can0", 1, 0.1))
        sent = bus.send.call_args.args[0]
        self.assertEqual(sent.arbitration_id, 0x141)
        self.assertEqual(sent.data, b"\x9a" + b"\x00" * 7)
        self.assertIn("Valid 0x9A response: yes", output.getvalue())

    def test_probe_no_response_and_counter_increase(self):
        before = snapshot()
        after = snapshot(state="ERROR-PASSIVE", tx=1, warning=1, passive=3)
        bus = Mock()
        bus.recv.return_value = None
        fake_can = SimpleNamespace(Message=FakeMessage, CanError=RuntimeError)
        output = io.StringIO()
        with patch.object(diagnostic, "read_snapshot", side_effect=[before, after]), \
             patch.object(diagnostic, "open_bus", return_value=bus), \
             patch.object(diagnostic, "can", fake_can), \
             patch.object(diagnostic.time, "sleep"), redirect_stdout(output):
            self.assertFalse(diagnostic.status_probe("can0", 1, 0.1))
        self.assertIn("distinguish protocol errors from missing ACK", output.getvalue())

    def test_cli_validation(self):
        for argv in (["--mode", "monitor", "--timeout", "0"],
                     ["--mode", "probe", "--motor-id", "0"],
                     ["--mode", "monitor", "--max-display", "1001"]):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                diagnostic.parse_args(argv)

    def test_error_frame_label_and_monitor_output_limit(self):
        messages = [FakeMessage(arbitration_id=0xC, is_error_frame=True) for _ in range(3)]
        messages.append(None)
        bus = Mock()
        bus.recv.side_effect = messages
        output = io.StringIO()
        with patch.object(diagnostic, "read_snapshot", return_value=snapshot()), \
             patch.object(diagnostic, "open_bus", return_value=bus), \
             redirect_stdout(output):
            self.assertEqual(diagnostic.monitor("can0", 0.1, max_display=1), 3)
        rendered = output.getvalue()
        self.assertIn("CAN error notification", rendered)
        self.assertIn("Further frames suppressed after 1", rendered)
        self.assertIn("including 3 kernel CAN error", rendered)

    def test_linux_error_frame_decoding(self):
        stuffing = FakeMessage(arbitration_id=0xC,
                               data=bytes.fromhex("00 30 04 00 00 00 08 87"),
                               is_error_frame=True)
        self.assertEqual(diagnostic.describe_can_error(stuffing),
                         ["RX passive", "TX passive", "bit stuffing error"])
        self.assertNotIn("no ACK", diagnostic.format_message(stuffing))
        no_ack = FakeMessage(arbitration_id=0x20, is_error_frame=True)
        self.assertEqual(diagnostic.describe_can_error(no_ack), ["no ACK"])


if __name__ == "__main__":
    unittest.main()
