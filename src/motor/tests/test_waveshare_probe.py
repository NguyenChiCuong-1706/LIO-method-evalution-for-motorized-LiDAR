import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import waveshare_probe


class WaveshareProbeTests(unittest.TestCase):
    def test_real_status_payload_is_reported_unknown(self):
        raw = bytes.fromhex("9A 20 32 0B 00 00 30 00")
        lines = waveshare_probe.describe_status1(raw)
        self.assertIn("28.66 V", lines[0])
        self.assertIn("LE=2866 BE=12811", lines[2])
        self.assertTrue(any("Fault_state=UNKNOWN" in line
                            for line in lines))
        frame = waveshare_probe.describe_frame("standard", 0x141, raw, 1.25)
        self.assertIn("DLC=8 DATA=9A 20 32 0B 00 00 30 00", frame[0])
        self.assertEqual(frame[7], "  Byte 6: 0x30")

    def test_real_stationary_feedback_is_provisional(self):
        lines = waveshare_probe.describe_status2(bytes.fromhex("9C 20 02 00 00 00 34 32"))
        self.assertIn("speed_raw=0", lines[0])
        self.assertIn("encoder_raw=12852", lines[0])
        self.assertIn("not yet verified", lines[1])
        moving_sample = waveshare_probe.describe_status2(
            bytes.fromhex("9C 23 02 00 01 00 34 32"))
        self.assertIn("nonzero raw sample", moving_sample[1])

    def test_status_frame_encodes_only_read_command(self):
        self.assertEqual(waveshare_probe.status_serial_frame(1).hex(" "),
                         "aa c8 41 01 9a 00 00 00 00 00 00 00 55")
        self.assertEqual(waveshare_probe.status_serial_frame(1, 0x9C).hex(" "),
                         "aa c8 41 01 9c 00 00 00 00 00 00 00 55")
        with self.assertRaises(ValueError):
            waveshare_probe.status_serial_frame(33)

    def test_settings_frame_matches_waveshare_layout(self):
        frame = waveshare_probe.settings_frame(500_000)
        self.assertEqual(len(frame), 20)
        self.assertEqual(frame[:5], bytes.fromhex("aa 55 12 03 01"))
        self.assertEqual(frame[13:15], bytes.fromhex("00 01"))
        self.assertEqual(frame[-1], sum(frame[2:-1]) & 0xFF)

    def test_parser_handles_chunking_and_unrelated_packets(self):
        response = bytes.fromhex("aa c8 41 01 9a 19 00 f0 00 00 00 00 55")
        buffer = bytearray(b"\x00" + response[:6])
        self.assertEqual(list(waveshare_probe.extract_frames(buffer)), [])
        buffer.extend(response[6:])
        self.assertEqual(list(waveshare_probe.extract_frames(buffer)),
                         [("standard", 0x141, response[4:-1])])
        self.assertEqual(buffer, b"")


if __name__ == "__main__":
    unittest.main()
