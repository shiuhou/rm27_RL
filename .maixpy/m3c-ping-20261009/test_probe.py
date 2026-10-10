import unittest
from pymavlink.dialects.v20 import common as mav
from probe import Parser, ping_frame, fresh_disarmed


class ProbeTests(unittest.TestCase):
    def test_tx_is_ping_only_and_matches_pymavlink(self):
        for seq in range(3):
            sender = mav.MAVLink(None, srcSystem=246, srcComponent=191)
            sender.seq = seq
            expected = sender.ping_encode(123456789, seq, 0, 0).pack(sender, force_mavlink1=True)
            self.assertEqual(ping_frame(123456789, seq), expected)
            parsed = mav.MAVLink(None).parse_buffer(expected)[0]
            self.assertEqual(parsed.get_type(), 'PING')
            self.assertEqual(parsed.target_system, 0)

    def test_receive_v1_v2_split_and_noise(self):
        for v1 in (True, False):
            sender = mav.MAVLink(None, srcSystem=1, srcComponent=1)
            frame = sender.heartbeat_encode(2, 12, 29, 0, 3).pack(sender, force_mavlink1=v1)
            p, output = Parser(), []
            for byte in b'noise' + frame:
                output += p.feed(bytes([byte]))
            self.assertEqual(len(output), 1)
            self.assertEqual(output[0][0:3], (0, 1, 1))
            self.assertEqual(output[0][3][6], 29)

    def test_crc_corrupt_rejected(self):
        data = bytearray(ping_frame(100, 0))
        data[-1] ^= 1
        self.assertEqual(Parser().feed(data), [])

    def test_ping_reply_decode(self):
        sender = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        reply = sender.ping_encode(123456789, 1, 246, 191).pack(sender)
        self.assertEqual(Parser().feed(reply)[0][0:3], (4, 1, 1))

    def test_fresh_disarmed_gate(self):
        self.assertFalse(fresh_disarmed(None, 10))
        self.assertFalse(fresh_disarmed((6, 29), 10))
        self.assertFalse(fresh_disarmed((10, 157), 10))
        self.assertFalse(fresh_disarmed((11, 29), 10))
        self.assertTrue(fresh_disarmed((9, 29), 10))


if __name__ == '__main__':
    unittest.main()
