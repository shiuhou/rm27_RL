"""Synthetic framing fixtures, never hardware observations."""
from bisect import bisect_left
import hashlib
import struct
import unittest
from pymavlink.dialects.v20 import common as mav

try:
    from ulog_mavlink import Reassembler, ReassemblyError, inspect_imu
except ImportError:
    Reassembler = ReassemblyError = inspect_imu = None

FMT = ('vehicle_imu:uint64_t timestamp;uint64_t timestamp_sample;'
       'uint32_t accel_device_id;uint32_t gyro_device_id;float[3] delta_angle;'
       'float[3] delta_velocity;uint32_t delta_angle_dt;uint32_t delta_velocity_dt;'
       'uint8_t delta_angle_clipping;uint8_t delta_velocity_clipping;'
       'uint8_t accel_calibration_count;uint8_t gyro_calibration_count;')


def record(kind, body):
    return struct.pack('<HB', len(body), ord(kind)) + body


def imu(stamp, *, cal=1, clip=0, accel=6946834, gyro=6684690):
    return struct.pack('<QQII6fII4B', stamp+50, stamp, accel, gyro,
                       .005, 0, 0, 0, 0, .049, 5000, 5000, clip, 0, cal, cal)


def fixture(*, tail=None, fmt=True, binding=True):
    parts = [b'ULog\x01\x12\x35\x01' + struct.pack('<Q', 100000),
             record('B', bytes(40))]
    if fmt:
        parts.append(record('F', FMT.encode()))
    if binding:
        parts.extend([record('A', struct.pack('<BH', i, i)+b'vehicle_imu')
                      for i in (0, 1)])
    parts.extend([record('D', struct.pack('<H', 0)+imu(100000)),
                  record('D', struct.pack('<H', 1)+imu(100000, accel=3604506, gyro=3604506)),
                  record('D', struct.pack('<H', 0)+(tail or imu(105000)))])
    return b''.join(parts)


def packets(raw, chunk=249, start_sequence=0):
    """Synthetic wrapping of a saved/fixture ULog, independent framing oracle."""
    starts = [0]
    cursor = 16
    while cursor < len(raw):
        starts.append(cursor)
        if cursor+3 > len(raw):
            break
        cursor += 3+int.from_bytes(raw[cursor:cursor+2], 'little')
    result = []
    enc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    for number, begin in enumerate(range(0, len(raw), chunk)):
        data = raw[begin:begin+chunk]
        index = bisect_left(starts, begin)
        offset = starts[index]-begin if index < len(starts) and starts[index] < begin+len(data) else 255
        typ = mav.MAVLink_logging_data_acked_message if number == 0 else mav.MAVLink_logging_data_message
        msg = typ(1, 191, (start_sequence+number) & 65535, len(data), offset,
                  list(data)+[0]*(249-len(data)))
        result.append(msg.pack(enc))
        enc.seq = (enc.seq+1) & 255
    return result


class ReassemblyTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(Reassembler, 'offline reassembly implementation is missing')

    def assemble(self, raw, chunk=249):
        r = Reassembler()
        for wire in packets(raw, chunk):
            r.feed_wire(wire, 1)
        self.assertEqual(r.finish(), raw)
        return r

    def test_split_frames_and_split_records(self):
        raw = fixture()
        r = Reassembler()
        wires = packets(raw, 13)
        for wire in wires:
            for value in wire:
                r.feed_wire(bytes([value]), 1)
        self.assertEqual(r.finish(), raw)

    def test_many_records_and_hash(self):
        raw = fixture()+b''.join(record('D', struct.pack('<H', 0)+imu(110000+5000*i)) for i in range(500))
        r = self.assemble(raw)
        self.assertEqual(hashlib.sha256(r.finish()).digest(), hashlib.sha256(raw).digest())

    def test_logging_sequence_wrap(self):
        key = b'char[65100] payload'
        raw = fixture()+record('I', bytes([len(key)])+key+b'x'*65100)
        r = self.assemble(raw, 1)
        self.assertGreater(r.packet_count, 65536)

    def test_acked_retry_is_not_appended(self):
        raw = fixture()
        wire = packets(raw)[0]
        r = Reassembler()
        self.assertEqual(r.feed_wire(wire, 1), [0])
        self.assertEqual(r.feed_wire(wire, 2), [0])
        for part in packets(raw)[1:]:
            r.feed_wire(part, 3)
        self.assertEqual(r.finish(), raw)
        self.assertEqual(r.retransmissions, 1)

    def test_unacked_duplicate_rejected(self):
        parts = packets(fixture(), 100)
        r = Reassembler()
        r.feed_wire(parts[0], 1)
        r.feed_wire(parts[1], 2)
        with self.assertRaisesRegex(ReassemblyError, 'sequence'):
            r.feed_wire(parts[1], 3)

    def test_gap_is_terminal(self):
        parts = packets(fixture(), 100)
        r = Reassembler()
        r.feed_wire(parts[0], 1)
        with self.assertRaisesRegex(ReassemblyError, 'sequence'):
            r.feed_wire(parts[2], 2)
        with self.assertRaises(ReassemblyError):
            r.feed_wire(parts[1], 3)
        with self.assertRaises(ReassemblyError):
            r.finish()

    def test_reordered_or_restarted_sequence_rejected(self):
        parts = packets(fixture(), 100)
        r = Reassembler()
        for wire in parts[:2]:
            r.feed_wire(wire, 1)
        with self.assertRaisesRegex(ReassemblyError, 'sequence'):
            r.feed_wire(parts[0], 2)

    def test_conflicting_acked_retry_rejected(self):
        raw = fixture()
        r = Reassembler()
        r.feed_wire(packets(raw)[0], 1)
        changed = bytearray(raw)
        changed[8] ^= 1
        with self.assertRaisesRegex(ReassemblyError, 'sequence'):
            r.feed_wire(packets(changed)[0], 2)

    def test_nonzero_initial_sequence_rejected(self):
        with self.assertRaisesRegex(ReassemblyError, 'sequence'):
            Reassembler().feed_wire(packets(fixture(), start_sequence=12)[0], 1)

    def test_bad_crc_rejected(self):
        wire = bytearray(packets(fixture())[0])
        wire[20] ^= 1
        r = Reassembler()
        with self.assertRaisesRegex(ReassemblyError, 'MAVLink'):
            r.feed_wire(wire, 1)
        with self.assertRaises(ReassemblyError):
            r.finish()

    def test_other_frames_can_interleave(self):
        r = Reassembler()
        hb = mav.MAVLink_heartbeat_message(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4, 0, 0, 0, 3)
        enc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        for wire in packets(fixture()):
            r.feed_wire(hb.pack(enc), 1)
            r.feed_wire(wire, 2)
        self.assertEqual(r.finish(), fixture())

    def test_wrong_route_rejected(self):
        msg = mav.MAVLink_logging_data_acked_message(2, 191, 0, 16, 0, list(fixture()[:16])+[0]*233)
        with self.assertRaisesRegex(ReassemblyError, 'route'):
            Reassembler().feed_wire(msg.pack(mav.MAVLink(None, srcSystem=1, srcComponent=1)), 1)

    def test_wrong_source_rejected(self):
        msg = mav.MAVLink_logging_data_acked_message(1, 191, 0, 16, 0, list(fixture()[:16])+[0]*233)
        with self.assertRaisesRegex(ReassemblyError, 'source'):
            Reassembler().feed_wire(msg.pack(mav.MAVLink(None, srcSystem=2, srcComponent=1)), 1)

    def test_invalid_length_or_offset_rejected(self):
        for length, offset in ((250, 0), (0, 0), (16, 16)):
            msg = mav.MAVLink_logging_data_acked_message(1, 191, 0, length, offset, [1]*249)
            with self.subTest(length=length, offset=offset), self.assertRaises(ReassemblyError):
                Reassembler().feed_wire(msg.pack(mav.MAVLink(None, srcSystem=1, srcComponent=1)), 1)

    def test_offset_must_match_actual_record_boundary(self):
        raw = fixture()
        parts = packets(raw)
        parsed = mav.MAVLink(None).parse_buffer(parts[1])[0]
        parsed.first_message_offset = 1 if parsed.first_message_offset == 0 else 0
        parts[1] = parsed.pack(mav.MAVLink(None, srcSystem=1, srcComponent=1))
        r = Reassembler()
        for wire in parts:
            r.feed_wire(wire, 1)
        with self.assertRaisesRegex(ReassemblyError, 'offset'):
            r.finish()

    def test_truncated_outer_frame_rejected(self):
        r = Reassembler()
        r.feed_wire(packets(fixture())[0][:-1], 1)
        with self.assertRaisesRegex(ReassemblyError, 'MAVLink'):
            r.finish()

    def test_truncated_ulog_record_rejected(self):
        r = Reassembler()
        for wire in packets(fixture()[:-1]):
            r.feed_wire(wire, 1)
        with self.assertRaisesRegex(ReassemblyError, 'truncated'):
            r.finish()

    def test_missing_format_or_binding_rejected(self):
        for raw in (fixture(fmt=False), fixture(binding=False)):
            r = Reassembler()
            for wire in packets(raw):
                r.feed_wire(wire, 1)
            with self.assertRaisesRegex(ReassemblyError, 'format|binding'):
                r.finish()

    def test_unknown_data_binding_rejected(self):
        raw = fixture()+record('D', struct.pack('<H', 66)+imu(110000))
        r = Reassembler()
        for wire in packets(raw):
            r.feed_wire(wire, 1)
        with self.assertRaisesRegex(ReassemblyError, 'binding'):
            r.finish()

    def test_memory_limit_rejected(self):
        r = Reassembler(max_bytes=100)
        with self.assertRaisesRegex(ReassemblyError, 'limit'):
            r.feed_wire(packets(fixture())[0], 1)

    def test_bad_magic_version_rejected(self):
        for index in (0, 7):
            raw = bytearray(fixture())
            raw[index] ^= 9
            r = Reassembler()
            for wire in packets(raw):
                r.feed_wire(wire, 1)
            with self.assertRaisesRegex(ReassemblyError, 'ULog header'):
                r.finish()

    def test_conflicting_format_rejected(self):
        raw = fixture()+record('F', FMT.replace('float[3]', 'double[3]').encode())
        r = Reassembler()
        for wire in packets(raw):
            r.feed_wire(wire, 1)
        with self.assertRaisesRegex(ReassemblyError, 'conflicting format'):
            r.finish()

    def test_bad_data_size_rejected(self):
        raw = fixture()+record('D', struct.pack('<H', 0)+imu(110000)+b'x')
        r = Reassembler()
        for wire in packets(raw):
            r.feed_wire(wire, 1)
        with self.assertRaisesRegex(ReassemblyError, 'corruption|decoding'):
            r.finish()

    def test_complete_prefix_cannot_prove_source_completeness(self):
        prefix = fixture()[:-65]
        r = self.assemble(prefix)
        self.assertTrue(r.summary()['finite_framing_verified'])
        self.assertEqual(r.summary()['upstream_source_complete'], 'UNKNOWN')
        self.assertFalse(r.summary()['openvins_admissible'])

    def test_dropout_kept_even_with_continuous_packets(self):
        raw = self.assemble(fixture()+record('O', struct.pack('<H', 0))).finish()
        result = inspect_imu(raw, instance=0, gyro_device=6684690, accel_device=6946834, epoch='SYNTHETIC_1')
        self.assertEqual(result['dropout_events'], 1)
        self.assertIn('logger_dropouts', result['issues'])
        self.assertFalse(result['openvins_admissible'])

    def test_imu_units_identity_intervals_and_instances(self):
        raw = self.assemble(fixture()).finish()
        result = inspect_imu(raw, instance=0, gyro_device=6684690, accel_device=6946834, epoch='SYNTHETIC_1')
        self.assertEqual(result['count'], 2)
        self.assertEqual(result['issues'], [])
        self.assertAlmostEqual(result['diagnostic_means'][0]['gyro_mean_rad_s'][0], 1.0, places=6)
        self.assertAlmostEqual(result['diagnostic_means'][0]['accel_mean_m_s2'][2], 9.8, places=5)
        self.assertIsNone(result['diagnostic_means'][0]['accel_end_us'])
        self.assertFalse(result['openvins_admissible'])
        with self.assertRaisesRegex(ValueError, 'identity'):
            inspect_imu(raw, instance=1, gyro_device=6684690, accel_device=6946834, epoch='SYNTHETIC_1')

    def test_imu_calibration_clipping_gap_detected(self):
        result = inspect_imu(fixture(tail=imu(115000, cal=2, clip=1)), instance=0,
                             gyro_device=6684690, accel_device=6946834, epoch='SYNTHETIC_1')
        self.assertEqual(result['issues'], ['calibration', 'clipping', 'gap'])
        self.assertFalse(result['openvins_admissible'])

    def test_zero_dt_nonfinite_and_bad_order_refused(self):
        for body in (imu(105000)[:48]+struct.pack('<II4B', 0, 5000, 0, 0, 1, 1),
                     imu(100000),
                     imu(105000)[:24]+struct.pack('<f', float('nan'))+imu(105000)[28:]):
            raw = fixture(tail=body)
            with self.subTest(), self.assertRaisesRegex(ValueError, 'positive|ordering|finite'):
                inspect_imu(raw, instance=0, gyro_device=6684690, accel_device=6946834, epoch='SYNTHETIC_1')


if __name__ == '__main__':
    unittest.main()
