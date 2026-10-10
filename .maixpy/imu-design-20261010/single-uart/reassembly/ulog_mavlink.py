"""Offline strict ULog-over-MAVLink reassembly. No device or transmitter API.

Reuse the saved CRC decoder and pyulog. ACK values are intents only. A failed
session is terminal; no partial output or recovery masquerades as a complete log.
"""
from bisect import bisect_left
import importlib.util
from io import BytesIO
from pathlib import Path
import struct

from pyulog import ULog

ROOT = Path(__file__).resolve().parents[3]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


decoder_module = _load('saved_imu_decoder', ROOT/'imu-input-20261010/imu_input.py')
diagnostics = _load('saved_imu_diagnostics', ROOT/'imu-design-20261010/offline.py')


class ReassemblyError(ValueError):
    pass


def _need(condition, message):
    if not condition:
        raise ReassemblyError(message)


def validate_ulog(raw):
    """Validate framing/binding order, then let pyulog decode actual formats.

    This finite-session prototype supports v1 without appended sections. It
    preserves O dropout records, which make loss-free qualification fail later.
    """
    _need(len(raw) >= 16 and raw[:8] == b'ULog\x01\x12\x35\x01', 'unsupported ULog header')
    starts = [0]
    formats, bindings = {}, {}
    cursor = 16
    while cursor < len(raw):
        _need(cursor+3 <= len(raw), 'truncated ULog record header')
        size, kind = struct.unpack_from('<HB', raw, cursor)
        end = cursor+3+size
        _need(end <= len(raw), 'truncated ULog record body')
        _need(kind in b'FADRIOBMPQLCS', 'unsupported ULog record type')
        starts.append(cursor)
        body = raw[cursor+3:end]
        if kind == ord('B'):
            _need(size == 40, 'unsupported flag record length')
            _need(body[8] & ~1 == 0 and not any(body[9:16]), 'unsupported incompatible flags')
            _need(not any(body[16:40]), 'appended ULog data unsupported')
        elif kind == ord('F'):
            _need(b':' in body, 'missing format name')
            name = body.split(b':', 1)[0]
            _need(name and (name not in formats or formats[name] == body), 'conflicting format definition')
            formats[name] = body
        elif kind == ord('A'):
            _need(size > 3, 'invalid binding record')
            msg_id = int.from_bytes(body[1:3], 'little')
            _need(body[3:] in formats, 'binding without format')
            _need(msg_id not in bindings, 'duplicate active binding')
            bindings[msg_id] = body[3:]
        elif kind == ord('R'):
            _need(size == 2 and int.from_bytes(body, 'little') in bindings, 'unknown removed binding')
            del bindings[int.from_bytes(body, 'little')]
        elif kind == ord('D'):
            _need(size > 2 and int.from_bytes(body[:2], 'little') in bindings, 'data without binding')
        elif kind == ord('O'):
            _need(size == 2, 'invalid dropout record')
        cursor = end
    try:
        log = ULog(BytesIO(raw), disable_str_exceptions=False)
    except (ValueError, KeyError, IndexError, TypeError, struct.error, RecursionError) as exc:
        raise ReassemblyError('ULog decoding failed: '+str(exc)) from exc
    _need(not log.file_corruption, 'pyulog detected corruption')
    return starts, log


class Reassembler:
    def __init__(self, *, source=(1, 1), target=(1, 191), max_bytes=64*1024*1024):
        _need(type(max_bytes) is int and 0 < max_bytes <= 64*1024*1024, 'invalid memory limit')
        self.source, self.target, self.max_bytes = source, target, max_bytes
        self.decoder = decoder_module.Decoder()
        self.buffer = bytearray()
        self.packet_offsets = []
        self.packet_count = self.retransmissions = 0
        self.expected_sequence = 0
        self.last_fingerprint = None
        self.failure = None
        self.finished = False

    def _reject(self, message):
        self.failure = message
        raise ReassemblyError(message)

    def _active(self):
        if self.failure:
            raise ReassemblyError(self.failure)
        if self.finished:
            raise ReassemblyError('session already finished')

    def feed_wire(self, wire, receipt_ns):
        self._active()
        acks = []
        for row in self.decoder.feed(wire, receipt_ns):
            acks.extend(self.feed(row))
        return acks

    def feed(self, row):
        """Consume existing Decoder output; return sequences needing ACK, send none."""
        self._active()
        if row.get('event') == 'bad_data':
            self._reject('invalid MAVLink data: '+row.get('reason', 'unknown'))
        if row.get('type') not in ('LOGGING_DATA', 'LOGGING_DATA_ACKED'):
            return []
        if row.get('crc_status') != 'VERIFIED' or row.get('mavlink_version') != 2:
            self._reject('unverified MAVLink logging frame')
        if (row.get('sysid'), row.get('compid')) != self.source:
            self._reject('unexpected logging source')
        f = row['fields']
        if (f.get('target_system'), f.get('target_component')) != self.target:
            self._reject('unexpected logging route')
        length, offset, sequence = (f.get(k) for k in ('length', 'first_message_offset', 'sequence'))
        if type(length) is not int or not 1 <= length <= 249:
            self._reject('invalid logging length')
        if type(offset) is not int or not (offset == 255 or 0 <= offset < length):
            self._reject('invalid first_message_offset')
        if type(sequence) is not int or not 0 <= sequence <= 65535:
            self._reject('invalid logging sequence')
        data = f.get('data')
        if not isinstance(data, list) or len(data) != 249 or any(type(v) is not int or not 0 <= v <= 255 for v in data):
            self._reject('invalid logging payload')
        payload = bytes(data[:length])
        acked = row['type'] == 'LOGGING_DATA_ACKED'
        fingerprint = (sequence, length, offset, acked, payload)
        if sequence != self.expected_sequence:
            if acked and fingerprint == self.last_fingerprint:
                self.retransmissions += 1
                return [sequence]
            self._reject('logging sequence discontinuity: expected %d, received %d' % (self.expected_sequence, sequence))
        if self.packet_count == 0 and (not acked or offset != 0):
            self._reject('session lacks reliable ULog header start')
        if len(self.buffer)+length > self.max_bytes or self.packet_count >= 300000:
            self._reject('reassembly limit exceeded')
        begin = len(self.buffer)
        self.buffer.extend(payload)
        self.packet_offsets.append((begin, begin+length, offset))
        self.packet_count += 1
        self.last_fingerprint = fingerprint
        self.expected_sequence = (sequence+1) & 65535
        return [sequence] if acked else []

    def finish(self):
        if self.failure:
            raise ReassemblyError(self.failure)
        if self.finished:
            return bytes(self.buffer)
        if self.decoder.parser.buf_len():
            self._reject('truncated MAVLink frame at end')
        try:
            raw = bytes(self.buffer)
            starts, _ = validate_ulog(raw)
            for begin, end, offset in self.packet_offsets:
                index = bisect_left(starts, begin)
                actual = starts[index]-begin if index < len(starts) and starts[index] < end else 255
                _need(offset == actual, 'first_message_offset differs from ULog boundary')
        except ReassemblyError as exc:
            self._reject(str(exc))
        self.finished = True
        return raw

    def summary(self):
        return dict(packet_count=self.packet_count, retransmissions=self.retransmissions,
                    bytes_reassembled=len(self.buffer), failure=self.failure,
                    finite_framing_verified=self.finished and self.failure is None,
                    upstream_source_complete='UNKNOWN', openvins_admissible=False)


def inspect_imu(raw, *, instance, gyro_device, accel_device, epoch):
    """Reuse source-semantic audit; never create OpenVINS samples or clock mapping."""
    _, log = validate_ulog(raw)
    formats = log.message_formats.get('vehicle_imu')
    _need(formats is not None, 'vehicle_imu format absent')
    required = [('uint64_t', 0, 'timestamp'), ('uint64_t', 0, 'timestamp_sample'),
                ('uint32_t', 0, 'accel_device_id'), ('uint32_t', 0, 'gyro_device_id'),
                ('float', 3, 'delta_angle'), ('float', 3, 'delta_velocity'),
                ('uint32_t', 0, 'delta_angle_dt'), ('uint32_t', 0, 'delta_velocity_dt'),
                ('uint8_t', 0, 'delta_angle_clipping'), ('uint8_t', 0, 'delta_velocity_clipping'),
                ('uint8_t', 0, 'accel_calibration_count'), ('uint8_t', 0, 'gyro_calibration_count')]
    fields = [field for field in formats.fields if not field[2].startswith('_padding')]
    _need(fields == required, 'vehicle_imu schema differs from audited PX4')
    candidates = [d for d in log.data_list if d.name == 'vehicle_imu' and d.multi_id == instance]
    _need(len(candidates) == 1, 'vehicle_imu instance absent or ambiguous')
    dataset = candidates[0].data
    rows = []
    for index in range(len(dataset['timestamp'])):
        row = {name: int(dataset[name][index]) for typ, count, name in required if count == 0}
        for name in ('delta_angle', 'delta_velocity'):
            row[name] = [float(dataset[name+'[%d]' % axis][index]) for axis in range(3)]
        _need((row['gyro_device_id'], row['accel_device_id']) == (gyro_device, accel_device), 'IMU identity mismatch')
        rows.append(row)
    result = diagnostics.audit(rows, units=diagnostics.UNITS, epoch=epoch)
    if 'ordering' in result['issues'] or 'publication_ordering' in result['issues']:
        raise ReassemblyError('IMU ordering invalid')
    result.update(dropout_events=len(log.dropouts),
                  dropout_duration_ms=sum(d.duration for d in log.dropouts),
                  source_instance=instance, gyro_device_id=gyro_device, accel_device_id=accel_device,
                  clock_status='UNKNOWN', openvins_admissible=False)
    if log.dropouts:
        result['issues'] = sorted(set(result['issues']) | {'logger_dropouts'})
    return result
