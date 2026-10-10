"""Replay EXISTING real ULog bytes in SYNTHETIC MAVLink framing; no UART IO.

Does not create a new capture, repaired log, synchronization or performance result.
Uses fixture packet wrapping and library parsing to prove byte preservation.
"""
import hashlib
from io import BytesIO
import json
from pathlib import Path

from pyulog import ULog
from test_ulog_mavlink import packets
from ulog_mavlink import Reassembler, ReassemblyError

ROOT = Path(__file__).resolve().parents[3]
SOURCES = (
    ('HISTORICAL_MICOAIR_V1152_BMI270', ROOT/'imu-offline-20261008/bmi270-disarmed-01.ulg',
     'd94a142c5a474e7a755bffd534a41cf3255081d87d4868dabf63493a5146f46d', 391),
    ('SAVED_PX4_D6F12AD1_BMI270_NOT_SELECTED_BMI088', ROOT/'px4-buffer-ab-20261009/A2.ulg',
     '3815e9e8437d68c5ae1dd465c05d947937af31e30d097f6b24e1f30c3ebcbc20', 306),
)


def replay(label, path, expected_hash, expected_dropouts):
    raw = path.read_bytes()
    before = hashlib.sha256(raw).hexdigest()
    if before != expected_hash:
        raise ValueError('saved input SHA mismatch: '+str(path))
    r = Reassembler()
    wires = packets(raw)
    for number, wire in enumerate(wires):
        # Artificial ordering token only: not a receive timestamp or rate measurement.
        r.feed_wire(wire, number)
    rebuilt = r.finish()
    if rebuilt != raw:
        raise AssertionError('reassembly changed saved bytes')
    log = ULog(BytesIO(rebuilt), disable_str_exceptions=False)
    if log.file_corruption or len(log.dropouts) != expected_dropouts:
        raise AssertionError('saved dropout/corruption evidence changed')
    counts = {d.name+':'+str(d.multi_id): len(d.data['timestamp'])
              for d in log.data_list if d.name in ('sensor_gyro_fifo', 'sensor_accel_fifo')}
    faults = {}
    for case in ('missing_packet', 'out_of_order', 'truncated_outer_frame'):
        bad = Reassembler()
        bad.feed_wire(wires[0], 0)
        try:
            if case == 'missing_packet':
                bad.feed_wire(wires[2], 1)
            elif case == 'out_of_order':
                bad.feed_wire(wires[1], 1)
                bad.feed_wire(wires[0], 2)
            else:
                bad.feed_wire(wires[1][:-1], 1)
                bad.finish()
        except ReassemblyError as exc:
            faults[case] = str(exc)
        else:
            raise AssertionError('fault accepted: '+case)
        try:
            bad.finish()
        except ReassemblyError:
            pass
        else:
            raise AssertionError('terminal rejection not preserved')
    if hashlib.sha256(path.read_bytes()).hexdigest() != before:
        raise AssertionError('original saved log changed')
    return dict(label=label, input_path=str(path), framing='SYNTHETIC_NOT_UART_CAPTURE',
                unchanged_source_sha256=before, exact_reassembly=True,
                **r.summary(), file_corruption=log.file_corruption,
                dropout_events=len(log.dropouts), dropout_duration_ms=sum(d.duration for d in log.dropouts),
                fifo_counts=counts, injected_faults_rejected=faults,
                actual_transport_rate='UNKNOWN', clock_sync='UNKNOWN',
                loss_free_imu='FAILED_SAVED_LOGGER_DROPOUTS')


if __name__ == '__main__':
    print(json.dumps([replay(*source) for source in SOURCES], indent=2))
