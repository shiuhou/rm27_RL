"""Offline PX4 FIFO evidence export. No serial access, repair or VIO admission.

Keep separate sensor streams; an exact-time join is diagnostic, never gap filling.
pyulog is needed only by the ULog CLI (existing research environment).
"""
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import statistics


def integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(name + ' must be a nonnegative integer')
    return value


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be positive and finite')
    return value


def decimal_text(value):
    return format(value, 'f').rstrip('0').rstrip('.') if value % 1 else str(int(value))


def expand_fifo_rows(rows, *, expected_device_id):
    integer(expected_device_id, 'expected device')
    result, last = [], None
    for record_index, row in enumerate(rows):
        device = integer(row['device_id'], 'device_id')
        if device != expected_device_id:
            raise ValueError('device identity mismatch; split datasets at changes')
        n = integer(row['samples'], 'samples')
        if not 1 <= n <= 32:
            raise ValueError('FIFO samples must be 1..32')
        anchor = integer(row['timestamp_sample'], 'timestamp_sample')
        publication = integer(row['timestamp'], 'timestamp')
        if publication < anchor:
            raise ValueError('publication precedes sample anchor')
        dt = Decimal(str(positive(row['dt'], 'dt')))
        scale = positive(row['scale'], 'scale')
        if any(len(row[axis]) < n for axis in 'xyz'):
            raise ValueError('truncated FIFO array')
        for i in range(n):
            stamp = Decimal(anchor) - (n-1-i)*dt
            if stamp < 0 or (last is not None and stamp <= last):
                raise ValueError('FIFO timestamps overlap, reset or are unordered')
            counts = [row[axis][i] for axis in 'xyz']
            if any(type(x) is not int or not -32768 <= x <= 32767 for x in counts):
                raise ValueError('FIFO counts must be signed int16')
            scaled = [x*scale for x in counts]
            if not all(math.isfinite(x) for x in scaled):
                raise ValueError('nonfinite SI conversion')
            result.append(dict(batch_record_index=record_index, sample_index_in_batch=i,
                batch_samples=n, device_id=device, timestamp_sample_us=anchor,
                timestamp_publish_us=publication, dt_us=str(dt), scale=scale,
                counts_xyz=counts, value_si=scaled, sample_time_us=decimal_text(stamp),
                time_reconstructed=n>1 and i != n-1,
                timing_evidence='SOURCE_SUPPORTED_NOT_ADC_VERIFIED',
                integer_rail_observed=any(x in (-32768,32767) for x in counts)))
            last = stamp
    if not result:
        raise ValueError('empty FIFO stream')
    return result


def exact_pairing(gyro, accel):
    def index(stream):
        keys=[Decimal(x['sample_time_us']) for x in stream]
        if any(b<=a for a,b in zip(keys,keys[1:])):
            raise ValueError('pairing requires strictly ordered unique times')
        return {key:i for i,key in enumerate(keys)}
    gi, ai = index(gyro), index(accel)
    common=sorted(gi.keys() & ai.keys())
    return dict(common_samples=len(common),gyro_only=len(gi)-len(common),
        accel_only=len(ai)-len(common),pairs=[[gi[t],ai[t]] for t in common],
        missing_physical_samples=None, method='exact timestamp intersection; no interpolation')


def cadence(stream):
    times=[Decimal(x['sample_time_us']) for x in stream]
    delta=[b-a for a,b in zip(times,times[1:])]
    median=statistics.median(delta) if delta else None
    return dict(samples=len(times), median_interval_us=str(median) if median else None,
        max_gap_us=str(max(delta)) if delta else None,
        gaps_over_1_5_median=sum(d>median*Decimal('1.5') for d in delta) if delta else 0,
        observed_hz=float(Decimal(len(delta))*1000000/(times[-1]-times[0])) if delta else None)


def transport_budget(rate_hz, payload_bytes, overhead_bytes, baud):
    positive(rate_hz,'rate'); positive(payload_bytes,'payload'); positive(baud,'baud')
    integer(overhead_bytes,'overhead')
    wire=(payload_bytes+overhead_bytes)*rate_hz*10
    return dict(wire_bits_per_s=wire, utilization=wire/baud,
        fits_theoretical_link=wire<=baud,
        assumptions='one packet per sample; UART 8N1; excludes other traffic and retransmits')


def ulog_rows(dataset):
    d=dataset.data
    for i in range(len(d['timestamp'])):
        row={k:int(d[k][i]) for k in ('timestamp','timestamp_sample','device_id','samples')}
        row.update(dt=float(d['dt'][i]),scale=float(d['scale'][i]))
        row.update({axis:[int(d[f'{axis}[{j}]'][i]) for j in range(32)] for axis in 'xyz'})
        yield row


def export_ulog(source, output, *, instance, gyro_device, accel_device):
    from pyulog import ULog
    source,output=Path(source),Path(output)
    if output.exists():
        raise FileExistsError(output)
    before=hashlib.sha256(source.read_bytes()).hexdigest()
    u=ULog(str(source))
    if u.file_corruption:
        raise ValueError('corrupt ULog is not exported as valid evidence')
    streams={name:expand_fifo_rows(ulog_rows(u.get_dataset('sensor_'+name+'_fifo',instance)),
        expected_device_id=device) for name,device in [('gyro',gyro_device),('accel',accel_device)]}
    pairing=exact_pairing(streams['gyro'],streams['accel'])
    pairing.pop('pairs')  # Full streams retain source associations; no lossy paired replacement.
    report=dict(schema_version=1, source_sha256=before, source_file=str(source.resolve()),
        firmware_reported=u.msg_info_dict.get('ver_sw'), instance=instance,
        gyro_device_id=gyro_device, accel_device_id=accel_device,
        clock_domain='PX4_HRT_BOOT',epoch_id='ulog-sha256:'+before,
        timestamp_unit='us',value_units={'gyro':'rad/s','accel':'m/s^2'},
        clock_mapping_to_camera=None, saturation_qualification='UNKNOWN',
        axes='driver-rotated sensor frame; board-to-body extrinsics not applied',
        file_corruption=False,dropout_events=len(u.dropouts),
        dropout_zero_ms=sum(x.duration==0 for x in u.dropouts),
        dropout_duration_ms_sum=sum(x.duration for x in u.dropouts),
        dropout_duration_ms_max=max((x.duration for x in u.dropouts),default=0),
        cadence={k:cadence(v) for k,v in streams.items()},exact_pairing=pairing,
        complete_loss_free='FAILED' if u.dropouts else 'UNKNOWN',
        synchronized_vio_eligible=False,
        limits=['No camera data or cross-clock mapping.',
                'No-dropout alone cannot establish no subscription/sensor losses.',
                'Multi-sample timing follows source anchor semantics; not hardware validated.',
                'Sample time strings preserve fractional dt, not nanosecond accuracy.',
                'Sequence positions are export indices, not hardware counters.'])
    if hashlib.sha256(source.read_bytes()).hexdigest()!=before:
        raise ValueError('input changed during analysis')
    output.mkdir(parents=True,exist_ok=False)
    hashes={}
    for name,rows in streams.items():
        target=output/(name+'.jsonl')
        with target.open('x',encoding='utf-8') as f:
            for row in rows:
                f.write(json.dumps(row,allow_nan=False)+'\n')
        hashes[target.name]=hashlib.sha256(target.read_bytes()).hexdigest()
    report['export_sha256']=hashes
    with (output/'report.json').open('x',encoding='utf-8') as f:
        json.dump(report,f,indent=2,allow_nan=False)
    return report


def main():
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ulog',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--instance',required=True,type=int)
    p.add_argument('--gyro-device',required=True,type=int)
    p.add_argument('--accel-device',required=True,type=int)
    a=p.parse_args()
    print(json.dumps(export_ulog(a.ulog,a.output,instance=a.instance,
        gyro_device=a.gyro_device,accel_device=a.accel_device),indent=2,allow_nan=False))


if __name__=='__main__':
    main()
