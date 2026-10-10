"""Passive MAVLink evidence decoder/analyzer. No serial or transmit API.

Semantics are tied to PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
No resampling, clock fitting, bias undoing or raw-sample qualification.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics
from pymavlink.dialects.v20 import common as m

IMU_TYPES=('HIGHRES_IMU','SCALED_IMU','SCALED_IMU2','SCALED_IMU3','RAW_IMU')

def clean(value):
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,(bytes,bytearray)): return value.hex()
    if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [clean(v) for v in value]
    return value

def wire_header(row):
    """pymavlink UNKNOWN messages do not populate their header object."""
    r=dict(row)
    if r['event']!='frame': return r
    wire=bytes.fromhex(r['wire_hex']); v2=wire[0]==253
    r.update(sysid=wire[5] if v2 else wire[3],compid=wire[6] if v2 else wire[4],
             seq=wire[4] if v2 else wire[2],
             msgid=int.from_bytes(wire[7:10],'little') if v2 else wire[5],
             crc_status='UNKNOWN_DIALECT_UNVERIFIED' if r['type'].startswith('UNKNOWN_') else 'VERIFIED')
    return r

class Decoder:
    def __init__(self):
        self.parser=m.MAVLink(None)
        self.parser.robust_parsing=True

    def feed(self,data,receipt_ns):
        rows=[]
        for msg in self.parser.parse_buffer(data) or []:
            if msg.get_type()=='BAD_DATA':
                rows.append(dict(event='bad_data',receipt_mono_ns=receipt_ns,
                                 reason=msg.reason,wire_hex=bytes(msg.data).hex()))
            else:
                wire=bytes(msg.get_msgbuf())
                rows.append(wire_header(dict(event='frame',receipt_mono_ns=receipt_ns,
                    sysid=msg.get_srcSystem(),compid=msg.get_srcComponent(),seq=msg.get_seq(),
                    msgid=msg.get_msgId(),type=msg.get_type(),wire_hex=wire.hex(),
                    mavlink_version=2 if wire[0]==253 else 1,fields=clean(msg.to_dict()))))
        return rows

def intervals(values,ticks_per_second):
    delta=[b-a for a,b in zip(values,values[1:])]
    positive=[d for d in delta if d>0]
    median=statistics.median(positive) if positive else None
    strict=bool(delta) and all(d>0 for d in delta)
    return dict(count=len(values),first=values[0] if values else None,last=values[-1] if values else None,
        duplicates=sum(d==0 for d in delta),backwards=sum(d<0 for d in delta),
        median_positive_interval=median,max_interval=max(positive) if positive else None,
        gap_events_gt_1_5_median=sum(d>1.5*median for d in positive) if median else 0,
        effective_hz=(len(values)-1)*ticks_per_second/(values[-1]-values[0]) if strict else None)

def imu_record(row,epoch):
    typ=row['type']; f=row['fields']
    if typ not in IMU_TYPES: raise ValueError('Not IMU telemetry')
    if (row['sysid'],row['compid'])!=(1,1): raise ValueError('Source not the audited PX4 endpoint')
    high=typ=='HIGHRES_IMU'; scaled=typ.startswith('SCALED_IMU')
    stamp=f.get('time_usec') if not scaled else f.get('time_boot_ms')
    accel=[f.get(axis+'acc') for axis in 'xyz']; gyro=[f.get(axis+'gyro') for axis in 'xyz']
    scale=lambda values,factor:[x*factor if x is not None else None for x in values]
    # RAW_IMU requires its own exact sender audit; never infer semantics from name.
    return dict(type=typ,sysid=row['sysid'],compid=row['compid'],packet_seq=row['seq'],
        source_timestamp=stamp,source_unit='ms' if scaled else 'us',source_domain='PX4_HRT',
        source_epoch=epoch,receipt_mono_ns=row['receipt_mono_ns'],receipt_domain='M3C_MONOTONIC',
        timestamp_semantic='GYRO_INTEGRATION_END' if high else 'PUBLICATION' if scaled else 'UNKNOWN',
        measurement_semantic='CALIBRATED_INTEGRAL_DIV_DT_MINUS_MATCHED_EKF_BIAS' if high else
            'CALIBRATED_INTEGRAL_DIV_DT_QUANTIZED' if scaled else 'UNKNOWN',
        accel_si=accel if high else scale(accel,9.80665/1000) if scaled else None,
        gyro_si=gyro if high else scale(gyro,.001) if scaled else None,
        accel_unit='m/s^2' if high or scaled else None,gyro_unit='rad/s' if high or scaled else None,
        body_frame='FRD' if high or scaled else 'UNKNOWN',fields_updated=f.get('fields_updated'),
        wire_imu_id=f.get('id'),hardware_device_id=None,integration_dt_us=None,
        mapped_ns=None,clock_map=None,synchronization_status='UNKNOWN',vio_admissible=False)

def wire_budget(payload_bytes,hz,baud,signed=False):
    wire=(payload_bytes+12+(13 if signed else 0))*hz
    return dict(wire_bytes_per_second=wire,capacity_bytes_per_second=baud/10,
                utilization=wire/(baud/10),assumption='8N1 MAVLink2 untruncated payload; no other traffic')

def summarize(rows):
    frames=[wire_header(r) for r in rows if r.get('event')=='frame']
    grouped=defaultdict(list); sources=defaultdict(list)
    for r in frames:
        grouped[f"{r['sysid']}:{r['compid']}:{r['type']}"] .append(r)
        sources[f"{r['sysid']}:{r['compid']}"] .append(r)
    source_stats={}
    for key,rs in sources.items():
        seq=dict(forward_missing_estimate=0,duplicates=0,backward_or_reset=0,discontinuity_events=0)
        for a,b in zip(rs,rs[1:]):
            d=(b['seq']-a['seq'])%256
            if d!=1: seq['discontinuity_events']+=1
            if d==0: seq['duplicates']+=1
            elif 1<d<128: seq['forward_missing_estimate']+=d-1
            elif d>=128: seq['backward_or_reset']+=1
        source_stats[key]=dict(frames=len(rs),sequence=seq,
            crc_unverified_frames=sum(r['crc_status']!='VERIFIED' for r in rs),
            caveat='8-bit source packet sequence only; resets/reorder/multiplexing/whole wraps ambiguous; NOT sample loss')
    groups={}
    for key,rs in grouped.items():
        entry=dict(count=len(rs),receipt=intervals([r['receipt_mono_ns'] for r in rs],1e9),
                   wire_bytes=sum(len(r['wire_hex'])//2 for r in rs),
                   versions=dict(Counter(r['mavlink_version'] for r in rs)))
        if rs[0]['type'] in IMU_TYPES and (rs[0]['sysid'],rs[0]['compid'])==(1,1):
            imus=[imu_record(r,'capture-local-epoch') for r in rs]
            entry['source_timestamp']=intervals([r['source_timestamp'] for r in imus],1000 if imus[0]['source_unit']=='ms' else 1e6)
            entry['semantics']=imus[0]
            for field in ('accel_si','gyro_si'):
                vectors=[r[field] for r in imus]
                good=[v for v in vectors if v is not None and all(x is not None and math.isfinite(x) for x in v)]
                entry[field]=dict(nonfinite_or_unknown=len(vectors)-len(good),
                    mean=[statistics.mean(v[i] for v in good) for i in range(3)] if good else None,
                    mean_norm=statistics.mean(math.sqrt(sum(x*x for x in v)) for v in good) if good else None)
            entry['fields_updated_counts']=dict(Counter(str(x['fields_updated']) for x in imus))
        groups[key]=entry
    bad=[r for r in rows if r.get('event')=='bad_data']
    return dict(frames=len(frames),bad_data_events=len(bad),bad_data_bytes=sum(len(r['wire_hex'])//2 for r in bad),
        sources=source_stats,messages=groups,sensor_loss_count=None,clock_synchronization='UNKNOWN',
        vio_suitability='NOT_QUALIFIED',session_events=[r for r in rows if r.get('event') not in ('frame','chunk','bad_data')])

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture'); p.add_argument('output'); args=p.parse_args()
    rows=[json.loads(line) for line in Path(args.capture).read_text(encoding='utf-8').splitlines()]
    # Re-decode raw chunks and require exact parity with saved decoder records.
    d=Decoder(); replay=[]
    for r in rows:
        if r['event']=='chunk': replay.extend(d.feed(bytes.fromhex(r['hex']),r['receipt_mono_ns']))
    stored=[r for r in rows if r['event'] in ('frame','bad_data')]
    # Older capture decoder left UNKNOWN header fields at defaults. Correct ONLY
    # header metadata from immutable wire bytes, and explicitly report that correction.
    normalized=[wire_header(r) for r in stored]
    corrections=sum(any(r.get(k)!=n.get(k) for k in ('sysid','compid','seq','msgid')) for r,n in zip(stored,normalized))
    if replay!=normalized: raise ValueError('Raw replay / decoded records disagree')
    out=Path(args.output); out.mkdir(exist_ok=False)
    report=summarize(rows); report['raw_replay_exact_after_header_normalization']=True
    report['legacy_unknown_header_corrections']=corrections
    (out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    with (out/'imu-telemetry.jsonl').open('x',encoding='utf-8') as f:
        for r in stored:
            if r.get('type') in IMU_TYPES and (r['sysid'],r['compid'])==(1,1):
                f.write(json.dumps(imu_record(r,Path(args.capture).stem),allow_nan=False)+'\n')
    print(json.dumps(dict(frames=report['frames'],bad_data_bytes=report['bad_data_bytes'],types=list(report['messages']),raw_replay_exact_after_header_normalization=True,legacy_unknown_header_corrections=corrections)))

if __name__=='__main__': main()
