"""Offline diagnostic means and bounds. NO serial/network, no OpenVINS exporter."""
import importlib.util
import json
import math
from pathlib import Path
import re

HERE=Path(__file__).resolve().parent
UNITS={'timestamp':'us','dt':'us','delta_angle':'rad','delta_velocity':'m/s','frame':'BODY_FRD'}
spec=importlib.util.spec_from_file_location('prior_imu',HERE.parent/'imu-input-20261010/imu_input.py')
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)

def need(value,message):
    if not value:raise ValueError(message)

def diagnostic(r,*,units,epoch):
    need(units==UNITS,'explicit SI integral units and body frame required')
    need(isinstance(epoch,str) and epoch not in ('','UNKNOWN'),'explicit evidence segment required')
    for k in ('timestamp','timestamp_sample','delta_angle_dt','delta_velocity_dt','accel_device_id','gyro_device_id'):
        need(type(r.get(k)) is int and r[k]>0,'positive integer required: '+k)
        bits=64 if k in ('timestamp','timestamp_sample') else 32
        need(r[k]<2**bits,'out of wire range: '+k)
    need(r['timestamp']>=r['timestamp_sample']>=r['delta_angle_dt'],'invalid source chronology')
    for k in ('accel_calibration_count','gyro_calibration_count'):
        need(type(r.get(k)) is int and 0<=r[k]<=255,'invalid calibration counter')
    for k in ('delta_angle_clipping','delta_velocity_clipping'):
        need(type(r.get(k)) is int and 0<=r[k]<=7,'invalid clipping bits')
    for k in ('delta_angle','delta_velocity'):
        v=r.get(k)
        need(isinstance(v,list) and len(v)==3 and all(type(x) in (int,float) and abs(x)<=3.4028234663852886e38 for x in v),'finite float32 3-vector required')
    return dict(original=dict(r),epoch=epoch,source_domain='PX4_HRT',
        gyro_mean_rad_s=[x*1e6/r['delta_angle_dt'] for x in r['delta_angle']],
        accel_mean_m_s2=[x*1e6/r['delta_velocity_dt'] for x in r['delta_velocity']],
        gyro_interval_us=[r['timestamp_sample']-r['delta_angle_dt'],r['timestamp_sample']],
        accel_end_us=None,measurement_semantic='CALIBRATED_CONING_INTEGRAL_MEAN',
        timestamp_semantic='GYRO_INTEGRATION_END',mapped_ns=None,clock_status='UNKNOWN',
        openvins_admissible=False)

def audit(rows,*,units,epoch,tolerance_us=2):
    need(bool(rows),'empty evidence')
    need(type(tolerance_us) is int and tolerance_us>=0,'integer tolerance required')
    means=[diagnostic(r,units=units,epoch=epoch) for r in rows]
    issues=set();residual=[]
    for r in rows:
        if r['delta_angle_clipping'] or r['delta_velocity_clipping']:issues.add('clipping')
    for a,b in zip(rows,rows[1:]):
        delta=b['timestamp_sample']-a['timestamp_sample']
        if delta<=0:issues.add('ordering')
        if b['timestamp']<=a['timestamp']:issues.add('publication_ordering')
        residual.append(delta-b['delta_angle_dt'])
        if delta>0 and abs(residual[-1])>tolerance_us:issues.add('gap' if residual[-1]>0 else 'overlap')
        for k in ('accel_device_id','gyro_device_id'):
            if a[k]!=b[k]:issues.add('identity')
        for k in ('accel_calibration_count','gyro_calibration_count'):
            if a[k]!=b[k]:issues.add('calibration')
    return dict(count=len(rows),issues=sorted(issues),gyro_coverage_residual_us=residual,
        source_intervals=prior.intervals([r['timestamp_sample'] for r in rows],1e6),
        diagnostic_means=means,accel_coverage='UNKNOWN_NO_SEPARATE_ENDPOINT',
        openvins_admissible=False,limits='Coverage residual is diagnostic, not exact sensor loss. Segment changes; no repair/interpolation.')

def cdr_size(path):
    """Scalar fixed-field message subset; follows pinned ucdr template alignment."""
    sizes={'uint64':8,'uint32':4,'int32':4,'float32':4,'uint8':1}
    offset=0
    for line in Path(path).read_text(encoding='utf-8-sig').splitlines():
        line=line.split('#')[0].strip()
        if not line or '=' in line:continue
        m=re.fullmatch(r'(\w+)(?:\[(\d+)\])?\s+\w+',line)
        need(m and m[1] in sizes,'unsupported message layout')
        size=sizes[m[1]];offset+=(-offset)%size+size*int(m[2] or 1)
    return offset

def xrce_budget(payload,hz,baud):
    need(type(payload) is int and payload>0 and hz>0 and baud>0,'positive budget inputs')
    # Session 0x81:4B header,4B subheader,4B WRITE_DATA object/request fields.
    # Serial:1 flag plus4 addresses/length plus2 CRC; everything but flag escaped.
    minimum=payload+12+7
    worst=1+2*(minimum-1)
    # Engineering model only: 1% stuffing,20% control/other-topic reservation.
    planned=(minimum+.01*(minimum-1))*hz*1.2
    return dict(payload=payload,hz=hz,baud=baud,minimum_bytes_s=minimum*hz,
        max_stuffed_bytes_s=worst*hz,planning_bytes_s=planned,
        minimum_utilization=minimum*hz*10/baud,max_utilization=worst*hz*10/baud,
        planning_utilization=planned*10/baud,
        caveat='One unfragmented WRITE_DATA/frame; not measured; setup/retries/unbudgeted topics excluded from strict bounds')

def saved_listener():
    records=[json.loads(x) for x in (HERE.parent/'imu-transport-20261010/control-session.jsonl').read_text().splitlines()]
    result={}
    for rec in records:
        command=rec.get('command','')
        if not command.startswith('listener vehicle_imu'):continue
        blocks=re.split(r'TOPIC: vehicle_imu instance \d+ #\d+',rec['output'])[1:]
        rows=[]
        for block in blocks:
            fields={}
            for key,value in re.findall(r'^\s+(\w+): (.+)$',block,re.M):
                value=value.strip()
                if value.startswith('['):fields[key]=json.loads(value)
                elif re.match(r'\d+',value):fields[key]=int(re.match(r'\d+',value)[0])
            rows.append(fields)
        result[command]=dict(provenance='REAL_SAVED_CONSOLE_ROUNDED_5_DECIMAL_VECTORS_NOT_FULL_STREAM',
            audit=audit(rows,units=UNITS,epoch='saved-control-session'))
    return result

def main():
    result=dict(saved_console=saved_listener(),budgets={},source='OFFLINE_ONLY')
    for name,path in [('vehicle_imu','imu-input-20261010/msg__VehicleImu.msg'),
        ('sensor_combined','imu-input-20261010/msg__SensorCombined.msg'),
        ('sensor_gyro','imu-transport-20261010/msg__SensorGyro.msg')]:
        size=cdr_size(HERE.parent/path)
        result['budgets'][name]=[xrce_budget(size,200,b) for b in (115200,230400,460800,921600)]
    with (HERE/'offline-results.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(dict(console={k:dict(count=v['audit']['count'],issues=v['audit']['issues']) for k,v in result['saved_console'].items()},
        vehicle_imu=result['budgets']['vehicle_imu'])))

if __name__=='__main__':main()
