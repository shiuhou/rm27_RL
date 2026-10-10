"""Offline budget and one-axis observation-model examples. No device APIs."""
import json
import math
from pathlib import Path
import re
import struct
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
import offline as prior

def profile(hz,baud):
    prior.need(type(hz) is int and hz>0 and type(baud) is int and baud>0,'positive integer rate/baud')
    # Fixed pinned schemas, not a generic replacement for PX4 code generation.
    selection=re.findall(r'^(uint\d+|int\d+)\s+(\w+)',(HERE/'SensorSelection.msg').read_text(encoding='utf-8-sig'),re.M)
    prior.need(selection==[('uint64','timestamp'),('uint32','accel_device_id'),('uint32','gyro_device_id')],'selection schema changed')
    status=re.findall(r'^(uint\d+|int\d+)\s+(\w+)([^\n]*)',(HERE/'TimesyncStatus.msg').read_text(encoding='utf-8-sig'),re.M)
    status=[(t,n) for t,n,rest in status if '=' not in rest.split('#')[0]]
    prior.need(status==[('uint64','timestamp'),('uint8','source_protocol'),('uint64','remote_timestamp'),
        ('int64','observed_offset'),('int64','estimated_offset'),('uint32','round_trip_time')],'timesync schema changed')
    sizes={'vehicle_imu':prior.cdr_size(HERE.parent.parent/'imu-input-20261010/msg__VehicleImu.msg'),
           'sensor_selection':struct.calcsize('<QII'),'timesync_status':struct.calcsize('<QB7xQqqI')}
    # Metadata budget ceilings, not claimed producer publication rates.
    rows={k:prior.xrce_budget(v,hz if k=='vehicle_imu' else 10,baud) for k,v in sizes.items()}
    result={k:sum(r[k] for r in rows.values()) for k in
        ('minimum_bytes_s','max_stuffed_bytes_s','planning_bytes_s','minimum_utilization','max_utilization','planning_utilization')}
    return dict(result,payload_sizes=sizes,topic_budgets=rows,
        limits='Proposed minimal export profile only; metadata each capped at10Hz; selection event-driven. Not stock DDS total. No measured throughput.')

def scalar_case(bias,slope,endpoints_us):
    prior.need(all(type(x) in (int,float) and math.isfinite(x) for x in (bias,slope)),'finite rate/rate-slope')
    prior.need(len(endpoints_us)==3 and all(type(x) is int and x>=0 for x in endpoints_us)
        and all(b>a for a,b in zip(endpoints_us,endpoints_us[1:])),'three ordered integer endpoints')
    means=[];rows=[]
    for a,b in zip(endpoints_us,endpoints_us[1:]):
        start,end=a/1e6,b/1e6
        delta=bias*(end-start)+.5*slope*(end*end-start*start)
        r=dict(timestamp=b+100,timestamp_sample=b,gyro_device_id=6684690,accel_device_id=6946834,
            delta_angle=[delta,0,0],delta_velocity=[0,0,0],delta_angle_dt=b-a,delta_velocity_dt=b-a,
            gyro_calibration_count=0,accel_calibration_count=0,delta_angle_clipping=0,delta_velocity_clipping=0)
        rows.append(r)
        means.append(prior.diagnostic(r,units=prior.UNITS,epoch='SYNTHETIC_SCALAR_MODEL'))
    dt=(endpoints_us[2]-endpoints_us[1])/1e6
    point_delta=.5*(means[0]['gyro_mean_rad_s'][0]+means[1]['gyro_mean_rad_s'][0])*dt
    true_delta=rows[1]['delta_angle'][0]
    return dict(provenance='SYNTHETIC_SCALAR_MODEL_NOT_CAPTURE_NOT_OPENVINS_RUN',
        endpoints_us=endpoints_us,angular_rate_intercept_rad_s=bias,angular_acceleration_rad_s2=slope,
        retained_interval_delta_rad=true_delta,endpoint_point_model_delta_rad=point_delta,
        difference_rad=point_delta-true_delta,openvins_admissible=False,
        limits='One-axis commuting rotations, zero sensor/filter/calibration error. Counterexample to equivalence, not predicted PX4/OpenVINS accuracy.')

if __name__=='__main__':
    result=dict(profiles={f'{hz}Hz_{baud}':profile(hz,baud) for hz in (200,400) for baud in (115200,460800,921600)},
        cases=[scalar_case(1,0,[0,5000,10000]),scalar_case(0,100,[0,5000,10000]),scalar_case(0,100,[0,4000,10000])])
    with (HERE/'results.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(result))
