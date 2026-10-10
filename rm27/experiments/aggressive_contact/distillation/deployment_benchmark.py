"""Recorded-estimate standalone API equivalence, host cost and wire projection."""
import argparse,json,hashlib,time,resource,platform
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ..model import VehicleEstimate,unit
from ..evidence import write_json
from .deployment import DeploymentPolicy,VehicleObservation,TargetEstimate,ThrottleCalibration,ArduPilotAdapter


def run(bundle,source,output):
    bundle,source,output=Path(bundle),Path(source),Path(output);output.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((bundle/'nominal_config.json').read_text());a=np.load(source/'student.npz');trace=np.load(source/'flight/trajectory.npz')['trace'];r=trace[0]
    initial=VehicleEstimate(r[1:4],r[8:11],Rotation.from_quat(r[[5,6,7,4]]).as_matrix(),r[11:14],r[0],'INITIAL_ESTIMATE')
    target=TargetEstimate(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),initial.timestamp,1.)
    api=DeploymentPolicy.from_bundle(bundle,initial,target);errors=[];cost=[];messages=[]
    info=json.loads((bundle/'nominal_vehicle.json').read_text())['vehicle_info']
    adapter=ArduPilotAdapter(ThrottleCalibration((0.,4*info['max_rotor_thrust_N_assumed']),(0.,1.),'NOMINAL LINEAR PROXY ONLY, NOT AP/HARDWARE CALIBRATION'),True)
    for clock,x,age,expected in zip(a['time_s'],a['observation'],a['observation_age_s'],a['student_action']):
        R=np.column_stack((x[11:14],x[14:17],np.cross(x[11:14],x[14:17])))
        estimate=VehicleEstimate(x[5:8]+target.position_nwu_m,x[8:11],R,x[17:20],clock-age,'RECORDED_SYNTHETIC_ESTIMATE')
        measurement=VehicleObservation(estimate,np.eye(3)*.02**2)
        start=time.perf_counter_ns();action=api.infer(measurement,float(clock));cost.append((time.perf_counter_ns()-start)/1000)
        got=np.r_[action.body_rates_flu_rad_s,action.collective_force_N];errors.append(np.max(abs(got-expected)))
        messages.append(adapter.packet(action,clock))
    write_json(output/'mavlink_fields.json',messages)
    r=dict(passed=bool(max(errors)<1e-8),samples=len(cost),max_action_error=float(max(errors)),max_observation_age_s=float(a['observation_age_s'].max()),
        inference_us=dict(median=float(np.median(cost)),p95=float(np.quantile(cost,.95)),p99=float(np.quantile(cost,.99)),max=float(max(cost))),
        process_max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,host=platform.platform(),
        scope='full standalone estimate/covariance/observer/policy/proxy API excluding physics and packet serialization; host only, no M3C/SITL',
        checkpoint_sha256=hashlib.sha256((bundle/'policy.npz').read_bytes()).hexdigest(),
        messages='fields built offline only, never transmitted; force/throttle lookup intentionally nominal test assumption')
    write_json(output/'summary.json',r);return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',required=True);p.add_argument('--input',required=True);p.add_argument('--output',required=True);a=p.parse_args();print(json.dumps(run(a.bundle,a.input,a.output),indent=2))
