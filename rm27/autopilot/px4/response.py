"""Local PX4 attitude/rate response bench with an explicit airborne fixture.

Not a dock flight or recovery acceptance run. Telemetry supplies control input;
MuJoCo truth supplies separate physics scoring. No hardware endpoint exists.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from rm27.integration.drone import add
from rm27.experiments.calibration import with_profile
from rm27.experiments.aggressive_contact.flight_control.plant import ContactPlant
from rm27.experiments.aggressive_contact.flight_control.estimator import TelemetryEstimator
from rm27.experiments.aggressive_contact.teacher_interface import RateThrust
from .runtime import Runtime,mavlink
from .control import ForceMapping,PX4Adapter

ROOT=Path(__file__).resolve().parents[3]


def reference(elapsed, layer, axis):
    """Balanced pulses leave no intentional final angle or lateral impulse."""
    value=0.
    if layer=='rate':
        if 1<=elapsed<1.5 or 2.5<=elapsed<3:value=.3
        elif 1.5<=elapsed<2.5:value=-.3
    else:
        if 1<=elapsed<2:value=math.radians(5)
        elif 2<=elapsed<3:value=-math.radians(5)
    result=np.zeros(3);result[axis]=value
    return result


def score(samples,layer,axis):
    measurements=np.asarray([s['measured'] for s in samples])
    desired=np.asarray([s['desired'] for s in samples])
    times=np.asarray([s['elapsed_s'] for s in samples])
    transitions=(1.,1.5,2.5) if layer=='rate' else (1.,2.)
    settle=.2 if layer=='rate' else .5
    steady=(times>=1)&(times<3)
    for change in transitions:steady &= ~((times>=change)&(times<change+settle))
    if not steady.any():raise ValueError('No steady response samples')
    error=measurements-desired
    rmse=float(np.sqrt(np.mean(error[steady,axis]**2)))
    bound=.15 if layer=='rate' else .05
    amplitude=.3 if layer=='rate' else math.radians(5)
    response=(times>=1)&(times<transitions[1])
    crossing=np.flatnonzero(response&(measurements[:,axis]>=.5*amplitude))
    latency=float(times[crossing[0]]-1) if len(crossing) else None
    terminal=times>=4
    terminal_rms=float(np.sqrt(np.mean(measurements[terminal,axis]**2))) if terminal.any() else None
    controls=np.asarray([s['controls'] for s in samples])
    saturation=np.any((controls>=.999)|(controls<=.001),axis=1)
    return dict(passed=rmse<=bound and latency is not None and latency<.5 and terminal_rms is not None and terminal_rms<bound,
        tracking_rmse=rmse,tracking_unit='rad/s' if layer=='rate' else 'rad',threshold=bound,
        transient_exclusion_s=settle,steady_samples=int(steady.sum()),response_half_amplitude_latency_s=latency,
        terminal_rms=terminal_rms,maximum_motor_fraction=float(controls.max()),
        saturated_sample_fraction=float(saturation.mean()),samples=len(samples),gate_version='PX4_RESPONSE_DEV_V2')


def plant(output,dynamics):
    root=ET.fromstring('<mujoco><option timestep=".001" integrator="RK4"/><asset/><worldbody/></mujoco>')
    config=with_profile(json.loads((ROOT/'configs/drone.json').read_text()),dynamics)
    info=add(root,'uav',[0,0,1.],[1,0,0,0],config)
    xml=ET.tostring(root,encoding='unicode');(output/'scene.xml').write_text(xml)
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    return ContactPlant((model,data,info,xml))


def run(output,*,dynamics=ROOT/'configs/dynamics_150g.json',layer='rate',axis=0,instance=25,duration=5.,profile='diagnostic'):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    p=plant(output,dynamics);estimator=TelemetryEstimator(p.origin);start=None;armed_at=None
    next_send=next_heartbeat=next_param=0.;next_action=10.;wall=time.monotonic();samples=[]
    summary=dict(passed=False,autopilot='px4',firmware='v1.17.0',run_type='closed-loop SITL airborne response bench',
                 layer=layer,axis=axis,profile=profile,mass_kg=p.info['mass_kg'],source='PX4 EKF telemetry with synthetic external vision',
                 initialization='explicit airborne fixture; no takeoff/landing qualification',failure=None)
    adapter=None
    try:
        with Runtime(p,output,instance,profile=profile) as rt,(output/'frames.jsonl').open('w') as stream:
            adapter=PX4Adapter(ForceMapping(4*p.maximum),rt.system)
            while time.monotonic()-wall<120:
                row=rt.step();t=float(p.data.time)
                for message in rt.receive():estimator.consume(message.to_dict(),t)
                if t>=next_heartbeat:rt.heartbeat();next_heartbeat=t+1
                if start is None:
                    if t>=next_param:rt.mav.param_request_list_send(rt.system,1);next_param=t+5
                    if t>=next_send:
                        action=RateThrust(np.zeros(3),p.info['mass_kg']*9.81)
                        rt.mav.set_attitude_target_send(**adapter.packet(action,t,rotation_nwu_flu=np.eye(3) if layer=='attitude' else None));next_send=t+.01
                    verified=all(k in rt.parameters and math.isclose(v,rt.parameters[k],rel_tol=1e-5,abs_tol=1e-5) for k,v in rt.expected.items())
                    if verified and t>=next_action:
                        next_action=t+2
                        if rt.telemetry.get('mode')!='OFFBOARD':rt.mode('OFFBOARD')
                        elif not rt.telemetry.get('armed'):rt.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
                    if rt.telemetry.get('armed') and rt.telemetry.get('mode')=='OFFBOARD':
                        if armed_at is None:armed_at=t
                        if t-armed_at>=4:
                            estimator.lock_clock();estimator.local_registration=estimator.position[1].copy();estimator.last_pos_time=-np.inf
                            p.release();start=t;next_send=t
                    if t>50:raise TimeoutError('response initialization timeout')
                else:
                    if not rt.telemetry.get('armed') or rt.telemetry.get('mode')!='OFFBOARD':raise RuntimeError('mode or arming lost')
                    observation=estimator.observation(t);s=observation.estimate
                    elapsed=t-start
                    desired=reference(elapsed,layer,axis)
                    physical_z=float(p.data.qpos[2]);physical_tilt=math.degrees(math.acos(np.clip(p.data.xmat[p.body].reshape(3,3)[2,2],-1,1)))
                    if physical_tilt>25 or abs(physical_z-p.origin[2])>.5 or np.linalg.norm(p.data.qpos[:2])>2.:
                        raise RuntimeError('response physical envelope exceeded')
                    if t>=next_send:
                        force=p.info['mass_kg']*max(0.,9.81+2*(p.origin[2]-s.position[2])-2*s.velocity[2])/max(.5,s.rotation[2,2])
                        force=float(np.clip(force,0.,4*p.maximum))
                        action=RateThrust(desired if layer=='rate' else np.zeros(3),force)
                        orientation=None if layer=='rate' else Rotation.from_rotvec(desired).as_matrix()
                        rt.mav.set_attitude_target_send(**adapter.packet(action,t,rotation_nwu_flu=orientation));next_send=t+.01
                    measured=s.omega_body if layer=='rate' else Rotation.from_matrix(s.rotation).as_rotvec()
                    sample=dict(timestamp=t,elapsed_s=elapsed,desired=desired.tolist(),measured=measured.tolist(),
                                truth=row['truth'],controls=row['controls'],position_estimate_nwu=s.position.tolist())
                    samples.append(sample);stream.write(json.dumps(sample,allow_nan=False)+'\n')
                    if elapsed>=duration:
                        summary['completed_duration_s']=elapsed;break
            if 'completed_duration_s' not in summary:raise TimeoutError('response wall deadline')
            (output/'parameters_readback.json').write_text(json.dumps(rt.parameters,indent=2)+'\n')
        summary.update(score(samples,layer,axis),fixture_released=True,
                       motor_order='custom PX4 allocator equals root physical order; no PWM permutation')
    except Exception as exc:
        summary['failure']=str(exc);(output/'failure.txt').write_text(traceback.format_exc())
    summary['runner_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2));return summary


def main():
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--output',type=Path,required=True)
    a.add_argument('--layer',choices=['rate','attitude'],default='rate');a.add_argument('--axis',choices=['roll','pitch','yaw'],default='roll')
    a.add_argument('--instance',type=int,default=25);a.add_argument('--dynamics',type=Path,default=ROOT/'configs/dynamics_150g.json')
    a.add_argument('--profile',choices=['diagnostic','response'],default='diagnostic')
    args=a.parse_args();r=run(args.output,layer=args.layer,axis=['roll','pitch','yaw'].index(args.axis),instance=args.instance,dynamics=args.dynamics,profile=args.profile)
    return 0 if r['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
