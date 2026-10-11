"""Apply the unchanged contact/recovery evaluator to recorded SITL motor inputs.

This replay is an evaluator adapter, NOT another independent closed-loop trial.
Its state match is checked against the original firmware-coupled flight.
"""
import argparse,json
from pathlib import Path
import numpy as np
import mujoco
from ..model import ROOT,TargetState,unit,truth_estimate
from ..maneuver import ManeuverSkill,metrics
from ..experiment import run_episode
from ..evidence import write_json
from ..distillation.dataset import designs


def evaluate(source,output):
    source,output=Path(source),Path(output);output.mkdir(parents=True,exist_ok=False)
    a=np.load(source/'flight.npz');x=a['trace'];release=np.load(source/'release_state.npz')
    config=source/'plant_config.json'
    if not config.exists():config=ROOT/'artifacts/phase3-student-bundle-02/nominal_config.json'
    c=json.loads(config.read_text());c['physics_dt_s']=.001;c['control_dt_s']=.002;c['horizon_s']=float(x[-1,0])
    model=mujoco.MjModel.from_binary_path(str(source/'plant/model.mjb'));model.opt.timestep=.001;data=mujoco.MjData(model)
    data.qpos[:]=release['qpos'];data.qvel[:]=release['qvel'];data.act[:]=release['act'];mujoco.mj_forward(model,data)
    info_path=source/'vehicle_info.json'
    info=json.loads(info_path.read_text()) if info_path.exists() else json.loads((ROOT/'artifacts/phase3-student-bundle-02/nominal_vehicle.json').read_text())['vehicle_info']
    initial=truth_estimate(model,data);target=TargetState(np.array(c['target']['center_m']),unit(c['target']['normal']),np.zeros((3,3)),0.)
    spec=json.loads((source/'launch.json').read_text());mode=spec.get('skill_mode','balanced')
    design_path=source/'teacher_design.json'
    design=json.loads(design_path.read_text()) if design_path.exists() else designs()[mode]
    class Replay(ManeuverSkill):
        def command(self,s,contact):
            super().command(s,contact) # evaluator reference/events only; discarded control
            # Each recorded row is the post-step state for its applied motor
            # command. Preserve that command across both 1 ms substeps and
            # across 4 ms PX4 cycles, rather than assuming AP's 2 ms packets.
            i=min(len(x)-1,int(np.searchsorted(x[:,0],s.timestamp+1e-9,side='right')));u=x[i,18:22]
            return u,bool(np.any(u<1e-8) or np.any(u>self.max_motor-1e-8))
    skill=Replay(c,design,initial,target,info,initial.position)
    r=run_episode(c,skill.p,output=output/'flight',plant_bundle=(model,data,info,(source/'plant/scene.xml').read_text()),initial_skill=skill)
    y=np.load(output/'flight/trajectory.npz')['trace'];r.update(metrics(r,y,target.normal,design))
    # Evaluator samples start at zero; compare overlapping original post-step states.
    indices=np.rint(x[:,0]/.002).astype(int);valid=indices<len(y)
    error=float(np.max(abs(y[indices[valid],1:18]-x[valid,1:18])))
    r.update(replay_max_state_error=error,replay_valid=error<1e-6,original_source=str(source),
             mode='RECORDED_SITL_MOTOR_INPUT_EVALUATION_REPLAY',
             reference_diagnostics='shadow evaluator only; actual policy estimates and commands are in the original flight',
             scope='unchanged contact/recovery gates on checked recorded-SITL-input replay; not a new independent flight')
    r['passed']=bool(r['passed'] and r['replay_valid']);write_json(output/'summary.json',r);return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);a=p.parse_args();r=evaluate(a.input,a.output);print(json.dumps(r,indent=2))
