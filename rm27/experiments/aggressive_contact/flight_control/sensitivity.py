"""Paired full-field estimator study. Frozen configuration, no fitting here."""
import argparse,copy,json,time,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ..model import ROOT,load_config,TargetState,frame
from ..evidence import snapshot,write_json,save_failure
from ..teacher import prepare
from ..maneuver import episode
from ..distillation.dataset import designs
from ..distillation.deployment import DeploymentPolicy,TargetEstimate
from ..teacher_interface import apply_proxy
from .synthetic import ErrorSensor
from .quality import EstimateQuality,CommitmentGuard,select_mode


class EstimatedSkill:
    def __init__(self,teacher,initial,target,info,case,requested,variant):
        self.teacher=teacher;self.sensor=ErrorSensor(case.get('sensor',{}));self.records=[]
        q=EstimateQuality.from_config(case.get('quality',{}));self.quality=q
        target=TargetEstimate(target.position+np.array(case.get('target_bias_m',[0,0,0])),
             Rotation.from_rotvec(np.deg2rad(case.get('target_rotation_deg',[0,0,0]))).apply(target.normal),
             q.target_covariance_m2,initial.timestamp,1.)
        mode=requested
        if variant=='scheduled':
            mode=select_mode(requested,q,target.normal_nwu,np.linalg.norm(teacher.cfg['contact']['offset_body_m']),np.eye(3)*.02**2,case.get('sensor',{}).get('delay_s',0.))
        self.selected=mode
        self.policy=DeploymentPolicy.from_bundle(ROOT/'artifacts/phase3-student-bundle-02',initial,target,mode=mode)
        self.actor=self.policy.actor
        self.guard=CommitmentGuard(self.actor,q) if variant in ('guard','scheduled') else None

    def __getattr__(self,name):
        try:return getattr(self.actor,name)
        except AttributeError:return getattr(self.teacher,name) # evaluator reference only

    def command(self,state,contact):
        obs=self.sensor.observation(state)
        if self.guard:self.guard.update(obs,state.timestamp)
        action=self.policy.infer(obs,state.timestamp)
        c=self.actor.cfg['controller'];gain=2*c['attitude_damping']*c['attitude_frequency_rad_s']
        motor,sat=apply_proxy(action,state.omega_body,self.actor.inertia,self.actor.inverse_mixer,gain,self.actor.max_motor,return_saturation=True)
        self.records.append(np.r_[state.timestamp,obs.estimate.timestamp,obs.estimate.position,obs.estimate.velocity,obs.estimate.rotation.ravel()])
        return motor,sat


def run(config,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);cfg=load_config();snapshot(output,cfg)
    spec=json.loads(Path(config).read_text());write_json(output/'manifest.json',dict(study=spec,config_sha256=hashlib.sha256(Path(config).read_bytes()).hexdigest(),
        checkpoint_sha256=hashlib.sha256((ROOT/'artifacts/phase3-student-bundle-02/policy.npz').read_bytes()).hexdigest()))
    prepared,template=prepare(cfg,output/'plant');results=[]
    with (output/'progress.jsonl').open('w',buffering=1) as log:
        for case in spec['cases']:
            for requested in spec.get('modes',['balanced']):
                for variant in spec.get('variants',['fixed']):
                    holder=[];dest=output/case['name']/requested/variant;t0=time.perf_counter()
                    def factory(teacher,initial,target,info):
                        x=EstimatedSkill(teacher,initial,target,info,case,requested,variant);holder.append(x);return x
                    try:
                        r=episode(cfg,designs()[requested],dest,case.get('physical_seed'),case.get('physical_scale',1.),prepared,compact=True,template=template,skill_factory=factory)
                        actor=holder[0];actor.actor.save(dest/'student.npz');np.savez_compressed(dest/'estimates.npz',trace=actor.records)
                        events=actor.guard.events if actor.guard else [];aborted=bool(actor.guard and actor.guard.aborted)
                        x=np.load(dest/'flight/trajectory.npz')['trace'];last=x[-1]
                        tilt=np.rad2deg(np.arccos(np.clip(Rotation.from_quat(last[[5,6,7,4]]).as_matrix()[2,2],-1,1)))
                        safe_abort=bool(aborted and r['first_contact'] is None and np.linalg.norm(last[8:11])<.3 and tilt<10 and np.linalg.norm(last[11:14])<.524 and r['min_altitude_m']>.5 and
                            set(r['failure_modes'])<={'TARGET_MISS','RECOVERY_TIMEOUT','NO_SEPARATION'})
                        r.update(requested_mode=requested,selected_mode=actor.selected,variant=variant,case=case['name'],aborted=aborted,safe_abort=safe_abort,
                                 safe_outcome=bool(r['passed'] or safe_abort),commit_events=events,scope='MuJoCo full field, estimated-ego hybrid + rate-P proxy, not firmware',wall_s=time.perf_counter()-t0)
                        write_json(dest/'summary.json',r)
                    except Exception as exc:r=save_failure(dest,exc)
                    row=dict(case=case['name'],mode=requested,variant=variant,result=r);results.append(row);log.write(json.dumps(row)+'\n')
                    print(case['name'],requested,variant,r['passed'],r.get('safe_abort'),r['failure_modes'],flush=True)
    result=dict(passed=all('RUNNER_ERROR' not in x['result']['failure_modes'] for x in results),scope='runner integrity only; per-flight contact and safe-abort are separate',results=results)
    write_json(output/'summary.json',result);return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--output',required=True);a=p.parse_args();r=run(a.config,a.output);raise SystemExit(0 if r['passed'] else 1)
