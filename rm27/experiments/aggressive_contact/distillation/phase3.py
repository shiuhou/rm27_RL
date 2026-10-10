"""Frozen paired Phase-3 studies. Every variant uses the same physical draws.

python -m ...distillation.phase3 run --config ... --output ...
Config and all policies are hashed BEFORE rollouts. No fitting in this runner.
"""
import argparse,copy,hashlib,json,time
from pathlib import Path
import numpy as np
from ..evidence import snapshot,write_json,save_failure
from ..model import load_config,TargetState
from ..teacher import prepare,stats
from ..maneuver import episode
from .dataset import designs
from .policy import StudentSkill
from .network import MLP
from .spline import SplinePolicy,fit
from .deployment import TransformedReference


class ModelReference:
    def __init__(self,teacher):self.teacher=teacher
    def reference(self,mode):return self
    def sample(self,t):
        duration=self.teacher.p['duration_s'];p,v,a,j=self.teacher.trajectory.sample(min(t,duration))
        if t>duration:
            h=min(.25,t-duration);p=p+v*h+.5*a*h*h;v=v+a*h
        return p,v,a,j


def run(config,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    spec=json.loads(Path(config).read_text());cfg=load_config();snapshot(output,cfg)
    hashes={v['checkpoint']:hashlib.sha256(Path(v['checkpoint']).read_bytes()).hexdigest() for v in spec['variants'].values() if v.get('checkpoint')}
    write_json(output/'frozen_manifest.json',dict(study=spec,checkpoint_sha256=hashes,config_sha256=hashlib.sha256(Path(config).read_bytes()).hexdigest()))
    prepared,template=prepare(cfg,output/'plant');results={}
    with (output/'progress.jsonl').open('w',buffering=1) as log:
        for name,variant in spec['variants'].items():
            results[name]={}
            kind=variant.get('kind','hybrid');checkpoint=variant.get('checkpoint')
            model=(MLP if kind=='bc' else SplinePolicy).load(checkpoint) if checkpoint else None
            for mode_name in spec.get('modes',['aggressive','balanced','conservative']):
                mode=['aggressive','balanced','conservative'].index(mode_name);design=designs()[mode_name]
                if kind=='adaptive':design=dict(method='staged',terminal=cfg['nominal_skill'])
                rows=[]
                for seed,scale in spec['cases']:
                    dest=output/name/mode_name/f'seed-{seed}';holder=[];begin=time.perf_counter()
                    def factory(teacher,initial,target,info):
                        nominal=target
                        noise=variant.get('target_position_std_m',0.);angle=variant.get('target_normal_std_deg',0.)
                        if noise or angle:
                            from scipy.spatial.transform import Rotation
                            rng=np.random.default_rng(seed+2000000)
                            target=TargetState(target.position+rng.normal(0,noise,3),Rotation.from_rotvec(rng.normal(0,np.deg2rad(angle),3)).apply(target.normal),np.eye(3)*noise**2,initial.timestamp)
                        actor=StudentSkill(teacher,initial,target,info,ModelReference(teacher) if kind=='model_reference' else model,mode,
                             dict(variant.get('estimator',{}),seed=seed+1000000),hybrid=kind!='bc',
                             oracle=variant.get('oracle',False),intervention=variant.get('intervention',0.),
                             detector_window_s=variant.get('detector_window_s',.02),
                             action_delay_s=variant.get('action_delay_s',0.),deployment=variant.get('deployment'))
                        if kind!='bc' and (noise or angle):actor.reference=TransformedReference(actor.reference,nominal,target)
                        holder.append(actor);return actor
                    try:
                        r=episode(cfg,design,dest,seed,scale,prepared,compact=True,template=template,
                                  skill_factory=None if kind in ('teacher','adaptive') else factory,
                                  action_proxy={'hz':200.} if kind in ('teacher','adaptive') else None)
                        if holder:
                            actor=holder[0];actor.save(dest/'student.npz')
                            r['estimated_contact_time_s']=actor.contact_time
                            r['contact_detection_delay_s']=None if actor.contact_time is None or r['first_contact'] is None else actor.contact_time-r['first_contact']['time_s']
                        r['wall_time_s']=time.perf_counter()-begin;write_json(dest/'summary.json',r)
                    except Exception as exc:r=save_failure(dest,exc)
                    rows.append(r)
                    log.write(json.dumps(dict(variant=name,mode=mode_name,seed=seed,scale=scale,passed=r['passed'],failures=r['failure_modes']))+'\n')
                results[name][mode_name]=stats(rows,.008)
                write_json(output/'partial.json',results)
    r=dict(passed=all('RUNNER_ERROR' not in x['failures'] for modes in results.values() for x in modes.values()),scope='runner integrity; successes are per-flight',results=results)
    write_json(output/'summary.json',r);return r


def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['run','fit','mission']);p.add_argument('--config',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--input',type=Path);p.add_argument('--intervals',type=int,default=64)
    a=p.parse_args()
    try:
        if a.command=='run':r=run(a.config,a.output)
        elif a.command=='fit':r=fit(a.input,a.output,a.intervals)
        else:
            from .mission import run as mission
            spec=json.loads(a.config.read_text())
            r=mission(spec['checkpoint'],a.output,mode=spec.get('mode','balanced'),seed=spec.get('seed'),
                      deployment=spec.get('deployment'),estimator_override=spec.get('estimator',{}))
    except Exception as exc:
        save_failure(a.output,exc);raise
    print(json.dumps(r,indent=2))
    raise SystemExit(0 if r['passed'] else 1)

if __name__=='__main__':main()
