"""Episode/scenario split, immutable new teacher draws, no old holdout access."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ..model import ROOT,VehicleEstimate,TargetState,unit,load_config
from ..teacher import prepare
from ..maneuver import episode
from ..teacher_data import export
from ..evidence import snapshot,write_json,save_failure
from .observation import CausalState,vector,history_vectors,SyntheticEstimator


def settings():return json.loads((ROOT/'configs/contact_distillation.json').read_text())
def designs():return {k:json.loads((ROOT/f'configs/contact_teacher_{k}.json').read_text()) for k in settings()['modes']}


def generate(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);cfg=load_config();snapshot(output,cfg)
    study=settings();write_json(output/'study.json',study);prepared,template=prepare(cfg,output/'plant');rows=[]
    with (output/'progress.jsonl').open('w',buffering=1) as log:
        for split,seeds in [('train',study['train_seeds']),('validation',study['validation_seeds'])]:
            for mode,p in designs().items():
                for seed in seeds:
                    scale=1. if seed%2==0 else 2.
                    dest=output/'episodes'/split/mode/f'seed-{seed}'
                    try:r=episode(cfg,p,dest,seed,scale,prepared,compact=True,template=template)
                    except Exception as exc:r=save_failure(dest,exc)
                    if 'RUNNER_ERROR' not in r['failure_modes']:
                        record=export(dest,output/'dataset'/split/mode/f'seed-{seed}',split,mode)
                        rows.append(record)
                    log.write(json.dumps(dict(split=split,mode=mode,seed=seed,passed=r['passed'],failures=r['failure_modes']))+'\n')
    write_json(output/'index.json',rows)
    result=dict(passed=len(rows)==3*(len(study['train_seeds'])+len(study['validation_seeds'])),episodes=len(rows),
                successes=sum(r['passed'] for r in rows),failed_sources=[r['path'] for r in rows if not r['passed']],
                scope='dataset pipeline; full seed split, failures retained separately, no historical holdout used')
    write_json(output/'summary.json',result);return result


def load_episode(path,mode,info,observer=True,degradation=None):
    path=Path(path);a=np.load(path/'demo.npz');m=json.loads((path/'metadata.json').read_text());c=m['task_config']
    target=TargetState(np.array(c['target']['center_m']),unit(c['target']['normal']),np.zeros((3,3)),0)
    ego=a['observation_ego'];t=a['time_s'];duration=m['outcome']['parameters']['duration_s']
    states=[VehicleEstimate(row[:3],row[3:6],Rotation.from_quat(row[[7,8,9,6]]).as_matrix(),row[10:13],float(clock),'DATASET_EGO') for row,clock in zip(ego,t)]
    causal=CausalState(c,info,target,states[0]);previous=np.r_[np.zeros(3),info['mass_kg']*9.81];obs=[]
    estimator=SyntheticEstimator(seed=int(m['outcome'].get('seed') or 0)+1000000,**(degradation or {}))
    for i,(state,clock) in enumerate(zip(states,t)):
        sensed,age=estimator.update(state) if degradation else (state,0.)
        act=causal.update(sensed,clock)
        obs.append(vector(sensed,target,act,clock,duration,mode,previous,causal.contact_time,age,observer))
        previous=a['teacher_rate_thrust'][i];causal.previous_motor=a['teacher_motor_N'][i]
    # Corrective/contact windows keep all frames; long steady hold is subsampled.
    contact=m['outcome'].get('first_contact');tc=None if contact is None else contact['time_s']
    weights=np.arange(len(t))%2==0
    if tc is not None:weights|=(t>=tc-.3)&(t<=tc+.5)
    return np.array(obs),a['teacher_rate_thrust'],weights,m


def arrays(root,lags=(0,),observer=True,extra=None,degradation=None):
    from ..model import build_model,scenario
    cfg=load_config();_,_,info,_=build_model(scenario(cfg),cfg)
    entries=json.loads((Path(root)/'index.json').read_text());groups={}
    for record in entries:
        group=record['scenario_group']
        if group in groups and groups[group]!=record['split']:raise ValueError('Scenario crosses episode split')
        groups[group]=record['split']
    out={s:[] for s in ('train','validation')};sources=[]
    for record in entries:
        if record['split'] not in out:raise ValueError('Holdout data is forbidden for fitting')
        # Failed trajectories are retained in the index for failure/ood analysis,
        # not blindly treated as successful expert action targets.
        if not record['passed']:continue
        mode=settings()['modes'].index(record['teacher'])
        X,Y,mask,m=load_episode(record['path'],mode,info,observer,degradation)
        out[record['split']].append((history_vectors(X,lags)[mask],Y[mask]))
        sources.append(dict(path=record['path'],split=record['split'],scenario_group=record['scenario_group'],
                            sha256=hashlib.sha256((Path(record['path'])/'demo.npz').read_bytes()).hexdigest()))
    if extra:
        declaration=json.loads((Path(extra)/'policy.json').read_text())
        if declaration['split']!='train' or not declaration['oracle']:raise ValueError('Corrective data must be declared training-only oracle rollouts')
        for path in sorted(Path(extra).glob('**/student.npz')):
            a=np.load(path);X=a['observation'];Y=a['oracle_label'];finite=np.isfinite(Y).all(1)
            # Full on-policy trajectories including failed trials, no filtering by success.
            out['train'].append((history_vectors(X,lags)[finite][::2],Y[finite][::2]))
            sources.append(dict(path=str(path),split='train_dagger',sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    return (*[np.concatenate([v[i] for v in out[s]]) for s in ('train','validation') for i in (0,1)],sources)
