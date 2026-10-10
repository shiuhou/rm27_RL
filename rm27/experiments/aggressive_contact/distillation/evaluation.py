"""Full-field paired closed-loop imitation evaluation, no fitting on holdout."""
import json,time,platform,hashlib,resource
from pathlib import Path
import numpy as np
from ..teacher import prepare,stats
from ..maneuver import episode
from ..model import load_config
from ..evidence import snapshot,write_json,save_failure
from .dataset import settings,designs
from .network import MLP
from .spline import SplinePolicy
from .policy import StudentSkill

PROFILES={
 'perfect':{},
 'noise':dict(position_std=.002,velocity_std=.02,attitude_std_deg=.2,rate_std=.02),
 'delay10':dict(delay_s=.01),
 'dropout':dict(dropout=.2),
 'noise_delay':dict(position_std=.002,velocity_std=.02,attitude_std_deg=.2,rate_std=.02,delay_s=.01),
 'action_delay20':{},
 'noise_filtered':dict(position_std=.002,velocity_std=.02,attitude_std_deg=.2,rate_std=.02),
 'noise_delay_filtered':dict(position_std=.002,velocity_std=.02,attitude_std_deg=.2,rate_std=.02,delay_s=.01),
 'perfect_filtered':{},
 'dropout_filtered':dict(dropout=.2),
}

def run(checkpoint,output,split='validation',profile='perfect',hybrid=False,oracle=False,observer=True,mix=0.,baseline=False):
    if mix and (not oracle or split!='train'):raise ValueError('Intervention only allowed for training oracle collection')
    output=Path(output);output.mkdir(parents=True,exist_ok=False);cfg=load_config();snapshot(output,cfg)
    study=settings();write_json(output/'study.json',study)
    if split=='train':seeds=study['train_seeds'][:8]
    elif split=='validation':seeds=study['validation_seeds']
    elif split=='holdout_small':seeds=study['holdout_small_seeds']
    elif split=='holdout_stress':seeds=study['holdout_stress_seeds']
    elif split=='demo':seeds=[None]
    else:raise ValueError(split)
    model=None if checkpoint is None else (SplinePolicy if hybrid else MLP).load(checkpoint)
    prepared,template=prepare(cfg,output/'plant');results={}
    write_json(output/'policy.json',dict(checkpoint=None if checkpoint is None else str(checkpoint),
       sha256=None if checkpoint is None else hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest(),profile=profile,hybrid=hybrid,oracle=oracle,observer=observer,split=split,intervention=mix,baseline=baseline))
    with (output/'progress.jsonl').open('w',buffering=1) as log:
        choices={'adaptive_nominal':dict(method='staged',terminal=cfg['nominal_skill'])} if baseline else designs()
        for mode,(name,design) in enumerate(choices.items()):
            rows=[]
            for seed in seeds:
                scale=2. if split=='holdout_stress' else 1. if split=='holdout_small' or seed is None or seed%2==0 else 2.
                path=output/name/f'seed-{seed}';holder=[]
                def factory(teacher,initial,target,info):
                    skill=StudentSkill(teacher,initial,target,info,model,mode,dict(PROFILES[profile],seed=0 if seed is None else seed+1000000),observer,oracle,hybrid,.02 if profile=='action_delay20' else 0.,.02 if 'filtered' in profile else 0.,mix)
                    holder.append(skill);return skill
                try:
                    row=episode(cfg,design,path,seed,scale,prepared,compact=True,template=template,skill_factory=factory if model else None,
                                action_proxy={'hz':200.} if model is None else None)
                    if holder:
                        holder[0].save(path/'student.npz');row['estimated_contact_time_s']=holder[0].contact_time
                        row['contact_detection_delay_s']=None if holder[0].contact_time is None or row['first_contact'] is None else holder[0].contact_time-row['first_contact']['time_s']
                        row['policy_checkpoint']=str(checkpoint);write_json(path/'summary.json',row)
                except Exception as exc:row=save_failure(path,exc)
                rows.append(row);log.write(json.dumps(dict(mode=name,seed=seed,passed=row['passed'],failures=row['failure_modes']))+'\n')
            results[name]=stats(rows,.008)
    result=dict(passed=all('RUNNER_ERROR' not in r['failures'] for r in results.values()),scope='pipeline integrity, inspect per-flight successes',results=results)
    write_json(output/'summary.json',result);return result


def benchmark(checkpoint,output,hybrid=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    model=(SplinePolicy if hybrid else MLP).load(checkpoint)
    fn=(lambda:model.reference(1).sample(2.)) if hybrid else (lambda:model(np.zeros(len(model.xmean))))
    for _ in range(500):fn()
    times=[]
    for _ in range(5000):
        start=time.perf_counter_ns();fn();times.append((time.perf_counter_ns()-start)/1000)
    result=dict(passed=True,host=platform.platform(),processor=platform.processor(),checkpoint_bytes=Path(checkpoint).stat().st_size,
                process_max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                inference_us=dict(median=float(np.median(times)),p95=float(np.quantile(times,.95)),p99=float(np.quantile(times,.99))),
                parameters=sum(r.spline.c.size for r in model.references) if hybrid else sum(w.size+b.size for w,b in zip(model.W,model.b)),
                scope='host single-call NumPy inference, synthetic input; hybrid reference only excludes geometric controller; NOT M3C')
    write_json(output/'summary.json',result);return result


def pipeline_benchmark(checkpoint,source,output,hybrid=True):
    """Full implemented actor/estimator/proxy CPU cost on recorded input states."""
    from scipy.spatial.transform import Rotation
    from ..model import build_model,scenario,truth_estimate,TargetState,VehicleEstimate,unit
    from ..maneuver import ManeuverSkill
    output=Path(output);output.mkdir(parents=True,exist_ok=False);source=Path(source)
    c=json.loads((source/'flight/config.json').read_text());x=np.load(source/'flight/trajectory.npz')['trace']
    _,_,info,_=build_model(scenario(c),c)
    target=TargetState(np.array(c['target']['center_m']),unit(c['target']['normal']),np.zeros((3,3)),0)
    states=[VehicleEstimate(row[1:4],row[8:11],Rotation.from_quat(row[[5,6,7,4]]).as_matrix(),row[11:14],row[0]) for row in x]
    teacher=ManeuverSkill(c,designs()['balanced'],states[0],target,info,states[0].position)
    model=(SplinePolicy if hybrid else MLP).load(checkpoint)
    actor=StudentSkill(teacher,states[0],target,info,model,1,PROFILES['noise_filtered'],hybrid=hybrid,detector_window_s=.02)
    durations=[]
    for s in states:
        start=time.perf_counter_ns();actor.command(s,False);durations.append((time.perf_counter_ns()-start)/1000)
    r=dict(passed=True,host=platform.platform(),samples=len(durations),control_period_us=c['control_dt_s']*1e6,
        latency_us=dict(median=float(np.median(durations)),p95=float(np.quantile(durations,.95)),p99=float(np.quantile(durations,.99)),maximum=float(max(durations))),
        process_max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        scope='entire implemented synthetic-estimator/observer/student/model feedback/rate proxy/log append; recorded state inputs, no physics step, not M3C or hard realtime')
    write_json(output/'summary.json',r);return r
