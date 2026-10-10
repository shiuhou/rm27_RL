"""Whole-path flight-quality search and paired forward-return benchmark.

Runs the existing MuJoCo plant and adaptive controller. No perception changes,
no hardware, no learned claim: scenario search optimizes global spline geometry
and timing, and evaluates physical first contact plus held recovery and return.
"""
import argparse
import copy
import json
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc

from .evidence import snapshot, write_json, save_failure
from .flight_quality import marker_geometry, metrics, view_geometry
from .maneuver import ManeuverSkill, episode
from .model import ROOT, TargetState, frame, load_config, truth_estimate, unit
from .teacher import prepare
from .teacher_return import mission

_PREPARED = None
_TEMPLATE = None
_CONFIG = None


def reference_metrics(design, prepared, mount):
    cfg,bundle,slot=prepared
    initial=truth_estimate(bundle[0],bundle[1])
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0.)
    skill=ManeuverSkill(cfg,design,initial,target,bundle[2],slot)
    t=np.linspace(0,skill.p['duration_s'],240)
    states=np.array([skill.trajectory.sample(v) for v in t])
    pos,vel,acc,jerk=(states[:,i] for i in range(4))
    R=np.array([frame(v+[0,0,9.81]) for v in acc])
    rate=Rotation.from_matrix(np.einsum('nji,njk->nik',R[:-1],R[1:])).magnitude()/np.diff(t)
    target_pose = dict(center_m=target.position, normal=target.normal,
                       in_plane_rotation_deg=cfg['target'].get('in_plane_rotation_deg', 0.))
    marker = marker_geometry(pos, R, target_pose, mount)
    inside = marker['inside']
    far=np.linalg.norm(pos-target.position,axis=1)>=.5
    speed=np.linalg.norm(vel,axis=1)
    return dict(duration_s=float(t[-1]),speed_m_s=float(speed.max()),
                tilt_deg=float(np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1))).max()),
                rate_rad_s=float(rate.max()),height_m=float(pos[:,2].max()),
                jerk_m_s3=float(np.linalg.norm(jerk,axis=1).max()),
                jerk_rms_m_s3=float(np.sqrt(np.mean(np.sum(jerk**2,axis=1)))),
                view_outside_far_fraction=float(np.mean(~inside[far])),
                marker_vertical_rms_deg=float(np.sqrt(np.mean(marker['vertical'][far]**2))),
                marker_horizontal_rms_deg=float(np.sqrt(np.mean(marker['horizontal'][far]**2))),
                reverse_distance_m=float(np.trapezoid(np.maximum(vel[:,0],0),t)),
                interior_slow_fraction=float(np.mean(speed[(t>.8)&(t<t[-1]-.5)]<.4)))


def reference_cost(r, limits):
    penalty=sum(20*max(0.,r[k]/limit-1.)**2 for k,limit in limits.items())
    return (penalty+18*r['view_outside_far_fraction']
            +.018*r['marker_vertical_rms_deg']**2+.008*r['marker_horizontal_rms_deg']**2
            +5*r['reverse_distance_m']+10*r['interior_slow_fraction']
            +.25*r['duration_s']+.04*r['jerk_rms_m_s3'])


def score(rows, study):
    # Same cost on design and validation, frozen before holdout.
    values=[]
    for r in rows:
        if 'quality' not in r:
            values.append(1e6);continue
        q=r['quality'];margin=q['contact_margin_mm'] or 0.
        limits=study['reference_limits']
        envelope=sum(20*max(0.,q[k]/limits[v]-1.)**2 for k,v in (
            ('pre_speed_max_m_s','speed_m_s'),('pre_tilt_max_deg','tilt_deg'),
            ('pre_rate_max_rad_s','rate_rad_s'),('outbound_altitude_max_m','height_m'),
            ('pre_jerk_max_m_s3','jerk_m_s3')))
        values.append(1000*(not r['passed'])+100*max(0.,1-margin/(study['qualification_margin_m']*1000))
                      +envelope+18*q['view_outside_far_fraction']
                      +.018*(q.get('marker_vertical_rms_deg', 0.)**2)
                      +.008*(q.get('marker_horizontal_rms_deg', 0.)**2)
                      +.04*q['pre_jerk_rms_m_s3']
                      +.25*(q['maneuver_time_s'] or 30)+10*q['continuous_slow_time_s'])
    return float(np.mean(values)+.25*max(values))


def _execute(job):
    design,seed,scale,path,full=job
    path=Path(path)
    try:
        if full:
            r=mission(_CONFIG,design,path,seed,scale,prepared=_PREPARED,template=_TEMPLATE)
        else:
            r=episode(_CONFIG,design,path,seed,scale,prepared=_PREPARED,compact=True,template=_TEMPLATE,finish_on_recovery=True)
        q=metrics(path);write_json(path/'flight_quality.json',q)
        r['quality']=q;r['artifact']=str(path.resolve())
        return r
    except Exception as exc:
        r=save_failure(path,exc);write_json(path/'requested_trial.json',dict(design=design,seed=seed,scale=scale))
        r['artifact']=str(path.resolve());return r


def aggregate(rows):
    from collections import Counter
    good=[r for r in rows if 'quality' in r]
    fields=['contact_margin_mm','contact_time_s','maneuver_time_s','mission_time_s','pre_speed_max_m_s',
            'pre_tilt_max_deg','pre_rate_max_rad_s','pre_jerk_rms_m_s3','pre_jerk_max_m_s3',
            'outbound_altitude_max_m','return_backward_fraction','view_outside_far_fraction',
            'marker_vertical_rms_deg','marker_horizontal_rms_deg','marker_view_margin_deg',
            'terminal_view_loss_s','post_rate_max_rad_s','recovery_time_s','continuous_slow_time_s']
    out=dict(trials=len(rows),successes=sum(r['passed'] for r in rows),
             failure_modes=dict(Counter(f for r in rows for f in r['failure_modes'])))
    for key in fields:
        values=[r['quality'][key] for r in good if r['quality'].get(key) is not None]
        out[key]=None if not values else dict(min=float(min(values)),median=float(np.median(values)),max=float(max(values)))
    return out


def run(output, study_path, workers=4):
    global _PREPARED,_TEMPLATE,_CONFIG
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    study=json.loads(Path(study_path).read_text());_CONFIG=load_config()
    snapshot(output,_CONFIG);write_json(output/'study.json',study)
    _PREPARED,_TEMPLATE=prepare(_CONFIG,output/'plant')
    base=json.loads((ROOT/study['source_design']).read_text())
    mount=json.loads((ROOT/'configs/contact_fpv_camera.json').read_text())
    pool=[]
    # Interpretable whole-curve time stretching is a comparator, not the only search family.
    for scale in (1.15,1.3,1.5,1.7):
        p=copy.deepcopy(base);p['segment_times_s']=[t*scale for t in p['segment_times_s']]
        p['alignment_lead_s']*=scale;pool.append(p)
    bounds=study['bounds']
    for sample in qmc.LatinHypercube(len(bounds),seed=study['search_seed']).random(study['reference_candidates']):
        p=copy.deepcopy(base)
        values={k:float(lo+x*(hi-lo)) for (k,(lo,hi)),x in zip(bounds.items(),sample)}
        p['segment_times_s']=[values.pop(k) for k in ('departure_time_s','cruise_time_s','arrival_time_s')]
        p.update(values);pool.append(p)
    screened=[]
    for i,p in enumerate(pool):
        r=reference_metrics(p,_PREPARED,mount)
        screened.append(dict(index=i,design=p,metrics=r,cost=reference_cost(r,study['reference_limits'])))
    write_json(output/'reference_screen.json',screened)
    finalists=sorted(screened,key=lambda r:r['cost'])[:study['rollout_candidates']]
    errors=[]
    with (output/'progress.jsonl').open('w',buffering=1) as log, ProcessPoolExecutor(max_workers=workers,mp_context=mp.get_context('fork')) as executor:
        def trials(designs,cases,stage,full=False):
            jobs=[];names=[]
            for name,design in designs.items():
                for seed,scale in cases:
                    path=output/stage/name/f'seed-{seed}-scale-{scale}'
                    jobs.append((design,seed,scale,str(path),full));names.append(name)
            rows={name:[] for name in designs}
            for name,r in zip(names,executor.map(_execute,jobs)):
                rows[name].append(r)
                if 'RUNNER_ERROR' in r['failure_modes']:errors.append(r['artifact'])
                log.write(json.dumps(dict(stage=stage,method=name,artifact=r['artifact'],passed=r['passed'],failures=r['failure_modes']))+'\n')
            write_json(output/f'{stage}.json',rows)
            return rows
        designs={f'candidate-{r["index"]:03d}':r['design'] for r in finalists}
        train=trials(designs,study['design_cases'],'design')
        ranks=sorted(designs,key=lambda name:score(train[name],study))
        validation=trials({name:designs[name] for name in ranks[:study['validation_candidates']]},study['validation_cases'],'validation')
        winner=min(validation,key=lambda name:score(validation[name],study))
        selected=copy.deepcopy(designs[winner]);selected['forward_return']=study['forward_return']
        heading_only=copy.deepcopy(base);heading_only['forward_return']=study['forward_return']
        methods=dict(baseline=base,heading_only=heading_only,smooth=selected)
        write_json(output/'selection.json',dict(winner=winner,design_costs={k:score(v,study) for k,v in train.items()},
                   validation_costs={k:score(v,study) for k,v in validation.items()},methods=methods,
                   frozen_before_holdout=True))
        write_json(output/'selected_design.json',selected)
        cases=[(study['holdout_seed_start']+i+(1000 if scale==2 else 0),float(scale))
               for scale in (1,2) for i in range(study['holdout_trials_per_scale'])]
        evaluation=trials(methods,cases,'holdout',True)
        demos=trials(methods,[(None,1.)],'demo',True)
    counts={}
    for name,rows in evaluation.items():
        counts[name]={str(scale):aggregate([r for r in rows if r.get('uncertainty_scale')==scale]) for scale in (1.,2.)}
    result=dict(passed=not errors and demos['smooth'][0]['passed'],
                passed_scope='runner integrity and selected nominal mission; randomized outcomes are separate',
                runner_errors=errors,evaluations=counts,nominal={k:v[0]['quality'] for k,v in demos.items() if 'quality' in v[0]},
                winner=winner,selection='design and validation only; no holdout tuning',
                closed_loop_trials=len(designs)*len(study['design_cases'])+len(validation)*len(study['validation_cases'])+len(methods)*(len(cases)+1))
    write_json(output/'summary.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--study',type=Path,default=ROOT/'configs/contact_smooth_flight_study.json')
    parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args()
    if args.output.exists():parser.error('Use a new artifact directory')
    if not 1<=args.workers<=8:parser.error('workers must be 1..8')
    try:
        result=run(args.output,args.study,args.workers)
    except Exception as exc:
        save_failure(args.output,exc);raise
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':
    main()
