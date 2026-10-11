"""Whole-maneuver teacher: design/validation/holdout, Pareto data and demos."""
import copy
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from .arena import compose
from .evidence import snapshot,write_json,save_failure
from .maneuver import episode
from .model import ROOT
from .robust import configured
from .teacher_data import features,mechanism_metrics,export
from .recoverability import RecoverabilityModel


def settings():return json.loads((ROOT/'configs/contact_teacher.json').read_text())


def prepare(cfg,output,*,source_path=None,asset_root_map=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    c,bundle,slot,_=compose(configured(cfg,'adaptive_nominal'),'full',output,
                          source_path=source_path,asset_root_map=asset_root_map)
    path=output/'model.mjb';mujoco.mj_saveModel(bundle[0],str(path))
    return (c,bundle,slot),(path,hashlib.sha256(path.read_bytes()).hexdigest())


def bank(study):
    pool=[]
    for source in study['source_designs']:
        base=json.loads((ROOT/source).read_text())
        for scale in study['time_scales']:
            p=copy.deepcopy(base)
            p['segment_times_s']=[v*scale for v in p['segment_times_s']]
            p['alignment_lead_s']*=scale
            pool.append(p)
    # Two departure-to-impact designs that explicitly change the braking thrust.
    for alpha in (9.,14.):
        p=copy.deepcopy(pool[9]);p['terminal']['terminal_specific_thrust_m_s2']=alpha
        pool.append(p)
    return pool


def qualify(row,margin):
    return bool(row['passed'] and row.get('first_contact') and row['first_contact']['region_margin_m']>=margin)


def stats(rows,margin):
    from .campaign import aggregate
    r=aggregate(rows)
    r['qualified_successes']=sum(qualify(x,margin) for x in rows)
    for key in ('time_to_confirmed_recovery_s','attitude_ready_lead_s','braking_lead_s','velocity_thrust_angle_deg',
                'collective_at_contact_N','effort_proxy_N32_s','max_preimpact_rate_rad_s'):
        values=[x[key] for x in rows if x.get(key) is not None]
        r[key]=None if not values else dict(median=float(np.median(values)),min=float(min(values)),max=float(max(values)))
    margins=[x['first_contact']['region_margin_m'] for x in rows if x.get('first_contact')]
    r['contact_margin_m']=None if not margins else dict(min=float(min(margins)),p05=float(np.quantile(margins,.05)),median=float(np.median(margins)))
    return r


def rank(rows,margin):
    times=[r.get('time_to_confirmed_recovery_s') or 30 for r in rows]
    return float(100*sum(not qualify(r,margin) for r in rows)+np.mean(times)+.5*max(times))


def run(cfg,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    snapshot(output,cfg)
    study=settings();write_json(output/'study.json',study)
    pool=bank(study);write_json(output/'candidates.json',pool)
    prepared,template=prepare(cfg,output/'plant')
    model=RecoverabilityModel.load(ROOT/study['surrogate_model'])
    model_sha=hashlib.sha256((ROOT/study['surrogate_model']).read_bytes()).hexdigest()
    margin=study['qualification_margin_m']
    errors=[];index=[];times={'prefix':0.,'full_design':0.}
    def execute(p,seed,scale,path,prefix=False,proxy=None):
        begin=time.perf_counter()
        try:
            row=episode(cfg,p,path,seed,scale,prepared,compact=True,template=template,
                        prefix_s=study['prefix_after_contact_s'] if prefix else None,action_proxy=proxy)
            row.update(mechanism_metrics(path))
            row['qualified_passed']=qualify(row,margin)
            row['wall_time_s']=time.perf_counter()-begin
            write_json(Path(path)/'summary.json',row)
            return row
        except Exception as exc:
            errors.append(str(path));return save_failure(path,exc)
    with (output/'progress.jsonl').open('w',buffering=1) as log:
        train,predicted=[],[]
        for i,p in enumerate(pool):
            costs=[];queries=[]
            for seed,scale in study['design_cases']:
                path=output/'prefix'/f'candidate-{i:02d}'/f'seed-{seed}'
                row=execute(p,seed,scale,path,True)
                times['prefix']+=row.get('wall_time_s',0.)
                f=features(path) if 'RUNNER_ERROR' not in row['failure_modes'] else None
                pred=None if f is None else model.predict(f)
                # Prefix failure does not imply recovery failure: future checks are incomplete.
                known=[r for r in row['failure_modes'] if r not in ('RECOVERY_TIMEOUT','NO_SEPARATION')]
                bad=bool(known or not row.get('valid_contact') or not row.get('first_contact')
                         or row['first_contact']['region_margin_m']<margin)
                cost=1e4 if pred is None else (100*bad+row['first_contact']['time_s']+.12+pred['remaining_time_s']
                       +100*(pred['recovery_score']<.8)+5*(not pred['supported']))
                costs.append(cost);queries.append(dict(seed=seed,prediction=pred,cost=cost))
            predicted.append(float(np.mean(costs)+.5*max(costs)))
            write_json(output/'prefix'/f'candidate-{i:02d}'/'queries.json',queries)
            log.write(json.dumps(dict(stage='prefix',candidate=i,score=predicted[-1]))+'\n')
        # Freeze surrogate ranking BEFORE oracle full-rollout comparison.
        predicted_top=np.argsort(predicted)[:6].tolist()
        write_json(output/'surrogate_selection.json',dict(scores=predicted,top6=predicted_top,model_sha256=model_sha))
        for i,p in enumerate(pool):
            rows=[]
            for seed,scale in study['design_cases']:
                path=output/'design'/f'candidate-{i:02d}'/f'seed-{seed}'
                row=execute(p,seed,scale,path);rows.append(row)
                times['full_design']+=row.get('wall_time_s',0.)
                index.append((path,'design',f'candidate-{i:02d}'))
            train.append(rows)
            log.write(json.dumps(dict(stage='design',candidate=i,passed=sum(r['passed'] for r in rows),
                                      qualified=sum(qualify(r,margin) for r in rows)))+'\n')
        train_scores=[rank(rows,margin) for rows in train]
        bins={'aggressive':[],'balanced':[],'conservative':[]}
        for i,p in enumerate(pool):
            t=sum(p['segment_times_s'])
            name='aggressive' if t<=study['time_bins_s'][0] else 'balanced' if t<=study['time_bins_s'][1] else 'conservative'
            bins[name].append(i)
        shortlist={k:sorted(v,key=lambda i:train_scores[i])[:study['validation_candidates_per_bin']] for k,v in bins.items()}
        write_json(output/'validation_selection.json',dict(shortlist=shortlist,train_scores=train_scores))
        validation={};points={};selected={}
        for name,ids in shortlist.items():
            for i in ids:
                rows=[]
                for seed,scale in study['validation_cases']:
                    path=output/'validation'/f'candidate-{i:02d}'/f'seed-{seed}'
                    row=execute(pool[i],seed,scale,path);rows.append(row)
                    index.append((path,'validation',f'candidate-{i:02d}'))
                validation[i]=rows
            chosen=min(ids,key=lambda i:rank(validation[i],margin))
            selected[name]=chosen;points[name]=pool[chosen]
        methods=dict(adaptive_nominal=dict(method='staged',terminal=cfg['nominal_skill']),robust_v2=dict(method='staged'),**points)
        write_json(output/'selection.json',dict(indices=selected,methods=methods,validation={str(i):stats(rows,margin) for i,rows in validation.items()}))
        for name,p in points.items():write_json(output/f'{name}.json',p)
        evaluations={}
        for split,scale,seed0 in [('small',1.,study['holdout_small_start']),('stress',2.,study['holdout_stress_start'])]:
            evaluations[split]={}
            for name,p in methods.items():
                rows=[]
                for seed in range(seed0,seed0+study['holdout_trials']):
                    path=output/'holdout'/split/name/f'seed-{seed}'
                    row=execute(p,seed,scale,path);rows.append(row)
                    index.append((path,'holdout',name))
                    log.write(json.dumps(dict(stage='holdout',split=split,method=name,seed=seed,
                                              passed=row['passed'],qualified=qualify(row,margin),failures=row['failure_modes']))+'\n')
                evaluations[split][name]=stats(rows,margin)
        interfaces={}
        for profile in study['interfaces']:
            label=profile['name'];proxy={k:v for k,v in profile.items() if k!='name'}
            interfaces[label]={}
            for name,p in points.items():
                rows=[]
                for seed,scale in study['validation_cases']:
                    path=output/'interfaces'/label/name/f'seed-{seed}'
                    row=execute(p,seed,scale,path,proxy=proxy);rows.append(row)
                interfaces[label][name]=stats(rows,margin)
        # Oracle audit reports what the prefix screen would have missed, no holdout retuning.
        oracle_best=int(np.argmin(train_scores));short_best=min(predicted_top,key=lambda i:train_scores[i])
        avoided_future=len(pool)-len(predicted_top)
        index_rows=[]
        for path,split,name in index:
            destination=output/'dataset'/split/name/path.parent.name/path.name
            index_rows.append(export(path,destination,split,name))
        write_json(output/'dataset/index.json',index_rows)
        result=dict(passed=not errors,passed_scope='teacher pipeline integrity; inspect qualified flight counts',
                    design_episodes=len(pool)*len(study['design_cases']),prefix_episodes=len(pool)*len(study['design_cases']),
                    holdout_episodes=10*study['holdout_trials'],indices=selected,evaluations=evaluations,interfaces=interfaces,
                    surrogate=dict(oracle_best=oracle_best,best_in_surrogate_top6=short_best,
                                   oracle_score=train_scores[oracle_best],screen_score=train_scores[short_best],
                                   nominal_future_rollouts_avoided=avoided_future*len(study['design_cases']),
                                   prefix_wall_s=times['prefix'],full_design_wall_s=times['full_design'],
                                   estimated_screen_pipeline_wall_s=times['prefix']+sum(train[i][j]['wall_time_s'] for i in predicted_top for j in range(len(study['design_cases'])))),
                    datasets=len(index_rows),dataset_samples=sum(r['samples'] for r in index_rows),runner_errors=errors)
        write_json(output/'summary.json',result);return result


def interface_campaign(cfg,source,output):
    """Re-run frozen points with rate interface; no search or holdout fitting."""
    output=Path(output);output.mkdir(parents=True,exist_ok=False);snapshot(output,cfg)
    source=Path(source);selection=json.loads((source/'selection.json').read_text())
    study=json.loads((source/'study.json').read_text());write_json(output/'study.json',study)
    prepared,template=prepare(cfg,output/'plant');results={}
    for profile in study['interfaces']:
        label=profile['name'];proxy={k:v for k,v in profile.items() if k!='name'};results[label]={}
        for name in ('conservative','balanced','aggressive'):
            rows=[]
            for seed,scale in study['validation_cases']:
                path=output/label/name/f'seed-{seed}'
                try:
                    row=episode(cfg,selection['methods'][name],path,seed,scale,prepared,compact=True,template=template,action_proxy=proxy)
                    row.update(mechanism_metrics(path));write_json(path/'summary.json',row)
                except Exception as exc:row=save_failure(path,exc)
                rows.append(row)
            results[label][name]=stats(rows,study['qualification_margin_m'])
    result=dict(passed=all('RUNNER_ERROR' not in r['failures'] for methods in results.values() for r in methods.values()),
                scope='pipeline integrity; independent rate-P proxy, NOT ArduPilot firmware/SITL; all proxy saturation counted',profiles=results)
    write_json(output/'summary.json',result);return result
