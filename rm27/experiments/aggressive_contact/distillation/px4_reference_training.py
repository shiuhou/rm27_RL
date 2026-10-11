"""Fit a conservative hybrid reference from fresh PX4 Teacher train episodes.

Episode splits and physical outcomes are retained. Validation never contributes
to the fit. The fitted spline retains the existing geometric feedback and causal
observer; it is not a direct action network or an accepted flight controller.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.interpolate import BSpline

from ..evidence import write_json


def eligible_cases(manifest):
    if manifest.get('autopilot')!='px4':raise ValueError('PX4 Teacher dataset required')
    cases=manifest['cases']
    if any(c.get('split') not in ('train','validation') for c in cases):
        raise ValueError('Only train/validation episodes are allowed; reserved holdout data is forbidden')
    if any(c.get('policy')!='teacher' or c.get('mode')!='conservative' for c in cases):
        raise ValueError('This fitted reference supports only conservative Teacher data')
    if len({c['name'] for c in cases})!=len(cases):raise ValueError('Duplicate episode identity')
    return cases


def fit(source, output, *, intervals=64):
    source=Path(source).resolve();output=Path(output).resolve()
    manifest=json.loads((source/'manifest.json').read_text())
    cases=eligible_cases(manifest)
    groups={key:[] for key in ('train','validation')};records=[];first=None;duration=None
    for case in cases:
        root=source/case['name'];summary=json.loads((root/'summary.json').read_text())
        accepted=summary.get('passed') is True
        record=dict(name=case['name'],split=case['split'],accepted_for_fitting=accepted,
            flight_summary_sha256=hashlib.sha256((root/'summary.json').read_bytes()).hexdigest(),
            task_failure=summary.get('failure') or summary.get('contact_evaluation',{}).get('failure_modes'))
        records.append(record)
        if not accepted:continue
        data=np.load(root/'teacher_data.npz')
        observation=data['observation'];labels=data['reference_pva']
        length=float(observation[0,1]*6)
        if duration is not None and abs(length-duration)>1e-8:raise ValueError('Mixed reference durations')
        duration=length
        # Teacher evaluates its trajectory on the observation clock. Its
        # recorded feature phase is on the later control clock; remove the
        # measured sample age instead of fitting jitter as reference motion.
        times=observation[:,0]*duration-(data['control_time_s']-data['source_time_s'])
        mask=(times>=0)&(times<duration)&~data['causal_contact_detected']
        if not np.isfinite(np.r_[times,labels.ravel()]).all():raise ValueError('Nonfinite training input')
        groups[case['split']].append((times[mask],labels[mask]))
        record['teacher_data_sha256']=hashlib.sha256((root/'teacher_data.npz').read_bytes()).hexdigest()
        if first is None:first=root
    if len(groups['train'])<2 or not groups['validation']:
        raise ValueError('At least two physically successful train episodes and one validation episode required')
    output.mkdir(parents=True,exist_ok=False)
    t=np.concatenate([r[0] for r in groups['train']]);y=np.concatenate([r[1] for r in groups['train']])
    knots=np.r_[[0.]*6,np.linspace(0,duration,intervals+1)[1:-1],[duration]*6]
    basis=BSpline(knots,np.eye(len(knots)-6),5)
    matrix=np.vstack([20*basis(t),basis.derivative(1)(t),.15*basis.derivative(2)(t)])
    labels=np.vstack([20*y[:,:3],y[:,3:6],.15*y[:,6:9]])
    regularizer=basis.derivative(3)(np.linspace(0,duration,250))*1e-6
    coefficients=np.linalg.lstsq(np.vstack([matrix,regularizer]),
        np.vstack([labels,np.zeros((len(regularizer),3))]),rcond=1e-10)[0]
    parameters=json.loads((first/'evaluation/flight/parameters.json').read_text())
    bundle=output/'bundle';bundle.mkdir()
    np.savez_compressed(bundle/'policy.npz',**{key+str(i):value for i in range(3)
        for key,value in [('knots',knots),('coeff',coefficients),('duration',duration)]})
    shutil.copyfile(first/'plant_config.json',bundle/'nominal_config.json')
    write_json(bundle/'nominal_vehicle.json',dict(vehicle_info=json.loads((first/'vehicle_info.json').read_text())))
    write_json(bundle/'task_parameters.json',{'conservative':parameters})
    write_json(bundle/'deployment_options.json',dict(action_hz=250,portable_observer=True,shared_adaptation=True,
        reference_time_source='observation'))
    write_json(bundle/'capabilities.json',dict(autopilot='px4',supported_modes=['conservative'],flight_qualified=False,
        method='fitted reference plus retained geometric feedback and causal observer',
        training_source=str(source),scope='180 g comparison research; no hardware deployment'))
    vt=np.concatenate([r[0] for r in groups['validation']]);vy=np.concatenate([r[1] for r in groups['validation']])
    prediction=np.column_stack([basis.derivative(i)(vt)@coefficients for i in range(3)])
    errors=np.sqrt(np.mean((prediction-vy)**2,axis=0))
    result=dict(passed=bool(np.isfinite(errors).all() and np.max(errors[:3])<=.002),
        flight_qualified=False,scope='train-only hybrid reference fitting with independent validation; no closed-loop acceptance',
        train_episodes=len(groups['train']),validation_episodes=len(groups['validation']),total_attempts=len(cases),
        retained_failed_episodes=sum(not r['accepted_for_fitting'] for r in records),
        reference_pva_validation_rmse=errors.tolist(),duration_s=duration,bundle=str(bundle),sources=records,
        manifest_sha256=hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest())
    write_json(output/'summary.json',result);write_json(output/'manifest.json',manifest)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--intervals',type=int,default=64)
    a=p.parse_args();return 0 if fit(a.source,a.output,intervals=a.intervals)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
