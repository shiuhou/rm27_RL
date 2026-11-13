"""R5.1: ten prescribed across-target variants, using the frozen R5 recorder."""
from __future__ import annotations
import argparse
import copy
import json
import numpy as np
from .asset import ROOT,read_json,digest,write_result
from . import backward_frontier as frontier

OUT=ROOT/'.agents/runs/asm2-terminal/r5-1'
PARENT=ROOT/'.agents/runs/asm2-terminal/r5'
ANCHORS=dict(dev_center=725,dev_fast=816,dev_near_slow=702,stationary_0=330,stationary_2=346)


def perturb(state,rotation,sign):
    if sign not in (-1,1):raise ValueError('Only the two prescribed signs are allowed')
    np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12,rtol=0)
    assert np.linalg.det(rotation)>0
    original=np.asarray(state);value=original.copy()
    value[:3]+=sign*.0005*rotation[:,1]
    value[7:10]+=sign*.005*rotation[:,1]
    dp=(value[:3]-original[:3])@rotation
    dv=(value[7:10]-original[7:10])@rotation
    np.testing.assert_allclose(dp,[0,sign*.0005,0],atol=1e-15,rtol=0)
    np.testing.assert_allclose(dv,[0,sign*.005,0],atol=1e-15,rtol=0)
    np.testing.assert_array_equal(value[3:7],original[3:7])
    np.testing.assert_array_equal(value[10:],original[10:])
    return value,dict(position_delta_world_m=(value[:3]-original[:3]).tolist(),
        velocity_delta_world_m_s=(value[7:10]-original[7:10]).tolist(),
        position_delta_target_m=dp.tolist(),velocity_delta_target_m_s=dv.tolist(),
        unchanged_quaternion_body_rates=True)


def freeze():
    if (OUT/'manifest.json').exists():raise FileExistsError('Manifest already frozen')
    old=read_json(PARENT/'manifest.json');complete=read_json(PARENT/'completion-verification.json')
    pins=dict(old['pins'])
    pins.update({str((PARENT/p).relative_to(ROOT)):h for p,h in complete['artifacts'].items()})
    for p,h in pins.items():
        if digest(ROOT/p)!=h:raise ValueError(f'Parent evidence changed: {p}')
    _,_,profile,spec,source_path,source,goal,command,limits=frontier.common()
    plant=frontier.native.NativeContact(profile,spec)
    assert plant.metadata==old['model']
    aircraft=[plant.model.geom(n).id for n in spec.entries]
    targets=[plant.model.geom(n).id for n in sorted(spec.legal_targets)]
    samples=[];controls=[]
    for case,frame in ANCHORS.items():
        nominal=next(s for s in old['samples'] if s['case']==case and s['frame']==frame and s['variant']=='nominal')
        control=read_json(PARENT/'cases'/nominal['id']/'outcome.json')
        assert control['result']['qualified'] and source['cases'][case]['split']=='development'
        record=source['cases'][case];trace=source_path.parent/record['trace']['path']
        assert digest(trace)==record['trace']['sha256']
        with np.load(trace,allow_pickle=False) as a:
            np.testing.assert_array_equal(nominal['state'],a['states'][frame,0])
            np.testing.assert_array_equal(nominal['motor_thrust_N'],a['motor_thrust_N'][frame,0])
            np.testing.assert_array_equal(nominal['motor_target_N'],a['motor_target_N'][frame,0])
        controls.append(dict(id=nominal['id'],path=str((PARENT/'cases'/nominal['id']/'outcome.json').relative_to(ROOT)),
            sha256=digest(PARENT/'cases'/nominal['id']/'outcome.json'),result=control['result'],
            initial_finite_registered_gap_m=nominal['finite_registered_gap_m']))
        for sign,label in [(-1,'across_minus'),(1,'across_plus')]:
            state,check=perturb(nominal['state'],spec.target_rotation,sign)
            s=copy.deepcopy(nominal);s.update(id=f'{case}-{frame}-{label}',variant=label,
                state=state.tolist(),nominal_id=nominal['id'],transform_check=check)
            for key in ['motor_thrust_N','motor_target_N']:
                np.testing.assert_array_equal(s[key],nominal[key])
                assert np.all((np.array(s[key])>=0)&(np.array(s[key])<=profile.max_rotor_thrust_N))
            contacts=frontier.native.restore(plant,state,np.array(s['motor_thrust_N']),np.array(s['motor_target_N']))
            assert not any(r['distance_m']<=0 or r['normal_force_N']>0 for r in contacts)
            assert frontier.native.flight_limits(state,spec,limits) is None
            gap,pair,queries=frontier.finite_registered_gap(plant.model,plant.data,aircraft,targets)
            assert gap>0
            s.update(finite_registered_gap_m=gap,closest_pair=[plant.model.geom(g).name for g in pair],clearance_queries=queries)
            samples.append(s)
    assert len(samples)==10
    for p in [ROOT/'src/rm27_asm2/cross_track_validation.py',ROOT/'tests/test_asm2_cross_track.py',
              PARENT/'completion-verification.json',ROOT/'.agents/docs/ASM2_R5_BACKWARD_FRONTIER.md']:
        pins[str(p.relative_to(ROOT))]=digest(p)
    write_result(OUT/'manifest.json',dict(schema='asm2_r51_cross_track_v1',date='2026-11-13',
        exact_rollouts=10,maximum_rollouts=10,horizon_s=old['horizon_s'],control_dt_s=profile.policy_dt_s,
        physics_dt_s=spec.dt,model=plant.metadata,controller=old['controller'],goal=goal.tolist(),limits=limits,
        target_rotation=spec.target_rotation.tolist(),across_tangent_world=spec.target_rotation[:,1].tolist(),
        selection='Five user-prescribed nominal anchors; negative then positive across variant each; no replacements.',
        perturbation=dict(target_axis=1,position_m=.0005,velocity_m_s=.005,
            meaning='Coupled numerical position/velocity sensitivity, not manufacturing or estimator uncertainty.'),
        controls=controls,samples=samples,pins=pins,training_transitions=0,
        interpretation='Endpoint tests on one transverse position/velocity direction; no certified open region or general robustness.'))
    print(json.dumps(dict(cases=[s['id'] for s in samples],across=spec.target_rotation[:,1].tolist(),
        minimum_clearance_m=min(s['finite_registered_gap_m'] for s in samples),manifest_sha256=digest(OUT/'manifest.json')),indent=2))


def run():
    frozen=read_json(OUT/'manifest.json')
    assert frozen['exact_rollouts']==len(frozen['samples'])==10
    assert {(s['case'],s['frame'],s['variant']) for s in frozen['samples']}=={
        (c,f,v) for c,f in ANCHORS.items() for v in ['across_minus','across_plus']}
    # Reuse the exact tested R5 rollout/recorder without editing its source or
    # opening the R5 output directory for writing. Its exclusive attempt markers
    # forbid replacements after an interrupted attempt.
    previous=frontier.OUT
    try:
        frontier.OUT=OUT
        frontier.run()
    finally:
        frontier.OUT=previous


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('command',choices=['freeze','run']);a=p.parse_args()
    {'freeze':freeze,'run':run}[a.command]()
