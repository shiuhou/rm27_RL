"""R5: frozen Fixed A, <=20 new starts; no training or reference controller."""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import re

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from . import terminal_native_eval as native
from .contact_witness import target_mesh
from .rotation_bridge import robust_frontier
from .r4_potential_report import finite_registered_gap

OUT=ROOT/'.agents/runs/asm2-terminal/r5'
CASES=('dev_center','dev_fast','dev_near_slow','stationary_0','stationary_2')
FRONT=ROOT/'.agents/runs/asm2-frontier/phase-r'


def select_frames(boundary, excluded):
    """Nearest and fourth-nearest unused boundaries, at most 50 ms earlier."""
    eligible=[i for i in range(boundary-1,boundary-11,-1) if i not in excluded]
    if len(eligible)<4:
        raise ValueError('Insufficient unseen frames in the preregistered 50 ms window')
    return eligible[0],eligible[3]


def prior_inventory():
    """Conservatively exclude frames regardless of controller or perturbation.

    Examine saved JSON evidence and NPZ reset metadata; no outcome ranking.
    Raw trajectory samples are not counted as evaluated initial conditions.
    """
    excluded=defaultdict(set); provenance={}; scanned=0
    pattern=re.compile(r'('+'|'.join(CASES)+r'):(\d+)')

    def walk(value, context=None):
        if isinstance(value,dict):
            name=next((value[k] for k in ('case','case_id','source_case','name')
                       if isinstance(value.get(k),str) and value[k] in CASES),context)
            for k in ('frame','source_frame','reset_frame'):
                if name in CASES and isinstance(value.get(k),int):excluded[name].add(value[k])
            for k,v in value.items():
                walk(k,name);walk(v,k if k in CASES else name)
        elif isinstance(value,list):
            for v in value:walk(v,context)
        elif isinstance(value,str):
            for name,frame in pattern.findall(value):excluded[name].add(int(frame))

    for base in sorted((ROOT/'.agents/runs').glob('asm2-*')):
        for p in sorted(base.rglob('*')):
            if (OUT in p.parents or 'cache' in p.parts or 'wandb' in p.parts
                    or not p.is_file() or p.suffix not in ('.json','.npz')):
                continue
            scanned+=1
            before=sum(map(len,excluded.values()))
            if p.suffix=='.json':
                walk(read_json(p))
            else:
                with np.load(p,allow_pickle=False) as a:
                    if {'case_ids','source_frame'}<=set(a.files):
                        for name,frame in zip(a['case_ids'],a['source_frame']):
                            if str(name) in CASES:excluded[str(name)].add(int(frame))
            if sum(map(len,excluded.values()))>before:provenance[str(p.relative_to(ROOT))]=digest(p)
    return excluded,dict(files_scanned=scanned,contributing_files=provenance,
                         excluded_frames={c:sorted(excluded[c]) for c in CASES},
                         policy='All identified initial frames excluded regardless of policy or perturbation; raw continuous trace frames are not starts.')


def common():
    manifest=read_json(FRONT/'manifest.json');old=read_json(FRONT/'results.json')
    if old['manifest_sha256']!=digest(FRONT/'manifest.json'):raise ValueError('Phase R pairing changed')
    for p,h in manifest['pins'].items():
        if digest(ROOT/p)!=h:raise ValueError(f'Frozen source changed: {p}')
    task,profile,spec,path,source,goal=native.load_evaluation_inputs(ROOT/manifest['configuration']['experiment'])
    fixed=manifest['controllers']['fixed_a'];cp=ROOT/fixed['checkpoint']
    if digest(cp)!=fixed['checkpoint_sha256']:raise ValueError('Fixed A changed')
    command,provenance=native.load_controller(cp.with_suffix('.npz'),fixed['checkpoint_sha256'],profile,goal)
    if provenance['weights_sha256']!=fixed['weights_sha256']:raise ValueError('Fixed A portable weights changed')
    limits=read_json(ROOT/task['pins']['handover_limits']['path'])['flight_envelope']
    return manifest,old,profile,spec,path,source,goal,command,limits


def freeze():
    if (OUT/'manifest.json').exists():raise FileExistsError('R5 manifest already frozen')
    manifest,old,profile,spec,path,source,goal,command,limits=common()
    excluded,inventory=prior_inventory()
    frontier=robust_frontier(old,['nominal','minus','plus'],CASES)
    plant=native.NativeContact(profile,spec)
    if plant.metadata!=manifest['model']:raise ValueError('Native model changed')
    aircraft=[plant.model.geom(n).id for n in spec.entries]
    targets=[plant.model.geom(n).id for n in sorted(spec.legal_targets)]
    samples=[];pins=dict(manifest['pins']);pins.update(inventory['contributing_files'])
    for case in CASES:
        record=source['cases'][case];trace=path.parent/record['trace']['path']
        if digest(trace)!=record['trace']['sha256']:raise ValueError('Source trace changed')
        pins[str(trace.resolve().relative_to(ROOT))]=digest(trace)
        with np.load(trace,allow_pickle=False) as a:
            x=a['states'][:,0];f=a['motor_thrust_N'][:,0];u=a['motor_target_N'][:,0];times=a['sample_times_s']
        near,far=select_frames(frontier[case],excluded[case])
        for frame,variant,sign in [(near,'nominal',0),(far,'nominal',0),(far,'minus',-1),(far,'plus',1)]:
            state=x[frame].copy();state[:3]+=sign*.0005*spec.target_rotation[:,0]
            state[7:10]+=sign*.005*spec.target_rotation[:,0]
            if frame<record['handover_frame'] or abs(times[frame]-frame*profile.policy_dt_s)>1e-10:
                raise ValueError('Not a post-handover control boundary')
            if any(np.any((v<0)|(v>profile.max_rotor_thrust_N)) for v in [f[frame],u[frame]]):
                raise ValueError('Invalid actuator state')
            rows=native.restore(plant,state,f[frame],u[frame])
            if any(r['distance_m']<=0 or r['normal_force_N']>0 for r in rows):
                raise ValueError('Start has signed or loaded contact')
            if native.flight_limits(state,spec,limits):raise ValueError('Start violates flight limits')
            gap,pair,queries=finite_registered_gap(plant.model,plant.data,aircraft,targets)
            if gap<=0:raise ValueError('Start lacks finite target clearance')
            braking_frames=[s['frame'] for s in manifest['samples'] if s['case']==case and s['source_phase']=='braking']
            samples.append(dict(id=f'{case}-{frame}-{variant}',case=case,frame=frame,variant=variant,
                original_robust_frame=frontier[case],backward_s=(frontier[case]-frame)*profile.policy_dt_s,
                source_time_s=float(times[frame]),source_contact_lead_s=record['first_contact_sample_s']-float(times[frame]),
                source_first_sampled_braking_frame=min(braking_frames),
                source_braking_lead_s=(min(braking_frames)-frame)*profile.policy_dt_s,
                state=state.tolist(),motor_thrust_N=f[frame].tolist(),motor_target_N=u[frame].tolist(),
                finite_registered_gap_m=gap,closest_pair=[plant.model.geom(g).name for g in pair],
                clearance_queries=queries,initial_native_signed_or_loaded_contacts=0))
    paths=[FRONT/'manifest.json',FRONT/'results.json',ROOT/'src/rm27_asm2/backward_frontier.py',
        ROOT/'src/rm27_asm2/r4_potential_report.py',ROOT/'tests/test_asm2_backward_frontier.py',
        ROOT/'.agents/runs/asm2-terminal/r4/verification.json',ROOT/'.agents/docs/ASM2_R4_BRAKING_POTENTIAL.md']
    for mode in ['fixed_a','phase4']:
        paths.append(ROOT/manifest['controllers'][mode]['checkpoint'])
    paths.append((ROOT/manifest['controllers']['fixed_a']['checkpoint']).with_suffix('.npz'))
    pins.update({str(p.relative_to(ROOT)):digest(p) for p in paths})
    write_result(OUT/'manifest.json',dict(schema='asm2_r5_backward_frontier_v1',date='2026-11-13',
        maximum_rollouts=20,horizon_s=2.3,control_dt_s=profile.policy_dt_s,physics_dt_s=spec.dt,
        selection_rule='Nearest and fourth-nearest unseen frames within 50 ms before each Phase R earliest 3/3 anchor. Nominal both, +/- at earlier frame. No adaptive replacements.',
        perturbation=dict(target_axis=0,position_m=.0005,velocity_m_s=.005,meaning='Historical numerical sensitivity offsets, not manufacturing uncertainty.'),
        model=plant.metadata,controller=manifest['controllers']['fixed_a'],goal=goal.tolist(),
        limits=limits,inventory=inventory,pins=pins,samples=samples,training_transitions=0,
        interpretation='Development-only sampled starts; no general reachability or continuous-frontier certificate.'))
    print(json.dumps(dict(frontiers=frontier,samples=[(s['id'],s['backward_s']) for s in samples],
                         archive_files_scanned=inventory['files_scanned']),indent=2))


class Capture:
    """Observe the existing native rollout without changing its stepping/scoring."""
    def __init__(self,plant,sample):
        self.plant=plant
        self.states=[np.array(sample['state'])];self.motors=[np.array(sample['motor_thrust_N'])]
        self.targets=[np.array(sample['motor_target_N'])];self.times=[0.]
        self.contacts=[];self.commands=[];self.command_times=[];self.allocated=[]

    def __enter__(self):
        self.step=self.plant.step_physics;self.control=self.plant.set_ctbr
        def step():
            rows=self.step()
            self.states.append(self.plant.state().copy());self.motors.append(self.plant.motor_thrust_N.copy())
            self.targets.append(self.plant.data.ctrl.copy());self.times.append(self.plant.tick*self.plant.dt)
            if rows:self.contacts.append(dict(tick=self.plant.tick,records=rows))
            return rows
        def control(cmd):
            saturated=self.control(cmd)
            self.commands.append(np.array(cmd).copy());self.allocated.append(self.plant.data.ctrl.copy())
            self.command_times.append(self.plant.tick*self.plant.dt)
            return saturated
        self.plant.step_physics=step;self.plant.set_ctbr=control
        return self

    def __exit__(self,*args):
        self.plant.step_physics=self.step;self.plant.set_ctbr=self.control


def run():
    frozen=read_json(OUT/'manifest.json')
    for p,h in frozen['pins'].items():
        if digest(ROOT/p)!=h:raise ValueError(f'Frozen R5 input changed: {p}')
    manifest,old,profile,spec,path,source,goal,command,limits=common()
    if len(frozen['samples'])>20:raise ValueError('Budget exceeded')
    plant=native.NativeContact(profile,spec)
    if plant.metadata!=frozen['model']:raise ValueError('Native model changed')
    first=frozen['samples'][0]
    native.restore(plant,*[np.array(first[k]) for k in ['state','motor_thrust_N','motor_target_N']])
    mesh=target_mesh(plant,spec.legal_targets);outputs=[]
    for sample in frozen['samples']:
        name=sample['id'];destination=OUT/'cases'/name;destination.mkdir(parents=True,exist_ok=True)
        result_path=destination/'outcome.json'
        if result_path.exists():
            outputs.append(read_json(result_path));continue
        # An interrupted case is never silently rerun: its attempt consumes
        # one of the 20 permitted continuations, even without a final outcome.
        write_result(destination/'started.json',dict(id=name,manifest_sha256=digest(OUT/'manifest.json')))
        with Capture(plant,sample) as capture:
            result,trace=native.rollout(plant,*[np.array(sample[k]) for k in ['state','motor_thrust_N','motor_target_N']],
                                        command,frozen['horizon_s'],limits)
        with (destination/'physics.npz').open('xb') as stream:
            np.savez_compressed(stream,states=np.array(capture.states),motor_thrust_N=np.array(capture.motors),
                motor_target_N=np.array(capture.targets),sample_times_s=np.array(capture.times),
                ctbr_N_rad_s=np.array(capture.commands),control_times_s=np.array(capture.command_times),
                allocated_motor_target_N=np.array(capture.allocated),dt=spec.dt)
        native.save_replay(destination/'replay.npz',name,trace,result,goal,mesh,profile.policy_dt_s)
        write_result(destination/'contacts.json',capture.contacts)
        final=dict(id=name,case=sample['case'],frame=sample['frame'],variant=sample['variant'],
            result=result,physics_steps=plant.tick,manifest_sha256=digest(OUT/'manifest.json'),
            files={p:digest(destination/p) for p in ['physics.npz','replay.npz','contacts.json']})
        write_result(result_path,final);outputs.append(final)
        print(name,result['qualified'],result['reason'],result['end_s'],flush=True)
    if not (OUT/'results.json').exists():
        write_result(OUT/'results.json',dict(manifest_sha256=digest(OUT/'manifest.json'),
            completed_rollouts=len(outputs),started_rollouts=len(list((OUT/'cases').glob('*/started.json'))),
            training_transitions=0,rows=outputs))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('command',choices=['freeze','run']);a=p.parse_args()
    {'freeze':freeze,'run':run}[a.command]()
