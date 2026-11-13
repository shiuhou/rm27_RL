"""R6.1: bounded native Direct Shooting and continuous Fixed A handover."""
from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
import multiprocessing
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .asset import ROOT,digest,read_json,write_result
from . import terminal_native_eval as native
from .backward_frontier import common,Capture
from .contact_task import ContactScorer
from .r4_potential_report import finite_registered_gap

CONFIG=ROOT/'configs/rm27_asm2_r61.json'

def config():
    c=read_json(CONFIG);return c,ROOT/c['output']

def encode(commands,hover):
    x=np.array(commands,dtype=float,copy=True);x[:,0]=(x[:,0]/hover-1)/.5;x[:,1:]/=2
    return np.clip(x,-1,1)

def decode(parameters,steps,hover):
    knots=np.asarray(parameters).reshape(-1,4)
    if not np.isfinite(knots).all() or np.max(np.abs(knots))>1+1e-12:raise ValueError('Invalid bounded CTBR parameters')
    u=np.column_stack([np.interp(np.linspace(0,1,steps),np.linspace(0,1,len(knots)),knots[:,i]) for i in range(4)])
    u[:,0]=hover*(1+.5*u[:,0]);u[:,1:]*=2
    return u

def terminal_residual(state,motors,targets,goal,tol):
    target=np.asarray(goal['state'])
    q=lambda x:Rotation.from_quat(np.asarray(x)[[1,2,3,0]])
    attitude=(q(target[3:7]).inv()*q(state[3:7])).as_rotvec()
    return np.r_[(state[:3]-target[:3])/tol['position_m'],(state[7:10]-target[7:10])/tol['velocity_m_s'],
        attitude/tol['attitude_rad'],(state[10:13]-target[10:13])/tol['body_rate_rad_s'],
        (motors-goal['motor_thrust_N'])/tol['rotor_thrust_N'],(targets-goal['motor_target_N'])/tol['held_target_N']]

def match_report(state,motors,targets,goal,tol):
    r=terminal_residual(state,motors,targets,goal,tol)
    ratios=dict(position=float(np.linalg.norm(r[:3])),velocity=float(np.linalg.norm(r[3:6])),attitude=float(np.linalg.norm(r[6:9])),
                body_rate=float(np.linalg.norm(r[9:12])),rotor=float(np.max(abs(r[12:16]))),held_target=float(np.max(abs(r[16:20]))))
    return dict(passed=all(v<=1 for v in ratios.values()),tolerance_ratios=ratios,normalized_residual=r.tolist())

def freeze():
    c,out=config()
    if (out/'manifest.json').exists():raise FileExistsError('Manifest already frozen')
    out.mkdir(parents=True,exist_ok=True)
    m,_,profile,spec,sourcepath,source,goal,command,limits=common()
    r5=read_json(ROOT/'.agents/runs/asm2-terminal/r5/manifest.json')
    plant=native.NativeContact(profile,spec);families=[];candidates=[];pins=dict(m['pins'])
    aircraft=[plant.model.geom(n).id for n in spec.entries];targets=[plant.model.geom(n).id for n in sorted(spec.legal_targets)]
    for case,seedframe in zip(c['sources'],c['seed_frames'],strict=True):
        start=next(r for r in m['samples'] if r['case']==case and r['phase']==c['start_phase'] and r['variant']=='nominal')
        folder=ROOT/f'.agents/runs/asm2-terminal/r5/cases/{case}-{seedframe}-nominal'
        outcome=read_json(folder/'outcome.json')
        if not outcome['result']['qualified']:raise ValueError('Goal trajectory is not native qualified')
        a=dict(np.load(folder/'physics.npz',allow_pickle=False));n=len(a['states'])-1
        ids=[i for i in range(50,n-399,50) if np.max(np.abs(a['states'][i,10:13]))<=1.8]
        if not ids:raise ValueError(f'No action-compatible successful suffix for {case}')
        i=ids[0];target=dict(state=a['states'][i].tolist(),motor_thrust_N=a['motor_thrust_N'][i].tolist(),motor_target_N=a['motor_target_N'][i].tolist(),
            trajectory=str((folder/'physics.npz').relative_to(ROOT)),sample=i,elapsed_s=float(a['sample_times_s'][i]),future_qualified_contact_s=outcome['result']['end_s']-float(a['sample_times_s'][i]))
        trace=sourcepath.parent/source['cases'][case]['trace']['path'];w=dict(np.load(trace,allow_pickle=False))
        raw=np.r_[w['ctbr_N_rad_s'].reshape(-1,4)[start['frame']:seedframe],a['ctbr_N_rad_s'][:i//50]]
        duration=(seedframe-start['frame'])*.005+float(a['sample_times_s'][i]);steps=math.ceil(duration*c['duration_factor']/.005)
        warm=np.column_stack([np.interp(np.linspace(0,1,c['knots']),np.linspace(0,1,len(raw)),raw[:,j]) for j in range(4)])
        params=encode(warm,profile.hover_thrust_N)
        for state,f,t in [(start['state'],start['motor_thrust_N'],start['motor_target_N']),(target['state'],target['motor_thrust_N'],target['motor_target_N'])]:
            records=native.restore(plant,np.asarray(state),np.asarray(f),np.asarray(t))
            if any(r['distance_m']<=0 or r['normal_force_N']>0 for r in records) or native.flight_limits(np.asarray(state),spec,limits):raise ValueError('Invalid selected state')
            if any(np.any((np.asarray(v)<0)|(np.asarray(v)>profile.max_rotor_thrust_N)) for v in (f,t)):raise ValueError('Actuator state outside rails')
        family=dict(source=case,start=start,target=target,steps=steps,duration_s=steps*.005,warm_parameters=params.tolist(),
            warm_start='Clipped archived CTBR initialization only, stretched to fixed duration. No tracking cost, no actor imitation.',start_slope_velocity_m_s=float(np.asarray(start['state'][7:10])@spec.target_rotation[:,0]))
        families.append(family)
        for variant,(axis,sign) in c['variants'].items():
            state=np.asarray(start['state']).copy();state[:3]+=sign*c['position_offset_m']*spec.target_rotation[:,axis];state[7:10]+=sign*c['velocity_offset_m_s']*spec.target_rotation[:,axis]
            f=np.asarray(start['motor_thrust_N']);t=np.asarray(start['motor_target_N']);records=native.restore(plant,state,f,t)
            if any(r['distance_m']<=0 or r['normal_force_N']>0 for r in records) or native.flight_limits(state,spec,limits):raise ValueError('Invalid perturbed reset')
            gap,pair,_=finite_registered_gap(plant.model,plant.data,aircraft,targets)
            if gap<=0:raise ValueError('Initial finite-body clearance failed')
            candidates.append(dict(id=f'{case}-{variant}',source=case,variant=variant,state=state.tolist(),motor_thrust_N=f.tolist(),motor_target_N=t.tolist(),initial_finite_target_gap_m=gap))
        for p in (trace,folder/'physics.npz',folder/'outcome.json'):pins[str(p.relative_to(ROOT))]=digest(p)
    if len(candidates)!=20:raise ValueError('Expected twenty predeclared validations')
    for p in [CONFIG,ROOT/'src/rm27_asm2/informed_connection.py',ROOT/'tests/test_asm2_informed_connection.py',
              ROOT/'.agents/runs/asm2-terminal/r6/completion-verification.json',ROOT/'.agents/runs/asm2-terminal/r5-1/manifest.json']:
        pins[str(p.relative_to(ROOT))]=digest(p)
    write_result(out/'manifest.json',dict(phase='R6.1',config=c,config_sha256=digest(CONFIG),pins=pins,model=plant.metadata,controller=m['controllers']['fixed_a'],
        families=families,candidates=candidates,limits=limits,goal=goal.tolist(),
        optimization_budget='Five nominal Direct Shooting solves, <=384 complete prefix integrations and <=900 s per source. No Fixed A evaluation inside optimizer. These are optimization forward evaluations, separate from the 20 final candidate validations.',
        validation_budget=20,perturbation_scope='Four signed slope/across numerical sensitivity cases per family; common optimized prefix, no reoptimization. Offsets are not manufacturing uncertainty.',
        tolerance_scope='Position/velocity use existing numerical sensitivity magnitudes. Other full-state tolerances are preregistered engineering match tolerances, not measured uncertainty or a certified success neighborhood.',
        decisions='A requires safe full prefix, complete terminal match, then native-qualified Fixed A contact. B requires safe full prefix and complete match but Fixed A rejection. Otherwise C/no matched connection; off-target contacts reported separately.',
        transfer='Same MuJoCo instance, qpos/qvel/act/ctrl/time retained; no teleport or solver reset at handover. Fixed A generates every continuation action.',training_transitions=0))
    print([(f['source'],f['start']['frame'],f['target']['elapsed_s'],f['duration_s']) for f in families])

def verified():
    c,out=config();m=read_json(out/'manifest.json')
    if digest(CONFIG)!=m['config_sha256']:raise ValueError('Configuration changed')
    for p,h in m['pins'].items():
        if digest(ROOT/p)!=h:
            amendment=read_json(out/'refinement-amendment.json') if (out/'refinement-amendment.json').exists() else {}
            allowed=(p=='src/rm27_asm2/informed_connection.py' and amendment.get('original_implementation_sha256')==h
                     and digest(ROOT/p)==amendment.get('amended_implementation_sha256')
                     and digest(out/'initial-implementation.py')==h)
            if not allowed:raise ValueError(f'Frozen input changed: {p}')
    return c,out,m

def path_residual(states,plant,limits):
    x=np.asarray(states);local=(x[:,:3]-plant.spec.target_center)@plant.spec.target_rotation
    bounds=plant.spec.field_geometry['selection'];lo=np.asarray(bounds['safety_lower_target_frame_m']);hi=np.asarray(bounds['safety_upper_target_frame_m'])
    tilt_cos=1-2*np.sum(x[:,4:6]**2,axis=1)
    return np.r_[np.maximum(lo-local,0).ravel()/.02,np.maximum(local-hi,0).ravel()/.02,
        np.maximum(np.linalg.norm(x[:,7:10],axis=1)-limits['max_com_speed_m_s'],0)/.1,
        np.maximum(np.linalg.norm(x[:,10:13],axis=1)-limits['max_body_rate_rad_s'],0)/.2,
        np.maximum(np.cos(np.deg2rad(limits['max_tilt_deg']))-tilt_cos,0)/.02]

def prefix(plant,start,commands,limits,scorer=None,full_checks=False):
    initial=native.restore(plant,np.asarray(start['state']),np.asarray(start['motor_thrust_N']),np.asarray(start['motor_target_N']))
    if scorer:scorer.observe(initial,plant.state(),0)
    states=[plant.state()];times=[0.];used=[];saturated=0;signed=any(r['distance_m']<=0 for r in initial);unsafe=None
    for u in commands:
        if signed:break
        saturated+=int(plant.set_ctbr(u));used.append(u)
        for _ in range(50):
            records=plant.step_physics()
            if scorer:scorer.observe(records,plant.pre_state,plant.tick)
            signed=any(r['distance_m']<=0 for r in records)
            if full_checks:unsafe=unsafe or native.flight_limits(plant.state(),plant.spec,limits)
            if signed:break
        states.append(plant.state());times.append(plant.tick*plant.dt)
        unsafe=unsafe or native.flight_limits(states[-1],plant.spec,limits)
    return dict(states=np.asarray(states),times=np.asarray(times),commands=np.asarray(used),saturated=saturated,signed=signed,unsafe=unsafe,
        completed=plant.tick==len(commands)*50,motors=plant.motor_thrust_N,targets=plant.data.ctrl.copy())

class SolveLimit(Exception):pass

def optimize(source,variant=None):
    c,out,m=verified();family=dict(next(f for f in m['families'] if f['source']==source))
    if variant is not None:
        amendment=read_json(out/'refinement-amendment.json');c=dict(c,optimizer=dict(c['optimizer'],**amendment['refinement_budget']))
        family['start']=next(s for s in m['candidates'] if s['id']==variant)
        family['warm_parameters']=read_json(out/'optimization'/source/'result.json')['best']['parameters']
    folder=out/'optimization'/(variant or source);folder.mkdir(parents=True,exist_ok=True)
    if (folder/'result.json').exists():return read_json(folder/'result.json')
    write_result(folder/'started.json',dict(manifest_sha256=digest(out/'manifest.json'),source=source))
    from .flight import load_profile
    from .contact_task import load_contact_spec
    plant=native.NativeContact(load_profile(),load_contact_spec());clock=time.perf_counter();count=0;best=None;history=[]
    def objective(p):
        nonlocal count,best
        if count>=c['optimizer']['max_forward_evaluations_per_source'] or time.perf_counter()-clock>=c['optimizer']['max_wall_s_per_source']:raise SolveLimit()
        count+=1;u=decode(p,family['steps'],plant.profile.hover_thrust_N);r=prefix(plant,family['start'],u,m['limits'])
        x=r['states'];padded=np.r_[x,np.repeat(x[-1:],family['steps']+1-len(x),axis=0)]
        terminal=terminal_residual(x[-1],r['motors'],r['targets'],family['target'],c['terminal_tolerances'])
        residual=np.r_[terminal,path_residual(padded,plant,m['limits']),100*float(r['signed']),10*r['saturated'],
                       100*(1-plant.tick/(family['steps']*50)),.01*np.diff(np.asarray(p).reshape(-1,4),axis=0).ravel()]
        cost=float(residual@residual)
        entry=dict(evaluation=count,cost=cost,terminal_norm=float(np.linalg.norm(terminal)),completed=r['completed'],signed=r['signed'],unsafe=r['unsafe'],saturated=r['saturated'],elapsed_s=time.perf_counter()-clock)
        history.append(entry)
        if best is None or cost<best['cost']:
            best=dict(entry,parameters=np.asarray(p).tolist(),match=match_report(x[-1],r['motors'],r['targets'],family['target'],c['terminal_tolerances']))
            np.savez_compressed(folder/'best-prefix.npz',states=x,sample_times_s=r['times'],commands=u,terminal_motors=r['motors'],terminal_targets=r['targets'])
        return residual
    status='solver_complete';message=None
    try:
        result=least_squares(objective,np.asarray(family['warm_parameters']).ravel(),bounds=(-1.,1.),diff_step=c['optimizer']['diff_step'],
                             max_nfev=c['optimizer']['max_nfev'],ftol=1e-7,xtol=1e-7,gtol=1e-7)
        message=result.message
    except SolveLimit:status='bounded_forward_or_time_limit'
    if best is None:raise RuntimeError('No optimization evaluation completed')
    result=dict(source=source,variant=variant,status=status,message=message,forward_evaluations=count,wall_s=time.perf_counter()-clock,best=best,history=history,
                physics_step_s=plant.dt,control_step_s=.005,model=plant.metadata,manifest_sha256=digest(out/'manifest.json'))
    write_result(folder/'result.json',result);print(source,status,count,best['match'],flush=True);return result

def optimize_all():
    c,out,m=verified()
    with ProcessPoolExecutor(max_workers=c['optimizer']['workers'],mp_context=multiprocessing.get_context('fork')) as pool:
        results=list(pool.map(optimize,c['sources']))
    write_result(out/'optimization-results.json',dict(results=results,training_transitions=0))

def validate(candidate_id):
    c,out,m=verified();sample=next(r for r in m['candidates'] if r['id']==candidate_id);family=next(f for f in m['families'] if f['source']==sample['source'])
    folder=out/'cases'/candidate_id;folder.mkdir(parents=True,exist_ok=True)
    if (folder/'outcome.json').exists():return read_json(folder/'outcome.json')
    # A started-but-unfinished case consumes one slot; never silently rerun.
    write_result(folder/'started.json',dict(manifest_sha256=digest(out/'manifest.json'),id=candidate_id))
    _,_,profile,spec,_,_,goal,command,limits=common();plant=native.NativeContact(profile,spec)
    optpath=out/'optimization'/(candidate_id if (out/'refinement-amendment.json').exists() else sample['source'])/'result.json';opt=read_json(optpath);commands=decode(opt['best']['parameters'],family['steps'],profile.hover_thrust_N)
    scorer=ContactScorer(spec);states=[];times=[];controls=[]
    with Capture(plant,sample) as capture:
        pre=prefix(plant,sample,commands,limits,scorer,True);states=list(pre['states']);times=list(pre['times']);controls=list(pre['commands'])
        match=match_report(plant.state(),plant.motor_thrust_N,plant.data.ctrl,family['target'],c['terminal_tolerances']);switch_tick=plant.tick
        safe=pre['completed'] and not pre['signed'] and not pre['unsafe'] and not pre['saturated']
        transfer_state=plant.state().copy();transfer_motors=plant.motor_thrust_N;transfer_targets=plant.data.ctrl.copy()
        result=None;safety=None;suffix_saturations=0
        if safe:
            for _ in range(round(c['continuation_horizon_s']/.005)):
                u=command(plant.state());suffix_saturations+=int(plant.set_ctbr(u));controls.append(u)
                for _ in range(50):
                    records=plant.step_physics();result=scorer.observe(records,plant.pre_state,plant.tick)
                    if result is not None:break
                states.append(plant.state());times.append(plant.tick*plant.dt);safety=native.flight_limits(states[-1],spec,limits)
                if result is not None or safety:break
        contact=result or scorer.finish();qualified=bool(safe and contact['qualified'] and safety is None)
    # Finite detailed target distances on saved 10 ms prefix samples, separate from live solver.
    import mujoco
    query=mujoco.MjData(plant.model);aircraft=[plant.model.geom(n).id for n in spec.entries];targets=[plant.model.geom(n).id for n in sorted(spec.legal_targets)]
    gaps=[]
    for i in sorted(set([*range(0,switch_tick+1,100),switch_tick])):
        state=capture.states[i];query.qpos[:],query.qvel[:]=state[:7],state[7:];query.act[:]=capture.motors[i];query.ctrl[:]=capture.targets[i];mujoco.mj_forward(plant.model,query)
        gap,pair,queries=finite_registered_gap(plant.model,query,aircraft,targets);gaps.append(dict(tick=i,gap_m=gap,closest_pair=[plant.model.geom(j).name for j in pair]))
    finite_clearance=all(r['gap_m']>0 for r in gaps);safe=bool(safe and finite_clearance)
    category='A' if safe and match['passed'] and qualified else 'B' if safe and match['passed'] else 'C'
    outcome=dict(id=candidate_id,source=sample['source'],variant=sample['variant'],category=category,prefix_feasible=safe,
        prefix_completed=pre['completed'],prefix_signed_contact=pre['signed'],prefix_envelope_failure=pre['unsafe'],prefix_saturated_controls=pre['saturated'],
        match=match,switch_tick=switch_tick,switch_time_s=switch_tick*plant.dt,transfer_state=transfer_state.tolist(),transfer_motors=transfer_motors.tolist(),transfer_targets=transfer_targets.tolist(),
        fixed_a_run=bool(pre['completed'] and not pre['signed'] and not pre['unsafe'] and not pre['saturated']),fixed_a_qualified=qualified,
        qualified=qualified,reason=safety or contact['reason'],contact=contact,safety_failure=safety,suffix_saturated_controls=suffix_saturations,
        finite_target_clearance=gaps,finite_clearance_scope='Exact numerical hull distance every 10 ms and endpoint; full detailed native signed collision checks every 0.1 ms. No continuous-time certificate.',
        control_effort=dict(max_abs_rate=float(np.max(abs(commands[:,1:]))),collective_range_N=[float(commands[:,0].min()),float(commands[:,0].max())],
            fraction_controls_near_action_limit=float(np.mean(np.any(np.abs(encode(commands,profile.hover_thrust_N))>=.95,axis=1))),
            min_rotor_rail_margin_N=float(min(np.min(capture.motors),profile.max_rotor_thrust_N-np.max(capture.motors)))),
        optimization_sha256=digest(optpath),manifest_sha256=digest(out/'manifest.json'),training_transitions=0)
    with (folder/'physics.npz').open('xb') as f:np.savez_compressed(f,states=capture.states,motor_thrust_N=capture.motors,motor_target_N=capture.targets,
        sample_times_s=capture.times,ctbr_N_rad_s=capture.commands,control_times_s=capture.command_times,allocated_motor_target_N=capture.allocated,switch_tick=switch_tick,dt=plant.dt)
    write_result(folder/'contacts.json',capture.contacts)
    native.save_replay(folder/'replay.npz',candidate_id,(np.asarray(states),np.asarray(times),np.asarray(controls)),outcome,goal,native.target_mesh(plant,spec.legal_targets),.005)
    write_result(folder/'outcome.json',outcome);print(candidate_id,category,safe,match['passed'],qualified,flush=True);return outcome

def validate_all():
    c,out,m=verified()
    # Seal every optimized control before opening any validation outcome.
    seal=dict(manifest_sha256=digest(out/'manifest.json'),optimization={r['id']:digest(out/'optimization'/r['id']/'result.json') for r in m['candidates']},amendment_sha256=digest(out/'refinement-amendment.json'))
    if (out/'selection.json').exists():
        if read_json(out/'selection.json')!=seal:raise ValueError('Optimized prefixes changed')
    else:write_result(out/'selection.json',seal)
    with ProcessPoolExecutor(max_workers=5,mp_context=multiprocessing.get_context('fork')) as pool:
        results=list(pool.map(validate,[r['id'] for r in m['candidates']]))
    write_result(out/'results.json',dict(rows=results,validations=len(results),selection_sha256=digest(out/'selection.json'),training_transitions=0))

def refine_one(candidate_id):
    _,_,m=verified();source=next(r['source'] for r in m['candidates'] if r['id']==candidate_id)
    return optimize(source,candidate_id)

def refine_all():
    c,out,m=verified()
    with ProcessPoolExecutor(max_workers=5,mp_context=multiprocessing.get_context('fork')) as pool:
        results=list(pool.map(refine_one,[r['id'] for r in m['candidates']]))
    write_result(out/'refinement-results.json',dict(results=results,amendment_sha256=digest(out/'refinement-amendment.json'),training_transitions=0))

def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['freeze','optimize','refine','validate']);a=p.parse_args()
    {'freeze':freeze,'optimize':optimize_all,'refine':refine_all,'validate':validate_all}[a.command]()
if __name__=='__main__':main()
