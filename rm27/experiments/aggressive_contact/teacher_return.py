"""Continue the same live whole-maneuver plant into the existing return route."""
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from .arena import Piecewise
from .control import Septic
from .heading import HeadingTurn, course_yaw
from .evidence import write_json
from .model import ROOT,truth_estimate
from .recording import save_overlay


def continue_return(cfg,bundle,controller,slot,output):
    model,data,info,_=bundle
    settings=json.loads((ROOT/'configs/contact_arena.json').read_text())
    goal=slot+[0,0,settings['return_height_above_slot_m']]
    center=np.array(cfg['target']['center_m']);z=settings['cruise_height_m']
    def state(p,v):return np.array(p),np.array(v),np.zeros(3),np.zeros(3)
    options=getattr(controller,'design',{}).get('forward_return')
    turn=None;turn_path=None;turn_duration=0.
    entry=state(data.qpos[:3].copy(),data.qvel[:3].copy())
    if options:
        # The travel axis is explicit in the flight configuration; no camera
        # rotation, pose reset, or velocity feedback sign trick is involved.
        initial=truth_estimate(model,data)
        yaw=float(np.arctan2(controller.last_Rd[1,0],controller.last_Rd[0,0]))
        final_yaw=course_yaw(goal-initial.position,options['forward_axis_body'])
        turn=HeadingTurn(yaw,final_yaw,options['turn_s'],options['max_yaw_rate_rad_s'],options['max_yaw_accel_rad_s2'])
        turn_duration=turn.duration
        turn_end=initial.position+[0,0,options['turn_lift_m']]
        turn_path=Septic(entry,state(turn_end,[0,0,0]),turn_duration)
        entry=state(turn_end,[0,0,0])
    path=Piecewise([entry,
                    state([center[0]+1.468,.5,z],[1.4,.1,0]),
                    state([slot[0]-1.15,slot[1],z],[1.2,0,-.1]),state(goal,[0,0,0])],
                   settings['return_segment_times_s'])
    start=float(data.time);dt=cfg['physics_dt_s'];stride=round(cfg['control_dt_s']/dt)
    # No plant reset, no actuator reset, no new observer. Resume free-flight adaptation.
    controller.contact_time=None
    rows=[];phases=[];failures=set();stable_time=0.;max_error=0.
    route_duration=turn_duration+path.times[-1]
    uid=model.body('uav').id
    for k in range(round((route_duration+settings['return_hold_s'])/dt)+1):
        elapsed=float(data.time)-start;s=truth_estimate(model,data)
        if k%stride==0:
            ref=turn_path.sample(elapsed) if turn is not None and elapsed<turn_duration else path.sample(elapsed-turn_duration)
            if turn is not None:
                yaw,yaw_rate,_,_=turn.sample(elapsed)
                controller.set_heading_reference(yaw,yaw_rate)
            data.ctrl[:],sat=controller.track(s,ref)
            error=float(np.linalg.norm(ref[0]-s.position));max_error=max(max_error,error)
            rows.append(np.r_[data.time,data.qpos.copy(),data.qvel.copy(),data.act.copy(),data.ctrl.copy(),
                              ref[0],data.ncon,sat,controller.last_reference_state])
            phases.append('RETURN_TURN' if elapsed<turn_duration else 'RETURN')
            if error>settings['maximum_route_tracking_error_m']:failures.add('ROUTE_TRACKING_ERROR')
        for contact in data.contact:
            if uid in model.geom_bodyid[contact.geom] and contact.dist<=0:failures.add('UNEXPECTED_ROUTE_CONTACT')
        if elapsed>=route_duration:
            stable=(np.linalg.norm(s.position-goal)<settings['return_position_tolerance_m']
                    and np.linalg.norm(s.velocity)<settings['return_speed_tolerance_m_s']
                    and s.rotation[2,2]>np.cos(np.deg2rad(10)) and np.linalg.norm(s.omega_body)<.5236)
            stable_time=stable_time+dt if stable else 0.
        if failures or elapsed>=route_duration+settings['return_hold_s']-1e-8:break
        mujoco.mj_step(model,data);mujoco.mj_forward(model,data)
    if stable_time<settings['return_hold_s']-2*dt:failures.add('RETURN_NOT_STABLE')
    if any(w.number for w in data.warning):failures.add('SIMULATOR_WARNING')
    prior=np.load(output/'flight/trajectory.npz')
    # Boundary sample stores the new return control, removing the duplicate timestamp.
    old=prior['trace'];keep=old[:,0]<start-1e-9
    np.savez_compressed(output/'trajectory.npz',trace=np.vstack([old[keep],rows]),
                        phases=np.r_[prior['phases'][keep],phases])
    flight=output/'flight'
    if (flight/'model.mjb').exists():
        template=flight/'model.mjb';sha=hashlib.sha256(template.read_bytes()).hexdigest()
    else:
        record=json.loads((flight/'model_template.json').read_text());template=flight/record['path'];sha=record['sha256']
    save_overlay(model,output,template,sha)
    result=dict(passed=not failures,failure_modes=sorted(failures),entry_time_s=start,
                completed_time_s=float(data.time),duration_s=float(data.time)-start,
                return_error_m=float(np.linalg.norm(data.qpos[:3]-goal)),return_speed_m_s=float(np.linalg.norm(data.qvel[:3])),
                max_tracking_error_m=max_error,stable_hold_s=stable_time,heading_turn_s=turn_duration,
                forward_return=options,
                continuity='same MjData and controller; no reset/teleport; position/velocity matched route entry')
    write_json(output/'return.json',result)
    return result


def mission(cfg,design,output,seed=None,scale=1.,prepared=None,template=None):
    from .maneuver import episode
    result=episode(cfg,design,output,seed,scale,prepared=prepared,compact=template is not None,template=template,
                   finish_on_recovery=True,after_recovery=continue_return)
    result['maneuver_passed']=result['passed']
    result['passed']=bool(result['passed'] and result.get('return',{}).get('passed'))
    result['failure_modes']=sorted(set(result['failure_modes']+result.get('return',{}).get('failure_modes',[])))
    result['scope']='HEADLESS truth motor mission: airborne dock departure/contact/held recovery/return-near-dock; no takeoff/landing'
    if 'return' in result:result['mission_duration_s']=result['return']['completed_time_s']
    write_json(Path(output)/'summary.json',result)
    return result


def campaign(cfg,source,output):
    from .teacher import prepare
    from .evidence import snapshot
    output=Path(output);output.mkdir(parents=True,exist_ok=False);snapshot(output,cfg)
    prepared,template=prepare(cfg,output/'plant')
    methods=json.loads((Path(source)/'selection.json').read_text())['methods']
    results={}
    for name,design in methods.items():
        results[name]=mission(cfg,design,output/name,prepared=prepared,template=template)
    result=dict(passed=all(r['passed'] for r in results.values()),methods=results)
    write_json(output/'summary.json',result);return result
