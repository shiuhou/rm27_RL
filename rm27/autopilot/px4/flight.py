"""PX4 micro-flight and audited-route runner with independent physical gates."""
import hashlib
import json
import math
from pathlib import Path
import time
import traceback
import mujoco
import numpy as np
from pymavlink.dialects.v20 import common as mavlink
from ...simulation.micro_plant import MicroPlant
from ..bridge.frames import to_ap
from .runtime import Runtime


class LandingMonitor:
    """Allow transient contact chatter, retaining the original slot/rest gates."""
    def __init__(self, contact_grace_s=.5):
        self.contact_grace_s=contact_grace_s
        self.last_contact=None

    def check(self, timestamp, local, contacts):
        if np.linalg.norm(local[:2])>.007 or abs(local[2]-.02)>.005:
            raise RuntimeError('Landing left assigned slot')
        if contacts:
            self.last_contact=timestamp
        elif self.last_contact is None or timestamp-self.last_contact>self.contact_grace_s:
            raise RuntimeError('Landing contact lost beyond grace window')


def confirm_auto_disarm(telemetry, timestamp, land_observed):
    """PX4 restores the prior user mode after automatic landing disarm."""
    ack=telemetry.get('command_ack',{}).get(mavlink.MAV_CMD_NAV_LAND,{})
    if not land_observed or ack.get('result')!=mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError('No accepted normal LAND sequence')
    if telemetry.get('armed') or telemetry.get('landed_state')!=mavlink.MAV_LANDED_STATE_ON_GROUND or timestamp-telemetry.get('landed_time',-100)>.5:
        raise RuntimeError('No fresh PX4 landed/disarmed confirmation')


def run(scene, output, instance=0, *, dynamics=None, view=False, speedup=None, route=None, profile='diagnostic'):
    speedup=(1. if view else 5.) if speedup is None else float(speedup)
    if not math.isfinite(speedup) or speedup<=0:raise ValueError('speedup must be positive')
    scene=Path(scene).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    plant=MicroPlant(scene,dynamics_profile=dynamics)
    if dynamics: (output/'dynamics.json').write_bytes(Path(dynamics).read_bytes())
    summary={'passed':False,'autopilot':'px4','firmware':'v1.17.0','run_type':'closed-loop SITL','headless':not view,'estimation_source':'synthetic ODOMETRY to EKF2','guidance':'diagnostic truth-assisted' if profile=='diagnostic' else 'EKF telemetry','gate_version':'MICRO_FLIGHT existing physical thresholds','events':[],'failure':None}
    metadata={'scene':str(scene),'scene_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'mass_kg':float(plant.model.body_mass[plant.body]),'origin_nwu':plant.origin.tolist(),'profile':profile,'dynamics_profile':str(dynamics),'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2))
    phase='INITIALIZE';phase_start=0.;stable_since=None;next_send=0.;next_action=10.;next_heartbeat=0.;next_params=0.
    hover=[];landing=None;land_observed=False;land_requested=False;land_monitor=LandingMonitor();max_tilt=max_force=max_penetration=0.;start=time.monotonic();viewer=None
    yaw=math.atan2(plant.data.xmat[plant.body].reshape(3,3)[1,0],plant.data.xmat[plant.body].reshape(3,3)[0,0])
    integral=np.zeros(2);last_send=None;rt=None
    def event(name,**fields):
        entry={'event':name,'physics_time':float(plant.data.time),**fields};summary['events'].append(entry);print(json.dumps(entry),flush=True)
    def transition(name):
        nonlocal phase,phase_start,stable_since
        phase=name;phase_start=float(plant.data.time);stable_since=None
        if name=='RETURN':integral[:]=0
        event(name)
    def target(p,xy_limit=.06,z_limit=0.,feedforward=None,kp=.8,kpz=None):
        nonlocal next_send,last_send
        t=float(plant.data.time)
        if t<next_send:return
        tel=rt.telemetry
        if t-tel.get('local_time',-100)>.5:raise RuntimeError('Stale LOCAL_POSITION_NED')
        measured=np.array(tel['local_position'])+np.array(tel['local_velocity'])*(t-tel['local_time'])
        if profile=='diagnostic':
            error=np.array(p)-(plant.qpos[:3]-plant.origin)
            goal=measured+to_ap(error)
            discrepancy=np.array(tel['local_velocity'])-to_ap(plant.qvel[:3])
            if np.linalg.norm(discrepancy)>.25:raise RuntimeError('Estimated/true velocity discrepancy exceeds .25 m/s')
        else:
            goal=to_ap(p);error=np.array(p)-to_ap(measured);discrepancy=np.zeros(3)
        dt=0. if last_send is None else t-last_send
        # PX4 already supplies position and velocity integrators. The legacy AP
        # outer-loop integral assumed GUID_OPTIONS disabled horizontal position.
        v=np.zeros(3) if feedforward is None else np.array(feedforward,copy=True)
        v[:2]+=kp*error[:2]+integral;v[2]+=(kp if kpz is None else kpz)*error[2]
        v[:2]=np.clip(v[:2],-xy_limit,xy_limit);v[2]=np.clip(v[2],-z_limit,z_limit)
        rt.position(goal,-yaw,discrepancy+to_ap(v));next_send=t+.02;last_send=t
    try:
        if view:
            from ...simulation.live_view import FlightView
            viewer=FlightView(plant)
        with Runtime(plant,output,instance,profile=profile) as rt, (output/'frames.jsonl').open('w') as frames, (output/'mavlink.jsonl').open('w') as messages:
            pacing=time.monotonic()
            while time.monotonic()-start<300:
                row=rt.step(); t=float(plant.data.time);tel=rt.telemetry
                for msg in rt.receive():
                    kind=msg.get_type();messages.write(json.dumps({'received_sim_s':t,'message':msg.to_dict()},default=str)+'\n')
                    if kind=='STATUSTEXT':event(kind,text=msg.text)
                    elif kind=='COMMAND_ACK':event(kind,command=msg.command,result=msg.result)
                if t>=next_heartbeat:rt.heartbeat();next_heartbeat=t+1
                if t>=next_params and phase=='INITIALIZE':
                    rt.mav.param_request_list_send(rt.system,1);next_params=t+5
                if viewer:
                    if not viewer.is_running():raise RuntimeError('Viewer closed')
                    viewer.update(phase,tel)
                delay=pacing+t/speedup-time.monotonic()
                if delay>0:time.sleep(min(delay,.02))
                local=plant.qpos[:3]-plant.origin;speed=float(np.linalg.norm(plant.qvel[:3]))
                tilt=math.acos(float(np.clip(plant.data.xmat[plant.body].reshape(3,3)[2,2],-1,1)))
                contacts=plant.contacts();max_tilt=max(max_tilt,math.degrees(tilt))
                row.update(phase=phase,telemetry=tel.copy(),contacts=contacts,all_qpos=plant.data.qpos.tolist(),all_qvel=plant.data.qvel.tolist(),all_act=plant.data.act.tolist(),all_motor_thrust_N=plant.data.actuator_force.tolist())
                frames.write(json.dumps(row,allow_nan=False)+'\n')
                if not np.isfinite(np.r_[plant.data.qpos,plant.data.qvel,plant.data.qacc]).all():raise RuntimeError('Nonfinite physics')
                if tilt>math.radians(route.maximum_tilt_deg if phase=='ROUTE' else 25) or speed>(route.maximum_speed_m_s if phase=='ROUTE' else 2) or (phase!='ROUTE' and (local[2]>1.1 or np.linalg.norm(local[:2])>.5)):
                    raise RuntimeError(f'Flight envelope exceeded: position={local}, speed={speed}, tilt={math.degrees(tilt)}')
                for qa,va,initial in plant.inactive:
                    if np.linalg.norm(plant.data.qpos[qa:qa+2]-initial[:2])>.001 or abs(plant.data.qpos[qa+2]-initial[2])>.002:raise RuntimeError('Inactive vehicle moved')
                    if t>1 and np.max(np.abs(plant.data.qvel[va:va+6]))>.001:raise RuntimeError('Inactive vehicle not resting')
                    q=plant.data.qpos[qa+3:qa+7]
                    if 1-2*(q[1]**2+q[2]**2)<math.cos(math.radians(1)):raise RuntimeError('Inactive vehicle tilted')
                inactive=np.setdiff1d(np.arange(plant.model.nu),plant.motors)
                if np.any(plant.data.ctrl[inactive]) or np.any(plant.data.actuator_force[inactive]):raise RuntimeError('Inactive motor received thrust')
                for c in contacts:
                    max_force=max(max_force,c['force_N']);max_penetration=max(max_penetration,-c['distance_m'])
                    if 'rm27_dock_collision_bottom' not in c['geoms'] and not (phase=='ROUTE' and route.contact_allowed(c)):raise RuntimeError('Unintended contact: '+str(c['geoms']))
                    if 'rm27_dock_collision_bottom' in c['geoms'] and (c['force_N']>10 or c['distance_m']<-.005):raise RuntimeError('Unstable landing contact')
                if phase in ('OFFSET','RETURN','ALIGN') and contacts:raise RuntimeError('Unexpected airborne contact')
                if phase!='INITIALIZE':
                    if t-tel.get('heartbeat_time',-100)>2:raise RuntimeError('Heartbeat stale')
                    allowed_mode=('OFFBOARD','LAND') if phase=='FINAL_DESCEND' and land_requested else ('OFFBOARD',)
                    if phase not in ('LAND','POST') and (not tel.get('armed') or tel.get('mode') not in allowed_mode):raise RuntimeError('Unexpected mode/disarm')
                if t>(route.physics_deadline if route else 160):raise RuntimeError('Physics deadline')
                if phase=='INITIALIZE':
                    if 'local_position' in tel: target([0,0,.02])
                    verified=all(k in rt.parameters and math.isclose(rt.parameters[k],v,abs_tol=1e-5,rel_tol=1e-5) for k,v in rt.expected.items())
                    if t>50 and not verified:raise RuntimeError('Parameter mismatch: '+str({k:(v,rt.parameters.get(k)) for k,v in rt.expected.items() if k not in rt.parameters or not math.isclose(rt.parameters[k],v,abs_tol=1e-5,rel_tol=1e-5)}))
                    if verified and t>=next_action and 'local_position' in tel:
                        next_action=t+2
                        if tel.get('mode')!='OFFBOARD':rt.mode('OFFBOARD')
                        elif not tel.get('armed'):rt.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
                    if verified and tel.get('armed') and tel.get('mode')=='OFFBOARD':
                        summary['parameters_verified']=True;transition('TAKEOFF')
                elif phase=='TAKEOFF':
                    target([0,0,min(.6,.02+.25*(t-phase_start))],z_limit=.25)
                    if abs(local[2]-.6)<.08 and speed<.08:
                        stable_since=t if stable_since is None else stable_since
                        if t-stable_since>=2:transition('HOVER')
                    else:stable_since=None
                    if t-phase_start>20:raise RuntimeError('Takeoff did not settle')
                elif phase=='HOVER':
                    target([0,0,.6])
                    if contacts or abs(local[2]-.6)>.1 or speed>.15:raise RuntimeError('Hover bound violated')
                    hover.append([t,*local,speed,math.degrees(tilt)])
                    if t-hover[0][0]>=(route.mission['pre_route_hover_s'] if route else 10):transition('ROUTE' if route else 'OFFSET')
                elif phase=='ROUTE':
                    route.check(plant.qpos[:3]);p=route.target(t,plant.qpos[:3],speed,contacts,plant.qvel[:3])
                    if p is None:
                        summary['route']={'passed':True,'visited':route.visited,'impact':route.impact};transition('RETURN')
                    else:target(p-plant.origin,route.horizontal_command_limit_m_s,route.vertical_command_limit_m_s,route.reference_velocity,route.mission['truth_position_gain_s_inv'],route.mission['truth_vertical_position_gain_s_inv'])
                elif phase=='OFFSET':
                    target([min(.12,.04*(t-phase_start)),0,.6])
                    if t-phase_start>6 and np.linalg.norm(local-[.12,0,.6])<.015 and speed<.02:transition('RETURN')
                    elif t-phase_start>20:raise RuntimeError('Offset did not settle')
                elif phase=='RETURN':
                    target([0 if route else max(0,.12-.04*(t-phase_start)),0,.6])
                    if t-phase_start>6 and np.linalg.norm(local[:2])<.004 and abs(local[2]-.6)<.015 and speed<.01:transition('DESCEND')
                    elif t-phase_start>25:raise RuntimeError('Return alignment failed')
                elif phase=='DESCEND':
                    target([0,0,max(.12,.6-.04*(t-phase_start))],z_limit=.04)
                    if abs(local[2]-.12)<.005 and abs(plant.qvel[2])<.01:transition('ALIGN')
                    elif t-phase_start>25:raise RuntimeError('Approach did not settle')
                elif phase=='ALIGN':
                    target([0,0,.12])
                    if np.linalg.norm(local[:2])<.003 and np.linalg.norm(plant.qvel[:2])<.006 and abs(local[2]-.12)<.005:
                        stable_since=t if stable_since is None else stable_since
                        if t-stable_since>=.75:transition('FINAL_DESCEND')
                    else:stable_since=None
                    if t-phase_start>20:raise RuntimeError('Alignment not stable')
                elif phase=='FINAL_DESCEND':
                    if tel.get('mode')!='LAND':
                        target([0,0,max(-.02,.12-.03*(t-phase_start))],z_limit=.03)
                        if local[2]<.06 and (not land_requested or t>=next_action):
                            rt.mode('LAND');land_requested=True;event('request_LAND')
                            next_action=t+1
                    if tel.get('mode')=='LAND' and tel.get('armed'):land_observed=True
                    if local[2]<.1 and np.linalg.norm(local[:2])>.006:raise RuntimeError('Alignment lost near rim')
                    if contacts:
                        if speed>.15:raise RuntimeError('Landing too fast')
                        landing={'physics_time':t,'local_position':local.tolist(),'speed_m_s':speed,'contacts':contacts}
                        land_monitor.check(t,local,contacts)
                        event('touchdown',**landing);next_action=t+1;transition('LAND')
                    elif t-phase_start>25:raise RuntimeError('Descent contact timeout')
                elif phase=='LAND':
                    if tel.get('mode')=='LAND' and tel.get('armed'):land_observed=True
                    land_monitor.check(t,local,contacts)
                    if not tel.get('armed'):
                        confirm_auto_disarm(tel,t,land_observed)
                        summary['auto_disarm']={'confirmed':True,'landed_state':tel['landed_state'],'post_disarm_mode':tel.get('mode')}
                        transition('POST')
                    elif tel.get('mode')!='LAND':
                        target([0,0,-.02],z_limit=.03)
                        if t>=next_action:rt.mode('LAND');next_action=t+1
                    if t-phase_start>15:raise RuntimeError('Auto-disarm timeout')
                elif phase=='POST':
                    if tel.get('armed') or not contacts or speed>.01 or np.linalg.norm(local[:2])>.007 or abs(local[2]-.02)>.005:raise RuntimeError('Disarmed resting gate failed')
                    if t-phase_start>=1 and np.max(np.abs(plant.data.actuator_force[plant.motors]))<1e-5:
                        summary['passed']=True;event('complete');break
            if not summary['passed']:raise TimeoutError('Wall-clock deadline')
    except (Exception,KeyboardInterrupt) as exc:
        summary['failure']=f'{type(exc).__name__}: {exc}';(output/'failure.txt').write_text(traceback.format_exc())
    finally:
        if viewer:viewer.close()
        summary.update(phase=phase,physics_time_s=float(plant.data.time),landing=landing,hover_samples=len(hover),max_tilt_deg=max_tilt,max_contact_force_N=max_force,max_penetration_m=max_penetration,final_position_local_nwu=(plant.qpos[:3]-plant.origin).tolist())
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        if rt:(output/'parameters_readback.json').write_text(json.dumps(rt.parameters,indent=2)+'\n')
    print(json.dumps(summary,indent=2));return 0 if summary['passed'] else 1
