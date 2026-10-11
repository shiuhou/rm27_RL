"""PX4 micro-flight and audited-route runner with independent physical gates."""
import hashlib
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import time
import traceback
import subprocess
import mujoco
import numpy as np
from pymavlink.dialects.v20 import common as mavlink
from ...simulation.micro_plant import MicroPlant
from ..bridge.frames import to_ap
from .runtime import Runtime
from .visual_return import VisualReturnGuard


@dataclass(frozen=True)
class FlightPolicy:
    """Task input and landing semantics; physical thresholds stay in the scorer."""
    causal_inputs: bool = False
    ground_rest_confirmation: bool = False
    retry_alignment: bool = False


LEGACY_POLICY = FlightPolicy()
PROTOTYPE_MISSION_POLICY = FlightPolicy(True, True, True)


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


def independent_resting_score(plant, final_local, *, check_motion=False):
    """JSON-safe physical result; this is never supplied to the mission actor."""
    passed=bool(plant.contacts() and np.linalg.norm(final_local[:2])<=.007 and abs(final_local[2]-.02)<=.005)
    speed=float(np.linalg.norm(plant.qvel[:3]))
    force=float(np.max(np.abs(plant.data.actuator_force[plant.motors])))
    if check_motion:
        passed=bool(passed and speed<=.01 and force<1e-5)
    return dict(passed=passed,source='MuJoCo truth, scoring only',
        horizontal_error_m=float(np.linalg.norm(final_local[:2])),height_error_m=float(abs(final_local[2]-.02)),
        physical_speed_m_s=speed,maximum_actual_motor_force_N=force)


def run(scene, output, instance=0, *, dynamics=None, view=False, speedup=None, route=None, profile='diagnostic', runtime_factory=Runtime,wall_deadline_s=300., policy=LEGACY_POLICY):
    speedup=(1. if view else 5.) if speedup is None else float(speedup)
    if not math.isfinite(speedup) or speedup<=0:raise ValueError('speedup must be positive')
    scene=Path(scene).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    plant=MicroPlant(scene,dynamics_profile=dynamics)
    if route:
        # The current aircraft contact replay converges at 1 ms versus .5 ms;
        # 2 ms versus 1 ms misses the 10% peak-force criterion. Dock-only
        # flights retain their native 2 ms step and 250 Hz HIL scheduling.
        plant.model.opt.timestep=.001
    if dynamics: (output/'dynamics.json').write_bytes(Path(dynamics).read_bytes())
    summary={'passed':False,'autopilot':'px4','firmware':'v1.17.0','run_type':'closed-loop SITL','headless':not view,'estimation_source':'synthetic ODOMETRY to EKF2','guidance':'diagnostic truth-assisted' if profile=='diagnostic' else 'EKF telemetry','gate_version':'MICRO_FLIGHT existing physical thresholds','events':[],'failure':None}
    telemetry_guidance=profile!='diagnostic'
    if profile in ('flow','mtf'):summary['estimation_source']='numerical optical flow, scene-ray range, simulated compass and IMU; no GPS or ODOMETRY'
    elif profile=='vio':summary['estimation_source']='instrumented native OpenVINS plus numerical flow/range/compass and IMU; development only'
    if profile in ('mtf','vio'):summary['flow_minimum_range_m']=.08
    elif profile=='combined':summary['estimation_source']='synthetic ODOMETRY plus numerical flow/range/compass and IMU'
    metadata={'scene':str(scene),'scene_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'mass_kg':float(plant.model.body_mass[plant.body]),'origin_nwu':plant.origin.tolist(),'profile':profile,'dynamics_profile':str(dynamics),'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    metadata['physics_timestep_s']=float(plant.model.opt.timestep)
    source_root=Path(__file__).resolve().parents[3]
    metadata['git_commit']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source_root,text=True).strip()
    difference=subprocess.check_output(['git','diff','--binary','HEAD'],cwd=source_root)
    (output/'source_diff.patch').write_bytes(difference)
    metadata['source_diff_sha256']=hashlib.sha256(difference).hexdigest()
    metadata['source_sha256']={str(path.relative_to(source_root)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [Path(__file__),Path(__file__).with_name('runtime.py'),source_root/'rm27/experiments/route_experiment.py']}
    if route:
        text=json.dumps(route.mission,indent=2,sort_keys=True)+'\n'
        (output/'mission.json').write_text(text)
        metadata['mission_sha256']=hashlib.sha256(text.encode()).hexdigest()
        (output/'route_audit.json').write_text(json.dumps(route.audit,indent=2)+'\n')
    metadata['flight_policy']=asdict(policy)
    if hasattr(plant,'physical_fixture_evidence'):
        metadata['physical_fixture']=plant.physical_fixture_evidence
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2))
    phase='INITIALIZE';phase_start=0.;stable_since=None;next_send=0.;next_action=10.;next_heartbeat=0.;next_params=0.
    hover=[];physical_hover=[];landing=None;land_observed=False;land_requested=False;landing_commitment=None;land_monitor=LandingMonitor();max_tilt=max_force=max_penetration=0.;start=time.monotonic();viewer=None
    physical_violations=[]
    if policy.causal_inputs:
        if not telemetry_guidance or not route:
            raise ValueError('Causal mission requires telemetry guidance and a surveyed-yaw route')
        yaw=float(route.mission['surveyed_initial_yaw_nwu_rad'])
        summary['actor_input_policy']='PX4 telemetry and current image observations; physical monitors score after actor completion'
    else:
        yaw=math.atan2(plant.data.xmat[plant.body].reshape(3,3)[1,0],plant.data.xmat[plant.body].reshape(3,3)[0,0])
    def physical_reject(reason,**fields):
        if policy.causal_inputs:
            physical_violations.append(dict(time_s=float(plant.data.time),reason=reason,**fields))
        else:
            raise RuntimeError(reason)
    flow_bootstrap=profile in ('mtf','vio')
    integral=np.zeros(2);last_send=None;rt=None;tag_correction=None;visual_guard=None
    reacquire_count=0
    def event(name,**fields):
        entry={'event':name,'physics_time':float(plant.data.time),**fields};summary['events'].append(entry);print(json.dumps(entry),flush=True)
    def transition(name):
        nonlocal phase,phase_start,stable_since
        phase=name;phase_start=float(plant.data.time);stable_since=None
        if name=='RETURN':integral[:]=0
        event(name)
    def bootstrap(thrust):
        nonlocal next_send
        t=float(plant.data.time)
        if t<next_send:return
        from scipy.spatial.transform import Rotation
        # Surveyed initial yaw and level attitude are fixed task references;
        # no physical state is read to hold the vehicle during bootstrap.
        q=Rotation.from_euler('z',-yaw).as_quat()[[3,0,1,2]]
        rt.mav.set_attitude_target_send(int(t*1000),rt.system,1,7,q.tolist(),0.,0.,0.,thrust)
        next_send=t+.02
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
            desired=np.array(p,dtype=float)
            if phase in ('RETURN','REACQUIRE','DESCEND','ALIGN','FINAL_DESCEND') and tag_correction is not None:
                desired[:2]+=tag_correction
            goal=to_ap(desired);error=desired-to_ap(measured);discrepancy=np.zeros(3)
        dt=0. if last_send is None else t-last_send
        # PX4 already supplies position and velocity integrators. The legacy AP
        # outer-loop integral assumed GUID_OPTIONS disabled horizontal position.
        v=np.zeros(3) if feedforward is None else np.array(feedforward,copy=True)
        v[:2]+=kp*error[:2]+integral;v[2]+=(kp if kpz is None else kpz)*error[2]
        v[:2]=np.clip(v[:2],-xy_limit,xy_limit);v[2]=np.clip(v[2],-z_limit,z_limit)
        acceleration=None
        if route and route.impact and t-route.impact['time_s'] < route.mission.get('separation_acceleration_duration_s',0.):
            acceleration=to_ap(route.armor_normal*route.mission.get('separation_acceleration_m_s2',0.))
        rt.position(goal,-yaw,discrepancy+to_ap(v),acceleration=acceleration);next_send=t+.02;last_send=t
    try:
        if view:
            from ...simulation.live_view import FlightView
            viewer=FlightView(plant)
        with runtime_factory(plant,output,instance,profile=profile) as rt, (output/'frames.jsonl').open('w') as frames, (output/'mavlink.jsonl').open('w') as messages:
            if getattr(rt,'return_camera',None):
                visual_guard=VisualReturnGuard(rt.return_camera.assigned_id)
            pacing=time.monotonic()
            while time.monotonic()-start<wall_deadline_s:
                if hasattr(rt,'return_phase'):rt.return_phase=phase
                row=rt.step(); t=float(plant.data.time);tel=rt.telemetry
                tag=getattr(rt,'return_observation',None)
                if visual_guard and not landing_commitment and 'local_position' in tel:
                    measured_xy=to_ap(np.asarray(tel['local_position'],dtype=float))[:2]
                    if visual_guard.observe(tag,t,measured_xy,telemetry_age_s=t-tel.get('local_time',-100)):
                        tag_correction=visual_guard.correction.copy()
                        summary['return_tag']=dict(tag_id=visual_guard.assigned_tag_id,last_seen_s=visual_guard.source_time,
                            position_relative_slot_nwu_xy_m=visual_guard.position.tolist(),
                            correction_nwu_xy_m=tag_correction.tolist(),source='rendered downward image and PX4 attitude')
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
                # Mission guidance in the EKF profile is driven by PX4's
                # telemetry estimate.  MuJoCo state remains available for
                # independent physics scoring and safety diagnostics only.
                physical_local=plant.qpos[:3]-plant.origin
                physical_speed=float(np.linalg.norm(plant.qvel[:3]))
                if telemetry_guidance and 'local_position' in tel:
                    local=to_ap(np.asarray(tel['local_position'],dtype=float))
                    speed=float(np.linalg.norm(to_ap(np.asarray(tel.get('local_velocity',[0.,0.,0.]),dtype=float))))
                elif policy.causal_inputs:
                    if tel.get('armed'):raise RuntimeError('No PX4 local navigation while armed')
                    local=np.zeros(3);speed=0.
                else:
                    local=physical_local;speed=physical_speed
                if telemetry_guidance and tag_correction is not None and phase in ('RETURN','REACQUIRE','DESCEND','ALIGN','FINAL_DESCEND','LAND','POST'):
                    local=local.copy();local[:2]-=tag_correction
                mission_contacts=contacts=plant.contacts()
                if telemetry_guidance:
                    # Contact is not an input to the sensor-only mission
                    # state machine.  PX4's landed state and range stream
                    # provide the task-side touchdown signal.
                    mission_contacts=[]
                landed_state=tel.get('landed_state')
                landed=landed_state==mavlink.MAV_LANDED_STATE_ON_GROUND
                tilt=math.acos(float(np.clip(plant.data.xmat[plant.body].reshape(3,3)[2,2],-1,1)))
                max_tilt=max(max_tilt,math.degrees(tilt))
                row.update(phase=phase,telemetry=tel.copy(),contacts=contacts,all_qpos=plant.data.qpos.tolist(),all_qvel=plant.data.qvel.tolist(),all_act=plant.data.act.tolist(),all_motor_thrust_N=plant.data.actuator_force.tolist())
                frames.write(json.dumps(row,allow_nan=False)+'\n')
                if not np.isfinite(np.r_[plant.data.qpos,plant.data.qvel,plant.data.qacc]).all():raise RuntimeError('Nonfinite physics')
                mission_tilt=tilt
                if policy.causal_inputs:
                    q=np.asarray(tel.get('quaternion',[]),float)
                    valid_attitude=(q.shape==(4,) and np.isfinite(q).all() and abs(np.linalg.norm(q)-1)<=.02 and
                        0<=t-tel.get('attitude_time',-100)<=.3)
                    if not valid_attitude and phase!='INITIALIZE':raise RuntimeError('No fresh valid PX4 attitude')
                    mission_tilt=(math.acos(float(np.clip(1-2*(q[1]**2+q[2]**2),-1,1))) if valid_attitude else 0.)
                if mission_tilt>math.radians(route.maximum_tilt_deg if phase=='ROUTE' else 25) or speed>(route.maximum_speed_m_s if phase=='ROUTE' else 2) or (phase!='ROUTE' and (local[2]>1.1 or np.linalg.norm(local[:2])>.5)):
                    raise RuntimeError(f'Flight envelope exceeded: position={local}, speed={speed}, tilt={math.degrees(mission_tilt)}')
                for qa,va,initial in plant.inactive:
                    if np.linalg.norm(plant.data.qpos[qa:qa+2]-initial[:2])>.001 or abs(plant.data.qpos[qa+2]-initial[2])>.002:physical_reject('Inactive vehicle moved')
                    if t>1 and np.max(np.abs(plant.data.qvel[va:va+6]))>.001:physical_reject('Inactive vehicle not resting')
                    q=plant.data.qpos[qa+3:qa+7]
                    if 1-2*(q[1]**2+q[2]**2)<math.cos(math.radians(1)):physical_reject('Inactive vehicle tilted')
                inactive=np.setdiff1d(np.arange(plant.model.nu),plant.motors)
                if np.any(plant.data.ctrl[inactive]) or np.any(plant.data.actuator_force[inactive]):physical_reject('Inactive motor received thrust')
                for c in contacts:
                    max_force=max(max_force,c['force_N']);max_penetration=max(max_penetration,-c['distance_m'])
                    if 'rm27_dock_collision_bottom' not in c['geoms'] and not (phase=='ROUTE' and route.contact_allowed(c)):physical_reject('Unintended contact',contact=c)
                    if 'rm27_dock_collision_bottom' in c['geoms'] and (c['force_N']>10 or c['distance_m']<-.005):physical_reject('Unstable landing contact')
                if phase in ('OFFSET','RETURN','ALIGN') and mission_contacts:raise RuntimeError('Unexpected airborne contact')
                if phase!='INITIALIZE':
                    if t-tel.get('heartbeat_time',-100)>2:raise RuntimeError('Heartbeat stale')
                    allowed_mode=('OFFBOARD','LAND') if phase=='FINAL_DESCEND' and land_requested else ('OFFBOARD',)
                    if phase not in ('LAND','POST') and (not tel.get('armed') or tel.get('mode') not in allowed_mode):raise RuntimeError('Unexpected mode/disarm')
                if visual_guard and not landing_commitment and not land_requested and phase in ('DESCEND','ALIGN','FINAL_DESCEND') and not visual_guard.fresh(t):
                    flags=tel.get('estimator_flags',0)
                    if not flags & mavlink.ESTIMATOR_POS_HORIZ_REL or t-tel.get('estimator_time',-100)>.3:
                        raise RuntimeError('Cannot reacquire AprilTag without valid navigation')
                    if reacquire_count>=2:
                        raise RuntimeError('AprilTag reacquisition attempts exhausted')
                    reacquire_count+=1;summary['tag_reacquire_count']=reacquire_count
                    transition('REACQUIRE')
                if t>(route.physics_deadline if route else 160):raise RuntimeError('Physics deadline')
                if phase=='INITIALIZE':
                    if flow_bootstrap:bootstrap(0.)
                    elif 'local_position' in tel: target([0,0,.02])
                    verified=all(k in rt.parameters and math.isclose(rt.parameters[k],v,abs_tol=1e-5,rel_tol=1e-5) for k,v in rt.expected.items())
                    if t>50 and not verified:raise RuntimeError('Parameter mismatch: '+str({k:(v,rt.parameters.get(k)) for k,v in rt.expected.items() if k not in rt.parameters or not math.isclose(rt.parameters[k],v,abs_tol=1e-5,rel_tol=1e-5)}))
                    if verified and t>=next_action and 'local_position' in tel:
                        next_action=t+2
                        if tel.get('mode')!='OFFBOARD':rt.mode('OFFBOARD')
                        elif not tel.get('armed'):rt.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
                    if verified and tel.get('armed') and tel.get('mode')=='OFFBOARD':
                        summary['parameters_verified']=True;transition('TAKEOFF')
                elif phase=='TAKEOFF':
                    if flow_bootstrap:
                        # The manufacturer's flow minimum is 8 cm, above the
                        # parked sensor. Lift on attitude/thrust, using range
                        # and fresh estimator validity for the handover.
                        if t-tel.get('distance_time',-100)>.1:raise RuntimeError('Bootstrap range stale')
                        height=tel['distance_m']
                        hover_fraction=rt.expected['MPC_THR_HOVER']
                        bootstrap(hover_fraction*float(np.clip(1+1.8*(.25-height),.85,1.2)))
                        flags=tel.get('estimator_flags',0)
                        valid=flags & mavlink.ESTIMATOR_VELOCITY_HORIZ and flags & mavlink.ESTIMATOR_POS_HORIZ_REL
                        if height>=.16 and valid and t-tel.get('estimator_time',-100)<.3:
                            flow_bootstrap=False;phase_start=t;next_send=0.;event('flow_navigation_handover',range_m=height)
                        elif t-phase_start>10:raise RuntimeError('No valid flow navigation after attitude takeoff')
                        continue
                    target([0,0,min(.6,.02+.25*(t-phase_start))],z_limit=.25)
                    if abs(local[2]-.6)<.08 and speed<.08:
                        stable_since=t if stable_since is None else stable_since
                        if t-stable_since>=2:transition('HOVER')
                    else:stable_since=None
                    if t-phase_start>20:raise RuntimeError('Takeoff did not settle')
                elif phase=='HOVER':
                    target([0,0,.6])
                    if mission_contacts or abs(local[2]-.6)>.1 or speed>.15:raise RuntimeError('Hover bound violated')
                    hover.append([t,*local,speed,math.degrees(tilt)])
                    physical_hover.append([t,*physical_local])
                    if t-hover[0][0]>=(route.mission['pre_route_hover_s'] if route else 10):transition('ROUTE' if route else 'OFFSET')
                elif phase=='ROUTE':
                    route_position=plant.origin+local if telemetry_guidance else plant.qpos[:3]
                    route_velocity=to_ap(np.asarray(tel.get('local_velocity',[0.,0.,0.]),dtype=float)) if telemetry_guidance else plant.qvel[:3]
                    previous_impact=route.impact
                    route.check(route_position);p=route.target(t,route_position,speed,mission_contacts,route_velocity)
                    if previous_impact is None and route.impact is not None:
                        next_send=0. # dispatch the separation command on the impact tick
                    if p is None:
                        summary['route']=route.result() if hasattr(route,'result') else {'passed':True,'visited':route.visited,'impact':route.impact}
                        transition('RETURN')
                    else:target(p-plant.origin,route.horizontal_command_limit_m_s,route.vertical_command_limit_m_s,route.reference_velocity,route.mission['truth_position_gain_s_inv'],route.mission['truth_vertical_position_gain_s_inv'])
                elif phase=='OFFSET':
                    target([min(.12,.04*(t-phase_start)),0,.6])
                    if t-phase_start>6 and np.linalg.norm(local-[.12,0,.6])<.015 and speed<.02:transition('RETURN')
                    elif t-phase_start>20:raise RuntimeError('Offset did not settle')
                elif phase=='RETURN':
                    target([0 if route else max(0,.12-.04*(t-phase_start)),0,.6])
                    if t-phase_start>6 and np.linalg.norm(local[:2])<.004 and abs(local[2]-.6)<.015 and speed<.01:
                        if visual_guard and not visual_guard.fresh(t):
                            raise RuntimeError('Assigned AprilTag not freshly observed for return alignment')
                        transition('DESCEND')
                    elif t-phase_start>25:raise RuntimeError('Return alignment failed')
                elif phase=='REACQUIRE':
                    # Return to the reviewed 60 cm observation altitude. This
                    # makes every subsequent descent start continuous and
                    # gives the tag its original field of view after loss.
                    target([0,0,.6],z_limit=.08)
                    if visual_guard.fresh(t) and abs(local[2]-.6)<.015 and speed<.03:
                        stable_since=t if stable_since is None else stable_since
                        if t-stable_since>=.5:transition('DESCEND')
                    else:stable_since=None
                    maximum_gap=route.mission.get('maximum_return_image_gap_s',15.) if policy.causal_inputs and route else 15.
                    image_gap=(t-visual_guard.source_time if visual_guard.source_time is not None else float('inf'))
                    if t-phase_start>15 or (not visual_guard.fresh(t) and image_gap>maximum_gap):
                        raise RuntimeError('AprilTag reacquisition timed out')
                elif phase=='DESCEND':
                    target([0,0,max(.12,.6-.04*(t-phase_start))],z_limit=.04)
                    estimated_vertical_speed=float(to_ap(np.asarray(tel.get('local_velocity',[0.,0.,0.]),dtype=float))[2]) if telemetry_guidance else float(plant.qvel[2])
                    if abs(local[2]-.12)<.005 and abs(estimated_vertical_speed)<.01:transition('ALIGN')
                    elif t-phase_start>25:raise RuntimeError('Approach did not settle')
                elif phase=='ALIGN':
                    target([0,0,.12])
                    estimated_horizontal_speed=float(np.linalg.norm(to_ap(np.asarray(tel.get('local_velocity',[0.,0.,0.]),dtype=float))[:2])) if telemetry_guidance else float(np.linalg.norm(plant.qvel[:2]))
                    if np.linalg.norm(local[:2])<.003 and estimated_horizontal_speed<.006 and abs(local[2]-.12)<.005 and (visual_guard is None or visual_guard.aligned(t)):
                        stable_since=t if stable_since is None else stable_since
                        if t-stable_since>=.75:transition('FINAL_DESCEND')
                    else:stable_since=None
                    if t-phase_start>20:raise RuntimeError('Alignment not stable')
                elif phase=='FINAL_DESCEND':
                    if tel.get('mode')!='LAND':
                        target([0,0,max(-.02,.12-.03*(t-phase_start))],z_limit=.03)
                        land_height=.12 if profile in ('mtf','vio') else .06
                        if local[2]<land_height and (not land_requested or t>=next_action):
                            if profile in ('mtf','vio') and landing_commitment is None:
                                flags=tel.get('estimator_flags',0)
                                if not flags & mavlink.ESTIMATOR_POS_HORIZ_REL or t-tel.get('estimator_time',-100)>.3:
                                    raise RuntimeError('No valid slot alignment at landing commitment')
                                if np.linalg.norm(local[:2])>.003 or (visual_guard and not visual_guard.aligned(t)):
                                    if not policy.retry_alignment:
                                        raise RuntimeError('No fresh aligned AprilTag at landing commitment')
                                    if reacquire_count>=2:
                                        raise RuntimeError('AprilTag reacquisition attempts exhausted')
                                    reacquire_count+=1;summary['tag_reacquire_count']=reacquire_count
                                    event('landing_alignment_retry',source='fresh PX4 navigation and assigned image',attempt=reacquire_count)
                                    transition('ALIGN' if visual_guard and visual_guard.fresh(t) else 'REACQUIRE')
                                    continue
                                landing_commitment=dict(local_position=local.tolist(),timestamp_s=t,
                                    policy='PX4 normal LAND below flow minimum; range/landed state complete the task; independent physics scores slot accuracy')
                                if visual_guard:
                                    landing_commitment['tag_source_time_s']=visual_guard.source_time
                                    landing_commitment['assigned_tag_id']=visual_guard.assigned_tag_id
                                summary['landing_commitment']=landing_commitment
                            rt.mode('LAND');land_requested=True;event('request_LAND')
                            next_action=t+1
                    if tel.get('mode')=='LAND' and tel.get('armed'):land_observed=True
                    # Use the same 7 mm slot envelope as the touchdown monitor;
                    # the earlier 6 mm pre-contact abort rejected otherwise
                    # valid runs when estimator and physics samples straddled
                    # the boundary by one 4 ms HIL tick.
                    if local[2]<.1 and np.linalg.norm(local[:2])>.007 and not (profile in ('mtf','vio') and landing_commitment):raise RuntimeError('Alignment lost near rim')
                    if mission_contacts or landed:
                        if speed>.15:raise RuntimeError('Landing too fast')
                        landing={'physics_time':t,'local_position':local.tolist(),'speed_m_s':speed,'contacts':contacts,'touchdown_source':'px4_landed_state' if telemetry_guidance else 'mujoco_contact'}
                        if not telemetry_guidance:land_monitor.check(t,local,contacts)
                        event('touchdown',**landing);next_action=t+1;transition('LAND')
                    elif t-phase_start>25:raise RuntimeError('Descent contact timeout')
                elif phase=='LAND':
                    if tel.get('mode')=='LAND' and tel.get('armed'):land_observed=True
                    if not telemetry_guidance:land_monitor.check(t,local,contacts)
                    if not tel.get('armed'):
                        confirm_auto_disarm(tel,t,land_observed)
                        summary['auto_disarm']={'confirmed':True,'landed_state':tel['landed_state'],'post_disarm_mode':tel.get('mode')}
                        transition('POST')
                    elif tel.get('mode')!='LAND':
                        target([0,0,-.02],z_limit=.03)
                        if t>=next_action:rt.mode('LAND');next_action=t+1
                    if t-phase_start>15:raise RuntimeError('Auto-disarm timeout')
                elif phase=='POST':
                    settled = landed if telemetry_guidance else bool(contacts)
                    height_tolerance=.01 if telemetry_guidance else .005
                    height=local[2]
                    if profile in ('flow','mtf','vio'):
                        # Range-referenced EKF2 has a floating terrain datum.
                        # Check measured ground clearance for resting height;
                        # do not interpret local z as a surveyed floor height.
                        if t-tel.get('distance_time',-100)>.1:raise RuntimeError('Resting range stale')
                        height=tel['distance_m'];height_tolerance=.005
                    resting_horizontal=np.linalg.norm(local[:2])
                    if profile in ('mtf','vio') and landing_commitment:
                        resting_horizontal=np.linalg.norm(landing_commitment['local_position'][:2])
                        # Flow is intentionally invalid below 8 cm. Fresh
                        # landed state and range establish rest; position is
                        # assessed by the separate physical scorer.
                        speed=abs(tel.get('local_velocity',[0,0,1])[2])
                    ground_confirmed=bool(policy.ground_rest_confirmation and profile in ('mtf','vio') and landing_commitment and
                        not tel.get('armed') and settled and 0<=t-tel.get('landed_time',-100)<=.3 and
                        0<=t-tel.get('heartbeat_time',-100)<=.3 and np.max(np.abs(row['controls']))<1e-6)
                    if (tel.get('armed') or not settled or (not ground_confirmed and speed>.01) or
                            resting_horizontal>.007 or abs(height-.02)>height_tolerance):
                        raise RuntimeError('Disarmed resting gate failed')
                    if ground_confirmed:
                        summary['ground_rest_confirmation']=dict(source='fresh PX4 landed/range/heartbeat and stopped actuator output',
                            estimated_vertical_velocity_m_s=tel.get('local_velocity',[0,0,None])[2],
                            below_flow_minimum=True,physical_velocity_source='independent scorer only')
                    stopped=(np.max(np.abs(row['controls']))<1e-6 if policy.ground_rest_confirmation else
                        np.max(np.abs(plant.data.actuator_force[plant.motors]))<1e-5)
                    if t-phase_start>=1 and stopped:
                        summary['passed']=True;event('complete');break
            if not summary['passed']:raise TimeoutError('Wall-clock deadline')
    except (Exception,KeyboardInterrupt) as exc:
        summary['failure']=f'{type(exc).__name__}: {exc}';(output/'failure.txt').write_text(traceback.format_exc())
    finally:
        if viewer:viewer.close()
        if visual_guard:
            summary['visual_return_admission']=dict(assigned_tag_id=visual_guard.assigned_tag_id,
                source_time_s=visual_guard.source_time,rejected=visual_guard.rejected,
                maximum_age_s=visual_guard.maximum_age_s,maximum_registration_m=visual_guard.maximum_correction_m)
        final_local=plant.qpos[:3]-plant.origin
        if physical_hover:
            h=np.asarray(physical_hover);xy=np.linalg.norm(h[:,1:3],axis=1);z=np.abs(h[:,3]-.6)
            summary['independent_hover_score']=dict(duration_s=float(h[-1,0]-h[0,0]),
                horizontal_maximum_m=float(xy.max()),horizontal_p95_m=float(np.percentile(xy,95)),
                vertical_maximum_m=float(z.max()),vertical_p95_m=float(np.percentile(z,95)),
                passed=bool(xy.max()<=.2 and z.max()<=.1),source='MuJoCo truth, scoring only')
        if summary['passed']:
            score=independent_resting_score(plant,final_local,check_motion=policy.ground_rest_confirmation)
            summary['independent_resting_score']=score
            physical_pass=score['passed']
            if not physical_pass or not summary.get('independent_hover_score',{}).get('passed'):
                summary['passed']=False;summary['failure']='Independent physical hover or dock resting gate failed'
            elif route and summary.get('route',{}).get('passed') is not True:
                summary['safe_return_completed']=True
                summary['passed']=False;summary['failure']='Route task failed; normal return and landing completed'
        if policy.causal_inputs:
            summary['independent_physical_monitor']=dict(passed=not physical_violations,violations=physical_violations,
                source='independent physics scoring only; no actor feedback')
            if physical_violations:
                summary['passed']=False
                summary['failure']=summary.get('failure') or 'Independent physical monitor rejected task'
        summary.update(phase=phase,physics_time_s=float(plant.data.time),landing=landing,hover_samples=len(hover),max_tilt_deg=max_tilt,max_contact_force_N=max_force,max_penetration_m=max_penetration,final_position_local_nwu=(plant.qpos[:3]-plant.origin).tolist())
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        if rt:(output/'parameters_readback.json').write_text(json.dumps(rt.parameters,indent=2)+'\n')
    print(json.dumps(summary,indent=2));return 0 if summary['passed'] else 1
