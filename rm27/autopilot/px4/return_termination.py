"""Bounded return-camera failure handling in the same live SITL process.

Admission uses only telemetry and the last assigned image. Physical state is
read separately for the termination score, never to select a landing command.
Successful termination always remains a failed mission.
"""
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .runtime import mavlink

CAMERA_FAILURES=('AprilTag reacquisition timed out','AprilTag reacquisition attempts exhausted')


def landing_admission(telemetry,tag,now,settings):
    if not tag or not tag.get('valid') or tag.get('assigned_tag_id')!=settings['assigned_tag_id']:
        return False
    if not 0<=now-tag.get('source_time_s',-100)<=settings['maximum_cached_tag_age_s']:return False
    if any(not 0<=now-telemetry.get(k,-100)<=.3 for k in ('local_time','attitude_time','estimator_time','heartbeat_time')):
        return False
    if not telemetry.get('estimator_flags',0)&mavlink.ESTIMATOR_POS_HORIZ_REL:return False
    position=np.asarray(telemetry.get('local_position',[]),float)
    velocity=np.asarray(telemetry.get('local_velocity',[]),float)
    q=np.asarray(telemetry.get('quaternion',[]),float)
    image=np.asarray(tag.get('position_relative_slot_nwu_xy_m',[]),float)
    if position.shape!=(3,) or velocity.shape!=(3,) or q.shape!=(4,) or image.shape!=(2,):return False
    if not np.isfinite(np.r_[position,velocity,q,image]).all() or abs(np.linalg.norm(q)-1)>.02:return False
    tilt=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()[2,2]
    return bool(telemetry.get('armed') and telemetry.get('mode')=='OFFBOARD' and
        tilt>=np.cos(np.deg2rad(10)) and np.linalg.norm(velocity)<=.05 and
        .08<=-position[2]<=.8 and np.linalg.norm(image)<=settings.get('maximum_cached_tag_xy_m',.004))


def continue_landing(runtime,settings):
    from .flight import confirm_auto_disarm
    runtime.return_phase='ABORT_LAND'
    start=float(runtime.plant.data.time);wall=time.monotonic();next_request=next_heartbeat=start
    observed=False;disarmed=None;last=None;events=[]
    unintended=[];stable=True
    with (runtime.output/'termination_frames.jsonl').open('x',buffering=1) as stream:
        while time.monotonic()-wall<settings['termination_wall_deadline_s']:
            now=float(runtime.plant.data.time)
            if now-start>settings['termination_physics_deadline_s']:raise TimeoutError('continued LAND physics deadline')
            if now>=next_heartbeat:runtime.heartbeat();next_heartbeat=now+1
            if runtime.telemetry.get('mode')!='LAND' and runtime.telemetry.get('armed') and now>=next_request:
                runtime.mode('LAND');next_request=now+1
            row=runtime.step();now=float(runtime.plant.data.time)
            for message in runtime.receive():
                if message.get_type() in ('COMMAND_ACK','STATUSTEXT'):
                    events.append(dict(timestamp_s=now,message=message.to_dict()))
            if runtime.telemetry.get('mode')=='LAND' and runtime.telemetry.get('armed'):observed=True
            # Independent score only: these contacts cannot affect commands.
            contacts=runtime.plant.contacts()
            unintended.extend(c for c in contacts if 'rm27_dock_collision_bottom' not in c['geoms'])
            stable &= all(c['force_N']<=10 and c['distance_m']>=-.005 for c in contacts)
            row.update(telemetry=runtime.telemetry.copy(),contacts=contacts,
                actual_motor_thrust_N=runtime.plant.data.actuator_force[runtime.plant.motors].tolist())
            stream.write(json.dumps(row,allow_nan=False)+'\n');last=row
            if not runtime.telemetry.get('armed'):
                confirm_auto_disarm(runtime.telemetry,now,observed)
                disarmed=now if disarmed is None else disarmed
                if now-disarmed>=1:break
        else:raise TimeoutError('continued LAND wall deadline')
    position=np.asarray(last['truth']['position_nwu'])-runtime.plant.origin
    checks=dict(automatic_disarm=disarmed is not None,normal_land_observed=observed,
        physical_rest=bool(last['contacts'] and np.linalg.norm(last['truth']['velocity_nwu'])<=.01),
        bounded_rest=bool(np.linalg.norm(position[:2])<=settings['safe_rest_horizontal_m'] and abs(position[2]-.02)<=.005),
        no_unintended_contact=not unintended,stable_contacts=bool(stable),
        motors_stopped=max(last['actual_motor_thrust_N'])<1e-5)
    return dict(passed=all(checks.values()),checks=checks,events=events,
        horizontal_rest_error_m=float(np.linalg.norm(position[:2])),duration_s=last['timestamp']-start,
        task_success=False,scope='Bounded return-camera loss termination; navigation/communication loss remain separate')


def handle_camera_failure(runtime,reason,cached_tag,settings):
    eligible=(reason in CAMERA_FAILURES and
        landing_admission(runtime.telemetry,cached_tag,float(runtime.plant.data.time),settings))
    result=dict(passed=False,task_success=False,admitted=bool(eligible),trigger=reason,
        gate_version=settings['gate_version'],
        run_type='headless closed-loop SITL; same physics continued through termination',
        strategy='bounded normal LAND with fresh valid navigation and previously aligned assigned tag')
    try:
        if eligible:result.update(continue_landing(runtime,settings))
    except Exception as error:
        result.update(passed=False,failure=f'{type(error).__name__}: {error}')
    (runtime.output/'termination_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
