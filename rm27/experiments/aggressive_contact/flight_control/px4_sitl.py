"""PX4 v1.17.0 backend for the existing Phase 4/5 contact research plant.

An explicit airborne initialization fixture precedes free flight. Policy input
is PX4 EKF telemetry; synthetic external navigation is a separate sensor path.
Recorded motors are scored with the existing contact/recovery evaluator.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import time
import traceback

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from rm27.autopilot.px4.runtime import Runtime, mavlink
from rm27.autopilot.px4.bridge import SensorProducer
from rm27.autopilot.px4.control import ForceMapping, PX4Adapter
from rm27.autopilot.bridge.frames import attitude_to_ap, to_ap
from ..model import ROOT, load_config, VehicleEstimate, TargetState, unit
from ..teacher import prepare
from ..teacher_interface import RateThrust, project
from ..evidence import snapshot, write_json
from ..distillation.deployment import DeploymentPolicy, TargetEstimate
from ..distillation.dataset import designs
from ..distillation.observation import CausalState, vector
from ..maneuver import ManeuverSkill
from .plant import ContactPlant
from .estimator import TelemetryEstimator
from .synthetic import ErrorSensor
from .quality import EstimateQuality, CommitmentGuard, select_mode


class ExternalNavigation(SensorProducer):
    """Phase 5 delayed/noisy physical sensors, serialized as PX4 ODOMETRY."""
    def __init__(self, plant, configuration):
        super().__init__(plant)
        self.configuration=configuration
        self.errors=ErrorSensor(configuration)
        self.last_source=None
        self.packets=0

    def odometry(self):
        d=self.plant.data
        state=VehicleEstimate(d.qpos[:3].copy(), d.qvel[:3].copy(),
            d.xmat[self.plant.body].reshape(3,3).copy(), d.qvel[3:6].copy(),
            float(d.time), 'SIMULATION_SENSOR_PRODUCER')
        sensed,_=self.errors.update(state)
        if self.last_source is not None and sensed.timestamp<=self.last_source:
            return None
        self.last_source=sensed.timestamp
        self.packets+=1
        position=to_ap(sensed.position-self.plant.origin)
        rotation=np.diag([1.,-1.,-1.])@sensed.rotation@np.diag([1.,-1.,-1.])
        velocity=rotation.T@to_ap(sensed.velocity)
        q=Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]].tolist()
        covariance=[0.]*21
        c=self.configuration
        for i,v in zip((0,6,11,15,18,20),[max(.003,c.get('position_std',0.))**2]*3+
                       [np.deg2rad(max(.1,c.get('attitude_std_deg',0.)))**2]*3):
            covariance[i]=v
        velocity_covariance=[0.]*21
        for i in (0,6,11):velocity_covariance[i]=max(.02,c.get('velocity_std',0.))**2
        for i in (15,18,20):velocity_covariance[i]=max(.01,c.get('rate_std',0.))**2
        return mavlink.MAVLink_odometry_message(max(1,round(sensed.timestamp*1e6)),
            mavlink.MAV_FRAME_LOCAL_NED,mavlink.MAV_FRAME_BODY_FRD,*position,q,
            *velocity,*to_ap(sensed.omega_body),covariance,velocity_covariance,0,
            mavlink.MAV_ESTIMATOR_TYPE_VISION,100)


class ResearchRuntime(Runtime):
    def __init__(self, plant, output, instance, study):
        super().__init__(plant,output,instance,profile='response')
        self.navigation=ExternalNavigation(plant,study.get('external_navigation',dict(update_hz=50)))
        self.sensor=self.navigation
        self.external_vision=False

    def step(self, timeout=5.):
        packet=self.navigation.odometry()
        if packet is not None:self.hil.send(packet)
        return super().step(timeout)


class TeacherPolicy:
    """Existing geometric teacher with contact inferred from its observations."""
    def __init__(self, cfg, info, initial, target, mode, design_overrides=None):
        committed=target.commit(initial.timestamp)
        design=copy.deepcopy(designs()[mode])
        for name,value in (design_overrides or {}).items():
            if name not in design:raise ValueError('Unknown Teacher design override: '+name)
            if isinstance(design[name],dict):design[name].update(value)
            else:design[name]=value
        self.actor=ManeuverSkill(cfg,design,initial,committed,info,initial.position)
        self.actor.deployment={}
        self.causal=CausalState(cfg,info,committed,initial,window_s=.02)
        self.mode=['aggressive','balanced','conservative'].index(mode)
        self.previous_action=np.r_[np.zeros(3),info['mass_kg']*9.81]
        self.records=[]

    def infer(self, observation, now):
        s=observation.estimate
        if not 0<=now-s.timestamp<=.1:raise ValueError('Teacher telemetry stale/future')
        actuator=self.causal.update(s,now)
        contact_elapsed=None if self.causal.contact_time is None else self.causal.contact_time-self.actor.start_time
        feature=vector(s,self.actor.target,actuator,now-self.actor.start_time,self.actor.p['duration_s'],
            self.mode,self.previous_action,contact_elapsed,now-s.timestamp)
        contact=self.causal.contact_time is not None and now-self.causal.contact_time<=.02
        if self.actor.deployment.get('contact_disabled'):
            command,_=self.actor.track(s,self.actor.reference.sample(now-self.actor.start_time))
        else:
            command,_=self.actor.command(s,contact)
        self.causal.previous_motor=command.copy()
        gain=2*self.actor.cfg['controller']['attitude_damping']*self.actor.cfg['controller']['attitude_frequency_rad_s']
        action=project(command,s.omega_body,self.actor.inertia,self.actor.mixer,gain)
        action=RateThrust(np.clip(action.body_rates_flu_rad_s,-20,20),action.collective_force_N)
        self.previous_action=np.r_[action.body_rates_flu_rad_s,action.collective_force_N]
        self.records.append((now,s.timestamp,feature,self.previous_action.copy()))
        return action

    def save(self,path):
        np.savez_compressed(path,control_time_s=np.array([r[0] for r in self.records]),
            source_time_s=np.array([r[1] for r in self.records]),
            observation=np.array([r[2] for r in self.records]),
            teacher_rate_thrust=np.array([r[3] for r in self.records]))


def run(output, mode='hover', observation='telemetry', instance=45, duration=8.,
        parameter_file=None, speedup=2., action_layer='rate', skill_mode='balanced',
        study_file=None, policy='student'):
    if observation!='telemetry':
        raise ValueError('PX4 research actor requires telemetry; inject Phase 5 errors through external_navigation')
    if parameter_file and Path(parameter_file).resolve()!=ROOT/'configs/px4/response.json':
        raise ValueError('PX4 research uses reviewed configs/px4/response.json; AP .parm files cannot be applied')
    if policy not in ('student','teacher') or action_layer not in ('rate','attitude'):
        raise ValueError('explicit student/teacher policy and rate/attitude action required')
    if not math.isfinite(speedup) or speedup<=0 or not 0<duration<=30:
        raise ValueError('positive speedup and duration <=30 s required')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    cfg=load_config();snapshot(output,cfg)
    inputs=json.loads((ROOT/'configs/px4_research_inputs.json').read_text())
    source_scene=(ROOT/inputs['scene']).resolve()
    if hashlib.sha256(source_scene.read_bytes()).hexdigest()!=inputs['scene_sha256']:
        raise ValueError('comparison source scene hash changed')
    asset_map={before:str((ROOT/after).resolve()) for before,after in inputs['asset_root_map'].items()}
    (c,bundle,slot),_=prepare(cfg,output/'plant',source_path=source_scene,asset_root_map=asset_map)
    plant=ContactPlant(bundle)
    write_json(output/'plant_config.json',c)
    write_json(output/'vehicle_info.json',plant.info)
    bundle_path=(ROOT/inputs['bundle']).resolve()
    interface=json.loads((ROOT/'configs/px4_contact_policy.json').read_text())
    deployment=json.loads((bundle_path/'deployment_options.json').read_text())
    deployment.update(interface['deployment_overrides'])
    write_json(output/'research_inputs.json',dict(configuration=inputs,interface=interface,
        bundle_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bundle_path.iterdir() if p.is_file()}))
    study=json.loads(Path(study_file).read_text()) if study_file else {}
    if study.get('diagnostic_correction','none')!='none' or study.get('sensor'):
        raise ValueError('PX4 Phase 5 uses external_navigation; privileged actor corrections are unsupported')
    if study.get('teacher_design_overrides') and policy!='teacher':
        raise ValueError('Teacher design overrides cannot silently change a pretrained Student reference')
    write_json(output/'estimator_study.json',study)
    estimator=TelemetryEstimator(plant.origin,source_name='PX4_EKF2_TELEMETRY')
    result=dict(passed=False,autopilot='px4',firmware='v1.17.0',events=[],mode=mode,
        observation=observation,policy=policy,action_layer=action_layer,skill_mode=skill_mode,
        model_mass_kg=plant.info['mass_kg'],run_type='closed-loop SITL airborne contact research',
        estimation_source='PX4 EKF2 and synthetic ODOMETRY sensor',gate_version='existing Phase 4/5 replay evaluator',
        scope='airborne comparison-model research; no target image, takeoff, return or landing qualification')
    frames=[];actions=[];estimates=[];start=armed_at=None;pilot=guard=None
    next_send=next_heartbeat=next_params=0.;next_action=10.;wall=time.monotonic()
    def event(name,**fields):
        row=dict(time_s=float(plant.data.time),event=name,**fields)
        result['events'].append(row);print(json.dumps(row),flush=True)
    try:
        with ResearchRuntime(plant,output,instance,study) as rt,(output/'frames.jsonl').open('x') as stream:
            adapter=PX4Adapter(ForceMapping(4*plant.maximum),rt.system)
            pacing=time.monotonic()
            while time.monotonic()-wall<180:
                row=rt.step();now=float(plant.data.time)
                for message in rt.receive():
                    estimator.consume(message.to_dict(),now)
                    if message.get_type()=='STATUSTEXT':event('STATUSTEXT',text=message.text)
                    if message.get_type()=='COMMAND_ACK':event('ACK',command=message.command,result=message.result)
                if now>=next_heartbeat:rt.heartbeat();next_heartbeat=now+1
                delay=pacing+now/speedup-time.monotonic()
                if delay>0:time.sleep(min(delay,.02))
                if start is None:
                    if now>=next_params:rt.mav.param_request_list_send(rt.system,1);next_params=now+5
                    if now>=next_send:
                        action=RateThrust(np.zeros(3),plant.info['mass_kg']*9.81)
                        rt.mav.set_attitude_target_send(**adapter.packet(action,now,
                            rotation_nwu_flu=np.eye(3) if action_layer=='attitude' else None))
                        next_send=now+.008
                    verified=all(k in rt.parameters and math.isclose(rt.parameters[k],v,rel_tol=1e-5,abs_tol=1e-5)
                                 for k,v in rt.expected.items())
                    if verified and now>=next_action:
                        next_action=now+2
                        if rt.telemetry.get('mode')!='OFFBOARD':rt.mode('OFFBOARD')
                        elif not rt.telemetry.get('armed'):rt.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
                    if rt.telemetry.get('armed') and rt.telemetry.get('mode')=='OFFBOARD':
                        if armed_at is None:armed_at=now
                        if now-armed_at>=4 and verified:
                            estimator.lock_clock();estimator.local_registration=estimator.position[1].copy();estimator.last_pos_time=-np.inf
                            initial=estimator.observation(now).estimate
                            target=TargetEstimate(np.array(c['target']['center_m'])+study.get('target_bias_m',np.zeros(3)),
                                unit(c['target']['normal']),np.zeros((3,3)),initial.timestamp,1.)
                            quality=EstimateQuality.from_config(study.get('quality',{}))
                            if study.get('schedule'):
                                skill_mode=select_mode(skill_mode,quality,target.normal_nwu,
                                    np.linalg.norm(c['contact']['offset_body_m']),np.eye(3)*.05**2,0.)
                            pilot=TeacherPolicy(c,plant.info,initial,target,skill_mode,study.get('teacher_design_overrides')) if policy=='teacher' else DeploymentPolicy.from_bundle(
                                bundle_path,initial,target,mode=skill_mode,options=deployment)
                            causal=pilot.causal if policy=='teacher' else pilot.actor.causal
                            causal.cfg=copy.deepcopy(causal.cfg)
                            causal.cfg['control_dt_s']=interface['causal_observer_dt_s']
                            if mode!='maneuver':
                                class Hover:
                                    def sample(self,t):return initial.position,np.zeros(3),np.zeros(3),np.zeros(3)
                                pilot.actor.reference=Hover()
                                if policy=='teacher':
                                    pilot.actor.trajectory=Hover();pilot.actor.split=None
                            if study.get('guard'):guard=CommitmentGuard(pilot.actor,quality)
                            plant.release();start=now;next_send=now
                            np.savez_compressed(output/'release_state.npz',qpos=plant.data.qpos,qvel=plant.data.qvel,act=plant.data.act,time=now)
                            write_json(output/'verified_parameters.json',{k:rt.parameters[k] for k in rt.expected})
                            if policy=='teacher':write_json(output/'teacher_design.json',pilot.actor.design)
                            launch=json.loads((output/'launch.json').read_text())
                            launch.update(skill_mode=skill_mode,policy=policy,action_layer=action_layer,initialization='explicit airborne fixture')
                            write_json(output/'launch.json',launch)
                            event('RELEASE',clock_offset_s=estimator.offset)
                    if now>50:raise RuntimeError('PX4 research initialization timeout')
                    continue
                if not rt.telemetry.get('armed') or rt.telemetry.get('mode')!='OFFBOARD':raise RuntimeError('Unexpected mode/disarm')
                if now-rt.telemetry.get('estimator_time',-100)>.3 or not rt.telemetry.get('estimator_flags',0)&mavlink.ESTIMATOR_POS_HORIZ_REL:
                    raise RuntimeError('PX4 research estimate invalid')
                obs=estimator.observation(now);s=obs.estimate;elapsed=now-start
                if now>=next_send:
                    if guard:guard.update(obs,now)
                    action=pilot.infer(obs,now)
                    if mode=='rate_step' and 1<=elapsed<3:
                        pulse=.3 if (elapsed-1)%.5<.25 else -.3
                        action=RateThrust(np.array([0.,pulse,0.]),plant.info['mass_kg']*9.81/max(.5,s.rotation[2,2]))
                    orientation=None
                    if action_layer=='attitude' and mode!='rate_step':
                        ref=pilot.actor.last_reference_state
                        orientation=Rotation.from_quat(ref[[7,8,9,6]]).as_matrix()
                    fields=adapter.packet(action,now,rotation_nwu_flu=orientation)
                    rt.mav.set_attitude_target_send(**fields);next_send=now+1/interface['command_hz']
                    actions.append(np.r_[elapsed,action.body_rates_flu_rad_s,action.collective_force_N,fields['thrust']])
                estimates.append(np.r_[elapsed,s.timestamp,now-s.timestamp,s.position,s.velocity,s.rotation.ravel(),s.omega_body])
                frames.append(np.r_[elapsed,plant.data.qpos,plant.data.qvel,plant.data.act,plant.data.ctrl])
                stream.write(json.dumps(dict(sim_time_s=now,truth=plant.truth(),controls=row['controls'],
                    estimate=dict(timestamp=s.timestamp,position=s.position.tolist(),source=s.source)),allow_nan=False)+'\n')
                if not np.isfinite(np.r_[plant.data.qpos,plant.data.qvel]).all() or plant.data.qpos[2]<.15 or np.linalg.norm(plant.data.qvel[:3])>12 or plant.data.xmat[plant.body].reshape(3,3)[2,2]<.1:
                    raise RuntimeError('Research evaluation envelope exceeded')
                if elapsed>=duration:result['completed']=True;event('FLIGHT_COMPLETE');break
            if not result.get('completed'):raise RuntimeError('PX4 research wall deadline')
            result['external_navigation_packets']=rt.navigation.packets
    except Exception as exc:
        result['failure']=f'{type(exc).__name__}: {exc}'
        (output/'traceback.log').write_text(traceback.format_exc());event('FAIL',reason=result['failure'])
    finally:
        np.savez_compressed(output/'flight.npz',trace=np.array(frames),actions=np.array(actions),estimates=np.array(estimates))
        if pilot and hasattr(pilot.actor,'save'):pilot.actor.save(output/'student.npz')
        if pilot and policy=='teacher':pilot.save(output/'teacher_data.npz')
        result.update(release_time_s=start,wall_s=time.monotonic()-wall,
            commitment_events=[] if guard is None else guard.events,aborted=bool(guard and guard.aborted))
        if frames:
            x=np.array(frames);e=np.array(estimates)
            result.update(position_estimate_rmse_m=float(np.sqrt(np.mean(np.sum((x[:,1:4]-e[:,3:6])**2,axis=1)))),
                maximum_observation_age_s=float(e[:,2].max()),final_displacement_m=float(np.linalg.norm(x[-1,1:4]-plant.origin)))
        if mode=='maneuver' and result.get('completed'):
            try:
                from .evaluation import evaluate
                result['contact_evaluation']=evaluate(output,output/'evaluation')
                result['passed']=result['contact_evaluation']['passed']
                if guard and guard.aborted:
                    evaluated=result['contact_evaluation'];last=frames[-1]
                    rotation=Rotation.from_quat(np.asarray(last)[[5,6,7,4]]).as_matrix()
                    result['safe_abort']=bool(evaluated['replay_valid'] and evaluated['first_contact'] is None
                        and np.linalg.norm(last[8:11])<.3 and rotation[2,2]>np.cos(np.deg2rad(10))
                        and np.linalg.norm(last[11:14])<.524 and evaluated['min_altitude_m']>.5
                        and evaluated['max_tilt_deg']<65 and set(evaluated['failure_modes'])<=
                        {'TARGET_MISS','RECOVERY_TIMEOUT','NO_SEPARATION'})
            except Exception as exc:result['evaluation_failure']=repr(exc)
        elif mode!='maneuver' and result.get('completed'):
            rates=np.array(frames)[:,11:14]
            result['settled_rate_rms_rad_s']=float(np.sqrt(np.mean(np.sum(rates[np.array(frames)[:,0]>2]**2,axis=1))))
            result['passed']=result['final_displacement_m']<.25 and result['settled_rate_rms_rad_s']<.3
        write_json(output/'summary.json',result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    p.add_argument('--mode',choices=['hover','rate_step','maneuver'],default='hover')
    p.add_argument('--policy',choices=['student','teacher'],default='student')
    p.add_argument('--action-layer',choices=['rate','attitude'],default='rate')
    p.add_argument('--skill-mode',choices=['aggressive','balanced','conservative'],default='balanced')
    p.add_argument('--instance',type=int,default=35);p.add_argument('--duration',type=float,default=8.)
    p.add_argument('--speedup',type=float,default=2.);p.add_argument('--study')
    a=p.parse_args();r=run(a.output,mode=a.mode,instance=a.instance,duration=a.duration,speedup=a.speedup,
        action_layer=a.action_layer,skill_mode=a.skill_mode,study_file=a.study,policy=a.policy)
    print(json.dumps(r,indent=2));return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
