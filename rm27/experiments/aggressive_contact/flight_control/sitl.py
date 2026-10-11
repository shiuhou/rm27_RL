"""LOCAL simulation-only actual ArduPilot/MuJoCo research integration.

No hardware/remote endpoint option. Firmware process, parameter store and logs
live inside a new artifact directory. Fixture initialization is explicitly
separate from the free flight. No truth correction is added to telemetry.
"""
import argparse,hashlib,json,math,select,socket,subprocess,time,traceback
from pathlib import Path
import numpy as np
import mujoco
from pymavlink import mavutil
from rm27.autopilot.bridge.bridge import Bridge
from rm27.autopilot.micro_flight import read_params
from rm27.autopilot.bridge.launch import EXPECTED_PARAMS
from ..model import ROOT,load_config,VehicleEstimate,unit
from ..teacher import prepare
from ..evidence import snapshot,write_json
from ..distillation.deployment import DeploymentPolicy,TargetEstimate,VehicleObservation,ArduPilotAdapter
from ..teacher_interface import RateThrust
from ..teacher_interface import attitude_ned_frd
from scipy.spatial.transform import Rotation
from .plant import ContactPlant
from .estimator import TelemetryEstimator


class SpinMapping:
    """Inverse of the selected SIM PWM->force plant with AP spin range.

    This is an exact NOMINAL simulation mapping for EXPO=0, angle boost OFF;
    not a hardware thrust curve and not correct for other AP configurations.
    """
    def __init__(self,maximum,minimum=.15,maximum_spin=.95):self.force=maximum;self.low=minimum;self.high=maximum_spin
    def __call__(self,f):return float(np.clip((f/self.force-self.low)/(self.high-self.low),0,1))


def run(output,mode='hover',observation='telemetry',instance=45,duration=8.,parameter_file=None,speedup=2.,action_layer='rate',skill_mode='balanced',study_file=None,autopilot='ardupilot',policy='student'):
    if autopilot=='px4':
        from .px4_sitl import run as px4_run
        return px4_run(output,mode,observation,instance,duration,parameter_file,speedup,action_layer,skill_mode,study_file,policy)
    if autopilot!='ardupilot' or policy!='student':
        raise ValueError('explicit supported autopilot/policy required')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False);cfg=load_config();snapshot(output,cfg)
    prepared,template=prepare(cfg,output/'plant');c,bundle,slot=prepared
    plant=ContactPlant(bundle);bridge=Bridge(plant)
    from .diagnostics import TruthHistory,ExternalNavigationSensor
    from .synthetic import ErrorSensor
    from .quality import EstimateQuality,CommitmentGuard,select_mode
    study=json.loads(Path(study_file).read_text()) if study_file else {}
    write_json(output/'estimator_study.json',study)
    history=TruthHistory();sensor=ErrorSensor(study.get('sensor',{}));quality=EstimateQuality.from_config(study.get('quality',{}));guard=None
    external=ExternalNavigationSensor(study['external_navigation'],plant.origin) if 'external_navigation' in study else None
    write_json(output/'plant_config.json',c)
    ap=ROOT/'rm27/autopilot/vendor/ardupilot';binary=ap/'build/sitl/bin/arducopter'
    parms=[ap/'Tools/autotest/default_params/copter.parm',ap/'Tools/autotest/default_params/copter-X.parm',ROOT/'rm27/autopilot/bridge/hover.parm',ROOT/'configs/micro_hover.parm',ROOT/'configs/contact_phase4/sitl.parm']
    if parameter_file:parms.append(Path(parameter_file).resolve())
    expected=dict(EXPECTED_PARAMS)
    for p in parms[3:]:expected.update(read_params(p))
    command=[str(binary),'--model','JSON:127.0.0.1','--rate','500','--speedup',str(speedup),'--instance',str(instance),'--defaults',','.join(map(str,parms)),'--home','-35.363261,149.165230,584,0','--wipe']
    write_json(output/'launch.json',dict(command=command,mode=mode,observation=observation,instance=instance,free_flight_duration_s=duration,action_layer=action_layer,skill_mode=skill_mode,
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),parameters={str(p):p.read_text() for p in parms},expected=expected,
        scope='LOCAL SITL ONLY, explicit airborne initialization fixture; no hardware/remote endpoint; synthetic SITL sensors, actual firmware EKF and PID'))
    estimate=TelemetryEstimator(plant.origin,velocity_std=.05);adapter=ArduPilotAdapter(SpinMapping(4*plant.maximum),True)
    params={};tel={};requested=False;start=None;pilot=None;next_send=0.;next_heartbeat=0.;next_arm=15.;armed_at=None
    result=dict(passed=False,events=[],mode=mode,observation=observation);rows=[];actions=[];estimates=[];t0=time.monotonic();sock=None;process=None;conn=None
    def event(name,**kw):
        r=dict(time_s=float(plant.data.time),event=name,**kw);result['events'].append(r);print(json.dumps(r),flush=True)
    def cmd(code,*values):conn.mav.command_long_send(1,1,code,0,*(list(values)+[0.]*(7-len(values))))
    try:
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',9002+10*instance));sock.setblocking(False)
        with (output/'sitl.log').open('w') as log,(output/'mavlink.jsonl').open('w',buffering=1) as messages,(output/'frames.jsonl').open('w',buffering=1) as frames:
            process=subprocess.Popen(command,cwd=output,stdout=log,stderr=subprocess.STDOUT)
            while conn is None:
                if process.poll() is not None or time.monotonic()-t0>20:raise RuntimeError('SITL startup failure')
                try:conn=mavutil.mavlink_connection(f'tcp:127.0.0.1:{5760+10*instance}',source_system=255)
                except OSError:time.sleep(.05)
            while time.monotonic()-t0<180:
                if process.poll() is not None:raise RuntimeError('SITL exited')
                if not select.select([sock],[],[],.1)[0]:continue
                payload,address=sock.recvfrom(1024);reply,row=bridge.process(payload);sock.sendto(reply,address);now=float(plant.data.time)
                while True:
                    msg=conn.recv_match(blocking=False)
                    if msg is None:break
                    kind=msg.get_type()
                    if kind=='BAD_DATA':continue
                    m=msg.to_dict();messages.write(json.dumps(dict(received_sim_s=now,message=m),default=str)+'\n');estimate.consume(m,now)
                    if kind=='HEARTBEAT':
                        tel.update(armed=bool(msg.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),mode=mavutil.mode_string_v10(msg))
                        if not requested:
                            conn.mav.request_data_stream_send(1,1,mavutil.mavlink.MAV_DATA_STREAM_ALL,20,1);conn.mav.param_request_list_send(1,1)
                            cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,32,20000)
                            cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,31,5000);requested=True
                    elif kind=='PARAM_VALUE':params[msg.param_id]=msg.param_value
                    elif kind=='EKF_STATUS_REPORT':tel['ekf_flags']=msg.flags
                    elif kind=='STATUSTEXT':
                        event('STATUSTEXT',text=msg.text)
                        if msg.text=='ArduPilot Ready':tel['ready']=True
                    elif kind=='COMMAND_ACK':event('ACK',command=msg.command,result=msg.result)
                if now>=next_heartbeat:
                    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0);next_heartbeat=now+1
                if row is None:continue
                if not np.isfinite(np.r_[plant.data.qpos,plant.data.qvel]).all():raise RuntimeError('Nonfinite plant')
                d=plant.data
                truth=VehicleEstimate(d.qpos[:3].copy(),d.qvel[:3].copy(),d.xmat[plant.body].reshape(3,3).copy(),d.qvel[3:].copy(),now,'TRUTH_SENSOR_PRODUCER')
                history.append(truth)
                if external:
                    external.send(conn,truth)
                    if start is None and int(now*500)%500==0:
                        conn.mav.set_gps_global_origin_send(1,int(-35.363261*1e7),int(149.165230*1e7),584000)
                if start is None:
                    if now>60:raise RuntimeError('Initialization timeout: '+repr(tel))
                    verified=all(k in params and math.isclose(params[k],v,abs_tol=1e-5,rel_tol=1e-6) for k,v in expected.items())
                    if verified and tel.get('ready') and now>=next_arm:
                        next_arm=now+2
                        if tel.get('mode')!='GUIDED':conn.set_mode('GUIDED');event('REQUEST_GUIDED')
                        elif not tel.get('armed') and tel.get('ekf_flags',0)&0x0B==0x0B:cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1);event('REQUEST_ARM')
                    if tel.get('armed'):
                        if armed_at is None:armed_at=now;event('ARMED')
                        if now>=next_send:
                            z=RateThrust(np.zeros(3),plant.info['mass_kg']*9.81)
                            conn.mav.set_attitude_target_send(**adapter.packet(z,now));next_send=now+.02
                        if now-armed_at>=4 and verified:
                            estimate.lock_clock();estimate.local_registration=estimate.position[1].copy();estimate.last_pos_time=-np.inf
                            sensed=estimate.observation(now).estimate
                            initial=VehicleEstimate(sensed.position,sensed.velocity,sensed.rotation,sensed.omega_body,now,'REGISTERED_EKF_START')
                            target=TargetEstimate(np.array(c['target']['center_m']),unit(c['target']['normal']),np.zeros((3,3)),now,1.)
                            target=TargetEstimate(target.position_nwu_m+study.get('target_bias_m',np.zeros(3)),target.normal_nwu,target.covariance_m2,now,1.)
                            if study.get('schedule'):
                                skill_mode=select_mode(skill_mode,quality,target.normal_nwu,np.linalg.norm(c['contact']['offset_body_m']),np.eye(3)*.02**2,study.get('sensor',{}).get('delay_s',0.))
                            pilot=DeploymentPolicy.from_bundle(ROOT/'artifacts/phase3-student-bundle-02',initial,target,mode=skill_mode)
                            if study.get('guard'):guard=CommitmentGuard(pilot.actor,quality)
                            if mode!='maneuver':
                                class Hover:
                                    def sample(self,t):return initial.position,np.zeros(3),np.zeros(3),np.zeros(3)
                                pilot.actor.reference=Hover()
                            plant.release();start=now;next_send=now
                            np.savez_compressed(output/'release_state.npz',qpos=plant.data.qpos,qvel=plant.data.qvel,act=plant.data.act,time=now)
                            write_json(output/'verified_parameters.json',{k:params[k] for k in expected})
                            event('RELEASE',clock_offset=estimate.offset,registration=estimate.local_registration.tolist())
                else:
                    elapsed=now-start
                    if not tel.get('armed') or tel.get('mode')!='GUIDED':raise RuntimeError('Unexpected disarm/mode')
                    obs=estimate.observation(now)
                    if observation=='synthetic':
                        obs=sensor.observation(truth)
                    elif observation=='truth':
                        # Explicit upper-bound diagnostic only; never mixed into telemetry policy.
                        d=plant.data;truth=VehicleEstimate(d.qpos[:3].copy(),d.qvel[:3].copy(),d.xmat[plant.body].reshape(3,3).copy(),d.qvel[3:].copy(),now,'TRUTH_INTERFACE_DIAGNOSTIC')
                        obs=VehicleObservation(truth,np.eye(3)*.02**2)
                    obs=history.correct(obs,study.get('diagnostic_correction','none'))
                    if now>=next_send-1e-9:
                        if guard:guard.update(obs,now)
                        action=pilot.infer(obs,now)
                        if mode=='rate_step' and 1<=elapsed<3:
                            pulse=.3 if (elapsed-1)%.5<.25 else -.3
                            rates=np.array([0.,pulse,0.]);force=plant.info['mass_kg']*9.81/max(.5,obs.estimate.rotation[2,2]);action=RateThrust(rates,force)
                        fields=adapter.packet(action,now)
                        if action_layer=='attitude' and mode!='rate_step':
                            ref=pilot.actor.last_reference_state
                            Rd=Rotation.from_quat(ref[[7,8,9,6]]).as_matrix()
                            fields.update(type_mask=0,q=attitude_ned_frd(Rd).tolist(),body_roll_rate=float(ref[10]),body_pitch_rate=float(-ref[11]),body_yaw_rate=float(-ref[12]))
                        conn.mav.set_attitude_target_send(**fields)
                        actions.append(np.r_[elapsed,action.body_rates_flu_rad_s,action.collective_force_N,fields['thrust']])
                        next_send+=.005
                    s=obs.estimate;estimates.append(np.r_[elapsed,s.timestamp,now-s.timestamp,s.position,s.velocity,s.rotation.ravel(),s.omega_body])
                    rows.append(np.r_[elapsed,plant.data.qpos.copy(),plant.data.qvel.copy(),plant.data.act.copy(),plant.data.ctrl.copy()])
                    frames.write(json.dumps(dict(sim_time_s=now,truth=plant.truth(),estimate=dict(timestamp=s.timestamp,position=s.position.tolist(),velocity=s.velocity.tolist(),rotation=s.rotation.tolist(),omega_body=s.omega_body.tolist()),pwm=row['pwm']))+'\n')
                    if plant.data.qpos[2]<.15 or np.linalg.norm(plant.data.qvel[:3])>12 or plant.data.xmat[plant.body].reshape(3,3)[2,2]<.1:raise RuntimeError('Evaluation envelope exceeded')
                    if mode!='maneuver' and np.linalg.norm(plant.data.qpos[:3]-plant.origin)>2:raise RuntimeError('Hover displacement >2m')
                    if elapsed>=duration:
                        event('FLIGHT_COMPLETE');break
            else:raise RuntimeError('Wall-clock timeout')
            if start is None:raise RuntimeError('No fixture release')
            result['completed']=True
    except Exception as exc:
        result['failure']=repr(exc);(output/'traceback.log').write_text(traceback.format_exc());event('FAIL',reason=repr(exc))
    finally:
        if conn:conn.close()
        if sock:sock.close()
        if process:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        np.savez_compressed(output/'flight.npz',trace=np.array(rows),actions=np.array(actions),estimates=np.array(estimates))
        if pilot:pilot.actor.save(output/'student.npz')
        write_json(output/'parameters_received.json',params)
        result.update(bridge=bridge.stats,wall_s=time.monotonic()-t0,release_time_s=start,clock_offset_s=estimate.offset)
        result.update(estimator_study=study,selected_skill_mode=skill_mode,external_navigation_packets=0 if external is None else external.count,
                      commitment_events=[] if guard is None else guard.events,aborted=bool(guard and guard.aborted))
        if rows:
            x=np.array(rows);e=np.array(estimates)
            result.update(free_flight_s=float(x[-1,0]),position_estimate_rmse_m=float(np.sqrt(np.mean(np.sum((x[:,1:4]-e[:,3:6])**2,axis=1)))),
                          velocity_estimate_rmse_m_s=float(np.sqrt(np.mean(np.sum((x[:,8:11]-e[:,6:9])**2,axis=1)))),max_observation_age_s=float(e[:,2].max()),
                          final_displacement_m=float(np.linalg.norm(x[-1,1:4]-plant.origin)),max_body_rate_rad_s=float(np.linalg.norm(x[:,11:14],axis=1).max()))
        result['passed']=bool(result.get('completed') and (mode=='maneuver' or result.get('final_displacement_m',10)<.25))
        result['completion_gate_passed']=result['passed']
        if mode=='hover' and rows:
            settled=np.array(rows)[np.array(rows)[:,0]>2]
            if len(settled):
                result['settled_rate_rms_rad_s']=float(np.sqrt(np.mean(np.sum(settled[:,11:14]**2,axis=1))))
                result['passed']=bool(result['passed'] and result['settled_rate_rms_rad_s']<.3)
        result['scope']='actual AP firmware closed loop; airborne initialization fixture, synthetic sensors; no hardware validation'
        if mode=='maneuver':
            result['passed']=False
            if rows:
                try:
                    from .evaluation import evaluate
                    adjudicated=evaluate(output,output/'evaluation')
                    result['contact_evaluation']=adjudicated
                    result['passed']=bool(result.get('completed') and adjudicated['passed'])
                    if guard and guard.aborted:
                        last=rows[-1];R=Rotation.from_quat(last[[5,6,7,4]]).as_matrix()
                        result['safe_abort']=bool(result.get('completed') and adjudicated['first_contact'] is None and np.linalg.norm(last[8:11])<.3 and R[2,2]>np.cos(np.deg2rad(10)) and np.linalg.norm(last[11:14])<.524 and adjudicated['min_altitude_m']>.5 and adjudicated['max_tilt_deg']<65 and set(adjudicated['failure_modes'])<={'TARGET_MISS','RECOVERY_TIMEOUT','NO_SEPARATION'})
                except Exception as exc:
                    result['evaluation_failure']=repr(exc)
                    (output/'evaluation_traceback.log').write_text(traceback.format_exc())
        write_json(output/'summary.json',result)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--mode',choices=['hover','rate_step','maneuver'],default='hover');p.add_argument('--observation',choices=['telemetry','truth','synthetic'],default='telemetry');p.add_argument('--instance',type=int,default=45);p.add_argument('--duration',type=float,default=8);p.add_argument('--parameters');p.add_argument('--speedup',type=float,default=2);p.add_argument('--study')
    p.add_argument('--action-layer',choices=['rate','attitude'],default='rate');p.add_argument('--skill-mode',choices=['aggressive','balanced','conservative'],default='balanced')
    p.add_argument('--autopilot',choices=['ardupilot','px4'],default='ardupilot')
    p.add_argument('--policy',choices=['student','teacher'],default='student')
    a=p.parse_args();r=run(a.output,a.mode,a.observation,a.instance,a.duration,a.parameters,a.speedup,a.action_layer,a.skill_mode,a.study,a.autopilot,a.policy);print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
