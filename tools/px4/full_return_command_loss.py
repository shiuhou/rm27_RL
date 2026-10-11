"""Complete image mission followed by upstream command loss over the dock.

The interruption is admitted only after fresh assigned image alignment. PX4
must invoke its onboard LAND failsafe; no external LAND is injected afterward.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from rm27.autopilot.px4 import image_mission
from rm27.autopilot.px4.runtime import mavlink
from rm27.autopilot.px4.return_fault_batch import frozen_inputs,ROOT,digest
from rm27.experiments import image_conditions, vio_initial_admission
from .ground_rest import prepare as prepare_ground_rest
from .initial_vio_admission import StableInitialAdmission


def eligible(tag,telemetry,phase,now):
    if phase not in ('RETURN','ALIGN') or not tag or tag.get('valid') is not True or tag.get('assigned_tag_id')!=21:
        return False
    if not 0<=now-tag.get('source_time_s',-100)<=.2:
        return False
    if any(not 0<=now-telemetry.get(k,-100)<=.2 for k in
           ['local_time','attitude_time','estimator_time']):
        return False
    # This profile requests5 Hz heartbeat. Include one control-tick margin;
    # a strict0.2 s bound resets stability just before the next reception.
    if not 0<=now-telemetry.get('heartbeat_time',-100)<=.3:
        return False
    xy=np.asarray(tag.get('position_relative_slot_nwu_xy_m',[]),float)
    velocity=np.asarray(telemetry.get('local_velocity',[]),float)
    return bool(xy.shape==(2,) and velocity.shape==(3,) and np.isfinite(np.r_[xy,velocity]).all() and
        np.linalg.norm(xy)<=.003 and np.linalg.norm(velocity)<=.03 and telemetry.get('armed') and
        telemetry.get('mode')=='OFFBOARD' and telemetry.get('estimator_flags',0)&mavlink.ESTIMATOR_POS_HORIZ_REL)


def monitor(runtime):
    start=float(runtime.plant.data.time);wall=time.monotonic()
    land_seen=False;disarmed=None;last=None;contacts=[]
    with (runtime.output/'command_loss_frames.jsonl').open('x',buffering=1) as stream:
        while time.monotonic()-wall<90:
            if runtime.plant.data.time-start>35:
                raise TimeoutError('Continued command-loss physics deadline')
            row=runtime.step();now=float(runtime.plant.data.time)
            runtime.receive()
            telemetry=runtime.telemetry
            land_seen |= telemetry.get('armed',False) and telemetry.get('mode')=='LAND'
            actual_contacts=runtime.plant.contacts();contacts.extend(actual_contacts)
            row.update(telemetry=telemetry.copy(),contacts=actual_contacts,
                actual_motor_thrust_N=runtime.plant.data.actuator_force[runtime.plant.motors].tolist())
            stream.write(json.dumps(row,allow_nan=False)+'\n');last=row
            if not telemetry.get('armed'):
                if not land_seen or telemetry.get('landed_state')!=mavlink.MAV_LANDED_STATE_ON_GROUND or now-telemetry.get('landed_time',-100)>.3:
                    raise RuntimeError('No actual failsafe landed/disarmed sequence')
                disarmed=now if disarmed is None else disarmed
                if now-disarmed>=1:
                    break
        else:
            raise TimeoutError('Continued command-loss wall deadline')
    position=np.asarray(last['truth']['position_nwu'])-runtime.plant.origin
    events=runtime.loss_events
    checks=dict(onboard_failsafe_seen=any('failsafe' in str(e).lower() for e in events),
        normal_land_observed=bool(land_seen),automatic_disarm=disarmed is not None,
        physical_rest=bool(last['contacts'] and np.linalg.norm(last['truth']['velocity_nwu'])<=.01),
        bounded_rest=bool(np.linalg.norm(position[:2])<=.025 and abs(position[2]-.02)<=.005),
        no_unintended_contact=all('rm27_dock_collision_bottom' in c['geoms'] for c in contacts),
        stable_contacts=all(c['force_N']<=10 and c['distance_m']>=-.005 for c in contacts),
        stopped_motors=max(last['actual_motor_thrust_N'])<1e-5)
    return dict(passed=all(checks.values()),task_success=False,checks=checks,events=events,
        resting_horizontal_error_m=float(np.linalg.norm(position[:2])),duration_s=last['timestamp']-start)


def run(args):
    camera=json.loads(args.camera_config.read_text())
    hashes=frozen_inputs(args.scene,args.config)
    files=[args.audit,args.camera_config,Path(__file__),*[Path(__file__).with_name(n+'.py') for n in
        ['ground_rest','initial_vio_admission','causal_flight']]]
    if args.conditions:files.append(args.conditions)
    hashes.update({str(p.resolve()):digest(p) for p in files})
    manifest=dict(input_sha256=hashes,camera_seed=camera['seed'],launcher_source=Path(__file__).read_text(),
        additional_sources={str(p):p.read_text() for p in files if p.suffix=='.py'},
        conditions=str(args.conditions),causal_mission_inputs=args.causal_mission_inputs,
        scope='Complete-route return or final ALIGN above assigned dock; fresh image within3 mm and slow navigation for0.25 s. Arbitrary route interruption and invalid navigation remain unqualified')
    with args.output.with_name(args.output.name+'-command-loss-manifest.json').open('x') as stream:
        stream.write(json.dumps(manifest,indent=2)+'\n')
    terminal={};original=image_mission.run;selected_run=original
    original_admission=vio_initial_admission.initial_consistent
    if args.causal_mission_inputs:
        if not args.conditions or json.loads(args.conditions.read_text()).get('initial_vio_admission') is not True:
            raise ValueError('Causal command loss requires explicit stable initial admission')
        selected_run,_=prepare_ground_rest(args.output,retry_alignment=True,causal_inputs=True)
        vio_initial_admission.initial_consistent=StableInitialAdmission()
    def fault_run(*positional,**keywords):
        base=keywords['runtime_factory']
        class CommandLossRuntime(base):
            def __init__(self,*p,**k):
                super().__init__(*p,**k);self.loss=None;self.stable=None;self.blocked=0;self.loss_events=[]
            def receive(self):
                messages=super().receive()
                if self.loss is not None:
                    for message in messages:
                        if message.get_type() in ('COMMAND_ACK','STATUSTEXT'):
                            self.loss_events.append(dict(timestamp_s=float(self.plant.data.time),message=message.to_dict()))
                return messages
            def step(self,timeout=5.):
                row=super().step(timeout);now=float(self.plant.data.time)
                if self.loss is None:
                    if eligible(self.return_observation,self.telemetry,self.return_phase,now):
                        self.stable=now if self.stable is None else self.stable
                        if now-self.stable>=.25:
                            self.loss=now
                            (self.output/'command_loss_injection.json').write_text(json.dumps(dict(
                                source='fresh assigned image and telemetry, no truth',onset_s=now,
                                tag=self.return_observation,telemetry=self.telemetry),indent=2)+'\n')
                    else:
                        self.stable=None
                return row
            def position(self,*p,**k):
                if self.loss is not None:
                    self.blocked+=1
                    return
                return super().position(*p,**k)
            def __exit__(self,kind,error,traceback):
                if self.loss is not None and self.blocked and str(error)=='Unexpected mode/disarm':
                    try:
                        terminal.update(monitor(self))
                    except Exception as failure:
                        terminal.update(passed=False,task_success=False,failure=f'{type(failure).__name__}: {failure}')
                    terminal.update(trigger=str(error),blocked_commands=self.blocked,onset_s=self.loss,
                        gate_version='PX4_FULL_RETURN_COMMAND_LOSS_DEVELOPMENT_V2')
                    (self.output/'termination_summary.json').write_text(json.dumps(terminal,indent=2)+'\n')
                return super().__exit__(kind,error,traceback)
        keywords['runtime_factory']=CommandLossRuntime
        return selected_run(*positional,**keywords)
    image_mission.run=fault_run
    try:
        if args.conditions:
            image_conditions.execute(args.scene,args.audit,args.config,args.camera_config,args.conditions,
                args.output,instance=args.instance)
        else:
            image_mission.execute(args.scene,args.audit,args.output,configuration=args.config,
                camera_configuration=args.camera_config,instance=args.instance)
    finally:
        image_mission.run=original
        vio_initial_admission.initial_consistent=original_admission
    summary=json.loads((args.output/'summary.json').read_text())
    checks=dict(injected=(args.output/'command_loss_injection.json').is_file(),
        failed_task=summary.get('passed') is False,
        original_route=summary.get('image_route',{}).get('passed') is True,
        original_recovery=summary.get('independent_recovery_score',{}).get('passed') is True,
        physical_termination=terminal.get('passed') is True)
    result=dict(passed=all(checks.values()),task_success=False,checks=checks,termination=terminal,
        run_type='headless closed-loop SITL continued through upstream setpoint interruption',scope=manifest['scope'])
    (args.output/'command_loss_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    for flag in ['audit','config','camera-config','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--instance',type=int,default=47)
    parser.add_argument('--conditions',type=Path)
    parser.add_argument('--causal-mission-inputs',action='store_true')
    raise SystemExit(run(parser.parse_args()))
