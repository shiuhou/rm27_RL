"""Continued sensor-only PX4 simulation after a real Offboard command loss.

No landing command is injected after loss. PX4's configured onboard failsafe
must land and automatically disarm; independent physics checks termination.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from ..autopilot.px4.flight import run
from ..autopilot.px4.runtime import Runtime,mavlink

ROOT=Path(__file__).resolve().parents[2]


def execute(scene,configuration,output,*,instance=47):
    output=Path(output).resolve();settings=json.loads(Path(configuration).read_text())
    terminal={};source=Path(__file__).read_bytes()

    class LossRuntime(Runtime):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs);self.phase=None;self.onset=None;self.blocked=0
            self.sensor.rng=np.random.default_rng(settings['sensor_seed'])
        @property
        def return_phase(self):return self.phase
        @return_phase.setter
        def return_phase(self,value):
            self.phase=value
            if value==settings['phase'] and self.onset is None:
                self.onset=float(self.plant.data.time)+settings['delay_s']
        def position(self,*args,**kwargs):
            if self.onset is not None and self.plant.data.time>=self.onset:
                self.blocked+=1;return
            return super().position(*args,**kwargs)
        def __exit__(self,kind,exc,traceback):
            try:
                if kind is not None and self.blocked and str(exc)=='Unexpected mode/disarm':
                    self.monitor_failsafe()
                else:terminal.update(passed=False,task_success=False,trigger=str(exc),blocked=self.blocked)
            except Exception as error:
                terminal.update(passed=False,task_success=False,failure=f'{type(error).__name__}: {error}')
            finally:
                (self.output/'termination_summary.json').write_text(json.dumps(terminal,indent=2)+'\n')
                super().__exit__(kind,exc,traceback)
        def monitor_failsafe(self):
            start=float(self.plant.data.time);wall=time.monotonic();land_seen=False;disarmed=None
            last=None;contacts=[];events=[]
            with (self.output/'termination_frames.jsonl').open('x',buffering=1) as stream:
                while time.monotonic()-wall<90:
                    if float(self.plant.data.time)-start>35:raise TimeoutError('failsafe physical deadline')
                    row=self.step();now=float(self.plant.data.time)
                    for message in self.receive():
                        if message.get_type()=='STATUSTEXT':events.append(dict(timestamp_s=now,text=message.text))
                    tel=self.telemetry
                    land_seen |= tel.get('mode')=='LAND' and tel.get('armed',False)
                    actual_contacts=self.plant.contacts();contacts.extend(actual_contacts)
                    row.update(telemetry=tel.copy(),contacts=actual_contacts,
                        actual_motor_thrust_N=self.plant.data.actuator_force[self.plant.motors].tolist())
                    stream.write(json.dumps(row,allow_nan=False)+'\n');last=row
                    if not tel.get('armed'):
                        if not land_seen or tel.get('landed_state')!=mavlink.MAV_LANDED_STATE_ON_GROUND or now-tel.get('landed_time',-100)>.3:
                            raise RuntimeError('no actual failsafe landed/disarmed sequence')
                        disarmed=now if disarmed is None else disarmed
                        if now-disarmed>=1:break
                else:raise TimeoutError('failsafe wall deadline')
            position=np.asarray(last['truth']['position_nwu'])-self.plant.origin
            checks=dict(failsafe_land_observed=bool(land_seen),automatic_disarm=disarmed is not None,
                physical_rest=bool(last['contacts'] and np.linalg.norm(last['truth']['velocity_nwu'])<=.01),
                bounded_rest=np.linalg.norm(position[:2])<=settings['safe_rest_horizontal_m'] and abs(position[2]-.02)<=.005,
                no_unintended_contact=all('rm27_dock_collision_bottom' in c['geoms'] for c in contacts),
                stable_contacts=all(c['force_N']<=10 and c['distance_m']>=-.005 for c in contacts),
                stopped_motors=max(last['actual_motor_thrust_N'])<1e-5)
            terminal.update(passed=all(checks.values()),task_success=False,checks={k:bool(v) for k,v in checks.items()},
                blocked_commands=self.blocked,onset_s=self.onset,events=events,
                resting_horizontal_error_m=float(np.linalg.norm(position[:2])),duration_s=last['timestamp']-start,
                scope='Sensor-only dock hover Offboard loss; no camera, complete route or invalid-navigation qualification')

    run(scene,output,instance,dynamics=ROOT/'configs/dynamics_150g.json',profile='mtf',
        speedup=3.,runtime_factory=LossRuntime)
    summary=json.loads((output/'summary.json').read_text())
    terminal.update(original_task_passed=summary.get('passed') is True,configuration=settings,
        gate_version=settings['gate_version'],runner_sha256=hashlib.sha256(source).hexdigest(),
        run_type='headless closed-loop SITL; same physics continued after command interruption')
    (output/'termination_runner.py').write_bytes(source)
    (output/'termination_summary.json').write_text(json.dumps(terminal,indent=2)+'\n')
    print(json.dumps(terminal,indent=2));return 0 if terminal.get('passed') is True else 1


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--instance',type=int,default=47)
    a=p.parse_args();return execute(a.scene,a.config,a.output,instance=a.instance)


if __name__=='__main__':raise SystemExit(main())
