"""Declared image/IMU and physical disturbances for the complete PX4 mission.

The mission still consumes rendered pixels and PX4 estimates. Perturbations
are applied at the sensor/plant boundary, with their source times recorded.
"""
import argparse
from collections import deque
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import MethodType

import numpy as np

from ..autopilot.px4 import image_mission
from ..autopilot.px4.return_faults import ImageFault,FaultCamera,assess
from .degraded_return import PermanentBlackout
from .vio_initial_admission import InitialContract
from .fast_bearing import detect as fast_bearing


class ConditionCamera:
    def __init__(self,camera,runtime,settings,seed,*,primary):
        self.camera,self.runtime,self.settings=camera,runtime,settings
        self.primary=primary;self.rng=np.random.default_rng(seed)
        self.queue=deque();self.records=[];self.drop_start=None
        delay=settings.get('primary_delay_frames',0) if primary else 0
        if type(delay) is not int or not 0<=delay<=3:raise ValueError('camera delay must be0..3 acquisition periods')
        self.delay=delay

    def capture(self,data,timestamp_us):
        frame=self.camera.capture(data,timestamp_us)
        role='primary' if self.primary else 'downward'
        brightness=self.settings.get(role+'_brightness',1.)
        noise=self.settings.get(role+'_noise_std_px',0.)
        if not np.isfinite([brightness,noise]).all() or not .5<=brightness<=1.5 or not 0<=noise<=8:
            raise ValueError('declared bounded camera brightness/noise required')
        if brightness!=1. or noise:
            pixels=np.clip(frame.image_rgb.astype(float)*brightness+
                self.rng.normal(0.,noise,frame.image_rgb.shape),0,255).astype(np.uint8)
            frame=replace(frame,image_rgb=pixels)
        dropout=self.settings.get('primary_dropout') if self.primary else None
        now=float(data.time)
        if dropout and self.drop_start is None and self.runtime.return_phase==dropout['phase']:
            self.drop_start=now+dropout['delay_s']
        dropped=bool(self.drop_start is not None and self.drop_start<=now<self.drop_start+dropout['duration_s'])
        if not dropped:self.queue.append(frame)
        delivered=self.queue.popleft() if not dropped and len(self.queue)>self.delay else None
        self.records.append(dict(camera=role,received_time_us=timestamp_us,
            source_time_us=delivered.timestamp_us if delivered else None,dropped=dropped,
            pixels_sha256=hashlib.sha256(delivered.image_rgb.tobytes()).hexdigest() if delivered else None))
        return delivered

    def close(self):self.camera.close()


class SensorCondition:
    """A phase-timed sensor fault, applied before the PX4 sensor packet."""
    def __init__(self,runtime,configuration):
        self.runtime=runtime;self.configuration=configuration;self.start=None;self.records=[]
        if configuration['kind'] not in ('flow_loss','range_jump','heading_error','low_thrust'):
            raise ValueError('supported physical sensor/actuator fault required')
        if not 0<=configuration['delay_s'] or not 0<configuration['duration_s']<=2.:
            raise ValueError('bounded physical fault schedule required')

    def active(self):
        now=float(self.runtime.plant.data.time);c=self.configuration
        if self.start is None and self.runtime.return_phase==c['phase']:self.start=now+c['delay_s']
        return bool(self.start is not None and self.start<=now<self.start+c['duration_s'])

    def transform(self,message):
        if message is None or not self.active():return message
        c=self.configuration;kind=c['kind'];changed=False
        if kind=='flow_loss' and message.get_type()=='HIL_OPTICAL_FLOW':
            message.quality=0;changed=True
        elif kind=='range_jump' and message.get_type()=='DISTANCE_SENSOR':
            message.current_distance=int(np.clip(message.current_distance+round(c['offset_m']*100),0,65535));changed=True
        elif kind=='heading_error' and message.get_type()=='HIL_SENSOR':
            angle=np.deg2rad(c['offset_deg']);x,y=message.xmag,message.ymag
            message.xmag=x*np.cos(angle)-y*np.sin(angle);message.ymag=x*np.sin(angle)+y*np.cos(angle);changed=True
        if changed:self.records.append(dict(timestamp_s=float(self.runtime.plant.data.time),kind=kind,
            phase=self.runtime.return_phase,message=message.to_dict()))
        return message


def execute(scene,audit,mission,camera,configuration,output,*,instance=46,
            flight_runner=None,admission=None,view=False,speedup=2.):
    output=Path(output).resolve();settings=json.loads(Path(configuration).read_text())
    source=Path(__file__).read_bytes()
    original=flight_runner if flight_runner is not None else image_mission.run
    cameras=[];fault=None;sensor_fault=None
    camera_settings=json.loads(Path(camera).read_text());factor=settings.get('imu_noise_factor',1.)
    if not np.isfinite(factor) or not .5<=factor<=2.:raise ValueError('bounded IMU noise multiplier required')
    for key in ('gyroscope_noise_std_rad_s','accelerometer_noise_std_m_s2'):
        camera_settings[key]*=factor
    effective_camera=output.with_name(output.name+'-effective-camera.json')
    with effective_camera.open('x') as stream:stream.write(json.dumps(camera_settings,indent=2)+'\n')
    wind=np.asarray(settings.get('constant_force_world_N',[0.,0.,0.]),float)
    if wind.shape!=(3,) or not np.isfinite(wind).all() or np.linalg.norm(wind)>.02:
        raise ValueError('explicit bounded simulation disturbance force required')
    # The currently frozen nominal batch retains its original publisher.
    # For delay development, preserve a separate exact publisher copy with
    # monotonicity checked per sensor: an old camera can arrive after new IMU.
    root=Path(__file__).resolve().parents[2]
    player_path=root/'tools/openvins/live_simulation_ros2.py'
    player=player_path.read_text()
    old="""                        if type(ns) is not int or ns < previous_ns:
                            raise ValueError('sensor time reversed')
                        previous_ns = ns"""
    if old not in player:raise ValueError('expected frozen publisher timestamp guard missing')
    player=player.replace('from transport import PacketDecoder, encode_packet',
        'from transport import PacketDecoder, encode_packet\nfrom sensor_clock import InputClock')
    player=player.replace('previous_ns = -1','clocks = InputClock()').replace(old,
        "                        clocks.observe(row['type'], ns)")
    player_copy=output.with_name(output.name+'-native-player.py')
    player_copy.write_text(player)
    clock_path=Path(__file__).with_name('sensor_clock.py').resolve()
    def run_conditions(*args,**kwargs):
        base=kwargs['runtime_factory']
        class ConditionRuntime(base):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                self.vio_player=player_copy
                self.vio_player_mounts=((clock_path,'/sensor_clock.py'),)

            def __enter__(self):
                nonlocal fault,sensor_fault
                if settings.get('sensor_fault'):
                    sensor_fault=SensorCondition(self,settings['sensor_fault'])
                    for name in ('hil_sensor','distance_sensor','optical_flow'):
                        original_sensor=getattr(self.sensor,name)
                        def changed_sensor(original_sensor=original_sensor):
                            return sensor_fault.transform(original_sensor())
                        setattr(self.sensor,name,changed_sensor)
                result=super().__enter__()
                self.admission_stream=None
                if settings.get('efficient_bearing'):
                    self.vio.detector.detect=MethodType(fast_bearing,self.vio.detector)
                if settings.get('initial_vio_admission'):
                    self.admission_stream=(self.output/'initial_vio_admission.jsonl').open('x',buffering=1)
                    contract=InitialContract(self.vio.contract,self.vio,self.admission_stream,admission=admission)
                    self.vio.contract=contract
                    original_observe=self.vio.observe_telemetry
                    def observe(telemetry,now):
                        original_observe(telemetry,now)
                        if self.vio.history and 'local_velocity' in telemetry:
                            time,position,q=self.vio.history[-1]
                            contract.history.append((time,position,q,telemetry['local_velocity']))
                            contract.history=contract.history[-500:]
                    self.vio.observe_telemetry=observe
                primary=ConditionCamera(self.vio.camera,self,settings,camera_settings['seed']+1_000_000,primary=True)
                self.vio.camera=primary;cameras.append(primary)
                down=ConditionCamera(self.return_camera.camera,self,settings,camera_settings['seed']+2_000_000,primary=False)
                self.return_camera.camera=down;cameras.append(down)
                config=settings.get('return_fault')
                if config:
                    if config['kind']=='permanent_missing':
                        fault=PermanentBlackout(down,self,dict(fault_phase=config['phase'],fault_delay_s=config['delay_s']))
                        self.return_camera.camera=fault
                    else:
                        fault=ImageFault(config);self.return_camera.camera=FaultCamera(down,self,fault)
                return result

            def advance(self,controls):
                force_active=self.telemetry.get('armed') and (not settings.get('force_phase') or self.return_phase==settings['force_phase'])
                self.applied_force=wind if force_active else np.zeros(3)
                self.plant.data.xfrc_applied[self.plant.body,:3]=self.applied_force
                if sensor_fault and sensor_fault.configuration['kind']=='low_thrust' and sensor_fault.active():
                    factor=sensor_fault.configuration['fraction']
                    if not np.isfinite(factor) or not .8<=factor<=1.:raise ValueError('bounded motor thrust fault required')
                    controls=np.asarray(controls)*factor
                    sensor_fault.records.append(dict(timestamp_s=float(self.plant.data.time),kind='low_thrust',
                        phase=self.return_phase,fraction=factor))
                self.applied_controls=np.asarray(controls).copy()
                return super().advance(controls)

            def step(self,timeout=5.):
                row=super().step(timeout)
                row['applied_force_world_N']=self.applied_force.tolist()
                row['commanded_controls']=row['controls']
                row['controls']=self.applied_controls.tolist()
                return row

            def close(self):
                stream=getattr(self,'admission_stream',None)
                if stream:stream.close();self.admission_stream=None
                return super().close()
        kwargs['runtime_factory']=ConditionRuntime
        return original(*args,**kwargs)

    image_mission.execute(scene,audit,output,configuration=mission,
        camera_configuration=effective_camera,instance=instance,flight_runner=run_conditions,
        view=view,speedup=speedup)
    summary=json.loads((output/'summary.json').read_text());records=fault.records if fault else []
    if settings.get('return_fault',{}).get('kind')=='permanent_missing':
        term=summary.get('fault_termination',{})
        passed=bool(records and summary.get('passed') is False and term.get('passed') is True and
            term.get('task_success') is False and summary.get('image_route',{}).get('passed') is True and
            summary.get('independent_recovery_score',{}).get('passed') is True)
        result=dict(passed=passed,task_success=False,safe_termination=term)
    elif fault:
        observations=[json.loads(line) for line in (output/'return_camera/observations.jsonl').open()]
        result=assess(summary,observations,records)
    else:result=dict(passed=summary.get('passed') is True,task_success=summary.get('passed') is True)
    result.update(configuration=settings,runner_sha256=hashlib.sha256(source).hexdigest(),
        native_player_sha256=hashlib.sha256(player_copy.read_bytes()).hexdigest(),
        native_clock_sha256=hashlib.sha256(clock_path.read_bytes()).hexdigest(),
        camera_seed=camera_settings['seed'],
        run_type=('live-view' if view else 'headless')+' closed-loop SITL complete image/VIO mission',
        scope='Declared simulated disturbances; not measured hardware noise, wind or camera calibration')
    (output/'condition_runner.py').write_bytes(source)
    (output/'condition_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'camera_delivery.jsonl').write_text(''.join(json.dumps(r)+'\n' for c in cameras for r in c.records))
    (output/'fault_injections.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    if sensor_fault:
        (output/'sensor_fault_injections.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in sensor_fault.records))
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--camera-config',type=Path,required=True);p.add_argument('--conditions',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--instance',type=int,default=46)
    a=p.parse_args();return execute(a.scene,a.audit,a.config,a.camera_config,a.conditions,a.output,instance=a.instance)


if __name__=='__main__':raise SystemExit(main())
