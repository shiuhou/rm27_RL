"""Explicit post-LAND development using fresh landed/range/output evidence.

Velocity below the aiding floor cannot veto landed confirmation. Actual
physical velocity and motor stop are checked by the independent scorer.
"""
import inspect
from pathlib import Path

from rm27.autopilot.px4 import flight
from rm27.autopilot.px4.return_fault_batch import ROOT
from rm27.autopilot.px4.runtime import mavlink
from .causal_flight import prepare_source


def prepare(output, *, retry_alignment=False, causal_inputs=False):
    path=Path(output).with_name(Path(output).name+'-flight-runner.py').resolve()
    text=inspect.getsource(flight.run)
    old="""                    if tel.get('armed') or not settled or speed>.01 or resting_horizontal>.007 or abs(height-.02)>height_tolerance:raise RuntimeError('Disarmed resting gate failed')
                    if t-phase_start>=1 and np.max(np.abs(plant.data.actuator_force[plant.motors]))<1e-5:"""
    new="""                    ground_confirmed=bool(profile in ('mtf','vio') and landing_commitment and
                        not tel.get('armed') and settled and 0<=t-tel.get('landed_time',-100)<=.3 and
                        0<=t-tel.get('heartbeat_time',-100)<=.3 and np.max(np.abs(row['controls']))<1e-6)
                    if (tel.get('armed') or not settled or (not ground_confirmed and speed>.01) or
                            resting_horizontal>.007 or abs(height-.02)>height_tolerance):
                        raise RuntimeError('Disarmed resting gate failed')
                    if ground_confirmed:
                        summary['ground_rest_confirmation']=dict(source='fresh PX4 landed/range/heartbeat and stopped actuator output',
                            estimated_vertical_velocity_m_s=tel.get('local_velocity',[0,0,None])[2],
                            below_flow_minimum=True,physical_velocity_source='independent scorer only')
                    if t-phase_start>=1 and np.max(np.abs(row['controls']))<1e-6:"""
    if text.count(old)!=1:
        raise ValueError('Expected frozen post-LAND context changed')
    text=text.replace(old,new)
    if retry_alignment:
        old="""                                if not flags & mavlink.ESTIMATOR_POS_HORIZ_REL or t-tel.get('estimator_time',-100)>.3 or np.linalg.norm(local[:2])>.003:
                                    raise RuntimeError('No valid slot alignment at landing commitment')
                                if visual_guard and not visual_guard.aligned(t):
                                    raise RuntimeError('No fresh aligned AprilTag at landing commitment')"""
        new="""                                if not flags & mavlink.ESTIMATOR_POS_HORIZ_REL or t-tel.get('estimator_time',-100)>.3:
                                    raise RuntimeError('No valid navigation at landing commitment')
                                if np.linalg.norm(local[:2])>.003 or (visual_guard and not visual_guard.aligned(t)):
                                    if reacquire_count>=2:
                                        raise RuntimeError('AprilTag reacquisition attempts exhausted')
                                    reacquire_count+=1;summary['tag_reacquire_count']=reacquire_count
                                    event('landing_alignment_retry',source='fresh PX4 navigation and assigned image',attempt=reacquire_count)
                                    transition('ALIGN' if visual_guard and visual_guard.fresh(t) else 'REACQUIRE')
                                    continue"""
        if text.count(old)!=1:
            raise ValueError('Expected frozen landing alignment admission changed')
        text=text.replace(old,new)
    old="""            physical_pass=bool(plant.contacts() and np.linalg.norm(final_local[:2])<=.007 and abs(final_local[2]-.02)<=.005)"""
    new="""            physical_pass=bool(plant.contacts() and np.linalg.norm(final_local[:2])<=.007 and abs(final_local[2]-.02)<=.005 and
                np.linalg.norm(plant.qvel[:3])<=.01 and np.max(np.abs(plant.data.actuator_force[plant.motors]))<1e-5)"""
    if text.count(old)!=1:
        raise ValueError('Expected frozen physical rest scorer changed')
    text=text.replace(old,new)
    old="""                horizontal_error_m=float(np.linalg.norm(final_local[:2])),height_error_m=float(abs(final_local[2]-.02)))"""
    new="""                horizontal_error_m=float(np.linalg.norm(final_local[:2])),height_error_m=float(abs(final_local[2]-.02)),
                physical_speed_m_s=float(np.linalg.norm(plant.qvel[:3])),
                maximum_actual_motor_force_N=float(np.max(np.abs(plant.data.actuator_force[plant.motors]))))"""
    if text.count(old)!=1:
        raise ValueError('Expected frozen physical rest reporting context changed')
    text=text.replace(old,new)
    if causal_inputs:
        text=prepare_source(text)
    text=text.replace('source_root=Path(__file__).resolve().parents[3]',f'source_root=Path({str(ROOT)!r})')
    text=text.replace("Path(__file__).with_name('runtime.py')","source_root/'rm27/autopilot/px4/runtime.py'")
    with path.open('x') as stream:
        stream.write(text)
    namespace=dict(flight.__dict__,__file__=str(path))
    exec(compile(text,str(path),'exec'),namespace)
    return namespace['run'],path


def biased_runtime(base,bias):
    if not 0<bias<=.1:
        raise ValueError('Explicit bounded post-LAND telemetry velocity bias required')
    class BiasedRuntime(base):
        def receive(self):
            messages=super().receive()
            if (self.return_phase=='POST' and not self.telemetry.get('armed') and
                    self.telemetry.get('landed_state')==mavlink.MAV_LANDED_STATE_ON_GROUND and
                    any(m.get_type()=='LOCAL_POSITION_NED' for m in messages)):
                original=list(self.telemetry['local_velocity'])
                modified=list(original);modified[2]+=bias
                self.telemetry['post_land_velocity_before_bias']=original
                self.telemetry['post_land_vertical_bias_m_s']=bias
                self.telemetry['local_velocity']=modified
            return messages
    return BiasedRuntime
