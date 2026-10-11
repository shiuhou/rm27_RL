"""Development VIO transport-failure LAND inside the known takeoff footprint."""
import json
import struct
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation

from rm27.autopilot.px4.runtime import Runtime, mavlink
from rm27.autopilot.px4.return_termination import continue_landing


def admission(telemetry, phase, now):
    if phase not in ('TAKEOFF','HOVER') or telemetry.get('mode')!='OFFBOARD' or not telemetry.get('armed'):
        return False
    if any(not 0<=now-telemetry.get(k,-100)<=.3 for k in
           ('local_time','attitude_time','estimator_time','heartbeat_time','distance_time')):
        return False
    if not .02<=telemetry.get('distance_m',-1)<=.8:
        return False
    flags = telemetry.get('estimator_flags',0)
    if flags & (mavlink.ESTIMATOR_POS_HORIZ_REL | mavlink.ESTIMATOR_VELOCITY_HORIZ) != (
            mavlink.ESTIMATOR_POS_HORIZ_REL | mavlink.ESTIMATOR_VELOCITY_HORIZ):
        return False
    p = np.asarray(telemetry.get('raw_local_position_ned',[]),float)
    v = np.asarray(telemetry.get('local_velocity',[]),float)
    q = np.asarray(telemetry.get('quaternion',[]),float)
    if p.shape!=(3,) or v.shape!=(3,) or q.shape!=(4,) or not np.isfinite(np.r_[p,v,q]).all():
        return False
    if abs(np.linalg.norm(q)-1)>.02:
        return False
    return bool(np.linalg.norm(p[:2])<=.02 and .02<=-p[2]<=.8 and
        np.linalg.norm(v[:2])<=.08 and abs(v[2])<=1.5 and
        Rotation.from_quat(q[[1,2,3,0]]).as_matrix()[2,2]>=np.cos(np.deg2rad(10)))


def range_fallback(runtime):
    """Reconfigure this isolated SITL to the tested fixed-dock range reference.

    Admission is confined to the known initial flat dock. This profile is
    unsuitable over varying terrain and is never sent to hardware.
    """
    settings = dict(EKF2_EV_CTRL=0,EKF2_HGT_REF=2,EKF2_RNG_CTRL=2,EKF2_BARO_CTRL=0)
    runtime.mode('LAND')
    for name,value in settings.items():
        runtime.parameters.pop(name,None)
        union = struct.unpack('<f',struct.pack('<i',value))[0]
        runtime.mav.param_set_send(runtime.system,1,name.encode(),union,mavlink.MAV_PARAM_TYPE_INT32)
    deadline = time.monotonic()+10
    contacts = []
    with (runtime.output/'native_fallback_frames.jsonl').open('x',buffering=1) as stream:
        while any(runtime.parameters.get(k)!=v for k,v in settings.items()):
            if time.monotonic()>deadline:
                raise TimeoutError('SITL range-reference parameter readback timed out')
            row = runtime.step()
            runtime.receive()
            physical_contacts = runtime.plant.contacts()
            contacts.extend(physical_contacts)
            row.update(telemetry=runtime.telemetry.copy(),contacts=physical_contacts)
            stream.write(json.dumps(row,allow_nan=False)+'\n')
    result = dict(verified=True,requested=settings,readback={k:runtime.parameters[k] for k in settings},
        early_contacts_safe=all('rm27_dock_collision_bottom' in c['geoms'] and
            c['force_N']<=10 and c['distance_m']>=-.005 for c in contacts),
        scope='Local SITL only; initial flat-dock range fallback; no hardware operation')
    (runtime.output/'native_fallback_parameters.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def guarded_runtime(base, *, injected_delay_s=None):
    if injected_delay_s is not None and not .1<=injected_delay_s<=.6:
        raise ValueError('Explicit bounded takeoff native transport fault required')
    class GuardRuntime(base):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.native_failed = False
            self.native_fault_start = None
            self.native_fault_injected = False

        def step(self,timeout=5.):
            if self.native_failed:
                # Keep real PX4 sensors, motor physics and telemetry alive;
                # stop reading the failed native transport. No synthetic EV.
                return Runtime.step(self,timeout)
            if injected_delay_s is not None:
                now = float(self.plant.data.time)
                if self.return_phase=='TAKEOFF' and self.native_fault_start is None:
                    self.native_fault_start = now+injected_delay_s
                if self.native_fault_start is not None and now>=self.native_fault_start and not self.native_fault_injected:
                    self.native_fault_injected = True
                    result = subprocess.run(['docker','stop','--time','1',self.vio.container],
                                            capture_output=True,text=True,timeout=10)
                    record = dict(timestamp_s=now,phase=self.return_phase,delay_s=injected_delay_s,
                                  transport='current native VIO container',returncode=result.returncode)
                    (self.output/'native_fault_injection.json').write_text(json.dumps(record,indent=2)+'\n')
                    if result.returncode!=0:
                        raise RuntimeError('Declared native transport fault injection failed')
            return super().step(timeout)

        def __exit__(self,kind,error,traceback):
            transport_error = isinstance(error,(BrokenPipeError,ConnectionResetError)) or (
                isinstance(error,RuntimeError) and 'native VIO transport disconnected' in str(error))
            if transport_error:
                now = float(self.plant.data.time)
                phase = self.return_phase
                result = dict(passed=False,task_success=False,trigger=str(error),phase=phase,
                    admission_telemetry=self.telemetry.copy(),onset_s=now,
                    admitted=admission(self.telemetry,phase,now),
                    gate_version='PX4_NATIVE_FAILURE_TAKEOFF_FOOTPRINT_TERMINATION_DEVELOPMENT_V1',
                    run_type='same headless closed-loop SITL continued after native VIO transport failure')
                try:
                    if result['admitted']:
                        self.native_failed = True
                        fallback = range_fallback(self)
                        result.update(continue_landing(self,dict(termination_wall_deadline_s=90,
                            termination_physics_deadline_s=35,safe_rest_horizontal_m=.025)))
                        result['fallback_parameters'] = fallback
                        result['passed'] &= fallback['early_contacts_safe']
                except Exception as failure:
                    result.update(passed=False,failure=f'{type(failure).__name__}: {failure}')
                result['scope'] = ('Known initial dock footprint only, fresh valid flow navigation, normal LAND; '
                    'does not qualify invalid navigation or native failure elsewhere on the route')
                (self.output/'termination_summary.json').write_text(json.dumps(result,indent=2)+'\n')
            return super().__exit__(kind,error,traceback)
    return GuardRuntime
