"""Explicit post-LAND development using fresh landed/range/output evidence.

Velocity below the aiding floor cannot veto landed confirmation. Actual
physical velocity and motor stop are checked by the independent scorer.
"""
import inspect
from pathlib import Path

from rm27.autopilot.px4 import flight
from rm27.autopilot.px4.return_fault_batch import ROOT
from rm27.autopilot.px4.runtime import mavlink


def prepare(output, *, retry_alignment=False, causal_inputs=False):
    path=Path(output).with_name(Path(output).name+'-flight-runner.py').resolve()
    text=inspect.getsource(flight.run)
    policy=flight.FlightPolicy(causal_inputs=causal_inputs,ground_rest_confirmation=True,
                              retry_alignment=retry_alignment)
    text=text.replace('policy=LEGACY_POLICY',f'policy={policy!r}')
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
