"""Physical RateThrust/attitude adapters with explicitly calibrated force mapping."""
import numpy as np
from scipy.spatial.transform import Rotation
from .runtime import mavlink

class ForceMapping:
    def __init__(self,maximum_collective_N):
        if not np.isfinite(maximum_collective_N) or maximum_collective_N<=0:raise ValueError('maximum force must be positive')
        self.maximum=float(maximum_collective_N)
    def __call__(self,force):
        if not np.isfinite(force) or not 0<=force<=self.maximum:raise ValueError('force outside calibrated range')
        return float(force/self.maximum)

class PX4Adapter:
    def __init__(self,force_to_throttle,target_system=1):self.mapping=force_to_throttle;self.system=target_system
    def packet(self,action,timestamp_s,rotation_nwu_flu=None):
        rates=np.asarray(action.body_rates_flu_rad_s,float)
        if rates.shape!=(3,) or not np.isfinite(rates).all() or not np.isfinite(timestamp_s) or timestamp_s<0:raise ValueError('invalid action/time')
        flip=np.diag([1.,-1.,-1.]);q=[1.,0.,0.,0.];mask=128
        if rotation_nwu_flu is not None:
            r=np.asarray(rotation_nwu_flu,float)
            if r.shape!=(3,3) or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1):raise ValueError('invalid attitude')
            q=Rotation.from_matrix(flip@r@flip).as_quat()[[3,0,1,2]].tolist()
            # Separate attitude arm; PX4 does not use roll/pitch feedforward here.
            mask=7
        rates=flip@rates
        return dict(time_boot_ms=round(timestamp_s*1000)%2**32,target_system=self.system,target_component=1,type_mask=mask,q=q,
                    body_roll_rate=float(rates[0]),body_pitch_rate=float(rates[1]),body_yaw_rate=float(rates[2]),thrust=self.mapping(action.collective_force_N))


class PX4PolicyAdapter:
    """Connect a timestamped policy action to PX4's attitude-target sender.

    The policy still receives only its registered estimate. This adapter owns
    the explicit force calibration and FLU-to-FRD packet conversion; it does
    not read MuJoCo truth or alter the estimator contract.
    """
    def __init__(self, policy, packet_adapter):
        if not hasattr(policy, 'infer') or not isinstance(packet_adapter, PX4Adapter):
            raise TypeError('policy.infer and PX4Adapter are required')
        self.policy, self.adapter = policy, packet_adapter

    def packet(self, observation, timestamp_s):
        action=self.policy.infer(observation,timestamp_s)
        return action, self.adapter.packet(action,timestamp_s)

    def send(self, runtime, observation, timestamp_s):
        action,fields=self.packet(observation,timestamp_s)
        runtime.mav.set_attitude_target_send(**fields)
        return action
