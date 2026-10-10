"""Simulation sensor producer. The consumer only receives VehicleEstimate.

Error draws belong to the producer/evaluator, never to the policy. Fixed biases
are in world NWU; attitude bias is body FLU. Acquisition timestamps are retained.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from ..model import VehicleEstimate
from ..distillation.observation import SyntheticEstimator
from ..distillation.deployment import VehicleObservation


class ErrorSensor:
    def __init__(self,config):
        self.config=dict(config);self.filtered=None;self.previous_time=None
        keys=('seed','position_std','velocity_std','attitude_std_deg','rate_std','delay_s','dropout','update_hz','jitter_s')
        self.sensor=SyntheticEstimator(**{k:v for k,v in config.items() if k in keys})

    def update(self,state):
        c=self.config
        p=state.position+np.array(c.get('position_bias_m',[0.,0.,0.]))
        v=state.velocity+np.array(c.get('velocity_bias_m_s',[0.,0.,0.]))
        tau=c.get('velocity_filter_tau_s',0.)
        if self.filtered is None:self.filtered=v.copy()
        elif tau and state.timestamp>self.previous_time:
            self.filtered+=(1-np.exp(-(state.timestamp-self.previous_time)/tau))*(v-self.filtered)
        else:self.filtered=v.copy()
        self.previous_time=state.timestamp
        R=state.rotation@Rotation.from_rotvec(np.deg2rad(c.get('attitude_bias_deg',[0.,0.,0.]))).as_matrix()
        sensed=VehicleEstimate(p,self.filtered.copy(),R,state.omega_body.copy(),state.timestamp,'SYNTHETIC_ERROR_PRODUCER')
        return self.sensor.update(sensed)

    def observation(self,state):
        s,_=self.update(state)
        return VehicleObservation(s,np.eye(3)*max(.02,self.config.get('velocity_std',0.))**2)
