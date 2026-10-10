"""Privileged diagnostic interventions, strictly outside the policy boundary."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from ..model import VehicleEstimate
from ..distillation.deployment import VehicleObservation
from .synthetic import ErrorSensor


class TruthHistory:
    def __init__(self):self.states=deque(maxlen=2000)
    def append(self,s):self.states.append(s)
    def at(self,t):
        times=np.array([s.timestamp for s in self.states]);i=np.searchsorted(times,t)
        if i==0:return self.states[0]
        if i==len(times):return self.states[-1]
        a,b=self.states[i-1],self.states[i];u=(t-a.timestamp)/(b.timestamp-a.timestamp)
        R=Slerp([a.timestamp,b.timestamp],Rotation.from_matrix([a.rotation,b.rotation]))([t]).as_matrix()[0]
        return VehicleEstimate(a.position+u*(b.position-a.position),a.velocity+u*(b.velocity-a.velocity),R,a.omega_body+u*(b.omega_body-a.omega_body),t,'PRIVILEGED_TIMESTAMP_ALIGNED_TRUTH')
    def correct(self,obs,component):
        if component=='none':return obs
        s=obs.estimate;truth=self.at(s.timestamp)
        return VehicleObservation(VehicleEstimate(truth.position if component in ('position','all') else s.position,
            truth.velocity if component in ('velocity','all') else s.velocity,
            truth.rotation if component in ('attitude','all') else s.rotation,
            truth.omega_body if component in ('attitude','all') else s.omega_body,s.timestamp,'PRIVILEGED_COMPONENT_'+component),obs.velocity_covariance_m2_s2)


class ExternalNavigationSensor:
    """Synthetic VIO-equivalent messages to actual AP EKF; not a vision system."""
    def __init__(self,config,origin):self.sensor=ErrorSensor(config);self.origin=np.array(origin);self.last=-np.inf;self.count=0
    def send(self,connection,truth):
        s,_=self.sensor.update(truth)
        if s.timestamp<=self.last:return
        self.last=s.timestamp;B=np.diag([1.,-1.,-1.]);p=B@(s.position-self.origin);v=B@s.velocity
        rpy=Rotation.from_matrix(B@s.rotation@B).as_euler('xyz')
        connection.mav.vision_position_estimate_send(int(s.timestamp*1e6),*p,*rpy)
        connection.mav.vision_speed_estimate_send(int(s.timestamp*1e6),*v)
        self.count+=1
