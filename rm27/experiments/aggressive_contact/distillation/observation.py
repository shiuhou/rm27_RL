"""Causal estimator boundary. This module never imports MuJoCo or contact truth."""
from dataclasses import dataclass
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from ..model import VehicleEstimate


@dataclass
class ActuatorEstimate:
    forces_N: np.ndarray
    specific_thrust_ratio: float


class SyntheticEstimator:
    def __init__(self, seed=0, position_std=0.,velocity_std=0.,attitude_std_deg=0.,rate_std=0.,delay_s=0.,dropout=0.,update_hz=None,jitter_s=0.):
        self.rng=np.random.default_rng(seed);self.params=(position_std,velocity_std,attitude_std_deg,rate_std)
        self.delay=delay_s;self.dropout=dropout;self.queue=deque();self.last=None
        self.period=0. if update_hz is None else 1./update_hz
        self.next_sample=-np.inf;self.jitter=jitter_s
    def update(self, state):
        p,v,a,w=self.params
        due=state.timestamp>=self.next_sample-1e-9
        if due:self.next_sample=state.timestamp+self.period
        if due and (self.last is None or self.rng.random()>=self.dropout):
            self.queue.append(VehicleEstimate(state.position+self.rng.normal(0,p,3),state.velocity+self.rng.normal(0,v,3),
                state.rotation@Rotation.from_rotvec(self.rng.normal(0,np.deg2rad(a),3)).as_matrix(),
                state.omega_body+self.rng.normal(0,w,3),state.timestamp,'SYNTHETIC_ESTIMATOR'))
        delay=max(0.,self.delay+(self.rng.uniform(-self.jitter,self.jitter) if self.jitter else 0.))
        while len(self.queue)>1 and self.queue[1].timestamp<=state.timestamp-delay+1e-9:self.last=self.queue.popleft()
        if self.queue and self.queue[0].timestamp<=state.timestamp-delay+1e-9:self.last=self.queue[0]
        if self.last is None:self.last=self.queue[0]
        # Age is separate from control clock. Values retain their acquisition timestamp.
        return self.last,max(0.,state.timestamp-self.last.timestamp)


class CausalState:
    def __init__(self,cfg,info,target,initial,window_s=0.):
        self.cfg,self.info,self.target=cfg,info,target
        self.motor=np.full(4,info['mass_kg']*9.81/4);self.previous_motor=self.motor.copy()
        self.window_s=window_s;self.samples=deque()
        self.impulse_threshold=.12
        self.ratio=1.;self.previous=None;self.contact_time=None;self.anchor=None;self.innovation=0.
    def update(self,s,clock):
        c=self.cfg['controller'];dt=self.cfg['control_dt_s']
        decay=np.exp(-dt/c.get('actuator_predictor_tau_s',.04))
        self.motor=decay*self.motor+(1-decay)*self.previous_motor
        if self.previous is not None and s.timestamp>self.previous.timestamp+1e-9:
            elapsed=s.timestamp-self.previous.timestamp
            b3=.5*(s.rotation[:,2]+self.previous.rotation[:,2])
            acc=(s.velocity-self.previous.velocity)/elapsed
            predicted=self.motor.sum()/self.info['mass_kg']
            residual=acc+np.array([0,0,9.81])-predicted*self.ratio*b3
            self.innovation=float(residual@self.target.normal)
            pc=s.position+s.rotation@self.cfg['contact']['offset_body_m']
            near=abs((pc-self.target.position)@self.target.normal)<.12 and np.linalg.norm(pc-self.target.position)<.3
            detected=self.innovation>12.
            if self.window_s:
                free_acc=predicted*self.ratio*b3-np.array([0,0,9.81])
                self.samples.append((s.timestamp,s.velocity.copy(),free_acc))
                while len(self.samples)>1 and self.samples[1][0]<=s.timestamp-self.window_s+1e-9:self.samples.popleft()
                old_t,old_v,old_a=self.samples[0]
                impulse=(s.velocity-old_v-.5*(old_a+free_acc)*(s.timestamp-old_t))@self.target.normal
                detected=(s.timestamp-old_t>=self.window_s*.75 and impulse>self.impulse_threshold)
            if self.contact_time is None and near and detected:
                self.contact_time=clock;self.anchor=s.position.copy()
            if self.contact_time is None and predicted>3:
                measured=float((acc+[0,0,9.81])@b3)
                beta=1-np.exp(-elapsed/c.get('thrust_adaptation_tau_s',.12))
                self.ratio+=beta*(np.clip(measured/predicted,.7,1.3)-self.ratio)
        self.previous=s
        return ActuatorEstimate(self.motor.copy(),self.ratio)


def vector(s,target,actuator,clock,duration,mode,previous_action,contact_time,age=0.,observer=True):
    # No teacher reference, teacher phase, actual contact bit or physical draw.
    elapsed=0. if contact_time is None else min(3.,clock-contact_time)
    return np.r_[clock/duration, duration/6., np.eye(3)[mode],s.position-target.position,s.velocity,
                 s.rotation[:,:2].T.ravel(),s.omega_body,
                 actuator.forces_N if observer else np.zeros(4),actuator.specific_thrust_ratio if observer else 1.,
                 previous_action,float(contact_time is not None),elapsed,age]


def history_vectors(X,lags):
    # Called separately for each episode: histories never cross episode boundaries.
    return np.concatenate([X[np.maximum(0,np.arange(len(X))-lag)] for lag in lags],axis=1)
