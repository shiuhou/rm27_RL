"""Timestamped deployment contracts and bounded causal state propagation.

No MuJoCo state, future sample, true motor state or physical random draw is used.
The static target is committed at reset; a moving-target/vision tracker is NOT
implemented. Force->throttle requires externally supplied calibration.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from ..model import VehicleEstimate, TargetState, frame
from ..teacher_interface import RateThrust, mavlink_packet


@dataclass(frozen=True)
class VehicleObservation:
    estimate: VehicleEstimate
    velocity_covariance_m2_s2: np.ndarray


@dataclass(frozen=True)
class TargetEstimate:
    position_nwu_m: np.ndarray
    normal_nwu: np.ndarray
    covariance_m2: np.ndarray
    timestamp_s: float
    confidence: float

    def commit(self, now, max_age_s=.25):
        values=np.r_[self.position_nwu_m,self.normal_nwu,np.asarray(self.covariance_m2).ravel(),self.timestamp_s,self.confidence]
        if not np.isfinite(values).all() or not 0<=now-self.timestamp_s<=max_age_s or not 0<self.confidence<=1:
            raise ValueError('Invalid, stale, future or unconfident target estimate')
        n=np.asarray(self.normal_nwu,float)
        if np.linalg.norm(n)<1e-8:raise ValueError('Target normal is zero')
        return TargetState(np.array(self.position_nwu_m,float),n/np.linalg.norm(n),np.array(self.covariance_m2,float),
                           self.timestamp_s,self.confidence,'COMMITTED_STATIC_ESTIMATE')


class TransformedReference:
    """Rigid registration of an existing fixed-scene reference to a task estimate."""
    def __init__(self,reference,nominal,estimated):
        self.reference=reference
        self.Q=frame(estimated.normal)@frame(nominal.normal).T
        self.shift=estimated.position-self.Q@nominal.position
    def sample(self,t):
        p,v,a,j=self.reference.sample(t)
        return self.Q@p+self.shift,self.Q@v,self.Q@a,self.Q@j


def propagate(s,now,actuator,mass,kind='kinematic',max_age_s=.035):
    """Bounded extrapolation, not a certified collision-state estimator.

    Kinematic: constant measured v and body rate. Dynamic adds nominal force
    acceleration; extrapolation across an unobserved impact remains uncertain.
    """
    age=float(now-s.timestamp)
    if age < -1e-8:raise ValueError('Future vehicle estimate')
    h=np.clip(age,0,max_age_s)
    a=np.zeros(3)
    if kind=='dynamic':a=s.rotation[:,2]*actuator.forces_N.sum()*actuator.specific_thrust_ratio/mass-[0,0,9.81]
    elif kind!='kinematic':raise ValueError(kind)
    R=s.rotation@Rotation.from_rotvec(s.omega_body*h).as_matrix()
    return VehicleEstimate(s.position+s.velocity*h+.5*a*h*h,s.velocity+a*h,R,s.omega_body.copy(),now,'CAUSAL_PROPAGATED')


@dataclass(frozen=True)
class ThrottleCalibration:
    """Explicit monotonic static force/throttle lookup; not measured by this code."""
    force_N: tuple
    throttle: tuple
    provenance: str
    def __post_init__(self):
        f,t=np.asarray(self.force_N),np.asarray(self.throttle)
        if len(f)<2 or f.shape!=t.shape or not np.isfinite(np.r_[f,t]).all() or np.any(np.diff(f)<=0) or np.any(np.diff(t)<=0) or t[0]<0 or t[-1]>1 or not self.provenance:
            raise ValueError('Explicit monotonic force/throttle calibration and provenance required')
    def __call__(self,force):
        if not self.force_N[0]<=force<=self.force_N[-1]:raise ValueError('Force outside calibrated range')
        return float(np.interp(force,self.force_N,self.throttle))


class ArduPilotAdapter:
    """Packet builder only: GUID_OPTIONS bit 3 required; sends no messages.

    This does not emulate AP attitude shaping, angle boost, thrust linearization,
    PID, filters or mixer. Rate command is FLU rad/s + total physical force N.
    """
    def __init__(self,calibration,thrust_semantics_confirmed=False):
        if not thrust_semantics_confirmed:raise ValueError('AP thrust semantics must be explicitly confirmed')
        self.calibration=calibration
    def packet(self,action,now):
        if not np.isfinite(np.r_[action.body_rates_flu_rad_s,action.collective_force_N,now]).all():raise ValueError('Nonfinite action')
        return mavlink_packet(action,now,self.calibration)


class DeploymentPolicy:
    """Reset committed target, infer(timestamped VehicleEstimate, now)->RateThrust.

    Initial pose must already be an estimate. Reference nominal registration is
    provided separately. No target truth or simulator object enters this API.
    Invalid / too-old ego samples raise rather than fabricate a safe flight action.
    A real system must connect that condition to its independently tested failsafe.
    """
    def __init__(self,cfg,parameters,initial,target_estimate,nominal_target,vehicle_info,model,mode,options=None):
        from .policy import StudentSkill,NominalContext
        target=target_estimate.commit(initial.timestamp)
        self.actor=StudentSkill(NominalContext(cfg,parameters,vehicle_info,target),initial,target,vehicle_info,model,mode,
                                hybrid=True,detector_window_s=.02,deployment=options)
        self.actor.reference=TransformedReference(self.actor.reference,nominal_target,target)
    @classmethod
    def from_bundle(cls,bundle,initial,target_estimate,mode='balanced',options=None):
        import json
        from pathlib import Path
        from .spline import SplinePolicy
        root=Path(bundle)
        if (root/'capabilities.json').exists():
            capabilities=json.loads((root/'capabilities.json').read_text())
            if mode not in capabilities['supported_modes']:
                raise ValueError('Mode is not supported by this fitted bundle: '+mode)
        cfg=json.loads((root/'nominal_config.json').read_text())
        params=json.loads((root/'task_parameters.json').read_text())[mode]
        info=json.loads((root/'nominal_vehicle.json').read_text())['vehicle_info']
        nominal=TargetState(np.array(cfg['target']['center_m']),np.array(cfg['target']['normal']),np.zeros((3,3)),initial.timestamp)
        if options is None and (root/'deployment_options.json').exists():
            options=json.loads((root/'deployment_options.json').read_text())
        return cls(cfg,params,initial,target_estimate,nominal,info,SplinePolicy.load(root/'policy.npz'),
                   ['aggressive','balanced','conservative'].index(mode),
                   dict(predict='dynamic',portable_observer=True) if options is None else options)
    def infer(self,vehicle,now):
        if isinstance(vehicle,VehicleObservation):
            cov=np.asarray(vehicle.velocity_covariance_m2_s2)
            if cov.shape!=(3,3) or not np.isfinite(cov).all() or not np.allclose(cov,cov.T) or np.linalg.eigvalsh(cov).min() < -1e-12:
                raise ValueError('Invalid velocity covariance')
            n=self.actor.target.normal
            sigma=np.sqrt(max(0.,n@cov@n))
            # Four-sigma Gaussian endpoint heuristic, NOT a calibrated contact
            # probability: modeling errors and correlated noise are not covered.
            self.actor.causal.impulse_threshold=max(.12,4*np.sqrt(2)*sigma)
            vehicle=vehicle.estimate
        if not -1e-9<=now-vehicle.timestamp<=.1+1e-9:raise ValueError('Vehicle estimate timestamp outside supported window')
        values=np.r_[vehicle.position,vehicle.velocity,vehicle.rotation.ravel(),vehicle.omega_body]
        if not np.isfinite(values).all():raise ValueError('Nonfinite vehicle estimate')
        self.actor.command(vehicle,False,control_clock=now)
        a=self.actor.previous_action
        return RateThrust(a[:3].copy(),float(a[3]))
    def actuator_estimate(self):
        from .observation import ActuatorEstimate
        return ActuatorEstimate(self.actor.causal.motor.copy(),self.actor.causal.ratio)
