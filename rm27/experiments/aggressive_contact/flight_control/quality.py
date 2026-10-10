"""Estimator-independent quality contract and a conservative commitment gate.

No simulator inputs. Covariances are estimates; an unreported bias cannot be
detected by this gate. This is an empirical guard, not a safety certificate.
"""
from dataclasses import dataclass
import numpy as np
from ..model import frame


def covariance(value):
    a=np.asarray(value,float)
    if a.shape!=(3,3) or not np.isfinite(a).all() or not np.allclose(a,a.T) or np.linalg.eigvalsh(a).min() < -1e-12:
        raise ValueError('Expected finite symmetric PSD covariance')
    return a


@dataclass(frozen=True)
class EstimateQuality:
    position_covariance_m2: np.ndarray
    attitude_covariance_rad2: np.ndarray
    target_covariance_m2: np.ndarray
    bias_bound_m: float = 0.
    target_confidence: float = 1.
    target_age_s: float = 0.
    source: str = 'DECLARED_NOMINAL_UNCERTAINTY'
    velocity_bias_bound_m_s: float = 0.
    tilt_bias_bound_deg: float = 0.
    relative_covariance_m2: np.ndarray | None = None

    def __post_init__(self):
        for a in (self.position_covariance_m2,self.attitude_covariance_rad2,self.target_covariance_m2):covariance(a)
        if self.relative_covariance_m2 is not None:covariance(self.relative_covariance_m2)
        if not np.isfinite([self.bias_bound_m,self.target_confidence,self.target_age_s,self.velocity_bias_bound_m_s,self.tilt_bias_bound_deg]).all() or min(self.bias_bound_m,self.target_age_s,self.velocity_bias_bound_m_s,self.tilt_bias_bound_deg)<0 or not 0<=self.target_confidence<=1:
            raise ValueError('Invalid quality declaration')

    @classmethod
    def from_config(cls,c):
        return cls(np.eye(3)*c.get('position_std_m',0.)**2,
                   np.eye(3)*np.deg2rad(c.get('attitude_std_deg',0.))**2,
                   np.eye(3)*c.get('target_std_m',0.)**2,c.get('bias_bound_m',0.),
                   c.get('target_confidence',1.),c.get('target_age_s',0.),c.get('source','ASSUMED'),
                   c.get('velocity_bias_bound_m_s',0.),c.get('tilt_bias_bound_deg',0.),
                   None if 'relative_covariance_m2' not in c else np.array(c['relative_covariance_m2']))

    def risk(self,normal,lever_m,velocity_covariance,age):
        tangent=frame(normal)[:,:2]
        P=self.position_covariance_m2+self.target_covariance_m2 if self.relative_covariance_m2 is None else self.relative_covariance_m2
        sigma=np.sqrt(max(0.,np.linalg.eigvalsh(tangent.T@P@tangent).max()))
        attitude=lever_m*np.sqrt(max(0.,np.linalg.eigvalsh(self.attitude_covariance_rad2).max()))
        velocity=np.sqrt(max(0.,np.linalg.eigvalsh(covariance(velocity_covariance)).max()))
        # Three-sigma heuristic plus explicitly declared bounded bias. Sensor
        # correlations/model mismatch are not a calibrated chance constraint.
        # Persistent bias is not white measurement noise: velocity feedback
        # converts it to position error (~Kd/Kp); tilt feeds back thrust error.
        # Conservative design-sweep heuristics, not universal physical bounds.
        bias=self.bias_bound_m+.5*self.velocity_bias_bound_m_s+.03*self.tilt_bias_bound_deg
        return float(3*(sigma+attitude+max(0.,age)*velocity)+bias)


class HoldReference:
    def __init__(self,position):self.position=np.array(position,float).copy()
    def sample(self,t):return self.position.copy(),np.zeros(3),np.zeros(3),np.zeros(3)


class BrakingReference:
    """Finite-time quintic stop; continuous p/v/a, terminal zero v/a.

    The guard must act early enough for this stopping corridor. No obstacle
    avoidance guarantee is implied by this local reference.
    """
    def __init__(self,position,velocity,acceleration,start,duration=1.2):
        self.start=start;self.duration=duration;T=duration
        self.c=np.zeros((6,3));self.c[:3]=[position,velocity,np.asarray(acceleration)/2]
        end=np.asarray(position)+np.asarray(velocity)*T/2
        A=np.array([[T**3,T**4,T**5],[3*T*T,4*T**3,5*T**4],[6*T,12*T*T,20*T**3]])
        self.c[3:]=np.linalg.solve(A,np.array([end-(self.c[0]+self.c[1]*T+self.c[2]*T*T),-self.c[1]-2*self.c[2]*T,-2*self.c[2]]))
    def sample(self,t):
        h=np.clip(t-self.start,0,self.duration);out=[]
        import math
        for d in range(4):out.append(sum((self.c[k]*math.factorial(k)/math.factorial(k-d)*h**(k-d) for k in range(d,6)),start=np.zeros(3)))
        if t-self.start>=self.duration:out[1:]=[np.zeros(3)]*3
        return tuple(out)


class CommitmentGuard:
    def __init__(self,actor,quality,budget_m=.015,commit_fraction=.5,max_age_s=.035):
        self.actor=actor;self.quality=quality;self.budget=budget_m;self.fraction=commit_fraction;self.max_age=max_age_s
        self.decision=None;self.events=[];self.aborted=False

    def update(self,observation,now):
        elapsed=now-self.actor.start_time
        if self.decision is not None or self.actor.contact_time is not None:return
        s=observation.estimate;age=now-s.timestamp
        risk=self.quality.risk(self.actor.target.normal,np.linalg.norm(self.actor.cfg['contact']['offset_body_m']),observation.velocity_covariance_m2_s2,age)
        if elapsed<self.fraction*self.actor.p['duration_s']:return
        reason=('UNCERTAINTY' if risk>self.budget else 'STALE_EGO' if age>self.max_age else
                'TARGET_QUALITY' if self.quality.target_confidence<.8 or self.quality.target_age_s>.25 else None)
        if reason:
            self.aborted=True;self.decision='ABORT'
            self.actor.reference=BrakingReference(s.position,s.velocity,self.actor.last_reference_state[3:6],elapsed)
            self.actor.deployment=dict(self.actor.deployment,contact_disabled=True)
            self.events.append(dict(time_s=elapsed,event='ABORT',reason=reason,risk_bound_m=risk,age_s=age))
        elif self.decision is None:
            self.decision='COMMIT';self.events.append(dict(time_s=elapsed,event='COMMIT',risk_bound_m=risk,age_s=age))


def select_mode(requested,quality,normal,lever,velocity_covariance,age):
    """Schedule once at reset; no discontinuous mid-flight spline switching."""
    risk=quality.risk(normal,lever,velocity_covariance,age)
    return 'conservative' if risk>.006 or age>.015 else requested


@dataclass(frozen=True)
class RelativeTargetObservation:
    """Future calibrated vision input, in body FLU at the vehicle timestamp.

    Covariance is relative translation uncertainty, conditional on the shared
    pose. It is NOT independent global-target covariance. Commit once at reset;
    this adapter does not implement an online target tracker/replanner.
    """
    position_body_m: np.ndarray
    normal_body: np.ndarray
    covariance_body_m2: np.ndarray
    timestamp: float
    confidence: float

    def register(self,vehicle,now):
        from ..distillation.deployment import TargetEstimate
        from ..model import unit
        covariance(self.covariance_body_m2)
        if abs(vehicle.timestamp-self.timestamp)>.001 or not 0<=now-self.timestamp<=.25:
            raise ValueError('Relative target and ego must be time-aligned and fresh')
        target=TargetEstimate(vehicle.position+vehicle.rotation@self.position_body_m,
            vehicle.rotation@unit(self.normal_body),vehicle.rotation@self.covariance_body_m2@vehicle.rotation.T,self.timestamp,self.confidence)
        target.commit(now) # Validate confidence and finite geometry at the boundary.
        return target
