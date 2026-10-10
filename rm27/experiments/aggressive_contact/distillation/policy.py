"""Student execution boundary: VehicleEstimate + TargetState -> RateThrust.

The simulation-only wrapper supplies estimator samples and applies the existing
rate-P proxy. Contact truth is deliberately ignored by the actor. DAgger labels
are optionally queried in a separate oracle after the student action is chosen.
"""
from collections import deque
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from ..teacher_interface import RateThrust,apply_proxy,project
from .observation import SyntheticEstimator,CausalState,vector,ActuatorEstimate


class StudentSkill:
    def __init__(self,teacher,initial,target,info,model,mode,estimator=None,observer=True,oracle=False,hybrid=False,action_delay_s=0.,detector_window_s=0.,intervention=0.,deployment=None):
        self.start_time=initial.timestamp
        self.deployment=deployment or {};self.next_action_time=-np.inf
        self.teacher=teacher;self.model=model;self.mode=mode;self.target=target;self.info=info
        self.estimator=SyntheticEstimator(**(estimator or {}));self.causal=CausalState(teacher.cfg,info,target,initial,detector_window_s)
        self.causal.impulse_threshold=max(.12,4*np.sqrt(2)*self.deployment.get('contact_velocity_std_m_s',0.))
        self.intervention=intervention
        self.use_observer=observer;self.oracle=oracle;self.hybrid=hybrid
        self.buffer=deque(maxlen=1+max(model.lags if not hybrid else [0]))
        self.previous_action=np.r_[np.zeros(3),info['mass_kg']*9.81]
        self.records=[];self.delay=action_delay_s;self.queue=[];self.held=RateThrust(np.zeros(3),info['mass_kg']*9.81)
        self.contact_time=None;self.phase='FREE_FLIGHT';self.recovery_anchor=None
        self.last_reference=initial.position.copy();self.last_reference_state=np.r_[np.zeros(6),[1,0,0,0],np.zeros(6)]
        if hybrid:
            # The existing geometric feedback is explicitly retained in this hybrid.
            from ..control import Skill
            feedback_cfg=copy.deepcopy(teacher.cfg)
            if not observer:
                feedback_cfg['controller'].update(actuator_feedback_gain=0.,adapt_specific_thrust=False)
            feedback_cfg['controller'].update(self.deployment.get('feedback_overrides',{}))
            self.feedback=Skill(feedback_cfg,teacher.p,initial,target,info)
            self.reference=model.reference(mode)
    def __getattr__(self,key):return getattr(self.teacher,key)
    @property
    def motor_estimate(self):return self.causal.motor
    @property
    def specific_thrust_ratio(self):return self.causal.ratio
    def command(self,state,contact,control_clock=None):
        sensed,age=self.estimator.update(state)
        clock=state.timestamp if control_clock is None else float(control_clock)
        age=max(age,clock-sensed.timestamp)
        elapsed=clock-self.start_time
        act=self.causal.update(sensed,clock);self.contact_time=self.causal.contact_time
        if self.deployment.get('contact_disabled'):self.contact_time=None
        self.phase=('RECOVERY' if self.contact_time is not None else 'PRE_CONTACT' if elapsed>.7*self.p['duration_s'] else 'FREE_FLIGHT')
        obs=vector(sensed,self.target,act,elapsed,self.p['duration_s'],self.mode,self.previous_action,
                   None if self.contact_time is None else self.contact_time-self.start_time,age,self.use_observer)
        self.buffer.append(obs)
        if self.hybrid:
            if self.deployment.get('predict'):
                from .deployment import propagate
                sensed=propagate(sensed,clock,act,self.mass,self.deployment['predict'])
            if self.contact_time is not None:
                if self.recovery_anchor is None:
                    self.recovery_anchor=self.causal.anchor+self.target.normal*self.p['retreat_m']
                    self.recovery_anchor[2]+=self.p['recovery_lift_m']
                desired=(self.recovery_anchor,np.zeros(3),np.zeros(3),np.zeros(3))
            else:desired=self.reference.sample(elapsed)
            f=self.feedback;f.contact_time=self.contact_time
            # Independent causal observer, same nominal actuator compensation.
            f.previous_motor_command=self.causal.previous_motor.copy()
            ratio=act.specific_thrust_ratio if self.deployment.get('shared_adaptation') else None
            command,sat=f.track(sensed,desired,specific_thrust_ratio=ratio)
            self.causal.ratio=f.specific_thrust_ratio
            c=self.cfg['controller'];gain=2*c['attitude_damping']*c['attitude_frequency_rad_s']
            z=project(command,sensed.omega_body,self.inertia,self.mixer,gain)
            action=np.r_[z.body_rates_flu_rad_s,z.collective_force_N]
            self.last_reference=f.last_reference;self.last_reference_state=f.last_reference_state
        else:
            h=np.concatenate([self.buffer[max(0,len(self.buffer)-1-lag)] for lag in self.model.lags])
            action=self.model(h);sat=False
            self.last_reference=state.position.copy()
            self.last_reference_state=np.r_[state.velocity,np.zeros(3),Rotation.from_matrix(state.rotation).as_quat()[[3,0,1,2]],action[:3],state.rotation[:,2]*action[3]]
        action=np.r_[np.clip(action[:3],-20,20),np.clip(action[3],0,4*self.max_motor)]
        self.previous_action=action.copy()
        if clock>=self.next_action_time-1e-9:
            self.queue.append((clock+self.delay,RateThrust(action[:3],action[3])))
            self.next_action_time=clock+1/self.deployment.get('action_hz',200.)
        while self.queue and self.queue[0][0]<=clock+1e-9:_,self.held=self.queue.pop(0)
        gain=2*self.cfg['controller']['attitude_damping']*self.cfg['controller']['attitude_frequency_rad_s']
        motor,clipped=apply_proxy(self.held,state.omega_body,self.inertia,self.inverse_mixer,gain,self.max_motor,
                                 gain_scale=self.deployment.get('inner_gain_scale',1.),return_saturation=True)
        if self.hybrid and self.deployment.get('direct_motor'):
            motor=command;clipped=False
        self.causal.previous_motor=motor.copy()
        if self.deployment.get('portable_observer'):
            # The plant's ideal inner gyro must NOT leak into actor motor state.
            # Predict nominal mixer output using only the actor's own estimate.
            self.causal.previous_motor=apply_proxy(self.held,sensed.omega_body,self.inertia,
                                                  self.inverse_mixer,gain,self.max_motor)
        label=np.full(4,np.nan)
        if self.oracle:
            # Privileged teacher may see real contact, but its output is ONLY a label.
            u,_=self.teacher.command(state,contact)
            a=project(u,state.omega_body,self.inertia,self.mixer,gain);label=np.r_[a.body_rates_flu_rad_s,a.collective_force_N]
            if self.intervention:
                action=(1-self.intervention)*action+self.intervention*label
                motor,clipped=apply_proxy(RateThrust(action[:3],action[3]),state.omega_body,self.inertia,self.inverse_mixer,gain,self.max_motor,return_saturation=True)
                self.previous_action=action.copy();self.causal.previous_motor=motor.copy()
            self.teacher.previous_motor_command=motor.copy()
        self.records.append((clock,obs.copy(),action.copy(),label,age,self.causal.innovation))
        return motor,bool(sat or clipped)
    def save(self,path):
        np.savez_compressed(path,time_s=np.array([r[0] for r in self.records]),observation=np.array([r[1] for r in self.records]),
                            student_action=np.array([r[2] for r in self.records]),oracle_label=np.array([r[3] for r in self.records]),
                            observation_age_s=np.array([r[4] for r in self.records]),innovation=np.array([r[5] for r in self.records]))


class NominalContext:
    """Nominal calibration/task constants, with no optimizer or teacher policy."""
    def __init__(self,cfg,parameters,info,target):
        self.cfg,self.p,self.target=cfg,parameters,target
        self.mass=info['mass_kg'];self.inertia=np.array(info['inertia_kg_m2']);self.max_motor=info['max_rotor_thrust_N_assumed']
        arm=info['rotor_arm_m'];yaw=info['yaw_torque_per_thrust_m']
        self.mixer=np.array([[1,1,1,1],[arm,-arm,-arm,arm],[-arm,-arm,arm,arm],[yaw,-yaw,yaw,-yaw]])
        self.inverse_mixer=np.linalg.inv(self.mixer)


class StudentAPI:
    """Reset once, then infer(VehicleEstimate)->RateThrust at 200 Hz.

    TargetState is a committed task estimate. The internal motor observer uses a
    NOMINAL rate-loop actuator prediction, not true simulator motors. Real AP
    throttle calibration and estimator/contact validation remain external.
    This standalone path never constructs ManeuverSkill or accesses MuJoCo data.
    """
    def __init__(self,cfg,parameters,initial,target,vehicle_info,model,mode,hybrid=False,detector_window_s=.02):
        self.actor=StudentSkill(NominalContext(cfg,parameters,vehicle_info,target),initial,target,vehicle_info,
                                model,mode,hybrid=hybrid,detector_window_s=detector_window_s)
    @classmethod
    def from_bundle(cls,bundle,initial,target,mode='balanced'):
        import json
        from pathlib import Path
        from .spline import SplinePolicy
        bundle=Path(bundle)
        cfg=json.loads((bundle/'nominal_config.json').read_text())
        params=json.loads((bundle/'task_parameters.json').read_text())[mode]
        info=json.loads((bundle/'nominal_vehicle.json').read_text())['vehicle_info']
        return cls(cfg,params,initial,target,info,SplinePolicy.load(bundle/'policy.npz'),
                   ['aggressive','balanced','conservative'].index(mode),hybrid=True)

    def infer(self,estimate,now=None):
        self.actor.command(estimate,False,control_clock=now)
        a=self.actor.previous_action
        return RateThrust(a[:3].copy(),float(a[3]))
    def actuator_estimate(self):
        return ActuatorEstimate(self.actor.causal.motor.copy(),self.actor.causal.ratio)
