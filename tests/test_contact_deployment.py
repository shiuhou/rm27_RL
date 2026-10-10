import numpy as np
import pytest
from rm27.experiments.aggressive_contact.model import VehicleEstimate,TargetState
from rm27.experiments.aggressive_contact.distillation.deployment import propagate,TargetEstimate,ThrottleCalibration,ArduPilotAdapter,TransformedReference
from rm27.experiments.aggressive_contact.distillation.observation import ActuatorEstimate,SyntheticEstimator
from rm27.experiments.aggressive_contact.teacher_interface import RateThrust


def state(t):return VehicleEstimate(np.array([0.,0.,1.]),np.array([2.,0.,0.]),np.eye(3),np.array([0.,1.,0.]),t)


def test_causal_prediction_uses_age_and_body_frame_rates():
    act=ActuatorEstimate(np.ones(4)*.18*9.81/4,1.)
    s=propagate(state(4),4.02,act,.18,'dynamic')
    np.testing.assert_allclose(s.position,[.04,0,1])
    np.testing.assert_allclose(s.velocity,[2,0,0])
    np.testing.assert_allclose(s.rotation[:,2],[np.sin(.02),0,np.cos(.02)])
    assert s.timestamp==4.02
    with pytest.raises(ValueError):propagate(state(4),3.99,act,.18)


def test_estimator_jitter_dropouts_lower_rate_are_monotonic():
    e=SyntheticEstimator(seed=7,delay_s=.02,jitter_s=.005,update_hz=50,dropout=.2)
    times=[]
    for i in range(200):
        s,age=e.update(state(i*.005));times.append(s.timestamp)
        assert s.timestamp<=i*.005+1e-9 and age>=0
    assert np.all(np.diff(times)>=0)
    assert len(set(times))<=50


def test_static_target_commit_rejects_stale_and_future_samples():
    t=TargetEstimate(np.ones(3),np.array([2.,0,0]),np.eye(3)*.001,5.,.8)
    np.testing.assert_allclose(t.commit(5.1).normal,[1,0,0])
    for now in (4.99,5.26):
        with pytest.raises(ValueError):t.commit(now)


def test_adapter_requires_calibration_semantics_and_correct_frd_signs():
    c=ThrottleCalibration((0.,4.),(0.,1.),'NOMINAL TEST ONLY')
    with pytest.raises(ValueError):ArduPilotAdapter(c)
    a=ArduPilotAdapter(c,True).packet(RateThrust(np.array([1.,2.,3.]),2.),2.)
    assert (a['body_roll_rate'],a['body_pitch_rate'],a['body_yaw_rate'])==(1.,-2.,-3.)
    assert a['type_mask']==128 and a['thrust']==.5 and a['time_boot_ms']==2000
    with pytest.raises(ValueError):c(5.)
    with pytest.raises(ValueError):ThrottleCalibration((0.,2.,1.),(0.,.5,1.),'bad')


def test_reference_registration_changes_all_derivatives_consistently():
    class Ref:
        def sample(self,t):return (np.array([t,0,1]),np.array([1,0,0]),np.zeros(3),np.zeros(3))
    n=TargetState(np.zeros(3),np.array([0,0,1]),np.zeros((3,3)),0.)
    e=TargetState(np.array([1,2,3]),np.array([1,0,0]),np.zeros((3,3)),0.)
    r=TransformedReference(Ref(),n,e)
    p,v,_,_=r.sample(1.)
    np.testing.assert_allclose((r.sample(1.001)[0]-p)/.001,v,atol=1e-10)
    np.testing.assert_allclose(r.Q@n.normal,e.normal)


def test_portable_observer_does_not_read_inner_loop_gyro():
    from rm27.experiments.aggressive_contact.distillation.policy import StudentSkill,NominalContext
    from rm27.experiments.aggressive_contact.model import build_model,scenario,load_config,truth_estimate,unit
    cfg=load_config();m,d,info,_=build_model(scenario(cfg),cfg);initial=truth_estimate(m,d)
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0.)
    class Ref:
        def reference(self,mode):return self
        def sample(self,t):return initial.position,np.zeros(3),np.zeros(3),np.zeros(3)
    class FixedEstimate:
        def update(self,s):
            return VehicleEstimate(initial.position,initial.velocity,initial.rotation,np.zeros(3),s.timestamp),0.
    def actor():
        a=StudentSkill(NominalContext(cfg,cfg['nominal_skill'],info,target),initial,target,info,Ref(),1,hybrid=True,deployment=dict(portable_observer=True,predict='dynamic'))
        a.estimator=FixedEstimate();return a
    a,b=actor(),actor()
    motors=[]
    for i in range(4):
        sa=VehicleEstimate(initial.position,initial.velocity,initial.rotation,np.zeros(3),i*.005)
        sb=VehicleEstimate(initial.position,initial.velocity,initial.rotation,np.array([1.,2.,3.]),i*.005)
        ma,_=a.command(sa,False);mb,_=b.command(sb,True);motors.append(np.linalg.norm(ma-mb))
        np.testing.assert_array_equal(a.previous_action,b.previous_action)
        np.testing.assert_array_equal(a.causal.motor,b.causal.motor)
    assert max(motors)>0 # independent simulation inner loops, identical actor output


def test_adapter_pymavlink_wire_roundtrip():
    from pymavlink.dialects.v20 import ardupilotmega as mav
    adapter=ArduPilotAdapter(ThrottleCalibration((0.,4.),(0.,1.),'test only'),True)
    fields=adapter.packet(RateThrust(np.array([.2,-.3,.4]),1.8),3.4)
    sender=mav.MAVLink(None);msg=sender.set_attitude_target_encode(**fields)
    parser=mav.MAVLink(None);decoded=parser.parse_char(msg.pack(sender))
    assert decoded.type_mask==128
    np.testing.assert_allclose([decoded.body_roll_rate,decoded.body_pitch_rate,decoded.body_yaw_rate],[.2,.3,-.4],rtol=1e-6)
    assert abs(decoded.thrust-.45)<1e-6


def test_analytic_reference_hover_and_shared_adaptation():
    from rm27.experiments.aggressive_contact.control import Skill
    from rm27.experiments.aggressive_contact.model import load_config,scenario,build_model,truth_estimate,unit
    cfg=load_config();cfg['controller'].update(rate_reference_source='analytic',adapt_specific_thrust=True)
    m,d,info,_=build_model(scenario(cfg),cfg);s=truth_estimate(m,d)
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0.)
    f=Skill(cfg,cfg['nominal_skill'],s,target,info)
    u,sat=f.track(s,(s.position,np.zeros(3),np.zeros(3),np.zeros(3)),specific_thrust_ratio=1.)
    np.testing.assert_allclose(u,np.ones(4)*info['mass_kg']*9.81/4,atol=1e-10)
    assert not sat
    f.track(s,(s.position,np.zeros(3),np.zeros(3),np.zeros(3)),specific_thrust_ratio=.9)
    assert f.specific_thrust_ratio==.9


def test_deployment_api_covariance_and_stale_input_contract():
    from rm27.experiments.aggressive_contact.distillation.deployment import DeploymentPolicy,VehicleObservation
    from rm27.experiments.aggressive_contact.model import load_config,build_model,scenario,truth_estimate,unit
    cfg=load_config();m,d,info,_=build_model(scenario(cfg),cfg);s=truth_estimate(m,d)
    target=TargetEstimate(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0.,1.)
    nominal=target.commit(0.)
    class Ref:
        def reference(self,mode):return self
        def sample(self,t):return s.position,np.zeros(3),np.zeros(3),np.zeros(3)
    api=DeploymentPolicy(cfg,cfg['nominal_skill'],s,target,nominal,info,Ref(),1,dict(portable_observer=True))
    result=api.infer(VehicleObservation(s,np.eye(3)*.05**2),0.)
    assert isinstance(result,RateThrust)
    assert abs(api.actor.causal.impulse_threshold-4*np.sqrt(2)*.05)<1e-12
    with pytest.raises(ValueError,match='timestamp'):api.infer(s,.101)
    with pytest.raises(ValueError,match='covariance'):api.infer(VehicleObservation(s,-np.eye(3)),0.)
