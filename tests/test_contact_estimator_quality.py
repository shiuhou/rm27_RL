import numpy as np
import pytest
from types import SimpleNamespace
from rm27.experiments.aggressive_contact.model import VehicleEstimate
from rm27.experiments.aggressive_contact.distillation.deployment import VehicleObservation
from rm27.experiments.aggressive_contact.flight_control.quality import EstimateQuality,BrakingReference,CommitmentGuard
from rm27.experiments.aggressive_contact.flight_control.quality import RelativeTargetObservation
from rm27.experiments.aggressive_contact.flight_control.synthetic import ErrorSensor
from rm27.experiments.aggressive_contact.flight_control.diagnostics import TruthHistory,ExternalNavigationSensor


def state(t):return VehicleEstimate(np.array([2*t,0.,2.]),np.array([2.,0,0]),np.eye(3),np.zeros(3),t)


def test_quality_projects_only_contact_tangents_and_accounts_for_bias():
    q=EstimateQuality(np.diag([.01**2,0,0]),np.zeros((3,3)),np.zeros((3,3)),.002)
    assert q.risk(np.array([1,0,0]),.042,np.zeros((3,3)),0)==.002
    assert abs(q.risk(np.array([0,0,1]),.042,np.zeros((3,3)),0)-.032)<1e-12
    with pytest.raises(ValueError):EstimateQuality(-np.eye(3),np.eye(3),np.eye(3))
    with pytest.raises(ValueError):EstimateQuality.from_config(dict(target_confidence=2))


def test_braking_matches_initial_derivatives_and_stops_without_jump():
    p=np.array([1.,2,3]);v=np.array([6.,.2,0]);a=np.array([0.,.1,.2]);r=BrakingReference(p,v,a,2.)
    for actual,expected in zip(r.sample(2.)[:3],(p,v,a)):np.testing.assert_allclose(actual,expected)
    end=r.sample(3.2);np.testing.assert_allclose(end[0],p+.6*v,atol=1e-12)
    for d in end[1:]:np.testing.assert_allclose(d,0,atol=1e-12)
    np.testing.assert_allclose(r.sample(3.2-1e-6)[0],end[0],atol=1e-9)


def test_commit_decision_uses_observation_quality_not_truth_contact():
    a=SimpleNamespace(start_time=0.,contact_time=None,target=SimpleNamespace(normal=np.array([0,0,1.])),
       cfg={'contact':{'offset_body_m':[0,0,-.042]}},p={'duration_s':4.},last_reference_state=np.zeros(16),deployment={},reference=None)
    g=CommitmentGuard(a,EstimateQuality.from_config({'bias_bound_m':.05}))
    g.update(VehicleObservation(state(1),np.eye(3)*.0004),1);assert not g.events
    g.update(VehicleObservation(state(2),np.eye(3)*.0004),2);assert g.aborted and g.events[0]['reason']=='UNCERTAINTY'
    g.update(VehicleObservation(state(3),np.eye(3)*.0004),3);assert len(g.events)==1
    assert isinstance(a.reference,BrakingReference)


def test_sensor_preserves_timestamps_and_bias_at_fixed_sample_rate():
    e=ErrorSensor(dict(position_bias_m=[0,.02,0],delay_s=.02,update_hz=50))
    for t in np.arange(0,.101,.005):s,age=e.update(state(t))
    assert abs(s.timestamp-.08)<1e-10 and abs(age-.02)<1e-10
    np.testing.assert_allclose(s.position,[.16,.02,2])


def test_component_diagnostic_keeps_other_measurements_and_time():
    h=TruthHistory();h.append(state(0));h.append(state(.1))
    fake=VehicleEstimate(np.ones(3),np.ones(3),np.eye(3),np.zeros(3),.05)
    o=h.correct(VehicleObservation(fake,np.eye(3)),'position')
    np.testing.assert_allclose(o.estimate.position,[.1,0,2]);np.testing.assert_array_equal(o.estimate.velocity,np.ones(3));assert o.estimate.timestamp==.05


def test_external_nav_frames_and_no_duplicate_measurement_sends():
    sent=[]
    m=SimpleNamespace(vision_position_estimate_send=lambda *a:sent.append(('p',a)),vision_speed_estimate_send=lambda *a:sent.append(('v',a)))
    sensor=ExternalNavigationSensor(dict(update_hz=50),[1,2,3]);conn=SimpleNamespace(mav=m)
    sensor.send(conn,state(0));sensor.send(conn,state(.005))
    assert len(sent)==2;np.testing.assert_allclose(sent[0][1][1:4],[-1,2,1]);np.testing.assert_allclose(sent[1][1][1:4],[2,0,0])


def test_relative_target_uses_common_frame_and_rejects_mixed_timestamps():
    v=state(.1);r=RelativeTargetObservation(np.array([2.,3,1]),np.array([0,0,1]),np.eye(3)*.001,.1,.9)
    t=r.register(v,.11);np.testing.assert_allclose(t.position_nwu_m-v.position,[2,3,1])
    with pytest.raises(ValueError):r.register(state(.12),.12)
    q=EstimateQuality(np.eye(3),np.zeros((3,3)),np.eye(3),relative_covariance_m2=np.zeros((3,3)))
    assert q.risk(np.array([0,0,1]),.042,np.zeros((3,3)),0)==0
