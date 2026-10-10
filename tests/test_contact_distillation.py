import numpy as np
from scipy.spatial.transform import Rotation
from rm27.experiments.aggressive_contact.distillation.observation import history_vectors,SyntheticEstimator,CausalState
from rm27.experiments.aggressive_contact.distillation.network import MLP,train
from rm27.experiments.aggressive_contact.model import VehicleEstimate,TargetState,load_config,build_model,scenario,truth_estimate,unit
from rm27.experiments.aggressive_contact.maneuver import ManeuverSkill,settings
from rm27.experiments.aggressive_contact.distillation.policy import StudentSkill


def state(t,v=0):return VehicleEstimate(np.array([0.,0.,1.]),np.array([v,0.,0.]),np.eye(3),np.zeros(3),t)


def test_history_is_causal_and_episode_local():
    x=np.arange(8).reshape(4,2);h=history_vectors(x,[0,2])
    np.testing.assert_array_equal(h[:,2:],x[[0,0,0,1]])
    x[3]=999
    np.testing.assert_array_equal(history_vectors(x,[0,2])[:3],h[:3])


def test_delay_never_uses_future_observations():
    e=SyntheticEstimator(delay_s=.01)
    for i in range(20):
        s,age=e.update(state(i*.005,i))
        assert s.timestamp<=i*.005
        if i>=2:assert s.timestamp<=i*.005-.01+1e-9
        assert age>=0


def test_student_ignores_simulator_contact_truth():
    cfg=load_config();m,d,info,_=build_model(scenario(cfg),cfg);s=truth_estimate(m,d)
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0)
    from rm27.experiments.aggressive_contact.distillation.observation import vector,ActuatorEstimate
    n=len(vector(s,target,ActuatorEstimate(np.ones(4),1),0,5,1,np.zeros(4),None))
    model=MLP([np.zeros((n,4))],[np.array([0.,0.,0.,info['mass_kg']*9.81])],np.zeros(n),np.ones(n),np.zeros(4),np.ones(4),[0])
    def actor():return StudentSkill(ManeuverSkill(cfg,settings()['initial_design'],s,target,info,np.array([12,1,1.74])),s,target,info,model,1)
    a,b=actor(),actor()
    for i in range(4):
        sample=VehicleEstimate(s.position,s.velocity,s.rotation,s.omega_body,i*.005)
        np.testing.assert_array_equal(a.command(sample,False)[0],b.command(sample,True)[0])
    assert a.contact_time is None and b.contact_time is None


def test_numpy_mlp_trains_and_roundtrips(tmp_path):
    rng=np.random.default_rng(2);x=rng.normal(size=(160,3));y=x@rng.normal(size=(3,4))*.1
    cfg=dict(hidden=[12],epochs=80,batch_size=40,learning_rate=.01)
    r=train(x[:120],y[:120],x[120:],y[120:],tmp_path/'fit',cfg,[0],seed=7)
    assert r['best_validation_normalized_mse']<.03
    model=MLP.load(tmp_path/'fit/policy.npz');model.save(tmp_path/'copy.npz')
    np.testing.assert_array_equal(model(x),MLP.load(tmp_path/'copy.npz')(x))


def test_impulse_window_rejects_zero_mean_velocity_noise_and_detects_real_change():
    cfg=load_config();_,_,info,_=build_model(scenario(cfg),cfg)
    n=unit(cfg['target']['normal']);p=np.array(cfg['target']['center_m'])
    target=TargetState(p,n,np.zeros((3,3)),0.)
    R=__import__('rm27.experiments.aggressive_contact.model',fromlist=['frame']).frame(n)
    center=p-R@cfg['contact']['offset_body_m']+n*.02
    causal=CausalState(cfg,info,target,state(0),window_s=.02)
    for i in range(30):
        # Deliberately alternating velocity error, no contact impulse.
        s=VehicleEstimate(center,n*(.015 if i%2 else -.015),R,np.zeros(3),i*.005)
        causal.update(s,s.timestamp)
    assert causal.contact_time is None
    for i in range(30,34):
        s=VehicleEstimate(center,n*.5,R,np.zeros(3),i*.005)
        causal.update(s,s.timestamp)
    assert causal.contact_time is not None


def test_standalone_api_requires_no_teacher_or_simulator_state():
    from rm27.experiments.aggressive_contact.distillation.policy import StudentAPI
    from rm27.experiments.aggressive_contact.distillation.observation import vector,ActuatorEstimate
    from rm27.experiments.aggressive_contact.teacher_interface import RateThrust
    cfg=load_config();_,_,info,_=build_model(scenario(cfg),cfg)
    s=state(0);target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0)
    size=len(vector(s,target,ActuatorEstimate(np.ones(4),1),0,1.4,1,np.zeros(4),None))
    model=MLP([np.zeros((size,4))],[np.array([0,0,0,1.8])],np.zeros(size),np.ones(size),np.zeros(4),np.ones(4),[0])
    api=StudentAPI(cfg,cfg['nominal_skill'],s,target,info,model,1)
    action=api.infer(s)
    assert isinstance(action,RateThrust)
    assert action.collective_force_N==1.8
    assert not hasattr(api.actor.teacher,'command')
    assert not hasattr(api.actor.teacher,'trajectory')


def test_split_guard_rejects_same_physical_scenario_across_partitions(tmp_path):
    import json
    import pytest
    from rm27.experiments.aggressive_contact.distillation.dataset import arrays
    entries=[dict(scenario_group='810000/1.0',split='train'),dict(scenario_group='810000/1.0',split='validation')]
    (tmp_path/'index.json').write_text(json.dumps(entries))
    with pytest.raises(ValueError,match='Scenario crosses'):arrays(tmp_path)


def test_standalone_clock_offset_does_not_change_policy_action():
    from rm27.experiments.aggressive_contact.distillation.policy import StudentAPI
    from rm27.experiments.aggressive_contact.distillation.observation import vector,ActuatorEstimate
    cfg=load_config();_,_,info,_=build_model(scenario(cfg),cfg)
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0)
    size=len(vector(state(0),target,ActuatorEstimate(np.ones(4),1),0,1.4,1,np.zeros(4),None))
    W=np.zeros((size,4));W[0,0]=.1
    model=MLP([W],[np.array([0,0,0,1.8])],np.zeros(size),np.ones(size),np.zeros(4),np.ones(4),[0])
    a=StudentAPI(cfg,cfg['nominal_skill'],state(0),target,info,model,1)
    b=StudentAPI(cfg,cfg['nominal_skill'],state(1000),target,info,model,1)
    for i in range(5):
        x=a.infer(state(i*.005),now=i*.005+.01)
        y=b.infer(state(1000+i*.005),now=1000+i*.005+.01)
        np.testing.assert_allclose(x.body_rates_flu_rad_s,y.body_rates_flu_rad_s,atol=1e-12)
    assert a.actor.records[-1][4]>.009
