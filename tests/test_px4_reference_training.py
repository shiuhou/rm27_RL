import pytest
import json
import numpy as np

from rm27.experiments.aggressive_contact.distillation.px4_reference_training import eligible_cases
from rm27.experiments.aggressive_contact.distillation.deployment import DeploymentPolicy
from rm27.experiments.aggressive_contact.distillation.policy import StudentSkill,NominalContext
from rm27.experiments.aggressive_contact.model import load_config,build_model,scenario,VehicleEstimate,TargetState,unit
from rm27.experiments.aggressive_contact.maneuver import settings


def test_fitted_px4_reference_rejects_reserved_episodes_and_mixed_policy_data():
    case=dict(name='episode-1',split='holdout',policy='teacher',mode='conservative')
    with pytest.raises(ValueError,match='holdout'):
        eligible_cases(dict(autopilot='px4',cases=[case]))
    case['split']='train';case['policy']='student'
    with pytest.raises(ValueError,match='Teacher'):
        eligible_cases(dict(autopilot='px4',cases=[case]))
    case['policy']='teacher'
    with pytest.raises(ValueError,match='Duplicate'):
        eligible_cases(dict(autopilot='px4',cases=[case,case]))


def test_single_mode_bundle_cannot_silently_run_another_strategy(tmp_path):
    (tmp_path/'capabilities.json').write_text(json.dumps({'supported_modes':['conservative']}))
    with pytest.raises(ValueError,match='not supported'):
        DeploymentPolicy.from_bundle(tmp_path,None,None,mode='balanced')


def test_hybrid_can_follow_the_teachers_observation_clock_without_restamping_state():
    cfg=load_config();_,_,info,_=build_model(scenario(cfg),cfg)
    initial=VehicleEstimate(np.array(cfg['target']['center_m'])+[1.,0,1.],np.zeros(3),np.eye(3),np.zeros(3),0.)
    state=VehicleEstimate(initial.position,np.zeros(3),np.eye(3),np.zeros(3),1.)
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0.)
    samples=[]
    class Reference:
        def sample(self,t):
            samples.append(t)
            return initial.position,np.zeros(3),np.zeros(3),np.zeros(3)
    class Model:
        def reference(self,mode):return Reference()
    context=NominalContext(cfg,settings()['terminal'],info,target)
    for options in ({},{'reference_time_source':'observation'}):
        actor=StudentSkill(context,initial,target,info,Model(),2,hybrid=True,deployment=options)
        actor.command(state,False,control_clock=1.028)
    assert samples==[1.028,1.]
    assert state.timestamp==1.
