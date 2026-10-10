"""Action-wire semantics, causal dataset boundaries and predictor support."""
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27.experiments.aggressive_contact.teacher_interface import (
    RateThrust,project,apply_proxy,attitude_ned_frd,mavlink_packet,FLIP)
from rm27.experiments.aggressive_contact.teacher_data import export,features,SCHEMA
from rm27.experiments.aggressive_contact.model import load_config,build_model,scenario,truth_estimate,TargetState,unit
from rm27.experiments.aggressive_contact.control import Skill
from rm27.experiments.aggressive_contact.experiment import run_episode
from rm27.experiments.aggressive_contact.recoverability import RecoverabilityModel


def test_projected_inner_loop_recovers_motor_wrench():
    cfg=load_config();m,d,info,_=build_model(scenario(cfg),cfg)
    skill=Skill(cfg,cfg['nominal_skill'],truth_estimate(m,d),TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0),info)
    rng=np.random.default_rng(129)
    for _ in range(30):
        motors=rng.uniform(.1,.9,4)*skill.max_motor;omega=rng.normal(0,2,3)
        action=project(motors,omega,skill.inertia,skill.mixer,30)
        applied=apply_proxy(action,omega,skill.inertia,skill.inverse_mixer,30,skill.max_motor)
        np.testing.assert_allclose(applied,motors,atol=2e-15)


def test_mavlink_wire_rate_signs_and_explicit_throttle_mapping():
    from pymavlink.dialects.v20 import ardupilotmega as mav
    action=RateThrust(np.array([.3,-.5,.7]),1.8)
    fields=mavlink_packet(action,1.25,lambda force:force/4)
    wire=mav.MAVLink(None)
    msg=wire.set_attitude_target_encode(**fields)
    decoded=mav.MAVLink(None).parse_char(msg.pack(wire))
    assert decoded.type_mask==128 and decoded.time_boot_ms==1250
    np.testing.assert_allclose([decoded.body_roll_rate,decoded.body_pitch_rate,decoded.body_yaw_rate],[.3,.5,-.7])
    assert decoded.thrust==pytest.approx(.45)
    with pytest.raises(ValueError):mavlink_packet(action,0,lambda force:2)
    R=Rotation.from_euler('xyz',[.1,.3,-.4]).as_matrix()
    q=attitude_ned_frd(R)
    np.testing.assert_allclose(Rotation.from_quat(q[[1,2,3,0]]).as_matrix(),FLIP@R@FLIP,atol=1e-15)


def test_prefix_features_and_export_do_not_invent_future_recovery(tmp_path):
    cfg=load_config()
    result=run_episode(cfg,output=tmp_path/'prefix',stop_after_contact_s=.12)
    assert result['prefix_only'] and result['first_contact']
    assert result['duration_s']-result['first_contact']['time_s']>=.12-1e-9
    assert features(tmp_path/'prefix').shape==(27,)
    # Alter only future labels; query features cannot change.
    old=features(tmp_path/'prefix')
    result['recovery_time_s']=99
    (tmp_path/'prefix/summary.json').write_text(json.dumps(result))
    np.testing.assert_array_equal(old,features(tmp_path/'prefix'))
    export(tmp_path/'prefix',tmp_path/'demo','design','test')
    data=np.load(tmp_path/'demo/demo.npz');meta=json.loads((tmp_path/'demo/metadata.json').read_text())
    assert meta['event_samples']['recovery']=={'available':False}
    assert np.all(np.diff(data['time_s'])>0)
    for group in SCHEMA['actor_allowlist']:
        assert not SCHEMA['groups'][group]['privileged_only']
        assert len(data[group])==len(data['time_s'])
    assert 'contact_truth' not in SCHEMA['actor_allowlist']
    assert 'phase' not in SCHEMA['actor_allowlist']


def test_recoverability_support_abstains_far_outside_training(tmp_path):
    rng=np.random.default_rng(23);X=rng.normal(size=(50,27));Y=np.c_[np.ones(50),np.ones(50),np.zeros(50)]
    model=RecoverabilityModel(X,Y)
    assert model.predict(X[0])['supported']
    assert not model.predict(np.ones(27)*100)['supported']
    model.save(tmp_path/'model.npz')
    assert RecoverabilityModel.load(tmp_path/'model.npz').predict(X[1])==model.predict(X[1])


def test_proxy_reports_its_own_motor_saturation():
    action=RateThrust(np.array([100.,0,0]),1.)
    command,saturated=apply_proxy(action,np.zeros(3),np.ones(3),np.eye(4),10.,1.,return_saturation=True)
    assert saturated
    assert np.all((command>=0)&(command<=1))
