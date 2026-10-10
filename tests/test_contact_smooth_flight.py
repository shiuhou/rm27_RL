"""Heading dynamics and flight-quality semantics, not camera cosmetics."""
import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27.experiments.aggressive_contact.control import Skill, oriented_frame
from rm27.experiments.aggressive_contact.flight_quality import view_geometry
from rm27.experiments.aggressive_contact.heading import HeadingTurn, course_yaw
from rm27.experiments.aggressive_contact.model import (
    TargetState, build_model, frame, load_config, marker_center, normal_alignment,
    scenario, truth_estimate, unit)


def test_marker_center_uses_asset_offset_not_patch_heading():
    base = {
        "center_m": [-11.468, .02, 1.0758],
        "normal": [.4617, 0., .887],
        "in_plane_rotation_deg": 0.,
    }
    rotated = dict(base, in_plane_rotation_deg=90.)
    expected = np.array([-11.515, .02, 1.17])
    np.testing.assert_allclose(marker_center(base), expected, atol=1e-12)
    np.testing.assert_allclose(marker_center(rotated), expected, atol=1e-12)


def test_marker_center_follows_normal_only_for_target_perturbation():
    base_normal = np.array([.4617, 0., .887])
    target_normal = unit([.35, .12, .928])
    base_center = np.array([-11.468, .02, 1.0758])
    base_marker = np.array([-11.515, .02, 1.17])
    expected = base_center + normal_alignment(base_normal, target_normal) @ (base_marker - base_center)
    target = dict(center_m=base_center, normal=target_normal, in_plane_rotation_deg=90.)
    np.testing.assert_allclose(marker_center(target), expected, atol=1e-12)


def test_antipodal_heading_turn_is_continuous_and_respects_rate_acceleration():
    turn=HeadingTurn(0.,np.pi,.2,1.3,1.8)
    samples=np.array([turn.sample(t) for t in np.linspace(0,turn.duration,2001)])
    np.testing.assert_allclose(samples[0],[0,0,0,0],atol=1e-12)
    np.testing.assert_allclose(samples[-1],[np.pi,0,0,0],atol=1e-11)
    assert np.max(abs(samples[:,1]))<=1.3+1e-10
    assert np.max(abs(samples[:,2]))<=1.8+1e-10
    assert np.all(np.diff(samples[:,0])>=-1e-12)
    for t in (turn.duration,turn.duration+10):
        np.testing.assert_allclose(turn.sample(t),[np.pi,0,0,0],atol=1e-11)
    # Cross the +/-pi branch cut by 20 degrees, not 340 degrees.
    short=HeadingTurn(np.deg2rad(170),np.deg2rad(-170))
    assert short.sample(short.duration)[0]==pytest.approx(np.deg2rad(190))


def test_course_points_body_travel_axis_forward_and_keeps_thrust():
    f=np.array([2.,-.2,9.81]);body_forward=np.array([-1.,0,0])
    yaw=course_yaw([1.,0.,0.],body_forward)
    R=oriented_frame(f,[np.cos(yaw),np.sin(yaw),0.])
    np.testing.assert_allclose(R[:,2],unit(f),atol=1e-12)
    np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12)
    assert np.linalg.det(R)==pytest.approx(1.)
    assert (R@body_forward)[0]>.95
    # Degenerate heading still returns a proper finite attitude.
    R=oriented_frame([1,0,0],[1,0,0])
    np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12)


def test_default_controller_preserves_frame_and_analytic_yaw_feedforward():
    cfg=load_config();cfg['controller']['rate_reference_source']='analytic'
    m,d,info,_=build_model(scenario(cfg),cfg)
    state=truth_estimate(m,d)
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0.)
    skill=Skill(cfg,cfg['nominal_skill'],state,target,info)
    f=np.array([1.,.2,3.])
    np.testing.assert_array_equal(skill.attitude_reference(f),frame(f))
    skill.set_heading_reference(.7,.8)
    skill.track(state,(state.position,np.zeros(3),np.zeros(3),np.zeros(3)))
    # A constant thrust direction with changing yaw must retain yaw rate.
    np.testing.assert_allclose(skill.last_reference_state[10:13],[0.,0.,.8],atol=1e-7)
    with pytest.raises(ValueError):skill.set_heading_reference(np.nan)


def test_geometry_proxy_uses_rectangular_fov_and_rejects_behind_camera():
    mount=dict(forward_body=[1,0,0],up_reference_body=[0,0,1],position_body_m=[0,0,0],vertical_fov_degrees=60.)
    R=np.repeat(np.eye(3)[None],4,axis=0)
    rays=np.array([[1,0,0],[-1,0,0],[1,.8,0],[1,0,.8]])
    inside,_,_,_=view_geometry(-rays,R,np.zeros(3),mount)
    assert inside.tolist()==[True,False,True,False]


def test_invalid_heading_inputs_fail_early():
    with pytest.raises(ValueError):HeadingTurn(0,1,max_rate_rad_s=0)
    with pytest.raises(ValueError):course_yaw([0,0,1],[-1,0,0])
