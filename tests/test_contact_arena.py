"""Geometry orientation, observer isolation, trajectory seams and viewer safety."""
from contextlib import nullcontext
import copy
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from rm27.experiments.aggressive_contact.arena import Piecewise, compose, transfer_controller
from rm27.experiments.aggressive_contact.control import Skill
from rm27.experiments.aggressive_contact.model import ROOT, TargetState, build_model, load_config, scenario, target_frame, truth_estimate, unit
from rm27.experiments.aggressive_contact.robust import configured


def controller(cfg, draw=None):
    m, d, info, xml = build_model(scenario(cfg) if draw is None else draw, cfg)
    target = TargetState(np.array(cfg['target']['center_m']), unit(cfg['target']['normal']), np.zeros((3, 3)), 0.)
    return m, d, Skill(cfg, cfg['nominal_skill'], truth_estimate(m, d), target, info)


def test_actual_armor_axes_and_half_extents(tmp_path):
    cfg = load_config()
    cfg['target']['in_plane_rotation_deg'] = 90
    m, d, _ = controller(cfg)
    R = d.geom_xmat[m.geom('armor').id].reshape(3, 3)
    np.testing.assert_allclose(R[:, 0], [0, 1, 0], atol=1e-10)
    np.testing.assert_allclose(m.geom_size[m.geom('armor').id], [.045, .0275, .003])
    np.testing.assert_allclose(R, target_frame(cfg['target']), atol=1e-12)


def test_actuator_observer_cannot_read_hidden_physics():
    cfg = configured(load_config(), 'adaptive_nominal')
    m, d, a = controller(cfg, scenario(cfg, 301, 2))
    m2, d2, b = controller(cfg, scenario(cfg, 302, 2))
    state = truth_estimate(m, d)
    # Same actor inputs/history must yield the same actions despite different hidden plants.
    for i in range(20):
        state = type(state)(state.position, state.velocity, state.rotation, state.omega_body, i*.005)
        ua, _ = a.track(state, a.trajectory.sample(0))
        ub, _ = b.track(state, a.trajectory.sample(0))
        # Initialize both observed histories identically at the first tick.
        if i == 0:
            transfer_controller(a, b)
        else:
            np.testing.assert_allclose(ua, ub, atol=1e-12)
    assert .7 <= a.specific_thrust_ratio <= 1.3


def test_piecewise_pvaj_continuity_and_nonzero_join_velocity():
    states = [(np.array(p), np.array(v), np.zeros(3), np.zeros(3)) for p, v in
              [([0, 0, 1], [0, 0, 0]), ([1, 0, 2], [1, 0, 0]), ([3, 0, 1], [0, 0, 0])]]
    path = Piecewise(states, [2., 2.])
    np.testing.assert_allclose(path.segments[0].sample(2.), path.segments[1].sample(0.), atol=1e-10)
    assert np.linalg.norm(path.sample(2.)[1]) == pytest.approx(1.)


def test_nonzero_time_terminal_and_contact_freezes_adaptation():
    cfg = configured(load_config(), 'adaptive_nominal')
    m, d, skill = controller(cfg)
    d.time = 12.
    state = truth_estimate(m, d)
    target = TargetState(np.array(cfg['target']['center_m']), unit(cfg['target']['normal']), np.zeros((3, 3)), 12.)
    delayed = Skill(cfg, cfg['nominal_skill'], state, target, {
        'mass_kg':skill.mass, 'inertia_kg_m2':skill.inertia, 'max_rotor_thrust_N_assumed':skill.max_motor,
        'rotor_arm_m':.03, 'yaw_torque_per_thrust_m':.018})
    delayed.command(state, False)
    assert delayed.phase == 'FREE_FLIGHT'
    previous = delayed.specific_thrust_ratio
    state = type(state)(state.position, state.velocity+np.array([10, 0, 10]), state.rotation, state.omega_body, 12.005)
    delayed.command(state, True)
    assert delayed.specific_thrust_ratio == previous


def test_live_view_uses_private_model_and_data(monkeypatch):
    import mujoco.viewer
    from rm27.experiments.aggressive_contact.viewing import LiveView
    m, d, _ = controller(load_config())
    original = d.qpos.copy()
    def launch(model, data, **kwargs):
        def sync(**kwargs):
            data.qpos[:3] = 99  # simulate UI perturbation
        return SimpleNamespace(lock=nullcontext, cam=SimpleNamespace(lookat=np.zeros(3)),
                               opt=SimpleNamespace(geomgroup=np.ones(6)), is_running=lambda:True,
                               sync=sync, set_texts=lambda x:None, close=lambda:None)
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', launch)
    view = LiveView(m, d)
    assert view.model is not m
    view.update(d, 'TEST', force=True)
    np.testing.assert_array_equal(d.qpos, original)
    view.key(ord('V'))
    assert view.overview
    view.close()


@pytest.mark.skipif(not (ROOT/'artifacts/dock-final/scene.xml').exists(), reason='Local raw scene absent')
def test_full_arena_composition_preserves_field_and_one_active_vehicle(tmp_path):
    cfg, bundle, slot, settings = compose(load_config(), 'full', tmp_path)
    m, d, info, xml = bundle
    assert m.nq == 7 and m.nv == 6 and m.nu == 4
    assert slot[0] > 0 and cfg['target']['center_m'][0] < 0
    assert cfg['target']['in_plane_rotation_deg'] == 90
    np.testing.assert_allclose(d.site_xpos[m.site('slot_1').id], slot)
