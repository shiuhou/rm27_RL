"""Global path continuity, early thrust direction and compact physical evidence."""
import hashlib

import mujoco
import numpy as np

from rm27.experiments.aggressive_contact.control import Septic
from rm27.experiments.aggressive_contact.maneuver import MinimumSnap, ManeuverSkill, settings
from rm27.experiments.aggressive_contact.model import TargetState, build_model, load_config, scenario, truth_estimate, unit
from rm27.experiments.aggressive_contact.recording import save_overlay, load_model


def test_single_segment_global_solution_matches_septic():
    start = np.array([[0, 0, 1], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
    end = np.array([[3, 0, 2], [-.3, 0, -.5], [5, 0, .5], [0, 0, 0]])
    a, b = MinimumSnap(start, end, [], [2.]), Septic(start, end, 2.)
    for t in np.linspace(0, 2, 11):
        np.testing.assert_allclose(a.sample(t), b.sample(t), atol=1e-10)


def test_global_seams_share_nonzero_velocity_acceleration_and_jerk():
    start = np.zeros((4, 3)); start[0, 2] = 2
    end = start.copy(); end[0] = [-12, 0, 1.2]; end[1] = [-.3, 0, -.5]; end[2] = [5, 0, .5]
    path = MinimumSnap(start, end, [[-1, 1, 2.6], [-9, .5, 2.2]], [1.2, 3, 1.6])
    for i in range(2):
        np.testing.assert_allclose(path.segments[i].sample(path.segments[i].duration), path.segments[i+1].sample(0), atol=1e-8)
        for d in (1, 2, 3):
            assert np.linalg.norm(path.knot_states[i+1, d]) > .05


def test_early_alignment_is_dynamical_and_continuous():
    cfg = load_config()
    m, d, info, _ = build_model(scenario(cfg), cfg)
    s = truth_estimate(m, d)
    n = unit(cfg['target']['normal'])
    target = TargetState(np.array(cfg['target']['center_m']), n, np.zeros((3, 3)), 0.)
    design = dict(settings()['initial_design'], alignment_lead_s=.4)
    skill = ManeuverSkill(cfg, design, s, target, info, np.array([12, 1, 1.74]))
    path = skill.trajectory
    np.testing.assert_allclose(path.first.sample(path.split), path.second.sample(0), atol=1e-8)
    for t in np.linspace(path.split, skill.p['duration_s'], 6):
        acc = path.sample(t)[2]
        np.testing.assert_allclose(unit(acc+[0, 0, 9.81]), n, atol=1e-10)
        assert np.linalg.norm(path.sample(t)[1]) > .1


def test_shared_model_overlay_reconstructs_exact_physics(tmp_path):
    cfg = load_config()
    model, data, _, _ = build_model(scenario(cfg), cfg)
    template = tmp_path/'template.mjb'
    mujoco.mj_saveModel(model, str(template))
    model.body_mass[model.body('uav').id] *= 1.03
    model.body_inertia[model.body('uav').id] *= 1.03
    model.actuator_gear[:] *= .95
    model.actuator_dynprm[:, 0] *= 1.2
    mujoco.mj_setConst(model, data)
    save_overlay(model, tmp_path, template, hashlib.sha256(template.read_bytes()).hexdigest())
    restored = load_model(tmp_path)
    a, b = mujoco.MjData(model), mujoco.MjData(restored)
    for d in (a, b):
        d.ctrl[:] = [.4, .41, .42, .39]
        d.act[:] = .4
    for _ in range(100):
        mujoco.mj_step(model, a)
        mujoco.mj_step(restored, b)
    np.testing.assert_array_equal(a.qpos, b.qpos)
    np.testing.assert_array_equal(a.qvel, b.qvel)
    np.testing.assert_array_equal(a.act, b.act)


def test_event_finish_keeps_complete_recovery_hold_and_replay(tmp_path):
    from rm27.experiments.aggressive_contact.experiment import run_episode
    from rm27.experiments.aggressive_contact.visualize import replay
    cfg = load_config()
    result = run_episode(cfg, output=tmp_path/'flight', stop_on_recovery=True)
    assert result['passed']
    event = next(e for e in result['events'] if e['event'] == 'RECOVERED')
    assert result['duration_s'] == event['time_s']
    assert event['time_s']-event['stable_since_s'] >= cfg['gates']['recovery_hold_s']
    assert result['duration_s'] < cfg['horizon_s']
    trace = np.load(tmp_path/'flight/trajectory.npz')['trace']
    assert trace[-1, 0] == result['duration_s']
    assert replay(tmp_path/'flight', tmp_path/'replay')['passed']
