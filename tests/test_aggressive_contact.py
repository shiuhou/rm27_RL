"""Research checks exercise endpoint mechanics, feedback, physics and evidence."""
import copy
import json
import subprocess
import sys

import numpy as np
import pytest

from rm27.experiments.aggressive_contact.control import Septic, Skill
from rm27.experiments.aggressive_contact.experiment import objective, run_episode
from rm27.experiments.aggressive_contact.model import TargetState, build_model, frame, load_config, scenario, truth_estimate, unit
from rm27.experiments.aggressive_contact.recovery import recover
from rm27.experiments.aggressive_contact.visualize import replay


def test_septic_endpoints_and_contact_frame():
    rng = np.random.default_rng(83)
    start, end = rng.normal(size=(4, 3)), rng.normal(size=(4, 3))
    trajectory = Septic(start, end, 1.7)
    np.testing.assert_allclose(trajectory.sample(0), start, atol=1e-10)
    np.testing.assert_allclose(trajectory.sample(1.7), end, atol=1e-9)
    for n in ([.4617, 0, .887], [1, 0, 0], [0, 0, 1]):
        R = frame(n)
        np.testing.assert_allclose(R.T@R, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(R[:, 2], unit(n))
        assert np.linalg.det(R) == pytest.approx(1)


def test_terminal_pose_follows_acceleration_and_bumper_offset():
    cfg = load_config()
    model, data, info, _ = build_model(scenario(cfg), cfg)
    t = TargetState(np.array(cfg['target']['center_m']), unit(cfg['target']['normal']), np.zeros((3, 3)), 0)
    skill = Skill(cfg, cfg['nominal_skill'], truth_estimate(model, data), t, info)
    p, v, a, jerk = skill.trajectory.sample(cfg['nominal_skill']['duration_s'])
    R = frame(a+[0, 0, 9.81])
    np.testing.assert_allclose(R[:, 2], t.normal, atol=1e-10)
    pc = p + R @ cfg['contact']['offset_body_m']
    np.testing.assert_allclose(pc, t.position+t.normal*(cfg['contact']['radius_m']-cfg['nominal_skill']['penetration_m']), atol=1e-10)
    assert -v@t.normal == pytest.approx(cfg['nominal_skill']['normal_speed_m_s'])


def test_scenario_determinism_and_hidden_physics():
    cfg = load_config()
    assert scenario(cfg, 101) == scenario(cfg, 101)
    a, b = scenario(cfg, 101), scenario(cfg, 102)
    assert a != b
    assert cfg['target']['center_m'] == [0., 0., 1.0758]
    m1, d1, info1, _ = build_model(a, cfg)
    m2, d2, info2, _ = build_model(b, cfg)
    assert info1 == info2  # Controller cannot read randomized plant parameters.
    assert m1.body_mass[1:].sum() != m2.body_mass[1:].sum()


def test_nominal_physical_contact_recovery_and_motor_replay(tmp_path):
    cfg = load_config()
    result = run_episode(cfg, output=tmp_path/'episode')
    assert result['passed'], result['failure_modes']
    assert result['first_contact']['pairs'] == [['bumper', 'armor']]
    assert result['normal_impulse_Ns'] > 0
    assert result['separated'] and result['recovery_time_s'] < 2
    assert replay(tmp_path/'episode', tmp_path/'replay')['passed']
    # Refuse accidental overwrite, including previously failed episodes.
    with pytest.raises(FileExistsError):
        run_episode(cfg, output=tmp_path/'episode')


def test_miss_and_stale_estimate_cannot_pass():
    cfg = load_config()
    physical = scenario(cfg)
    physical['target']['center_m'][1] += .35
    result = run_episode(cfg, physical=physical)
    assert not result['passed'] and 'TARGET_MISS' in result['failure_modes']
    cfg['uncertainty']['target_latency_s'] = .5
    result = run_episode(cfg)
    assert not result['passed'] and 'ESTIMATOR_FAILURE' in result['failure_modes']


def test_short_bumper_cannot_hide_guard_or_fuselage_first_contact():
    cfg = load_config()
    cfg['contact']['offset_body_m'][2] = -.005
    result = run_episode(cfg)
    assert not result['passed']
    assert 'WRONG_FIRST_CONTACT' in result['failure_modes']


def test_recovery_initial_state_and_joint_objective():
    cfg = load_config()
    initial = dict(roll_deg=0, pitch_deg=15, altitude_m=1.2, velocity_m_s=[0, 0, 0],
                   omega_rad_s=[0, 0, 0], motor_hover_fraction=1)
    assert recover(cfg, initial)['passed']
    result = run_episode(cfg)
    failed = copy.deepcopy(result)
    failed.update(passed=False, recovery_time_s=None, secondary_contact=True)
    assert objective(failed, False) == objective(result, False)
    assert objective(failed, True) > objective(result, True)


def test_cli_preserves_bad_config_traceback(tmp_path):
    cfg = tmp_path/'invalid.json'
    cfg.write_text('{"physics_dt_s": -1, "control_dt_s": .005}')
    output = tmp_path/'failed'
    proc = subprocess.run([sys.executable, '-m', 'rm27.experiments.aggressive_contact', 'episode',
                           '--config', str(cfg), '--output', str(output)], capture_output=True, text=True)
    assert proc.returncode != 0
    assert not json.loads((output/'summary.json').read_text())['passed']
    assert (output/'traceback.log').read_text()
    assert (output/'config-input.json').read_bytes() == cfg.read_bytes()
