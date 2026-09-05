import ast
from pathlib import Path

import numpy as np
import pytest

from astra.geometry import camera_basis, project, unproject, limit_specific_force, swept_distance
from astra.interfaces import EgoEstimate, ImageObservation
from astra.autonomy import Guidance
from astra.config import Config
from astra.simulation import simulate


def test_camera_roundtrip_and_body_pitch():
    up = np.array([[0., 0., 1.]])
    basis = camera_basis(up, np.zeros(1), 0.)
    np.testing.assert_allclose(basis[0, :, 2], [1, 0, 0])
    rel = np.array([[4., -.4, .2]])
    uv, angle, visible = project(rel, basis, .1, 90., 640, 480, 2.)
    assert visible[0]
    np.testing.assert_allclose(unproject(uv, angle, basis, .1, 90., 640), rel, atol=1e-10)
    pitched = camera_basis(np.array([[.5, 0., np.sqrt(.75)]]), np.zeros(1), 0.)
    assert pitched[0, 2, 2] < 0  # Forward acceleration pitches fixed camera down.


def test_projection_rejects_behind_outside_and_small():
    b = camera_basis(np.tile([0., 0., 1.], (3, 1)), np.zeros(3), 0.)
    _, _, visible = project(np.array([[-1, 0, 0], [1, 3, 0], [1000, 0, 0]]), b, .1, 60, 640, 480, 2)
    assert not visible.any()


def test_off_axis_clipped_sphere_has_no_size_measurement():
    # Center is inside, but surface crosses the right frustum plane.
    _, _, visible = project(np.array([[.866, .6, 1.]]), np.eye(3)[None], .1, 90, 640, 480, 2)
    assert not visible[0]


def test_compensated_pursuit_extrapolates_without_intercept_lead():
    e = EgoEstimate(.1, np.zeros((1, 3)), np.zeros((1, 3)), np.eye(3)[None])
    g = Guidance(1, 'compensated_pursuit', Config())
    g.seen[:] = True
    g.position[:] = [4, 0, 0]
    g.velocity[:] = [0, 2, 0]
    g.epoch[:] = 0
    acceleration, _ = g.command(e)
    assert acceleration[0, 1]/acceleration[0, 0] == pytest.approx(.2/4)


def test_force_limits_and_hover():
    force = limit_specific_force(np.array([[100., 100., 50.], [0, 0, 9.81]]), np.array([1.5, 3]), 45)
    assert np.linalg.norm(force[0]) <= 1.5 * 9.81 + 1e-10
    assert np.linalg.norm(force[0, :2]) <= force[0, 2] + 1e-10
    np.testing.assert_allclose(force[1], [0, 0, 9.81])


def test_swept_distance_detects_between_step_contact():
    d, fraction = swept_distance(np.array([[1., 0, 0]]), np.array([[-1., 0, 0]]))
    assert d[0] == 0
    assert fraction[0] == .5


def test_guidance_never_uses_undelivered_observation():
    c = Config()
    e = EgoEstimate(0., np.zeros((1, 3)), np.zeros((1, 3)), np.eye(3)[None])
    b = camera_basis(np.array([[0., 0, 1.]]), np.zeros(1), 0)
    obs = ImageObservation(0., .08, np.zeros((1, 2)), np.array([.02]), np.ones(1, bool), e.position.copy(), b)
    g = Guidance(1, 'lead', c)
    with pytest.raises(ValueError, match='delivered'):
        g.observe(obs, 0.)
    a, _ = g.command(e)
    np.testing.assert_allclose(a, 0)


def test_autonomy_has_no_truth_imports():
    for module in ['autonomy', 'interfaces', 'geometry']:
        tree = ast.parse((Path(__file__).parents[1] / f'{module}.py').read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert not any(s in (node.module or '') for s in ('simulation', 'scoring', 'scenario'))


def test_deterministic_replay_and_blind_sensor():
    cfg = Config(duration_s=2., camera_dropout=1., ego_velocity_noise=0., ego_drift_m_s=0.)
    a = simulate(cfg, 'lead', 8, 123)
    b = simulate(cfg, 'lead', 8, 123)
    assert a == b
    assert all(r['observations'] == 0 for r in a)
    assert all(r['max_speed_m_s'] == 0 for r in a)


def test_static_center_contact_in_ideal_case():
    cfg = Config(duration_s=10., pixel_noise=0., radius_noise_fraction=0., camera_dropout=0., ego_velocity_noise=0., ego_drift_m_s=0.)
    rows = simulate(cfg, 'pursuit', 1, 1, scenario='static')
    assert rows[0]['success']
    assert rows[0]['max_tilt_deg'] <= cfg.tilt_deg + 1e-6
