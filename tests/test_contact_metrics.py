import numpy as np
import pytest
from types import SimpleNamespace
from rm27.experiments.contact_metrics import armor_contacts, point_approach_speed
from rm27.experiments.contact_probe import probe


def test_rotated_contact_force_and_geometry_order(monkeypatch):
    # Rotate normal from +X to +Y; tangential force must not enter compression.
    frame = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.]])
    model = SimpleNamespace(geom=lambda g: SimpleNamespace(name=str(g)))
    contact = SimpleNamespace(geom=[0, 1], frame=frame.ravel(), pos=np.zeros(3), dist=0.)
    data = SimpleNamespace(ncon=1, contact=[contact])
    def force(m, d, i, out):
        out[:] = [3, 4, 0, 0, 0, 0]
    monkeypatch.setattr('rm27.experiments.contact_metrics.mujoco.mj_contactForce', force)
    row = armor_contacts(model, data, 0, [0, 1, 0])[0]
    assert row['compressive_normal_force_N'] == 3
    np.testing.assert_allclose(row['force_on_armor_world_N'], [4, -3, 0])
    row = armor_contacts(model, data, 1, [0, -1, 0])[0]
    assert row['compressive_normal_force_N'] == 3
    np.testing.assert_allclose(row['force_on_armor_world_N'], [-4, 3, 0])


def test_contact_point_rotation_and_plate_motion():
    assert point_approach_speed([-1, 0, 0], [0, 0, 2], [0, 1, 0], [.5, 0, 0], [1, 0, 0]) == 3.5


@pytest.mark.parametrize('back', [False, True])
def test_force_integral_matches_momentum(back):
    result = probe(.000125, back=back)
    assert result['passed']
    assert result['momentum_relative_error'] < .01
