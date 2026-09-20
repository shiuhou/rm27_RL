import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import pytest
from rm27.experiments.calibration import load_profile, with_profile, verify_compiled
from rm27.integration.drone import add

ROOT = Path(__file__).resolve().parents[1]


def compile_config(cfg):
    root = ET.fromstring('<mujoco><worldbody/></mujoco>')
    add(root, 'test', [0, 0, 1], [1, 0, 0, 0], cfg)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def test_nominal_profile_preserves_compiled_dynamics():
    cfg = json.loads((ROOT/'configs/drone.json').read_text())
    a = compile_config(cfg)
    b = compile_config(with_profile(cfg, ROOT/'configs/dynamics_provisional.json'))
    for name in ('body_mass', 'body_inertia', 'actuator_gear', 'actuator_ctrlrange', 'actuator_dynprm'):
        np.testing.assert_allclose(getattr(a, name), getattr(b, name), atol=1e-15, rtol=1e-14)


def test_profile_rejects_invalid_inertia_and_missing_evidence(tmp_path):
    profile = json.loads((ROOT/'configs/dynamics_provisional.json').read_text())
    path = tmp_path/'profile.json'
    profile['values']['inertia_kg_m2'] = [1, 1, 5]
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError, match='triangle'):
        load_profile(path)
    profile['status'] = 'fitted_candidate'
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError, match='evidence'):
        load_profile(path)


def test_changed_dynamics_reach_compiled_model(tmp_path):
    profile = json.loads((ROOT/'configs/dynamics_provisional.json').read_text())
    profile['values'].update(mass_kg=.2, max_rotor_thrust_N=.9, motor_time_constant_s=.06)
    path = tmp_path/'profile.json'
    path.write_text(json.dumps(profile))
    cfg = with_profile(json.loads((ROOT/'configs/drone.json').read_text()), path)
    model = compile_config(cfg)
    assert model.body_mass[1] == pytest.approx(.2)
    assert np.all(model.actuator_ctrlrange[:, 1] == .9)
    assert np.all(model.actuator_dynprm[:, 0] == .06)
    verify_compiled(model, 1, np.arange(4), profile)
    with pytest.raises(ValueError, match='differ'):
        verify_compiled(model, 1, np.arange(4), load_profile(ROOT/'configs/dynamics_provisional.json'))
