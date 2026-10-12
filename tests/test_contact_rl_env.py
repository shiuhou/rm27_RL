"""Bounded checks through the contact plant, action chain and shared evaluator."""
from copy import deepcopy
import json
import subprocess
import sys

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27.experiments.aggressive_contact.evaluation import ContactEvaluator
from rm27.experiments.aggressive_contact.model import ROOT, build_model, frame, scenario, truth_estimate, unit
from rm27.experiments.calibration import with_profile
from rm27.experiments.contact_metrics import armor_contacts
from rm27_contact_rl import ContactRecoveryEnv
from rm27_contact_rl.baselines import CTBRBaseline


@pytest.fixture
def env():
    instance = ContactRecoveryEnv()
    instance.reset(seed=23)
    try:
        yield instance
    finally:
        instance.close()


def place_data(model, data, position, rotation=None, velocity=None, omega=None):
    """Set a test initial state; subsequent intervals use real MuJoCo dynamics."""
    rotation = np.eye(3) if rotation is None else rotation
    data.qpos[:3] = position
    xyzw = Rotation.from_matrix(rotation).as_quat()
    data.qpos[3:7] = xyzw[[3, 0, 1, 2]]
    data.qvel[:] = 0
    if velocity is not None:
        data.qvel[:3] = velocity
    if omega is not None:
        data.qvel[3:6] = omega
    mujoco.mj_forward(model, data)


def place(env, position, rotation=None, velocity=None, omega=None):
    place_data(env.plant.model, env.plant.data, position, rotation, velocity, omega)


def fresh_evaluator(env):
    evaluator = ContactEvaluator(env.plant.cfg, env.plant.physical)
    evaluator.initialize(env.plant.model, env.plant.data, env.plant.state)
    return evaluator


def bumper_contact(env, penetration=.001, approach_speed=.65):
    cfg = env.plant.cfg
    normal = unit(cfg["target"]["normal"])
    rotation = frame(normal)
    center = np.asarray(cfg["target"]["center_m"])
    position = (center + normal * (cfg["contact"]["radius_m"] - penetration)
                - rotation @ cfg["contact"]["offset_body_m"])
    place(env, position, rotation, -normal * approach_speed)
    return normal


def test_profile_reset_baseline_and_instance_isolation(env):
    model, data, info = env.plant.model, env.plant.data, env.plant.info
    values = json.loads((ROOT / "configs/dynamics_150g.json").read_text())["values"]
    uid = model.body("uav").id
    assert model.body_mass[uid] == pytest.approx(.15)
    np.testing.assert_allclose(model.body_inertia[uid], values["inertia_kg_m2"])
    np.testing.assert_allclose(model.actuator_ctrlrange[:, 1], values["max_rotor_thrust_N"])
    np.testing.assert_allclose(model.actuator_dynprm[:, 0], values["motor_time_constant_s"])
    np.testing.assert_allclose(model.actuator_gear[:, 5],
                               np.array([1, -1, 1, -1]) * values["yaw_torque_per_thrust_m"])
    assert info["yaw_torque_per_thrust_m"] == values["yaw_torque_per_thrust_m"]
    hover = info["mass_kg"] * 9.81
    initial_observation, _ = env.reset(seed=23)
    assert initial_observation.shape == (26,)
    assert initial_observation.dtype == np.float32
    assert env.observation_space.contains(initial_observation)
    np.testing.assert_allclose(data.act, hover / 4)
    np.testing.assert_allclose(data.ctrl, hover / 4)
    np.testing.assert_allclose(initial_observation[-5:-1], [hover, 0, 0, 0])
    assert data.time == 0 and env.evaluator.completed_intervals == 0
    assert env.evaluator.impulse == 0 and env.evaluator.hold_start is None
    baseline = CTBRBaseline(env)
    np.testing.assert_allclose(baseline.skill.motor_estimate, data.act)
    np.testing.assert_allclose(baseline.skill.previous_motor_command, data.ctrl)
    np.testing.assert_array_equal(baseline.skill.previous_velocity, np.zeros(3))
    np.testing.assert_allclose(baseline.skill.previous_rotation, np.eye(3))
    assert baseline.skill.specific_thrust_ratio == 1
    assert baseline.skill.last_time is None
    baseline.step(env)
    np.testing.assert_array_equal(baseline.skill.previous_motor_command, data.ctrl)

    other = ContactRecoveryEnv()
    try:
        other_initial, _ = other.reset(seed=23)
        assert other.plant.data is not data
        assert other.evaluator is not env.evaluator
        assert other.np_random is not env.np_random
        env.step([0, 0, 0, 0])
        assert other.plant.data.time == 0
        np.testing.assert_allclose(other.plant.data.act, hover / 4)
        np.testing.assert_allclose(other.previous_action_ctbr, [hover, 0, 0, 0])
        assert other.evaluator.completed_intervals == 0
        again, _ = env.reset(seed=23)
        baseline.reset(env)
        np.testing.assert_array_equal(again, other_initial)
        assert env.plant.data.time == 0 and env.evaluator.completed_intervals == 0
        np.testing.assert_allclose(baseline.skill.motor_estimate, data.act)
        np.testing.assert_allclose(baseline.skill.previous_motor_command, data.ctrl)
        assert baseline.skill.last_time is None
    finally:
        other.close()


def test_action_clipping_memory_authority_and_illegal_input(env):
    data = env.plant.data
    unchanged = [data.qpos.copy(), data.qvel.copy(), data.act.copy(),
                 data.ctrl.copy(), env.previous_action_ctbr.copy()]
    for invalid in ([1, 0, 0], [1, np.nan, 0, 0], [np.inf, 0, 0, 0]):
        with pytest.raises(ValueError):
            env.step(invalid)
        assert data.time == 0 and env.evaluator.completed_intervals == 0
        for actual, expected in zip((data.qpos, data.qvel, data.act, data.ctrl,
                                     env.previous_action_ctbr), unchanged):
            np.testing.assert_array_equal(actual, expected)

    action = np.array([env.action_space.high[0] + 10, 100, -100, 100.])
    observation, _, terminated, truncated, info = env.step(action)
    clipped = np.clip(action, env.action_space.low, env.action_space.high)
    assert not terminated and not truncated
    assert info["action_clipped"] and info["physics_substeps"] == 5
    assert info["step_elapsed_s"] == pytest.approx(.005)
    np.testing.assert_allclose(observation[-5:-1], clipped)
    np.testing.assert_allclose(env.previous_action_ctbr, clipped)
    assert np.all(data.ctrl >= 0)
    assert np.all(data.ctrl <= env.plant.info["max_rotor_thrust_N_assumed"])
    # The command observation does not falsely report achieved body rates.
    assert not np.allclose(data.qvel[3:6], clipped[1:])

    other = ContactRecoveryEnv()
    try:
        env.reset(seed=23)
        other.reset(seed=23)
        hover = env.plant.info["mass_kg"] * 9.81
        env.step([hover, 1., 0, 0])
        other.step([0, 0, 0, 0])
        assert data.qvel[3] > 0
        assert data.qvel[2] > other.plant.data.qvel[2]
    finally:
        other.close()


def test_ground_termination_stops_substeps_and_preserves_terminal_result(env):
    data = env.plant.data
    position = data.qpos[:3].copy()
    position[2] = .008
    place(env, position)
    hover = env.plant.info["mass_kg"] * 9.81
    observation, reward, terminated, truncated, info = env.step([hover, 0, 0, 0])
    assert terminated and not truncated
    assert info["physics_substeps"] == 1
    assert data.time == pytest.approx(.001)
    assert info["time_s"] == pytest.approx(data.time)
    assert info["step_elapsed_s"] == pytest.approx(.001)
    assert "GROUND_CONTACT" in info["result"]["failure_modes"]
    assert reward == -10
    assert observation[-1] == pytest.approx(env.plant.cfg["horizon_s"] - data.time)
    final_qpos, final_qvel = data.qpos.copy(), data.qvel.copy()
    result = env.evaluator.finalize(env.plant.model, data)
    for key, value in result.items():
        assert info["result"][key] == value
    with pytest.raises(RuntimeError):
        env.step([hover, 0, 0, 0])
    np.testing.assert_array_equal(data.qpos, final_qpos)
    np.testing.assert_array_equal(data.qvel, final_qvel)
    assert data.time == pytest.approx(.001)
    assert env.evaluator.finalize(env.plant.model, data) == result
    info["result"]["failure_modes"].clear()
    assert "GROUND_CONTACT" in env.evaluator.finalize(env.plant.model, data)["failure_modes"]

    # A short configured task deadline exercises a different terminal cause
    # inside the five-interval policy step, without a long or repeated trial.
    env.reset(seed=23)
    env.horizon_s = .003
    observation, reward, terminated, truncated, info = env.step([hover, 0, 0, 0])
    assert terminated and not truncated
    assert info["physics_substeps"] == 3
    assert info["time_s"] == pytest.approx(.003)
    assert observation[-1] == 0
    assert info["result"]["failure_modes"] == ["TARGET_MISS"]
    assert "GROUND_CONTACT" not in info["result"]["failure_modes"]
    assert reward == -10


def test_interval_force_uses_completed_time_and_refresh_is_idempotent(env):
    model, data = env.plant.model, env.plant.data
    normal = bumper_contact(env, penetration=-.00025)
    evaluator = fresh_evaluator(env)
    assert evaluator.first is None and evaluator.impulse == 0
    assert evaluator.completed_intervals == 0
    before = truth_estimate(model, data)
    mujoco.mj_step(model, data)
    assert not armor_contacts(model, data, model.geom("armor").id, normal)
    evaluator.integrate_interval(model, data, before.timestamp)
    mujoco.mj_forward(model, data)
    evaluator.snapshot(model, data, truth_estimate(model, data), before)
    assert evaluator.first["time_s"] == pytest.approx(.001)
    assert evaluator.first["pre_state_timestamp_s"] == 0
    np.testing.assert_array_equal(evaluator.first["pre_position_world_m"], before.position)
    np.testing.assert_array_equal(evaluator.first["pre_velocity_world_m_s"], before.velocity)
    assert evaluator.peak_force > 0
    assert evaluator.impulse == 0  # The new endpoint force has not acted for an interval yet.
    before = truth_estimate(model, data)
    mujoco.mj_step(model, data)
    force = sum(row["compressive_normal_force_N"] for row in
                armor_contacts(model, data, model.geom("armor").id, normal))
    assert force > 0
    evaluator.integrate_interval(model, data, before.timestamp, saturated=True)
    actual_dt = float(data.time) - before.timestamp
    assert evaluator.impulse == pytest.approx(force * actual_dt)
    assert evaluator.completed_intervals == 2
    with pytest.raises(ValueError):
        evaluator.integrate_interval(model, data, before.timestamp, saturated=True)
    assert evaluator.completed_intervals == 2
    assert evaluator.impulse == pytest.approx(force * actual_dt)
    mujoco.mj_forward(model, data)
    evaluator.snapshot(model, data, truth_estimate(model, data), before)
    original = evaluator.summary(model, data)
    for _ in range(2):
        mujoco.mj_forward(model, data)
        evaluator.snapshot(model, data, truth_estimate(model, data), before)
        assert evaluator.summary(model, data) == original
        assert evaluator.completed_intervals == 2
        assert data.time == pytest.approx(2 * actual_dt)


def test_simultaneous_wrong_contact_cannot_be_overwritten(env):
    cfg = deepcopy(env.plant.cfg)
    # A dedicated short-bumper fixture creates simultaneous designated and guard contacts.
    cfg["contact"]["offset_body_m"] = [0, 0, -.019]
    physical = scenario(cfg)
    vehicle = with_profile(json.loads(env.sources["vehicle_config"].read_text()),
                           env.sources["dynamics_profile"])
    model, data, _, _ = build_model(physical, cfg, vehicle_config=vehicle)
    data.ctrl[:] = data.act
    evaluator = ContactEvaluator(cfg, physical)
    normal = unit(cfg["target"]["normal"])
    rotation = frame(normal)
    # Bring the body deep enough for the real guard/rim pair alongside bumper/armor.
    center = np.asarray(cfg["target"]["center_m"])
    place_data(model, data, center + normal * .016, rotation, -normal * .65)
    evaluator.initialize(model, data, truth_estimate(model, data))
    first = deepcopy(evaluator.first)
    assert first is not None
    pairs = [set(names) for names in first["pairs"]]
    assert {"armor", "bumper"} in pairs
    assert any(pair != {"armor", "bumper"} for pair in pairs)
    assert not first["valid_part"]
    assert "WRONG_FIRST_CONTACT" in evaluator.reasons
    place_data(model, data, center + [0.5, 0, .3])
    evaluator.advance(model, data, truth_estimate(model, data))
    assert evaluator.first == first
    assert "WRONG_FIRST_CONTACT" in evaluator.reasons
    assert not evaluator.finalize(model, data)["passed"]


def test_brief_contact_survives_until_next_policy_boundary(env):
    model, data = env.plant.model, env.plant.data
    normal = bumper_contact(env, penetration=1e-5, approach_speed=-.65)
    # Initial contact is intentionally outgoing: only contact-latch persistence is at issue.
    evaluator = fresh_evaluator(env)
    assert evaluator.first is not None and evaluator.contact_latch
    for _ in range(5):
        evaluator.advance(model, data, truth_estimate(model, data))
    actual_contacts = armor_contacts(model, data, model.geom("armor").id, normal)
    assert not any(row["distance_m"] <= 0 for row in actual_contacts)
    assert evaluator.contact_latch
    assert evaluator.first["time_s"] < data.time
    assert evaluator.completed_intervals == 5


def test_hold_resets_and_recovery_time_marks_stable_start(env):
    model, data = env.plant.model, env.plant.data
    bumper_contact(env)
    evaluator = fresh_evaluator(env)
    assert evaluator.first is not None and not evaluator.reasons
    center = np.asarray(env.plant.cfg["target"]["center_m"])
    place(env, center + [.4, 0, .3])
    evaluator.advance(model, data, truth_estimate(model, data))
    first_hold = evaluator.hold_start
    assert first_hold is not None
    for _ in range(10):
        evaluator.advance(model, data, truth_estimate(model, data))
    assert evaluator.recovery_time is None
    position = data.qpos[:3].copy()
    place(env, position, velocity=[.4, 0, 0])
    evaluator.advance(model, data, truth_estimate(model, data))
    assert evaluator.hold_start is None
    place(env, data.qpos[:3].copy())
    evaluator.advance(model, data, truth_estimate(model, data))
    second_hold = evaluator.hold_start
    assert second_hold is not None and second_hold > first_hold
    hold = env.plant.cfg["gates"]["recovery_hold_s"]
    dt = env.plant.cfg["physics_dt_s"]
    while float(data.time) < second_hold + hold - dt / 2:
        evaluator.advance(model, data, truth_estimate(model, data))
    if evaluator.recovery_time is None:
        evaluator.advance(model, data, truth_estimate(model, data))
    assert evaluator.recovery_time == pytest.approx(second_hold - evaluator.first["time_s"])
    assert evaluator.separated
    assert evaluator.finalize(model, data)["passed"]
    assert sum(event["event"] == "RECOVERED" for event in evaluator.events) == 1


def test_installed_environment_loads_configuration_outside_repository(tmp_path):
    command = (
        "from rm27_contact_rl import ContactRecoveryEnv; "
        "env=ContactRecoveryEnv(); observation,info=env.reset(seed=23); "
        "assert observation.shape==(26,); assert env.plant.info['mass_kg']==.15; env.close()"
    )
    result = subprocess.run([sys.executable, "-c", command], cwd=tmp_path,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
