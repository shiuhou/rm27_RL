"""Bounded arrival checks using the shared native-MuJoCo execution path."""
from copy import deepcopy

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27_contact_physics.arrival import ArrivalEvaluator
from rm27_contact_rl.arrival_env import AgileTerminalArrivalEnv


@pytest.fixture
def env():
    instance = AgileTerminalArrivalEnv()
    instance.reset(seed=23, options={"initial_coordinates": [1.2, 0, 0]})
    try:
        yield instance
    finally:
        instance.close()


def place(env, position, rotation=None, velocity=None, omega=None):
    """Supply test initial conditions; production steps still use real dynamics."""
    rotation = np.eye(3) if rotation is None else rotation
    data = env.plant.data
    data.qpos[:3] = position
    xyzw = Rotation.from_matrix(rotation).as_quat()
    data.qpos[3:7] = xyzw[[3, 0, 1, 2]]
    data.qvel[:] = 0
    if velocity is not None:
        data.qvel[:3] = velocity
    if omega is not None:
        data.qvel[3:6] = omega
    mujoco.mj_forward(env.plant.model, data)


def fresh_evaluator(env, task=None):
    evaluator = ArrivalEvaluator(env.rl_config if task is None else task,
                                 env.plant.physical)
    evaluator.initialize(env.plant.model, env.plant.data, env.plant.state)
    env.evaluator = evaluator
    return evaluator


def near_arrival(env):
    geometry = env.evaluator.geometry
    # Outside the arrival gap tolerance at t=0; the real first interval enters it.
    position = geometry["position_target"] + .0225 * geometry["normal"]
    place(env, position, geometry["target_rotation"], geometry["velocity_target"])
    fresh_evaluator(env)
    assert not env.evaluator.done


def test_front_surface_geometry_and_material_velocity_are_distinct(env):
    geometry = env.evaluator.geometry
    model, data = env.plant.model, env.plant.data
    armor = model.geom("armor").id
    normal, center = geometry["normal"], geometry["center"]
    half_thickness = model.geom_size[armor, 2]
    np.testing.assert_allclose(data.geom_xpos[armor] + half_thickness * normal,
                               center, atol=1e-12)
    np.testing.assert_allclose(data.geom_xpos[armor] - half_thickness * normal,
                               center - 2 * half_thickness * normal, atol=1e-12)
    np.testing.assert_allclose(geometry["bumper_target"] - geometry["radius"] * normal,
                               center + .20 * normal, atol=1e-12)

    omega = np.array([.2, -.4, .6])
    velocity = np.array([.7, -.3, .2])
    rotation = geometry["target_rotation"]
    place(env, geometry["position_target"], rotation, velocity, omega)
    metrics = env.evaluator.metrics(env.plant.state)
    np.testing.assert_allclose(metrics["bumper_center"], geometry["bumper_target"])
    assert metrics["plane_gap_m"] == pytest.approx(.2, abs=1e-12)
    # This difference is rigid material rotation on the sphere, not dq/dt.
    np.testing.assert_allclose(
        metrics["material_velocity_at_facing_point"] - metrics["bumper_center_velocity"],
        np.cross(rotation @ omega, -geometry["radius"] * normal), atol=1e-12)
    assert not np.allclose(metrics["material_velocity_at_facing_point"],
                           metrics["bumper_center_velocity"])


def test_reset_observation_permissions_and_instance_isolation(env):
    other = AgileTerminalArrivalEnv()
    try:
        observation, info = env.reset(seed=23)
        other_observation, _ = other.reset(seed=23)
        np.testing.assert_array_equal(observation, other_observation)
        state, geometry = env.plant.state, env.evaluator.geometry
        expected = np.r_[geometry["position_target"] - state.position, geometry["normal"],
                         state.velocity, state.rotation.ravel(), state.omega_body,
                         [env.plant.hover_force, 0, 0, 0], env.horizon_s].astype(np.float32)
        np.testing.assert_array_equal(observation, expected)
        assert observation.shape == (26,) and env.observation_space.contains(observation)
        assert info["task_id"] == "agile_terminal_arrival_v1"
        assert info["observation_schema"] == "arrival_world_v1"
        assert env.task_id != "contact_recovery_v1"
        np.testing.assert_allclose(env.plant.data.act, env.plant.hover_force / 4)
        np.testing.assert_allclose(env.plant.data.ctrl, env.plant.hover_force / 4)
        coordinates = np.asarray(info["initial_coordinates"])
        assert 1.15 <= coordinates[0] <= 1.25
        assert np.all(np.abs(coordinates[1:]) <= .025)

        # Actor reads current plant state and task geometry, not cached scoring.
        env.evaluator.last_metrics["velocity_error_m_s"] = 999.
        env.evaluator.events.append({"event": "SCORING_DIAGNOSTIC_ONLY"})
        np.testing.assert_array_equal(env._observation(), observation)
        assert env.plant.data is not other.plant.data
        assert env.evaluator is not other.evaluator and env.np_random is not other.np_random
        env.step(env.previous_action_ctbr.copy())
        assert other.plant.data.time == 0 and other.evaluator.completed_intervals == 0
        np.testing.assert_allclose(other.previous_action_ctbr, [other.plant.hover_force, 0, 0, 0])
        env.reset(seed=23, options={"initial_coordinates": [1.2, 0, 0]})
        assert env.evaluator.completed_intervals == 0 and env.evaluator.events == []
        assert env.plant.data.time == 0
        with pytest.raises(ValueError):
            env.reset(options={"initial_coordinates": [2, 0, 0]})
    finally:
        other.close()


def test_real_step_arrival_terminates_early_and_rewards_event_once(env):
    near_arrival(env)
    observation, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert terminated and not truncated
    assert info["physics_substeps"] == 1 and info["time_s"] == pytest.approx(.001)
    assert info["step_elapsed_s"] == pytest.approx(.001)
    assert observation[-1] == pytest.approx(env.horizon_s - .001)
    result = info["result"]
    assert result["passed"] and result["termination_reason"] == "ARRIVAL_SUCCESS"
    assert result["arrival_time_s"] == pytest.approx(.001)
    assert not result["contact_recovery_success"]
    assert result["completed_physics_intervals"] == 1
    assert info["reward_components"]["event"] == 20
    assert reward == pytest.approx(sum(info["reward_components"].values()))
    assert sum(event["event"] == "ARRIVAL_SUCCESS" for event in result["events"]) == 1
    final_position = env.plant.data.qpos.copy()
    with pytest.raises(RuntimeError):
        env.step(env.previous_action_ctbr.copy())
    np.testing.assert_array_equal(env.plant.data.qpos, final_position)
    assert env.plant.data.time == pytest.approx(.001)


def test_all_terminal_gates_share_one_state_and_safety_has_priority(env):
    geometry = env.evaluator.geometry
    place(env, geometry["position_target"], geometry["target_rotation"], np.zeros(3))
    evaluator = fresh_evaluator(env)
    assert evaluator.last_metrics["plane_gap_error_m"] == pytest.approx(0, abs=1e-12)
    assert not evaluator.arrived  # position and attitude alone are insufficient
    # A different snapshot has the required velocity but is far from the goal.
    env.step(env.previous_action_ctbr.copy())
    place(env, geometry["position_target"] + .1 * geometry["normal"],
          geometry["target_rotation"], geometry["velocity_target"])
    env.step(env.previous_action_ctbr.copy())
    assert not evaluator.arrived

    env.reset(seed=23, options={"initial_coordinates": [1.2, 0, 0]})
    near_arrival(env)
    task = deepcopy(env.rl_config)
    # Narrow a test-only safety boundary so entry into the goal is simultaneously unsafe.
    position = env.plant.state.position.copy()
    task["safety"]["position_min_m"][0] = position[0] - .0005
    evaluator = fresh_evaluator(env, task)
    assert not evaluator.done
    _, _, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert terminated and not truncated and info["physics_substeps"] == 1
    assert "FLIGHT_VOLUME_EXIT" in info["result"]["failure_modes"]
    assert not info["result"]["passed"] and not evaluator.arrived
    assert not any(event["event"] == "ARRIVAL_SUCCESS" for event in evaluator.events)


def test_completed_interval_reward_and_refresh_are_idempotent(env):
    before = env.plant.state
    initial_position = before.position.copy()
    _, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert not terminated and not truncated
    assert info["physics_substeps"] == 5 and info["step_elapsed_s"] == pytest.approx(.005)
    assert info["reward_components"]["event"] == 0
    np.testing.assert_allclose(env.plant.state.position, initial_position, atol=1e-12)
    assert info["reward_components"]["progress"] == pytest.approx(0, abs=1e-12)
    assert reward == pytest.approx(sum(info["reward_components"].values()))
    # Stationary nominal hover has constant state: compare 1 ms and 5 ms costs
    # through the same production reward function, rather than another scorer.
    metrics = env.evaluator.metrics(env.plant.state)
    one_interval = env._process_reward(metrics, metrics, .001, False)
    assert info["reward_components"]["cost"] == pytest.approx(5 * one_interval["cost"])
    original = env.evaluator.summary(env.plant.model, env.plant.data)
    totals = deepcopy(env.reward_component_totals)
    for _ in range(2):
        mujoco.mj_forward(env.plant.model, env.plant.data)
        env.evaluator.snapshot(env.plant.model, env.plant.data, env.plant.state)
        assert env.evaluator.summary(env.plant.model, env.plant.data) == original
        assert env.evaluator.completed_intervals == 5
        assert env.reward_component_totals == totals
        assert env.plant.data.time == pytest.approx(.005)
    with pytest.raises(ValueError):
        env.evaluator.integrate_interval(env.plant.model, env.plant.data, before.timestamp)
    assert env.evaluator.completed_intervals == 5


def test_short_timeout_cost_uses_only_elapsed_intervals(env):
    task = deepcopy(env.rl_config)
    task["horizon_s"] = .003
    env.horizon_s = .003
    fresh_evaluator(env, task)
    observation, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert terminated and not truncated and info["physics_substeps"] == 3
    assert observation[-1] == 0 and info["time_s"] == pytest.approx(.003)
    assert info["result"]["failure_modes"] == ["ARRIVAL_TIMEOUT"]
    assert info["reward_components"]["event"] == -20
    assert info["reward_components"]["cost"] < 0
    assert reward == pytest.approx(sum(info["reward_components"].values()))


def test_solver_contact_cannot_be_erased_by_clear_endpoint(env):
    geometry = env.evaluator.geometry
    normal, rotation = geometry["normal"], geometry["target_rotation"]
    # The interval begins with a shallow outgoing collision and ends separated.
    position = (geometry["center"] + (geometry["radius"] - 1e-5) * normal
                - rotation @ geometry["offset_body"])
    place(env, position, rotation, .65 * normal)
    _, _, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert terminated and not truncated and info["physics_substeps"] == 1
    assert "ARMOR_CONTACT" in info["result"]["failure_modes"]
    assert not info["result"]["passed"]
    contacts = [event for event in info["result"]["events"] if event["event"] == "SAFETY_CONTACT"]
    assert any(event["sampling_source"] == "COMPLETED_INTERVAL_SOLVER" for event in contacts)
    assert env.evaluator.reasons  # later clear snapshots cannot erase this history
