"""Bounded arrival checks using the shared native-MuJoCo execution path."""
from copy import deepcopy
import json

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


def test_flatness_reference_reaches_terminal_set_through_real_step(env):
    from rm27_contact_rl.baselines import FlatnessReference

    env.step(env.previous_action_ctbr.copy())
    with pytest.raises(RuntimeError, match="immediately after"):
        FlatnessReference(env, 1.)
    env.reset(options={"initial_coordinates": [1.2, 0, 0]})
    reference = FlatnessReference(env, 1.)
    terminated = False
    while not terminated:
        _, _, terminated, _, info = env.step(reference.action(env))
    result = info["result"]
    assert result["passed"] and result["termination_reason"] == "ARRIVAL_SUCCESS"
    assert result["arrival_time_s"] < 1. and result["saturation_fraction"] == 0
    assert result["informed_prefix_s"] == 0


def test_scored_segment_excludes_prefix_reward_but_keeps_physical_history(env):
    for _ in range(4):
        env.step(env.previous_action_ctbr.copy())
    intervals = env.evaluator.completed_intervals
    env.start_scored_segment()
    assert env.informed_prefix_s == pytest.approx(.02)
    assert env.reward_component_totals == dict(progress=0., cost=0., event=0.)
    _, reward, _, _, info = env.step(env.previous_action_ctbr.copy())
    assert env.evaluator.completed_intervals == intervals + 5
    assert sum(env.reward_component_totals.values()) == pytest.approx(reward)
    assert info["episode_elapsed_s"] == pytest.approx(.025)


def test_violation_ratio_is_one_exactly_at_declared_tolerances(env):
    terminal = env.rl_config["terminal"]
    speed = terminal["normal_speed_m_s"]
    at_limits = dict(plane_gap_error_m=terminal["gap_tolerance_m"],
                     tangent_position_error_m=terminal["tangent_position_tolerance_m"],
                     com_normal_speed_m_s=speed + terminal["normal_speed_tolerance_m_s"],
                     material_normal_speed_m_s=speed - terminal["normal_speed_tolerance_m_s"],
                     com_tangent_speed_m_s=terminal["tangent_speed_tolerance_m_s"],
                     material_tangent_speed_m_s=terminal["tangent_speed_tolerance_m_s"],
                     attitude_error_deg=terminal["attitude_tolerance_deg"],
                     angular_velocity_error_rad_s=terminal["body_rate_tolerance_rad_s"])
    assert env.evaluator.terminal_violation_ratio(at_limits) <= 1.
    for name in at_limits:
        if name != "material_normal_speed_m_s":
            beyond = dict(at_limits, **{name: np.nextafter(at_limits[name] * 1.001, np.inf)})
            assert env.evaluator.terminal_violation_ratio(beyond) > 1., name


def test_best_violation_ratio_agrees_with_real_arrival_decisions(env):
    rng = np.random.default_rng(4)
    geometry = env.evaluator.geometry
    axes = geometry["target_rotation"]
    outcomes = []
    for _ in range(60):
        env.reset(options={"initial_coordinates": [1.2, 0, 0]})
        tilt = Rotation.from_rotvec(np.deg2rad(rng.uniform(-13, 13, 3))).as_matrix()
        place(env, geometry["position_target"] + .0225 * geometry["normal"] + axes[:, :2] @ rng.uniform(-.02, .02, 2),
              axes @ tilt, geometry["velocity_target"] + rng.uniform(-.4, .4, 3), rng.uniform(-1.3, 1.3, 3))
        fresh_evaluator(env)
        terminated = False
        while not terminated:
            _, _, terminated, _, info = env.step(env.previous_action_ctbr.copy())
        result = info["result"]
        outcomes.append(result["passed"])
        # Tracked only while success was still possible, so any qualifying sample arrives.
        if result["passed"]:
            assert result["best_terminal_violation_ratio"] <= 1.
        else:
            assert result["best_terminal_violation_ratio"] is None or result["best_terminal_violation_ratio"] > 1.
    assert 0 < sum(outcomes) < len(outcomes)


def test_precision_bonus_is_off_by_default_and_only_changes_the_terminal_event(env, tmp_path):
    from rm27_contact_rl.arrival_env import DEFAULT_ARRIVAL_CONFIG
    shaped = AgileTerminalArrivalEnv(DEFAULT_ARRIVAL_CONFIG.with_name("terminal_arrival_precision.json"))
    try:
        assert shaped.rl_config["reward"]["terminal_precision_bonus"] == 15
        for instance in (env, shaped):
            instance.reset(options={"initial_coordinates": [1.2, 0, 0]})
        action = np.array([2.4, 0., 0., 0.])
        while True:
            a, b = env.step(action), shaped.step(action)
            np.testing.assert_array_equal(a[0], b[0])
            if a[2]:
                break
            assert a[1] == b[1]
        plain, bonus = a[4]["result"], b[4]["result"]
        ratio = plain["best_terminal_violation_ratio"]
        assert not plain["passed"] and ratio == bonus["best_terminal_violation_ratio"] > 1
        assert plain["terminal_precision_bonus"] == 0
        assert plain["reward_status"] == "FROZEN_I4_FIRST_EXPERIMENT_NOT_VALIDATED_LEARNING"
        assert bonus["terminal_precision_bonus"] == pytest.approx(15 / ratio)
        assert b[1] - a[1] == pytest.approx(15 / ratio)
        shaped.reset(options={"initial_coordinates": [1.2, 0, 0]})
        near_arrival(shaped)
        _, reward, terminated, _, info = shaped.step(shaped.previous_action_ctbr.copy())
        assert terminated and info["result"]["passed"] and info["reward_components"]["event"] == 35
    finally:
        shaped.close()
    bad = json.loads(DEFAULT_ARRIVAL_CONFIG.read_text())
    bad["reward"]["terminal_precision_bonus"] = -1
    (tmp_path / "bad.json").write_text(json.dumps(bad))
    with pytest.raises(ValueError):
        AgileTerminalArrivalEnv(tmp_path / "bad.json")


def test_moderate_terminal_set_is_config_only_and_reachable_by_the_reference():
    from rm27_contact_rl.arrival_env import DEFAULT_ARRIVAL_CONFIG
    from rm27_contact_rl.baselines import FlatnessReference

    strict = json.loads(DEFAULT_ARRIVAL_CONFIG.with_name("terminal_arrival_precision.json").read_text())
    moderate = json.loads(DEFAULT_ARRIVAL_CONFIG.with_name("terminal_arrival_moderate_precision.json").read_text())
    assert {k: v for k, v in moderate.items() if k not in ("terminal", "status")} == \
        {k: v for k, v in strict.items() if k not in ("terminal", "status")}
    terminal = moderate["terminal"]
    speed, tolerance = terminal["normal_speed_m_s"], terminal["normal_speed_tolerance_m_s"]
    assert (speed - tolerance, speed + tolerance) == (1.5, 3.0) and terminal["gap_m"] == strict["terminal"]["gap_m"]
    env = AgileTerminalArrivalEnv(DEFAULT_ARRIVAL_CONFIG.with_name("terminal_arrival_moderate_precision.json"))
    try:
        env.reset(options={"initial_coordinates": [1.25, .025, -.025]})
        np.testing.assert_allclose(env.geometry["velocity_target"], -2.25 * env.geometry["normal"])
        reference = FlatnessReference(env, 1.)
        terminated = False
        while not terminated:
            _, _, terminated, _, info = env.step(reference.action(env))
        assert info["result"]["passed"]
    finally:
        env.close()


def test_dynamics_scales_change_only_the_plant_and_survive_snapshot_restore(env):
    model = env.plant.model
    uav, motor = model.body("uav").id, model.actuator("uav_motor_0").id
    mass, gain = float(model.body_mass[uav]), float(model.actuator_gainprm[motor, 0])
    env.set_dynamics_scales(mass=1.1, thrust=.9)
    assert model.body_mass[uav] == pytest.approx(1.1 * mass) and model.actuator_gainprm[motor, 0] == pytest.approx(.9 * gain)
    assert env.plant.hover_force == pytest.approx(mass * 9.81, rel=1e-3)  # controller side stays nominal
    env.reset(options={"initial_coordinates": [1.2, 0, 0]})
    for _ in range(3):
        env.step(env.previous_action_ctbr.copy())
    snapshot = env.episode_snapshot()
    reference = [env.step(np.array([1.8, 1., 0., 0.]))[0] for _ in range(3)]
    env.restore_episode(snapshot)
    env.set_dynamics_scales()  # a later draw must not leak into the restored flight
    env.restore_episode(snapshot)
    assert env.dynamics_scales == dict(mass=1.1, thrust=.9) and model.body_mass[uav] == pytest.approx(1.1 * mass)
    for expected in reference:
        np.testing.assert_array_equal(env.step(np.array([1.8, 1., 0., 0.]))[0], expected)
    env.set_dynamics_scales()
    assert model.body_mass[uav] == mass and model.actuator_gainprm[motor, 0] == gain
    with pytest.raises(ValueError):
        env.set_dynamics_scales(mass=0.)

