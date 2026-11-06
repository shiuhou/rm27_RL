"""Bounded Stage 1 contract checks using one precompiled extracted-scene model.

The expensive immutable source-scene compilation belongs to the engineering
smoke run. Set RM27_STAGE1_TEST_MODEL to its private MJB cache to exercise these
checks; no test rebuilds the scene or performs policy optimization.
"""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27_contact_rl.paths import ROOT


@pytest.fixture(scope="module")
def stage1_env():
    from rm27_contact_rl.stage1_env import Stage1ContactEnv, DEFAULT_STAGE1_CONFIG
    from rm27_contact_rl.stage1_plant import Stage1Plant

    model_path = os.environ.get("RM27_STAGE1_TEST_MODEL")
    if not model_path:
        pytest.skip("Set RM27_STAGE1_TEST_MODEL to the bounded smoke run's private cached MJB")
    path = Path(model_path)
    if not path.is_file():
        pytest.fail(f"Explicit Stage 1 test model is absent: {path}")
    config = json.loads(DEFAULT_STAGE1_CONFIG.read_text())
    plant = Stage1Plant(config, model_path=path)
    instance = Stage1ContactEnv(DEFAULT_STAGE1_CONFIG, plant=plant)
    try:
        yield instance
    finally:
        instance.close()


@pytest.fixture
def env(stage1_env):
    stage1_env.horizon_s = float(stage1_env.rl_config["horizon_s"])
    stage1_env.reset(seed=23)
    try:
        yield stage1_env
    finally:
        stage1_env.horizon_s = float(stage1_env.rl_config["horizon_s"])


def place(env, position, rotation, velocity=None, omega=None):
    """Set only a test initial state; subsequent contact comes from real mj_step."""
    plant = env.plant
    qadr, dadr = plant.qpos_address, plant.dof_address
    plant.data.qpos[qadr:qadr + 3] = position
    plant.data.qpos[qadr + 3:qadr + 7] = Rotation.from_matrix(rotation).as_quat()[[3, 0, 1, 2]]
    plant.data.qvel[dadr:dadr + 3] = np.zeros(3) if velocity is None else velocity
    plant.data.qvel[dadr + 3:dadr + 6] = np.zeros(3) if omega is None else omega
    mujoco.mj_forward(plant.model, plant.data)


def evaluator_at_current_state(env, config=None):
    from rm27_contact_rl.stage1_evaluation import Stage1Evaluator

    env.evaluator = Stage1Evaluator(env.rl_config if config is None else config, env.plant)
    env.evaluator.initialize()
    env._terminal_result = None
    env._active = True
    return env.evaluator


def contact_state(env, gap=0.0, velocity=None, rotation=None, across=0.0):
    geometry = env.geometry
    rotation = geometry["target_rotation"] if rotation is None else rotation
    position = (geometry["center"] + across * geometry["target_rotation"][:, 1]
                + geometry["normal"] * (geometry["radius"] + gap)
                - rotation @ geometry["offset_body"])
    place(env, position, rotation, velocity)
    return position


def active_overlaps(env):
    """Parked aircraft are irrelevant to the active aircraft's clear start."""
    return [contact for contact in env.plant.data.contact
            if contact.dist <= 0 and env.plant.active_geom_ids.intersection(map(int, contact.geom))]


def hover_until_terminal(env, max_controls):
    for _ in range(max_controls):
        observation, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
        assert np.isfinite(observation).all() and np.isfinite(reward)
        assert not truncated
        if terminated:
            return info
    pytest.fail(f"Expected physical terminal event within {max_controls} controls")


def test_reset_variations_are_reproducible_and_hover_actuators_are_initialized(env):
    observation, info = env.reset(seed=91)
    initial = env.initial_perturbation.copy()
    first_state = env.plant.state
    repeated, _ = env.reset(seed=91)
    np.testing.assert_array_equal(repeated, observation)
    np.testing.assert_array_equal(env.initial_perturbation, initial)
    assert env.initial_ranges.shape == (12, 2)
    assert np.all(initial >= env.initial_ranges[:, 0])
    assert np.all(initial <= env.initial_ranges[:, 1])
    assert np.linalg.norm(first_state.velocity) > 0
    assert np.linalg.norm(first_state.omega_body) > 0
    assert not np.allclose(first_state.rotation, np.eye(3))
    np.testing.assert_allclose(env.plant.data.act[env.plant.actuator_state_ids], env.plant.hover_force / 4)
    np.testing.assert_allclose(env.plant.data.ctrl[env.plant.actuator_ids], env.plant.hover_force / 4)
    assert env.plant.data.time == 0 and env.evaluator.completed_intervals == 0
    assert info["task_id"] == "fixed_surface_contact_v1"
    assert info["observation_schema"] == "fixed_surface_state_v1"
    changed, _ = env.reset(seed=92)
    assert not np.array_equal(changed, observation)
    nominal, _ = env.reset(options={"initial_perturbation": np.zeros(12)})
    np.testing.assert_allclose(env.plant.state.velocity, 0)
    np.testing.assert_allclose(env.plant.state.omega_body, 0)
    np.testing.assert_allclose(env.plant.state.rotation, np.eye(3), atol=1e-12)
    assert np.isfinite(nominal).all()
    for invalid in (np.zeros(11), np.full(12, np.nan), env.initial_ranges[:, 1] + 1):
        with pytest.raises(ValueError):
            env.reset(options={"initial_perturbation": invalid})


def test_actor_observation_excludes_motors_contacts_and_evaluator_diagnostics(env):
    observation = env._observation()
    state, geometry = env.plant.state, env.geometry
    expected = np.r_[geometry["center"] - state.position, geometry["normal"],
                     state.velocity, state.rotation.ravel(), state.omega_body,
                     env.previous_action_ctbr, env.horizon_s].astype(np.float32)
    np.testing.assert_array_equal(observation, expected)
    assert observation.shape == (26,) and observation.dtype == np.float32
    assert env.observation_space.contains(observation)
    env.evaluator.events.append({"event": "DIAGNOSTIC_ONLY"})
    env.evaluator.first_contact = {"privileged_test_marker": 999}
    env.evaluator.reasons.add("DIAGNOSTIC_ONLY")
    saved = env.plant.data.act.copy()
    env.plant.data.act[env.plant.actuator_state_ids] = 0
    np.testing.assert_array_equal(env._observation(), observation)
    env.plant.data.act[:] = saved


def test_action_limits_are_finite_and_preserve_lagged_motor_states(env):
    before_time = env.plant.data.time
    for invalid in ([1, 0, 0], [1, np.nan, 0, 0], [1, 0, 0, np.inf]):
        with pytest.raises(ValueError):
            env.step(invalid)
        assert env.plant.data.time == before_time
    requested = np.asarray(env.action_space.high, float) + [10, 100, 100, 100]
    observation, reward, terminated, truncated, info = env.step(requested)
    assert np.isfinite(observation).all() and np.isfinite(reward)
    assert not truncated
    assert info["action_clipped"] and info["proxy_saturated"]
    np.testing.assert_allclose(env.previous_action_ctbr, env.action_space.high)
    commands = env.plant.data.ctrl[env.plant.actuator_ids]
    states = env.plant.data.act[env.plant.actuator_state_ids]
    assert np.all(commands >= 0) and np.all(commands <= env.plant.maximum)
    assert not np.allclose(commands, states)
    assert 1 <= info["physics_substeps"] <= 5
    assert env.plant.data.time == pytest.approx(info["physics_substeps"] * .001)


def test_normalized_ctbr_hover_mapping_reuses_existing_action_chain(env):
    from rm27_contact_rl.ppo import HOVER_CENTERED, NormalizedCTBR

    wrapped = NormalizedCTBR(env, HOVER_CENTERED)
    np.testing.assert_array_equal(wrapped.hover_action, np.zeros(4))
    np.testing.assert_allclose(wrapped.action(np.zeros(4)), [env.plant.hover_force, 0, 0, 0])
    np.testing.assert_allclose(wrapped.action(np.ones(4)), env.action_space.high)
    negative = wrapped.action(-np.ones(4))
    assert negative[0] < env.action_space.low[0]
    np.testing.assert_allclose(negative[1:], env.action_space.low[1:])
    np.testing.assert_allclose(np.clip(negative, env.action_space.low, env.action_space.high),
                               env.action_space.low)
    for invalid in ([0, 0, 0], [0, 0, np.nan, 0]):
        with pytest.raises(ValueError):
            wrapped.action(invalid)


def test_virtual_contact_target_pose_without_physical_contact_cannot_succeed(env):
    contact_state(env, gap=.002)
    evaluator = evaluator_at_current_state(env)
    assert not evaluator.done and not evaluator.first_contact
    _, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert np.isfinite(reward) and not truncated
    assert not terminated
    assert env.evaluator.first_contact is None
    assert not env.evaluator.summary()["passed"]


def test_timeout_terminates_on_exact_one_millisecond_boundary(env):
    config = deepcopy(env.rl_config)
    config["horizon_s"] = .003
    env.horizon_s = .003
    evaluator_at_current_state(env, config)
    observation, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert terminated and not truncated
    assert info["physics_substeps"] == 3
    assert info["time_s"] == pytest.approx(.003)
    assert observation[-1] == 0
    assert info["result"]["failure_modes"] == ["CONTACT_TIMEOUT"]
    assert not info["result"]["passed"]
    assert reward == pytest.approx(sum(info["reward_components"].values()))
    final_qpos, final_time = env.plant.data.qpos.copy(), env.plant.data.time
    with pytest.raises(RuntimeError):
        env.step(env.previous_action_ctbr.copy())
    np.testing.assert_array_equal(env.plant.data.qpos, final_qpos)
    assert env.plant.data.time == final_time


def test_evaluation_set_keeps_center_and_all_signed_axis_cases():
    from rm27_contact_rl.stage1_env import fixed_evaluation_initials

    initials = fixed_evaluation_initials()
    assert len(initials) == 19
    values = [np.asarray(entry, float) for entry in initials]
    assert all(value.shape == (12,) and np.isfinite(value).all() for value in values)
    np.testing.assert_array_equal(values[0], np.zeros(12))
    nonzero = values[1:]
    assert all(np.count_nonzero(value) == 1 for value in nonzero)
    for value in nonzero:
        assert any(np.array_equal(other, -value) for other in nonzero)


def test_real_contact_event_is_required_and_captures_precontact_state(env):
    # The fixture starts outside the surface and moves inward through MuJoCo.
    # Its same 6 ms solve contains the original outer collision shell and
    # the shallow inner armor collision shell: neither pair may be dropped.
    contact_state(env, gap=.001, velocity=-.2 * env.geometry["normal"])
    assert not active_overlaps(env)
    evaluator = evaluator_at_current_state(env)
    assert not evaluator.done and evaluator.first_contact is None
    info = hover_until_terminal(env, max_controls=2)
    assert evaluator.first_contact is not None
    contact = evaluator.first_contact
    assert info["physics_substeps"] == 1
    assert contact["time_s"] == pytest.approx(.006)
    assert info["result"]["completed_physics_intervals"] == 6
    surface_names = {name for record in contact["contacts"] for name in record["geoms"]
                     if name != env.plant.bottom_geom_name}
    assert surface_names == {"rmuc2027_surface_35364", "rmuc2027_surface_36293"}
    assert all(env.plant.bottom_geom_name in record["geoms"] for record in contact["contacts"])
    assert all(record["designated"] for record in contact["contacts"])
    assert all(record["normal_force_N"] > 0 for record in contact["contacts"])
    assert info["result"]["passed"] and not info["result"]["failure_modes"]
    assert contact["valid_geometry"] and contact["accepted"]
    assert contact["pre_state"]["timestamp_s"] == pytest.approx(contact["interval_start_s"])
    assert contact["post_state"]["timestamp_s"] == pytest.approx(contact["time_s"])
    assert contact["time_s"] > contact["interval_start_s"]
    assert info["result"]["parameter_status"] == "PROPOSED_UNCONFIRMED"
    assert not info["result"].get("official_success", False)


def test_outside_region_first_contact_has_a_physically_clear_initial_state(env):
    # 1 mm was insufficient at across=60 mm because this existing edge
    # collision shell protrudes. A predeclared 30 mm gap avoids that overlap.
    contact_state(env, gap=.03, across=.06, velocity=-.5 * env.geometry["normal"])
    assert not active_overlaps(env)
    evaluator = evaluator_at_current_state(env)
    assert not evaluator.done and evaluator.first_contact is None
    info = hover_until_terminal(env, max_controls=20)
    result, contact = info["result"], evaluator.first_contact
    assert contact is not None
    assert contact["time_s"] == pytest.approx(.061)
    assert result["completed_physics_intervals"] == 61
    assert info["physics_substeps"] == 1
    assert result["failure_modes"] == ["CONTACT_OUTSIDE_REGION"]
    assert not result["passed"] and not contact["accepted"]
    assert all(record["designated"] and record["normal_force_N"] > 0
               for record in contact["contacts"])
    assert all(abs(record["local_coordinates_m"][1]) > env.geometry["halfsize"][1]
               for record in contact["gates"])


def test_wrong_aircraft_component_first_contact_remains_rejected(env):
    geometry = env.geometry
    rotation = geometry["target_rotation"] @ np.diag([1., -1., -1.])
    place(env, geometry["center"] + .12 * geometry["normal"], rotation,
          velocity=-.5 * geometry["normal"])
    assert not active_overlaps(env)
    evaluator = evaluator_at_current_state(env)
    assert not evaluator.done
    info = hover_until_terminal(env, max_controls=20)
    result, contact = info["result"], evaluator.first_contact
    assert contact is not None
    assert contact["time_s"] == pytest.approx(.079)
    assert result["completed_physics_intervals"] == 79
    assert "UNDESIGNATED_CONTACT" in result["failure_modes"]
    assert not result["passed"] and not contact["accepted"]
    assert all("drone_1_collision_core_board_29" in record["geoms"]
               and not record["designated"] and record["normal_force_N"] > 0
               for record in contact["contacts"])


def test_original_outside_region_fixture_still_rejects_initial_overlap(env):
    contact_state(env, gap=.001, across=.06, velocity=-.2 * env.geometry["normal"])
    assert active_overlaps(env)
    evaluator = evaluator_at_current_state(env)
    result = evaluator.summary()
    assert evaluator.done and result["failure_modes"] == ["INITIAL_OVERLAP"]
    assert result["completed_physics_intervals"] == 0
    assert not result["passed"] and result["first_contact"] is None


def test_contact_geometry_configuration_preserves_all_original_numeric_contracts():
    from rm27_contact_rl.stage1_env import DEFAULT_STAGE1_CONFIG

    original = json.loads((ROOT / "configs/stage1/task.json").read_text())
    corrected = json.loads(DEFAULT_STAGE1_CONFIG.read_text())
    assert corrected.pop("contact_surface_registration")
    assert len(corrected.pop("expected_contact_surface_registration_sha256")) == 64
    assert corrected == original


@pytest.mark.parametrize("change,message", [
    ("manifest_hash", "registration hash mismatch"),
    ("source_hash", "source hash mismatch"),
    ("source_visual", "audited selected plate"),
    ("unknown_geometry", "unknown"),
    ("region", "original target frame and allowed region"),
])
def test_contact_registration_rejects_changed_or_unknown_evidence(env, tmp_path, change, message):
    from rm27_contact_rl.stage1_plant import _apply_contact_surface_registration

    config = deepcopy(env.rl_config)
    registry_path = ROOT / config["contact_surface_registration"]
    original = registry_path.read_text()
    registry = json.loads(original)
    target = deepcopy(env.plant.target_mapping)
    target["registered_contact_geoms"] = registry["original_registered_contact_geoms"][:]
    if change == "source_hash":
        registry["sources"]["collision_geoms"]["sha256"] = "0" * 64
    elif change == "source_visual":
        registry["additional_contact_geoms"][0]["source_visual"] = "visual_118"
    elif change == "unknown_geometry":
        registry["additional_contact_geoms"][0]["geom"] = "unknown_contact_geometry"
    elif change == "region":
        registry["target_reference"]["contact_region_halfsize_slope_across_m"][0] += .001
    changed = tmp_path / "changed_contact_registry.json"
    changed.write_text(original + "\n" if change == "manifest_hash" else json.dumps(registry))
    config["contact_surface_registration"] = str(changed)
    if change != "manifest_hash":
        config["expected_contact_surface_registration_sha256"] = hashlib.sha256(changed.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match=message):
        _apply_contact_surface_registration(config, target, dict(env.plant.sources),
                                            env.plant.model, env.plant.data)


def test_initial_overlap_is_rejected_without_becoming_a_success(env):
    contact_state(env, gap=-.001, velocity=-.2 * env.geometry["normal"])
    evaluator = evaluator_at_current_state(env)
    assert evaluator.done
    summary = evaluator.summary()
    assert not summary["passed"]
    assert "INITIAL_OVERLAP" in summary["failure_modes"]
    assert summary["completed_physics_intervals"] == 0


def test_reward_weights_do_not_change_independent_contact_scoring(env):
    from rm27_contact_rl.stage1_evaluation import Stage1Evaluator

    first = Stage1Evaluator(env.rl_config, env.plant)
    first.initialize()
    changed = deepcopy(env.rl_config)
    changed["reward"] = {key: value * 7 for key, value in changed["reward"].items()}
    second = Stage1Evaluator(changed, env.plant)
    second.initialize()
    assert first.summary() == second.summary()
    np.testing.assert_array_equal(first.metrics(env.plant.state)["position_error_m"],
                                  second.metrics(env.plant.state)["position_error_m"])


def test_stage1_ppo_factory_and_checkpoint_are_separate_from_old_tasks(env, tmp_path, monkeypatch):
    import torch
    from stable_baselines3 import PPO
    import rm27_contact_rl.stage1_env as stage1_module
    from rm27_contact_rl.stage1_plant import Stage1Plant
    from rm27_contact_rl.ppo import (_load_checkpoint, _model_state, _save_checkpoint,
                                    load_train_config, make_vec_env)
    from rm27_contact_rl.provenance import provenance

    config = load_train_config(ROOT / "configs/stage1/ppo_foundation.json")
    assert config["task_id"] == "fixed_surface_contact_v1"
    assert config["observation_schema"] == "fixed_surface_state_v1"
    real_class = stage1_module.Stage1ContactEnv

    def reused_model(config_path):
        task = json.loads(Path(config_path).read_text())
        return real_class(config_path, plant=Stage1Plant(
            task, model=env.plant.model, output_dir=Path(os.environ["RM27_STAGE1_TEST_MODEL"]).parent))

    monkeypatch.setattr(stage1_module, "Stage1ContactEnv", reused_model)
    torch.set_num_threads(1)
    vec = make_vec_env(config, n_envs=1)
    restored = None
    try:
        observation = vec.reset()
        assert observation.shape == (1, 26) and np.isfinite(observation).all()
        assert vec.venv.envs[0].unwrapped.task_id == "fixed_surface_contact_v1"
        model = PPO("MlpPolicy", vec, seed=0, device="cpu", **config["ppo"])
        action, _ = model.predict(observation, deterministic=True)
        observation, reward, done, infos = vec.step(action)
        assert np.isfinite(observation).all() and np.isfinite(reward).all()
        source = provenance(vec.venv.envs[0].unwrapped, algorithm="PPO_CPU", seed=0)
        checkpoint = tmp_path / "stage1_checkpoint"
        metadata = _save_checkpoint(checkpoint, model, vec, config, source, 0)
        recovered, restored, _ = _load_checkpoint(
            checkpoint, training=False, n_envs=1, expected_task="fixed_surface_contact_v1")
        assert _model_state(recovered, restored) == metadata["state"]
        assert not restored.training and not restored.norm_reward
        with pytest.raises(ValueError, match="different task"):
            _load_checkpoint(checkpoint, training=False, expected_task="agile_terminal_arrival_v1")
        assert model.num_timesteps == 0  # This is inference/plumbing, not policy optimization.
    finally:
        vec.close()
        if restored is not None:
            restored.close()


def test_beta_comparison_restores_original_initial_and_reloads_mapping(env, tmp_path, monkeypatch):
    import torch
    from stable_baselines3 import PPO
    import rm27_contact_rl.stage1_env as stage1_module
    from rm27_contact_rl.stage1_plant import Stage1Plant
    from rm27_contact_rl.stage1_runner import initialize_comparison
    from rm27_contact_rl.ppo import (load_train_config, make_vec_env, _model_state,
                                    _save_checkpoint, _load_checkpoint)
    from rm27_contact_rl.provenance import provenance

    config = load_train_config(ROOT / "configs/stage1/ppo_stage1a_beta01_seed0.json")
    original_class = stage1_module.Stage1ContactEnv
    def reused_model(path):
        task = json.loads(Path(path).read_text())
        return original_class(path, plant=Stage1Plant(task, model=env.plant.model,
            output_dir=Path(os.environ["RM27_STAGE1_TEST_MODEL"]).parent))
    monkeypatch.setattr(stage1_module, "Stage1ContactEnv", reused_model)
    torch.set_num_threads(1)
    vec = make_vec_env(config)
    restored = None
    try:
        model = PPO("MlpPolicy", vec, seed=0, device="cpu", **config["ppo"])
        vec.reset()
        # Exercise real restoration, not just a fresh-seed equality assertion.
        with torch.no_grad():
            next(model.policy.parameters()).add_(.01)
        vec.obs_rms.mean += .01
        evidence = initialize_comparison(model, vec, config)
        assert evidence["exact_original_initial_state"] and evidence["seeded_rng_preserved"]
        assert _model_state(model, vec) == evidence["restored_state"]
        wrapped = vec.venv.envs[0].env
        np.testing.assert_array_equal(wrapped.action([0, 1, -1, .5]),
                                      [env.plant.hover_force, 2, -2, 1])
        np.testing.assert_array_equal(wrapped.unwrapped._action_high[1:], [20., 20., 20.])
        source = provenance(wrapped.unwrapped, algorithm="PPO_CPU", seed=0)
        path = tmp_path / "beta_initial"
        _save_checkpoint(path, model, vec, config, source, 0)
        loaded, restored, meta = _load_checkpoint(path, training=False, n_envs=1,
                                                 expected_task="fixed_surface_contact_v1")
        np.testing.assert_array_equal(restored.venv.envs[0].env.action([0, 1, -1, .5]), wrapped.action([0, 1, -1, .5]))
        assert _model_state(loaded, restored) == meta["state"]
        assert not restored.training
        with pytest.raises(ValueError, match="zero-update"):
            initialize_comparison(model, vec, dict(config, initial_checkpoint=
                "artifacts/agile_flight/stage1a_seed0_01/run/checkpoints/final"))
        changed = deepcopy(config)
        changed["ppo"]["gamma"] = .98
        with pytest.raises(ValueError, match="only rate scale"):
            initialize_comparison(model, vec, changed)
        assert wrapped.unwrapped.plant.data.time == 0  # No rollout or optimizer.
    finally:
        vec.close()
        if restored is not None:
            restored.close()


def test_complete_short_episode_replays_all_scene_and_motor_states(env, tmp_path):
    """Replay an entire declared short episode, including its exact terminal interval."""
    records = []
    for _ in range(2):
        env.horizon_s = .013
        env.reset(seed=7)
        config = deepcopy(env.rl_config)
        config["horizon_s"] = .013
        evaluator_at_current_state(env, config)
        rows = []
        terminated = False
        while not terminated:
            observation, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
            assert not truncated
            rows.append(np.r_[env.plant.data.time, env.plant.data.qpos,
                              env.plant.data.qvel, env.plant.data.act,
                              env.plant.data.ctrl, observation, reward])
            assert len(rows) <= 3
        assert info["physics_substeps"] == 3
        assert info["result"]["failure_modes"] == ["CONTACT_TIMEOUT"]
        records.append(np.asarray(rows))
    np.testing.assert_array_equal(records[0], records[1])
    path = tmp_path / "complete_episode.npz"
    np.savez_compressed(path, states=records[0])
    with np.load(path) as saved:
        np.testing.assert_array_equal(saved["states"], records[0])


@pytest.mark.parametrize("section,key,value", [
    (None, "horizon_s", float("nan")),
    (None, "horizon_s", float("inf")),
    ("action", "body_rate_limit_rad_s", float("nan")),
    ("reward", "guide_distance_scale_m", 0.),
    ("reward", "alignment_distance_scale_m", -1.),
    ("safety", "max_speed_m_s", 0.),
    ("safety", "max_body_rate_rad_s", -1.),
])
def test_invalid_numeric_contract_is_rejected_before_scene_loading(tmp_path, section, key, value):
    from rm27_contact_rl.stage1_env import DEFAULT_STAGE1_CONFIG, load_stage1_config

    config = json.loads(DEFAULT_STAGE1_CONFIG.read_text())
    (config if section is None else config[section])[key] = value
    path = tmp_path / "invalid_task.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_stage1_config(path)


def test_foundation_config_does_not_authorize_a_long_ppo_run(tmp_path):
    from rm27_contact_rl.ppo import load_train_config

    source = ROOT / "configs/stage1/ppo_foundation.json"
    config = load_train_config(source)
    assert config["protocol"] == "stage1_foundation"
    assert config["budget"]["transitions"] == 64
    assert config["budget"]["learning_wall_s"] <= 60
    invalid = deepcopy(config)
    invalid["budget"]["transitions"] = 128
    path = tmp_path / "unapproved_long_run.json"
    path.write_text(json.dumps(invalid))
    with pytest.raises(ValueError):
        load_train_config(path)


class _TinyContactPlant:
    """Physical event fixture for scorer logic, separate from extracted-scene evidence."""

    bottom_geom_name = "bottom"

    def __init__(self, *, wrong_part=False, simultaneous_wrong_part=False):
        from rm27_contact_physics.model import VehicleEstimate

        self.estimate_type = VehicleEstimate
        bottom_enabled = int(not wrong_part)
        wrong_enabled = int(wrong_part or simultaneous_wrong_part)
        xml = f'''<mujoco model="stage1_scorer_unit_fixture">
          <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
          <worldbody>
            <geom name="surface" type="box" pos="0 0 -.005" size=".2 .2 .005"/>
            <body name="vehicle" pos="0 0 .0602">
              <freejoint/>
              <inertial pos="0 0 0" mass=".15" diaginertia=".00015 .00015 .00025"/>
              <geom name="bottom" type="sphere" pos="0 0 -.05" size=".01" mass="0"
                    contype="{bottom_enabled}" conaffinity="{bottom_enabled}"/>
              <geom name="wrong_part" type="sphere" pos=".04 0 -.05" size=".01" mass="0"
                    contype="{wrong_enabled}" conaffinity="{wrong_enabled}"/>
            </body>
          </worldbody>
        </mujoco>'''
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.data.qvel[2] = -.3
        self.body_id = self.model.body("vehicle").id
        self.active_geom_ids = {self.model.geom(name).id for name in ("bottom", "wrong_part")}
        self.target_mapping = dict(armor_surface_center_m=[0, 0, 0], armor_normal=[0, 0, 1],
                                   slope_axis=[-1, 0, 0], across_axis=[0, 1, 0],
                                   contact_region_halfsize_slope_across_m=[.1, .1],
                                   registered_contact_geoms=["surface"])
        self.contact_spec = dict(offset_body_m=[0, 0, -.05], radius_m=.01)
        mujoco.mj_forward(self.model, self.data)

    @property
    def state(self):
        return self.estimate_type(self.data.qpos[:3].copy(), self.data.qvel[:3].copy(),
                                  self.data.xmat[self.body_id].reshape(3, 3).copy(),
                                  self.data.qvel[3:6].copy(), float(self.data.time))


def _tiny_evaluator(*, wrong_part=False, simultaneous_wrong_part=False):
    from rm27_contact_rl.stage1_env import DEFAULT_STAGE1_CONFIG
    from rm27_contact_rl.stage1_evaluation import Stage1Evaluator

    plant = _TinyContactPlant(wrong_part=wrong_part,
                              simultaneous_wrong_part=simultaneous_wrong_part)
    config = json.loads(DEFAULT_STAGE1_CONFIG.read_text())
    config["safety"]["position_min_target_frame_m"] = [-1, -1, -1]
    config["safety"]["position_max_target_frame_m"] = [1, 1, 1]
    evaluator = Stage1Evaluator(config, plant)
    evaluator.initialize()
    assert not evaluator.done
    return plant, config, evaluator


def test_scorer_requires_solver_event_instead_of_target_distance():
    plant, _, evaluator = _tiny_evaluator()
    # Exact nominal geometry can be approached without an active solved contact.
    assert plant.data.ncon == 0
    assert not evaluator.summary()["passed"]
    evaluator.advance(plant.state)
    assert evaluator.first_contact is None
    assert not evaluator.summary()["passed"]
    assert evaluator.completed_intervals == 1


@pytest.mark.parametrize("wrong_part", [False, True])
def test_scorer_classifies_actual_designated_and_wrong_part_contact(wrong_part):
    plant, _, evaluator = _tiny_evaluator(wrong_part=wrong_part)
    for _ in range(4):
        evaluator.advance(plant.state)
        if evaluator.done:
            break
    summary = evaluator.summary()
    assert evaluator.done and evaluator.first_contact is not None
    contact = evaluator.first_contact
    assert contact["contacts"] and any(record["normal_force_N"] > 0 for record in contact["contacts"])
    assert contact["pre_state"]["timestamp_s"] < contact["post_state"]["timestamp_s"]
    assert contact["time_s"] == pytest.approx(plant.data.time)
    if wrong_part:
        assert not summary["passed"]
        assert "UNDESIGNATED_CONTACT" in summary["failure_modes"]
    else:
        assert summary["passed"]
        assert not summary["failure_modes"]
    terminal_time = plant.data.time
    with pytest.raises(RuntimeError, match="finished"):
        evaluator.advance(plant.state)
    assert plant.data.time == terminal_time


def test_safety_failure_overrides_a_valid_contact_in_the_same_interval():
    plant, config, evaluator = _tiny_evaluator()
    # Move from the already clear initialized state into a shallow incoming
    # contact fixture. The next actual solve simultaneously exits a tight
    # test-only boundary; neither the contact event nor its force is mocked.
    plant.data.qpos[2] = .05999
    mujoco.mj_forward(plant.model, plant.data)
    config["safety"]["position_min_target_frame_m"][2] = .059989
    evaluator.advance(plant.state)
    summary = evaluator.summary()
    assert evaluator.first_contact is not None
    assert any(record["normal_force_N"] > 0 for record in evaluator.first_contact["contacts"])
    assert "FLIGHT_VOLUME_EXIT" in summary["failure_modes"]
    assert not summary["passed"]
    assert not evaluator.first_contact["accepted"]
    assert summary["completed_physics_intervals"] == 1


def test_simultaneous_illegal_pair_rejects_an_otherwise_valid_solved_contact():
    plant, _, evaluator = _tiny_evaluator(simultaneous_wrong_part=True)
    for _ in range(4):
        evaluator.advance(plant.state)
        if evaluator.done:
            break
    result = evaluator.summary()
    assert evaluator.done and evaluator.first_contact is not None
    contacts = evaluator.first_contact["contacts"]
    assert any(record["designated"] for record in contacts)
    assert any(not record["designated"] for record in contacts)
    assert all(record["normal_force_N"] > 0 for record in contacts)
    assert "UNDESIGNATED_CONTACT" in result["failure_modes"]
    assert not result["passed"] and not evaluator.first_contact["accepted"]


@pytest.mark.parametrize("face", ["side", "back"])
def test_registered_geometry_does_not_legalize_its_side_or_back_face(face):
    from rm27_contact_rl.stage1_evaluation import Stage1Evaluator

    plant, config, _ = _tiny_evaluator()
    # The same registered box has a top at z=0, sides at x=+/-0.2 and
    # a back face at z=-0.01. Approach those real surfaces from clear states.
    if face == "side":
        plant.data.qpos[:3] = [.2102, 0., .045]
        plant.data.qvel[:3] = [-.3, 0., 0.]
    else:
        plant.data.qpos[:3] = [0., 0., .0298]
        plant.data.qvel[:3] = [0., 0., .3]
    mujoco.mj_forward(plant.model, plant.data)
    assert plant.data.ncon == 0
    evaluator = Stage1Evaluator(config, plant)
    evaluator.initialize()
    assert not evaluator.done
    for _ in range(4):
        evaluator.advance(plant.state)
        if evaluator.done:
            break
    result = evaluator.summary()
    assert evaluator.done and evaluator.first_contact is not None
    contacts = evaluator.first_contact["contacts"]
    assert all(record["designated"] and record["normal_force_N"] > 0 for record in contacts)
    assert "WRONG_TARGET_FACE" in result["failure_modes"]
    assert "UNDESIGNATED_CONTACT" not in result["failure_modes"]
    assert not result["passed"]


@pytest.mark.parametrize("status,allowed", [("PROPOSED_UNCONFIRMED", False), ("CONFIRMED", False)])
def test_formal_training_scope_requires_both_confirmed_parameters_and_permission(tmp_path, status, allowed):
    from rm27_contact_rl.stage1_env import DEFAULT_STAGE1_CONFIG
    from rm27_contact_rl.stage1_runner import check_training_scope

    task = json.loads(DEFAULT_STAGE1_CONFIG.read_text())
    task.update(parameter_status=status, training_allowed=allowed, parameters_confirmed=True)
    path = tmp_path / "provisional_task.json"
    path.write_text(json.dumps(task))
    with pytest.raises(ValueError):
        check_training_scope(dict(protocol="stage1_state_ppo", env_config=str(path)))


def test_zero_step_replay_checks_initial_frame_values(env, tmp_path):
    from rm27_contact_rl.ppo import HOVER_CENTERED, NormalizedCTBR
    from rm27_contact_rl.stage1_runner import record_episode, verify_episode_replay

    geometry = env.geometry
    rotation = geometry["target_rotation"]
    position = (geometry["center"] + geometry["normal"] * (geometry["radius"] - .001)
                - rotation @ geometry["offset_body"])
    fixture_state = dict(position=position.tolist(), rotation=rotation.tolist(),
                         velocity=(-.2 * geometry["normal"]).tolist(), omega_body=[0., 0., 0.])
    output = tmp_path / "initial_overlap_episode"
    result = record_episode(NormalizedCTBR(env, HOVER_CENTERED), output, np.zeros(12),
                            fixture_state=fixture_state)
    assert not result["passed"]
    assert result["failure_modes"] == ["INITIAL_OVERLAP"]
    assert result["completed_physics_intervals"] == 0
    replay = verify_episode_replay(output)
    assert replay["passed"]
    assert replay["physics_intervals"] == 0
    archive_path = output / "trajectory.npz"
    with np.load(archive_path) as archive:
        altered = {key: archive[key].copy() for key in archive.files}
    assert len(altered["time_s"]) == 1
    altered["qpos"][0, env.plant.qpos_address] += 1e-3
    np.savez_compressed(archive_path, **altered)
    corrupted = verify_episode_replay(output)
    assert not corrupted["passed"]
    assert corrupted["physics_intervals"] == 0
    assert corrupted["matching_terminal_result"]
    assert corrupted["max_absolute_errors"]["qpos"] == pytest.approx(1e-3)


def test_evaluation_budget_pauses_between_phases_and_keeps_counters(monkeypatch):
    from rm27_contact_rl import stage1_runner as runner
    clock = [100.]
    monkeypatch.setattr(runner.time, "monotonic", lambda: clock[0])
    budget = runner.SmokeBudget(controls=2, substeps=10, wall_s=3)
    budget.pause()
    clock[0] += 1800
    with budget.activity():
        budget.before_step()
        budget.record({"physics_substeps": 3})
        clock[0] += 1
    clock[0] += 1800
    assert budget.summary()["elapsed_wall_s"] == 1
    with budget.activity():
        budget.before_step()
        budget.record({"physics_substeps": 5})
        with pytest.raises(TimeoutError):
            budget.before_step()
    assert budget.controls == 2 and budget.substeps == 8
    wall = runner.SmokeBudget(controls=100, substeps=500, wall_s=2)
    clock[0] += 2
    with pytest.raises(TimeoutError):
        wall.before_step()


def test_training_budget_stops_before_an_extra_real_physics_step(env):
    from rm27_contact_rl.stage1_runner import BudgetedTrainingEnv, SmokeBudget
    from rm27_contact_rl.ppo import NormalizedCTBR, HOVER_CENTERED, _WallLimit
    budget = SmokeBudget(controls=1, substeps=5, wall_s=30)
    wrapped = BudgetedTrainingEnv(NormalizedCTBR(env, HOVER_CENTERED), budget)
    wrapped.reset(seed=0, options={"initial_perturbation": np.zeros(12)})
    result = wrapped.step(np.zeros(4))
    assert budget.controls == 1 and budget.substeps == result[4]["physics_substeps"]
    time_before, qpos = env.plant.data.time, env.plant.data.qpos.copy()
    with pytest.raises(_WallLimit):
        wrapped.step(np.zeros(4))
    assert env.plant.data.time == time_before
    np.testing.assert_array_equal(env.plant.data.qpos, qpos)


def test_budget_limited_recording_retains_partial_state_without_claiming_termination(env, tmp_path):
    from rm27_contact_rl.stage1_runner import SmokeBudget, record_episode, verify_episode_replay
    from rm27_contact_rl.ppo import NormalizedCTBR, HOVER_CENTERED
    path = tmp_path / "partial"
    result = record_episode(NormalizedCTBR(env, HOVER_CENTERED), path, np.zeros(12),
                            budget=SmokeBudget(controls=1, substeps=5, wall_s=30))
    assert result["execution_status"] == "INCOMPLETE_BUDGET"
    assert not result["passed"] and not result["failure_modes"]
    with np.load(path / "trajectory.npz") as values:
        assert len(values["time_s"]) == 2
        assert values["physics_substeps"].sum() == 5
    with pytest.raises(ValueError, match="Incomplete"):
        verify_episode_replay(path)


def test_stage1a_approval_is_simulation_only_and_keeps_numeric_task_contract():
    from rm27_contact_rl.ppo import load_train_config
    from rm27_contact_rl.stage1_runner import check_training_scope, validate_train_config
    proposed = json.loads((ROOT / "configs/stage1/task_contact_geometry.json").read_text())
    approved = json.loads((ROOT / "configs/stage1/task_stage1a_seed0.json").read_text())
    for key in ("reward", "contact_acceptance", "initial_state", "safety", "action", "horizon_s",
                "physics_dt_s", "policy_dt_s", "contact_surface_registration", "expected_contact_surface_registration_sha256"):
        assert approved[key] == proposed[key]
    config = load_train_config(ROOT / "configs/stage1/ppo_stage1a_seed0.json")
    check_training_scope(config)
    assert approved["confirmation_scope"].startswith("USER_APPROVED_SIMULATION_ONLY")
    for change in ({"transitions": 131584, "learning_wall_s": 1800},
                   {"transitions": 131072, "learning_wall_s": 1801}):
        with pytest.raises(ValueError):
            validate_train_config(dict(config, budget=change))
    with pytest.raises(ValueError):
        validate_train_config(dict(config, evaluation_budget=dict(controls=30400, physics_substeps=152000, wall_s=601)))


def test_training_representative_preserves_complete_episode_and_replay(env, tmp_path):
    from rm27_contact_rl.stage1_env import Stage1ContactEnv
    from rm27_contact_rl.stage1_runner import BudgetedTrainingEnv, SmokeBudget, verify_episode_replay
    from rm27_contact_rl.ppo import NormalizedCTBR, HOVER_CENTERED
    config = deepcopy(env.rl_config)
    config["horizon_s"] = .005
    path = tmp_path / "short_task.json"
    path.write_text(json.dumps(config))
    short = Stage1ContactEnv(path, plant=env.plant)
    output = tmp_path / "recorded_training"
    wrapped = BudgetedTrainingEnv(NormalizedCTBR(short, HOVER_CENTERED),
                                  SmokeBudget(controls=2, substeps=10, wall_s=30), output=output)
    wrapped.reset(seed=0, options={"initial_perturbation": np.zeros(12)})
    assert wrapped.step(np.zeros(4))[2]
    summary = json.loads((output / "episode_00000/summary.json").read_text())
    assert summary["failure_modes"] == ["CONTACT_TIMEOUT"]
    assert summary["execution_status"] == "COMPLETE"
    assert verify_episode_replay(output / "episode_00000")["passed"]
    assert len((output / "episodes.jsonl").read_text().splitlines()) == 1
