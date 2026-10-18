"""Bounded PPO adapter checks, without policy optimization or baseline trials."""
from copy import deepcopy
import json

import numpy as np
import pytest
from stable_baselines3.common.vec_env import VecNormalize

from rm27_contact_rl import ContactRecoveryEnv
from rm27_contact_rl.ppo import NormalizedCTBR, load_train_config, make_vec_env
from rm27_contact_rl.rm27_adapter import DEFAULT_CONFIG, ROOT


def short_task_config(tmp_path):
    """Exercise the existing deadline, with unchanged physical success gates."""
    scene = json.loads((ROOT / "configs/aggressive_contact.json").read_text())
    scene["horizon_s"] = .003
    scene_path = tmp_path / "short_scene.json"
    scene_path.write_text(json.dumps(scene))
    environment = json.loads(DEFAULT_CONFIG.read_text())
    environment["experiment_config"] = str(scene_path)
    environment_path = tmp_path / "short_environment.json"
    environment_path.write_text(json.dumps(environment))
    config = deepcopy(load_train_config())
    config["env_config"] = str(environment_path)
    return config


def rms_state(vec):
    return tuple(np.asarray(value).copy()
                 for value in (vec.obs_rms.mean, vec.obs_rms.var, vec.obs_rms.count,
                               vec.ret_rms.mean, vec.ret_rms.var, vec.ret_rms.count))


def test_normalized_ctbr_uses_physical_chain_and_hover_initialization():
    wrapped = NormalizedCTBR(ContactRecoveryEnv())
    direct = ContactRecoveryEnv()
    try:
        wrapped.reset(seed=0)
        direct.reset(seed=0)
        physical = wrapped.unwrapped
        hover = physical.plant.hover_force
        np.testing.assert_allclose(wrapped.hover_action, [-1 / 3, 0, 0, 0], atol=1e-7)
        np.testing.assert_allclose(wrapped.action(wrapped.hover_action), [hover, 0, 0, 0], atol=1e-7)
        assert wrapped.action(np.zeros(4))[0] == pytest.approx(2.20725, abs=1e-7)
        assert wrapped.action(np.zeros(4))[0] != pytest.approx(hover)
        np.testing.assert_allclose(wrapped.action([2, -2, 0, 3]),
                                   [4 * physical.plant.maximum, -20, 0, 20], atol=1e-6)

        data = physical.plant.data
        before = tuple(value.copy() for value in
                       (data.qpos, data.qvel, data.act, data.ctrl, physical.previous_action_ctbr))
        for invalid in ([0, 0, 0], [0, np.nan, 0, 0], [0, 0, 0, np.inf]):
            with pytest.raises(ValueError):
                wrapped.step(invalid)
            assert data.time == 0 and physical.evaluator.completed_intervals == 0
            for actual, expected in zip((data.qpos, data.qvel, data.act, data.ctrl,
                                         physical.previous_action_ctbr), before):
                np.testing.assert_array_equal(actual, expected)

        command = np.array([hover, 4, -2, 1.])
        normalized = np.array([wrapped.hover_action[0], .2, -.1, .05])
        observation, reward, terminated, truncated, info = wrapped.step(normalized)
        reference, ref_reward, ref_terminated, ref_truncated, ref_info = direct.step(command)
        np.testing.assert_allclose(observation, reference, rtol=1e-6, atol=1e-7)
        np.testing.assert_allclose(data.qpos, direct.plant.data.qpos, rtol=1e-7, atol=1e-9)
        np.testing.assert_allclose(data.qvel, direct.plant.data.qvel, rtol=1e-6, atol=1e-8)
        np.testing.assert_allclose(info["motor_commands_N"], ref_info["motor_commands_N"], atol=1e-7)
        np.testing.assert_allclose(observation[-5:-1], command, atol=1e-7)
        np.testing.assert_allclose(physical.previous_action_ctbr, command, atol=1e-7)
        assert (reward, terminated, truncated) == (ref_reward, ref_terminated, ref_truncated)
        assert info["physics_substeps"] == 5 and info["time_s"] == pytest.approx(.005)
    finally:
        wrapped.close()
        direct.close()


def test_vector_autoreset_preserves_deadline_terminal_state_and_result(tmp_path):
    vec = make_vec_env(short_task_config(tmp_path))
    try:
        assert vec.num_envs == 4
        assert vec.norm_obs and not vec.norm_reward
        vec.reset()
        actions = np.tile([-1 / 3, 0, 0, 0], (vec.num_envs, 1))
        observations, rewards, dones, infos = vec.step(actions)
        np.testing.assert_array_equal(dones, np.ones(4, dtype=bool))
        np.testing.assert_array_equal(rewards, np.full(4, -10))
        for index, info in enumerate(infos):
            assert not info["TimeLimit.truncated"]
            assert info["physics_substeps"] == 3
            assert info["time_s"] == pytest.approx(.003)
            assert not info["result"]["passed"]
            assert info["result"]["failure_modes"] == ["TARGET_MISS"]
            assert info["termination_reason"] == ["TARGET_MISS"]
            assert info["episode"]["l"] == 1
            terminal = vec.unnormalize_obs(info["terminal_observation"])
            restarted = vec.unnormalize_obs(observations[index])
            assert terminal[-1] == pytest.approx(0, abs=1e-7)
            assert restarted[-1] == pytest.approx(.003, abs=1e-7)
            env = vec.venv.envs[index].unwrapped
            assert env.plant.data.time == 0
            assert env._active and env._terminal_result is None
    finally:
        vec.close()


def test_normalization_restore_and_evaluation_are_frozen(tmp_path):
    config = short_task_config(tmp_path)
    training = make_vec_env(config, n_envs=1)
    evaluation = None
    restored = None
    try:
        training.reset()
        training.step(np.array([[-1 / 3, 0, 0, 0]]))
        stats_path = tmp_path / "vecnormalize.pkl"
        training.save(stats_path)
        expected_stats = rms_state(training)
        raw_observation = training.venv.envs[0].unwrapped._observation()
        expected_normalized = training.normalize_obs(raw_observation)

        evaluation = make_vec_env(config, n_envs=1, training=False)
        assert not evaluation.training and not evaluation.norm_reward
        frozen_initial = rms_state(evaluation)
        evaluation.reset()
        evaluation.step(np.array([[-1 / 3, 0, 0, 0]]))
        for actual, expected in zip(rms_state(evaluation), frozen_initial):
            np.testing.assert_array_equal(actual, expected)

        restored = VecNormalize.load(stats_path, evaluation.venv)
        restored.training = False
        restored.norm_reward = False
        for actual, expected in zip(rms_state(restored), expected_stats):
            np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(restored.normalize_obs(raw_observation), expected_normalized)
        restored.reset()
        restored.step(np.array([[-1 / 3, 0, 0, 0]]))
        for actual, expected in zip(rms_state(restored), expected_stats):
            np.testing.assert_array_equal(actual, expected)
    finally:
        training.close()
        # Loading replaces the normalization wrapper, retaining its new plant.
        if restored is not None:
            restored.close()
        elif evaluation is not None:
            evaluation.close()


def test_task_configuration_preserves_i3_and_bounds_i4(tmp_path):
    old = load_train_config()
    arrival_path = ROOT / "configs/agile_flight/ppo_terminal_arrival.json"
    config = load_train_config(arrival_path)
    assert old["budget"] == {"transitions": 65536, "learning_wall_s": 1800}
    assert config["budget"] == {"transitions": 524288, "learning_wall_s": 900}
    assert config["ppo"] == old["ppo"]
    invalid = [
        {"observation_schema": "contact_world_v1"},
        {"protocol": "i3_engineering"},
        {"n_envs": 5},
        {"seed": -1},
        {"ppo": dict(config["ppo"], batch_size=255)},
        {"budget": {"transitions": 2099200, "learning_wall_s": 900}},
        {"budget": {"transitions": 524288, "learning_wall_s": 3601}},
        {"budget": {"transitions": 524287, "learning_wall_s": 900}},
    ]
    path = tmp_path / "training.json"
    for difference in invalid:
        path.write_text(json.dumps(dict(config, **difference)))
        with pytest.raises(ValueError):
            load_train_config(path)


def test_hover_centered_arrival_parameterization_is_config_scoped(tmp_path):
    import torch
    from stable_baselines3 import PPO
    from rm27_contact_rl.arrival_env import AgileTerminalArrivalEnv

    first = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival.json")
    config = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_hover_centered.json")
    assert {k: v for k, v in config.items() if k not in ("action_parameterization", "ppo", "status")} == \
        {k: v for k, v in first.items() if k not in ("ppo", "status")}
    assert dict(config["ppo"], policy_kwargs=first["ppo"]["policy_kwargs"]) == first["ppo"]

    wrapped = NormalizedCTBR(AgileTerminalArrivalEnv(), config["action_parameterization"])
    direct = AgileTerminalArrivalEnv()
    try:
        wrapped.reset(options={"initial_coordinates": [1.2, 0., 0.]})
        direct.reset(options={"initial_coordinates": [1.2, 0., 0.]})
        plant = wrapped.unwrapped.plant
        hover, maximum = plant.hover_force, 4 * plant.maximum
        np.testing.assert_array_equal(wrapped.hover_action, np.zeros(4))
        np.testing.assert_allclose(wrapped.action(np.zeros(4)), [hover, 0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(wrapped.action([1, 1, -1, 2]), [maximum, 20, -20, 20], atol=1e-12)
        assert wrapped.action([-1, 0, 0, 0])[0] == pytest.approx(2 * hover - maximum)
        # Below-zero collective goes to the environment's reported input clip.
        _, _, _, _, info = wrapped.step([-1, .1, 0, 0])
        _, _, _, _, ref_info = direct.step([0., 2., 0., 0.])
        assert info["action_clipped"] and ref_info["action_clipped"] is False
        np.testing.assert_allclose(wrapped.unwrapped.previous_action_ctbr, [0, 2, 0, 0], atol=1e-12)
        np.testing.assert_allclose(info["motor_commands_N"], ref_info["motor_commands_N"], atol=1e-12)
    finally:
        wrapped.close()
        direct.close()

    torch.set_num_threads(1)
    vec = make_vec_env(config, n_envs=1)
    try:
        model = PPO("MlpPolicy", vec, seed=0, device="cpu", **config["ppo"])
        np.testing.assert_array_equal(model.policy.log_std.detach().numpy(), -np.ones(4))
    finally:
        vec.close()

    path = tmp_path / "training.json"
    contact = load_train_config()
    for invalid in (dict(contact, action_parameterization="hover_centered_v1"),
                    dict(contact, ppo=dict(contact["ppo"], policy_kwargs=dict(
                        contact["ppo"]["policy_kwargs"], log_std_init=-1.))),
                    dict(config, action_parameterization="tanh"),
                    dict(config, ppo=dict(config["ppo"], policy_kwargs=dict(
                        config["ppo"]["policy_kwargs"], log_std_init=.5)))):
        path.write_text(json.dumps(invalid))
        with pytest.raises(ValueError):
            load_train_config(path)


def test_informed_reset_is_training_only_and_dynamically_consistent(tmp_path):
    from stable_baselines3.common.monitor import Monitor
    from rm27_contact_rl.arrival_env import AgileTerminalArrivalEnv
    from rm27_contact_rl.baselines import FlatnessReference
    from rm27_contact_rl.ppo import InformedReset, InformedResetCurriculum

    second = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_hover_centered.json")
    config = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_informed_reset.json")
    assert {k: v for k, v in config.items() if k not in ("informed_reset", "status")} == \
        {k: v for k, v in second.items() if k != "status"}
    control = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_informed_reset_k8.json")
    varied = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_informed_reset_k8_p08.json")
    assert dict(control, informed_reset=config["informed_reset"], status=None) == dict(config, status=None)
    assert control["informed_reset"] == dict(config["informed_reset"], reference_reuse_episodes=8)
    assert dict(varied, status=None, informed_reset=dict(varied["informed_reset"], probability=.5)) == \
        dict(control, status=None)
    assert varied["informed_reset"]["probability"] == .8
    band = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_informed_reset_k8_p08_band.json")
    assert dict(band, status=None, informed_reset=varied["informed_reset"]) == dict(varied, status=None)
    assert band["informed_reset"] == dict(varied["informed_reset"], advance_statistic="frontier_band_v1")
    precision = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_informed_band_precision.json")
    assert dict(precision, status=None, env_config=band["env_config"]) == dict(band, status=None)
    assert precision["env_config"] == "configs/agile_flight/terminal_arrival_precision.json"
    moderate = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_moderate_precision.json")
    assert dict(moderate, status=None, env_config=precision["env_config"]) == dict(precision, status=None)
    long = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_moderate_precision_long.json")
    assert dict(long, status=None, budget=moderate["budget"]) == dict(moderate, status=None)
    assert long["budget"] == {"transitions": 2097152, "learning_wall_s": 3600}

    spec = dict(config["informed_reset"], probability=1., initial_lead_s=.2, min_lead_s=.2)
    curriculum = InformedResetCurriculum(spec)
    informed = Monitor(InformedReset(AgileTerminalArrivalEnv(), curriculum))
    direct = AgileTerminalArrivalEnv()
    try:
        observation, info = informed.reset(seed=5)
        base = informed.unwrapped
        prefix = spec["reference_duration_s"] - spec["initial_lead_s"]
        assert info["informed_prefix_s"] == pytest.approx(prefix)
        assert observation[-1] == pytest.approx(base.horizon_s - prefix)
        assert base.reward_component_totals == dict(progress=0., cost=0., event=0.)
        # The same reference flown directly from the same coordinates gives the same state.
        direct.reset(options={"initial_coordinates": base.initial_coordinates})
        reference = FlatnessReference(direct, spec["reference_duration_s"])
        while direct.plant.data.time < prefix - 1e-9:
            direct.step(reference.action(direct))
        for name in ("qpos", "qvel", "act", "ctrl"):
            np.testing.assert_array_equal(getattr(base.plant.data, name), getattr(direct.plant.data, name))
        np.testing.assert_array_equal(base.previous_action_ctbr, direct.previous_action_ctbr)
        terminated = False
        while not terminated:
            _, _, terminated, _, info = informed.step(base.previous_action_ctbr.copy())
        assert info["episode"]["r"] == pytest.approx(info["result"]["total_reward"], abs=1e-6)
        assert info["result"]["informed_prefix_s"] == pytest.approx(prefix)
        assert len(curriculum.window) == 1
        # Explicit evaluation resets are never informed.
        _, info = informed.reset(options={"initial_coordinates": [1.2, 0., 0.]})
        assert "informed_prefix_s" not in info and base.informed_prefix_s == 0
    finally:
        informed.close()
        direct.close()

    for _ in range(spec["advance_window_episodes"] - 1):
        curriculum.record(True, .05)
    state = curriculum.update()
    assert state["advanced"] and curriculum.lead_s == pytest.approx(.3) and curriculum.window == []
    assert not curriculum.update()["advanced"]

    # The frontier-band statistic ignores short, already mastered leads.
    band = InformedResetCurriculum(dict(spec, advance_statistic="frontier_band_v1", initial_lead_s=.4))
    for _ in range(spec["advance_window_episodes"]):
        band.record(True, .1)
    assert band.window == [] and not band.update()["advanced"]
    for success in [True] * 16 + [False] * 16:
        band.record(success, .35)
    assert band.update()["advanced"] and band.lead_s == pytest.approx(.5)

    training, evaluation = make_vec_env(config, n_envs=1), make_vec_env(config, n_envs=1, training=False)
    try:
        assert training.informed_curriculum is not None and evaluation.informed_curriculum is None
    finally:
        training.close()
        evaluation.close()

    path = tmp_path / "training.json"
    for invalid in (dict(load_train_config(), informed_reset=config["informed_reset"]),
                    dict(config, informed_reset=dict(config["informed_reset"], min_lead_s=.2)),
                    dict(config, informed_reset=dict(config["informed_reset"], reference="teacher")),
                    dict(config, informed_reset=dict(config["informed_reset"], extra=1)),
                    dict(config, informed_reset=dict(config["informed_reset"], advance_statistic="deterministic"))):
        path.write_text(json.dumps(invalid))
        with pytest.raises(ValueError):
            load_train_config(path)


def test_reference_snapshot_reuse_matches_a_fresh_prefix_bit_for_bit():
    from rm27_contact_rl.arrival_env import AgileTerminalArrivalEnv
    from rm27_contact_rl.baselines import FlatnessReference
    from rm27_contact_rl.ppo import InformedReset, InformedResetCurriculum

    def same(left, right):
        if isinstance(left, dict):
            return left.keys() == right.keys() and all(same(left[k], right[k]) for k in left)
        if isinstance(left, (list, tuple)):
            return len(left) == len(right) and all(same(a, b) for a, b in zip(left, right))
        if isinstance(left, np.ndarray):
            return np.array_equal(left, right)
        return left == right or (left != left and right != right)

    config = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival_informed_reset.json")
    spec = dict(config["informed_reset"], probability=1., initial_lead_s=.6, reference_reuse_episodes=3)
    curriculum = InformedResetCurriculum(spec)
    reused, fresh = InformedReset(AgileTerminalArrivalEnv(), curriculum), AgileTerminalArrivalEnv()
    fields = [
        "time", "qpos", "qvel", "act", "ctrl", "qacc", "qacc_warmstart", "act_dot", "xpos", "xquat",
        "sensordata", "efc_force", "qfrc_actuator", "actuator_force"]
    try:
        for episode in range(5):  # crosses one reference refresh
            observation, info = reused.reset(seed=11 if episode == 0 else None)
            base = reused.unwrapped
            fresh.reset(options={"initial_coordinates": base.initial_coordinates})
            reference = FlatnessReference(fresh, spec["reference_duration_s"])
            while fresh.plant.data.time < base.plant.data.time - 1e-12:
                fresh.step(reference.action(fresh))
            fresh.start_scored_segment()
            for name in fields:
                np.testing.assert_array_equal(getattr(base.plant.data, name), getattr(fresh.plant.data, name), err_msg=name)
            np.testing.assert_array_equal(observation, fresh._observation())
            for name in ("previous_action_ctbr", "informed_prefix_s", "reward_component_totals",
                         "action_clipped_time_s", "motor_command_saturation_time_s", "episode_start"):
                assert same(getattr(base, name), getattr(fresh, name)), name
            assert same(base.evaluator.summary(base.plant.model, base.plant.data),
                        fresh.evaluator.summary(fresh.plant.model, fresh.plant.data))
            for action in ([1.2, 3., -2., 1.], [1.6, -1., 0., 0.], [1.4, 0., 2., -1.]):
                a, b = reused.step(np.array(action)), fresh.step(np.array(action))
                np.testing.assert_array_equal(a[0], b[0])
                assert a[1] == b[1] and a[2] == b[2]
                if a[2]:
                    assert same(a[4]["result"], b[4]["result"])
                    break
        assert curriculum.reference_flights == 2
    finally:
        reused.close()
        fresh.close()


def test_arrival_vector_autoreset_keeps_task_result_and_frozen_evaluation(tmp_path):
    task = json.loads((ROOT / "configs/agile_flight/terminal_arrival.json").read_text())
    task["horizon_s"] = .003
    env_path = tmp_path / "arrival.json"
    env_path.write_text(json.dumps(task))
    config = deepcopy(load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival.json"))
    config["env_config"] = str(env_path)
    vec = make_vec_env(config, training=False)
    try:
        before = rms_state(vec)
        observations = vec.reset()
        assert observations.shape == (4, 26)
        _, rewards, dones, infos = vec.step(np.tile([-1 / 3, 0, 0, 0], (4, 1)))
        assert dones.all() and np.isfinite(rewards).all()
        for info in infos:
            assert info["task_id"] == "agile_terminal_arrival_v1"
            assert info["observation_schema"] == "arrival_world_v1"
            assert info["result"]["failure_modes"] == ["ARRIVAL_TIMEOUT"]
            assert info["result"]["episode_elapsed_s"] == pytest.approx(.003)
            assert info["physics_substeps"] == 3 and not info["TimeLimit.truncated"]
            assert sum(info["result"]["reward_component_totals"].values()) == pytest.approx(info["episode"]["r"], abs=1e-6)
            assert "terminal_observation" in info
            assert info["result"]["task_id"] == "agile_terminal_arrival_v1"
        for actual, expected in zip(rms_state(vec), before):
            np.testing.assert_array_equal(actual, expected)
    finally:
        vec.close()


def test_checkpoint_task_schema_and_legacy_contact_guard_without_optimization(tmp_path):
    import torch
    from stable_baselines3 import PPO
    from rm27_contact_rl.ppo import _load_checkpoint, _model_state, _save_checkpoint
    from rm27_contact_rl.provenance import provenance

    config = load_train_config(ROOT / "configs/agile_flight/ppo_terminal_arrival.json")
    torch.set_num_threads(1)
    vec = make_vec_env(config, n_envs=1)
    restored = None
    try:
        model = PPO("MlpPolicy", vec, seed=0, device="cpu", **config["ppo"])
        vec.reset()
        source = provenance(vec.venv.envs[0].unwrapped, algorithm="PPO_CPU", seed=0)
        checkpoint = tmp_path / "arrival_checkpoint"
        meta = _save_checkpoint(checkpoint, model, vec, config, source, 0)
        recovered, restored, recovered_meta = _load_checkpoint(
            checkpoint, training=False, n_envs=1, expected_task="agile_terminal_arrival_v1")
        assert recovered_meta["task"] == {
            "task_id": "agile_terminal_arrival_v1", "observation_schema": "arrival_world_v1"}
        assert _model_state(recovered, restored) == meta["state"]
        assert not restored.training and not restored.norm_reward
        with pytest.raises(ValueError, match="different task"):
            _load_checkpoint(checkpoint, training=False, expected_task="contact_recovery_v1")
        meta["task"]["observation_schema"] = "contact_world_v1"
        (checkpoint / "checkpoint.json").write_text(json.dumps(meta))
        with pytest.raises(ValueError, match="task/schema"):
            _load_checkpoint(checkpoint, training=False)
        del meta["task"]
        (checkpoint / "checkpoint.json").write_text(json.dumps(meta))
        with pytest.raises(ValueError, match="Contact-only"):
            _load_checkpoint(checkpoint, training=False)
    finally:
        vec.close()
        if restored is not None:
            restored.close()

    # An actual checkpoint with omitted legacy metadata still loads as Contact.
    config = load_train_config()
    vec = make_vec_env(config, n_envs=1)
    restored = None
    try:
        model = PPO("MlpPolicy", vec, seed=0, device="cpu", **config["ppo"])
        vec.reset()
        source = provenance(vec.venv.envs[0].unwrapped, algorithm="PPO_CPU", seed=0)
        checkpoint = tmp_path / "legacy_contact"
        meta = _save_checkpoint(checkpoint, model, vec, config, source, 0)
        del meta["task"]
        (checkpoint / "checkpoint.json").write_text(json.dumps(meta))
        recovered, restored, _ = _load_checkpoint(checkpoint, training=False, n_envs=1)
        assert _model_state(recovered, restored) == meta["state"]
    finally:
        vec.close()
        if restored is not None:
            restored.close()
