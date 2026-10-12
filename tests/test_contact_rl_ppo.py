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
