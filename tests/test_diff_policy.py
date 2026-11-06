"""Bounded checks of the I4 differentiable actor/critic integration contract.

These tests perform no optimization loop. Policy improvement and checkpoint
evaluation are exercised by the separately budgeted E1-A smoke runner.
"""
from copy import deepcopy

import mujoco
import numpy as np
import pytest
import torch
from scipy.spatial.transform import Rotation

from rm27_contact_physics.arrival import ArrivalEvaluator
from rm27_contact_rl.arrival_env import AgileTerminalArrivalEnv
from rm27_contact_rl.diff_dynamics import DynamicsParameters, step_control
from rm27_contact_rl.diff_policy import (
    ActorCritic,
    arrival_observation,
    load_checkpoint,
    physical_action,
    rollout_window,
    save_checkpoint,
)
from rm27_contact_rl.diff_validation import Budget
from rm27_contact_rl.ppo import NormalizedCTBR


@pytest.fixture
def env():
    instance = AgileTerminalArrivalEnv("configs/agile_flight/terminal_arrival_moderate_precision.json")
    instance.reset(seed=7, options={"initial_coordinates": [1.2, 0., 0.]})
    try:
        yield instance
    finally:
        instance.close()


def state(env):
    data = env.plant.data
    return torch.tensor(np.r_[data.qpos, data.qvel, data.act], dtype=torch.float64)


def hover_policy(env):
    policy = ActorCritic(env, hidden_size=16)
    with torch.no_grad():
        for parameter in policy.actor.parameters():
            parameter.zero_()
    return policy


def bounded_rollout(env, policy, *, steps=16):
    # A test cannot accidentally grow into an episode/training run.
    budget = Budget(dict(reference_transition_cap=16, physics_substep_cap=80, wall_seconds_cap=30))
    return rollout_window(env, policy, steps=steps, budget=budget)


def fresh_evaluator(env):
    env.evaluator = ArrivalEvaluator(env.rl_config, env.plant.physical, env.episode_start)
    env.evaluator.initialize(env.plant.model, env.plant.data, env.plant.state)


def test_actor_observation_matches_existing_environment_after_action(env):
    env.plant.reset(position=[.3, -.1, 1.7], velocity=[.2, -.15, .1],
                    rotation=Rotation.from_rotvec([.25, -.3, .2]).as_matrix(),
                    omega_body=[.3, -.4, .2])
    fresh_evaluator(env)
    env.step(np.array([1.7, .2, -.3, .4]))
    observation = arrival_observation(state(env), torch.tensor(env.previous_action_ctbr),
                                      env.plant.data.time - env.episode_start, env)
    assert observation.shape == (26,)
    np.testing.assert_array_equal(observation.detach().numpy(), env._observation().astype(np.float64))
    # Scoring history is neither an input nor an actor-observation shortcut.
    env.evaluator.last_metrics["velocity_error_m_s"] = 999.
    env.evaluator.events.append(dict(event="SCORING_ONLY"))
    unchanged = arrival_observation(state(env), torch.tensor(env.previous_action_ctbr),
                                    env.plant.data.time - env.episode_start, env)
    torch.testing.assert_close(unchanged, observation, rtol=0, atol=0)


def test_internal_motor_state_changes_dynamics_without_actor_privilege(env):
    torch.manual_seed(11)
    policy = ActorCritic(env, hidden_size=16)
    initial = state(env)
    other = initial.clone()
    other[13:] *= .25
    previous = torch.tensor(env.previous_action_ctbr)
    first = arrival_observation(initial, previous, 0., env)
    second = arrival_observation(other, previous, 0., env)
    torch.testing.assert_close(first, second, rtol=0, atol=0)
    torch.testing.assert_close(policy.act(first), policy.act(second), rtol=0, atol=0)
    parameters = DynamicsParameters.from_env(env)
    action = physical_action(policy.act(first), env)
    a = step_control(initial, action, parameters).state
    b = step_control(other, action, parameters).state
    assert torch.linalg.vector_norm(a[7:10] - b[7:10]) > 1e-4


@pytest.mark.parametrize("parameterization", ["bounds_affine_v1", "hover_centered_v1"])
@pytest.mark.parametrize("normalized", [[.3, -.4, .5, -.6], [-2., 2., -3., 4.]])
def test_normalized_action_mapping_matches_existing_wrapper(env, parameterization, normalized):
    requested = torch.tensor(normalized, dtype=torch.float64)
    expected = NormalizedCTBR(env, parameterization).action(requested.numpy())
    actual = physical_action(requested, env, parameterization=parameterization)
    np.testing.assert_array_equal(actual.detach().numpy(), expected)


def test_fully_clipped_action_has_zero_gradient_without_removing_saturation(env):
    normalized = torch.tensor([-2., 2., -2., 2.], dtype=torch.float64, requires_grad=True)
    physical = physical_action(normalized, env)
    result = step_control(state(env), physical, DynamicsParameters.from_env(env))
    gradient, = torch.autograd.grad(result.state.square().sum(), normalized)
    torch.testing.assert_close(gradient, torch.zeros(4, dtype=torch.float64), rtol=0, atol=0)
    assert bool(((result.raw_motor_commands < 0) |
                 (result.raw_motor_commands > env.plant.maximum)).any())
    assert bool(((result.motor_commands == 0) |
                 (result.motor_commands == env.plant.maximum)).any())


def test_full_80_ms_chain_has_actor_gradient_and_detached_critic_target(env):
    torch.manual_seed(19)
    policy = ActorCritic(env, hidden_size=16)
    window = bounded_rollout(env, policy)
    assert not window.terminated
    assert window.substeps == [5] * 16
    assert len(window.rewards) == 16
    assert env.plant.data.time == pytest.approx(.08)
    assert torch.isfinite(window.objective)
    assert not window.critic_target.requires_grad
    assert window.final_observation.shape == (26,)
    endpoint_gradient, = torch.autograd.grad(window.bootstrap, window.final_observation,
                                            retain_graph=True)
    assert torch.isfinite(endpoint_gradient).all()
    assert torch.linalg.vector_norm(endpoint_gradient) > 0
    window.objective.backward()
    actor_gradients = [p.grad for p in policy.actor.parameters()]
    assert all(g is not None and torch.isfinite(g).all() for g in actor_gradients)
    assert sum(float(g.square().sum()) for g in actor_gradients) > 0
    assert all(p.grad is None for p in policy.critic.parameters())
    # Critic training uses a detached target and has no actor-gradient route.
    policy.zero_grad(set_to_none=True)
    loss = (policy.value(window.initial_observation.detach()) - window.critic_target).square().mean()
    loss.backward()
    assert all(p.grad is None for p in policy.actor.parameters())
    critic_gradients = [p.grad for p in policy.critic.parameters()]
    assert all(g is not None and torch.isfinite(g).all() for g in critic_gradients)
    assert sum(float(g.square().sum()) for g in critic_gradients) > 0


@pytest.mark.parametrize("kind,expected_substeps", [
    ("arrival_2", 2), ("arrival_5", 5), ("safety_5", 5), ("timeout_3", 3),
])
def test_first_terminal_substep_stops_rollout_and_disables_bootstrap(env, kind, expected_substeps):
    geometry = env.geometry
    if kind.startswith("arrival"):
        distance = .0425 if kind == "arrival_2" else .0505
        env.plant.reset(position=geometry["position_target"] + distance * geometry["normal"],
                        rotation=geometry["target_rotation"], velocity=geometry["velocity_target"])
    elif kind.startswith("safety"):
        env.plant.reset(position=[1.3955, 0., 2.], velocity=[1., 0., 0.])
    else:
        env.rl_config = deepcopy(env.rl_config)
        env.rl_config["horizon_s"] = env.horizon_s = .003
    fresh_evaluator(env)
    policy = hover_policy(env)
    result = bounded_rollout(env, policy)
    assert result.terminated
    assert result.substeps == [expected_substeps]
    assert len(result.rewards) == 1 and len(result.observations) == len(result.actions) == 1
    assert float(result.bootstrap) == 0
    assert env.plant.data.time == pytest.approx(expected_substeps * .001)
    torch.testing.assert_close(result.final_state.detach(), state(env), rtol=1e-11, atol=1e-12)
    assert env.evaluator.done
    assert env.evaluator.arrived == kind.startswith("arrival")
    if kind.startswith("timeout"):
        assert "ARRIVAL_TIMEOUT" in env.evaluator.reasons
    elif kind.startswith("safety"):
        assert "FLIGHT_VOLUME_EXIT" in env.evaluator.reasons
    terminal_position = env.plant.data.qpos.copy()
    empty = bounded_rollout(env, policy)
    assert empty.terminated and empty.substeps == [] and empty.rewards == []
    assert float(empty.objective) == float(empty.bootstrap) == 0
    np.testing.assert_array_equal(env.plant.data.qpos, terminal_position)


def test_checkpoint_roundtrip_preserves_actor_critic_and_predict_interface(env, tmp_path):
    torch.manual_seed(31)
    policy = ActorCritic(env, hidden_size=16)
    path = tmp_path / "policy.pt"
    save_checkpoint(path, policy, env, metadata=dict(test="roundtrip"))
    restored = load_checkpoint(path, env)
    observation = env._observation()
    np.testing.assert_array_equal(policy.predict(observation)[0], restored.predict(observation)[0])
    tensor = torch.tensor(observation.astype(np.float64))
    torch.testing.assert_close(policy.value(tensor), restored.value(tensor), rtol=0, atol=0)
    assert policy.predict(np.stack([observation, observation]))[0].shape == (2, 4)


def test_checkpoint_rejects_changed_environment_config(env, tmp_path):
    path = tmp_path / "policy.pt"
    save_checkpoint(path, ActorCritic(env, hidden_size=16), env)
    env.rl_config = deepcopy(env.rl_config)
    env.rl_config["reward"]["progress_gain"] += .1
    with pytest.raises(ValueError):
        load_checkpoint(path, env)


@pytest.mark.parametrize("field", ["action_parameterization", "mujoco_version", "sources"])
def test_checkpoint_rejects_changed_action_version_or_source_contract(env, tmp_path, field):
    path = tmp_path / "policy.pt"
    save_checkpoint(path, ActorCritic(env, hidden_size=16), env)
    payload = torch.load(path, weights_only=True)
    if field == "sources":
        name = next(iter(payload["contract"][field]))
        payload["contract"][field][name] = "0" * 64
    else:
        payload["contract"][field] = "unsupported_contract"
    changed = tmp_path / "changed.pt"
    torch.save(payload, changed)
    with pytest.raises(ValueError, match="contract mismatch"):
        load_checkpoint(changed, env)


def test_mass_randomization_path_is_rejected_before_any_reference_step(env):
    policy = ActorCritic(env, hidden_size=16)
    env.set_dynamics_scales(mass=1.15, thrust=1.)
    before = env.plant.data.time
    with pytest.raises(ValueError, match="nominal"):
        bounded_rollout(env, policy)
    assert env.plant.data.time == before


def test_initial_contact_has_no_actor_call_or_bootstrap(env, monkeypatch):
    env.plant.reset(position=[0., 0., .01])
    fresh_evaluator(env)
    policy = hover_policy(env)

    def forbidden_actor(_):
        raise AssertionError("A terminal initial state must not query the actor")

    monkeypatch.setattr(policy, "act", forbidden_actor)
    result = bounded_rollout(env, policy)
    assert result.terminated and result.substeps == [] and result.actions == []
    assert float(result.objective) == float(result.bootstrap) == 0
    assert env.plant.data.time == 0
    assert "GROUND_CONTACT" in env.evaluator.reasons


def test_saturation_history_survives_the_window_boundary(env, monkeypatch):
    policy = hover_policy(env)
    env.evaluator.sat_run = env.rl_config["safety"]["max_saturation_run_s"] - .0005
    monkeypatch.setattr(policy, "act", lambda observation: observation.new_tensor([0., 1., -1., 1.]))
    result = bounded_rollout(env, policy)
    assert result.terminated and result.substeps == [1]
    assert float(result.bootstrap) == 0
    assert result.diagnostics["motor_saturated_controls"] == 1
    assert result.diagnostics["motor_at_bound_controls"] == 1
    assert env.evaluator.sat_run > env.rl_config["safety"]["max_saturation_run_s"]
