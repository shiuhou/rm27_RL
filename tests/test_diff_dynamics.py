"""Actual compiled I4 dynamics, including activation delay and full CTBR hold."""
import mujoco
import numpy as np
import pytest
import torch
from scipy.spatial.transform import Rotation

from rm27_contact_rl.arrival_env import AgileTerminalArrivalEnv
from rm27_contact_rl.diff_dynamics import DynamicsParameters, step_control
from rm27_contact_rl.diff_validation import Budget, reference
from rm27_contact_rl.diff_analysis import agreement, classify, event_mask


@pytest.fixture
def env():
    instance = AgileTerminalArrivalEnv("configs/agile_flight/terminal_arrival_moderate_precision.json")
    instance.reset(seed=7)
    yield instance
    instance.close()


def state(plant):
    return np.r_[plant.data.qpos, plant.data.qvel, plant.data.act]


@pytest.mark.parametrize("mass,thrust,omega,action", [
    (1., 1., [0., 0., 0.], [1.4715, 0., 0., 0.]),
    (1., 1., [0., 0., 0.], [3., 2., -1., .5]),
    (1., 1., [8., -9., 6.], [1.8, -10., 3., -5.]),
    (1., .85, [3., -4., 5.], [6., -25., 24., 21.]),
    (1., 1.15, [-5., 4., -3.], [-1., 25., -23., -21.]),
])
def test_all_physical_endpoints_match_mujoco(env, mass, thrust, omega, action):
    env.set_dynamics_scales(mass=mass, thrust=thrust)
    p = env.plant
    p.reset(position=[0., 0., 2.], velocity=[.4, -.3, .2], omega_body=omega,
            rotation=Rotation.from_rotvec([.4, -.2, .3]).as_matrix())
    p.data.act[:] = [.1, .3, .6, .9]
    mujoco.mj_forward(p.model, p.data)
    params = DynamicsParameters.from_env(env)
    before = state(p)
    initial = torch.tensor(before)
    result = step_control(initial, torch.tensor(action, dtype=torch.float64), params)
    commands, _ = p.apply_ctbr(np.clip(action, env._action_low, env._action_high))
    np.testing.assert_allclose(result.motor_commands.numpy(), commands, atol=1e-14)
    for j in range(5):
        mujoco.mj_step(p.model, p.data)
        mujoco.mj_forward(p.model, p.data)
        assert p.data.ncon == 0
        np.testing.assert_allclose(result.endpoints[j].numpy(), state(p), rtol=1e-11, atol=1e-12)
    np.testing.assert_array_equal(initial.numpy(), before)


def test_zero_rate_gradient_and_batch_are_finite(env):
    p = env.plant
    x = torch.tensor(state(p), requires_grad=True)
    u = torch.tensor([p.hover_force, 0., 0., 0.], dtype=torch.float64, requires_grad=True)
    params = DynamicsParameters.from_env(env)
    assert torch.autograd.gradcheck(lambda a: step_control(x, a, params).state, (u,), fast_mode=True)
    assert torch.autograd.gradgradcheck(lambda a: step_control(x, a, params).state, (u,), fast_mode=True)
    batch = step_control(x.repeat(3, 1), u.repeat(3, 1), params)
    torch.testing.assert_close(batch.state, step_control(x, u, params).state.repeat(3, 1))


def test_unsupported_dynamics_and_invalid_input_are_rejected(env):
    params = DynamicsParameters.from_env(env)
    x = torch.tensor(state(env.plant))
    u = torch.tensor([1., 0., 0., 0.], dtype=torch.float64)
    bad = x.clone()
    bad[3:7] *= 2
    with pytest.raises(ValueError, match="normalized"):
        step_control(bad, u, params)
    with pytest.raises(ValueError, match="matching"):
        step_control(x, u.float(), params)
    env.plant.model.actuator_actearly[0] = True
    with pytest.raises(ValueError, match="Unsupported"):
        DynamicsParameters.from_env(env)


def test_existing_runtime_mass_scaling_is_not_silently_reinterpreted(env):
    env.set_dynamics_scales(mass=1.15, thrust=1.)
    mujoco.mj_forward(env.plant.model, env.plant.data)
    matrix = np.zeros((6, 6))
    mujoco.mj_fullM(env.plant.model, env.plant.data, matrix)
    assert matrix[0, 0] != pytest.approx(env.plant.model.body_mass[env._uav_body])
    with pytest.raises(ValueError, match="mass cache"):
        DynamicsParameters.from_env(env)


def test_reference_budget_stops_before_an_extra_transition(env):
    budget = Budget(dict(reference_transition_cap=1, physics_substep_cap=5, wall_seconds_cap=30))
    commands = np.tile([env.plant.hover_force, 0., 0., 0.], (2, 1))
    with pytest.raises(RuntimeError, match="budget exhausted"):
        reference(env, state(env.plant), commands, budget)
    assert (budget.controls, budget.substeps) == (1, 5)
    assert env.plant.data.time == pytest.approx(.005)


def test_reference_preserves_partial_control_terminal_event(env):
    geometry = env.geometry
    env.plant.reset(position=geometry["position_target"] + .0425 * geometry["normal"],
                    rotation=geometry["target_rotation"], velocity=geometry["velocity_target"])
    budget = Budget(dict(reference_transition_cap=2, physics_substep_cap=10, wall_seconds_cap=30))
    commands = np.tile([env.plant.hover_force, 0., 0., 0.], (2, 1))
    values, steps, event = reference(env, state(env.plant), commands, budget)
    assert event["arrived"] and event["partial_controls"] == 1
    assert steps.tolist() == [2, 0]
    assert np.isfinite(values[0]).all() and np.isnan(values[1]).all()
    assert budget.substeps == 2 and env.plant.data.time == pytest.approx(.002)


def test_zero_derivatives_use_absolute_error_and_no_cosine():
    reference = np.zeros((1, 17))
    candidate = reference.copy()
    candidate[0, 0] = 1e-8
    result = agreement(candidate, reference, np.array([True]), rtol=.05)["all"]
    assert result["near_zero"] == 1 and result["failures"] == 0
    assert result["min_cosine"] is None and result["max_relative"] is None
    candidate[0, 0] = 1e-4
    assert agreement(candidate, reference, np.array([True]), rtol=.05)["all"]["failures"] == 1


def test_event_on_fifth_substep_is_excluded_from_smooth_derivative():
    event = dict(arrived=True, reasons=[], elapsed_s=.005)
    assert event_mask(event, [1, 8, 32, 128]).all()
    event["elapsed_s"] = .006
    assert event_mask(event, [1, 8, 32, 128]).tolist() == [False, True, True, True]


def test_strict_saturation_and_boundary_are_separate(env):
    params = DynamicsParameters.from_env(env)
    initial = np.tile(state(env.plant), (3, 1))
    actions = np.array([[[env.plant.hover_force, 0., 0., 0.]],
                        [[-.2, 25., 0., 0.]], [[0., 0., 0., 0.]]])
    budget = Budget(dict(reference_transition_cap=0, physics_substep_cap=0, wall_seconds_cap=30))
    assert classify(initial, actions, params, [1], budget).ravel().tolist() == [
        "interior", "strict_saturation", "boundary"]
    assert budget.controls == budget.substeps == 0
