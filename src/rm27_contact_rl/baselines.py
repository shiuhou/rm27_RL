"""Privileged existing Skill projected through the environment's CTBR chain."""
from __future__ import annotations

import math

import numpy as np

from rm27_contact_physics.control import Septic, Skill
from rm27_contact_physics.model import TargetState, frame
from rm27_contact_physics.teacher_interface import project


class CTBRBaseline:
    """Uses true contact events; equal action chain is not equal observation access."""

    privileged = True

    def __init__(self, env):
        self.reset(env)

    def reset(self, env):
        if not env._active:
            raise RuntimeError("Reset the environment before constructing/resetting its baseline")
        plant = env.plant
        target = TargetState(env.target_position.copy(), env.target_normal.copy(),
                             np.eye(3) * plant.cfg["uncertainty"]["target_position_std_m"] ** 2,
                             plant.state.timestamp)
        self.skill = Skill(plant.cfg, dict(plant.cfg["nominal_skill"]),
                           plant.state, target, plant.info, marker_config=plant.marker_config)
        self.teacher_saturated = False

    def action(self, env):
        if not env._active:
            raise RuntimeError("Baseline cannot command an inactive episode")
        state = env.plant.state
        motors, self.teacher_saturated = self.skill.command(state, env.last_step_contact)
        ctbr = project(motors, state.omega_body, self.skill.inertia,
                       self.skill.mixer, env.plant.rate_gain)
        # Env owns physical input clipping and proxy allocation for every policy.
        return np.r_[ctbr.collective_force_N, ctbr.body_rates_flu_rad_s]

    def observe_applied(self, env):
        self.skill.previous_motor_command = env.plant.data.ctrl.copy()

    def step(self, env):
        transition = env.step(self.action(env))
        self.observe_applied(env)
        transition[-1]["baseline_teacher_saturated"] = bool(self.teacher_saturated)
        transition[-1]["baseline_privileged"] = True
        return transition


class ArrivalCTBRBaseline(CTBRBaseline):
    """Fixed boundary reference; no contact FSM or PPO imitation labels."""

    privileged = False

    def reset(self, env):
        if not env._active:
            raise RuntimeError("Reset Arrival before constructing its baseline")
        plant = env.plant
        target = TargetState(env.geometry["center"].copy(), env.target_normal.copy(),
                             np.zeros((3, 3)), plant.state.timestamp)
        self.skill = Skill(plant.cfg, dict(plant.cfg["nominal_skill"]), plant.state,
                           target, plant.info, marker_config=plant.marker_config)
        self.teacher_saturated = False
        self.duration_s = float(env.rl_config["baseline"]["duration_s"])
        normal = env.target_normal
        gravity = -float(plant.model.opt.gravity[2])
        if normal[2] <= 0:
            raise ValueError("Declared terminal thrust alignment requires positive vertical component")
        acceleration = gravity / normal[2] * normal - np.array([0., 0., gravity])
        start = (plant.state.position, plant.state.velocity, np.zeros(3), np.zeros(3))
        end = (env.geometry["position_target"], env.geometry["velocity_target"], acceleration, np.zeros(3))
        self.trajectory = Septic(start, end, self.duration_s)
        self.start_time = plant.state.timestamp

    def action(self, env):
        if not env._active:
            raise RuntimeError("Baseline cannot command an inactive episode")
        state = env.plant.state
        elapsed = state.timestamp - self.start_time
        desired = self.trajectory.sample(elapsed)
        if elapsed > self.duration_s:
            h = elapsed - self.duration_s
            p, v, a, j = desired
            desired = (p + v * h + .5 * a * h * h, v + a * h, a, j)
        motors, self.teacher_saturated = self.skill.track(state, desired)
        action = project(motors, state.omega_body, self.skill.inertia,
                         self.skill.mixer, env.plant.rate_gain)
        return np.r_[action.collective_force_N, action.body_rates_flu_rad_s]

    def step(self, env):
        transition = env.step(self.action(env))
        self.observe_applied(env)
        transition[-1]["baseline_controller_saturated"] = bool(self.teacher_saturated)
        transition[-1]["baseline_privileged"] = False
        return transition


class FlatnessReference:
    """Offline feasible arrival reference used only to generate training reset states.

    A degree-9 flat-output polynomial runs from the current reset state (hover,
    zero derivatives) to the declared terminal state with the traditional
    reference's terminal thrust (along n) and zero jerk/snap, so the terminal
    body rate is zero and the attitude is ``frame(n)``. Collective and body
    rates are inverted through the profile motor lag and the nominal rate-loop
    gain. It supplies no policy labels, observations or rewards.
    """

    def __init__(self, env, duration_s, grid_dt_s=1e-3):
        if not env._active or env.plant.state.timestamp != env.episode_start:
            raise RuntimeError("Construct the reference immediately after an Arrival reset")
        self.duration_s = float(duration_s)
        if not 0 < self.duration_s <= env.horizon_s:
            raise ValueError("Reference duration must lie inside the task horizon")
        plant, geometry = env.plant, env.geometry
        gravity = -float(plant.model.opt.gravity[2])
        normal = geometry["normal"]
        if normal[2] <= 0:
            raise ValueError("Declared terminal thrust alignment requires positive vertical component")
        zero = np.zeros(3)
        start = [plant.state.position, zero, zero, zero, zero]
        end = [geometry["position_target"], geometry["velocity_target"],
               gravity / normal[2] * normal - np.array([0., 0., gravity]), zero, zero]
        T = self.duration_s
        system = np.zeros((10, 10))
        for derivative in range(5):
            for power in range(derivative, 10):
                factor = math.factorial(power) / math.factorial(power - derivative)
                if power == derivative:
                    system[derivative, power] = factor
                system[5 + derivative, power] = factor * T ** (power - derivative)
        coefficients = np.linalg.solve(system, np.array(start + end))
        self.time_s = np.arange(0., T + 1e-12, grid_dt_s)
        acceleration = sum(np.outer(p * (p - 1) * self.time_s ** (p - 2), coefficients[p]) for p in range(2, 10))
        thrust_vector = acceleration + np.array([0., 0., gravity])
        force = plant.info["mass_kg"] * np.linalg.norm(thrust_vector, axis=1)
        rotations = np.array([frame(vector) for vector in thrust_vector])
        skew = np.einsum("tji,tjk->tik", rotations, np.gradient(rotations, grid_dt_s, axis=0))
        omega = np.stack([skew[:, 2, 1], skew[:, 0, 2], skew[:, 1, 0]], axis=1)
        omega_dot = np.gradient(omega, grid_dt_s, axis=0)
        lag = float(plant.profile["values"]["motor_time_constant_s"])
        self.collective_N = force + lag * np.gradient(force, grid_dt_s)
        self.rates_rad_s = omega + (omega_dot + lag * np.gradient(omega_dot, grid_dt_s, axis=0)) / plant.rate_gain
        self.hold = np.array([force[-1], 0., 0., 0.])

    def action(self, env):
        """Physical CTBR for the current policy interval, sampled at its midpoint."""
        midpoint = env.plant.state.timestamp - env.episode_start + env.policy_dt_s / 2
        if midpoint > self.duration_s:
            return self.hold.copy()
        return np.r_[np.interp(midpoint, self.time_s, self.collective_N),
                     [np.interp(midpoint, self.time_s, self.rates_rad_s[:, axis]) for axis in range(3)]]
