"""Privileged existing Skill projected through the environment's CTBR chain."""
from __future__ import annotations

import numpy as np

from rm27_contact_physics.control import Septic, Skill
from rm27_contact_physics.model import TargetState
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
