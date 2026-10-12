"""Privileged existing Skill projected through the environment's CTBR chain."""
from __future__ import annotations

import numpy as np

from rm27.experiments.aggressive_contact.control import Skill
from rm27.experiments.aggressive_contact.model import TargetState
from rm27.experiments.aggressive_contact.teacher_interface import project


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
                           plant.state, target, plant.info)
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
