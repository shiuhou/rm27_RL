"""Use the existing RM27 scene, calibrated overrides and CTBR rate proxy."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import mujoco
import numpy as np

from rm27_contact_physics.calibration import load_profile, verify_compiled, with_profile
from rm27_contact_physics.control import motor_mixer
from rm27_contact_rl.paths import ROOT
from rm27_contact_physics.model import build_model, load_config, scenario, truth_estimate
from rm27_contact_physics.teacher_interface import RateThrust, apply_proxy

DEFAULT_CONFIG = ROOT / "configs/contact_rl/minimal.json"


def load_rl_config(path=DEFAULT_CONFIG):
    """Resolve references against installed root assets, never the caller's cwd."""
    path = Path(path).resolve()
    cfg = json.loads(path.read_text())
    if cfg.get("schema_version") != 1:
        raise ValueError("Contact RL config requires schema_version 1")
    sources = {name: (ROOT / cfg[name]).resolve()
               for name in ("experiment_config", "vehicle_config", "dynamics_profile")}
    sources["rl_config"] = path
    base = load_config(sources["experiment_config"])
    sources["target_geometry"] = (ROOT / base["target_config"]).resolve()
    period = float(cfg["policy_dt_s"])
    rate_limit = float(cfg["action"]["body_rate_limit_rad_s"])
    if (not np.isfinite([period, rate_limit]).all() or period <= 0 or rate_limit <= 0
            or not np.isclose(period, base["control_dt_s"], rtol=0, atol=1e-12)):
        raise ValueError("I1 policy period must equal the configured positive control period")
    rewards = cfg["reward"]
    if set(rewards) != {"valid_contact", "separation", "success", "failure"}:
        raise ValueError("Expected the four declared event reward weights")
    if not np.isfinite(list(rewards.values())).all():
        raise ValueError("Reward weights must be finite")
    return cfg, base, sources


class RM27Plant:
    """One environment owns one model and its complete mutable physics state."""

    def __init__(self, cfg, sources):
        self.cfg = copy.deepcopy(cfg)
        self.marker_config = json.loads(sources["target_geometry"].read_text())
        self.physical = scenario(self.cfg, seed=None)
        self.profile = load_profile(sources["dynamics_profile"])
        vehicle = with_profile(json.loads(sources["vehicle_config"].read_text()),
                               sources["dynamics_profile"])
        self.vehicle_config = copy.deepcopy(vehicle)
        self.model, self.data, self.info, self.xml = build_model(
            self.physical, self.cfg, vehicle_config=vehicle)
        motors = [self.model.actuator(f"uav_motor_{i}").id for i in range(4)]
        verify_compiled(self.model, self.model.body("uav").id, motors, self.profile)
        self.initial_qpos = self.data.qpos.copy()
        self.initial_qvel = self.data.qvel.copy()
        self.inertia = np.asarray(self.info["inertia_kg_m2"], dtype=float)
        self.mixer = motor_mixer(self.info["rotor_arm_m"], self.info["yaw_torque_per_thrust_m"])
        self.inverse_mixer = np.linalg.inv(self.mixer)
        controller = self.cfg["controller"]
        self.rate_gain = 2 * controller["attitude_damping"] * controller["attitude_frequency_rad_s"]
        self.maximum = float(self.info["max_rotor_thrust_N_assumed"])
        self.hover_force = float(self.info["mass_kg"] * -self.model.opt.gravity[2])

    @property
    def state(self):
        return truth_estimate(self.model, self.data)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = self.initial_qvel
        self.data.act[:] = self.hover_force / 4
        self.data.ctrl[:] = self.hover_force / 4
        mujoco.mj_forward(self.model, self.data)
        return self.state

    def apply_ctbr(self, clipped_ctbr):
        """Return actual motor *commands*, not actual lagged rotor thrust."""
        action = RateThrust(np.asarray(clipped_ctbr[1:], float), float(clipped_ctbr[0]))
        motors, saturated = apply_proxy(
            action, self.state.omega_body, self.inertia, self.inverse_mixer,
            self.rate_gain, self.maximum, return_saturation=True)
        self.data.ctrl[:] = motors
        return motors.copy(), saturated
