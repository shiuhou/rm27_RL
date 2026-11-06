"""Use the existing RM27 scene, calibrated overrides and CTBR rate proxy."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from rm27_contact_rl.paths import ROOT
from rm27_contact_physics.model import load_config
from rm27_contact_physics.plant import RM27Plant  # Existing adapter import remains supported.

DEFAULT_CONFIG = ROOT / "configs/contact_rl/minimal.json"


def load_rl_config(path=DEFAULT_CONFIG, *, task_id="contact_recovery_v1"):
    """Resolve references against installed root assets, never the caller's cwd."""
    path = Path(path).resolve()
    cfg = json.loads(path.read_text())
    if cfg.get("schema_version") != 1:
        raise ValueError("Contact RL config requires schema_version 1")
    if cfg.get("task_id", "contact_recovery_v1") != task_id:
        raise ValueError("Environment configuration belongs to a different task")
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
    if task_id == "contact_recovery_v1" and set(rewards) != {"valid_contact", "separation", "success", "failure"}:
        raise ValueError("Expected the four declared event reward weights")
    if not np.isfinite(list(rewards.values())).all():
        raise ValueError("Reward weights must be finite")
    return cfg, base, sources
