"""Authorization and immutable inputs for the first SF Stage 1 PPO experiment."""

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/rm27_stage1_sim_experiment.json"
RUN = ROOT / ".agents/runs/rm27-stage1/ppo-contact"
EXPERIMENT = "sf_rm27_stage1_sim_ppo_01"
STAGE1B_CONFIG = ROOT / "configs/rm27_stage1b_sim_experiment.json"
STAGE1B_EXPERIMENT = "sf_rm27_stage1b_residual_01"
STAGE1C_CONFIG = ROOT / "configs/rm27_stage1c_sim_experiment.json"
STAGE1C_EXPERIMENT = "sf_rm27_stage1c_handover_01"
STAGE1C_RUN = RUN.parent / "ppo-handover"
_STAGE1A_SHA = "bb16443fabbda5d1358afe566968505493998a6f7f57d3fdccdd8059b9630670"
_NEAR_CHECKPOINT_SHA = "728183aea46b80e5fb536dfaf5e15be783fbe764cfd643eb2a0f24408f84b433"


def digest(path: Path) -> str:
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            sha.update(block)
    return sha.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def load_experiment(path: Path = CONFIG):
    config = read_json(path)
    if (config["schema_version"] != 1 or config["experiment"] != EXPERIMENT
            or config["scope"] != "simulation_only_state_based_direct_ctbr_ppo"
            or config["authorized"] is not True
            or config["parameter_status"] != "PROVISIONAL_AUTHORIZED_FOR_THIS_SIMULATION_EXPERIMENT"
            or config["budget"] != {"candidates": 2, "transitions": 100_000_000, "learning_s": 1800}):
        raise ValueError("Stage 1 requires the explicitly authorized SF simulation experiment")
    assets = {name: ROOT / item["path"] for name, item in config["assets"].items()}
    for name, asset in assets.items():
        if digest(asset) != config["assets"][name]["sha256"]:
            raise ValueError(f"Stage 1 hash mismatch for {name}: {asset}")
    frozen = read_json(assets["task"])
    validation = read_json(assets["native_config"])
    if any(source["training_allowed"] is not False or source["parameter_status"] != "PROPOSED_UNCONFIRMED"
           for source in (frozen, validation)):
        raise ValueError("The protected native task and path-only validation config must remain validation-only")
    for key in ("contact_acceptance", "safety", "initial_state", "reward", "action", "horizon_s", "physics_dt_s",
                "policy_dt_s", "rate_gain", "expected_contact_surface_registration_sha256"):
        if frozen[key] != validation[key]:
            raise ValueError(f"Native validation contract changed: {key}")
    geometry = read_json(assets["geometry"])
    preflight = read_json(assets["isaac_preflight"])
    if (geometry["model_sha256"] != config["assets"]["native_scene"]["sha256"]
            or geometry["registry_sha256"] != config["assets"]["registration"]["sha256"]
            or frozen["expected_contact_surface_registration_sha256"] != geometry["registry_sha256"]
            or preflight["scene_sha256"] != config["assets"]["isaac_scene"]["sha256"]
            or not preflight["pass"] or preflight["device"] != "cuda:0"
            or len(preflight["reset_overlaps"]) != 19 or any(preflight["reset_overlaps"].values())
            or len(preflight["fixtures"]) != 5
            or not all(row["comparison"]["classification_agrees"] for row in preflight["fixtures"].values())):
        raise ValueError("Stage 1 requires the verified archive and GPU collision evidence")
    return config, assets, frozen, geometry


def load_stage1b_experiment(path: Path = STAGE1B_CONFIG):
    config = read_json(path)
    if (config["schema_version"] != 1 or config["experiment"] != STAGE1B_EXPERIMENT
            or config["scope"] != "simulation_only_state_based_direct_ctbr_residual_ppo"
            or config["authorized"] is not True
            or config["parameter_status"] != "PROVISIONAL_AUTHORIZED_FOR_THIS_SIMULATION_EXPERIMENT"
            or config["budget"] != {"candidates": 1, "transitions": 3_000_000, "learning_s": 900}
            or config["raw_bounds"] != [.35, .85, .35, .40]):
        raise ValueError("Stage 1-B requires its explicitly authorized bounded residual experiment")
    source = config["source_experiment"]
    source_path = ROOT / source["path"]
    if source_path != CONFIG or source["sha256"] != _STAGE1A_SHA or digest(source_path) != _STAGE1A_SHA:
        raise ValueError("Stage 1-B source experiment changed")
    source_config, assets, frozen, geometry = load_experiment(source_path)
    near = config["near_checkpoint"]
    near_path = ROOT / near["path"]
    if near["sha256"] != _NEAR_CHECKPOINT_SHA or digest(near_path) != _NEAR_CHECKPOINT_SHA:
        raise ValueError("Stage 1-B near checkpoint changed")
    if config.get("goal_inset_m", source_config["goal_inset_m"]) != source_config["goal_inset_m"]:
        raise ValueError("Stage 1-B contact goal differs from the validated Track lift")
    config["goal_inset_m"] = source_config["goal_inset_m"]
    config["assets"] = source_config["assets"] | {"near_checkpoint": near}
    assets["near_checkpoint"] = near_path
    return config, assets, frozen, geometry


def load_stage1c_experiment(path: Path = STAGE1C_CONFIG):
    config = read_json(path)
    if (config["schema_version"] != 1 or config["experiment"] != STAGE1C_EXPERIMENT
            or config["scope"] != "simulation_only_state_based_direct_ctbr_informed_ppo"
            or config["authorized"] is not True
            or config["budget"] != {"candidates": 2, "transitions": 12_000_000, "learning_s": 3600,
                                    "integration_s": 900, "evaluation_s": 900}):
        raise ValueError("Stage 1-C requires its explicitly authorized Pilot config")
    source = config["source_experiment"]
    if ROOT / source["path"] != CONFIG or source["sha256"] != _STAGE1A_SHA or digest(CONFIG) != _STAGE1A_SHA:
        raise ValueError("Stage 1-C immutable source experiment changed")
    parent, assets, frozen, geometry = load_experiment(CONFIG)
    for name, record in config["inputs"].items():
        asset = ROOT / record["path"]
        if name in assets or (name != "unseen_suite" and digest(asset) != record["sha256"]):
            raise ValueError(f"Stage 1-C hash or ownership mismatch for {name}: {asset}")
        assets[name] = asset
    handover = read_json(assets["handover_contract"])
    if (handover["training_authorized"] is not False
            or config["nominal_standoff_m"] != handover["training_distribution_proposal"]["nominal_standoff_m"]
            or config["goal_inset_m"] != parent["goal_inset_m"]):
        raise ValueError("Stage 1-C must keep the frozen handover region and contact goal")
    config["assets"] = parent["assets"] | config["inputs"]
    return config, assets, frozen, geometry
