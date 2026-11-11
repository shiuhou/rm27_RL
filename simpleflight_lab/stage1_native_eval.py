"""Replay an RM27 Stage 1 policy in the frozen full-field MuJoCo evaluator."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from hashlib import sha256
import io
import json
import math
import multiprocessing
from pathlib import Path
from tempfile import TemporaryDirectory
import time

import numpy as np

from .stage1_experiment import (
    CONFIG as _EXPERIMENT, ROOT as _ROOT, STAGE1B_EXPERIMENT, digest as file_sha,
    load_stage1b_experiment, read_json as load_json,
)

_RUN = _ROOT / ".agents/runs/rm27-stage1"
_RECOVERED = _RUN / "recovered"
_FROZEN = _ROOT / "upstream/rm27_RL/configs/stage1/task_contact_geometry.json"
_SOURCE_CHECKPOINT_SHA = "ce098cb4e374079283509f52ef5af20268db260712a31b7eaad3bb30ea8ea7db"
_SIM_REVISION = "6ec552b4a836c601295b7657c78c4bd3af977600"
_HIDDEN_LAYERS = ((3, 256, None), (4, 256, 256), (6, 256, None),
                  (7, 256, 256), (8, 256, None), (9, 256, 4))
_PORTABLE_SCHEMA = "sf_rm27_stage1_actor_v1"
_RESIDUAL_SCHEMA = "sf_rm27_stage1b_actor_v1"
_PILOT_TASK = "rm27_stage1c"
_PILOT_EXPERIMENT = "sf_rm27_stage1c_handover_01"
_PILOT_SUITES = {"development": 8, "fixed": 20, "unseen": 20, "handover57": 57}
_DEVELOPMENT_CLOSE_M = 0.10
_NEAR_CLEARANCE_M = 0.0165
_NEAR_INWARD_SPEED_M_S = 0.22
_TRIM_ACTION = np.zeros(4, dtype=np.float32)
_SAFETY_FAILURES = {"NUMERICAL_FAILURE", "FLIGHT_VOLUME_EXIT", "EXCESSIVE_SPEED", "EXCESSIVE_BODY_RATE",
                    "MUJOCO_WARNING", "INITIAL_OVERLAP", "NONFINITE_POLICY_ACTION"}
_PATH_KEYS = {"scene_directory", "scene_cache_directory", "target_mapping", "target_registration_source",
              "vehicle_config", "dynamics_profile", "contact_fixture_config", "contact_surface_registration"}


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    return value


def publish_summary(path, summary):
    """Publish a complete JSON file atomically and preserve existing results."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=path.parent) as scratch:
        temporary = Path(scratch) / path.name
        temporary.write_text(json.dumps(finite_json(summary), indent=2, allow_nan=False) + "\n")
        path.hardlink_to(temporary)


class NativeActor:
    """Independent float32 implementation of the verified Torch mean actor."""

    def __init__(self, weights, residual_bounds=None):
        self.track = self.layers(weights, 45)
        self.bounds = residual_bounds
        if self.bounds is None:
            self.normal = self.array(weights, "normal_weights.weight", (256, 3))
        else:
            self.residual = self.layers(weights, 21, "residual.")

    @classmethod
    def layers(cls, weights, input_size, prefix=""):
        layers = []
        for i, inputs, outputs in ((0, input_size, None), (1, input_size, 256), *_HIDDEN_LAYERS):
            weight_shape = (inputs,) if outputs is None else (outputs, inputs)
            w = cls.array(weights, f"{prefix}mean.{i}.weight", weight_shape)
            b = cls.array(weights, f"{prefix}mean.{i}.bias", weight_shape[:1])
            layers.append((i, w, b))
        return layers

    @staticmethod
    def array(weights, key, shape):
        value = np.asarray(weights[key])
        if value.shape != shape or value.dtype != np.float32 or not np.isfinite(value).all():
            raise ValueError(f"Invalid Stage 1 actor tensor {key}")
        return value

    @staticmethod
    def mean(x, layers, normal_projection=None):
        for i, w, b in layers:
            if w.ndim == 1:
                x = x - x.mean(axis=-1, keepdims=True)
                x = x * np.reciprocal(np.sqrt((x * x).mean(axis=-1, keepdims=True) + np.float32(1e-5)))
                x = x * w + b
            else:
                x = x @ w.T + b
            if i == 1 and normal_projection is not None:
                x += normal_projection
            if i in (1, 4):
                negative = x < 0
                x[negative] = np.expm1(x[negative])
        return x

    def __call__(self, features):
        x = np.asarray(features, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != 21:
            raise ValueError("Stage 1 actor expects batched 21-dimensional state")
        lifted = np.concatenate((np.tile(x[:, :3], (1, 10)), x[:, 3:18]), axis=1)
        if self.bounds is None:
            return self.mean(lifted, self.track, x[:, -3:] @ self.normal.T)
        return self.mean(lifted, self.track) + self.bounds * np.tanh(self.mean(x, self.residual))


def parity(actor, features, expected) -> float:
    if features.shape != (64, 21) or expected.shape != (64, 4) or not np.isfinite(expected).all():
        raise ValueError("Invalid independent actor parity fixtures")
    actual = actor(features)
    np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=2e-5)
    return float(np.max(np.abs(actual - expected)))


def load_residual_policy(saved, experiment_path):
    """Load the residual and reconstruct its immutable Track prior on CPU."""
    import torch
    from .ppo import Actor, UPSTREAM_REVISION
    from .stage1_residual import Stage1ResidualPolicy

    config, assets, _, _ = load_stage1b_experiment(experiment_path)
    expected = dict(format=1, task="rm27_stage1b", experiment=STAGE1B_EXPERIMENT,
                    upstream_revision=UPSTREAM_REVISION, rm27_sim_revision=_SIM_REVISION,
                    policy_dt=.005, actor_obs=21, config_sha256=file_sha(experiment_path),
                    source_checkpoint_sha256=_SOURCE_CHECKPOINT_SHA, raw_bounds=config["raw_bounds"],
                    goal_inset_m=config["goal_inset_m"])
    frontier = saved.get("frontier_index")
    if any(saved.get(k) != v for k, v in expected.items()) or type(frontier) is not int or not 0 <= frontier <= 6:
        raise ValueError("Stage 1-B checkpoint provenance or residual schema mismatch")
    track_bytes = assets["checkpoint"].read_bytes()
    if sha256(track_bytes).hexdigest() != _SOURCE_CHECKPOINT_SHA:
        raise ValueError("Stage 1-B requires the immutable Track prior")
    track = Actor(45)
    track.load_state_dict(torch.load(io.BytesIO(track_bytes), map_location="cpu", weights_only=True)["actor"], strict=True)
    policy = Stage1ResidualPolicy(track)
    np.testing.assert_array_equal(policy.raw_bound.numpy(), np.asarray(config["raw_bounds"], dtype=np.float32))
    policy.residual.load_state_dict(saved["actor"], strict=True)
    return policy.eval(), track


def load_pilot_config(path):
    from .stage1_experiment import load_stage1c_experiment

    config, _, _, _ = load_stage1c_experiment(path)
    return config


def load_stage1c_policy(saved, experiment_path, bridge_config=None):
    """Reload the standalone Stage 1-C actor, without dispatching to Track."""
    from .ppo import UPSTREAM_REVISION
    from .stage1_experiment import load_stage1c_experiment
    from .stage1_task import Stage1Actor

    config, _, _, geometry = load_stage1c_experiment(experiment_path)
    expected = dict(format=1, task=_PILOT_TASK, experiment=_PILOT_EXPERIMENT, actor_obs=21,
                    upstream_revision=UPSTREAM_REVISION, rm27_sim_revision=_SIM_REVISION, policy_dt=.005,
                    config_sha256=file_sha(experiment_path), source_checkpoint_sha256=_SOURCE_CHECKPOINT_SHA,
                    goal_inset_m=config["goal_inset_m"],
                    bridge_config_sha256=file_sha(bridge_config) if bridge_config is not None else None)
    if bridge_config is not None:
        expected["bridge_experiment"] = load_json(bridge_config)["experiment"]
    if any(saved.get(key) != value for key, value in expected.items()):
        raise ValueError("Stage 1-C checkpoint provenance or observation schema mismatch")
    policy = Stage1Actor()
    policy.load_state_dict(saved["actor"], strict=True)
    return policy.eval(), geometry


def export_actor(args) -> None:
    import torch
    from .ppo import Actor, UPSTREAM_REVISION

    if args.output.suffix != ".npz" or args.output.exists():
        raise ValueError("Provide a new .npz output for the portable actor")
    torch.set_num_threads(1)
    checkpoint_bytes = args.checkpoint.read_bytes()
    checkpoint_sha = sha256(checkpoint_bytes).hexdigest()
    saved = torch.load(io.BytesIO(checkpoint_bytes), map_location="cpu", weights_only=True)
    config_sha = file_sha(args.experiment)
    experiment = load_json(args.experiment)
    baseline = saved["task"] == "track"
    residual = saved["task"] == "rm27_stage1b"
    pilot = experiment["experiment"] == _PILOT_EXPERIMENT
    if args.bridge_config is not None and (baseline or residual or not pilot):
        raise ValueError("A bridge config requires the standalone Stage 1-C actor")
    if pilot and baseline:
        experiment = load_pilot_config(args.experiment)
    if (saved["format"] != 1 or saved["upstream_revision"] != UPSTREAM_REVISION
            or saved["rm27_sim_revision"] != _SIM_REVISION or saved["policy_dt"] != .005):
        raise ValueError("Checkpoint does not match the RM27 clock and simulator contract")
    if residual:
        policy, track = load_residual_policy(saved, args.experiment)
        weights = {k: v.detach().numpy() for k, v in track.state_dict().items()}
        weights.update({f"residual.{k}": v.detach().numpy() for k, v in policy.residual.state_dict().items()})
    elif baseline:
        if checkpoint_sha != _SOURCE_CHECKPOINT_SHA or saved["actor_obs"] != 45:
            raise ValueError("Only the selected immutable Track checkpoint is a Stage 1 baseline")
        policy = Actor(45)
    elif pilot:
        policy, _ = load_stage1c_policy(saved, args.experiment, args.bridge_config)
    else:
        from .stage1_task import Stage1Actor

        if (saved["task"] != "rm27_stage1" or saved["actor_obs"] != 21
                or saved["config_sha256"] != config_sha
                or saved["source_checkpoint_sha256"] != _SOURCE_CHECKPOINT_SHA):
            raise ValueError("Stage 1 checkpoint provenance or observation schema mismatch")
        policy = Stage1Actor()
    if not residual:
        if baseline or not pilot:
            policy.load_state_dict(saved["actor"], strict=True)
            policy.eval()
        weights = {k: v.detach().numpy() for k, v in policy.state_dict().items()}
    if baseline:
        weights["normal_weights.weight"] = np.zeros((256, 3), np.float32)

    # Include a broad set of displacement/attitude/velocity values and the
    # nominal state. Both runtimes verify the same un-squashed mean outputs.
    rng = np.random.default_rng(927)
    observations = rng.normal(0, .35, size=(64, 21)).astype(np.float32)
    observations[0, :] = 0
    observations[0, 6:15] = np.eye(3, dtype=np.float32).ravel()
    observations[0, 18:] = [-.4617148, 0, .88702846]
    with torch.inference_mode():
        inputs = torch.from_numpy(observations)
        if baseline:
            inputs = torch.cat((inputs[:, :3].repeat(1, 10), inputs[:, 3:18]), dim=-1)
        expected = (policy.contact_mean(inputs) if residual else policy(inputs)).numpy()
    bounds = np.asarray(saved["raw_bounds"], dtype=np.float32) if residual else None
    maximum_error = parity(NativeActor(weights, bounds), observations, expected)
    metadata = dict(schema=_RESIDUAL_SCHEMA if residual else _PORTABLE_SCHEMA, task=saved["task"], baseline=baseline,
                    checkpoint_sha256=checkpoint_sha, config_sha256=config_sha,
                    source_checkpoint_sha256=_SOURCE_CHECKPOINT_SHA, update=saved["update"],
                    frames=saved["frames"], candidate=saved.get("candidate"),
                    goal_inset_m=experiment["goal_inset_m"] if baseline else saved["goal_inset_m"],
                    maximum_parity_error=maximum_error)
    if residual:
        metadata.update(raw_bounds=saved["raw_bounds"], frontier_index=saved["frontier_index"])
    if pilot:
        metadata["experiment"] = _PILOT_EXPERIMENT
        if not baseline:
            metadata["frontier_index"] = saved.get("frontier_index")
    if args.bridge_config is not None:
        metadata.update(bridge_config_sha256=saved["bridge_config_sha256"],
                        bridge_experiment=saved["bridge_experiment"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as output:
        np.savez_compressed(output, **weights, metadata=json.dumps(metadata, allow_nan=False),
                            parity_input=observations, parity_mean=expected)
    print(f"STAGE1_ACTOR_EXPORTED {args.output}, max raw-action difference {maximum_error:.3g}")


def native_recorded_root(config):
    """Find the workspace recorded by the frozen, path-only validation config."""
    relative_cache = (_RECOVERED / "relocated-scene").relative_to(_ROOT)
    recorded_cache = Path(config["scene_cache_directory"])
    recorded_root = recorded_cache.parents[len(relative_cache.parts) - 1]
    if not recorded_cache.is_absolute() or recorded_root / relative_cache != recorded_cache:
        raise ValueError("Frozen native scene cache has an unexpected workspace layout")
    return recorded_root


def relocate_native_paths(config):
    """Resolve frozen workspace paths after a rename without rewriting the archive."""
    recorded_root = native_recorded_root(config)
    return {
        key: str(_ROOT / Path(value).relative_to(recorded_root)) if key in _PATH_KEYS else value
        for key, value in config.items()
    }


def relocate_native_manifest(original, relocated):
    """Keep original manifests intact; materialize path-only runtime metadata."""
    cache = Path(relocated["scene_cache_directory"])
    manifest = load_json(cache / "stage1_scene.json")
    recorded_root = native_recorded_root(original)
    if recorded_root == _ROOT:
        return
    for record in manifest["inputs"].values():
        if "path" in record:
            record["path"] = str(_ROOT / Path(record["path"]).relative_to(recorded_root))
    runtime = _RUN / "workspace-paths"
    runtime.mkdir(exist_ok=True)
    for name, hash_key in (("scene.xml", "scene_sha256"), ("scene.mjb", "compiled_model_sha256")):
        source = cache / name
        if file_sha(source) != manifest[hash_key]:
            raise ValueError(f"Frozen native scene hash mismatch during relocation: {name}")
        link = runtime / name
        try:
            link.symlink_to(Path("../recovered/relocated-scene") / name)
        except FileExistsError:
            if not link.is_symlink() or link.resolve() != source.resolve():
                raise ValueError(f"Unexpected native runtime scene link: {link}")
    destination = runtime / "stage1_scene.json"
    contents = json.dumps(manifest, indent=2) + "\n"
    if not destination.exists() or destination.read_text() != contents:
        with TemporaryDirectory(dir=runtime) as scratch:
            temporary = Path(scratch) / destination.name
            temporary.write_text(contents)
            temporary.replace(destination)
    relocated["scene_cache_directory"] = str(runtime)


def open_environment(standoff: float):
    from rm27_contact_rl import stage1_plant
    from rm27_contact_rl.stage1_env import Stage1ContactEnv, load_stage1_config

    config_path = _RECOVERED / "validation-config.json"
    config = load_stage1_config(config_path)
    frozen = load_stage1_config(_FROZEN)
    if {k: v for k, v in config.items() if k not in _PATH_KEYS} != {
        k: v for k, v in frozen.items() if k not in _PATH_KEYS
    }:
        raise ValueError("Independent scoring requires the path-only relocated frozen config")
    original = config
    config = relocate_native_paths(original)
    relocate_native_manifest(original, config)
    stage1_plant.ROOT = _RECOVERED / "rm27_RL"
    plant = stage1_plant.Stage1Plant(config)
    env = Stage1ContactEnv(config_path, plant=plant)
    env.rl_config.update({key: config[key] for key in _PATH_KEYS if key in config})
    # Only a separately labeled short-distance diagnostic may alter the reset
    # standoff. The frozen contact and safety gates remain in use.
    env.rl_config["initial_state"]["normal_standoff_m"] = standoff
    return env


def actor_features(state, goal, normal, offset):
    bumper = state.position + state.rotation @ offset
    return np.concatenate((goal - bumper, state.velocity, state.rotation.T.reshape(-1),
                           state.omega_body, normal)).astype(np.float32)[None, :]


def near_pose(geometry, initial, inward_speed_m_s):
    """Small deterministic perturbations in the separate near-aligned diagnostic."""
    from scipy.spatial.transform import Rotation

    center, normal, frame, radius, offset = (
        geometry[k] for k in ("center", "normal", "target_rotation", "radius", "offset_body")
    )
    delta = np.asarray(initial) * .25
    delta[:3] = delta[:3].clip(-np.array([.005, .005, .002]), [.005, .005, .002])
    rotation = frame @ Rotation.from_rotvec(delta[6:9]).as_matrix()
    position = center + (radius + _NEAR_CLEARANCE_M) * normal - rotation @ offset + frame @ delta[:3]
    return dict(position=position, rotation=rotation,
                velocity=delta[3:6] - inward_speed_m_s * normal, omega_body=delta[9:12]), delta


def frontier_pose(geometry, initial, clearance, position_halfwidth):
    """Deterministic zero-nominal-velocity samples of the current training band."""
    if clearance == _NEAR_CLEARANCE_M:
        return near_pose(geometry, initial, 0.)
    from scipy.spatial.transform import Rotation

    center, normal, frame, radius, offset = (
        geometry[k] for k in ("center", "normal", "target_rotation", "radius", "offset_body")
    )
    tilt = np.clip((1. - clearance) / .90, 0., 1.)
    delta = np.asarray(initial) * (1. - .75 * tilt)
    halfwidth = tilt * np.array([.005, .005, .002]) + (1. - tilt) * position_halfwidth
    delta[:3] = delta[:3].clip(-halfwidth, halfwidth)
    aligned = Rotation.from_matrix(frame).as_rotvec()
    rotation = (Rotation.from_rotvec(tilt * aligned) * Rotation.from_rotvec(delta[6:9])).as_matrix()
    position = center + (radius + clearance) * normal - rotation @ offset + frame @ delta[:3]
    return dict(position=position, rotation=rotation, velocity=delta[3:6], omega_body=delta[9:12]), delta


def episode(case):
    name, initial, standoff = case
    env, actor, inset, hold, near_speed, track_com_reference, frontier_gap = _EVALUATION
    plant = env.plant
    geom = env.geometry
    env.rl_config["initial_state"]["normal_standoff_m"] = standoff
    pose, applied = None, initial
    if frontier_gap is not None:
        pose, applied = frontier_pose(geom, initial, frontier_gap, env.initial_ranges[:3, 1])
    elif near_speed is not None:
        pose, applied = near_pose(geom, initial, near_speed)
    if pose is not None:
        reset_plant = plant.reset
        # Only the diagnostic reset pose differs. Use the frozen environment
        # to initialize its scorer, overlap check, motors and episode state.
        plant.reset = lambda **kwargs: reset_plant(**pose)
        try:
            env.reset(options={"initial_perturbation": initial})
        finally:
            del plant.reset
    else:
        env.reset(options={"initial_perturbation": initial})
    evaluator = env.evaluator
    normal, center, offset, radius = (geom[k] for k in ("normal", "center", "offset_body", "radius"))
    nominal = center + standoff * normal
    goal = nominal + offset if hold else center + (radius - inset) * normal
    actor_offset = offset
    if track_com_reference:
        goal -= offset[2] * normal
        actor_offset = np.zeros_like(offset)
    max_ticks = round(env.horizon_s / env.physics_dt_s)
    max_controls = math.ceil(max_ticks / env.physics_substeps)
    state = np.full((max_ticks + 1, 13), np.nan)
    motor_state = np.full((max_ticks + 1, 4), np.nan)
    plane_gap = np.full(max_ticks + 1, np.nan)
    bumper = np.full((max_ticks + 1, 3), np.nan)
    commands = np.full((max_controls, 4), np.nan)
    raw_actions = np.full((max_controls, 4), np.nan, dtype=np.float32)
    motor_commands = np.full((max_controls, 4), np.nan)
    saturated = np.zeros(max_controls, dtype=bool)

    def record(tick, physical_state):
        state[tick] = np.r_[plant.data.qpos[plant.qpos_slice], plant.data.qvel[plant.qvel_slice]]
        motor_state[tick] = plant.data.act[plant.actuator_state_ids]
        bumper[tick] = physical_state.position + physical_state.rotation @ offset
        plane_gap[tick] = (bumper[tick] - center) @ normal - radius

    record(0, plant.state)
    advance = evaluator.advance

    def tracked_advance(before, saturated=False):
        tick = evaluator.completed_intervals
        after = advance(before, saturated=saturated)
        if evaluator.completed_intervals > tick:
            record(evaluator.completed_intervals, after)
        return after

    # Attach telemetry to the actual frozen scorer's interval path. Its method,
    # stage advancement, contact classification, and reward remain unchanged.
    evaluator.advance = tracked_advance
    controls = 0
    policy_failure = None
    for j in range(max_controls):
        if evaluator.done:
            break
        raw = (_TRIM_ACTION if actor is None else
               actor(actor_features(plant.state, goal, normal, actor_offset))[0])
        if not np.isfinite(raw).all():
            policy_failure = "NONFINITE_POLICY_ACTION"
            break
        command = np.tanh(raw)
        requested = np.r_[plant.hover_force * (1.0 + .5 * command[3]), 2.0 * command[:3]]
        _, _, _, _, info = env.step(requested)
        commands[j] = info["previous_action_ctbr"]
        raw_actions[j] = raw
        motor_commands[j] = plant.data.ctrl[plant.motor_ids]
        saturated[j] = info["proxy_saturated"]
        controls += 1
    evaluator.advance = advance
    result = evaluator.summary()
    if policy_failure is not None:
        result.update(passed=False, failure_modes=[policy_failure])
    tick = evaluator.completed_intervals
    if policy_failure is None and not evaluator.done:
        raise RuntimeError(f"Frozen evaluator did not finish {name} by its declared horizon")
    if abs(float(plant.data.time) - tick * env.physics_dt_s) > 1e-9:
        raise RuntimeError(f"Native physics clock differs from interval count for {name}")
    first = result["first_contact"]
    if first is not None:
        for contact in first["contacts"]:
            contact["normal_impulse_N_s"] = contact["normal_force_N"] * env.physics_dt_s
            contact["wrench_impulse_contact_frame"] = (
                np.asarray(contact["wrench_contact_frame"]) * env.physics_dt_s).tolist()
    distance = np.linalg.norm(bumper[:tick + 1] - radius * normal - center, axis=-1)
    recorded = np.isfinite(distance) & np.isfinite(state[:tick + 1, 3:7]).all(axis=1)
    if recorded.any():
        nearest = int(np.where(recorded, distance, np.inf).argmin())
        q = state[nearest, 3:7]
        from scipy.spatial.transform import Rotation

        body_up = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()[:, 2]
        result["closest_surface_target"] = dict(
            distance_m=float(distance[nearest]), time_s=nearest * env.physics_dt_s,
            normal_gap_m=float(plane_gap[nearest]), attitude_wxyz=q.tolist(),
            bottom_normal_error_deg=float(np.rad2deg(np.arccos(np.clip(body_up @ normal, -1, 1)))),
            position_m=state[nearest, :3].tolist(), velocity_m_s=state[nearest, 7:10].tolist(),
            angular_velocity_rad_s=state[nearest, 10:13].tolist(),
        )
    result["control_ticks"] = controls
    result["frozen_terminal"] = bool(evaluator.done)
    result["saturated_controls"] = int(saturated.sum())
    result["initial_perturbation"] = initial.tolist()
    result["nominal_standoff_m"] = standoff
    if pose is not None:
        result["diagnostic_scaled_perturbation"] = applied.tolist()
    if hold:
        from .rm27_legacy import load_legacy_flight

        g = load_legacy_flight("hover").gate
        tail = state[round((env.horizon_s - g.window_s) / env.physics_dt_s):tick + 1]
        metrics = None
        if tick == max_ticks and result["failure_modes"] == ["CONTACT_TIMEOUT"] and np.isfinite(tail).all():
            tilt = np.rad2deg(2 * np.arcsin(np.clip(np.linalg.norm(tail[:, 4:6], axis=1), 0., 1.)))
            metrics = dict(position_peak_m=float(np.linalg.norm(tail[:, :3] - nominal, axis=1).max()),
                           speed_peak_m_s=float(np.linalg.norm(tail[:, 7:10], axis=1).max()),
                           tilt_peak_deg=float(tilt.max()),
                           body_rate_peak_rad_s=float(np.linalg.norm(tail[:, 10:13], axis=1).max()))
        result["hold_metrics"] = metrics
        result["hold_passed"] = bool(metrics and metrics["position_peak_m"] <= g.position_m
                                     and metrics["speed_peak_m_s"] <= g.speed_m_s
                                     and metrics["tilt_peak_deg"] <= g.tilt_deg
                                     and metrics["body_rate_peak_rad_s"] <= g.body_rate_rad_s)
    return name, result, dict(state=state, motor_state=motor_state, plane_gap=plane_gap, bumper=bumper,
                              commands=commands, raw_actions=raw_actions, motor_commands=motor_commands,
                              saturated=saturated, goal=goal)


def pilot_cases(args, experiment, metadata, historical):
    suite = args.suite
    if suite == "handover57":
        key = "handover_contract"
    else:
        key = "unseen_suite" if suite == "unseen" else "gate_suites"
    selection_sha = None
    if args.selection is not None or suite == "unseen" or (suite == "fixed" and not metadata["baseline"]):
        if args.selection is None:
            raise ValueError("Freeze the development-selected checkpoint before running either learned gate or any unseen case")
        receipt_bytes = args.selection.read_bytes()
        receipt = json.loads(receipt_bytes)
        expected = dict(config_sha256=metadata["config_sha256"],
                        gate_suites_sha256=experiment["assets"]["gate_suites"]["sha256"],
                        unseen_suite_sha256=experiment["assets"]["unseen_suite"]["sha256"])
        selected = receipt.get("checkpoint_sha256", "")
        if (any(receipt.get(k) != v for k, v in expected.items()) or not isinstance(selected, str) or len(selected) != 64
                or any(ch not in "0123456789abcdef" for ch in selected)
                or (not metadata["baseline"] and selected != metadata["checkpoint_sha256"])):
            raise ValueError("Gate checkpoint or suite digests differ from the frozen selection receipt")
        selection_sha = sha256(receipt_bytes).hexdigest()
    asset = experiment["assets"][key]
    data = (_ROOT / asset["path"]).read_bytes()
    if sha256(data).hexdigest() != asset["sha256"]:
        raise ValueError(f"Frozen evaluation asset differs from the Pilot: {key}")
    manifest = json.loads(data)
    if suite == "handover57":
        distances = manifest["evaluation"]["nominal_standoffs_m"]
        if len(historical) != 19 or distances != [1.025, 1.125, 1.225] or manifest["evaluation"]["episodes"] != 57:
            raise ValueError("The frozen handover strata changed")
        cases = [(f"{distance:.3f}m_{name}", initial, distance)
                 for distance in distances for name, initial in historical]
    else:
        spec = manifest["suites"][suite]
        rows = np.asarray(list(spec["starts"].values()), dtype=np.float64)
        bounds = np.array([[1., 1.25], [-.03, .03], [-.03, .03], [-2., 2.]])
        if (manifest["contract_id"] != "sf_rm27_stage1c_stationary_gate_v1" or spec["episodes"] != _PILOT_SUITES[suite]
                or rows.shape != (_PILOT_SUITES[suite], 4) or not np.isfinite(rows).all()
                or np.any(rows < bounds[:, 0]) or np.any(rows > bounds[:, 1])
                or len(np.unique(rows, axis=0)) != len(rows)):
            raise ValueError("Invalid frozen stationary handover suite")
        cases = []
        for name, (distance, slope, across, yaw_deg) in zip(spec["starts"], rows, strict=True):
            initial = np.zeros(12)
            initial[:2] = slope, across
            initial[8] = np.deg2rad(yaw_deg)
            cases.append((name, initial, float(distance)))
    return cases, asset["sha256"], selection_sha


def evaluate(args) -> None:
    from rm27_contact_rl import stage1_evaluation
    from rm27_contact_rl.stage1_runner import evaluation_initials

    start_wall = time.perf_counter()
    if not 0 < args.standoff_m <= 1.0 or (args.hold and args.standoff_m != 1.0):
        raise ValueError("Choose standoff in (0, 1] m; hover regression uses the full 1 m start")
    if args.hold and args.case:
        raise ValueError("Hover regression has its own fixed case set")
    if args.near_aligned and (args.hold or args.standoff_m != 1.0):
        raise ValueError("The near-aligned diagnostic defines its own initial geometry")
    near_speed = args.near_inward_speed_m_s
    if near_speed is not None and (not args.near_aligned or not 0 <= near_speed <= _NEAR_INWARD_SPEED_M_S):
        raise ValueError(f"Near inward speed requires --near-aligned and must be in [0, {_NEAR_INWARD_SPEED_M_S}] m/s")
    if args.near_aligned and near_speed is None:
        near_speed = _NEAR_INWARD_SPEED_M_S
    if args.near_trim_only and not args.near_aligned:
        raise ValueError("The trim-only control requires --near-aligned")
    if args.track_com_reference and (args.hold or args.near_aligned or args.standoff_m != 1.0):
        raise ValueError("The Track COM reference diagnostic requires full 1 m approach starts")
    if args.frontier is not None and (args.case or args.hold or args.near_aligned
                                     or args.track_com_reference or args.standoff_m != 1.0):
        raise ValueError("A frontier evaluation uses all 19 fixed starts and its own reset geometry")
    if args.suite and (args.case or args.hold or args.near_aligned or args.track_com_reference
                       or args.frontier is not None or args.standoff_m != 1.0):
        raise ValueError("A Pilot suite fixes its own resets and retains its entire denominator")
    if args.constant_trim and args.suite not in ("development", "fixed", "unseen"):
        raise ValueError("The matched constant trim control uses a stationary Pilot suite")
    if args.selection and args.suite not in ("fixed", "unseen"):
        raise ValueError("The selection receipt is only used by the fixed and unseen Pilot gates")
    if not 1 <= args.workers <= 4:
        raise ValueError("Native evaluation is bounded to at most four CPU workers")
    summary_path, telemetry_path = args.output.with_suffix(".json"), args.output.with_suffix(".npz")
    if summary_path.exists() or telemetry_path.exists():
        raise FileExistsError(f"Choose a fresh output prefix: {args.output}")
    config_sha = file_sha(args.experiment)
    experiment = load_json(args.experiment)
    pilot = experiment["experiment"] == _PILOT_EXPERIMENT
    if pilot:
        experiment = load_pilot_config(args.experiment)
    if args.suite and not pilot:
        raise ValueError("A Pilot suite requires the Stage 1-C experiment config")
    trim = args.near_trim_only or args.constant_trim
    with np.load(args.actor, allow_pickle=False) as saved:
        metadata = json.loads(saved["metadata"].item())
        residual = metadata["schema"] == _RESIDUAL_SCHEMA
        if (metadata["schema"] not in (_PORTABLE_SCHEMA, _RESIDUAL_SCHEMA) or metadata["config_sha256"] != config_sha
                or metadata["source_checkpoint_sha256"] != _SOURCE_CHECKPOINT_SHA
                or metadata["checkpoint_sha256"] != file_sha(args.checkpoint)
                or metadata.get("bridge_config_sha256") != (file_sha(args.bridge_config) if args.bridge_config else None)):
            raise ValueError("Native actor does not match its source checkpoint and experimental config")
        if args.bridge_config and (not pilot or metadata["baseline"]
                                   or metadata.get("bridge_experiment") != load_json(args.bridge_config)["experiment"]):
            raise ValueError("A bridge config requires its matching standalone actor")
        if ((args.track_com_reference or trim)
                and (not metadata["baseline"] or metadata["checkpoint_sha256"] != _SOURCE_CHECKPOINT_SHA)):
            raise ValueError("The COM reference and trim-only controls require the immutable selected Track actor")
        if pilot and (metadata.get("experiment") != _PILOT_EXPERIMENT or residual
                      or metadata["task"] != ("track" if metadata["baseline"] else _PILOT_TASK)
                      or metadata["goal_inset_m"] != experiment["goal_inset_m"]):
            raise ValueError("Portable actor does not match the standalone Stage 1-C policy contract")
        if residual:
            experiment, _, _, _ = load_stage1b_experiment(args.experiment)
            frontier = metadata["frontier_index"]
            if (metadata["task"] != "rm27_stage1b" or metadata["baseline"]
                    or metadata["raw_bounds"] != experiment["raw_bounds"]
                    or metadata["goal_inset_m"] != experiment["goal_inset_m"]
                    or type(frontier) is not int or not 0 <= frontier <= 6):
                raise ValueError("Portable Stage 1-B actor metadata differs from the approved experiment")
            bounds = np.asarray(experiment["raw_bounds"], dtype=np.float32)
        else:
            bounds = None
        if args.frontier is not None and (not residual or args.frontier != frontier):
            raise ValueError("A frontier run requires a matching Stage 1-B checkpoint")
        actor = NativeActor(saved, bounds)
        max_error = parity(actor, saved["parity_input"], saved["parity_mean"])

    scorer_sha = file_sha(Path(stage1_evaluation.__file__))
    if (scorer_sha != experiment["assets"]["scorer"]["sha256"]
            or file_sha(_FROZEN) != experiment["assets"]["task"]["sha256"]
            or file_sha(_RECOVERED / "validation-config.json") != experiment["assets"]["native_config"]["sha256"]):
        raise ValueError("Frozen MuJoCo scorer, task, or validation provenance changed")
    # Trained actors use their recorded goal offset, which can differ between
    # approved candidates. The selected Track baseline uses the experiment goal.
    inset = float(experiment["goal_inset_m"] if metadata["baseline"] else metadata["goal_inset_m"])
    if not math.isfinite(inset) or inset < 0:
        raise ValueError("Actor goal inset must be a finite nonnegative distance")
    frontier_gap = None
    if args.frontier is not None and args.frontier < 6:
        band = experiment["frontier_clearance_m"][args.frontier]
        frontier_gap = _NEAR_CLEARANCE_M if args.frontier == 0 else math.fsum(band) / 2
    env = open_environment(args.standoff_m)
    scene_sha = file_sha(Path(env.plant.sources["private_scene_binary"]))
    if scene_sha != experiment["assets"]["native_scene"]["sha256"]:
        env.close()
        raise ValueError("Frozen MuJoCo model differs from the pinned scene")
    if args.track_com_reference and not np.allclose(env.geometry["offset_body"][:2], 0., rtol=0., atol=1e-12):
        env.close()
        raise ValueError("The COM reference requires a bumper offset along body Z")
    suite_sha = selection_sha = None
    try:
        historical = evaluation_initials(env)
        if args.suite:
            cases, suite_sha, selection_sha = pilot_cases(args, experiment, metadata, historical)
        else:
            cases = [(name, initial, args.standoff_m) for name, initial in historical]
            if args.hold:
                # Nominal plus one signed pair each for position, velocity and attitude.
                subset = {"nominal", "position_2_-1", "position_2_+1", "velocity_0_-1", "velocity_0_+1",
                          "attitude_1_-1", "attitude_1_+1"}
                cases = [case for case in cases if case[0] in subset]
            elif args.case:
                cases = [case for case in cases if case[0] == args.case]
                if not cases:
                    raise ValueError(f"Unknown frozen Stage 1 case {args.case!r}")
        global _EVALUATION
        _EVALUATION = (env, None if trim else actor, inset, args.hold, near_speed,
                       args.track_com_reference, frontier_gap)
        try:
            if args.workers == 1:
                evaluated = list(map(episode, cases))
            else:
                # Children share the read-only 399 MB model and mutate their own MjData.
                with ProcessPoolExecutor(max_workers=args.workers, mp_context=multiprocessing.get_context("fork")) as pool:
                    evaluated = list(pool.map(episode, cases))
        finally:
            del _EVALUATION
    finally:
        env.close()

    profile = (f"stage1c_{args.suite}{'_trim' if trim else ''}" if args.suite else
               f"stationary_frontier_{args.frontier}" if args.frontier is not None else
               "near_trim_control" if args.near_trim_only else
               "track_com_reference_diagnostic" if args.track_com_reference else
               "hold_regression" if args.hold else "near_aligned_diagnostic" if args.near_aligned else
               "single_case_diagnostic" if args.case else "full_1m" if args.standoff_m == 1.0 else "short_diagnostic")
    outcomes = {name: outcome for name, outcome, _ in evaluated}
    outcome_key = "hold_passed" if args.hold else "passed"
    success = np.array([item[outcome_key] for item in outcomes.values()])
    ticks = np.array([item["completed_physics_intervals"] for item in outcomes.values()])
    contacts = [item for item in outcomes.values() if item["first_contact"] is not None]
    reference_point = env.geometry["center"].copy()
    if args.hold:
        reference_point += args.standoff_m * env.geometry["normal"]
    summary = dict(
        task=_PILOT_TASK if pilot else "rm27_stage1b" if residual else "rm27_stage1", profile=profile,
        standoff_m=None if args.near_aligned or frontier_gap is not None or args.suite else args.standoff_m,
        goal_inset_m=inset,
        episodes=len(evaluated), success_count=int(success.sum()), physical_contact_count=len(contacts),
        qualified_contact_count=sum(x["passed"] for x in contacts),
        unqualified_contact_count=sum(not x["passed"] for x in contacts),
        termination_reasons=dict(Counter(reason for outcome in outcomes.values() for reason in outcome["failure_modes"])),
        checkpoint=str(args.checkpoint), checkpoint_sha256=metadata["checkpoint_sha256"],
        actor=str(args.actor), actor_sha256=file_sha(args.actor),
        actor_metadata=metadata, independent_maximum_parity_error=max_error,
        physics_dt_s=env.physics_dt_s, policy_dt_s=env.policy_dt_s,
        frozen_private_scene_sha256=scene_sha,
        scorer=next(iter(outcomes.values()))["evaluation_protocol"], scorer_sha256=scorer_sha,
        contact_gates=env.rl_config["contact_acceptance"],
        scope="Frozen full-field MuJoCo simulation and unchanged first-contact scorer",
        cases=outcomes,
    )
    if near_speed is not None:
        summary["near_reset"] = dict(nominal_bumper_clearance_m=_NEAR_CLEARANCE_M,
                                    nominal_inward_speed_m_s=near_speed,
                                    attitude="aligned to the frozen target frame",
                                    perturbation_scale=.25, lateral_limit_m=.005, normal_limit_m=.002)
    if args.track_com_reference:
        summary["track_com_reference_m"] = evaluated[0][2]["goal"].tolist()
    if trim:
        summary["trim_ctbr"] = [env.plant.hover_force, 0., 0., 0.]
    if args.frontier is not None:
        summary.update(frontier_index=args.frontier, nominal_bumper_clearance_m=frontier_gap,
                       nominal_inward_speed_m_s=0.)
    if args.suite:
        summary.update(suite=args.suite, suite_sha256=suite_sha, selection_sha256=selection_sha,
                       command_source="constant_trim" if trim else "frozen_track" if metadata["baseline"] else "learned_actor")
    if args.suite == "handover57":
        summary["strata"] = {
            f"{distance:.3f}": dict(episodes=sum(case["nominal_standoff_m"] == distance for case in outcomes.values()),
                                      qualified=sum(case["nominal_standoff_m"] == distance and case["passed"]
                                                    for case in outcomes.values()),
                                      physical_contacts=sum(case["nominal_standoff_m"] == distance
                                                            and case["first_contact"] is not None
                                                            for case in outcomes.values()))
            for distance in sorted({case["nominal_standoff_m"] for case in outcomes.values()})
        }
    traces = {key: np.stack([trace[key] for _, _, trace in evaluated], axis=1)
              for key in evaluated[0][2] if key != "goal"}
    initial_state = traces["state"][0]
    s = traces.pop("state")[1:]
    reference = np.broadcast_to(reference_point, s[..., :3].shape).copy()
    reference[np.arange(len(s))[:, None] >= ticks[None, :]] = np.nan
    distance = np.linalg.norm(s[..., :3] - reference, axis=-1)
    # Keep row 0 at t=dt for the shared replay command, with the reset state separate.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with telemetry_path.open("xb") as output:
        np.savez_compressed(output, **traces, initial_state=initial_state, position=s[..., :3], quat=s[..., 3:7],
                            linear_velocity=s[..., 7:10], angular_velocity=s[..., 10:13], reference=reference,
                            ctbr_command=traces["commands"], dt=env.physics_dt_s, seed="fixed",
                            finished=np.array([item["frozen_terminal"] for item in outcomes.values()]),
                            success=success, first_done_tick=ticks, task=summary["task"], profile=profile,
                            rmse_m=np.sqrt(np.divide(np.nansum(distance**2, axis=0), ticks,
                                                    out=np.full(len(ticks), np.nan), where=ticks > 0)),
                            names=tuple(outcomes), actor_goal=np.stack([trace["goal"] for _, _, trace in evaluated]))
    summary["evaluation_wall_s"] = time.perf_counter() - start_wall
    publish_summary(summary_path, summary)
    from .monitoring import publish_evaluation
    publish_evaluation(summary, args.output, source="mujoco_independent",
                       controller="constant_control" if trim else "track_control" if metadata["baseline"] else "ppo")
    print(f"STAGE1_MUJOCO_COMPLETE {profile} {int(success.sum())}/{len(evaluated)}, {len(contacts)} physical contacts")


def development_score(path, config_sha, assets, expected_names):
    raw = path.read_bytes()
    summary = json.loads(raw)
    metadata = summary["actor_metadata"]
    expected = dict(task=_PILOT_TASK, profile="stage1c_development", suite="development",
                    command_source="learned_actor", episodes=_PILOT_SUITES["development"],
                    suite_sha256=assets["gate_suites"]["sha256"],
                    frozen_private_scene_sha256=assets["native_scene"]["sha256"],
                    scorer_sha256=assets["scorer"]["sha256"])
    checkpoint = _ROOT / summary["checkpoint"]
    actor_path = _ROOT / summary["actor"]
    checkpoint_sha = file_sha(checkpoint)
    if (any(summary.get(k) != v for k, v in expected.items())
            or summary["checkpoint_sha256"] != checkpoint_sha or metadata["checkpoint_sha256"] != checkpoint_sha
            or summary["actor_sha256"] != file_sha(actor_path) or metadata["config_sha256"] != config_sha
            or metadata["source_checkpoint_sha256"] != _SOURCE_CHECKPOINT_SHA
            or metadata.get("baseline") is not False or metadata.get("experiment") != _PILOT_EXPERIMENT
            or metadata.get("candidate") not in (1, 2) or type(metadata.get("update")) is not int
            or type(metadata.get("frames")) is not int or metadata["update"] <= 0 or metadata["frames"] <= 0
            or list(summary["cases"]) != expected_names):
        raise ValueError(f"Development score does not match the frozen Pilot: {path}")
    with np.load(actor_path, allow_pickle=False) as portable:
        if json.loads(portable["metadata"].item()) != metadata:
            raise ValueError(f"Portable actor metadata differs from the development score: {path}")
    cases = list(summary["cases"].values())
    passed = np.array([case["passed"] for case in cases], dtype=bool)
    intervals = [case["completed_physics_intervals"] for case in cases]
    contacts = [case["first_contact"] for case in cases if case["first_contact"] is not None]
    qualified = int(passed.sum())
    if (qualified != summary["success_count"] or qualified != summary["qualified_contact_count"]
            or len(contacts) != summary["physical_contact_count"]
            or qualified != sum(contact["accepted"] for contact in contacts)):
        raise ValueError(f"Development counts disagree with frozen case outcomes: {path}")
    with np.load(path.with_suffix(".npz"), allow_pickle=False) as trace:
        np.testing.assert_array_equal(trace["names"], expected_names)
        np.testing.assert_array_equal(trace["success"], passed)
        np.testing.assert_array_equal(trace["first_done_tick"], intervals)
        if trace["task"].item() != _PILOT_TASK or trace["profile"].item() != expected["profile"]:
            raise ValueError(f"Development trace task or profile differs from its score: {path}")

    def metric(key):
        values = [case.get("closest_surface_target", {}).get(key) for case in cases]
        values = [math.inf if x is None else float(x) for x in values]
        if any(not x >= 0 for x in values):
            raise ValueError(f"Invalid {key} in {path}")
        return np.asarray(values)

    distance = metric("distance_m")
    angle = metric("bottom_normal_error_deg")
    close = distance <= _DEVELOPMENT_CLOSE_M
    return dict(checkpoint=str(checkpoint), checkpoint_sha256=checkpoint_sha,
                actor=str(actor_path), actor_sha256=summary["actor_sha256"],
                development=str(path.resolve()), development_sha256=sha256(raw).hexdigest(),
                candidate=metadata["candidate"], update=metadata["update"], frames=metadata["frames"],
                qualified=qualified, valid_geometry_contacts=sum(contact["valid_geometry"] for contact in contacts),
                safety_terminations=sum(bool(_SAFETY_FAILURES.intersection(case["failure_modes"])) for case in cases),
                close_approach_cases=int(close.sum()),
                median_close_alignment_error_deg=float(np.median(angle[close])) if close.any() else math.inf,
                median_closest_alignment_error_deg=float(np.median(angle)),
                median_closest_bumper_error_m=float(np.median(distance)))


def select_pilot(args):
    if args.output.exists():
        raise FileExistsError(f"Pilot selection is already frozen: {args.output}")
    config = load_pilot_config(args.experiment)
    config_sha = file_sha(args.experiment)
    assets = config["assets"]
    development = load_json(_ROOT / assets["gate_suites"]["path"])["suites"]["development"]
    ranked = [development_score(path, config_sha, assets, list(development["starts"])) for path in args.development]
    if len({row["checkpoint_sha256"] for row in ranked}) != len(ranked):
        raise ValueError("Each checkpoint can enter the development selection once")
    ranked.sort(key=lambda row: (-row["qualified"], -row["valid_geometry_contacts"], row["safety_terminations"],
                                 -row["close_approach_cases"], row["median_close_alignment_error_deg"],
                                 row["median_closest_bumper_error_m"],
                                 row["candidate"], row["update"], row["checkpoint"]))
    selected = ranked[0]
    receipt = dict(schema_version=1, experiment=_PILOT_EXPERIMENT, config_sha256=config_sha,
                   gate_suites_sha256=assets["gate_suites"]["sha256"], unseen_suite_sha256=assets["unseen_suite"]["sha256"],
                   checkpoint=selected["checkpoint"], checkpoint_sha256=selected["checkpoint_sha256"],
                   actor=selected["actor"], actor_sha256=selected["actor_sha256"], development_ranking=ranked,
                   close_approach_threshold_m=_DEVELOPMENT_CLOSE_M,
                   ranking_rule="Most qualified, most valid geometry, fewest safety terminations, most close approaches, "
                                "lowest median alignment on close cases, lowest overall median bumper error, "
                                "lower candidate, lower update, path.",
                   ranking_provenance="Close-approach tie-break specified before 15:44 UTC+08:00 on 2026-11-08, "
                                      "using only Candidate 1 outcomes. Candidate 2's first development run "
                                      "started after 15:47:15 local time.")
    publish_summary(args.output, receipt)
    print(f"STAGE1C_SELECTION_FROZEN c{selected['candidate']} u{selected['update']} {selected['checkpoint_sha256']}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("export", "evaluate"):
        command = commands.add_parser(name)
        command.add_argument("--checkpoint", required=True, type=Path)
        command.add_argument("--experiment", type=Path, default=_EXPERIMENT)
        command.add_argument("--bridge-config", type=Path, help="Separate pinned config for a braking bridge checkpoint")
        command.add_argument("--output", required=True, type=Path)
        if name == "evaluate":
            command.add_argument("--actor", required=True, type=Path)
            command.add_argument("--workers", type=int, default=4)
            command.add_argument("--standoff-m", type=float, default=1.0)
            command.add_argument("--hold", action="store_true")
            command.add_argument("--case", help="One named start for a diagnostic replay")
            command.add_argument("--near-aligned", action="store_true",
                                 help="Separate near-wall aligned diagnostic using the frozen scorer")
            command.add_argument("--near-inward-speed-m-s", type=float,
                                 help=f"Override the near diagnostic's {_NEAR_INWARD_SPEED_M_S} m/s inward reset speed")
            command.add_argument("--track-com-reference", action="store_true",
                                 help="Use a fixed COM reference for the selected Track actor's full 1 m approach")
            command.add_argument("--near-trim-only", action="store_true",
                                 help="Use hover collective and zero rates for the selected Track near diagnostic")
            command.add_argument("--frontier", type=int, choices=range(7),
                                 help="Run 19 zero-nominal-speed Stage 1-B promotion starts for the matching band")
            command.add_argument("--suite", choices=tuple(_PILOT_SUITES),
                                 help="Run a frozen Stage 1-C suite, with no case exclusions")
            command.add_argument("--selection", type=Path, help="Frozen Stage 1-C development selection receipt")
            command.add_argument("--constant-trim", action="store_true",
                                 help="Matched hover collective and zero body-rate setpoints for a stationary suite")
    command = commands.add_parser("select", help="Freeze the Stage 1-C checkpoint using development scores only")
    command.add_argument("--experiment", required=True, type=Path)
    command.add_argument("--development", required=True, nargs="+", type=Path)
    command.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    {"export": export_actor, "evaluate": evaluate, "select": select_pilot}[args.command](args)


if __name__ == "__main__":
    main()
