"""Bounded action-mapping intervention; frozen PPO, original MuJoCo/scorer.

Run with the repository .venv and a fresh --output directory. The wall budget
includes imports, initialization, simulation and per-case evidence writing.
No training entry point is called. Offline presentation is a separate action.
"""
import time

STARTED = time.monotonic()
import argparse
import hashlib
import json
import os
from pathlib import Path
import pickle
import signal

os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
for variable in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                 "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[variable] = "1"

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--config", type=Path, default=ROOT / "configs/stage1/action_scale_diagnostic.json")
args = parser.parse_args()
cfg = json.loads(args.config.read_text())
assert cfg["protocol"] == "stage1_action_scale_diagnostic_no_training"
assert cfg["rate_scale_betas"] == [1., .1]
assert cfg["budget"] == dict(controls=4096, physics_substeps=20480, wall_s=120)
output = args.output.resolve()
output.mkdir(parents=True, exist_ok=False)


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def deadline(*_):
    raise TimeoutError("Action-scale diagnostic wall budget exhausted")


signal.signal(signal.SIGALRM, deadline)
signal.setitimer(signal.ITIMER_REAL, max(.001, 120 - (time.monotonic() - STARTED)))
import mujoco
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from stable_baselines3 import PPO
from rm27_contact_rl.ppo import NormalizedCTBR
from rm27_contact_rl.stage1_env import Stage1ContactEnv
from rm27_contact_rl.stage1_plant import Stage1Plant
from rm27_contact_rl.stage1_runner import SmokeBudget, _episode_sample, _write_episode


def forbidden(*_, **__):
    raise RuntimeError("Training and optimizer updates are forbidden in this diagnostic")


PPO.learn = PPO.train = forbidden
torch.optim.Adam.step = forbidden
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
budget = SmokeBudget(controls=4096, substeps=20480, wall_s=120)
budget.started, budget.deadline = STARTED, STARTED + 120
reference = ROOT / cfg["reference_run"]
metadata = json.loads((reference / "evaluation_after/episode_00/config.json").read_text())
checkpoint = reference / "checkpoints" / cfg["checkpoint"]
init_start = time.monotonic()
plant = Stage1Plant(metadata["rl"], model_path=metadata["private_model_path"])
env = Stage1ContactEnv(ROOT / cfg["task_config"], plant=plant)
assert env.rl_config == metadata["rl"]
assert env._action_high[1:].tolist() == [20.] * 3
model = PPO.load(checkpoint / "model.zip", device="cpu")
model.policy.set_training_mode(False)
for parameter in model.policy.parameters():
    parameter.requires_grad_(False)
with (checkpoint / "normalize.pkl").open("rb") as stream:
    normalizer = pickle.load(stream)
normalizer.training, normalizer.norm_reward = False, False


def network_hash():
    h = hashlib.sha256()
    for name, tensor in model.policy.state_dict().items():
        h.update(name.encode())
        h.update(tensor.detach().cpu().numpy().tobytes())
    return h.hexdigest()


def normalizer_hash():
    return hashlib.sha256(pickle.dumps((normalizer.obs_rms, normalizer.ret_rms))).hexdigest()


network_before, normalizer_before = network_hash(), normalizer_hash()
initialization_s = time.monotonic() - init_start
ids, actids = plant.motor_ids, plant.actuator_state_ids
motor_limit = plant.maximum
original_step = mujoco.mj_step
active_substeps = []


def measured_step(m, d):
    # Observe the genuine step at the original call site; no replacement physics.
    if budget.substeps >= budget.max_substeps or time.monotonic() >= budget.deadline:
        raise TimeoutError("Action-scale physical-step budget exhausted")
    assert m is plant.model and d is plant.data
    before_time = float(d.time)
    before_act = d.act[actids].copy()
    original_step(m, d)
    budget.substeps += 1
    active_substeps.append(dict(time_start_s=before_time, time_end_s=float(d.time),
        motor_activation_start_N=before_act, motor_activation_end_N=d.act[actids].copy(),
        motor_command_N=d.ctrl[ids].copy(), actual_force_N=d.actuator_force[ids].copy(),
        actual_wrench=plant.mixer @ d.actuator_force[ids],
        position=d.qpos[plant.qa:plant.qa+3].copy(), quaternion_wxyz=d.qpos[plant.qa+3:plant.qa+7].copy(),
        velocity=d.qvel[plant.va:plant.va+3].copy(), omega_body=d.qvel[plant.va+3:plant.va+6].copy()))


mujoco.mj_step = measured_step
results = {}


def run_case(name, beta, axis=None, sign=0, actor=False):
    case_started = time.monotonic()
    case_id = f"{name}_beta_{beta:g}"
    wrapper = NormalizedCTBR(env, cfg["action_parameterization"], rate_scale_beta=beta)
    raw, info = wrapper.reset(seed=0, options={"initial_perturbation": np.zeros(12)})
    initial_state = plant.snapshot()
    rows = [_episode_sample(env, np.zeros(4), env.previous_action_ctbr, 0., info)]
    controls = []
    active_substeps.clear()
    max_s = cfg["frozen_policy_max_s"] if actor else (cfg["hover_s"] if axis is None else cfg["pulse_s"] + cfg["release_s"])
    count = int(round(max_s / env.policy_dt_s))
    budget_error = None
    done = not env._active
    for control in range(count):
        if done:
            break
        try:
            budget.before_step()
            if actor:
                observation = normalizer.normalize_obs(raw)
                action, _ = model.predict(observation, deterministic=True)
                with torch.no_grad():
                    actor_mean = model.policy.get_distribution(torch.as_tensor(
                        observation.reshape(1, -1), device="cpu")).distribution.mean.numpy().ravel()
            else:
                command = np.r_[plant.hover_force, np.zeros(3)]
                if axis is not None and control < round(cfg["pulse_s"] / env.policy_dt_s):
                    command[axis + 1] = sign * cfg["pulse_rate_rad_s"]
                unscaled = command.copy()
                unscaled[1:] /= beta
                action = (unscaled - wrapper.center) / wrapper.half_range
                actor_mean = action.copy()
                assert np.max(np.abs(action)) <= 1.
                np.testing.assert_allclose(wrapper.action(action), command, rtol=0, atol=1e-12)
            requested = wrapper.action(action)
            applied = np.clip(requested, env._action_low, env._action_high)
            omega = plant.state.omega_body
            torque = plant.inertia * plant.rate_gain * (applied[1:] - omega) + np.cross(omega, plant.inertia * omega)
            desired = np.r_[applied[0], torque]
            raw_motors = plant.inverse_mixer @ desired
            substep_start = len(active_substeps)
            start_time = float(plant.data.time)
            budget.controls += 1
            raw, reward, done, truncated, info = wrapper.step(action)
            assert not truncated
            samples = active_substeps[substep_start:]
            assert len(samples) == info["physics_substeps"]
            allocated = plant.mixer @ plant.data.ctrl[ids]
            np.testing.assert_allclose(plant.data.ctrl[ids], np.clip(raw_motors, 0, motor_limit), rtol=0, atol=1e-12)
            controls.append(dict(time_start_s=start_time, time_end_s=float(plant.data.time),
                requested_wrench=desired, raw_motor_command_N=raw_motors,
                allocated_wrench=allocated,
                actual_wrench_mean=np.mean([s["actual_wrench"] for s in samples], axis=0),
                normalized_action=action.copy(), actor_mean=actor_mean.copy(),
                requested_ctbr=requested.copy(), applied_ctbr=applied,
                mixer_clipped=bool(info["proxy_saturated"]), physical_ctbr_clipped=bool(info["action_clipped"])))
            rows.append(_episode_sample(env, action, requested, reward, info, actor_mean))
        except TimeoutError as error:
            budget_error = str(error)
            break
    status = "INCOMPLETE_BUDGET" if budget_error else ("TASK_TERMINATED" if done else "DIAGNOSTIC_WINDOW_COMPLETE")
    result = env.evaluator.summary()
    result.update(case=case_id, action_mapping=wrapper.mapping_identity, execution_status=status,
        diagnostic_window_s=max_s, actual_duration_s=float(plant.data.time), budget_error=budget_error,
        total_reward=float(sum(row["reward"] for row in rows)),
        reward_component_totals=env.reward_component_totals.copy(),
        policy="FROZEN_FINAL_ACTOR_AND_NORMALIZER" if actor else "MATCHED_PHYSICAL_CTBR",
        fixture_state=None, initial_perturbation=np.zeros(12).tolist())
    case_dir = output / case_id
    _write_episode(case_dir, env, rows, initial_state, result)
    for filename, records in (("control_chain.npz", controls), ("physical_substeps.npz", active_substeps)):
        if records:
            np.savez_compressed(case_dir / filename, **{key: np.asarray([r[key] for r in records]) for key in records[0]})
    omega = np.array([r["omega_body"] for r in active_substeps])
    rot = np.array([r["rotation"] for r in rows])
    yaw = np.unwrap(np.arctan2(rot[:, 1, 0], rot[:, 0, 0]))
    metrics = dict(controls=len(controls), physical_substeps=len(active_substeps),
        max_body_rate_rad_s=float(np.linalg.norm(omega, axis=1).max()) if len(omega) else 0.,
        final_body_rate_rad_s=plant.state.omega_body.tolist(),
        max_command_rate_norm_rad_s=max((float(np.linalg.norm(c["applied_ctbr"][1:])) for c in controls), default=0.),
        mixer_clip_fraction=float(np.mean([c["mixer_clipped"] for c in controls])) if controls else 0.,
        actor_clip_fraction=float(np.mean([np.any(np.abs(c["actor_mean"]) > 1) for c in controls])) if actor and controls else 0.,
        physical_ctbr_clip_fraction=float(np.mean([c["physical_ctbr_clipped"] for c in controls])) if controls else 0.,
        world_yaw_span_deg=float(np.degrees(np.ptp(yaw))),
        orientation_travel_deg=float(np.degrees(Rotation.from_matrix(np.swapaxes(rot[:-1], 1, 2) @ rot[1:]).magnitude().sum())),
        final_position_delta_m=(plant.state.position-rows[0]["position"]).tolist(),
        final_velocity_m_s=plant.state.velocity.tolist(),
        max_allocation_error_wrench=np.max(np.abs(np.array([c["allocated_wrench"]-c["requested_wrench"] for c in controls])), axis=0).tolist() if controls else None,
        max_actual_mean_error_wrench=np.max(np.abs(np.array([c["actual_wrench_mean"]-c["requested_wrench"] for c in controls])), axis=0).tolist() if controls else None,
        actual_motor_force_range_N=[float(np.min([s["actual_force_N"] for s in active_substeps])), float(np.max([s["actual_force_N"] for s in active_substeps]))] if active_substeps else None,
        motor_activation_range_N=[float(np.min([s["motor_activation_end_N"] for s in active_substeps])), float(np.max([s["motor_activation_end_N"] for s in active_substeps]))] if active_substeps else None,
        case_wall_s=time.monotonic()-case_started)
    if axis is not None and len(omega):
        metrics["pulse_tracking_peak_signed_rate_rad_s"] = float(np.max(sign*omega[:, axis]))
        metrics["release_final_error_rad_s"] = float(np.linalg.norm(plant.state.omega_body))
    result["diagnostic_metrics"] = metrics
    dump(case_dir / "summary.json", result)
    results[case_id] = result
    print(json.dumps(dict(case=case_id, status=status, failure=result["failure_modes"], duration=result["actual_duration_s"],
                          max_rate=metrics["max_body_rate_rad_s"], saturation=metrics["mixer_clip_fraction"],
                          budget=budget.summary())), flush=True)
    if budget_error:
        raise TimeoutError(budget_error)


error = None
try:
    for name, axis, sign in [("hover", None, 0)] + [(f"pulse_{axis}_{sign:+d}", axis, sign) for axis in range(3) for sign in (-1, 1)]:
        for beta in cfg["rate_scale_betas"]:
            run_case(name, beta, axis, sign)
    for beta in cfg["rate_scale_betas"]:
        run_case("final_actor", beta, actor=True)
except Exception as exc:
    error = dict(type=type(exc).__name__, message=str(exc))
finally:
    signal.setitimer(signal.ITIMER_REAL, 0)
    mujoco.mj_step = original_step
    frozen = dict(network_unchanged=network_hash() == network_before,
                  normalizer_statistics_unchanged=normalizer_hash() == normalizer_before,
                  network_sha256=network_before, normalizer_statistics_sha256=normalizer_before,
                  checkpoint_sha256=sha(checkpoint / "model.zip"), normalize_file_sha256=sha(checkpoint / "normalize.pkl"))
    source_paths = [Path(__file__), args.config, Path(__import__("rm27_contact_rl.ppo", fromlist=["__file__"]).__file__)]
    source_paths += list(plant.sources.values()) + [ROOT / cfg["task_config"]]
    dump(output / "summary.json", dict(protocol=cfg["protocol"], config=cfg, budget=budget.summary(),
        error=error, completed_cases=len(results), expected_cases=16, cases=results, frozen=frozen,
        initialization_s=initialization_s, import_and_startup_s=init_start-STARTED, model_compilation_s=0.,
        training_transitions=0, optimizer_updates=0, cpu_affinity=sorted(os.sched_getaffinity(0)),
        os_threads=len(list(Path('/proc/self/task').iterdir())), torch_threads=torch.get_num_threads(),
        versions=dict(mujoco=mujoco.__version__, torch=torch.__version__),
        sources_sha256={str(p.resolve()): sha(p) for p in source_paths},
        note="Diagnostic-window completion is not task success; terminal failures and all substeps preserved."))
    env.close()
if error:
    raise SystemExit(error)
