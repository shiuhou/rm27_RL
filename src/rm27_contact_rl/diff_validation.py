"""Bounded E0 forward/gradient audit; no optimizer, actor or training entry point.

Run from the RL root with --output pointing at a fresh artifact directory.
Existing inputs.json is allowed for a pre-registered run, but results never
overwrite a previous attempt. All reference integration uses ArrivalEvaluator.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np
import torch
from scipy.spatial.transform import Rotation

import rm27_contact_physics
from rm27_contact_physics.arrival import ArrivalEvaluator
from .arrival_env import AgileTerminalArrivalEnv
from .diff_dynamics import DynamicsParameters, step_control
from .paths import ROOT


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Budget:
    def __init__(self, config):
        self.config, self.started = config, time.monotonic()
        self.controls = self.substeps = 0

    def tick(self, *, controls=0, substeps=0):
        if (self.controls + controls > self.config["reference_transition_cap"]
                or self.substeps + substeps > self.config["physics_substep_cap"]
                or time.monotonic() - self.started >= self.config["wall_seconds_cap"]):
            raise RuntimeError("E0 computation budget exhausted; do not extend or resume this run")
        self.controls += controls
        self.substeps += substeps

    def summary(self):
        return dict(reference_controls=self.controls, physics_substeps=self.substeps,
                    wall_seconds=time.monotonic() - self.started)


def pack(plant):
    return np.r_[plant.data.qpos, plant.data.qvel, plant.data.act]


def reference(env, initial, actions, budget):
    """Complete CTBR chain, stopping at the first authoritative event at 1 ms.

    This is a physics window, not an episode restart for policy training. Each
    window initializes its own evaluator history; full archived episodes are
    also replayed separately to check historical event/trajectory compatibility.
    """
    p = env.plant
    mujoco.mj_resetData(p.model, p.data)
    p.data.qpos[:] = initial[:7]
    p.data.qvel[:] = initial[7:13]
    p.data.act[:] = initial[13:]
    mujoco.mj_forward(p.model, p.data)
    evaluator = ArrivalEvaluator(env.rl_config, p.physical)
    evaluator.initialize(p.model, p.data)
    values = np.full((len(actions), 17), np.nan)
    steps = np.zeros(len(actions), dtype=int)
    for j, action in enumerate(actions):
        if evaluator.done:
            break
        budget.tick(controls=1)
        _, saturated = p.apply_ctbr(np.clip(action, env._action_low, env._action_high))
        for _ in range(env.physics_substeps):
            budget.tick(substeps=1)
            before_time = p.data.time
            evaluator.advance(p.model, p.data, p.state, saturated=saturated)
            if (not np.isfinite(pack(p)).all() or np.any(p.data.warning.number)
                    or not np.isclose(p.data.time - before_time, env.physics_dt_s, atol=1e-12, rtol=0)):
                raise RuntimeError("Nonfinite state, MuJoCo warning or clock discontinuity")
            steps[j] += 1
            if evaluator.done:
                break
        values[j] = pack(p)
    return values, steps, dict(arrived=evaluator.arrived, reasons=sorted(evaluator.reasons),
                               elapsed_s=float(p.data.time), partial_controls=int(np.sum((steps > 0) & (steps < 5))))


def load_archives(config):
    archives = []
    for algorithm in ("ppo", "sac"):
        directory = ROOT / config[f"{algorithm}_evaluation"]
        summary = json.loads((directory / "summary.json").read_text())
        if summary.get("passed") is not True or summary["success_count"] != 9:
            raise ValueError("E0 requires the declared valid nine-case archived baseline")
        files = sorted(directory.glob("episode_*/trajectory.npz"))
        if len(files) != 9:
            raise ValueError("Expected exactly nine archived fixed evaluation cases")
        for path in files:
            with np.load(path, allow_pickle=False) as data:
                archives.append(dict(algorithm=algorithm, path=str(path),
                                     state=np.concatenate((data["qpos"], data["qvel"], data["motor_state_N"]), axis=-1),
                                     action=data["previous_action_ctbr"].copy(),
                                     substeps=data["physics_substeps"].copy()))
    return archives


def make_cases(env, archives, config):
    rng = np.random.default_rng(config["seed"])
    count, length = config["states_per_group"], max(config["horizon_steps"])
    states, controls, cases = [], [], []
    for group in ("hover", "maneuver", "terminal_approach", "saturation"):
        for i in range(count):
            archive = archives[i % len(archives)]
            if group in ("maneuver", "terminal_approach"):
                # Last endpoint stays one control BEFORE the original event.
                end = len(archive["state"]) - 2
                start = (int(rng.integers(0, end - length + 1)) if group == "maneuver"
                         else end - length - (i // len(archives)))
                if start < 0:
                    raise ValueError("Archive too short for predeclared 640 ms window")
                x = archive["state"][start].copy()
                u = archive["action"][start + 1:start + 1 + length].copy()
                if not np.all(archive["substeps"][start + 1:start + 1 + length] == 5):
                    raise ValueError("Window contains a partial archived control")
            else:
                start = 0
                x = archive["state"][0].copy()
                u = np.tile([env.plant.hover_force, 0., 0., 0.], (length, 1))
                # First nine hover cases retain the official nine initial states.
                if i >= 9 or group == "saturation":
                    x[7:10] = rng.uniform(-.1, .1, 3)
                    x[10:13] = rng.uniform(-.2, .2, 3)
                    q = Rotation.from_rotvec(rng.uniform(-.05, .05, 3)).as_quat()
                    x[3:7] = q[[3, 0, 1, 2]]
                if group == "saturation":
                    # Short hard pulse then hover; exact boundaries are covered
                    # separately. Outside bounds is a smooth constant branch.
                    u[:4, 0] = 5. if i % 2 else -.2
                    u[:4, 1 + i % 3] = 25. if i % 2 else -25.
            states.append(x)
            controls.append(u)
            cases.append(dict(group=group, archive=archive["path"], start_row=start))
    return np.stack(states), np.stack(controls), cases


def surrogate(initial, actions, params, horizons, budget, *, branches=False):
    states, signatures = [], []
    x = torch.cat((initial[..., :3], torch.nn.functional.normalize(initial[..., 3:7], dim=-1),
                   initial[..., 7:]), -1)
    for j in range(actions.shape[-2]):
        budget.tick()
        result = step_control(x, actions[..., j, :], params)
        x = result.state
        if branches:
            a, raw = actions[..., j, :], result.raw_motor_commands
            signatures.append(torch.cat(((a > params.action_low) & (a < params.action_high),
                                         (raw > 0) & (raw < params.maximum)), -1))
        if j + 1 in horizons:
            states.append(x)
    if branches:
        return torch.stack(states, -2), torch.stack(signatures, -2)
    return torch.stack(states, -2)


def errors(predicted, actual):
    q, r = predicted[..., 3:7], actual[..., 3:7]
    # Quaternion chord distance, sign invariant and accurate near zero.
    chord = np.minimum(np.linalg.norm(q - r, axis=-1), np.linalg.norm(q + r, axis=-1))
    return dict(position_m=np.linalg.norm(predicted[..., :3] - actual[..., :3], axis=-1),
                velocity_m_s=np.linalg.norm(predicted[..., 7:10] - actual[..., 7:10], axis=-1),
                attitude_deg=np.rad2deg(4 * np.arcsin(np.clip(chord / 2, 0, 1))),
                body_rate_rad_s=np.linalg.norm(predicted[..., 10:13] - actual[..., 10:13], axis=-1))


def derivative_metrics(candidate, target, mask):
    # Fixed physical normalization; quaternion derivative uses 2*dq, locally radians.
    scale = np.array([1.] * 3 + [.5] * 4 + [6.] * 3 + [20.] * 3 + [1.103625] * 4)
    a, b = candidate / scale, target / scale
    results = {}
    for name, index in {"all": slice(None), "position": slice(0, 3), "attitude": slice(3, 7),
                        "velocity": slice(7, 10), "body_rate": slice(10, 13), "motors": slice(13, 17)}.items():
        aa, bb = a[..., index], b[..., index]
        norm = np.linalg.norm(bb, axis=-1)
        active = mask & (norm > 1e-8)
        relative = np.linalg.norm(aa - bb, axis=-1) / np.maximum(norm, 1e-8)
        cosine = (aa * bb).sum(-1) / np.maximum(np.linalg.norm(aa, axis=-1) * norm, 1e-30)
        results[name] = dict(count=int(active.sum()), near_zero=int((mask & ~active).sum()),
                             max_relative=float(relative[mask].max()) if mask.any() else None,
                             min_cosine=float(cosine[active].min()) if active.any() else None)
    return results


def run(config, output, report, budget):
    torch.set_num_threads(1)
    env = AgileTerminalArrivalEnv(ROOT / config["task_config"])
    env.reset(seed=config["seed"])
    params = DynamicsParameters.from_env(env)
    archives = load_archives(config)
    # Re-certify recorded INPUT replay on current sources, not learned policy
    # performance. No policy is loaded, and old summaries remain untouched.
    replay = []
    for archive in archives:
        values, steps, event = reference(env, archive["state"][0], archive["action"][1:], budget)
        valid = steps > 0
        distance = errors(values[valid], archive["state"][1:][valid])
        replay.append(dict(algorithm=archive["algorithm"], path=archive["path"], event=event,
                           substeps_identical=bool(np.array_equal(steps, archive["substeps"][1:])),
                           max_errors={k: float(v.max()) for k, v in distance.items()}))
    report["archived_input_replay"] = replay
    print("Archived-input replay complete; building 128 fixed E0 windows.", flush=True)
    initial, actions, cases = make_cases(env, archives, config)
    horizons = config["horizon_steps"]
    indices = np.array(horizons) - 1
    rng = np.random.default_rng(config["seed"] + 1)
    dx = rng.normal(size=initial.shape)
    dx[:, 3:7] -= (dx[:, 3:7] * initial[:, 3:7]).sum(-1, keepdims=True) * initial[:, 3:7]
    dx *= np.array([1.] * 3 + [.5] * 4 + [6.] * 3 + [20.] * 3 + [1.103625] * 4)
    du = rng.normal(size=actions.shape) * np.array([4.4145, 20., 20., 20.])
    eps = config["finite_difference_epsilon"]
    x, u, vx, vu = (torch.tensor(a, dtype=torch.float64) for a in (initial, actions, dx, du))
    with torch.no_grad():
        predicted, branch = surrogate(x, u, params, horizons, budget, branches=True)
        plus, branch_plus = surrogate(x + eps * vx, u + eps * vu, params, horizons, budget, branches=True)
        minus, branch_minus = surrogate(x - eps * vx, u - eps * vu, params, horizons, budget, branches=True)
    _, jvp = torch.autograd.functional.jvp(
        lambda a, b: surrogate(a, b, params, horizons, budget), (x, u), (vx, vu), strict=True)
    if not torch.isfinite(jvp).all():
        raise RuntimeError("Nonfinite autograd derivative")
    self_fd = ((plus - minus) / (2 * eps)).numpy()
    smooth = np.stack([((branch_plus[:, :h] == branch_minus[:, :h])
                       & (branch_plus[:, :h] == branch[:, :h])).all(dim=-1).all(dim=-1).numpy()
                       for h in horizons], axis=-1)
    reference_values, reference_steps, events = [], [], []
    for sign in (0, 1, -1):
        values, steps, variant_events = [], [], []
        for i in range(len(initial)):
            perturbed = initial[i] + sign * eps * dx[i]
            perturbed[3:7] /= np.linalg.norm(perturbed[3:7])
            result, counts, event = reference(env, perturbed, actions[i] + sign * eps * du[i], budget)
            values.append(result[indices]); steps.append(counts[indices]); variant_events.append(event)
        reference_values.append(np.stack(values)); reference_steps.append(np.stack(steps)); events.append(variant_events)
        print(f"Reference variant {sign:+d} complete: {budget.controls} controls, {budget.substeps} substeps.", flush=True)
    full = np.stack(reference_steps) == 5
    valid = full.all(axis=0)
    smooth &= valid
    fd = (reference_values[1] - reference_values[2]) / (2 * eps)
    predicted = predicted.numpy()
    forward = errors(predicted, reference_values[0])
    report["windows"] = dict(count=len(initial), groups=dict(Counter(c["group"] for c in cases)),
                              events=events, completed_by_horizon=full[0].sum(axis=0).tolist(),
                              smooth_by_horizon=smooth.sum(axis=0).tolist(),
                              nonsmooth_or_event_by_horizon=(~smooth).sum(axis=0).tolist())
    report["forward"] = {}
    for j, h in enumerate(horizons):
        active = full[0, :, j]
        report["forward"][str(h * 5)] = {k: dict(p95=float(np.percentile(v[active, j], 95)),
                                                    maximum=float(v[active, j].max())) for k, v in forward.items()}
    report["self_gradient"] = derivative_metrics(jvp.numpy(), self_fd, smooth)
    report["reference_gradient"] = derivative_metrics(jvp.numpy(), fd, smooth)
    # Exact clamp boundaries: compare left and right finite differences, NEVER
    # conflate their average or Torch's chosen subgradient with a true derivative.
    boundary = []
    base = archives[0]["state"][0].copy()
    for name, action, direction in (
        ("collective_lower_and_motor_lower", [0., 0., 0., 0.], [1., 0., 0., 0.]),
        ("collective_upper_and_motor_upper", [4 * env.plant.maximum, 0., 0., 0.], [1., 0., 0., 0.]),
        ("body_rate_upper", [env.plant.hover_force, 20., 0., 0.], [0., 1., 0., 0.]),
    ):
        action, direction = np.asarray(action), np.asarray(direction)
        pairs = []
        for sign in (0, 1, -1):
            a = action + sign * eps * direction
            actual, _, _ = reference(env, base, a[None], budget)
            prediction = step_control(torch.tensor(base), torch.tensor(a), params).state.detach().numpy()
            pairs.append((prediction, actual[0]))
        sides = []
        for side in (1, 2):
            pfd = (pairs[side][0] - pairs[0][0]) / eps
            rfd = (pairs[side][1] - pairs[0][1]) / eps
            sides.append(float(np.linalg.norm(pfd - rfd) / max(np.linalg.norm(rfd), 1e-8)))
        boundary.append(dict(name=name, right_relative=sides[0], left_relative=sides[1],
                             reference_right_norm=float(np.linalg.norm((pairs[1][1] - pairs[0][1]) / eps)),
                             reference_left_norm=float(np.linalg.norm((pairs[0][1] - pairs[2][1]) / eps))))
    report["boundary_one_sided"] = boundary
    # At least eight smooth cases at each horizon, and all fixed windows must
    # complete. Event exclusions therefore cannot manufacture a passing gate.
    report["checks"] = dict(
        coverage=bool(full.all() and np.all(smooth.sum(axis=0) >= 8)),
        forward=all(report["forward"]["640"][k]["p95"] <= limit for k, limit in config["forward_p95_limits"].items()),
        self_gradient=all(v["max_relative"] is not None and v["max_relative"] <= config["self_gradient_relative_limit"]
                          for v in report["self_gradient"].values()),
        reference_gradient=all(v["max_relative"] is not None and v["max_relative"] <= config["reference_gradient_relative_limit"]
                               and (v["min_cosine"] is None or v["min_cosine"] >= config["reference_gradient_cosine_min"])
                               for v in report["reference_gradient"].values()),
        boundary=all(max(v["left_relative"], v["right_relative"]) <= config["reference_gradient_relative_limit"] for v in boundary),
        archived_input_replay=all(v["substeps_identical"] and v["event"]["arrived"]
                                  and all(v["max_errors"][k] <= limit for k, limit in config["forward_p95_limits"].items()) for v in replay))
    np.savez_compressed(output / "samples.npz", initial=initial, actions=actions, state_directions=dx,
                        action_directions=du, predicted=predicted, reference=np.stack(reference_values),
                        reference_substeps=np.stack(reference_steps), smooth=smooth,
                        autograd=jvp.numpy(), self_fd=self_fd, reference_fd=fd)
    (output / "cases.json").write_text(json.dumps(cases, indent=2) + "\n")
    env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/agile_flight/diff_e0.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    # Exclusive creation: even aborted attempts cannot be silently overwritten.
    with (args.output / "started.json").open("x") as file:
        json.dump(dict(config=config, mujoco=mujoco.__version__, torch=torch.__version__), file, indent=2)
    manifest_path = args.output / "inputs.json"
    if not manifest_path.exists():
        files = list((ROOT / "src").rglob("*.py")) + list((ROOT / "configs").rglob("*.json"))
        files += list(Path(rm27_contact_physics.__file__).parent.rglob("*.py"))
        for key in ("ppo_evaluation", "sac_evaluation"):
            files += list((ROOT / config[key]).rglob("*.json")) + list((ROOT / config[key]).rglob("*.npz"))
        manifest_path.write_text(json.dumps(dict(protected_sha256={str(f): sha(f) for f in files},
                                                 pre_registered_config=config), indent=2) + "\n")
    manifest = json.loads(manifest_path.read_text())
    protected = manifest["protected_sha256"]
    implementation = {str(Path(__file__)): sha(__file__),
                      str(Path(__file__).with_name("diff_dynamics.py")): sha(Path(__file__).with_name("diff_dynamics.py"))}
    report = dict(status="NOT_RUN", config=config, implementation_sha256=implementation,
                  protected_input_count=len(protected), policy_training="NOT_RUN",
                  scope="nominal I4 contact-free physics and recorded-input replay; no policy performance claim")
    budget = Budget(config)
    try:
        if manifest["pre_registered_config"] != config or any(sha(p) != h for p, h in protected.items()):
            raise RuntimeError("Pre-registered input or configuration changed")
        run(config, args.output, report, budget)
        budget.tick()
        report["status"] = "PASS" if all(report["checks"].values()) else "FAIL"
    except Exception as exc:
        report["status"], report["error"] = "FAIL", repr(exc)
        raise
    finally:
        unchanged = all(Path(p).exists() and sha(p) == h for p, h in (protected | implementation).items())
        report["inputs_unchanged"] = unchanged
        if not unchanged:
            report["status"] = "FAIL"
        report["budget"] = budget.summary()
        (args.output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({k: report[k] for k in ("status", "inputs_unchanged", "budget")}), flush=True)
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
