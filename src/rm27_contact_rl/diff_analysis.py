"""Stratified E0 analysis and bounded supplementary probes of an existing audit.

The original audit and its verdict are immutable inputs. Supplementary physics
is charged to its remaining budget, with a conservative allowance for previous
unmetered development checks. No policy is loaded or optimized.
"""
from __future__ import annotations

import argparse
from dataclasses import fields
from importlib import metadata
import json
from pathlib import Path
import subprocess
import tarfile
import time

import mujoco
import numpy as np
import torch

import rm27_contact_physics
from .arrival_env import AgileTerminalArrivalEnv
from .diff_dynamics import DynamicsParameters, step_control
from .diff_validation import Budget, errors, reference, sha, surrogate
from .paths import ROOT
from .ppo import HOVER_CENTERED, NormalizedCTBR


STATE_SCALE = np.array([1.] * 3 + [.5] * 4 + [6.] * 3 + [20.] * 3 + [1.103625] * 4)
BLOCKS = dict(all=slice(None), position=slice(0, 3), attitude=slice(3, 7),
              velocity=slice(7, 10), body_rate=slice(10, 13), motors=slice(13, 17))


def agreement(candidate, target, mask, *, rtol, atol=1e-7, zero_floor=1e-8):
    """Absolute-plus-relative criterion; cosine is undefined for near-zero vectors.

    Fixed normalization: metres, quaternion differential/0.5 (local radians),
    6 m/s, 20 rad/s and 1.103625 nominal motor N. Absolute tolerances are in
    this normalized derivative space; they are not trajectory-error tolerances.
    """
    a, b = np.asarray(candidate) / STATE_SCALE, np.asarray(target) / STATE_SCALE
    result = {}
    for name, index in BLOCKS.items():
        aa, bb = a[..., index][mask], b[..., index][mask]
        an, bn = np.linalg.norm(aa, axis=-1), np.linalg.norm(bb, axis=-1)
        absolute = np.linalg.norm(aa - bb, axis=-1)
        relative_defined = bn > zero_floor
        cosine_defined = relative_defined & (an > zero_floor)
        cosine = ((aa * bb).sum(-1)[cosine_defined]
                  / (an[cosine_defined] * bn[cosine_defined]))
        accepted = absolute <= atol + rtol * bn
        result[name] = dict(count=int(len(bn)), near_zero=int((~relative_defined).sum()),
                            max_absolute=float(absolute.max()) if len(bn) else None,
                            max_relative=float((absolute[relative_defined] / bn[relative_defined]).max())
                            if relative_defined.any() else None,
                            min_cosine=float(np.clip(cosine, -1, 1).min()) if len(cosine) else None,
                            reference_norm_min=float(bn.min()) if len(bn) else None,
                            reference_norm_max=float(bn.max()) if len(bn) else None,
                            failures=int((~accepted).sum()), rtol=rtol, atol=atol,
                            zero_floor=zero_floor)
    return result


def event_mask(event, horizons):
    """An event on the fifth substep is still an event, not a smooth endpoint."""
    done = bool(event["arrived"] or event["reasons"])
    return np.array([done and event["elapsed_s"] <= h * .005 + 1e-12 for h in horizons])


def classify(initial, actions, params, horizons, budget):
    """Interior, strictly saturated, or exact/near boundary, by whole prefix.

    Boundary tolerance is 1e-10 of each actuator/action scale. Strictly clipped
    branches remain differentiable locally, including a zero action derivative.
    This geometric classification does not assert a unique event derivative.
    """
    x = torch.tensor(initial, dtype=torch.float64)
    u = torch.tensor(actions, dtype=torch.float64)
    near, saturated = [], []
    action_scale = params.action_high - params.action_low
    for j in range(u.shape[1]):
        budget.tick()
        step = step_control(x, u[:, j], params)
        x = step.state
        action_margin = torch.minimum((u[:, j] - params.action_low) / action_scale,
                                      (params.action_high - u[:, j]) / action_scale)
        motor_margin = torch.minimum(step.raw_motor_commands / params.maximum,
                                     (params.maximum - step.raw_motor_commands) / params.maximum)
        margin = torch.cat((action_margin, motor_margin), -1)
        near.append((margin.abs() <= 1e-10).any(-1))
        saturated.append((margin < -1e-10).any(-1))
    near, saturated = torch.stack(near, 1), torch.stack(saturated, 1)
    labels = np.empty((len(initial), len(horizons)), dtype="U24")
    for j, h in enumerate(horizons):
        boundary = near[:, :h].any(-1).numpy()
        clipped = saturated[:, :h].any(-1).numpy()
        labels[:, j] = np.where(boundary, "boundary", np.where(clipped, "strict_saturation", "interior"))
    return labels


def stratify(data, cases, params, config, budget):
    h = config["horizon_steps"]
    labels = classify(data["initial"], data["actions"], params, h, budget)
    forward = errors(data["predicted"], data["reference"][0])
    result = {}
    for group in ("all", "hover", "maneuver", "terminal_approach", "saturation"):
        selected = np.array([group == "all" or c["group"] == group for c in cases])
        result[group] = {}
        for j, steps in enumerate(h):
            masks = {name: selected & (labels[:, j] == name)
                     for name in ("interior", "strict_saturation", "boundary")}
            result[group][str(steps * 5)] = dict(
                count=int(selected.sum()), counts={k: int(v.sum()) for k, v in masks.items()},
                forward={k: dict(p95=float(np.percentile(v[selected, j], 95)),
                                 maximum=float(v[selected, j].max())) for k, v in forward.items()},
                gradients={name: agreement(data["autograd"][:, j], data["reference_fd"][:, j],
                                           mask & data["smooth"][:, j], rtol=.05)
                           for name, mask in masks.items() if name != "boundary"})
    return result, labels


def existing_directional_sides(data, labels, params, config, budget):
    """Use already paid MuJoCo +/- traces for one-sided boundary consistency."""
    x, u, vx, vu = (torch.tensor(data[name], dtype=torch.float64) for name in
                    ("initial", "actions", "state_directions", "action_directions"))
    eps = config["finite_difference_epsilon"]
    def forward(a, b):
        return surrogate(a, b, params, config["horizon_steps"], budget)
    result = []
    for variant, sign in ((1, 1), (2, -1)):
        value, jvp = torch.autograd.functional.jvp(
            forward, (x + sign * eps * vx, u + sign * eps * vu), (vx, vu), strict=True)
        fd = (data["reference"][variant] - data["reference"][0]) / (sign * eps)
        self_fd = (value.numpy() - data["predicted"]) / (sign * eps)
        mask = labels == "boundary"
        result.append(dict(side="right" if sign == 1 else "left", epsilon=eps,
                           reference=agreement(jvp.numpy(), fd, mask, rtol=.05),
                           self_model=agreement(jvp.numpy(), self_fd, mask, rtol=1e-4)))
    return result


def freeze_sources(env, original, output):
    protected = original["protected_sha256"]
    paths = [Path(p) for p in protected if "/artifacts/" not in p]
    paths += [Path(__file__), ROOT / "src/rm27_contact_rl/diff_dynamics.py",
              ROOT / "src/rm27_contact_rl/diff_validation.py", ROOT / "pyproject.toml",
              ROOT / "requirements-contact-rl.lock", ROOT / "tests/test_diff_dynamics.py"]
    paths = sorted(set(paths))
    physics_root = Path(rm27_contact_physics.__file__).resolve().parents[4]
    def git(repo):
        return {key: subprocess.check_output(["git", *args], cwd=repo, text=True).strip()
                for key, args in dict(head=["rev-parse", "HEAD"], branch=["branch", "--show-current"],
                                      status=["status", "--short"]).items()}
    contents = {}
    with tarfile.open(output / "sources.tar.gz", "x:gz") as archive:
        for path in paths:
            alias = ("rl/" + str(path.relative_to(ROOT)) if path.is_relative_to(ROOT)
                     else "physics/" + str(path.relative_to(physics_root)))
            archive.add(path, arcname=alias, recursive=False)
            contents[str(path)] = dict(archive_path=alias, sha256=sha(path))
    m = env.plant.model
    matrix = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, env.plant.data, matrix)
    (output / "model.xml").write_text(env.plant.xml)
    params = DynamicsParameters.from_env(env)
    compiled = {field.name: (getattr(params, field.name).tolist() if isinstance(getattr(params, field.name), torch.Tensor)
                            else getattr(params, field.name)) for field in fields(params)}
    compiled.update(dof_M0=m.dof_M0.tolist(), mass_matrix=matrix.tolist(),
                    body_mass=m.body_mass.tolist(), body_inertia=m.body_inertia.tolist(),
                    actuator_gainprm=m.actuator_gainprm.tolist(), actuator_gear=m.actuator_gear.tolist())
    result = dict(repositories=dict(rl=git(ROOT), physics=git(physics_root)),
                  packages={name: metadata.version(name) for name in
                            ("mujoco", "numpy", "scipy", "torch", "stable-baselines3", "gymnasium")},
                  loaded_paths=dict(rl=str(ROOT / "src/rm27_contact_rl"), physics=rm27_contact_physics.__file__),
                  content=contents, compiled=compiled, source_archive_sha256=sha(output / "sources.tar.gz"),
                  model_xml_sha256=sha(output / "model.xml"))
    (output / "sources.json").write_text(json.dumps(result, indent=2) + "\n")
    return contents


def probe(env, params, data, indices, kind, epsilons, config, budget, report, samples):
    """Independent action/state probes; both directional sides at clamp boundaries.

    For a boundary, JVP at the small one-sided shifted input is compared with
    that side's finite secant, not with a fictitious centered derivative.
    Epsilon sweep exposes finite-step/roundoff sensitivity rather than fitting.
    """
    h = config["horizon_steps"]
    x = torch.tensor(data["initial"][indices], dtype=torch.float64)
    u = torch.tensor(data["actions"][indices], dtype=torch.float64)
    vx = torch.zeros_like(x) if kind != "state_only" else torch.tensor(data["state_directions"][indices])
    vu = torch.zeros_like(u) if kind == "state_only" else torch.tensor(data["action_directions"][indices])
    wrapper = NormalizedCTBR(env, HOVER_CENTERED)
    if kind == "normalized_action_only":
        normalized = []
        cases = report["case_sources"]
        for index in indices:
            case = cases[index]
            with np.load(case["archive"]) as archive:
                start = case["start_row"] + 1
                normalized.append(archive["action_normalized"][start:start + max(h)])
        u = torch.tensor(np.stack(normalized), dtype=torch.float64)
        vu = vu / torch.tensor(wrapper.half_range)
    if kind == "fully_clipped_action_only":
        vu = torch.zeros_like(u)
        vu[:, :4, 0] = 1.
    center, half = torch.tensor(wrapper.center), torch.tensor(wrapper.half_range)
    def map_action(value):
        return center + value.clamp(-1, 1) * half if kind == "normalized_action_only" else value
    def forward(a, b):
        return surrogate(a, map_action(b), params, h, budget)
    base, base_jvp = torch.autograd.functional.jvp(forward, (x, u), (vx, vu), strict=True)
    reference_base = data["reference"][0, indices]
    if kind == "normalized_action_only":
        # Validate the existing wrapper at the recorded inputs, without learn().
        physical = np.stack([[wrapper.action(a) for a in episode] for episode in u.numpy()])
        np.testing.assert_allclose(map_action(u).numpy(), physical, rtol=0, atol=1e-14)
        np.testing.assert_allclose(np.clip(physical, env._action_low, env._action_high),
                                   data["actions"][indices], rtol=0, atol=1e-12)
    entries = report.setdefault("independent_probes", {}).setdefault(kind, [])
    for eps in epsilons:
        side_values, side_jvp, refs, events = [], [], [], []
        for sign in (1, -1):
            value, jvp = torch.autograd.functional.jvp(
                forward, (x + sign * eps * vx, u + sign * eps * vu), (vx, vu), strict=True)
            if not bool(torch.isfinite(value).all() & torch.isfinite(jvp).all()):
                raise RuntimeError("Nonfinite supplemental surrogate or derivative")
            side_values.append(value.detach().numpy()); side_jvp.append(jvp.detach().numpy())
            variant, variant_events = [], []
            for i, index in enumerate(indices):
                initial = (x[i] + sign * eps * vx[i]).numpy().copy()
                initial[3:7] /= np.linalg.norm(initial[3:7])
                action = map_action(u[i] + sign * eps * vu[i]).numpy()
                report["progress"]["active_reference_variant"] = dict(
                    kind=kind, epsilon=eps, sign=sign, case_index=index,
                    started_controls=budget.controls, started_substeps=budget.substeps)
                values, steps, event = reference(env, initial, action, budget)
                variant.append(values[np.array(h) - 1]); variant_events.append(event)
                report["progress"]["completed_reference_variants"].append(
                    dict(kind=kind, epsilon=eps, sign=sign, case_index=index,
                         completed_steps=int((steps == 5).sum()), event=event))
                report["progress"]["active_reference_variant"] = None
            refs.append(np.stack(variant)); events.append(np.stack([event_mask(e, h) for e in variant_events]))
        valid = ~(events[0] | events[1]) & np.isfinite(refs[0]).all(-1) & np.isfinite(refs[1]).all(-1)
        sides = []
        for j, sign in enumerate((1, -1)):
            fd = (refs[j] - reference_base) / (sign * eps)
            self_fd = (side_values[j] - base.detach().numpy()) / (sign * eps)
            sides.append(dict(side="right" if sign == 1 else "left",
                              reference=agreement(side_jvp[j], fd, valid, rtol=.05),
                              self_model=agreement(side_jvp[j], self_fd, valid, rtol=1e-4)))
        centered = (refs[0] - refs[1]) / (2 * eps)
        entry = dict(epsilon=eps, indices=indices, valid_by_horizon=valid.sum(0).tolist(),
                     sides=sides,
                     selected_autograd_vs_centered_secant=agreement(base_jvp.numpy(), centered, valid, rtol=.05),
                     warning="Centered secant is diagnostic only at boundary points; no unique Jacobian is claimed.")
        entries.append(entry)
        samples[f"{kind}_{eps:g}_reference"] = np.stack(refs)
        samples[f"{kind}_{eps:g}_one_sided_jvp"] = np.stack(side_jvp)
        print(f"{kind}, epsilon={eps:g}: completed; charged controls={budget.controls}", flush=True)


def event_probes(env, config, budget):
    geometry = env.geometry
    records = []
    for name in ("arrival_substep_2", "arrival_substep_5", "safety_substep_5", "initial_contact"):
        if name.startswith("arrival"):
            distance = .0425 if name.endswith("2") else .0505
            env.plant.reset(position=geometry["position_target"] + distance * geometry["normal"],
                            rotation=geometry["target_rotation"], velocity=geometry["velocity_target"])
        elif name.startswith("safety"):
            env.plant.reset(position=[1.3955, 0., 2.], velocity=[1., 0., 0.])
        else:
            env.plant.reset(position=[0., 0., .01])
        p = env.plant
        initial = np.r_[p.data.qpos, p.data.qvel, p.data.act]
        actions = np.tile([p.hover_force, 0., 0., 0.], (2, 1))
        _, steps, event = reference(env, initial, actions, budget)
        mask = event_mask(event, config["horizon_steps"])
        records.append(dict(name=name, completed_substeps=steps.tolist(), event=event,
                            event_mask_by_horizon=mask.tolist(), differentiability="NOT_APPLICABLE",
                            handling="Stop before any subsequent control; do not differentiate through reset or event identity."))
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    old = json.loads((args.audit / "summary.json").read_text())
    original = json.loads((args.audit / "inputs.json").read_text())
    config = old["config"]
    with np.load(args.audit / "samples.npz", allow_pickle=False) as saved:
        data = {key: saved[key] for key in saved.files}
    cases = json.loads((args.audit / "cases.json").read_text())
    # Conservative CHARGE, not a fabricated measurement of previous checks.
    reserve = dict(reference_controls=512, physics_substeps=4096, wall_seconds=30.)
    budget = Budget(config)
    budget.controls = old["budget"]["reference_controls"] + reserve["reference_controls"]
    budget.substeps = old["budget"]["physics_substeps"] + reserve["physics_substeps"]
    budget.started -= old["budget"]["wall_seconds"] + reserve["wall_seconds"]
    report = dict(status="NOT_RUN", prior_summary_sha256=sha(args.audit / "summary.json"),
                  policy_training="NOT_RUN", case_sources=cases, reserve=reserve,
                  research_targets=config["forward_p95_limits"],
                  progress=dict(completed_reference_variants=[]),
                  planned_probes=dict(indices=[32, 36, 41, 45, 64, 68, 73, 77],
                                      action_only_epsilon=[1e-5, 1e-6, 1e-7],
                                      state_only_epsilon=[1e-6], normalized_action_only_epsilon=[1e-6],
                                      fully_clipped_indices=[96, 97], fully_clipped_epsilon=[1e-5, 1e-6, 1e-7]))
    (args.output / "preregistered.json").write_text(json.dumps(report, indent=2) + "\n")
    samples, contents = {}, {}
    audit_hashes = {str(p): sha(p) for p in args.audit.iterdir() if p.is_file()}
    protected = original["protected_sha256"] | old["implementation_sha256"] | audit_hashes
    started = time.monotonic()
    env = None
    try:
        if any(sha(p) != value for p, value in protected.items()):
            raise RuntimeError("Original audit source or evidence changed")
        torch.set_num_threads(1)
        env = AgileTerminalArrivalEnv(ROOT / config["task_config"])
        env.reset(seed=config["seed"])
        params = DynamicsParameters.from_env(env)
        contents = freeze_sources(env, original, args.output)
        with torch.no_grad():
            report["stratified"], labels = stratify(data, cases, params, config, budget)
            samples["strata"] = labels
            # Same frozen reference; precision analysis costs no extra MuJoCo steps.
            single = surrogate(torch.tensor(data["initial"], dtype=torch.float32),
                               torch.tensor(data["actions"], dtype=torch.float32),
                               DynamicsParameters.from_env(env, dtype=torch.float32),
                               config["horizon_steps"], budget).numpy()
        report["original_event_counts"] = [sum(bool(e["arrived"] or e["reasons"]) for e in variant)
                                            for variant in old["windows"]["events"]]
        report["boundary_directional_sides"] = existing_directional_sides(data, labels, params, config, budget)
        report["float32_forward"] = {str(h * 5): {k: dict(p95=float(np.percentile(v[:, j], 95)), maximum=float(v[:, j].max()))
                                                   for k, v in errors(single, data["reference"][0]).items()}
                                     for j, h in enumerate(config["horizon_steps"])}
        samples["float32_predicted"] = single
        indices = report["planned_probes"]["indices"]
        for kind, epsilons, selected in (
            ("action_only", [1e-5, 1e-6, 1e-7], indices),
            ("state_only", [1e-6], indices),
            ("normalized_action_only", [1e-6], indices),
            ("fully_clipped_action_only", [1e-5, 1e-6, 1e-7], [96, 97]),
        ):
            probe(env, params, data, selected, kind, epsilons, config, budget, report, samples)
        report["events"] = event_probes(env, config, budget)
        budget.tick()
        report["status"] = "COMPLETE"
    except Exception as exc:
        report["error"] = repr(exc)
        report["status"] = "PARTIAL_BUDGET" if "budget exhausted" in str(exc) else "FAIL"
        if report["status"] == "FAIL":
            raise
    finally:
        if env is not None:
            env.close()
        unchanged = all(Path(p).exists() and sha(p) == value for p, value in protected.items())
        unchanged &= all(sha(p) == identity["sha256"] for p, identity in contents.items())
        report["inputs_unchanged"] = bool(unchanged)
        if not unchanged:
            report["status"] = "FAIL"
        report["budget"] = dict(charged_totals=budget.summary(), conservative_prior_check_reserve=reserve,
                                 measured_reference_controls=budget.controls - reserve["reference_controls"],
                                 measured_physics_substeps=budget.substeps - reserve["physics_substeps"],
                                 supplementary_wall_seconds=time.monotonic() - started,
                                 original_audit=old["budget"])
        if samples:
            np.savez_compressed(args.output / "samples.npz", **samples)
        (args.output / "analysis.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({key: report[key] for key in ("status", "inputs_unchanged", "budget")}), flush=True)
    if report["status"] == "FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
