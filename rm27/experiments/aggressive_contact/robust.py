"""V2: actuator observer, thrust adaptation, and margin-aware scenario selection."""
import copy
import json
from pathlib import Path
import time

import numpy as np
from scipy.stats import qmc

from .campaign import aggregate, paired_comparison
from .evidence import save_failure, snapshot, write_json
from .experiment import run_episode
from .model import ROOT, scenario


def settings():
    return json.loads((ROOT/"configs/aggressive_contact_v2.json").read_text())


def configured(cfg, kind):
    c = copy.deepcopy(cfg)
    if kind in ("actuator_only", "adaptive_nominal", "robust_v2"):
        c["controller"].update(settings()["controller"])
        if kind == "actuator_only":
            c["controller"]["adapt_specific_thrust"] = False
    return c


def robust_loss(row, margin):
    if "RUNNER_ERROR" in row["failure_modes"]:
        return 1e6
    first = row["first_contact"]
    if first is None:
        return 500+100*row["closest_bumper_center_m"]
    return (200*(not row["passed"]) + 80*(not row["valid_contact"])
            + 500*max(0., margin-first["region_margin_m"])
            + 80*first["position_error_m"] + 2*(row["recovery_time_s"] if row["recovery_time_s"] is not None else 4)
            + .15*row["max_post_body_rate_rad_s"] + .2*row["parameters"]["duration_s"])


def robust_campaign(cfg, output, candidates=None, trials=None):
    if candidates is not None and candidates < 2:
        raise ValueError("At least two candidates required")
    if trials is not None and trials < 1:
        raise ValueError("Positive evaluation count required")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    study = settings()
    write_json(output/"study.json", study)
    count, trials = candidates or study["candidate_count"], trials or study["evaluation_trials"]
    bounds = study["bounds"]
    pool = [dict(cfg["nominal_skill"]), dict(study["v1_parameters"])]
    for x in qmc.LatinHypercube(len(bounds), seed=study["search_seed"]).random(count-2):
        pool.append(dict(cfg["nominal_skill"], **{key:float(lo+(hi-lo)*a) for a, (key, (lo, hi)) in zip(x, bounds.items())}))
    write_json(output/"candidates.json", pool)
    started = time.monotonic()
    errors = []
    def execute(c, p, seed, scale, path):
        try:
            return run_episode(c, p, seed, scale, output=path)
        except Exception as exc:
            errors.append(str(path))
            return save_failure(path, exc)
    progress = (output/"progress.jsonl").open("w", buffering=1)
    try:
        costs = []
        c = configured(cfg, "robust_v2")
        c["target"]["in_plane_rotation_deg"] = 90.
        for i, p in enumerate(pool):
            rows = [execute(c, p, seed, scale, output/"training"/f"candidate-{i:03d}"/f"s{scale}-seed-{seed}")
                    for scale in study["training_scales"] for seed in study["training_seeds"]]
            losses = np.array([robust_loss(r, study["margin_objective_m"]) for r in rows])
            cost = float(losses.mean()+.5*np.sort(losses)[-max(1, len(losses)//4):].mean())
            costs.append(cost)
            progress.write(json.dumps(dict(stage="training", candidate=i, cost=cost, successes=sum(r["passed"] for r in rows)))+"\n")
        index = int(np.argmin(costs))
        methods = dict(nominal=cfg["nominal_skill"], v1=study["v1_parameters"], actuator_only=cfg["nominal_skill"],
                       adaptive_nominal=cfg["nominal_skill"], robust_v2=pool[index])
        # Freeze before reading any fresh evaluation result.
        write_json(output/"selection.json", dict(index=index, costs=costs, methods=methods))
        write_json(output/"selected_parameters.json", pool[index])
        write_json(output/"selected_config.json", c)
        evaluations, comparisons = {}, {}
        for geometry, angle in study["geometry_splits"].items():
            evaluations[geometry], comparisons[geometry] = {}, {}
            for split, scale, start in (("small", 1., study["evaluation_seed_start"]), ("stress", 2., study["stress_evaluation_seed_start"])):
                raw = {}
                evaluations[geometry][split], comparisons[geometry][split] = {}, {}
                for method, p in methods.items():
                    method_cfg = configured(cfg, method)
                    method_cfg["target"]["in_plane_rotation_deg"] = angle
                    rows = []
                    for seed in range(start, start+trials):
                        row = execute(method_cfg, p, seed, scale, output/"evaluation"/geometry/split/method/f"seed-{seed}")
                        rows.append(row)
                        progress.write(json.dumps(dict(stage="evaluation", geometry=geometry, split=split, method=method,
                                                       seed=seed, passed=row["passed"], failures=row["failure_modes"]))+"\n")
                    raw[method] = rows
                    stats = aggregate(rows)
                    margins = [r["first_contact"]["region_margin_m"] for r in rows if r.get("first_contact")]
                    stats["contact_margin_m"] = None if not margins else dict(min=float(min(margins)), p05=float(np.percentile(margins, 5)), median=float(np.median(margins)))
                    evaluations[geometry][split][method] = stats
                for method in ("nominal", "v1", "actuator_only", "adaptive_nominal"):
                    comparisons[geometry][split][method] = paired_comparison(raw[method], raw["robust_v2"])
        demo = execute(c, pool[index], None, 1, output/"demo")
        result = dict(passed=not errors and demo["passed"], passed_scope="runner integrity and selected nominal demo; see randomized counts",
                      evaluations=evaluations, comparisons=comparisons, selected_index=index, methods=methods,
                      training_episodes=count*len(study["training_seeds"])*len(study["training_scales"]),
                      evaluation_episodes=20*trials, runner_errors=errors, wall_time_s=time.monotonic()-started,
                      mode="HEADLESS_TRUTH_MOTOR_CONTROL", hardware_validated=False)
        write_json(output/"summary.json", result)
        return result
    finally:
        progress.close()


def causal_study(cfg, output, trials=32):
    """Paired interventions on historical v1 stress seeds, now development data."""
    if trials < 1:
        raise ValueError("Positive trial count required")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    p = settings()["v1_parameters"]
    conditions = ["unchanged", "nominal_lag", "nominal_thrust_mass", "nominal_target", "nominal_contact_geometry", "nominal_lag_thrust_mass"]
    results, raw = {}, {}
    for condition in conditions:
        rows = []
        for seed in range(202000, 202000+trials):
            draw = scenario(cfg, seed, 2.)
            if condition in ("nominal_lag", "nominal_lag_thrust_mass"):
                draw["draw"]["motor_tau_scale"] = 1.
            if condition in ("nominal_thrust_mass", "nominal_lag_thrust_mass"):
                draw["draw"]["thrust_scale"] = draw["draw"]["mass_scale"] = 1.
            if condition == "nominal_target":
                for key in ("center_m", "normal"):
                    draw["target"][key] = cfg["target"][key]
            if condition == "nominal_contact_geometry":
                draw["contact"]["offset_body_m"] = cfg["contact"]["offset_body_m"]
            rows.append(run_episode(cfg, p, seed, 2., physical=draw, output=output/condition/f"seed-{seed}"))
        raw[condition], results[condition] = rows, aggregate(rows)
    comparisons = {c:paired_comparison(raw["unchanged"], raw[c]) for c in conditions[1:]}
    result = dict(passed=True, passed_scope="paired intervention execution, not all flight trials", results=results,
                  comparisons=comparisons, trials_per_condition=trials,
                  scope="historical stress seeds used for mechanism analysis; not fresh held-out confirmation")
    write_json(output/"summary.json", result)
    return result


def validate_v2(cfg, source, output):
    """Numerical checks after selection; no tuning from these results."""
    output, source = Path(output), Path(source)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    selection = json.loads((source/"selection.json").read_text())
    rows = []
    for method in ("adaptive_nominal", "robust_v2"):
        for seed in (None, 301, 302):
            runs = []
            for dt in (.001, .0005, .00025):
                c = configured(cfg, method)
                c["physics_dt_s"] = dt
                c["target"]["in_plane_rotation_deg"] = 90.
                runs.append(run_episode(c, selection["methods"][method], seed, 2.,
                                        output=output/method/f"seed-{seed}-dt-{dt}"))
            comparisons = []
            for a, b in zip(runs[:-1], runs[1:]):
                impulse = abs(a["normal_impulse_Ns"]-b["normal_impulse_Ns"])/max(b["normal_impulse_Ns"], 1e-12)
                force = abs(a["peak_normal_force_N"]-b["peak_normal_force_N"])/max(b["peak_normal_force_N"], 1e-12)
                same = a["failure_modes"] == b["failure_modes"] and a["valid_contact"] == b["valid_contact"]
                comparisons.append(dict(impulse_relative_error=impulse, peak_force_relative_error=force,
                                        same_classification=same, passed=bool(impulse < .05 and force < .1 and same)))
            rows.append(dict(method=method, seed=seed, comparisons= comparisons,
                             failures=[r["failure_modes"] for r in runs]))
    result = dict(passed=all(c["passed"] for row in rows for c in row["comparisons"]),
                  passed_scope="both adjacent timestep checks; randomized reliability is separate",
                  timesteps_s=[.001, .0005, .00025], control_dt_s=cfg["control_dt_s"], rows=rows)
    write_json(output/"summary.json", result)
    return result
