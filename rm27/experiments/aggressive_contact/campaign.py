"""Paired scenario optimization: one search pool, two declared objectives.

Evaluation seeds never influence selection. Training failures stay in the loss
and every held-out trial stays in denominators. This is derivative-free policy
parameter search, not RL, a viability certificate, or a real distribution.
"""
from collections import Counter
import json
from pathlib import Path
import time

import numpy as np
from scipy.stats import qmc

from .evidence import save_failure, snapshot, write_json
from .experiment import objective, run_episode


def wilson(k, n):
    z = 1.95996398454
    mid = (k/n+z*z/(2*n))/(1+z*z/n)
    half = z*np.sqrt(k/n*(1-k/n)/n+z*z/(4*n*n))/(1+z*z/n)
    return [float(mid-half), float(mid+half)]


def aggregate(rows):
    n, k = len(rows), sum(r["passed"] for r in rows)
    def quantiles(field):
        values = [r[field] for r in rows if r.get(field) is not None]
        return None if not values else dict(n=len(values), median=float(np.median(values)),
                                           p95=float(np.percentile(values, 95)), max=float(max(values)))
    contact = [r["first_contact"] for r in rows if r.get("first_contact")]
    return dict(trials=n, successes=k, success_rate=k/n, wilson_95=wilson(k, n),
                valid_contacts=sum(r.get("valid_contact", False) for r in rows),
                failures=dict(Counter(f for r in rows for f in r["failure_modes"])),
                recovery_time_s=quantiles("recovery_time_s"),
                max_post_body_rate_rad_s=quantiles("max_post_body_rate_rad_s"),
                altitude_loss_m=quantiles("altitude_loss_m"),
                saturation_fraction=quantiles("saturation_fraction"),
                contact_normal_speed_m_s=None if not contact else [float(min(r["normal_speed_m_s"] for r in contact)),
                                                                   float(max(r["normal_speed_m_s"] for r in contact))],
                contact_error_m_median=None if not contact else float(np.median([r["position_error_m"] for r in contact])))


def paired_comparison(a, b, seed=7301):
    delta = np.array([int(y["passed"])-int(x["passed"]) for x, y in zip(a, b, strict=True)])
    rng = np.random.default_rng(seed)
    boot = delta[rng.integers(0, len(delta), size=(10000, len(delta)))].mean(axis=1)
    return dict(success_rate_difference=float(delta.mean()), paired_bootstrap_95=np.quantile(boot, [.025, .975]).tolist(),
                improved=int(sum(delta > 0)), regressed=int(sum(delta < 0)), tied=int(sum(delta == 0)))


def candidate_pool(cfg, count):
    if count < 2:
        raise ValueError("At least two candidates required")
    bounds = cfg["search"]["bounds"]
    points = qmc.LatinHypercube(d=len(bounds), seed=cfg["search"]["seed"]).random(count-1)
    pool = [dict(cfg["nominal_skill"])]
    for point in points:
        p = dict(cfg["nominal_skill"])
        for x, (name, (lo, hi)) in zip(point, bounds.items()):
            p[name] = float(lo+x*(hi-lo))
        pool.append(p)
    return pool


def campaign(cfg, output, candidates=32, trials=32):
    if trials < 1:
        raise ValueError("Positive evaluation count required")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    start = time.monotonic()
    pool = candidate_pool(cfg, candidates)
    write_json(output/"candidates.json", pool)
    all_train, losses = [], []
    def execute(path, p, seed, scale=1):
        try:
            return run_episode(cfg, p, seed, scale=scale, output=path)
        except Exception as exc:
            return save_failure(path, exc)
    with (output/"progress.jsonl").open("w", buffering=1) as progress:
        for i, p in enumerate(pool):
            rows = [execute(output/"training"/f"candidate-{i:03d}"/f"seed-{seed}", p, seed)
                    for seed in cfg["search"]["training_seeds"]]
            all_train.append(rows)
            scores = []
            for aware in (False, True):
                values = [objective(r, aware) if "RUNNER_ERROR" not in r["failure_modes"] else 1e6 for r in rows]
                scores.append(float(np.mean(values)+.25*np.max(values)))
            losses.append(scores)
            progress.write(json.dumps(dict(stage="training", candidate=i, scores=scores,
                                            successes=sum(r["passed"] for r in rows)))+"\n")
        selected = np.argmin(np.array(losses), axis=0).tolist()
        methods = dict(nominal=pool[0], contact_only=pool[selected[0]], recovery_aware=pool[selected[1]])
        write_json(output/"selection.json", dict(selected_indices=selected, losses=losses, methods=methods,
                                               selection="mean training loss + 0.25 worst training loss; common candidate pool"))
        # Freeze this artifact before evaluating any test seed.
        write_json(output/"selected_parameters.json", methods["recovery_aware"])
        evaluations, comparisons = {}, {}
        for name, scale, offset in (("small_uncertainty", 1., 0), ("stress_2x", 2., 10000)):
            evaluations[name] = {}
            raw = {}
            for method, p in methods.items():
                rows = []
                for j in range(trials):
                    seed = cfg["search"]["evaluation_seed_start"] + offset + j
                    r = execute(output/"evaluation"/name/method/f"seed-{seed}", p, seed, scale)
                    rows.append(r)
                    progress.write(json.dumps(dict(stage="evaluation", split=name, method=method,
                                                   seed=seed, passed=r["passed"], failures=r["failure_modes"]))+"\n")
                raw[method] = rows
                evaluations[name][method] = aggregate(rows)
            comparisons[name] = {name: paired_comparison(raw[name], raw["recovery_aware"])
                                 for name in ("nominal", "contact_only")}
        nominal_demo = execute(output/"demo", methods["recovery_aware"], None)
        errors = sum("RUNNER_ERROR" in r["failure_modes"] for rows in all_train for r in rows)
        errors += sum(stats["failures"].get("RUNNER_ERROR", 0) for group in evaluations.values() for stats in group.values())
        result = dict(passed=bool(nominal_demo["passed"] and errors == 0),
                      passed_scope="research harness completed without runner errors and selected nominal demo passed; NOT all randomized trials",
                      candidate_count=candidates, train_episode_count=candidates*len(cfg["search"]["training_seeds"]),
                      evaluation_episode_count=6*trials, selected_indices=selected, methods=methods,
                      evaluations=evaluations, comparisons=comparisons, runner_errors=errors,
                      wall_time_s=time.monotonic()-start,
                      mode="HEADLESS_MUJOCO_CLOSED_LOOP_MOTOR_CONTROL_TRUTH_TEACHER",
                      hardware_validated=False, sitl_validated=False, s2_passed=False)
        write_json(output/"summary.json", result)
    return result
