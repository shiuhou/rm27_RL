"""Summarize Phase 4 braking continuation from recorded training and MuJoCo results."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from hashlib import sha256
import json
from pathlib import Path
import re
from statistics import median

RUN = Path(__file__).resolve().parent
PREVIOUS = RUN.parent / "phase4r2"
CONTROLS = RUN.parent / "phase4r" / "native-dev-controls.json"
POINT_METRICS = (
    "body_to_target_alignment_deg",
    "tangential_speed_m_s",
    "normal_approach_speed_m_s",
)
BRAKING_BANDS = ((.1, "under_100_ms"), (.25, "100_to_250_ms"), (float("inf"), "250_ms_or_more"))
TERMINAL_BANDS = ((.005, "under_5_ms"), (.025, "5_to_25_ms"), (float("inf"), "25_ms_or_more"))
CONTROLLER_LABELS = {
    "candidate": "PPO",
    "track": "Frozen Track",
    "trim": "Constant commands",
    "action_limited_deterministic": "Action-limited deterministic",
}


def read(path):
    return json.loads(path.read_bytes())


def source(path):
    return {"path": str(path), "sha256": sha256(path.read_bytes()).hexdigest()}


def trial_id(row):
    return row["population"], row["phase"], row["case"], row["source_frame"]


def indexed(rows):
    by_id = {trial_id(row): row for row in rows}
    if len(by_id) != len(rows):
        raise ValueError("Repeated evaluation reset")
    return by_id


def first_touch(row):
    return row.get("contact", {}).get("first_touch") or row.get("first_touch")


def point_stats(manifolds, *, body_only=False):
    """Give each episode equal weight using its worst first-touch point."""
    values = defaultdict(list)
    for manifold in manifolds:
        points = manifold.get("records", ())
        if body_only:
            points = [point for point in points if point["group"] == "body"]
        for metric in POINT_METRICS:
            present = [point[metric] for point in points if metric in point]
            if present:
                values[metric].append(max(present))
    return {
        metric: {
            "episodes": len(samples),
            "median": median(samples),
            "min": min(samples),
            "max": max(samples),
        }
        for metric, samples in values.items()
    }


def summarize(rows):
    outcomes = Counter(row["reason"] for row in rows)
    touched = [(row, manifold) for row in rows if (manifold := first_touch(row)) is not None]
    bodies = [manifold for _, manifold in touched if "body" in manifold["groups"]]
    forbidden = sum(
        "body" in touch["groups"]
        or any(point.get("target_registered") is False for point in touch.get("records", ()))
        for _, touch in touched
    )
    return {
        "episodes": len(rows),
        "qualified": sum(row["qualified"] for row in rows),
        "flight_envelope_exits": sum(row["safety_failure"] is not None for row in rows),
        "mixer_saturation_controls": sum(row["saturation_controls"] for row in rows),
        "first_touch_episodes": len(touched),
        "first_touch_body_episodes": len(bodies),
        "first_touch_forbidden_episodes": forbidden,
        "forbidden_airframe_first_touch": outcomes["FORBIDDEN_AIRFRAME_FIRST_TOUCH"],
        "forbidden_target_first_touch": outcomes["FORBIDDEN_TARGET_FIRST_TOUCH"],
        "outcomes": dict(sorted(outcomes.items())),
        "first_touch_groups": dict(sorted(Counter("+".join(t["groups"]) for _, t in touched).items())),
        "first_touch_worst_point": point_stats([t for _, t in touched]),
        "qualified_first_touch_worst_point": point_stats([t for r, t in touched if r["qualified"]]),
        "body_first_touch_worst_body_point": point_stats(bodies, body_only=True),
    }


def grouped_rows(rows):
    """Keep different experiment populations apart, including repeated handover resets."""
    populations = defaultdict(list)
    for row in rows:
        populations[row["population"], row["phase"]].append(row)
    return populations


def lead_bands(phase, rows):
    bins = defaultdict(list)
    bands = BRAKING_BANDS if phase == "braking" else TERMINAL_BANDS
    for row in rows:
        lead = row.get("source_contact_lead_s")
        label = "unknown_source_lead" if lead is None else next(
            name for upper, name in bands if lead < upper
        )
        bins[label].append(row)
    return bins


def by_cohort(rows):
    cohorts = {}
    for (population, phase), cases in sorted(grouped_rows(rows).items()):
        record = summarize(cases)
        if phase in ("braking", "terminal"):
            record["lead_bands"] = {
                name: summarize(group) for name, group in lead_bands(phase, cases).items()
            }
        cohorts.setdefault(population, {})[phase] = record
    return cohorts


def paired(candidate, baseline):
    candidate_by_id, baseline_by_id = indexed(candidate), indexed(baseline)
    if candidate_by_id.keys() != baseline_by_id.keys():
        raise ValueError("Candidate and comparison policies used different resets")
    both = candidate_only = baseline_only = neither = 0
    for key, row in candidate_by_id.items():
        result = row["qualified"], baseline_by_id[key]["qualified"]
        if result == (True, True):
            both += 1
        elif result == (True, False):
            candidate_only += 1
        elif result == (False, True):
            baseline_only += 1
        else:
            neither += 1
    return {
        "both_qualify": both, "ppo_only": candidate_only,
        "control_only": baseline_only, "neither_qualifies": neither,
    }


def paired_by_cohort(candidate, baseline):
    candidate_groups, baseline_groups = grouped_rows(candidate), grouped_rows(baseline)
    if candidate_groups.keys() != baseline_groups.keys():
        raise ValueError("Candidate and comparison policies used different cohorts")
    result = {}
    for (population, phase), rows in sorted(candidate_groups.items()):
        controls = baseline_groups[population, phase]
        metrics = paired(rows, controls)
        if phase in ("braking", "terminal"):
            comparator_bands = lead_bands(phase, controls)
            metrics["lead_bands"] = {
                name: paired(group, comparator_bands[name])
                for name, group in lead_bands(phase, rows).items()
            }
        result.setdefault(population, {})[phase] = metrics
    return result


def archived_controls():
    results = read(CONTROLS)["results"]
    by_case_frame = {(r["case"], r["source_frame"]): r for r in results["track"]["episodes"]}
    authority = read(PREVIOUS / "action-authority.json")["results"]["current"]
    action_limited = []
    for row in authority["episodes"]:
        archived = by_case_frame[row["case"], row["source_frame"]]
        action_limited.append(dict(row, population=archived["population"], phase=archived["phase"],
                                   source_contact_lead_s=archived["source_contact_lead_s"]))
    return {
        "track": results["track"]["episodes"],
        "trim": results["trim"]["episodes"],
        "action_limited_deterministic": action_limited,
    }


def development_evaluations(learning, controls):
    previous = PREVIOUS / "learning" / "development-result.json"
    reports = {0: previous}
    for path in sorted(learning.glob("eval-*.json")):
        report = read(path)
        transitions = report.get("additional_transitions")
        if transitions is None:
            match = re.fullmatch(r"eval-(\d+)", path.stem)
            if match is None:
                raise ValueError(f"Expected eval-<additional-transitions>.json or metadata in {path}")
            transitions = int(match.group(1))
        if transitions in reports:
            if transitions != 0 or reports[0] != previous:
                raise ValueError(f"Duplicate development checkpoint at {transitions}")
        reports[transitions] = path
    evaluations = []
    for transitions, path in sorted(reports.items()):
        result = read(path)["results"]["candidate"]
        rows = result["episodes"]
        if len(rows) != 36 or any(row["phase"] != "braking" for row in rows):
            raise ValueError(f"Expected the frozen 36-start development suite in {path}")
        grouped = by_cohort(rows)
        (braking,) = (phases["braking"] for phases in grouped.values())
        evaluations.append({
            "additional_transitions": transitions,
            "source": source(path),
            "actor": result["actor"],
            "braking": braking,
            "paired_controls": {
                name: paired_by_cohort(rows, control)["recovery-development"]["braking"]
                for name, control in controls.items()
            },
        })
    return evaluations


def final_evaluation(path):
    report = read(path)
    groups = {name: result["episodes"] for name, result in report["results"].items()}
    return {
        "source": source(path),
        "candidate": report["results"]["candidate"]["actor"],
        "cohorts": {name: by_cohort(rows) for name, rows in groups.items()},
        "paired_controls": {
            name: paired_by_cohort(groups["candidate"], rows)
            for name, rows in groups.items() if name != "candidate"
        },
    }


def training_log(learning):
    path = learning / "learning.jsonl"
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def plot(metrics, destination):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.ticker import FuncFormatter

    evaluations = metrics["development"]
    controls = metrics["development_controls"]
    x = [row["additional_transitions"] for row in evaluations]
    evaluations = [row["braking"] for row in evaluations]
    logs = metrics["training_log"]
    rows = 3 if logs else 2
    figure, axes = plt.subplots(rows, 2, figsize=(10.5, 3.4 * rows + .4), layout="constrained")
    figure.suptitle("ASM2 braking continuation · independent MuJoCo development suite",
                    x=.025, ha="left", fontsize=15, weight="bold")
    qualified, lead, failures, speeds = axes.flat[:4]
    palette = ["#167D87", "#A95114", "#58448E"]
    qualified.plot(x, [row["qualified"] for row in evaluations], color=palette[0],
                   marker="o", linewidth=2.5, label="PPO")
    qualified.axhline(15, color="#404954", linestyle="--", label="Promotion threshold")
    for (name, summary), color in zip(controls.items(), ["#A95114", "#58448E", "#547944"]):
        qualified.axhline(summary["qualified"], color=color, linestyle=":", label=CONTROLLER_LABELS[name])
    qualified.set(title="Qualified braking starts", ylabel="Starts of 36", ylim=(0, 38))
    qualified.legend(loc="center right", fontsize=7.5, frameon=False)
    for (name, label), color in zip(
            [("under_100_ms", "<100 ms"), ("100_to_250_ms", "100–<250 ms"),
             ("250_ms_or_more", "≥250 ms")], palette):
        bins = [row.get("lead_bands", {}).get(name, {}) for row in evaluations]
        rates = [100 * b["qualified"] / b["episodes"] if b.get("episodes") else float("nan") for b in bins]
        lead.plot(x, rates, marker="o", color=color, label=label)
    lead.set(title="Qualified by witness lead time", ylabel="Qualified (%)", ylim=(-3, 105))
    lead.legend(loc="best", fontsize=8, frameon=False)
    for key, label, color in (
        ("forbidden_airframe_first_touch", "Forbidden airframe first touch", palette[1]),
        ("forbidden_target_first_touch", "Forbidden target first touch", palette[2]),
        ("flight_envelope_exits", "Flight-envelope exit", palette[0]),
    ):
        failures.plot(x, [row[key] for row in evaluations], marker="o", color=color, label=label)
    failures.set(title="Development failures", ylabel="Starts of 36", ylim=(-1, 37))
    failures.legend(loc="best", fontsize=8, frameon=False)
    for key, label, color in (
        ("tangential_speed_m_s", "Tangential speed", palette[1]),
        ("normal_approach_speed_m_s", "Normal approach speed", palette[0]),
    ):
        speeds.plot(x, [row["body_first_touch_worst_body_point"].get(key, {}).get("median", float("nan"))
                        for row in evaluations], marker="o", color=color, label=label)
    speeds.set(title="Pre-contact velocity in body-first episodes", ylabel="Median worst body point (m/s)")
    speeds.legend(loc="best", fontsize=8, frameon=False)
    logged_frames = [row["transitions"] for row in logs]
    if logs:
        reward, ppo = axes[2]
        reward.plot(logged_frames, [row["mean_reward"] for row in logs], color=palette[0], linewidth=1.7)
        reward.set(title="Isaac rollout reward", ylabel="Mean reward per control step")
        kl_line, = ppo.plot(logged_frames, [row["loss"][4] for row in logs], color=palette[0],
                            linewidth=1.7, label="Approximate KL")
        ppo.set(title="PPO updates and exploration", ylabel="Approximate KL")
        if all("action_std" in row for row in logs):
            scale_axis = ppo.twinx()
            std_line, = scale_axis.plot(
                logged_frames, [sum(row["action_std"]) / len(row["action_std"]) for row in logs],
                color=palette[1], linewidth=1.5, label="Action standard deviation",
            )
            scale_axis.set_ylabel("Mean Gaussian standard deviation")
            scale_axis.spines["top"].set_visible(False)
            ppo.legend(handles=[kl_line, std_line], loc="best", fontsize=8, frameon=False)
    limit = max(x + logged_frames)
    for ax in axes.flat:
        ax.set_xlabel("Additional training transitions")
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v / 1000:g}k" if v else "0"))
        ax.set_xlim(-.03 * limit if limit else -1, 1.03 * limit if limit else 1)
        ax.grid(axis="y", color="#e6e9eb", linewidth=.8)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)
    with destination.open("xb") as f:
        figure.savefig(f, dpi=180, format="png")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--learning", type=Path, default=RUN / "learning")
    parser.add_argument("--old-report", type=Path, help="Selected actor on the archived regression suite")
    parser.add_argument("--fresh-report", type=Path, help="Selected actor on the preregistered fresh suite")
    parser.add_argument("--metrics", type=Path, default=RUN / "analyze_metrics.json")
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()
    if args.metrics.exists() or (args.plot and args.plot.exists()):
        parser.error("Choose unused analysis outputs")
    controls = archived_controls()
    evaluations = development_evaluations(args.learning, controls)
    if len(evaluations) < 2:
        parser.error("At least one continuation checkpoint is required")
    metrics = {
        "schema": "asm2_braking_continuation_analysis_v1",
        "development_controls": {
            name: by_cohort(rows)["recovery-development"]["braking"]
            for name, rows in controls.items()
        },
        "control_sources": [source(CONTROLS), source(PREVIOUS / "action-authority.json")],
        "development": evaluations,
        "training_log": training_log(args.learning),
    }
    result = args.learning / "result.json"
    if result.is_file():
        metrics["training_result"] = read(result)
    for key, report in (("old_regression", args.old_report), ("fresh_generalization", args.fresh_report)):
        if report:
            final = final_evaluation(report)
            if final["candidate"]["checkpoint_sha256"] not in {
                row["actor"]["checkpoint_sha256"] for row in evaluations
            }:
                raise ValueError("Final report actor does not match an evaluated development checkpoint")
            metrics[key] = final
    if args.old_report and args.fresh_report and (
            metrics["old_regression"]["candidate"]["checkpoint_sha256"]
            != metrics["fresh_generalization"]["candidate"]["checkpoint_sha256"]):
        raise ValueError("Final reports use different checkpoints")
    with args.metrics.open("x") as f:
        json.dump(metrics, f, indent=2, allow_nan=False)
        f.write("\n")
    if args.plot:
        plot(metrics, args.plot)
    for record in evaluations:
        braking = record["braking"]
        print(f'{record["additional_transitions"]:>7} transitions  '
              f'{braking["qualified"]}/{braking["episodes"]} qualified  '
              f'{braking["forbidden_airframe_first_touch"]} forbidden airframe  '
              f'{braking["flight_envelope_exits"]} envelope exits')


if __name__ == "__main__":
    main()
