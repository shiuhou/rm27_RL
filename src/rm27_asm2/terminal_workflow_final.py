"""Independent final comparison for the resumable ASM2 terminal trainer."""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from contextlib import nullcontext
import csv
import json
import multiprocessing
from pathlib import Path
import runpy
import sys
import time

import numpy as np

from . import terminal_native_eval as native
from .asset import ROOT, digest, read_json
from simpleflight_lab.stage1_training import write_json

CLOSED = {"TRANSITION_LIMIT", "TIME_LIMIT", "PERSISTENT_RETENTION_COLLAPSE"}


def freeze_selection(cfg, directory, frozen):
    """Record the development-only choices before opening the final case arrays."""
    selections = {}
    ledger = read_json(directory / "ledger.json")
    if ledger["config_sha256"] != frozen["config_sha256"]:
        raise ValueError("Final evaluation requires the frozen campaign ledger")
    for arm in ("A", "B"):
        path = directory / arm / "result.json"
        result = read_json(path)
        selected = result["selected"]
        if (result["stop_reason"] not in CLOSED or not selected
                or result["accounting"] != ledger["arms"][arm]
                or ledger["arms"][arm]["pending"] is not None):
            raise ValueError(f"Arm {arm} must finish learning and development selection first")
        if selected != max(result["evaluations"], key=lambda row: row["rank"]):
            raise ValueError(f"Arm {arm} does not match the development ranking")
        checkpoint = ROOT / selected["checkpoint"]
        weights = checkpoint.with_suffix(".npz")
        report = read_json(ROOT / selected["result"])
        provenance = dict(checkpoint_sha256=digest(checkpoint), weights_sha256=digest(weights))
        if any(report[key] != value for key, value in provenance.items()):
            raise ValueError(f"Arm {arm} development report does not match the selected actor")
        selections[arm] = dict(checkpoint=selected["checkpoint"], **provenance,
                               selected_at_transitions=selected["transitions"],
                               learning=result["accounting"],
                               result_sha256=digest(path), stop_reason=result["stop_reason"])
    parent = ROOT / cfg["parent"]
    selections["phase4"] = dict(checkpoint=cfg["parent"], checkpoint_sha256=digest(parent),
                                weights_sha256=digest(parent.with_suffix(".npz")))
    record = dict(freeze_sha256=digest(directory / "freeze.json"),
                  ledger_sha256=digest(directory / "ledger.json"), selections=selections)
    path = directory / "final-selection.json"
    if path.exists():
        if read_json(path) != record:
            raise ValueError("Selection changed after final evaluation was opened")
    else:
        write_json(path, record)
    return selections


def legacy_adapters(parent_dir):
    """Load the frozen Phase 4 evaluation helpers without changing the scorer."""
    sys.path.insert(0, str(parent_dir))
    try:
        old = runpy.run_path(str(parent_dir / "old_regression.py"))
        fresh = runpy.run_path(str(parent_dir / "fresh_evaluation.py"))
    finally:
        sys.path.pop(0)
    return old, fresh


def cohort(row):
    if row["population"] == "reached_handover":
        return "reached_handover"
    if row["phase"] == "handover":
        return "handover_snapshot"
    return row["phase"]


def summarize(rows, contact_metrics):
    regions = {}
    for row in rows:
        regions.setdefault(cohort(row), []).append(row)
    return {name: dict(**native.summary(group), **contact_metrics(group),
                       body_first=sum(any(p["group"] == "body" for p in
                                          (row["contact"]["first_touch"] or {}).get("records", ()))
                                      for row in group))
            for name, group in regions.items()}


def episode_identity(row):
    return (row["population"], row["case"], row["source_frame"], row["phase"], row["horizon_s"])


def score_suite(plant, controllers, populations, manifest, horizons, limits, workers,
                contact_metrics, *, replay_dir=None, goal=None):
    worker_args = (plant, controllers, horizons, limits, False)
    pool = (nullcontext() if workers == 1 else
            ProcessPoolExecutor(max_workers=workers, mp_context=multiprocessing.get_context("fork"),
                                initializer=native.init_worker, initargs=worker_args))
    results = {}
    if replay_dir:
        replay_dir.mkdir(exist_ok=True)
        mesh = native.target_mesh(plant, plant.spec.legal_targets)
    with pool as executor:
        for mode, (controller, provenance) in controllers.items():
            tasks = [(mode, population, data, index,
                      manifest["cases"][str(data["case_ids"][index])].get("first_contact_sample_s"))
                     for population, data, indices in populations for index in indices]
            outputs = (executor.map(native.worker_episode, tasks, chunksize=2) if executor else
                       (native.run_episode(*worker_args, task) for task in tasks))
            rows, replays = [], {}
            for task, (row, _) in zip(tasks, outputs, strict=True):
                rows.append(row)
                if replay_dir is None or mode not in ("A", "B") or not row["controls"]:
                    continue
                status = "qualified" if row["qualified"] else "failed"
                key = f"{cohort(row)}-{status}"
                if key in replays:
                    continue
                _, _, data, index, _ = task
                replay, trace = native.rollout(
                    plant, data["states"][index], data["motor_thrust_N"][index],
                    data["motor_target_N"][index], controller, row["horizon_s"], limits)
                if any(replay[k] != row[k] for k in ("qualified", "reason", "end_s", "contact")):
                    raise ValueError("Native replay did not reproduce the scored episode")
                path = replay_dir / f"{mode}-{key}.npz"
                label = f"{mode}-{row['case']}-{row['source_frame']}"
                if path.exists():
                    with np.load(path, allow_pickle=False) as previous:
                        if (previous["names"].tolist() != [label]
                                or previous["contact_outcome"].item() != row["reason"]
                                or not np.array_equal(previous["states"][:, 0], trace[0])
                                or not np.array_equal(previous["sample_times_s"], trace[1])
                                or not np.array_equal(previous["ctbr_N_rad_s"][:, 0], trace[2])):
                            raise ValueError(f"Saved replay differs from the selected episode: {path}")
                else:
                    native.save_replay(path, label, trace, replay, goal, mesh, plant.profile.policy_dt_s)
                replays[key] = dict(path=str(path.relative_to(ROOT)), sha256=digest(path),
                                    case=row["case"], source_frame=row["source_frame"])
            cohorts = summarize(rows, contact_metrics)
            results[mode] = dict(actor=provenance, cohorts=cohorts, episodes=rows, replays=replays)
            print("WORKFLOW_FINAL", mode, {k: (v["qualified"], v["episodes"])
                                          for k, v in cohorts.items()}, flush=True)
    return results


def archived_controls(path, plant, controllers, episodes, contact_metrics, required_metadata):
    report = read_json(path)
    if report["model"] != plant.metadata or any(report.get(k) != v for k, v in required_metadata.items()):
        raise ValueError(f"Historical model or suite differs: {path}")
    expected = [episode_identity(row) for row in episodes]
    reused = {}
    for name, mode in (("phase4", "candidate"), ("track", "track"), ("trim", "trim"),
                       ("deterministic", "deterministic")):
        source = report["results"][mode]
        provenance = source["actor"]
        expected_controller = controllers[name][1]
        if name == "deterministic" and "fresh_manifest_sha256" in required_metadata:
            # The historical braking evaluator recorded the map and source hashes.
            # Its manifest separately pins the module implementing the clip.
            expected_controller = {k: v for k, v in expected_controller.items()
                                   if not k.startswith("action_map_")}
        if any(provenance.get(k) != v for k, v in expected_controller.items()):
            raise ValueError(f"Historical controller differs: {path} {mode}")
        if [episode_identity(row) for row in source["episodes"]] != expected:
            raise ValueError(f"Historical trial order or horizon differs: {path} {mode}")
        reused[name] = dict(actor=provenance, cohorts=summarize(source["episodes"], contact_metrics),
                            reused_from=dict(path=str(path.relative_to(ROOT)), sha256=digest(path),
                                             controller=mode))
    return reused


def evaluate_final(cfg, directory):
    from .terminal_workflow import verify_freeze

    started = time.monotonic()
    frozen = verify_freeze(cfg, directory)
    selected = freeze_selection(cfg, directory, frozen)
    development = development_curves(directory)
    selection_sha = digest(directory / "final-selection.json")
    parent_dir = (ROOT / cfg["parent"]).parents[1]
    old, fresh = legacy_adapters(parent_dir)
    task, profile, spec, manifest_path, manifest, goal = native.load_evaluation_inputs(ROOT / cfg["task_config"])
    limits = read_json(native.HANDOVER_CONFIG)["flight_envelope"]
    plant = native.NativeContact(profile, spec)
    horizons = dict(terminal=task["horizon_s"], braking=task["braking_horizon_s"],
                    handover=task["handover_horizon_s"])
    actors = {name: native.load_controller((ROOT / pin["checkpoint"]).with_suffix(".npz"),
                                           pin["checkpoint_sha256"], profile, goal)
              for name, pin in selected.items()}
    for name, (_, provenance) in actors.items():
        if provenance["weights_sha256"] != selected[name]["weights_sha256"]:
            raise ValueError(f"Portable actor changed: {name}")
    track = native.flight_settings()["ver2_track_checkpoint"]
    if digest(ROOT / track["path"]) != track["sha256"]:
        raise ValueError("Frozen Track checkpoint changed")
    actors["track"] = native.load_controller(native.TRACK_WEIGHTS, track["sha256"], profile, goal)
    trim = np.array([profile.hover_thrust_N, 0., 0., 0.])
    actors["trim"] = (lambda _: trim, dict(ctbr_N_rad_s=trim.tolist()))
    source = manifest_path.parent
    workers = cfg["workers"]

    def evaluate_suite(name, populations, historical=None, metadata=None):
        output = directory / f"native-{name}.json"
        if output.exists():
            report = read_json(output)
            if report["selection_sha256"] != selection_sha:
                raise ValueError(f"Cannot reuse final result with a different selection: {name}")
            return report
        suite_start = time.monotonic()
        # Preserve the source-reference tick and the capped live-state command.
        deterministic = old["load_action_limited_deterministic"](source, manifest, populations, profile, spec)
        controls = dict(actors, deterministic=deterministic)
        if historical:
            run = {arm: actors[arm] for arm in ("A", "B")}
            replay_dir = None
        else:
            run = controls
            replay_dir = directory / "final-replays"
        results = score_suite(plant, run, populations, manifest, horizons, limits, workers,
                              fresh["contact_metrics"], replay_dir=replay_dir, goal=goal)
        if historical:
            results.update(archived_controls(historical, plant, controls, results["A"]["episodes"],
                                             fresh["contact_metrics"], metadata))
        report = dict(suite=name, selection_sha256=selection_sha, model=plant.metadata,
                      scorer_sha256=spec.contract_sha256, evaluator_sha256=digest(native.__file__),
                      horizons_s=horizons, results=results, training_transitions=0,
                      wall_s=time.monotonic() - suite_start)
        if name == "final":
            report.update(reset_archive_sha256=frozen["archives"]["final.npz"],
                          source_overlap=frozen["final_source_overlap"],
                          shared_trajectories=frozen["final_shared_trajectories"])
        write_json(output, report)
        return report

    with np.load(directory / "final.npz", allow_pickle=False) as data:
        final = dict(data)
    groups = [(str(region), final, np.flatnonzero(final["region"] == region))
              for region in dict.fromkeys(final["region"])]
    final_report = evaluate_suite("final", groups)
    heldout = native.load_starts(source, manifest["splits"]["heldout"], expected_count=105, profile=profile)
    reached = native.load_starts(source, manifest["heldout_handover"], expected_count=4, profile=profile)
    old_report = evaluate_suite(
        "historical-regression", [("heldout", heldout, range(105)), ("reached_handover", reached, range(4))],
        parent_dir / "native-old-regression.json",
        dict(config_sha256=digest(ROOT / cfg["task_config"]), reset_manifest_sha256=digest(manifest_path),
             evaluator_sha256=digest(native.__file__)))
    fresh_manifest = read_json(parent_dir / "fresh_manifest.json")
    for path, expected in fresh_manifest["source_pins"].items():
        if digest(ROOT / path) != expected:
            raise ValueError(f"Historical braking source changed: {path}")
    braking = native.load_starts(parent_dir, fresh_manifest["archive"], expected_count=18, profile=profile)
    # This report used the same capped reference before the wrapper recorded its own hashes.
    fresh_report = evaluate_suite(
        "historical-braking", [("fresh", braking, range(18))], parent_dir / "native-fresh.json",
        dict(fresh_manifest_sha256=digest(parent_dir / "fresh_manifest.json")))
    reports = [final_report, old_report, fresh_report]
    result = dict(selection_sha256=selection_sha, development=development,
                  suites={report["suite"]: {mode: data["cohorts"] for mode, data in report["results"].items()}
                          for report in reports},
                  final_replays={arm: final_report["results"][arm]["replays"] for arm in ("A", "B")},
                  evaluator_wall_s=sum(report["wall_s"] for report in reports),
                  this_invocation_wall_s=time.monotonic() - started)
    write_json(directory / "native-comparison.json", result)
    return result


def development_curves(directory):
    """Compare the checkpoints actually evaluated at the same learned transition count."""
    curves, usage = {}, {}
    for arm in ("A", "B"):
        folder = directory / arm
        result = read_json(folder / "result.json")
        with (folder / "learning.jsonl").open() as stream:
            logged = {row["transitions"]: row for row in map(json.loads, stream)}
        curves[arm] = []
        for entry in result["evaluations"]:
            charged = entry["transitions"]
            measured = logged.get(charged, {})
            evaluation = ROOT / entry["result"]
            raw_path = evaluation.with_name(f"{evaluation.stem}-development.json")
            dev = read_json(raw_path)["results"]["candidate"]
            cohorts = entry["cohorts"]
            curves[arm].append(dict(
                arm=arm, charged_transitions=charged,
                trained_transitions=entry.get("completed_transitions",
                                              measured.get("completed_transitions", 0 if charged == 0 else None)),
                learning_s=entry.get("learning_s", measured.get("learning_s", 0. if charged == 0 else None)),
                selected=entry == result["selected"], sampler_stage=entry["sampler_stage"],
                early_qualified=dev["rotation_order"]["early"]["qualified"],
                middle_qualified=dev["rotation_order"]["middle"]["qualified"],
                late_qualified=dev["rotation_order"]["late"]["qualified"],
                rotation_qualified=cohorts["rotation"]["qualified"],
                rotation_exits=cohorts["rotation"]["safety_failures"],
                braking_qualified=cohorts["braking"]["qualified"],
                braking_exits=cohorts["braking"]["safety_failures"],
                terminal_qualified=cohorts["terminal"]["qualified"],
                terminal_exits=cohorts["terminal"]["safety_failures"],
            ))
        latest = logged[max(logged)] if logged else None
        usage[arm] = dict(accounting=result["accounting"],
                          completed_transitions=result["completed_transitions"],
                          evaluation_wall_s=result["evaluation_wall_s"], restart_count=result["restart_count"],
                          last_learning_log=latest,
                          effective_transitions_per_s=(
                              result["completed_transitions"] / result["accounting"]["learning_s"]
                              if result["accounting"]["learning_s"] else None))
        if latest:
            usage[arm]["optimization"] = {
                name: dict(zip(("median", "p95", "max"), map(float, np.quantile(values, (.5, .95, 1.)))))
                for name, values in {
                    "collect_s": [row["collect_s"] for row in logged.values()],
                    "ppo_update_s": [row["update_s"] for row in logged.values()],
                    "approx_kl": [row["loss"][4] for row in logged.values()],
                }.items()
            }
    csv_path = directory / "development-curves.csv"
    rows = curves["A"] + curves["B"]
    with csv_path.with_suffix(".tmp").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    csv_path.with_suffix(".tmp").replace(csv_path)
    indexed = {arm: {row["trained_transitions"]: row for row in series
                     if row["trained_transitions"] is not None} for arm, series in curves.items()}
    common = [dict(trained_transitions=count, A=indexed["A"][count], B=indexed["B"][count])
              for count in sorted(indexed["A"].keys() & indexed["B"].keys())]
    return dict(csv=str(csv_path.relative_to(ROOT)), per_arm=curves, matched_milestones=common,
                usage=usage, denominators=dict(rotation=36, per_rotation_band=12, braking=36, terminal=30))


def plot_development(directory):
    """Render the saved native development curve when matplotlib is available."""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.ticker import MaxNLocator

    with (directory / "development-curves.csv").open(newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if row["trained_transitions"]]
    fields = (
        ("rotation_qualified", "Rotation qualified (of 36)", 36),
        ("rotation_exits", "Rotation envelope exits (of 36)", 36),
        ("braking_qualified", "Braking qualified (of 36)", 36),
        ("terminal_qualified", "Terminal qualified (of 30)", 30),
    )
    fig, axes = plt.subplots(2, 2, figsize=(10, 6.4), sharex=True, layout="constrained")
    for ax, (field, title, limit) in zip(axes.flat, fields):
        for arm, label, color in (("A", "Fixed", "#1665a2"), ("B", "Adaptive", "#c55716")):
            samples = [row for row in rows if row["arm"] == arm]
            x = [int(row["trained_transitions"]) / 1000 for row in samples]
            y = [int(row[field]) for row in samples]
            ax.plot(x, y, "o-", color=color, label=label)
            chosen = [i for i, row in enumerate(samples) if row["selected"] == "True"]
            ax.scatter([x[i] for i in chosen], [y[i] for i in chosen], facecolor="white",
                       edgecolor=color, s=90, marker="D", linewidth=1.6, zorder=3)
        ax.set(title=title, ylim=(-1, limit + 1))
        ax.grid(alpha=.25)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=6, integer=True))
    axes[0, 0].legend(frameon=False)
    fig.supxlabel("Completed training transitions (thousands)")
    fig.suptitle("Independent MuJoCo development scores · diamonds mark selected checkpoints")
    destination = directory / "development-curves.png"
    fig.savefig(destination, dpi=180)
    plt.close(fig)
    return destination
