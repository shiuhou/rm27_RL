"""Summarize the frozen Phase R2 development comparison and export paired replays."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
from statistics import median

import numpy as np

from . import terminal_native_eval as native
from .asset import ROOT, digest, read_json
from .terminal_workflow_final import CLOSED
from simpleflight_lab.stage1_training import write_json

ARMS = ("A", "B")
DEVELOPMENT_COUNTS = {"rotation": 36, "braking": 36, "terminal": 30}
VARIANTS = {"nominal", "minus", "plus"}


def bridge_key(row):
    frame = row["source_frame"] if "source_frame" in row else row["frame"]
    return str(row["case"]), int(frame), str(row["variant"])


def summarize(rows):
    """Keep rejection reasons, timeouts and envelope exits distinct."""
    first_touch = [r for r in rows if r["contact"]["first_touch"] is not None]
    contact_failures = [r for r in first_touch if not r["qualified"] and not r["safety_failure"]]
    return dict(
        episodes=len(rows),
        qualified=sum(r["qualified"] for r in rows),
        reasons=dict(Counter(r["reason"] for r in rows)),
        exits=sum(bool(r["safety_failure"]) for r in rows),
        envelope_exit_reasons=dict(Counter(r["safety_failure"] for r in rows if r["safety_failure"])),
        timeouts=sum(r["reason"] == "NO_SIGNED_TOUCH" and not r["safety_failure"] for r in rows),
        first_touches=len(first_touch),
        body_first=sum("body" in r["contact"]["first_touch"]["groups"] for r in first_touch),
        contact_failure_reasons=dict(Counter(r["reason"] for r in contact_failures)),
        saturated_rollouts=sum(r["saturation_controls"] > 0 for r in rows),
        saturation_controls=sum(r["saturation_controls"] for r in rows),
        controls=sum(r["controls"] for r in rows),
    )


def bridge_counts(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[bridge_key(row)[:2]].append(row)
    if len(groups) != 12 or any(len(group) != 3 or {r["variant"] for r in group} != VARIANTS
                                for group in groups.values()):
        raise ValueError("Bridge gate requires all three variants at exactly 12 distinct anchors")
    return {anchor: sum(r["qualified"] for r in group) for anchor, group in groups.items()}


def score_report(path, checkpoint, expected):
    report = read_json(path)
    weights = checkpoint.with_suffix(".npz")
    if (report["checkpoint_sha256"] != digest(checkpoint)
            or report["weights_sha256"] != digest(weights)):
        raise ValueError(f"Evaluation checkpoint provenance mismatch: {path}")
    bridge = report["bridge"]["rows"]
    bridge_keys = [bridge_key(r) for r in bridge]
    if len(bridge_keys) != len(expected) or set(bridge_keys) != expected:
        raise ValueError(f"Bridge trials differ from frozen archive: {path}")
    counts = bridge_counts(bridge)
    summary = summarize(bridge)
    summary.update(anchors=len(counts), robust=sum(n == 3 for n in counts.values()),
                   partial=sum(0 < n < 3 for n in counts.values()),
                   failing=sum(n == 0 for n in counts.values()),
                   anchor_variant_successes=[sum(n == i for n in counts.values()) for i in range(4)])
    if any(report["bridge"][k] != summary[k] for k in ("robust", "qualified", "episodes", "exits")):
        raise ValueError(f"Incorrect bridge aggregate in {path}")
    regions = {"bridge": summary}
    other = report["development"]["episodes"]
    for name, count in DEVELOPMENT_COUNTS.items():
        rows = [r for r in other if r["phase"] == name]
        identities = {(r["case"], r["source_frame"], r["horizon_s"]) for r in rows}
        if len(rows) != count or len(identities) != count:
            raise ValueError(f"Incomplete {name} development suite: {path}")
        regions[name] = summarize(rows)
    if len(other) != sum(DEVELOPMENT_COUNTS.values()):
        raise ValueError(f"Unexpected development rows in {path}")
    return regions, report


def paired_bridge(a_rows, b_rows):
    a = {bridge_key(r): r for r in a_rows}
    b = {bridge_key(r): r for r in b_rows}
    if len(a) != len(a_rows) or len(b) != len(b_rows) or a.keys() != b.keys():
        raise ValueError("Expected exact paired bridge evaluations")
    if any((a[k]["horizon_s"], a[k]["lead_s"]) != (b[k]["horizon_s"], b[k]["lead_s"]) for k in a):
        raise ValueError("Bridge lead or horizon changed between arms")
    a_counts, b_counts = bridge_counts(a_rows), bridge_counts(b_rows)
    groups = Counter((a[k]["qualified"], b[k]["qualified"]) for k in a)
    anchor_rows = [dict(case=case, frame=frame, A_variants=a_counts[case, frame],
                        B_variants=b_counts[case, frame]) for case, frame in sorted(a_counts)]
    outcome_pairs = Counter((a[k]["reason"], b[k]["reason"]) for k in a)
    both_failed = [k for k in a if not (a[k]["qualified"] or b[k]["qualified"])]
    return dict(
        episodes=len(a),
        both_qualified=groups[True, True],
        gains=groups[False, True],
        losses=groups[True, False],
        neither_qualified=groups[False, False],
        robust_gains=sum(a_counts[k] < 3 and b_counts[k] == 3 for k in a_counts),
        robust_losses=sum(a_counts[k] == 3 and b_counts[k] < 3 for k in a_counts),
        exit_to_nonexit_among_both_failed=sum(bool(a[k]["safety_failure"]) and not b[k]["safety_failure"]
                                              for k in both_failed),
        nonexit_to_exit_among_both_failed=sum(not a[k]["safety_failure"] and bool(b[k]["safety_failure"])
                                              for k in both_failed),
        new_first_touch_among_both_failed=sum(a[k]["contact"]["first_touch"] is None
                                             and b[k]["contact"]["first_touch"] is not None for k in both_failed),
        median_end_time_delta_s_among_both_failed=(
            median(b[k]["end_s"] - a[k]["end_s"] for k in both_failed) if both_failed else None),
        outcome_pairs=[dict(A=ra, B=rb, episodes=n) for (ra, rb), n in sorted(outcome_pairs.items())],
        anchors=anchor_rows,
    )


def gate_decision(limits, scores):
    a, b = scores["A"], scores["B"]
    delta = b["bridge"]["robust"] - a["bridge"]["robust"]
    checks = dict(
        robust_anchors=b["bridge"]["robust"] >= limits["robust_anchors"],
        anchor_delta=delta >= limits["anchor_delta"],
        bridge_exits=b["bridge"]["exits"] <= limits["max_exits"],
        braking_retention=b["braking"]["qualified"] >= limits["braking_qualified"],
        braking_exits=b["braking"]["exits"] <= limits["max_exits"],
        terminal_retention=b["terminal"]["qualified"] >= limits["terminal_qualified"],
        terminal_exits=b["terminal"]["exits"] <= limits["max_exits"],
    )
    return dict(decision="GO" if all(checks.values()) else "NO_GO", anchor_delta=delta, checks=checks)


def make_replays(cfg, directory, suite, comparison, reports):
    """Replay up to two matched cases and require exact native outcome parity."""
    _, profile, spec, _, _, goal = native.load_evaluation_inputs(ROOT / cfg["task_config"])
    limits = read_json(native.HANDOVER_CONFIG)["flight_envelope"]
    plant = native.NativeContact(profile, spec)
    mesh = native.target_mesh(plant, spec.legal_targets)
    a, b = ({bridge_key(r): r for r in reports[arm]["bridge"]["rows"]} for arm in ARMS)
    candidates = sorted(a, key=lambda k: (k[2] != "nominal", k))
    gains = [k for k in candidates if b[k]["qualified"] and not a[k]["qualified"]]
    losses = [k for k in candidates if a[k]["qualified"] and not b[k]["qualified"]]
    exit_changes = [k for k in candidates if bool(a[k]["safety_failure"]) != bool(b[k]["safety_failure"])]
    categories = (gains, losses, exit_changes, [k for k in candidates if b[k]["qualified"]],
                  [k for k in candidates if not b[k]["qualified"]])
    picked = []
    for group in categories:
        option = next((k for k in group if k[:2] not in {p[:2] for p in picked}), None)
        if option is not None:
            picked.append(option)
        if len(picked) == 2:
            break
    suite_indices = {(str(case), int(frame), str(variant)): i for i, (case, frame, variant) in enumerate(
        zip(suite["case_ids"], suite["source_frame"], suite["variant"], strict=True))}
    replays = {}
    replay_dir = directory / "final-replays"
    replay_dir.mkdir(exist_ok=True)
    for arm, lookup in (("A", a), ("B", b)):
        checkpoint = ROOT / comparison["arms"][arm]["checkpoint"]
        command, _ = native.load_controller(checkpoint.with_suffix(".npz"), digest(checkpoint), profile, goal)
        for key in picked:
            row = lookup[key]
            i = suite_indices[key]
            replay, trace = native.rollout(plant, suite["states"][i], suite["motor_thrust_N"][i],
                                          suite["motor_target_N"][i], command, row["horizon_s"], limits)
            fields = ("qualified", "reason", "safety_failure", "saturation_controls", "end_s", "contact")
            if any(replay[field] != row[field] for field in fields):
                raise ValueError(f"Native replay did not reproduce {arm} {key}")
            case, frame, variant = key
            label = f"{arm}-{case}-{frame}-{variant}"
            path = replay_dir / f"{label}.npz"
            if path.exists():
                with np.load(path, allow_pickle=False) as saved:
                    states, times, commands = trace
                    if (saved["names"].tolist() != [label]
                            or saved["contact_outcome"].item() != replay["reason"]
                            or not np.array_equal(saved["states"][:, 0], states)
                            or not np.array_equal(saved["sample_times_s"], times)
                            or not np.array_equal(saved["ctbr_N_rad_s"][:, 0], commands)):
                        raise ValueError(f"Existing replay differs from native trace: {path}")
            else:
                native.save_replay(path, label, trace, replay, goal, mesh, profile.policy_dt_s)
            replays[label] = dict(path=str(path.relative_to(ROOT)), sha256=digest(path),
                                  arm=arm, case=case, frame=frame, variant=variant,
                                  reason=replay["reason"], qualified=replay["qualified"])
    return replays


def verify_initial_actors(parent, records):
    with np.load(parent.with_suffix(".npz"), allow_pickle=False) as source:
        tensors = set(source.files) - {"checkpoint_sha256"}
        if not tensors or source["checkpoint_sha256"].item() != digest(parent):
            raise ValueError("Frozen parent portable actor has invalid checkpoint provenance")
        for arm, record in records.items():
            checkpoint = ROOT / record[0]["checkpoint"]
            with np.load(checkpoint.with_suffix(".npz"), allow_pickle=False) as actor:
                if (set(actor.files) - {"checkpoint_sha256"} != tensors
                        or actor["checkpoint_sha256"].item() != digest(checkpoint)
                        or any(not np.array_equal(actor[name], source[name]) for name in tensors)):
                    raise ValueError(f"Arm {arm} did not start with the frozen parent actor")
    return len(tensors)


def write_curves(directory, by_arm):
    fields = ("bridge_robust", "bridge_qualified", "bridge_partial", "bridge_exits",
              "rotation_qualified", "rotation_exits", "braking_qualified", "braking_exits",
              "terminal_qualified", "terminal_exits")
    rows = []
    for arm in ARMS:
        for entry, _, summary, measured, selected in by_arm[arm]:
            row = dict(arm=arm, charged_transitions=entry["transitions"],
                       trained_transitions=measured["completed_transitions"],
                       learning_s=measured["learning_s"], selected=selected)
            row.update({field: summary[region][key] for field in fields
                        for region, key in [field.split("_", 1)]})
            rows.append(row)
    csv_path = directory / "development-curves.csv"
    with csv_path.with_suffix(".tmp").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    csv_path.with_suffix(".tmp").replace(csv_path)
    return csv_path


def finalize(cfg, directory):
    """Use the latest common positive checkpoint as the predeclared promotion comparison."""
    from .terminal_workflow import verify_freeze

    directory = Path(directory)
    frozen = verify_freeze(cfg, directory)
    ledger_path = directory / "ledger.json"
    ledger = read_json(ledger_path)
    if ledger["config_sha256"] != frozen["config_sha256"]:
        raise ValueError("Campaign ledger does not match frozen config")
    output = directory / "bridge-comparison.json"
    source_paths = [directory / arm / "result.json" for arm in ARMS]
    source_hashes = dict(freeze_sha256=digest(directory / "freeze.json"), ledger_sha256=digest(ledger_path),
                         arms={arm: digest(path) for arm, path in zip(ARMS, source_paths, strict=True)})
    if output.exists():
        result = read_json(output)
        if result["sources"] != source_hashes:
            raise ValueError("Saved bridge comparison has different source hashes")
        for replay in result["replays"].values():
            if digest(ROOT / replay["path"]) != replay["sha256"]:
                raise ValueError("Saved bridge replay failed provenance check")
        return result

    with np.load(directory / "bridge-validation.npz", allow_pickle=False) as archive:
        suite = dict(archive)
    expected = {(str(case), int(frame), str(variant)) for case, frame, variant in
                zip(suite["case_ids"], suite["source_frame"], suite["variant"], strict=True)}
    if len(expected) != len(suite["states"]) or len(expected) != 36:
        raise ValueError("Invalid frozen bridge-validation denominator")
    by_arm, usage, selected = {}, {}, {}
    for arm, path in zip(ARMS, source_paths, strict=True):
        result = read_json(path)
        accounting = ledger["arms"][arm]
        history = result["evaluations"]
        if (result["stop_reason"] not in CLOSED or result["accounting"] != accounting
                or accounting["pending"] is not None or not history
                or result["selected"] != max(history, key=lambda h: h["rank"])):
            raise ValueError(f"Arm {arm} has no closed, development-selected result")
        with (directory / arm / "learning.jsonl").open() as stream:
            logged = {row["transitions"]: row for row in map(json.loads, stream)}
        records = []
        for entry in history:
            checkpoint, report_path = (ROOT / entry[key] for key in ("checkpoint", "result"))
            regions, report = score_report(report_path, checkpoint, expected)
            frames = entry["transitions"]
            initial = dict(completed_transitions=0, learning_s=0.)
            measured = logged.get(frames, initial if frames == 0 else entry)
            if "completed_transitions" not in measured or "learning_s" not in measured:
                raise ValueError(f"Missing completed transition/time record for {arm} at {frames}")
            record = (entry, report, regions, measured, entry == result["selected"])
            records.append(record)
            if record[-1]:
                selected[arm] = record
        by_arm[arm] = records
        usage[arm] = dict(
            accounting=accounting,
            completed_transitions=result["completed_transitions"],
            evaluation_wall_s=result["evaluation_wall_s"],
            restart_count=result["restart_count"],
            stop_reason=result["stop_reason"],
            last_learning_log=logged[max(logged)] if logged else None,
        )

    def compare(records):
        arms = {}
        for arm, (entry, report, scores, measured, is_selected) in records.items():
            arms[arm] = dict(checkpoint=entry["checkpoint"], checkpoint_sha256=report["checkpoint_sha256"],
                             evaluation=entry["result"], evaluation_sha256=digest(ROOT / entry["result"]),
                             charged_transitions=entry["transitions"],
                             trained_transitions=measured["completed_transitions"],
                             learning_s=measured["learning_s"], selected=is_selected, scores=scores)
        a, b = (records[arm][1] for arm in ARMS)
        a_dev, b_dev = (report["development"]["episodes"] for report in (a, b))
        identities = lambda rows: {(r["case"], r["source_frame"], r["phase"], r["horizon_s"]) for r in rows}
        if identities(a_dev) != identities(b_dev):
            raise ValueError("Development retention trials are not matched across arms")
        return dict(arms=arms, paired_bridge=paired_bridge(a["bridge"]["rows"], b["bridge"]["rows"]))

    maps = {arm: {item[3]["completed_transitions"]: item for item in records}
            for arm, records in by_arm.items()}
    initial_records = {arm: maps[arm][0] for arm in ARMS}
    initial = compare(initial_records)
    initial["actor_tensors_exact_to_frozen_parent"] = verify_initial_actors(ROOT / cfg["parent"], initial_records)
    common = sorted(maps["A"].keys() & maps["B"].keys())
    positive = [n for n in common if n > 0]
    equal_records = {arm: maps[arm][positive[-1]] for arm in ARMS} if positive else None
    matched = compare(equal_records) if equal_records else None
    selected_pair = compare(selected)
    gate = cfg["bridge"]["gate"]
    promotion = (gate_decision(gate, {arm: matched["arms"][arm]["scores"] for arm in ARMS}) if matched else
                 dict(decision="NO_GO", reason="NO_COMMON_POSITIVE_EVALUATION"))
    curves = write_curves(directory, by_arm)
    replay_records = equal_records if matched else selected
    replays = make_replays(cfg, directory, suite, matched or selected_pair,
                           {arm: replay_records[arm][1] for arm in ARMS})
    result = dict(
        schema="rm27_asm2_rotation_bridge_report_v1",
        sources=source_hashes,
        gate=gate, promotion=promotion,
        promotion_basis="latest common positive completed transition milestone",
        denominators=dict(anchors=12, variants_per_anchor=3, bridge=36, **DEVELOPMENT_COUNTS),
        common_evaluated_transitions=common,
        equal_budget_endpoint=matched,
        selected_development_pair=selected_pair,
        initialization=initial,
        usage=usage, learning_curves=str(curves.relative_to(ROOT)), replays=replays,
    )
    write_json(output, result)
    print(f"R2 {promotion['decision']} · {output}", flush=True)
    return result


def plot(directory):
    """Render the frozen development curves in a Python environment with matplotlib."""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.ticker import MaxNLocator

    with (directory / "development-curves.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    gate = read_json(directory / "bridge-comparison.json")["gate"]
    fields = (
        ("bridge_robust", "Bridge anchors with all variants qualified", 12, gate["robust_anchors"]),
        ("bridge_exits", "Bridge envelope exits", 36, gate["max_exits"]),
        ("braking_qualified", "Braking qualifications", 36, gate["braking_qualified"]),
        ("terminal_qualified", "Terminal qualifications", 30, gate["terminal_qualified"]),
    )
    figure, axes = plt.subplots(2, 2, figsize=(10, 6.4), sharex=True, layout="constrained")
    for ax, (field, title, limit, target) in zip(axes.flat, fields, strict=True):
        for arm, label, color in (("A", "Control", "#1665a2"), ("B", "Bridge", "#c55716")):
            samples = [r for r in rows if r["arm"] == arm]
            x = [int(r["trained_transitions"]) / 1000 for r in samples]
            y = [int(r[field]) for r in samples]
            ax.plot(x, y, "o-", color=color, label=label)
            chosen = [i for i, row in enumerate(samples) if row["selected"] == "True"]
            ax.scatter([x[i] for i in chosen], [y[i] for i in chosen], marker="D", s=65,
                       facecolor="white", edgecolor=color, linewidth=1.6, zorder=3)
        ax.axhline(target, color="#737f8a", linestyle="--", linewidth=.8)
        ax.set(title=f"{title} (of {limit})", ylim=(-1, limit + 1))
        ax.grid(alpha=.25)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=6, integer=True))
    axes[0, 0].legend(frameon=False)
    figure.supxlabel("Completed training transitions (thousands)")
    figure.suptitle("Phase R2 native development evaluation · diamonds mark development selection")
    destination = directory / "development-curves.png"
    figure.savefig(destination, dpi=180)
    plt.close(figure)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot the finalized Phase R2 comparison.")
    parser.add_argument("directory", type=Path)
    print(plot(parser.parse_args().directory))
