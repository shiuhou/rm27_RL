"""Read-only native trajectory diagnostics for the instrumented Phase R2 comparison."""
from __future__ import annotations

from collections import Counter, defaultdict
from contextlib import contextmanager
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from . import terminal_native_eval as native
from .actuator_memory import region_features
from .asset import ROOT, digest, read_json, write_result


COLUMNS = ("target_x_m", "target_y_m", "target_normal_m", "inward_normal_speed_m_s", "alignment_deg")
VARIANTS = {"nominal", "minus", "plus"}
SCIENTIFIC_RULE = dict(
    transitions_per_arm=98_304, minimum_net_qualifications=1, minimum_positive_net_anchors=2,
    retention=dict(braking=dict(trials=36, minimum_qualified=32, max_exits=0),
                   terminal=dict(trials=30, minimum_qualified=27, max_exits=0)),
)


def transition_diagnostic(states, times, spec, region, outcome):
    """Measure descriptor entry at sampled native control boundaries before first touch."""
    features, inward = region_features(states, spec)
    inside = inward & np.all((features >= region["min"]) & (features <= region["max"]), axis=1)
    touch = outcome["contact"]["first_touch"]
    eligible = np.isfinite(features).all(axis=1)
    if touch is not None:
        eligible &= times < touch["interval_end_s"] - 1e-12
    if outcome["safety_failure"] is not None:
        eligible &= times < outcome["end_s"] - 1e-12

    initially_inside = bool(eligible[0] and inside[0])
    initially_outside = bool(eligible[0] and not inside[0])
    entries = np.flatnonzero(inside[1:] & eligible[1:]) + 1
    first = entries[0] if initially_outside and len(entries) else None
    return dict(initially_inside=initially_inside, initially_outside=initially_outside,
                initial_features=features[0].tolist() if np.isfinite(features[0]).all() else None,
                initial_moving_inward=bool(inward[0]), newly_entered=first is not None,
                first_entry_s=float(times[first]) if first is not None else None,
                entry_features=features[first].tolist() if first is not None else None)


@contextmanager
def native_transition_diagnostics(protocol_path, expected_sha256):
    """Annotate the frozen evaluator's own rollouts, including its forked workers."""
    if digest(protocol_path) != expected_sha256:
        raise ValueError("Frozen actuator-memory M2 protocol changed")
    protocol = read_json(protocol_path)
    region = protocol["region"]
    bounds = np.asarray([region["min"], region["max"]], dtype=np.float64)
    if (tuple(region["columns"]) != COLUMNS or bounds.shape != (2, len(COLUMNS))
            or not np.isfinite(bounds).all() or np.any(bounds[0] > bounds[1])):
        raise ValueError("Invalid frozen transition/braking descriptor")
    rollout = native.rollout

    def observe(plant, state, motors, targets, command, horizon, limits):
        if plant.metadata != protocol["model"] or limits != protocol["limits"]:
            raise ValueError("Native model or flight envelope differs from the frozen M2 protocol")
        result, trace = rollout(plant, state, motors, targets, command, horizon, limits)
        result["transition_region"] = transition_diagnostic(trace[0], trace[1], plant.spec, region, result)
        return result, trace

    with patch.object(native, "rollout", observe):
        yield


def _trial_counts(rows):
    metrics = [r["transition_region"] for r in rows]
    touches = [r for r in rows if r["contact"]["first_touch"] is not None]
    return dict(
        trials=len(rows), initially_inside=sum(d["initially_inside"] for d in metrics),
        initially_outside=sum(d["initially_outside"] for d in metrics),
        newly_entered=sum(d["newly_entered"] for d in metrics),
        entered_and_qualified=sum(d["newly_entered"] and r["qualified"] for d, r in zip(metrics, rows, strict=True)),
        qualified=sum(r["qualified"] for r in rows),
        signed_first_touches=len(touches), rejected_first_touches=sum(not r["qualified"] for r in touches),
        contact_rejection_reasons=dict(Counter(r["contact"]["reason"] for r in touches
                                             if not r["contact"]["qualified"])),
        safety_failures=dict(Counter(r["safety_failure"] for r in rows if r["safety_failure"] is not None)),
        saturation_controls=sum(r["saturation_controls"] for r in rows),
        reasons=dict(Counter(r["reason"] for r in rows)),
    )


def _bridge_anchors(rows):
    anchors = defaultdict(dict)
    for row in rows:
        key = row["case"], row["frame"]
        if row["variant"] in anchors[key]:
            raise ValueError(f"Repeated native evaluation trial: {key} {row['variant']}")
        anchors[key][row["variant"]] = row
    if len(anchors) != 12 or any(variants.keys() != VARIANTS for variants in anchors.values()):
        raise ValueError("Expected the complete 12-anchor, three-variant bridge suite")
    return anchors


def summarize_bridge_transition(rows):
    """Keep initial occupancy, autonomous entry, and native outcome denominators explicit."""
    anchors = _bridge_anchors(rows)
    return dict(**_trial_counts(rows), anchors=[
        dict(case=case, frame=frame, **_trial_counts(list(variants.values())))
        for (case, frame), variants in sorted(anchors.items())
    ])


def _retention(report):
    result = {}
    for phase, rule in SCIENTIFIC_RULE["retention"].items():
        total, minimum, exits = rule["trials"], rule["minimum_qualified"], rule["max_exits"]
        row = report["development"]["cohorts"][phase]
        if row["episodes"] != total:
            raise ValueError(f"Incomplete {phase} retention suite")
        result[phase] = dict(
            trials=total, qualified=row["qualified"], exits=row["safety_failures"], minimum=minimum,
            passed=row["qualified"] >= minimum and row["safety_failures"] <= exits,
        )
    return result


def _paired_outcome(row):
    touch = row["contact"]["first_touch"]
    return dict(qualified=row["qualified"], reason=row["reason"], safety_failure=row["safety_failure"],
                saturation_controls=row["saturation_controls"], native_reason=row["contact"]["reason"],
                first_touch=None if touch is None else {
                    k: touch[k] for k in ("sample_time_s", "groups", "aircraft_colliders", "target_colliders")
                }, transition_region=row["transition_region"])


def paired_bridge_readout(control_report, intervention_report):
    """Evaluate the frozen scientific learning signal independently of contact promotion."""
    control_rows, intervention_rows = (r["bridge"]["rows"] for r in (control_report, intervention_report))
    control, intervention = map(_bridge_anchors, (control_rows, intervention_rows))
    if control.keys() != intervention.keys():
        raise ValueError("Bridge comparison uses different reserved anchors")
    gains, losses, paired = [], [], []
    for case, frame in sorted(control):
        left, right = control[case, frame], intervention[case, frame]
        net = 0
        for variant in sorted(VARIANTS):
            a, b = left[variant], right[variant]
            if any(a[k] != b[k] for k in ("phase", "source_frame", "source_contact_lead_s", "horizon_s")):
                raise ValueError(f"Bridge trial metadata differs: {case}:{frame} {variant}")
            delta = int(b["qualified"]) - int(a["qualified"])
            net += delta
            if delta:
                (gains if delta > 0 else losses).append(dict(
                    case=case, frame=frame, variant=variant, control=_paired_outcome(a), intervention=_paired_outcome(b),
                ))
        paired.append(dict(case=case, frame=frame, net_qualification_delta=net,
                           control=_trial_counts(list(left.values())), intervention=_trial_counts(list(right.values()))))
    net = len(gains) - len(losses)
    positive_anchors = [dict(case=a["case"], frame=a["frame"]) for a in paired if a["net_qualification_delta"] > 0]
    retention = dict(control=_retention(control_report), intervention=_retention(intervention_report))
    retained = all(r["passed"] for r in retention["intervention"].values())
    checks = dict(net_qualifications=net >= SCIENTIFIC_RULE["minimum_net_qualifications"],
                  distinct_positive_anchors=len(positive_anchors) >= SCIENTIFIC_RULE["minimum_positive_net_anchors"],
                  retained=retained)
    return dict(
        trials_per_arm=len(control_rows), net_qualification_delta=net, gain_trials=gains, loss_trials=losses,
        positive_net_anchors=positive_anchors, retention=retention,
        scientific_checks=checks, scientific_learning_signal=all(checks.values()),
        control=_trial_counts(control_rows), intervention=_trial_counts(intervention_rows), anchors=paired,
    )


def _regional_availability(arm_dir, accounting, trained):
    from .rotation_bridge_metrics import SUMMARY_FIELDS
    from .terminal_curriculum import BANDS

    steps = dict.fromkeys(BANDS, 0)
    available = set()
    files = {}
    for path in sorted((arm_dir / "regional-rollouts").glob("rollout-*.npz")):
        with np.load(path, allow_pickle=False) as archive:
            summary = json.loads(archive["summary_json"].item())
            columns = set(archive.files)
        update = summary["updates"]
        regions = summary["regions"]
        if (not 0 < update <= accounting["updates"] or update in available or regions.keys() != steps.keys()
                or summary["completed_transitions"] > trained
                or not set(SUMMARY_FIELDS).issubset(columns)
                or not all(not row["steps"] or set(SUMMARY_FIELDS).issubset(row) for row in regions.values())):
            raise ValueError(f"Invalid regional PPO metrics file: {path}")
        available.add(update)
        files[str(path.relative_to(ROOT))] = digest(path)
        for band, row in regions.items():
            steps[band] += row["steps"]
    return dict(
        updates_expected=accounting["updates"], updates_recorded=len(available),
        missing_updates=sorted(set(range(1, accounting["updates"] + 1)) - available),
        per_band_transitions=steps, transitions_recorded=sum(steps.values()),
        complete=len(available) == accounting["updates"] and sum(steps.values()) == trained,
        fields=list(SUMMARY_FIELDS), files=files,
    )


def finalize_diagnostics(cfg, directory, comparison):
    """Publish the predeclared paired readout from independently evaluated endpoint files."""
    directory = Path(directory)
    endpoint = comparison["equal_budget_endpoint"]
    target = SCIENTIFIC_RULE["transitions_per_arm"]
    if cfg["budget"]["per_arm_transitions"] != target or not endpoint:
        raise ValueError("Scientific readout requires the authorized matched endpoint")
    reports, usage, sources = {}, {}, {}
    for arm in ("A", "B"):
        record = endpoint["arms"][arm]
        accounting = comparison["usage"][arm]["accounting"]
        trained = record["trained_transitions"]
        if (record["charged_transitions"] != target or trained != target
                or accounting["transitions"] != target or accounting["pending"] is not None):
            raise ValueError(f"Arm {arm} has not completed exactly {target:,} matched transitions")
        path = ROOT / record["evaluation"]
        if digest(path) != record["evaluation_sha256"]:
            raise ValueError(f"Arm {arm} endpoint evaluation changed")
        reports[arm] = read_json(path)
        if reports[arm]["checkpoint_sha256"] != record["checkpoint_sha256"]:
            raise ValueError(f"Arm {arm} endpoint actor changed")
        seconds = accounting["learning_s"]
        if not 0. < seconds < float("inf"):
            raise ValueError(f"Invalid measured learning time for arm {arm}")
        sources[arm] = dict(evaluation=record["evaluation"], evaluation_sha256=record["evaluation_sha256"],
                            checkpoint=record["checkpoint"], checkpoint_sha256=record["checkpoint_sha256"])
        usage[arm] = dict(trained_transitions=trained, learning_seconds=seconds,
                          learning_transitions_per_second=trained / seconds,
                          native_evaluation_wall_seconds=comparison["usage"][arm]["evaluation_wall_s"],
                          regional_metrics=_regional_availability(directory / arm, accounting, trained))
    experiment = cfg["diagnostics"]
    protocol = ROOT / experiment["transition_protocol"]
    if digest(protocol) != experiment["transition_protocol_sha256"]:
        raise ValueError("Frozen actuator-memory M2 protocol changed")
    result = dict(
        schema="rm27_asm2_rotation_bridge_diagnostics_v1",
        endpoint_transitions=target, scientific_rule=SCIENTIFIC_RULE,
        frozen_promotion=comparison["promotion"], source_arms=sources,
        strict_comparison_sha256=digest(directory / "bridge-comparison.json"),
        transition_descriptor=dict(protocol=experiment["transition_protocol"],
                                   sha256=experiment["transition_protocol_sha256"],
                                   **read_json(protocol)["region"]),
        usage=usage, readout=paired_bridge_readout(reports["A"], reports["B"]),
    )
    path = directory / "bridge-diagnostics.json"
    if path.exists():
        if read_json(path) != result:
            raise ValueError("Saved bridge diagnostics differ from endpoint evidence")
    else:
        write_result(path, result)
    return result
