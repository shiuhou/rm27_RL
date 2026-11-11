"""Freeze and score unused Phase 3 braking frames with the native scorer."""
from argparse import ArgumentParser
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
import multiprocessing
from pathlib import Path
import time

import numpy as np

from rm27_asm2.handover_curriculum import _source_trace, _split_arrays, native_verifier
from rm27_asm2.terminal_native_eval import (
    HANDOVER_CONFIG, ROOT, TRACK_WEIGHTS, NativeContact, digest, flight_settings,
    init_worker, load_controller, load_deterministic, load_evaluation_inputs,
    load_starts, read_json, save_replay, summary, target_mesh,
    worker_episode, write_result,
)

RUN = Path(__file__).resolve().parent
CONFIG = ROOT / "configs/rm27_asm2_terminal_recovery.json"
ARCHIVE = RUN / "fresh_resets.npz"
MANIFEST = RUN / "fresh_manifest.json"
LEAD_BANDS_S = ((0.0, 0.1), (0.1, 0.25), (0.25, float("inf")))
LEAD_NAMES = ("under_100_ms", "100_to_250_ms", "250_ms_or_more")
LEAD_TARGETS_S = ((.060, .085), (.140, .205), (.280, .325))
PRIOR_EVALUATIONS = tuple(RUN.parent / phase / "native-heldout.json" for phase in ("phase4-pilot", "phase4r2"))


def source_pins(manifest_path, manifest, names):
    source = [manifest_path, CONFIG, HANDOVER_CONFIG, Path(__file__), TRACK_WEIGHTS, *PRIOR_EVALUATIONS]
    source.append(ROOT / flight_settings()["ver2_track_checkpoint"]["path"])
    source.append(manifest_path.parent / manifest["splits"]["heldout"]["path"])
    source.append(manifest_path.parent / manifest["heldout_handover"]["path"])
    for name in names:
        for kind in ("trace", "report"):
            pin = manifest["cases"][name][kind]
            path = (manifest_path.parent / pin["path"]).resolve()
            if digest(path) != pin["sha256"]:
                raise ValueError(f"Frozen Phase 3 {kind} changed for {name}")
            source.append(path)
    source.extend(ROOT / "src/rm27_asm2" / name for name in (
        "terminal_native_eval.py", "handover_curriculum.py", "contact_native.py", "contact_task.py",
    ))
    return {str(path.resolve().relative_to(ROOT)): digest(path) for path in source}


def freeze():
    if ARCHIVE.exists() or MANIFEST.exists():
        raise FileExistsError("Fresh suite already frozen")
    cfg, profile, spec, manifest_path, manifest, _ = load_evaluation_inputs(CONFIG)
    directory = manifest_path.parent
    heldout = load_starts(directory, manifest["splits"]["heldout"], expected_count=105, profile=profile)
    handovers = load_starts(directory, manifest["heldout_handover"], expected_count=4, profile=profile)
    names = sorted(manifest["splits"]["heldout"]["reset_cases"])
    if len(names) != 3 or any(name in manifest["splits"]["development"]["cases"] for name in names):
        raise ValueError("Expected three development-independent held-out source trajectories")
    pins = source_pins(manifest_path, manifest, names)
    previous = {(row["case"], row["source_frame"]) for path in PRIOR_EVALUATIONS
                for controller in read_json(path)["results"].values() for row in controller["episodes"]}
    verify = native_verifier(NativeContact(profile, spec))
    rows, cohorts = [], []
    for name in names:
        source = manifest["cases"][name]
        touch = source["first_contact_sample_s"]
        if not source["qualified"] or source["split"] != "heldout":
            raise ValueError(f"Expected a qualifying held-out witness for {name}")
        trace = directory / source["trace"]["path"]
        states, motors, targets, times = _source_trace(trace.read_bytes(), source["handover_frame"], touch, profile)
        braking = np.sort(heldout["source_frame"][(heldout["case_ids"] == name) & (heldout["phase"] == "braking")])
        used = set(heldout["source_frame"][heldout["case_ids"] == name])
        used.update(handovers["source_frame"][handovers["case_ids"] == name])
        used.update(frame for case, frame in previous if case == name)
        interior = [(frame, int(left), int(right)) for left, right in zip(braking, braking[1:])
                    for frame in range(left+1, right)]
        for band, (lo, hi), desired in zip(LEAD_NAMES, LEAD_BANDS_S, LEAD_TARGETS_S):
            for target_lead in desired:
                options = [(frame, left, right) for frame, left, right in interior
                           if frame not in used and lo <= touch-times[frame] < hi]
                frame, left, right = min(options, key=lambda item: (abs(touch-times[item[0]]-target_lead), item[0]))
                used.add(frame)
                exact = verify(states[frame], motors[frame], targets[frame])
                if not exact["passed"]:
                    raise ValueError(f"Native restore failed for {name}:{frame}: {exact}")
                rows.append((states[frame], motors[frame], targets[frame], times[frame], frame, "braking", name))
                cohorts.append(dict(case=name, source_frame=frame, bracket=[left, right], lead_band=band,
                                    source_contact_lead_s=float(touch-times[frame]), restore=exact))
    with ARCHIVE.open("xb") as stream:
        np.savez_compressed(stream, **_split_arrays(rows, profile.policy_dt_s))
    write_result(MANIFEST, dict(
        schema="asm2_phase4_meaningful_fresh_braking_v1", frozen_utc=datetime.now(timezone.utc).isoformat(),
        purpose="Final generalization to previously unevaluated braking snapshots of held-out Phase 3 witnesses",
        selection="Unused interior frames between adjacent archived braking frames nearest to fixed lead targets. "
                  "Selection uses trace metadata, prior trial identifiers, and native reset validity only.",
        lead_targets_s=LEAD_TARGETS_S, source_trajectories=names, rows=cohorts,
        archive=dict(path=ARCHIVE.name, sha256=digest(ARCHIVE)), source_pins=pins,
        horizon_s=cfg["braking_horizon_s"], control_dt_s=profile.policy_dt_s, native_dt_s=spec.dt,
        contact_contract_sha256=spec.contract_sha256, profile_sha256=profile.profile_sha256,
        matched_controls=["frozen_track", "constant_hover_trim", "phase3_feedback_clipped_to_policy_action_map"],
        deterministic_action_map=dict(rate_abs_rad_s=[2, 2, 2], thrust_hover_range=[.5, 1.5]),
    ))
    print(f"FROZEN {len(rows)} {MANIFEST} {digest(MANIFEST)}", flush=True)


class ClippedReference:
    def __init__(self, ref, hover):
        self.ref = ref
        self.lower = np.array([.5*hover, -2., -2., -2.])
        self.upper = np.array([1.5*hover, 2., 2., 2.])

    def command(self, state, tick):
        return np.clip(self.ref.command(state, tick), self.lower, self.upper)


def contact_metrics(rows):
    fields = (
        "body_to_target_alignment_deg",
        "normal_approach_speed_m_s",
        "tangential_speed_m_s",
    )
    per_trial = {field: [] for field in fields}
    forbidden = 0
    touched = 0
    for row in rows:
        event = row["contact"]["first_touch"]
        if event is None:
            continue
        touched += 1
        forbidden += any(p["group"] == "body" or not p["target_registered"] for p in event["records"])
        for field in fields:
            per_trial[field].append(max(p[field] for p in event["records"]))
    return dict(
        first_contacts=touched, forbidden_first_contacts=forbidden,
        median_worst_first_contact={field: float(np.median(values)) if values else None
                                    for field, values in per_trial.items()},
    )


def evaluate(args):
    started = time.monotonic()
    cfg, profile, spec, manifest_path, manifest, goal = load_evaluation_inputs(CONFIG)
    frozen = read_json(MANIFEST)
    if frozen["schema"] != "asm2_phase4_meaningful_fresh_braking_v1" or len(frozen["rows"]) != 18:
        raise ValueError("Expected frozen 18-row fresh suite")
    for name, expected in frozen["source_pins"].items():
        if digest(ROOT / name) != expected:
            raise ValueError(f"Frozen fresh-suite source changed: {name}")
    data = load_starts(RUN, frozen["archive"], expected_count=18, profile=profile)
    indices = range(len(data["states"]))
    reference, provenance = load_deterministic(
        manifest_path.parent, manifest, [("fresh", data, indices)], profile, spec,
    )
    authority = {name: (ClippedReference(ref, profile.hover_thrust_N), gate)
                 for name, (ref, gate) in reference.items()}
    track_sha = flight_settings()["ver2_track_checkpoint"]["sha256"]
    trim = np.array([profile.hover_thrust_N, 0., 0., 0.])
    controllers = {
        "candidate": load_controller(args.actor, digest(args.checkpoint), profile, goal),
        "track": load_controller(TRACK_WEIGHTS, track_sha, profile, goal),
        "trim": (lambda _: trim, {"ctbr_N_rad_s": trim.tolist()}),
        "deterministic": (authority, dict(provenance, action_map=frozen["deterministic_action_map"])),
    }
    plant = NativeContact(profile, spec)
    limits = read_json(HANDOVER_CONFIG)["flight_envelope"]
    horizons = {"braking": frozen["horizon_s"], "handover": cfg["handover_horizon_s"]}
    mesh = None
    if args.replay_dir:
        args.replay_dir.mkdir(parents=True, exist_ok=False)
        mesh = target_mesh(plant, spec.legal_targets)
    results = {}
    with ProcessPoolExecutor(max_workers=args.workers, mp_context=multiprocessing.get_context("fork"),
                             initializer=init_worker,
                             initargs=(plant, controllers, horizons, limits, mesh is not None)) as pool:
        for mode, (_, actor) in controllers.items():
            tasks = [(mode, "fresh", data, idx, manifest["cases"][str(data["case_ids"][idx])]["first_contact_sample_s"])
                     for idx in indices]
            rows, replays = [], {}
            for row, trace in pool.map(worker_episode, tasks):
                rows.append(row)
                status = "qualified" if row["qualified"] else "failed"
                if trace is not None and len(trace[0]) > 1 and status not in replays:
                    path = args.replay_dir / f"{mode}-braking-{status}.npz"
                    name = f"{mode}-{row['case']}-{row['source_frame']}"
                    save_replay(path, name, trace, row, goal, mesh, profile.policy_dt_s)
                    replays[status] = dict(path=str(path), sha256=digest(path))
            bands = {}
            for band, (lo, hi) in zip(LEAD_NAMES, LEAD_BANDS_S):
                group = [row for row in rows if lo <= row["source_contact_lead_s"] < hi]
                bands[band] = dict(**summary(group), **contact_metrics(group))
            stats = dict(**summary(rows), **contact_metrics(rows))
            results[mode] = dict(actor=actor, summary=stats, by_lead_band=bands, episodes=rows, replays=replays)
            print(f"FRESH_BRAKING {mode} {stats['qualified']}/{stats['episodes']}", flush=True)
    write_result(args.output, dict(schema="asm2_phase4_meaningful_fresh_native_v1",
                                  fresh_manifest_sha256=digest(MANIFEST), model=plant.metadata,
                                  wall_s=time.monotonic()-started, training_transitions=0, results=results))


def main():
    parser = ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    subcommands.add_parser("freeze")
    evaluation = subcommands.add_parser("evaluate")
    evaluation.add_argument("--actor", type=Path, required=True)
    evaluation.add_argument("--checkpoint", type=Path, required=True)
    evaluation.add_argument("--output", type=Path, required=True)
    evaluation.add_argument("--replay-dir", type=Path)
    evaluation.add_argument("--workers", type=int, default=8, choices=range(1, 17))
    args = parser.parse_args()
    if args.command == "freeze":
        freeze()
    else:
        if args.output.exists() or (args.replay_dir and args.replay_dir.exists()):
            parser.error("Choose new output and replay paths")
        evaluate(args)


if __name__ == "__main__":
    main()
