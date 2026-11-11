"""Freeze witness-derived ASM2 reset states for development and held-out evaluation."""

from __future__ import annotations

import argparse
from collections import Counter
from hashlib import sha256
from io import BytesIO
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from .asset import ROOT, digest, obj_mesh, write_result
from .contact_task import PATCH_GROUPS, load_contact_spec
from .flight import load_profile
from .flight_suite import rotation_matrix


SCHEMA = "rm27_asm2_handover_resets_v1"
SPLITS = ("development", "heldout")
KINDS = ("nominal", "boundary", "stationary")
PHASES = ("terminal", "braking", "rotation", "handover")
MAX_FRAMES_PER_PHASE_CASE = 12
NAV_CONFIG = ROOT / "configs/rm27_asm2_handover_navigation.json"
RUN_CONFIG = ROOT / "configs/rm27_asm2_handover.json"


def _check_native_provenance(model, spec, profile, field_arrays_sha):
    field = model["field"]
    field_pin = spec.config["field_geometry"]
    if (model["backend"] != "MuJoCo" or model["version"] != "3.12.0"
            or model["physics"]["timestep_s"] != spec.dt
            or model["detailed_aircraft_colliders"] != len(spec.entries)
            or model["profile_sha256"] != profile.profile_sha256
            or model["contract_sha256"] != spec.contract_sha256
            or field["geometry_json_sha256"] != field_pin["sha256"]
            or field["geometry_npz_sha256"] != field_arrays_sha
            or field["native_scene_sha256"] != spec.field_geometry["model_sha256"]
            or field["registration_sha256"] != spec.config["registration"]["sha256"]):
        raise ValueError("Reset witness uses a different native model or contact contract")


def _contact_time(result, spec):
    if (result["qualified"] is not True or result["reason"] != "QUALIFIED"
            or result["contract_sha256"] != spec.contract_sha256 or result["physics_dt_s"] != spec.dt
            or result["positive_separation_compressive_records"]):
        raise ValueError("Reset witness lacks a qualified Phase 2 contact outcome")
    first, loaded = result["first_touch"], result["first_loaded"]
    if not first or not loaded or first["tick"] <= 0 or first["tick"] > loaded["tick"]:
        raise ValueError("Invalid signed-touch and loaded-contact event ordering")
    for event in (first, loaded):
        groups = set(event["groups"])
        targets = set(event["target_colliders"])
        if not groups or not targets or not groups <= PATCH_GROUPS or not targets <= spec.legal_targets:
            raise ValueError("Unregistered or forbidden pair in the qualified contact outcome")
    time_s = (first["tick"] - 1) * spec.dt
    if not np.isclose(first["sample_time_s"], time_s, atol=1e-12, rtol=0):
        raise ValueError("First-contact sample time disagrees with the native tick")
    impulse = loaded["compressive_normal_impulse_N_s"]
    if not np.isfinite(impulse) or impulse < spec.config["thresholds"]["normal_impulse_min_N_s"]:
        raise ValueError("Qualified witness lacks the minimum compressive impulse")
    return time_s


def _source_trace(raw, handover_frame, first_contact_s, profile):
    with np.load(BytesIO(raw), allow_pickle=False) as archive:
        states = archive["states"]
        thrust = archive["motor_thrust_N"]
        targets = archive["motor_target_N"]
        times = archive["sample_times_s"]
        frame = archive["handover_frame"]
        if (archive["dt"].item() != profile.policy_dt_s
                or frame.dtype.kind not in "iu" or frame.item() != handover_frame):
            raise ValueError("Trace timing or handover index disagrees with its report")
    n = len(times)
    for value, shape in ((states, (n, 1, 13)), (thrust, (n, 1, 4)),
                         (targets, (n, 1, 4)), (times, (n,))):
        if value.shape != shape or value.dtype != np.float64 or not np.isfinite(value).all():
            raise ValueError(f"Expected finite native float64 trace array of shape {shape}")
    if (n < 1 or not 0 <= handover_frame < n
            or not np.all(np.diff(times) > 0) or times[0] != 0.
            or not np.allclose(times[:-1], np.arange(n - 1) * profile.policy_dt_s, atol=1e-10, rtol=0)
            or (n > 1 and times[-1] - times[-2] > profile.policy_dt_s + 1e-10)
            or (first_contact_s is not None
                and not times[handover_frame] < first_contact_s < times[-1])):
        raise ValueError("Expected a continuous native trace with a pre-contact handover")
    if not np.allclose(np.linalg.norm(states[:, 0, 3:7], axis=1), 1., atol=1e-10, rtol=0):
        raise ValueError("Trace quaternion is not unit length")
    maximum = profile.max_rotor_thrust_N + 1e-12
    if any(np.any((values < -1e-12) | (values > maximum)) for values in (thrust, targets)):
        raise ValueError("Recorded motor state or held target exceeds the actuator limits")
    return states[:, 0], thrust[:, 0], targets[:, 0], times


def _phase_labels(states, frames, vertices, spec, terminal_distance):
    """Tag one trace by measured state, without fitting or replaying controls."""
    snapshot = states[frames]
    normal = spec.target_normal
    rotation = rotation_matrix(snapshot[:, 3:7])
    directions = np.einsum("nji,j->ni", rotation, normal)
    support = directions @ vertices.T
    nearest = np.argmin(support, axis=1)
    point_offsets = np.einsum("nij,nj->ni", rotation, vertices[nearest])
    gap = (snapshot[:, :3] + point_offsets - spec.target_center) @ normal
    omega = np.einsum("nij,nj->ni", rotation, snapshot[:, 10:13])
    point_velocity = snapshot[:, 7:10] + np.cross(omega, point_offsets)
    normal_speed = -(point_velocity @ normal)
    tangent_speed = np.linalg.norm(point_velocity + normal_speed[:, None] * normal, axis=1)
    alignment = np.rad2deg(np.arccos(np.clip(directions[:, 2], -1., 1.)))
    limits = spec.config["thresholds"]
    terminal = ((gap <= terminal_distance)
                & (normal_speed >= limits["normal_speed_min_m_s"])
                & (normal_speed <= limits["normal_speed_max_m_s"])
                & (tangent_speed <= limits["tangent_speed_max_m_s"])
                & (alignment <= limits["alignment_max_deg"])
                & (np.linalg.norm(snapshot[:, 10:13], axis=1) <= limits["body_rate_max_rad_s"]))
    inward = -(snapshot[:, 7:10] @ normal)
    braking = (np.arange(len(frames)) > np.argmax(inward)) & (inward > 0.)
    phase = np.full(len(frames), "rotation", dtype="<U8")
    phase[braking] = "braking"
    phase[terminal] = "terminal"
    phase[0] = "handover"
    return phase


def native_verifier(plant):
    """Check exact pose/actuator restoration and collision state with fresh MjData."""
    import mujoco

    model = plant.model
    data = mujoco.MjData(model)

    def verify(state, thrust, target):
        mujoco.mj_resetData(model, data)
        data.qpos[:], data.qvel[:] = state[:7], state[7:]
        data.act[:], data.ctrl[:] = thrust, target
        mujoco.mj_forward(model, data)
        exact = (np.array_equal(data.qpos, state[:7]) and np.array_equal(data.qvel, state[7:])
                 and np.array_equal(data.act, thrust) and np.array_equal(data.ctrl, target))
        signed = 0
        loaded = 0
        minimum = None
        force = np.empty(6)
        for index in range(data.ncon):
            distance = float(data.contact[index].dist)
            signed += int(distance <= 0)
            minimum = distance if minimum is None else min(minimum, distance)
            mujoco.mj_contactForce(model, data, index, force)
            loaded += int(np.any(force != 0.))
        finite = np.isfinite(data.qacc).all() and np.isfinite(data.act_dot).all()
        warnings = sum(warning.number for warning in data.warning)
        return dict(passed=bool(exact and finite and not warnings and not signed and not loaded),
                    exact_restore=exact, finite_dynamics=bool(finite), warnings=warnings,
                    signed_contacts=signed, force_bearing_contacts=loaded,
                    minimum_reported_separation_m=minimum)

    return verify


def _split_arrays(rows, dt):
    return dict(
        states=np.asarray([row[0] for row in rows], dtype=np.float64).reshape(-1, 13),
        motor_thrust_N=np.asarray([row[1] for row in rows], dtype=np.float64).reshape(-1, 4),
        motor_target_N=np.asarray([row[2] for row in rows], dtype=np.float64).reshape(-1, 4),
        sample_times_s=np.asarray([row[3] for row in rows], dtype=np.float64),
        source_frame=np.asarray([row[4] for row in rows], dtype=np.int64),
        phase=np.asarray([row[5] for row in rows], dtype="<U8"),
        case_ids=np.asarray([row[6] for row in rows], dtype=str),
        dt=np.asarray(dt, dtype=np.float64),
    )


def build_dataset(run_dir, output=None, *, verify_reset=None, spec=None, profile=None):
    """Read immutable case reports and write two isolated reset populations."""
    run_dir = Path(run_dir).resolve()
    output = (run_dir / "reset-dataset" if output is None else Path(output)).resolve()
    if output.exists():
        raise FileExistsError(output)
    spec = load_contact_spec() if spec is None else spec
    profile = load_profile() if profile is None else profile
    summary_path = run_dir / "summary.json"
    summary_bytes = summary_path.read_bytes()
    summary = json.loads(summary_bytes)
    if (summary["schema"] != "rm27_asm2_handover_results_v1"
            or summary["contact_contract_sha256"] != spec.contract_sha256
            or summary["profile_sha256"] != profile.profile_sha256
            or summary["config_sha256"] != digest(RUN_CONFIG)
            or summary["navigation_config_sha256"] != digest(NAV_CONFIG)
            or summary["training_transitions"] != 0):
        raise ValueError("Expected a completed Phase 3 deterministic evaluation run")
    recipes = {row["name"]: row for row in json.loads(NAV_CONFIG.read_bytes())["recipes"]}
    heldout_outcomes = {case: dict(status="UNRUN" if case in summary["unrun"] else "NOT_IN_RUN")
                        for case, recipe in recipes.items() if recipe["split"] == "heldout"}
    field_arrays_sha = digest((ROOT / spec.config["field_geometry"]["path"]).with_suffix(".npz"))
    patches = [spec.source / "mujoco" / entry["mesh"] for entry in spec.entries.values()
               if entry["group"] in PATCH_GROUPS]
    if len(patches) != 15:
        raise ValueError("Expected the frozen 15 ASM2 contact patches")
    vertices = np.concatenate([obj_mesh(path)[0] for path in patches])
    # Two actuator time constants at the largest Phase 2 admissible normal speed.
    terminal_distance = (2 * profile.motor_time_constant_s
                         * spec.config["thresholds"]["normal_speed_max_m_s"])
    cases, rows, rejected = {}, {split: [] for split in SPLITS}, []
    heldout_rows, heldout_qualified = [], []
    trace_splits = {}
    restore_tested = 0
    for path in sorted(run_dir.glob("*.json")):
        raw_report = path.read_bytes()
        report = json.loads(raw_report)
        if not isinstance(report, dict) or "case" not in report:
            continue
        case, split, kind = report["case"], report["split"], report["kind"]
        if case != path.stem or split not in SPLITS or kind not in KINDS:
            raise ValueError(f"Invalid case identity, split or kind in {path}")
        recipe = recipes.get(case)
        if ((kind != "stationary" and (not recipe or recipe["split"] != split or recipe["kind"] != kind))
                or (kind == "stationary" and (recipe is not None or split != "development"))):
            raise ValueError(f"Case differs from the frozen navigation split: {case}")
        qualified = report["qualified"]
        if any(type(flag) is not bool for flag in (
                qualified, report["navigation"]["passed"], report["safety"]["passed"])):
            raise ValueError(f"Expected boolean case qualification and safety flags in {path}")
        trace = path.with_suffix(".npz")
        raw_trace = trace.read_bytes() if trace.is_file() else None
        trace_hash = sha256(raw_trace).hexdigest() if raw_trace is not None else None
        heldout_gate = split == "heldout" and report["navigation"]["passed"]
        if trace_hash != report["trace_sha256"] or (qualified or heldout_gate) and raw_trace is None:
            raise ValueError(f"Trace SHA-256 mismatch for {case}")
        result = report["result"]
        case_row = dict(
            split=split, kind=kind, qualified=qualified,
            reason=report["reason"],
            contact_reason=result["reason"] if result is not None else None,
            report=dict(path=os.path.relpath(path, output), sha256=sha256(raw_report).hexdigest()),
            trace=dict(path=os.path.relpath(trace, output), sha256=trace_hash) if raw_trace is not None else None,
            navigation=report["navigation"], safety=report["safety"],
            handover_frame=report["handover_frame"], candidate_counts={}, emitted_counts={},
        )
        cases[case] = case_row
        if split == "heldout" and not heldout_gate:
            heldout_outcomes[case] = dict(status="NAVIGATION_REJECTED", reason=report["reason"])
        if not qualified and not heldout_gate:
            continue
        if qualified and not (report["navigation"]["passed"] and report["safety"]["passed"] and result):
            raise ValueError(f"Qualified case {case} lacks safe and continuous navigation evidence")
        if trace_splits.setdefault(trace_hash, split) != split:
            raise ValueError(f"One source trace appears in both populations: {case}")
        _check_native_provenance(report["model"], spec, profile, field_arrays_sha)
        first_contact_s = (_contact_time(result, spec) if qualified else
                           result["first_touch"]["sample_time_s"] if result and result["first_touch"] else None)
        handover = report["handover_frame"]
        if type(handover) is not int:
            raise ValueError(f"Expected an integer handover frame for {case}")
        states, thrust, targets, times = _source_trace(raw_trace, handover, first_contact_s, profile)
        if (not np.array_equal(states[handover], report["handover_state"])
                or times[handover] != report["handover_time_s"]):
            raise ValueError(f"Trace and report disagree on the recorded handover state: {case}")
        case_row["first_contact_sample_s"] = first_contact_s
        case_row["model"] = report["model"]
        checked_frames = {}

        def restored(frame, phase):
            nonlocal restore_tested
            if verify_reset is None:
                return True
            if frame not in checked_frames:
                restore_tested += 1
                checked = verify_reset(states[frame], thrust[frame], targets[frame])
                checked_frames[frame] = checked["passed"] is True
                if not checked_frames[frame]:
                    rejected.append(dict(case=case, split=split, phase=phase, source_frame=int(frame),
                                         sample_time_s=float(times[frame]), verification=checked))
            return checked_frames[frame]

        if heldout_gate:
            accepted = restored(handover, "handover")
            heldout_outcomes[case] = dict(status="INCLUDED" if accepted else "RESTORE_REJECTED",
                                          witness_qualified=qualified, witness_reason=report["reason"],
                                          source_frame=handover)
            if accepted:
                heldout_rows.append((states[handover], thrust[handover], targets[handover],
                                     times[handover], handover, "handover", case))
                heldout_qualified.append(qualified)
        if not qualified:
            continue
        # Keep strict pre-touch frames. A tiny numeric allowance excludes
        # equivalent times computed with different multiplication orders.
        frames = np.flatnonzero((np.arange(len(times)) >= handover) & (times < first_contact_s - 1e-12))
        if not len(frames) or frames[0] != handover:
            raise ValueError(f"No pre-contact handover snapshot for {case}")
        phases = _phase_labels(states, frames, vertices, spec, terminal_distance)
        counts = Counter()
        for phase in PHASES:
            eligible = frames[phases == phase]
            case_row["candidate_counts"][phase] = len(eligible)
            if not len(eligible):
                continue
            selected = np.linspace(0, len(eligible) - 1, min(len(eligible), MAX_FRAMES_PER_PHASE_CASE))
            for frame in eligible[np.rint(selected).astype(int)]:
                if not restored(frame, phase):
                    continue
                rows[split].append((states[frame], thrust[frame], targets[frame],
                                    times[frame], frame, phase, case))
                counts[phase] += 1
        case_row["emitted_counts"] = {phase: counts[phase] for phase in PHASES}
    keys = ("split", "kind", "qualified", "reason")
    frozen_cases = {row["case"]: {key: row[key] for key in keys} for row in summary["cases"]}
    actual_cases = {case: {key: row[key] for key in keys} for case, row in cases.items()}
    if (not cases or len(frozen_cases) != summary["evaluated"]
            or summary["qualified"] != sum(row["qualified"] for row in cases.values())
            or frozen_cases != actual_cases or set(cases) & set(summary["unrun"])):
        raise ValueError("Witness reports do not match the completed evaluation denominator")

    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=output.parent, prefix=".reset-dataset-") as temporary:
        stage = Path(temporary) / "bundle"
        stage.mkdir()
        populations = {}
        for split in SPLITS:
            arrays = _split_arrays(rows[split], profile.policy_dt_s)
            archive = stage / f"{split}.npz"
            np.savez_compressed(archive, **arrays)
            ids = sorted(case for case, info in cases.items() if info["split"] == split)
            counts = Counter(arrays["phase"].tolist())
            populations[split] = dict(
                path=archive.name, sha256=digest(archive), cases=ids,
                qualified_cases=sum(cases[case]["qualified"] for case in ids),
                reset_cases=sorted(set(arrays["case_ids"].tolist())), resets=len(rows[split]),
                phase_counts={phase: counts[phase] for phase in PHASES},
            )
        heldout_archive = stage / "heldout-handover.npz"
        np.savez_compressed(heldout_archive, **_split_arrays(heldout_rows, profile.policy_dt_s),
                            witness_qualified=np.asarray(heldout_qualified, dtype=bool))
        manifest = dict(
            schema=SCHEMA, scope="simulation-only witness-derived reset candidates",
            training_allowed=False, training_transitions=0,
            generator_sha256=digest(__file__), profile_sha256=profile.profile_sha256,
            contact_contract_sha256=spec.contract_sha256, profile_source_pins=profile.source_pins,
            field_geometry=spec.config["field_geometry"], registration=spec.config["registration"],
            patch_mesh_pins={str(path.relative_to(ROOT)): digest(path) for path in patches},
            physics_dt_s=spec.dt, control_dt_s=profile.policy_dt_s, cases=cases, splits=populations,
            heldout_handover=dict(
                path=heldout_archive.name, sha256=digest(heldout_archive),
                frozen_recipe_denominator=len(heldout_outcomes), resets=len(heldout_rows),
                cases=heldout_outcomes,
                selection="Navigation-passed gate states, regardless of later witness outcome",
            ),
            source_run=dict(path=os.path.relpath(summary_path, output),
                            sha256=sha256(summary_bytes).hexdigest(),
                            config_sha256=summary["config_sha256"],
                            navigation_config_sha256=summary["navigation_config_sha256"],
                            evaluated=summary["evaluated"], qualified=summary["qualified"],
                            unrun=summary["unrun"]),
            selection=dict(
                max_per_phase_per_case=MAX_FRAMES_PER_PHASE_CASE, terminal_gap_limit_m=terminal_distance,
                handover="Exact recorded navigation-to-maneuver boundary",
                terminal="Nearest patch vertex inside the approach distance and Phase 2 kinematic envelope",
                braking="Inward-moving state after peak measured COM inward speed",
                rotation="Other post-handover rotation/translation states",
                sampling="Evenly spaced control frames within each case and phase, strictly before signed touch",
            ),
            restoration=dict(status="CHECKED" if verify_reset is not None else "NOT_RUN",
                             tested=restore_tested, passed=restore_tested - len(rejected), rejected=rejected),
            runtime_contract=dict(
                state="float64 COM xyz and wxyz body-to-world quaternion, world linear velocity, FLU body rates",
                motors="Four rotor thrusts and held targets in N, source rotor order (+,+),(+,-),(-,-),(-,+)",
                frame="World axes translated by the frozen Field target center",
                requirements=[
                    "Verify artifact hashes, use the matching plant, geometry and control/physics timing.",
                    "Restore pose, velocities, current motor thrusts and held targets together.",
                    "Check contact-free restoration on the destination backend before use.",
                    "Use development resets exclusively for development and held-out cases for evaluation.",
                    "Issue independent control commands after reset. The arrays carry reset state only.",
                ],
            ),
        )
        write_result(stage / "manifest.json", manifest)
        if output.exists():
            raise FileExistsError(output)
        stage.rename(output)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify-native", action="store_true",
                        help="Re-forward every selected reset on the detailed native contact model")
    args = parser.parse_args()
    profile, spec = load_profile(), load_contact_spec()
    check = None
    if args.verify_native:
        from .contact_native import NativeContact

        check = native_verifier(NativeContact(profile, spec))
    manifest = build_dataset(args.run_dir, args.output, verify_reset=check, spec=spec, profile=profile)
    print("ASM2_HANDOVER_RESET_DATASET", {
        split: manifest["splits"][split]["resets"] for split in SPLITS
    }, manifest["restoration"]["status"], flush=True)


if __name__ == "__main__":
    main()
