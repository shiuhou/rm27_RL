"""Phase 4R matched first-contact diagnostic. Reads Phase 3 development only."""

from __future__ import annotations

import argparse
from importlib.util import module_from_spec, spec_from_file_location
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

from rm27_asm2.asset import digest, read_json, write_result
from rm27_asm2.contact_task import PATCH_GROUPS, load_contact_spec
from rm27_asm2.flight import load_profile
from rm27_asm2.terminal_preflight import RUN as PILOT, batch, cases, load_development, load_experiment

WORLDS = 16
TRIM = (("dev_center", 744), ("dev_fast", 890), ("dev_fast", 894), ("dev_lateral_positive", 790))
WINDOW_S = .120
BRAKING = (("dev_far", 867, "DESIGNATED_LOADED"),
           ("dev_lateral_positive", 763, "FORBIDDEN_AIRFRAME_FIRST_TOUCH"))
BRAKING_WINDOW_S = .150
TIMING_TOLERANCE_S = .002
NEAR_MISS_OFFSET_M = .00015
NEAR_MISS_OUTWARD_SPEED_M_S = .01


def load_controls(include_braking):
    config, pins = load_experiment()
    profile, spec = load_profile(), load_contact_spec()
    native = read_json(PILOT / "native-preflight.json")
    dev = load_development(pins["resets"])
    controls = cases(config, pins, native, spec)[:-1]
    trim = np.array([[profile.hover_thrust_N, 0., 0., 0.]])
    for case, frame in TRIM:
        row, = np.flatnonzero((dev["case_ids"] == case) & (dev["source_frame"] == frame))
        controls.append(dict(name=f"{case}:{frame}", state=dev["states"][row],
                             thrust=dev["motor_thrust_N"][row], target=dev["motor_target_N"][row],
                             tape=trim, source_window_s=WINDOW_S, expected="DESIGNATED_LOADED"))
    source = native["positives"]["dev_center"]["reset"]
    with np.load(pins["dev_center"], allow_pickle=False) as trace:
        frame = int(np.searchsorted(trace["sample_times_s"], source["native_first_signed_source_s"], side="left")-1)
        state = trace["states"][frame, 0].copy()
        thrust = trace["motor_thrust_N"][frame, 0].copy()
        target = trace["motor_target_N"][frame, 0].copy()
    state[:3] += NEAR_MISS_OFFSET_M*spec.target_normal
    state[7:10] = NEAR_MISS_OUTWARD_SPEED_M_S*spec.target_normal
    state[10:13] = 0.
    controls.append(dict(name="near_miss_receding_patch", state=state, thrust=thrust, target=target, tape=trim,
                         source_window_s=WINDOW_S, expected="NO_SIGNED_TOUCH", source_frame=frame))
    if include_braking:
        archive = pins["resets"].parent / "development.npz"
        if digest(archive) != read_json(pins["resets"])["splits"]["development"]["sha256"]:
            raise ValueError("Development reset archive changed")
        with np.load(archive, allow_pickle=False) as bank:
            for case, frame, expected in BRAKING:
                idx, = np.flatnonzero((bank["case_ids"] == case) & (bank["source_frame"] == frame)
                                      & (bank["phase"] == "braking"))
                controls.append(dict(name=f"{case}:{frame}", state=bank["states"][idx],
                                     thrust=bank["motor_thrust_N"][idx], target=bank["motor_target_N"][idx],
                                     tape=trim, source_window_s=BRAKING_WINDOW_S, expected=expected))
    window = BRAKING_WINDOW_S if include_braking else WINDOW_S
    for control in controls:
        # Use one time window for every world. A short archive can be extended
        # only by holding its last CTBR command, as in Phase 4 preflight.
        control["source_window_s"] = window
    return profile, spec, controls


def native_run(profile, spec, controls):
    from mujoco import mj_geomDistance
    from rm27_asm2.contact_native import NativeContact

    old = spec_from_file_location("phase4_native_preflight", PILOT / "native_preflight.py")
    module = module_from_spec(old)
    old.loader.exec_module(module)
    plant = NativeContact(profile, spec)
    near_miss_gap = None
    results = []
    for row in controls:
        if row["name"] == "near_miss_receding_patch":
            initial = plant.reset(row["state"])
            if any(p["distance_m"] <= 0 for p in initial):
                raise ValueError("The near-miss fixture starts touching")
            model, data = plant.model, plant.data
            patch = [model.geom(name).id for name, e in spec.entries.items() if e["group"] in PATCH_GROUPS]
            armor = [model.geom(name).id for name in spec.legal_targets]
            near_miss_gap = min(mj_geomDistance(model, data, a, b, 0.001, None) for a in patch for b in armor)
            if not 0 < near_miss_gap < 0.0005:
                raise ValueError(f"Near-miss fixture is outside the positive sub-mm gap: {near_miss_gap}")
        measurement = module.replay(plant, tuple(row[key] for key in ("state", "thrust", "target")), row["tape"],
                                    row["source_window_s"], 0.)
        result = measurement["result"]
        label = "DESIGNATED_LOADED" if result["reason"] == "QUALIFIED" else result["reason"]
        if label != row["expected"]:
            raise ValueError(f"Frozen native control changed: {row['name']} {label}")
        results.append(dict(name=row["name"], label=label, first_touch=result["first_touch"],
                            first_loaded=result["first_loaded"], interval_end_s=measurement["final_interval_end_s"]))
        print(f"ASM2_4R_NATIVE {row['name']} {label} {measurement['final_interval_end_s']:.6f}", flush=True)
    return dict(passed=True, physics_dt_s=plant.dt, results=results, near_miss_initial_patch_to_armor_m=near_miss_gap,
                contact_config_sha256=spec.contract_sha256)


def isaac_run(profile, spec, controls, dt, layout, native_ref, contact_offset_m=None):
    from rm27_asm2.terminal_isaac import TerminalSim

    if (native_ref["physics_dt_s"] != spec.dt or native_ref["contact_config_sha256"] != spec.contract_sha256
            or native_ref["passed"] is not True):
        raise ValueError("Expected the frozen native timestep and successful matched reference")
    refs = {row["name"]: row for row in native_ref["results"]}
    if set(refs) != {r["name"] for r in controls}:
        raise ValueError("Matched control identities changed")
    offset = {} if contact_offset_m is None else {"contact_offset_m": contact_offset_m}
    sim = TerminalSim(WORLDS, layout, profile, spec, diagnostic=True, physics_dt=dt, **offset)
    assignment = np.arange(WORLDS) % len(controls)
    if layout:
        assignment = np.roll(assignment, 3)
    reports = [batch(sim, controls, mapping, 0.) for mapping in (assignment, assignment[::-1])]
    for report in reports:
        for row in report["rows"]:
            ref = refs[row["name"]]
            row["native_reason"] = ref["label"]
            t = ref["first_touch"]
            row["native_groups"] = t["groups"] if t is not None else []
            reference_s = t["sample_time_s"] if t is not None else None
            row["native_first_touch_s"] = reference_s
            tick = row["signed_tick"]
            row["first_signed_interval_s"] = [(tick-1)*dt, tick*dt] if tick is not None else None
            row["timing_abs_error_s"] = abs((tick-1)*dt-reference_s) if None not in (tick, reference_s) else None
            row["same_native_label"] = row["reason"] == ref["label"]
            row["same_first_contact_groups"] = row["native_groups"] == ([] if row["contact"] is None else row["contact"]["groups"])
        mismatch = sorted({(r["name"], r["native_reason"], r["reason"]) for r in report["rows"] if not r["same_native_label"]})
        print(f"ASM2_4R_ISAAC dt={dt:g} layout={layout} matched={WORLDS-len([r for r in report['rows'] if not r['same_native_label']])}/{WORLDS} mismatches={mismatch}", flush=True)
    reproducible = True
    for control in controls:
        rows = [r for report in reports for r in report["rows"] if r["name"] == control["name"]]
        labels = {r["reason"] for r in rows}
        ticks = [r["signed_tick"] for r in rows if r["signed_tick"] is not None]
        if len(labels) != 1 or (ticks and (len(ticks) != len(rows) or max(ticks)-min(ticks) > 1)):
            reproducible = False
    valid = reproducible and all(
        b["finite"] and b["exact_motor_readback"] and b["reset_max_abs_error"] < 1e-6
        and max(b["first_target_error_N"], b["first_lag_error_N"], b["first_control_lag_error_N"]) < 1e-6
        and all(r["same_native_label"] and
                (r["timing_abs_error_s"] is None or r["timing_abs_error_s"] <= TIMING_TOLERANCE_S)
                for r in b["rows"])
        for b in reports
    )
    return dict(passed=valid, reproducible_layout=reproducible, layout=layout, world_count=WORLDS, batches=reports,
                physics_dt_s=sim.dt, control_dt_s=profile.policy_dt_s, timing_tolerance_s=TIMING_TOLERANCE_S,
                contact_offset_override_m=contact_offset_m, asset_sha256=sim.asset_sha256,
                contact_config_sha256=spec.contract_sha256)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("native", "isaac"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dt", type=float)
    parser.add_argument("--contact-offset-m", type=float)
    parser.add_argument("--layout", type=int, choices=(0, 1))
    parser.add_argument("--native-reference", type=Path)
    parser.add_argument("--include-braking", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose an unused evidence path")
    started = time.monotonic()
    profile, spec, controls = load_controls(args.include_braking)
    if args.backend == "isaac":
        if args.dt is None or args.dt <= 0 or not math.isclose(profile.policy_dt_s/args.dt, round(profile.policy_dt_s/args.dt)):
            parser.error("Require a positive physics step dividing the fixed policy interval")
        if args.layout is None or args.native_reference is None:
            parser.error("Isaac requires --layout and --native-reference")
        from isaaclab.app import AppLauncher
        launcher = AppLauncher(headless=True)
        reference = read_json(args.native_reference)
        report = isaac_run(profile, spec, controls, args.dt, args.layout, reference, args.contact_offset_m)
        report["native_reference_sha256"] = digest(args.native_reference)
    else:
        report = native_run(profile, spec, controls)
    write_result(args.output, dict(**report, scope="Phase 4R development-only matched controls",
                                  training_transitions=0, diagnostic_seconds=time.monotonic()-started))
    code = 0 if report["passed"] else 1
    if args.backend == "isaac":
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
