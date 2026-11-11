"""Compare the frozen aircraft collision assets on Phase 4R development controls."""

from collections import deque
import argparse
from importlib.util import module_from_spec, spec_from_file_location
import os
from pathlib import Path
import sys
import time

import numpy as np

from rm27_asm2.asset import ROOT, digest, load_inputs, read_json, write_result
from rm27_asm2.contact_isaac import EVIDENCE, PROXY_MANIFEST
from rm27_asm2.contact_task import ContactScorer
from rm27_asm2.proxy import load_proxy
from rm27_asm2.terminal_isaac import TerminalSim
from rm27_asm2.terminal_preflight import batch

RUN = Path(__file__).resolve().parent
SOURCE = RUN.parent / "phase4r"
ORIGIN = "dev_lateral_positive:763"
FOCUSED = (ORIGIN, "dev_far:867", "field_inverted_body", "near_miss_receding_patch")
DT = .0001
OFFSET = 1e-7
BODY_HULL = "collision_0_570"


def corrected_asset(spec):
    """Build a diagnostic overlay that references one missing original body hull."""
    config, _, _, _ = load_inputs()
    reference = ROOT / config["candidate_usd"]
    baseline = read_json(EVIDENCE / "isaac-proxy-field.json")
    manifest = digest(PROXY_MANIFEST)
    proxy = EVIDENCE / f"proxy-{manifest[:12]}.usda"
    if (baseline["proxy"]["manifest_sha256"] != manifest or digest(proxy) != baseline["asset_sha256"]
            or digest(reference) != config["candidate_sha256"]
            or baseline["contact_config_sha256"] != spec.contract_sha256):
        raise ValueError("Frozen collision inputs changed")
    entries = load_proxy()["entries"]
    entry = spec.entries[BODY_HULL]
    if entry["group"] != "body" or any(row["name"] == BODY_HULL for row in entries):
        raise ValueError("Expected one missing body hull with its original identity")
    path = RUN / "proxy-with-reference-body.usda"
    proxy_path, detailed_path = (os.path.relpath(p, path.parent) for p in (proxy, reference))
    content = f'''#usda 1.0
(
    defaultPrim = "Drone"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Drone" (prepend references = @{proxy_path}@</Drone>)
{{
    over "Collision"
    {{
        over "{BODY_HULL}" (prepend references = @{detailed_path}@</Drone/Collision/{BODY_HULL}>)
        {{
        }}
    }}
}}
'''
    try:
        with path.open("x") as f:
            f.write(content)
    except FileExistsError:
        if path.read_text() != content:
            raise ValueError("Existing diagnostic overlay differs from the pinned reference")
    return path, [*entries, entry]


class TraceSim(TerminalSim):
    """Keep pre-step states and all signed pairs of the first reported interval."""

    def __init__(self, *args, **kwargs):
        self.first = {}
        super().__init__(*args, **kwargs)

    def _finish_tick(self, worlds, states, rotations, momentum):
        for world, state, impulse in zip(worlds, states, momentum):
            if world not in self.first and world in self.signed:
                pairs = {}
                for air, target, group, separation, _, _ in self.signed[world]:
                    key = air, target, group
                    pairs[key] = min(separation, pairs.get(key, 0.))
                self.first[world] = dict(pre_state=state.tolist(), momentum_residual_Ns=impulse.tolist(),
                                         pairs=[[*key, pairs[key]] for key in sorted(pairs)])
        super()._finish_tick(worlds, states, rotations, momentum)


def load_cases():
    spec = spec_from_file_location("phase4r_contact_gate", SOURCE / "contact_gate.py")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.load_controls(include_braking=True)


def native(profile, spec, controls):
    from rm27_asm2.contact_native import NativeContact
    from rm27_asm2.terminal_native_eval import restore

    plant = NativeContact(profile, spec)
    result = {}
    for row in controls:
        scorer = ContactScorer(spec, dt=plant.dt)
        records = restore(plant, row["state"], row["thrust"], row["target"])
        outcome = scorer.observe(records, plant.state(), 0)
        first = None
        recent = deque(maxlen=6)
        substeps = round(profile.policy_dt_s/plant.dt)
        for i in range(round(row["source_window_s"]/plant.dt)):
            if outcome is not None:
                break
            if i % substeps == 0:
                plant.set_ctbr(row["tape"][min(i//substeps, len(row["tape"])-1)])
            recent.append(dict(pre_state=plant.state().tolist(), thrust=plant.motor_thrust_N.tolist(),
                               target=plant.data.ctrl.tolist(), time_s=plant.tick*plant.dt))
            records = plant.step_physics()
            if first is None and any(r["distance_m"] <= 0. for r in records):
                first = dict(**recent[-1], probe=recent[0],
                             signed=[r for r in records if r["distance_m"] <= 0.])
            outcome = scorer.observe(records, plant.pre_state, plant.tick)
        outcome = outcome or scorer.finish()
        label = "DESIGNATED_LOADED" if outcome["qualified"] else outcome["reason"]
        if label != row["expected"]:
            raise ValueError(f"Native baseline changed for {row['name']}: {label}")
        result[row["name"]] = dict(label=label, first=first)
        print("4R2_NATIVE", row["name"], label, first and first["time_s"], flush=True)
    return dict(physics_dt_s=plant.dt, controls=result, contact_config_sha256=spec.contract_sha256)


def isaac(profile, spec, controls, asset, layout, gate):
    override = corrected_asset(spec) if asset == "corrected" else None
    sim = TraceSim(16 if gate else len(controls), layout, profile, spec, diagnostic=True,
                   physics_dt=DT, contact_offset_m=OFFSET, detailed=asset == "detailed", asset_override=override)
    order = np.arange(sim.worlds) % len(controls)
    if layout:
        order = np.roll(order, 3)
    out = []
    refs = {r["name"]: r["first_touch"] for r in read_json(SOURCE / "native-gate-braking.json")["results"]}
    start = time.monotonic()
    for assignment in ((order, order[::-1]) if gate else (order,)):
        sim.first.clear()
        run = batch(sim, controls, assignment, 0.)
        for row in run["rows"]:
            row["first_signed"] = sim.first.get(row["world"])
            ref = refs[row["name"]]
            dt = None if ref is None or row["signed_tick"] is None else (row["signed_tick"]-1)*DT-ref["sample_time_s"]
            row["native_signed_time_delta_s"] = dt
        out.append(run)
        print("4R2_ISAAC", asset, layout, [(r["name"], r["reason"], r["signed_tick"]) for r in run["rows"]], flush=True)
    passed = all(
        batch["finite"] and batch["exact_motor_readback"] and batch["reset_max_abs_error"] < 1e-6
        and max(batch["first_target_error_N"], batch["first_lag_error_N"], batch["first_control_lag_error_N"]) < 1e-6
        and all(r["reason"] == r["expected"] and
                (r["native_signed_time_delta_s"] is None or abs(r["native_signed_time_delta_s"]) <= .002)
                for r in batch["rows"])
        for batch in out
    )
    return dict(physics_dt_s=sim.dt, control_dt_s=profile.policy_dt_s, contact_offset_m=OFFSET,
                asset=asset, asset_sha256=sim.asset_sha256, layout=layout, contact_config_sha256=spec.contract_sha256,
                rollout_seconds=time.monotonic()-start, batches=out, passed=passed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", required=True, choices=("native", "isaac"))
    parser.add_argument("--asset", choices=("detailed", "proxy", "corrected"), default="proxy")
    parser.add_argument("--layout", type=int, choices=(0, 1), default=0)
    parser.add_argument("--gate", action="store_true")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a fresh result path")
    started = time.monotonic()
    profile, spec, rows = load_cases()
    controls = rows if args.gate else [next(r for r in rows if r["name"] == name) for name in FOCUSED]
    if args.backend == "isaac":
        from isaaclab.app import AppLauncher
        app = AppLauncher(headless=True)
        report = isaac(profile, spec, controls, args.asset, args.layout, args.gate)
    else:
        report = native(profile, spec, controls)
    write_result(args.output, dict(**report, diagnostic_seconds=time.monotonic()-started,
                                  source_gate_sha256=digest(SOURCE / "native-gate-braking.json"), training_transitions=0))
    if args.backend == "isaac":
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
