"""Replay the ten frozen Phase 4R.2 first-contact controls."""

import argparse
from collections import deque
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

from .asset import ROOT, digest, load_inputs, read_json, write_result
from .contact_isaac import EVIDENCE, PROXY_MANIFEST
from .contact_task import ContactScorer, load_contact_spec
from .flight import allocate_ctbr, load_profile
from .flight_suite import rotation_matrix
from .proxy import load_proxy
from .terminal_isaac import TerminalSim

RUN = ROOT / ".agents/runs/asm2-terminal/phase4r2"
EVIDENCE_DIR = ROOT / "evidence/phase4r2"
DT = .0001
OFFSET = 1e-7
BODY_HULL = "collision_0_570"


def runtime_settings(sim):
    """Read back the composed contact settings for the first environment."""
    import carb
    import isaaclab
    from pxr import PhysxSchema, Usd, UsdPhysics

    scene, = (prim for prim in sim.stage.Traverse() if prim.IsA(UsdPhysics.Scene))
    scene_keys = ("solverType", "broadphaseType", "enableGPUDynamics", "enableCCD",
                  "enableStabilization", "enableEnhancedDeterminism", "timeStepsPerSecond",
                  "bounceThreshold", "frictionOffsetThreshold", "frictionCorrelationDistance",
                  "minPositionIterationCount", "maxPositionIterationCount",
                  "minVelocityIterationCount", "maxVelocityIterationCount")
    settings = {key: scene.GetAttribute("physxScene:" + key).Get() for key in scene_keys}
    env = sim.stage.GetPrimAtPath(sim.env_paths[0])
    drone = env.GetChild("Drone")
    body_keys = ("solverPositionIterationCount", "solverVelocityIterationCount", "enableCCD",
                 "enableSpeculativeCCD", "enableGyroscopicForces", "sleepThreshold",
                 "linearDamping", "angularDamping")
    body = {key: drone.GetAttribute("physxRigidBody:" + key).Get() for key in body_keys}
    offsets = {}
    for name in ("Drone", "Field"):
        shapes = [prim for prim in Usd.PrimRange(env.GetChild(name)) if prim.HasAPI(UsdPhysics.CollisionAPI)]
        contact = [PhysxSchema.PhysxCollisionAPI(prim).GetContactOffsetAttr().Get() for prim in shapes]
        rest = [PhysxSchema.PhysxCollisionAPI(prim).GetRestOffsetAttr().Get() for prim in shapes]
        np.testing.assert_allclose(contact, OFFSET, atol=1e-13, rtol=0)
        np.testing.assert_array_equal(rest, 0.)
        offsets[name] = dict(shapes=len(shapes), contact_m=sorted(set(contact)), rest_m=sorted(set(rest)))
    np.testing.assert_allclose(sim.com_actual[:, :3], 0., atol=1e-8, rtol=0)
    report_flags = {key: carb.settings.get_settings().get_as_bool("/physics/" + key)
                    for key in ("disableContactProcessing", "suppressReadback")}
    assert settings["solverType"] == "PGS" and settings["enableGPUDynamics"]
    assert offsets["Drone"]["shapes"] == 3522 and offsets["Field"]["shapes"] == 1822
    assert not any(report_flags.values())
    return dict(scene=settings, body=body, offsets=offsets, contact_report_flags=report_flags,
                isaac_lab=getattr(isaaclab, "__version__", "see image ID"), numpy=np.__version__,
                torch=sim.torch.__version__, torch_cuda=sim.torch.version.cuda,
                gpu=sim.torch.cuda.get_device_name(0), mass_kg=float(sim.mass_actual[0]),
                com_body_m=sim.com_actual[0, :3].tolist(), inertia_body_kg_m2=sim.inertia_actual[0].tolist())


# INSERT_FROZEN_FUNCTIONS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", required=True, choices=("native", "isaac"))
    parser.add_argument("--layout", type=int, choices=(0, 1), default=0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a fresh result path")
    profile, spec = load_profile(), load_contact_spec()
    ref_path = EVIDENCE_DIR / "native-gate-braking.json"
    ref = read_json(ref_path)
    controls = read_json(EVIDENCE_DIR / "controls.json")
    receipt = read_json(EVIDENCE_DIR / "backend-receipt.json")
    labels = {row["name"]: row["label"] for row in ref["results"]}
    if (len(controls) != len(labels) or len(labels) != 10 or not ref["passed"]
            or ref["contact_config_sha256"] != spec.contract_sha256
            or spec.contract_sha256 != receipt["contact_contract_sha256"]
            or digest(ref_path) != receipt["run_pins"][".agents/runs/asm2-terminal/phase4r/native-gate-braking.json"]
            or labels != {row["name"]: row["expected"] for row in controls}):
        raise ValueError("Frozen gate identities or labels changed")
    if args.backend == "native":
        from importlib.metadata import version

        for row in controls:
            for key in ("state", "thrust", "target", "tape"):
                row[key] = np.asarray(row[key], dtype=float)
        report = native(profile, spec, controls)
        report.update(passed=True, mujoco_version=version("mujoco"), numpy_version=version("numpy"))
    else:
        from isaaclab.app import AppLauncher
        app = AppLauncher(headless=True)
        # Validate the overlay before attempting to construct any GPU worlds.
        if digest(RUN / "proxy-with-reference-body.usda") != receipt["asset_sha256"]:
            raise ValueError("Phase 4R.2 overlay changed")
        report = isaac(profile, spec, controls, "corrected", args.layout, gate=True)
        if report["asset_sha256"] != receipt["asset_sha256"]:
            raise ValueError("Live asset differs from the Phase 4R.2 overlay")
    write_result(args.output, dict(**report, source_gate_sha256=digest(ref_path), training_transitions=0))
    if args.backend == "isaac":
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0 if report["passed"] else 1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
