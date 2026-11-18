"""Pinned G0 consumer: lossless offline replay, never policy or plant execution.

Shared contracts stay in the published package. Synthetic observation projection
tests the existing RL feature convention only; it grants no runtime trust, model
equivalence, physical branching or Direct-CTBR authority.
"""

from copy import deepcopy
from hashlib import sha1, sha256
import importlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from .asset import ROOT
from .demonstration_learning import observations


REVISION = "9ea80518259173c15882ae8a3ab6059eb2643336"
PACKAGE_ROOT = ROOT / "upstream/rm27_g0/9ea8051"
RECEIPT_PATH = ROOT / ".agents/runs/n1-r9-a/acquisition.json"
RECEIPT_SHA256 = "0fd9e74b8b99edd15da1b5986ea4fcfbb498fc1e09962e52ed1203cd90eb1641"

FEATURES = (
    (0, 30, "target goal minus COM, repeated ten times", "position_W_m + registered target"),
    (30, 33, "world velocity", "velocity_W_m_s"),
    (33, 42, "body FLU to world rotation, transposed row-major", "q_WB_wxyz"),
    (42, 45, "FLU body rates", "body_rates_B_rad_s"),
)


class SharedScenarioConsumer:
    """Verify acquired Git bytes before loading the unmodified official APIs."""

    def __init__(self, package_root=PACKAGE_ROOT, receipt_path=RECEIPT_PATH):
        self.root = Path(package_root).resolve()
        receipt_bytes = Path(receipt_path).read_bytes()
        if sha256(receipt_bytes).hexdigest() != RECEIPT_SHA256:
            raise ValueError("acquisition receipt SHA-256 mismatch")
        self.receipt = json.loads(receipt_bytes)
        if (self.receipt["source_revision"] != REVISION
                or self.receipt["source_repository"] != "https://github.com/shiuhou/rm27_drones"
                or self.receipt["contracts_version"] != "0.1.0-draft"
                or self.receipt["snapshot_format"] != "rm27.handover/1"):
            raise ValueError("unsupported publication identity")
        for relative, expected in self.receipt["files"].items():
            path = self.root / relative
            if path.is_symlink() or not path.resolve().is_relative_to(self.root):
                raise ValueError("package file escapes pinned root")
            payload = path.read_bytes()
            git_blob = sha1(b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()
            if (sha256(payload).hexdigest() != expected["sha256"]
                    or git_blob != expected["git_blob"] or len(payload) != expected["bytes"]):
                raise ValueError(f"pinned package SHA-256/Git blob mismatch: {relative}")
        expected_python = {p for p in self.receipt["files"] if p.startswith("rm27_exchange/")}
        actual_python = {p.relative_to(self.root).as_posix()
                         for p in (self.root / "rm27_exchange").rglob("*.py")}
        if actual_python != expected_python:
            raise ValueError("unexpected code in pinned package")
        # Refuse a preloaded producer checkout/other G0 version, rather than
        # silently using whichever same-named package happened to import first.
        for name, module in tuple(sys.modules.items()):
            if name == "rm27_exchange" or name.startswith("rm27_exchange."):
                origin = getattr(module, "__file__", None)
                if origin is None or not Path(origin).resolve().is_relative_to(self.root):
                    raise ValueError("foreign rm27_exchange module already loaded")
        sys.path.insert(0, str(self.root))
        try:
            self.api = importlib.import_module("rm27_exchange.handover")
            self.scenarios = importlib.import_module("rm27_exchange.handover_scenarios")
            self.validation = importlib.import_module("rm27_exchange.validation")
            self.blocked = importlib.import_module("rm27_exchange.offline_contracts").Blocked
        finally:
            sys.path.remove(str(self.root))

    def load(self):
        # Only published projections were acquired; raw source-log verification
        # is unavailable locally and must not be relabelled as completed.
        return self.scenarios.load_scenario_set(self.root, verify_originals=False)

    def replay(self):
        return [self.api.decode_snapshot(self.api.encode(snapshot))
                for snapshot in self.scenarios.replay_snapshots(self.root)]

    def assess(self, snapshot):
        """Inventory original provenance and gaps without filling state channels."""
        self.api.validate_snapshot(snapshot)
        state = snapshot["state"]["record"] if snapshot["state"] is not None else None
        missing = []
        for name in self.api.CHANNELS:
            if state is None or state.get(name) is None:
                missing.append(name)
        for name in ("identity", "registration", "target_geometry"):
            if snapshot[name] is None:
                missing.append(name)
        actuator = state.get("actuator_state") if state else None
        if not actuator or actuator.get("rotor_thrust_N") is None:
            missing.append("actual_rotor_thrust")
        if not actuator or actuator.get("motor_command_applied") is None:
            missing.append("held_motor_target_and_timing")
        if not snapshot["control_history"]["records"]:
            missing.append("control_history")
        actor_missing = [name for name in missing if name in
                         (*self.api.CHANNELS, "identity", "registration", "target_geometry")]
        return {
            "snapshot_id": snapshot["snapshot_id"], "source": deepcopy(snapshot["source"]),
            "state_metadata": deepcopy(state["metadata"]) if state else None,
            "identity": deepcopy(snapshot["identity"]),
            "registration": deepcopy(snapshot["registration"]),
            "lineage": deepcopy(state["metadata"]["lineage"] if state else
                                snapshot["native_record"]["lineage"]),
            "missing": missing,
            "actor_observation": {
                "status": "BLOCKED", "missing": actor_missing,
                "reason": "No receiver-validated runtime/model/world mapping; no GT fallback",
                "feature_layout": FEATURES, "dimension": 45,
                "normalization": "float32 physical features; checkpoint Actor LayerNorm unchanged",
                "history": "delta repeated x10 is not temporal history; actor has no motor/history input",
            },
            "physical_branching": {
                "status": "BLOCKED", "reason":
                "Complete actuator/held-command history, clock/control phase, remaining horizon and "
                "receiver model/target compatibility must be established independently of schema admission",
            },
            "runtime_trusted": False,
        }

    def synthetic_actor_observation(self, snapshot, registry, *, now_s, goal_offset_m):
        """Format test in registered producer world, translated to target centre.

        This is deliberately synthetic-only. Registration inversion alone does
        not establish that producer world/target/model equal the 4090 plant.
        No policy inference, scalar normalizer, history filling or actuator reset.
        """
        if registry.mode != "SYNTHETIC_TEST":
            raise self.blocked("runtime observation mapping is not validated for this release")
        state = self.api.admit_snapshot(snapshot, registry, now_s=now_s)
        target = snapshot["target_geometry"]
        if target is None:
            raise self.blocked("missing registered target geometry; cannot construct Actor goal")
        target = target["record"]
        if target["motion"] != "STATIC" or target["target_axes"] != ["across", "slope", "normal"]:
            raise self.blocked("unsupported target motion/axes")
        target_frame = target["target_frame_WT"]
        q_target = np.asarray(target_frame["rotation_parent_child_wxyz"], dtype=float)
        if not np.isclose(np.linalg.norm(q_target), 1., rtol=0, atol=1e-8):
            raise self.blocked("target quaternion must be unit WXYZ")
        target_rotation = Rotation.from_quat(q_target[[1, 2, 3, 0]]).as_matrix()
        normal = np.asarray(target["surface_normal_W"], dtype=float)
        if not np.allclose(target_rotation[:, 2], normal, atol=1e-8, rtol=0):
            raise self.blocked("target normal disagrees with registered target axes")
        if not np.isfinite(goal_offset_m) or goal_offset_m < 0:
            raise self.blocked("invalid receiver goal offset")
        center = (np.asarray(target_frame["translation_parent_m"], dtype=float)
                  + target_rotation @ np.asarray(target["armor_surface"]["surface_center_T_m"]))
        clock, frame = registry.clock, registry.frame
        stamp = (state["state_time_s"] - clock.offset_s) / clock.scale
        world, _, _ = frame.transforms(stamp, source_id=frame.source_id,
                                      clock_id=clock.source_id, epoch=registry.epoch)
        # The shared position is COM. Both position and target receive the same
        # registered translation, which cancels in the target-centred difference.
        flu_frd = np.diag([1., -1., -1.])
        q = np.asarray(state["q_WB_wxyz"])
        rotation = world.T @ Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix() @ flu_frd
        xyzw = Rotation.from_matrix(rotation).as_quat()
        physical = np.concatenate((world.T @ (np.asarray(state["position_W_m"]) - center),
                                   xyzw[[3, 0, 1, 2]], world.T @ state["velocity_W_m_s"],
                                   flu_frd @ state["body_rates_B_rad_s"]))
        goal = goal_offset_m * (world.T @ normal)
        result = observations(physical[None, :], goal)[0]
        if result.shape != (45,) or not np.isfinite(result).all():
            raise self.blocked("invalid actor features")
        return result

    def fixed_protocol(self):
        """Freeze identities and denominators before any policy is evaluated."""
        dataset = self.load()
        snapshots = self.replay()
        cases = []
        for record, snapshot in zip(dataset["records"], snapshots, strict=True):
            if record["scenario_id"] != snapshot["snapshot_id"]:
                raise self.blocked("scenario/snapshot identity mismatch")
            cases.append({
                "scenario_id": snapshot["snapshot_id"], "split": record["split"],
                "source": deepcopy(record["source"]), "source_line": record["source_line"],
                "snapshot_sha256": self.api.digest(snapshot),
                "eligibility": "BLOCKED", "missing": self.assess(snapshot)["missing"],
                "metrics": {name: "NOT_EVALUATED" for name in (
                    "native_first_contact", "forbidden_contact", "separation", "stable_recovery",
                    "flight_envelope", "timing", "full_task")},
            })
        return {
            "experiment_id": "n1-r9-a", "scope": "OFFLINE_FIXED_HANDOVER_PREPARATION",
            "source_revision": REVISION, "contracts_version": "0.1.0-draft",
            "manifest_sha256": self.receipt["files"][self.scenarios.DEFAULT_MANIFEST]["sha256"],
            "case_selection": "ALL_PUBLISHED_ROWS_IN_ORDER_NO_OUTCOME_SELECTION",
            "evaluation_conditions": {
                "handover_mode": "FIXED_SNAPSHOT_NO_LEARNED_SELECTOR",
                "input_representation": "RECORDED is storage provenance; diagnostic GT is never an estimate",
                "initial_state": "Exact declared physical, actuator, command/history, clock/epoch/control phase",
                "remaining_horizon": "Must be declared per scenario; not inferred or silently restarted",
                "receiver_registration": "Require explicit target/world, geometry, actuator, timing and scorer compatibility",
                "checkpoint_selection": "Register policy/checkpoint/normalizer hashes before outcome evaluation",
                "actor_features": FEATURES, "actor_dimension": 45,
                "actor_normalization": "Existing learned LayerNorm; critic ValueNorm is not observation normalization",
                "actor_history": "Ten copies of current goal delta; no motor, previous action or horizon input",
                "actor_action": "Unchanged tanh [p,q,r,T] -> physical [mg*(1+0.5*T),2*p,2*q,2*r]",
                "source_split": "Family/trajectory disjoint; no train/held-out claim from current dev-only catalog",
                "success": "Report contact, separation, stable recovery, envelope and timing separately; full task requires all",
                "unavailable_metrics": "NOT_EVALUATED, never zero-filled or promoted from local first contact",
            },
            "cases": cases,
            "denominators": {"catalogued": len(cases), "eligible": 0, "blocked": len(cases),
                             "evaluated": 0, "independent_held_out_families": 0},
            "fixed_handover_rule": "Every predeclared valid slice enters denominator; no selector filtering",
            "future_e2e_rule": "All valid navigation attempts; separate acceptance/abort/conditional success",
            "dataset_readiness": dataset["dataset_readiness"], "reasons": dataset["reasons"],
            "originals_verified": dataset["originals_verified"],
            "protocol_scope": "Published REFERENCE_TRAJECTORY offline lease conformance only",
            "direct_ctbr_authority": "BLOCKED_NOT_ESTABLISHED_BY_G0_PROTOCOL",
            "full_task_evaluator": "BLOCKED_CONTACT_ONLY_IS_NOT_COMPLETE_TASK",
            "r9_b": {"mechanism": "UNSELECTED", "training": "NOT_AUTHORIZED", "native": "NOT_AUTHORIZED"},
        }


def main():
    """Offline inspection to stdout; never creates a simulator or invokes policy."""
    consumer = SharedScenarioConsumer()
    print(json.dumps(consumer.fixed_protocol(), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
