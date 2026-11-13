"""Run the instrumented, transition-matched Phase R2 comparison."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import signal
import sys
import time
import traceback
from types import SimpleNamespace

import numpy as np

from . import terminal_workflow as workflow
from .asset import ROOT, digest, read_json, write_result
from .terminal_curriculum import BANDS, CampaignLedger, ResetSampler
from simpleflight_lab.stage1_training import LearningBudget, LearningLimit

CONFIG = ROOT / "configs/rm27_asm2_rotation_bridge_instrumented.json"


def configuration(path):
    experiment = read_json(path)
    n = experiment["per_arm_transitions"]
    if experiment["schema"] != "rm27_asm2_rotation_bridge_instrumented_v1" or n != 98_304:
        raise ValueError("Expected exactly 98,304 transitions per arm")
    base, archived = workflow.configuration(ROOT / experiment["base_config"])
    output = ROOT / experiment["output"]
    if output == archived:
        raise ValueError("A new comparison requires its own output directory")
    cfg = dict(base, config_path=str(path.relative_to(ROOT)), output=experiment["output"],
               budget=dict(per_arm_transitions=n, total_transitions=2*n), diagnostics=experiment)
    return cfg, archived, output


class TransitionLedger(CampaignLedger):
    def remaining(self, arm):
        rows = self.data["arms"]
        n = min(self.caps["per_arm_transitions"] - rows[arm]["transitions"],
                self.caps["total_transitions"] - sum(r["transitions"] for r in rows.values()))
        return n, float("inf")

    def reserve(self, arm, transitions):
        if transitions > self.remaining(arm)[0]:
            raise LearningLimit("TRANSITION_LIMIT")
        row = self.data["arms"][arm]
        if row["pending"] is not None:
            raise RuntimeError("Existing unsettled PPO update")
        row["transitions"] += transitions
        row["pending"] = dict(transitions=transitions, learning_s=0., started_unix_s=time.time())
        self.flush()
        return 0.


class TransitionBudget(LearningBudget):
    def check(self):
        if self.stop_reason is not None:
            raise LearningLimit(self.stop_reason)


@contextmanager
def training_budget():
    # Scope the stopping rule to this process. Keep the pinned workflow and
    # resume format available for historical experiments with learning deadlines.
    old = workflow.CampaignLedger, workflow.LearningBudget, workflow.signal
    workflow.CampaignLedger, workflow.LearningBudget = TransitionLedger, TransitionBudget
    workflow.signal = SimpleNamespace(signal=signal.signal, SIGTERM=signal.SIGTERM, SIGALRM=signal.SIGALRM,
                                      ITIMER_REAL=signal.ITIMER_REAL, setitimer=lambda *_: None)
    try:
        yield
    finally:
        workflow.CampaignLedger, workflow.LearningBudget, workflow.signal = old


def certify_bank(cfg, archived, output):
    """Recheck only the archived nominal bridge training rows under the frozen variants."""
    from . import terminal_native_eval as native
    from .handover_curriculum import native_verifier
    from .policy_frontier import perturbed_state
    from .rotation_bridge import RESET_KEYS
    from .terminal_workflow_final import legacy_adapters

    base, _ = workflow.configuration(ROOT / cfg["diagnostics"]["base_config"])
    old = workflow.verify_freeze(base, archived)
    receipt = read_json(archived / "bridge-bank.json")
    frontier = read_json(ROOT / cfg["bridge"]["source_run"] / "manifest.json")
    _, profile, spec, manifest_path, manifest, _ = native.load_evaluation_inputs(ROOT / cfg["task_config"])
    train = workflow.load_bank(archived)
    indices = np.flatnonzero(train["band"] == "rotation_bridge")
    settings = receipt["design"]["perturbation"]
    tangent = spec.target_rotation[:, settings["axis_target_frame"]]
    names = list(settings["variants"])
    probes = {key: np.repeat(train[key][indices], len(names), axis=0) for key in RESET_KEYS}
    for i, row in enumerate(indices):
        for j, sign in enumerate(settings["variants"].values()):
            probes["states"][i*len(names)+j] = perturbed_state(
                train["states"][row], tangent, sign, settings["position_m"], settings["velocity_m_s"])
    with np.load(archived / "bridge-validation.npz", allow_pickle=False) as data:
        if set(train["case_ids"]) & set(data["case_ids"]):
            raise ValueError("Training and reserved development witness sources overlap")
    legacy, _ = legacy_adapters((ROOT / frontier["controllers"]["phase4"]["checkpoint"]).parents[1])
    capped = legacy["load_action_limited_deterministic"](
        manifest_path.parent, manifest, [("bridge-training-certification", probes, range(len(probes["states"])))],
        profile, spec)
    expected = receipt["deterministic"]
    training_reference = dict(expected, sources={case: expected["sources"][case] for case in capped[0]})
    if capped[1] != training_reference or expected != frontier["controllers"]["deterministic"]:
        raise ValueError("Capped feasibility reference changed")
    plant = native.NativeContact(profile, spec)
    verify = native_verifier(plant)
    limits = read_json(native.HANDOVER_CONFIG)["flight_envelope"]
    jobs = [("deterministic", "bridge-training-certification", probes, i,
             manifest["cases"][str(case)]["first_contact_sample_s"]) for i, case in enumerate(probes["case_ids"])]
    for i, state in enumerate(probes["states"]):
        if (not verify(state, probes["motor_thrust_N"][i], probes["motor_target_N"][i])["passed"]
                or native.flight_limits(state, spec, limits) is not None):
            raise ValueError(f"Invalid training-bank certification reset {i}")
    with ProcessPoolExecutor(max_workers=cfg["workers"], mp_context=multiprocessing.get_context("fork"),
            initializer=native.init_worker, initargs=(plant, {"deterministic": capped},
                {"handover": frontier["horizon_s"]}, limits, False)) as pool:
        outcomes = [row for row, _ in pool.map(native.worker_episode, jobs)]
    keep = train["band"] != "rotation_bridge"
    checks = []
    for i, row in enumerate(indices):
        trials = [dict(variant=name, **outcome) for name, outcome in zip(
            names, outcomes[i*len(names):(i+1)*len(names)], strict=True)]
        keep[row] = all(t["qualified"] and not t["outside_actor_action_map_controls"] and
                        not t["saturation_controls"] for t in trials)
        checks.append(dict(case=str(train["case_ids"][row]), frame=int(train["source_frame"][row]),
                           kept=bool(keep[row]), trials=trials))
    write_result(output / "training-bank-certification.json", dict(
        archived_pins=old["archives"], reference=capped[1], perturbation=settings, candidates=checks))
    if len(set(train["case_ids"][keep & (train["band"] == "rotation_bridge")])) < 2:
        raise ValueError("Insufficient robust earlier resets across training witnesses")
    filtered = {k: v[keep] if v.ndim else v for k, v in train.items()}
    with (output / "training.npz").open("xb") as f:
        np.savez_compressed(f, **filtered)
    shutil.copyfile(archived / "bridge-validation.npz", output / "bridge-validation.npz")
    effective = {}
    for arm in ("A", "B"):
        weights = ResetSampler(filtered["band"], "fixed", cfg["seed"], workflow.sampler_settings(cfg, arm)).weights()
        np.testing.assert_allclose(weights.sum(), 1., rtol=0., atol=1e-12)
        effective[arm] = {b: float(weights[filtered["band"] == b].sum()) for b in BANDS}
        np.testing.assert_allclose(list(effective[arm].values()),
            [cfg["bridge"]["mixtures"][arm].get(b, 0.) for b in BANDS], rtol=0., atol=1e-12)
    return effective


def freeze(cfg, archived, output):
    from . import rotation_bridge_diagnostics, rotation_bridge_metrics
    if (output / "freeze.json").exists():
        verify_freeze(cfg, output)
        return
    output.mkdir(parents=True, exist_ok=False)
    experiment = cfg["diagnostics"]
    protocol = ROOT / experiment["transition_protocol"]
    if digest(protocol) != experiment["transition_protocol_sha256"]:
        raise ValueError("Frozen transition region changed")
    effective = certify_bank(cfg, archived, output)
    paths = (Path(__file__), Path(rotation_bridge_diagnostics.__file__), Path(rotation_bridge_metrics.__file__),
             ROOT / "run_asm2_terminal_pilot.sh", protocol, ROOT / experiment["base_config"], archived / "freeze.json")
    write_result(output / "freeze.json", dict(
        config_sha256=digest(ROOT / cfg["config_path"]),
        archives={n: digest(output / n) for n in (
            "training.npz", "bridge-validation.npz", "training-bank-certification.json")},
        source_pins={str(p.relative_to(ROOT)): digest(p) for p in paths},
        effective_mixtures=effective, seed=cfg["seed"], budget=cfg["budget"], wall_clock_limit=None,
        legacy_development=workflow.development_module(cfg)["load_protocol"](False)[-1],
        scientific_decision=rotation_bridge_diagnostics.SCIENTIFIC_RULE,
        strict_promotion=cfg["bridge"]["gate"], endpoint=cfg["budget"]["per_arm_transitions"],
        frozen_unix_s=time.time()))


def verify_freeze(cfg, output):
    frozen = workflow.verify_freeze(cfg, output)
    for name, pin in frozen["source_pins"].items():
        if digest(ROOT / name) != pin:
            raise ValueError(f"Instrumented R2 source changed after freeze: {name}")
    return frozen


def run(cfg, output):
    from .rotation_bridge_diagnostics import native_transition_diagnostics
    experiment = cfg["diagnostics"]
    started, clock = time.time(), time.perf_counter()
    status = "interrupted"
    try:
        with native_transition_diagnostics(ROOT / experiment["transition_protocol"], experiment["transition_protocol_sha256"]):
            workflow.host_run(cfg, output)
        status = "complete"
    finally:
        with (output / "wall-clock.jsonl").open("a") as f:
            f.write(json.dumps(dict(start_unix_s=started, wall_s=time.perf_counter()-clock, status=status)) + "\n")


def train(cfg, output, arm, resume, stop_updates):
    from isaaclab.app import AppLauncher
    launcher = AppLauncher(headless=True)
    code = 0
    try:
        import torch
        from .rotation_bridge_metrics import install_metrics
        folder = output / arm
        checkpoint = torch.load(folder / "resume.pt", map_location="cpu", weights_only=True) if resume else {}
        with training_budget(), install_metrics(folder,
                completed_transitions=checkpoint.get("completed_transitions", 0),
                updates=checkpoint.get("workflow_updates", 0)):
            workflow.train(cfg, output, arm, resume, stop_updates)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workflow-config", type=Path, default=CONFIG)
    parser.add_argument("command", choices=("freeze", "run", "train", "final"))
    parser.add_argument("--arm", choices=("A", "B"))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--stop-updates", type=int, default=0)
    args = parser.parse_args()
    cfg, archived, output = configuration(args.workflow_config.resolve())
    if args.command == "freeze":
        freeze(cfg, archived, output)
        return
    verify_freeze(cfg, output)
    if args.command == "run":
        run(cfg, output)
    elif args.command == "train":
        if args.arm is None:
            parser.error("train requires --arm")
        train(cfg, output, args.arm, args.resume, args.stop_updates)
    else:
        from .rotation_bridge_diagnostics import finalize_diagnostics
        from .rotation_bridge_report import finalize
        finalize_diagnostics(cfg, output, finalize(cfg, output))


if __name__ == "__main__":
    main()
