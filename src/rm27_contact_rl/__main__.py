"""Fixed-scene baseline, bounded CPU PPO and independent checkpoint evaluation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .baselines import CTBRBaseline
from .env import ContactRecoveryEnv
from .rm27_adapter import DEFAULT_CONFIG
from .paths import ROOT
from .provenance import provenance



def baseline(config, output):
    """Do not search initial states or alter gates if the nominal baseline fails."""
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    with_env = ContactRecoveryEnv(config)
    try:
        obs, reset_info = with_env.reset(seed=0)
        controller = CTBRBaseline(with_env)
        source = provenance(with_env, algorithm="PRIVILEGED_EXISTING_SKILL_PROJECTED_TO_SHARED_CTBR_ENV")
        config_bundle = dict(rl=with_env.rl_config, experiment=with_env.plant.cfg,
                             physical=with_env.plant.physical, vehicle=with_env.plant.vehicle_config,
                             profile=with_env.plant.profile)
        (output / "config.json").write_text(json.dumps(config_bundle, indent=2, allow_nan=False) + "\n")
        rows = []
        def record(observation, reward, info):
            data = with_env.plant.data
            rows.append(dict(time_s=float(data.time), qpos=data.qpos.copy(), qvel=data.qvel.copy(),
                             motor_state_N=data.act.copy(), motor_commands_N=data.ctrl.copy(),
                             observation=observation.copy(), previous_action_ctbr=with_env.previous_action_ctbr.copy(),
                             reward=float(reward), proxy_saturated=info["proxy_saturated"],
                             physics_substeps=info["physics_substeps"], contact_seen=info["contact_seen"],
                             baseline_phase=controller.skill.phase,
                             baseline_teacher_saturated=bool(controller.teacher_saturated)))
        record(obs, 0., reset_info)
        terminated = False
        total_reward = 0.
        while not terminated:
            obs, reward, terminated, truncated, info = controller.step(with_env)
            if truncated:
                raise RuntimeError("I1 task unexpectedly truncated instead of terminated")
            total_reward += reward
            record(obs, reward, info)
        result = info["result"]
        result.update(provenance=source, baseline_privileged=True,
                      nominal_skill_parameters=dict(with_env.plant.cfg["nominal_skill"]),
                      total_event_reward=total_reward, strategy_learning="NOT_RUN",
                      environment_contract_checks="REPORTED_SEPARATELY_IN_TEST_RESULTS")
        np.savez_compressed(output / "trajectory.npz",
                            **{key: np.asarray([row[key] for row in rows]) for key in rows[0]})
        (output / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(json.dumps(dict(passed=result["passed"], failure_modes=result["failure_modes"],
                              episode_elapsed_s=result["episode_elapsed_s"], output=str(output)),
                         allow_nan=False))
        # Exit 2 is a recorded task failure; unhandled execution errors retain
        # their traceback and Python's exit 1 rather than becoming false evidence.
        return 0 if result["passed"] else 2
    finally:
        with_env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("baseline", help="one fixed nominal 150 g privileged CTBR round")
    run.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    run.add_argument("--output", type=Path, required=True, help="new persistent evidence directory")
    arrival_baseline = commands.add_parser("arrival-baseline", help="nine fixed terminal-arrival traditional comparisons")
    arrival_baseline.add_argument("--config", type=Path,
                                  default=ROOT / "configs/agile_flight/terminal_arrival.json")
    arrival_baseline.add_argument("--output", type=Path, required=True)
    train_parser = commands.add_parser("train", help="bounded CPU PPO with task-specific evaluation")
    train_parser.add_argument("--config", type=Path, default=None)
    train_parser.add_argument("--output", type=Path, required=True)
    evaluate_parser = commands.add_parser("evaluate", help="task-specific deterministic evaluation, frozen normalization")
    evaluate_parser.add_argument("--checkpoint", type=Path, required=True)
    evaluate_parser.add_argument("--output", type=Path, required=True)
    evaluate_parser.add_argument("--task", choices=("contact_recovery_v1", "agile_terminal_arrival_v1"),
                                 help="reject checkpoints belonging to a different task")
    importer = commands.add_parser("track-import", help="replay finished run directories into Weights & Biases")
    importer.add_argument("runs", type=Path, nargs="+", help="training result directories")
    importer.add_argument("--project", default=None)
    importer.add_argument("--entity", default=None)
    importer.add_argument("--mode", choices=("online", "offline"), default=None)
    importer.add_argument("--robustness-dir", type=Path, default=None,
                          help="optional <run-name>.jsonl robustness rows to attach as summary")
    args = parser.parse_args()
    if args.command == "track-import":
        from .tracking import DEFAULT_PROJECT, import_run
        for run in args.runs:
            rows = None
            if args.robustness_dir and (args.robustness_dir / f"{run.name}.jsonl").is_file():
                rows = [json.loads(line) for line in (args.robustness_dir / f"{run.name}.jsonl").read_text().splitlines() if line]
            print(json.dumps(dict(run=run.name, wandb_id=import_run(run, project=args.project or DEFAULT_PROJECT,
                                                                     entity=args.entity, mode=args.mode, robustness=rows))))
        return 0
    if args.command == "baseline":
        return baseline(args.config, args.output)
    # Baseline/environment imports do not require training libraries.
    from .ppo import DEFAULT_TRAIN_CONFIG, evaluate, evaluate_arrival_baseline, train
    if args.command == "arrival-baseline":
        result = evaluate_arrival_baseline(args.config, args.output)
        print(json.dumps({key: result[key] for key in ("success_count", "episode_count", "failure_counts")}))
        return 0 if result["passed"] else 2
    if args.command == "train":
        return train(args.config or DEFAULT_TRAIN_CONFIG, args.output)
    result = evaluate(args.checkpoint, args.output, expected_task=args.task)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
