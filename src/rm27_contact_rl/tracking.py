"""Optional Weights & Biases tracking of Arrival/Contact training runs.

One flattening of the trainer's per-update evidence serves both live logging
(``RM27_WANDB=1``, set by ``docker/run.sh --wandb``) and importing finished run
directories. ``wandb`` is imported only when tracking is used, so ordinary
training, evaluation and tests never load it. Tracking reads evidence; it does
not change training, evaluation or the saved results.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

DEFAULT_PROJECT = "rm27-agile-flight-rl"


def update_metrics(evidence, chunk_transitions):
    """Flatten one ``updates.jsonl`` record into scalar W&B metrics."""
    episodes = evidence["episodes"]
    metrics = {"updates/completed": evidence["completed_rollout_updates"],
               "time/learning_wall_s": evidence["learning_wall_s"],
               "episodes/completed": episodes["completed"], "episodes/successful": episodes["successful"]}
    if episodes["completed"]:
        metrics["episodes/success_rate"] = episodes["successful"] / episodes["completed"]
        metrics["episodes/mean_return"] = episodes["mean_return"]
    for reason, count in episodes["failure_counts"].items():
        metrics[f"failures/{reason}"] = count
    for name, value in evidence["reward_components"].items():
        metrics[f"reward/{name}"] = value
    metrics["actions/normalized_clip_rate"] = evidence["normalized_action_clipped_steps"] / chunk_transitions
    metrics["actions/proxy_saturation_rate"] = evidence["proxy_saturated_steps"] / chunk_transitions
    metrics["actions/input_clip_rate"] = evidence["action_clipped_steps"] / chunk_transitions
    for axis, count in enumerate(evidence.get("normalized_action_clipped_steps_per_axis") or []):
        metrics[f"actions/clip_rate_axis{axis}"] = count / chunk_transitions
    for axis, fraction in enumerate((evidence.get("policy_mean_on_rollout") or {}).get("outside_bounds_fraction", [])):
        metrics[f"policy/mean_outside_bounds_axis{axis}"] = fraction
    curriculum = evidence.get("informed_reset")
    if curriculum:
        for key in ("lead_s", "window_success_rate", "informed_completed", "informed_successful",
                    "full_start_completed", "full_start_successful", "prefix_failures", "reference_flights"):
            if curriculum.get(key) is not None:
                metrics[f"curriculum/{key}"] = curriculum[key]
        metrics["curriculum/advanced"] = int(curriculum["advanced"])
        if curriculum["full_start_completed"]:
            metrics["curriculum/full_start_success_rate"] = (curriculum["full_start_successful"]
                                                             / curriculum["full_start_completed"])
        if curriculum["informed_completed"]:
            metrics["curriculum/informed_success_rate"] = (curriculum["informed_successful"]
                                                           / curriculum["informed_completed"])
    for name, value in evidence["metrics"].items():
        if value is not None:
            metrics[name] = value
    return metrics


def run_config(training, rl=None):
    """W&B config: the training configuration plus labels used to group runs."""
    labels = dict(algorithm=training.get("algorithm", "PPO"),
                  curriculum=bool(training.get("informed_reset")),
                  history_steps=(training.get("observation_history") or {}).get("steps", 1),
                  dynamics_randomization=bool(training.get("dynamics_randomization")),
                  env_config=Path(training["env_config"]).stem)
    if rl is not None:
        labels["terminal"] = rl.get("terminal")
        labels["reward"] = rl.get("reward")
    return dict(labels, training=training)


def run_tags(config):
    tags = [config["algorithm"], "curriculum" if config["curriculum"] else "no-curriculum", config["env_config"]]
    if config["history_steps"] > 1:
        tags.append(f"history{config['history_steps']}")
    if config["dynamics_randomization"]:
        tags.append("dynamics-randomization")
    return tags


def summary_metrics(summary, run_dir=None, robustness=None):
    """Final scalars: stop condition, budget use, the three evaluations and optional robustness."""
    values = {key: summary.get(key) for key in ("stop_reason", "training_transitions",
                                                  "completed_rollout_updates", "learning_wall_s")}
    for group in ("before", "after", "traditional"):
        brief = summary.get(group) or {}
        if "success_count" in brief:
            values[f"eval/{group}_success"] = brief["success_count"]
            values[f"eval/{group}_episodes"] = brief["episode_count"]
    if run_dir is not None:
        times = [json.loads(path.read_text()).get("arrival_time_s")
                 for path in sorted(Path(run_dir).glob("evaluation_after/episode_*/summary.json"))]
        times = [t for t in times if t is not None]
        if times:
            values["eval/after_arrival_time_mean_s"] = sum(times) / len(times)
            values["eval/after_arrival_time_max_s"] = max(times)
    for row in robustness or []:
        name = row["condition"].replace(" ", "_").replace("/", "_")
        values[f"robust/{name}/declared_success"] = int(row["declared"].split("/")[0])
        values[f"robust/{name}/held_out_success"] = int(row["held_out"].split("/")[0])
    return values


def _run_id(name):
    return hashlib.sha1(name.encode()).hexdigest()[:16]


class LiveTracker:
    """Log a training run while it executes; one W&B run per results directory."""

    def __init__(self, output, training, rl, *, project=None, entity=None):
        import wandb
        config = run_config(training, rl)
        self.run = wandb.init(project=project or os.environ.get("RM27_WANDB_PROJECT", DEFAULT_PROJECT),
                              entity=entity or os.environ.get("WANDB_ENTITY") or None,
                              name=Path(output).name, id=_run_id(Path(output).name), dir=str(output),
                              group=config["env_config"], tags=run_tags(config), config=config,
                              job_type="train", resume="never")

    def log_update(self, evidence, chunk_transitions):
        self.run.log(update_metrics(evidence, chunk_transitions), step=int(evidence["state"]["num_timesteps"]))

    def finish(self, summary, run_dir):
        self.run.summary.update(summary_metrics(summary, run_dir))
        self.run.finish()


def live_tracker(output, training, rl):
    """Tracker when the launcher enabled W&B, else None; failures are reported, not fatal."""
    if os.environ.get("RM27_WANDB") != "1":
        return None, None
    try:
        return LiveTracker(output, training, rl), None
    except Exception as exc:  # A tracking outage must not invalidate the experiment.
        return None, f"{type(exc).__name__}: {exc}"


def import_run(run_dir, *, project=DEFAULT_PROJECT, entity=None, mode=None, robustness=None):
    """Replay a finished run directory's saved evidence into one W&B run."""
    import wandb
    run_dir = Path(run_dir).resolve()
    saved = json.loads((run_dir / "config.json").read_text())
    summary = json.loads((run_dir / "summary.json").read_text())
    training = saved["training"]
    config = run_config(training, saved.get("rl"))
    chunk = training["n_envs"] * (training["sac"]["chunk_steps"] if training.get("algorithm") == "SAC"
                                  else training["ppo"]["n_steps"])
    run = wandb.init(project=project, entity=entity, name=run_dir.name, id=_run_id(run_dir.name),
                     dir=str(run_dir), group=config["env_config"], tags=run_tags(config) + ["imported"],
                     config=config, job_type="train-import", resume="never", mode=mode)
    try:
        with (run_dir / "updates.jsonl").open() as stream:
            for line in stream:
                evidence = json.loads(line)
                run.log(update_metrics(evidence, chunk), step=int(evidence["state"]["num_timesteps"]))
        run.summary.update(summary_metrics(summary, run_dir, robustness))
    finally:
        run.finish()
    return run.id
