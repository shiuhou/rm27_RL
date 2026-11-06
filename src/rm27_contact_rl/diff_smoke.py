"""One bounded CPU E1-A integration check; this is not an E1-B trainer.

The original MuJoCo environment and ArrivalEvaluator own every event. Counters
reserve five physical intervals before each control, including partial controls.
The only accepted protocol is the checked-in engineering smoke configuration.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import time

import gymnasium as gym
import numpy as np
import torch

from rm27_contact_physics.arrival import ArrivalEvaluator
from .arrival_env import AgileTerminalArrivalEnv
from .diff_analysis import freeze_sources
from .diff_dynamics import DynamicsParameters, step_control
from .diff_policy import (ActorCritic, load_checkpoint,
                          physical_action, rollout_window, save_checkpoint)
from .diff_validation import Budget, pack, sha
from .paths import ROOT
from .ppo import (HOVER_CENTERED, NormalizedCTBR, _arrival_episode,
                  _arrival_initials, _arrival_summary, _state_hash)


CONFIG = ROOT / "configs/agile_flight/diff_e1a.json"


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def flat(parameters, *, gradients=False):
    return torch.cat([(p.grad if gradients else p).detach().reshape(-1)
                      for p in parameters])


def assign(parameters, vector):
    with torch.no_grad():
        offset = 0
        for p in parameters:
            p.copy_(vector[offset:offset + p.numel()].reshape_as(p))
            offset += p.numel()


def batch(env, policy, config, budget):
    """Fixed fresh resets; no reference actions, informed resets or imitation."""
    windows = []
    for index in range(config["batch_size"]):
        env.reset(seed=config["seed"] + index)
        windows.append(rollout_window(env, policy, steps=config["window_steps"],
                                     gamma=config["gamma"], budget=budget))
    return windows


def loss_of(windows):
    return -torch.stack([w.objective for w in windows]).mean()


class SmokeBudget(Budget):
    def __init__(self, config):
        super().__init__(config)
        self.actual_controls = self.actual_substeps = 0

    def summary(self):
        return dict(super().summary(), actual_reference_controls=self.actual_controls,
                    actual_physics_substeps=self.actual_substeps)


class CountedArrivalEnv(AgileTerminalArrivalEnv):
    def __init__(self, config, budget):
        super().__init__(config)
        self.smoke_budget = budget

    def step(self, action):
        result = super().step(action)
        self.smoke_budget.actual_controls += 1
        self.smoke_budget.actual_substeps += int(result[4]["physics_substeps"])
        return result


class MeteredEvaluation(gym.Wrapper):
    def __init__(self, env, budget):
        super().__init__(env)
        self.budget = budget
        self.actual_substeps = 0

    def step(self, action):
        self.budget.tick(controls=1, substeps=5)
        result = self.env.step(action)
        self.actual_substeps += int(result[4]["physics_substeps"])
        return result


class RawObservation:
    @staticmethod
    def normalize_obs(observation):
        # ActorCritic owns its fixed, checkpointed normalization of these same
        # 26 observation fields. This adapter does not expose the internal state.
        return observation


def evaluate(policy, config, budget, output, *, center_only=False):
    base = CountedArrivalEnv(ROOT / config["task_config"], budget)
    episode = MeteredEvaluation(NormalizedCTBR(base, HOVER_CENTERED), budget)
    try:
        output.mkdir(parents=True, exist_ok=False)
        initials = _arrival_initials(base)
        if center_only:
            initials = initials[:1]
        episodes = [_arrival_episode(episode, output / f"episode_{i:02d}", coordinates,
                                    model=policy, vec=RawObservation(), seed=config["seed"])
                    for i, coordinates in enumerate(initials)]
        result = _arrival_summary(episodes)
        result.update(interface_status="PASS", actor_observation_fields=26,
                      algorithm="ANALYTIC_BPTT_E1A_CPU", actual_substeps=episode.actual_substeps)
        write_json(output / "summary.json", result)
        return result
    finally:
        episode.close()


def probes(env, policy, config, budget):
    """Original scorer events, including complete-control events and deadline."""
    hover = copy.deepcopy(policy)
    with torch.no_grad():
        for p in hover.actor.parameters():
            p.zero_()
    records = []
    geometry = env.geometry
    for name, distance in (("arrival_substep_2", .0425), ("arrival_substep_5", .0505),
                           ("safety_substep_5", None), ("initial_contact", None)):
        env.reset(seed=config["seed"])
        if name.startswith("arrival"):
            env.plant.reset(position=geometry["position_target"] + distance * geometry["normal"],
                            rotation=geometry["target_rotation"], velocity=geometry["velocity_target"])
        elif name.startswith("safety"):
            env.plant.reset(position=[1.3955, 0., 2.], velocity=[1., 0., 0.])
        else:
            env.plant.reset(position=[0., 0., .01])
        env.evaluator = ArrivalEvaluator(env.rl_config, env.plant.physical, env.episode_start)
        env.evaluator.initialize(env.plant.model, env.plant.data)
        window = rollout_window(env, hover, steps=16, gamma=config["gamma"], budget=budget)
        expected = [] if name == "initial_contact" else [2 if name.endswith("2") else 5]
        require(window.terminated and window.substeps == expected,
                f"Incorrect authoritative event mask: {name}")
        require(float(window.bootstrap) == 0., "Terminal bootstrap must be zero")
        records.append(dict(name=name, substeps=window.substeps,
                            bootstrap=float(window.bootstrap.detach()),
                            reasons=sorted(env.evaluator.reasons), arrived=env.evaluator.arrived,
                            time_s=float(env.plant.data.time), diagnostics=window.diagnostics))
    env.reset(seed=config["seed"])
    deadline = None
    for _ in range(38):  # ceil(3 seconds / 80 ms), without resetting history
        window = rollout_window(env, hover, steps=16, gamma=config["gamma"], budget=budget)
        if window.terminated:
            deadline = window
            break
    require(deadline is not None and "ARRIVAL_TIMEOUT" in env.evaluator.reasons,
            "Hover must reach the original deadline without an artificial reset")
    require(float(deadline.bootstrap) == 0., "Task timeout must not bootstrap")
    records.append(dict(name="original_3s_timeout", time_s=float(env.plant.data.time),
                        bootstrap=float(deadline.bootstrap.detach()),
                        reasons=sorted(env.evaluator.reasons),
                        completed_intervals=env.evaluator.completed_intervals,
                        diagnostics=deadline.diagnostics))
    env.reset(seed=config["seed"])
    params = DynamicsParameters.from_env(env)
    x = torch.tensor(pack(env.plant), dtype=torch.float64)
    normalized = torch.tensor([-.8, 0., 0., 0.], dtype=torch.float64, requires_grad=True)
    result = step_control(x, physical_action(normalized, env), params)
    derivative = torch.autograd.grad(result.state[:3].sum(), normalized)[0]
    require(float(derivative[0]) == 0., "Strict collective clipping must retain its zero derivative")
    saturated = torch.tensor([.8, 1., -1., 1.], dtype=torch.float64)
    result = step_control(x, physical_action(saturated, env), params)
    raw = result.raw_motor_commands.detach().numpy()
    require(bool(np.any(raw < 0.) or np.any(raw > env.plant.maximum)),
            "Motor saturation probe did not exercise a clipped branch")
    records.append(dict(name="strict_saturation", clipped_collective_derivative=float(derivative[0]),
                        raw_motor_commands=raw.tolist(),
                        clipped_motor_commands=result.motor_commands.detach().tolist()))
    return records


def run(output):
    config = json.loads(CONFIG.read_text())
    require(config["protocol"] == "E1-A_ENGINEERING_ONLY"
            and config["window_steps"] == 16 and config["actor_updates"] <= 3
            and config["critic_updates_per_actor"] <= 2 and config["batch_size"] <= 8
            and config["reference_transition_cap"] <= 8192
            and config["physics_substep_cap"] <= 40960 and config["wall_seconds_cap"] <= 300
            and config["cpu_threads"] == 1, "Configuration exceeds the E1-A smoke authorization")
    output = Path(output).resolve()
    if output.exists():
        require({p.name for p in output.iterdir()} == {"inputs.json"},
                "E1-A output already contains a run; never overwrite or resume it")
    else:
        output.mkdir(parents=True, exist_ok=False)
    inputs_path = output / "inputs.json"
    if inputs_path.exists():
        protected = json.loads(inputs_path.read_text())["protected_sha256"]
    else:
        original = json.loads((ROOT / config["e0_audit"] / "inputs.json").read_text())
        frozen = json.loads((ROOT / config["e0_stratified"] / "sources.json").read_text())
        protected = original["protected_sha256"] | {p: v["sha256"] for p, v in frozen["content"].items()}
        write_json(inputs_path, dict(protocol=config["protocol"], protected_sha256=protected))
    require(all(Path(p).is_file() and sha(p) == h for p, h in protected.items()),
            "Protected E0 / pre-existing input changed before the smoke")
    torch.set_num_threads(1)
    torch.manual_seed(config["seed"])
    budget = SmokeBudget(config)
    env = CountedArrivalEnv(ROOT / config["task_config"], budget)
    env.reset(seed=config["seed"])
    extra = [CONFIG, Path(__file__), ROOT / "src/rm27_contact_rl/diff_policy.py",
             ROOT / "tests/test_diff_policy.py"]
    freeze_sources(env, dict(protected_sha256=protected | {str(p): sha(p) for p in extra}), output)
    write_json(output / "preregistered.json", config)
    policy = ActorCritic(env, hidden_size=config["hidden_size"])
    actor_parameters = list(policy.actor.parameters())
    critic_parameters = list(policy.critic.parameters())
    actor_optimizer = torch.optim.SGD(actor_parameters, lr=config["actor_learning_rate"])
    critic_optimizer = torch.optim.SGD(critic_parameters, lr=config["critic_learning_rate"],
                                       momentum=config["critic_momentum"])
    report = dict(protocol=config["protocol"], status="RUNNING", passed=False,
                  updates=[], e1b_training="NOT_RUN", source_mutation=False,
                  cost_accounting="Five substeps reserved before each reference control; partial controls overcharged")
    initial_actor = flat(actor_parameters).clone()
    initial_critic = flat(critic_parameters).clone()
    try:
        for update in range(config["actor_updates"]):
            budget.tick()
            policy.zero_grad(set_to_none=True)
            started = time.monotonic()
            windows = batch(env, policy, config, budget)
            rollout_seconds = time.monotonic() - started
            loss = loss_of(windows)
            require(bool(torch.isfinite(loss)), "Nonfinite actor objective")
            if update == 0:
                endpoint_gradient, = torch.autograd.grad(windows[0].bootstrap, windows[0].final_observation,
                                                        retain_graph=True)
                require(bool(torch.isfinite(endpoint_gradient).all()) and float(endpoint_gradient.norm()) > 0.,
                        "Critic bootstrap must preserve the endpoint-observation gradient")
                report["bootstrap_endpoint_gradient_norm"] = float(endpoint_gradient.norm())
            started = time.monotonic()
            loss.backward()
            backward_seconds = time.monotonic() - started
            gradient = flat(actor_parameters, gradients=True)
            require(bool(torch.isfinite(gradient).all()) and float(gradient.norm()) > 1e-12,
                    "Actor gradient must be finite and nonzero")
            require(all(p.grad is None for p in critic_parameters), "Actor backward leaked into critic weights")
            if update == 0:
                before = flat(actor_parameters).clone()
                direction = gradient / gradient.norm()
                values = []
                try:
                    for sign in (1, -1):
                        assign(actor_parameters, before + sign * config["direction_epsilon"] * direction)
                        with torch.no_grad():
                            values.append(float(loss_of(batch(env, policy, config, budget))))
                finally:
                    assign(actor_parameters, before)
                finite_difference = (values[0] - values[1]) / (2 * config["direction_epsilon"])
                autodiff = float(gradient @ direction)
                error = abs(finite_difference - autodiff)
                accepted = error <= (config["direction_absolute_tolerance"]
                                     + config["direction_relative_tolerance"] * abs(finite_difference))
                report["parameter_direction"] = dict(autodiff=autodiff, finite_difference=finite_difference,
                                                     absolute_error=error, passed=accepted)
                require(accepted, "Actor parameter-direction finite difference disagrees")
            before = flat(actor_parameters).clone()
            torch.nn.utils.clip_grad_norm_(actor_parameters, config["gradient_clip"], error_if_nonfinite=True)
            actor_optimizer.step()
            delta = flat(actor_parameters) - before
            directional_change = float(gradient @ delta)
            require(float(delta.norm()) > 0. and directional_change < 0., "Actor step is not a descent direction")
            with torch.no_grad():
                after_loss = float(loss_of(batch(env, policy, config, budget)))
            require(after_loss <= float(loss.detach()) + 1e-9, "Same-batch frozen-critic actor objective increased")
            observations = torch.stack([w.initial_observation.detach() for w in windows])
            targets = torch.stack([w.critic_target for w in windows]).detach()
            require(not targets.requires_grad, "Critic targets must be detached")
            value_before = torch.stack([policy.value(o) for o in observations])
            mse_before = float(torch.mean((value_before.detach() - targets) ** 2))
            actor_snapshot = flat(actor_parameters).clone()
            critic_gradient_norms = []
            for _ in range(config["critic_updates_per_actor"]):
                budget.tick()
                policy.zero_grad(set_to_none=True)
                values = torch.stack([policy.value(o) for o in observations])
                critic_loss = torch.mean((values - targets) ** 2)
                critic_loss.backward()
                cg = flat(critic_parameters, gradients=True)
                require(bool(torch.isfinite(cg).all()), "Nonfinite critic gradient")
                critic_gradient_norms.append(float(cg.norm()))
                require(all(p.grad is None for p in actor_parameters), "Critic fitting leaked into actor")
                torch.nn.utils.clip_grad_norm_(critic_parameters, config["gradient_clip"], error_if_nonfinite=True)
                critic_optimizer.step()
            require(torch.equal(flat(actor_parameters), actor_snapshot), "Critic fitting changed actor weights")
            with torch.no_grad():
                mse_after = float(torch.mean((torch.stack([policy.value(o) for o in observations]) - targets) ** 2))
            require(mse_after < mse_before, "Critic did not reduce its fixed-target error")
            report["updates"].append(dict(index=update + 1, loss_before=float(loss.detach()), loss_after=after_loss,
                                           actor_gradient_norm=float(gradient.norm()), actor_delta_norm=float(delta.norm()),
                                           gradient_dot_delta=directional_change, critic_mse_before=mse_before,
                                           critic_mse_after=mse_after, critic_gradient_norms=critic_gradient_norms,
                                           bootstrap_values=[float(w.bootstrap.detach()) for w in windows],
                                           terminated=[w.terminated for w in windows],
                                           rollout_seconds=rollout_seconds, backward_seconds=backward_seconds,
                                           diagnostics=[w.diagnostics for w in windows]))
            write_json(output / "summary.json", report)
        report["probes"] = probes(env, policy, config, budget)
        env.reset(seed=config["seed"])
        checkpoint = output / "checkpoint"
        checkpoint.mkdir(exist_ok=False)
        save_checkpoint(checkpoint / "policy.pt", policy, env,
                        metadata=dict(protocol=config["protocol"], actor_updates=len(report["updates"]),
                                      critic_updates=2 * len(report["updates"]), config=config))
        torch.save(dict(actor=actor_optimizer.state_dict(), critic=critic_optimizer.state_dict()),
                   checkpoint / "optimizers.pt")
        restored = load_checkpoint(checkpoint / "policy.pt", env)
        require(all(torch.equal(v, restored.state_dict()[k]) for k, v in policy.state_dict().items()),
                "Checkpoint actor/critic/normalizer state changed on load")
        probe_observation = env._observation()
        require(np.array_equal(policy.predict(probe_observation)[0], restored.predict(probe_observation)[0]),
                "Checkpoint inference probe changed")
        restored_optimizers = torch.load(checkpoint / "optimizers.pt", weights_only=True)
        restored_actor_optimizer = torch.optim.SGD(restored.actor.parameters(), lr=config["actor_learning_rate"])
        restored_critic_optimizer = torch.optim.SGD(restored.critic.parameters(), lr=config["critic_learning_rate"],
                                                  momentum=config["critic_momentum"])
        restored_actor_optimizer.load_state_dict(restored_optimizers["actor"])
        restored_critic_optimizer.load_state_dict(restored_optimizers["critic"])
        require(_state_hash(actor_optimizer.state_dict()) == _state_hash(restored_actor_optimizer.state_dict())
                and _state_hash(critic_optimizer.state_dict()) == _state_hash(restored_critic_optimizer.state_dict()),
                "Optimizer state contents were not restored exactly")
        evaluation = evaluate(policy, config, budget, output / "evaluation")
        loaded_evaluation = evaluate(restored, config, budget, output / "evaluation_loaded", center_only=True)
        require(evaluation["episodes"][0] == loaded_evaluation["episodes"][0],
                "Loaded checkpoint produced a different original-MuJoCo episode")
        report.update(status="COMPLETE", passed=True,
                      actor_total_delta_norm=float((flat(actor_parameters) - initial_actor).norm()),
                      critic_total_delta_norm=float((flat(critic_parameters) - initial_critic).norm()),
                      checkpoint_roundtrip="PASS", optimizer_state_roundtrip="PASS",
                      evaluation=dict(interface_status="PASS", success_count=evaluation["success_count"],
                                      episode_count=evaluation["episode_count"],
                                      failure_counts=evaluation["failure_counts"],
                                      terminal_arrival=evaluation["terminal_arrival"],
                                      loaded_center_episode_equal=True),
                      e1b_readiness="ENGINEERING_READY_RESEARCH_TRAINING_NOT_RUN")
    except Exception as error:
        report.update(status="PARTIAL_BUDGET" if "budget exhausted" in str(error) else "FAIL",
                      error=f"{type(error).__name__}: {error}")
        raise
    finally:
        report["cost"] = budget.summary()
        report["protected_inputs_unchanged"] = all(Path(p).is_file() and sha(p) == h for p, h in protected.items())
        sources = json.loads((output / "sources.json").read_text())["content"]
        report["run_sources_unchanged"] = all(Path(p).is_file() and sha(p) == v["sha256"] for p, v in sources.items())
        report["source_mutation"] = not (report["protected_inputs_unchanged"] and report["run_sources_unchanged"])
        if report["source_mutation"]:
            report.update(status="FAIL", passed=False)
        write_json(output / "summary.json", report)
        env.close()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = run(args.output)
    print(json.dumps({k: report[k] for k in ("status", "passed", "cost", "evaluation")}))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
