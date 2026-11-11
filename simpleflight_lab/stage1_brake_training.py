"""One bounded PGS braking bridge candidate after frozen backend confirmation."""

import argparse
from collections import Counter
import csv
import fcntl
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback

from .stage1_experiment import ROOT, STAGE1C_CONFIG, STAGE1C_EXPERIMENT, digest, load_stage1c_experiment, read_json
from .stage1_training import LearningBudget, LearningLimit, make_environment, write_json


CONFIG = ROOT / "configs/rm27_stage1_pgs_braking_pilot.json"
RUN = ROOT / ".agents/runs/rm27-stage1/pgs-braking-pilot"
BUDGET = {"candidates": 1, "transitions": 3_000_000, "learning_s": 900, "evaluation_s": 600}
EXPERIMENT = "sf_rm27_stage1_pgs_braking_pilot_01"


def load_config(path):
    config = read_json(path)
    if (config["schema_version"] != 1 or config["experiment"] != EXPERIMENT
            or config["scope"] != "simulation_only_state_based_direct_ctbr_braking_bridge"
            or config["budget"] != BUDGET
            or config["sim"] != {"worlds": 128, "solver": "PGS", "contact_offset_m": 1e-6, "rest_offset_m": 0.0}):
        raise ValueError("Braking pilot configuration or authorization differs")
    pins = {name: ROOT / item["path"] for name, item in config["pins"].items()}
    for name, pin in pins.items():
        if digest(pin) != config["pins"][name]["sha256"]:
            raise ValueError(f"Braking pilot input changed: {name}")
    receipt = read_json(pins["backend"])
    contract = read_json(pins["contract"])
    if (pins["parent"] != STAGE1C_CONFIG or receipt["status"] != "PASS"
            or receipt["contract_sha256"] != digest(pins["contract"])
            or contract["hashes"]["offset_layer"] != digest(pins["scene"])
            or contract["worlds"] != 128 or not receipt["repeat_labels_identical"]
            or set(receipt["runs"]) != {"D", "E", "D-repeat"}
            or any(not all(run["checks"].values()) for run in receipt["runs"].values())):
        raise ValueError("Fresh 128-world backend confirmation is required")
    return config, pins


def assert_live_scene(expected):
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    scene = next(prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene))
    if (scene.GetAttribute("physxScene:solverType").Get() != expected["solver"]
            or scene.GetAttribute("physxScene:enableCCD").Get()
            or not scene.GetAttribute("physxScene:enableGPUDynamics").Get()):
        raise ValueError("Live PhysX scene differs from the verified GPU PGS configuration")
    root = stage.GetPrimAtPath("/World/Stage1Envs/env_0")
    shapes = [prim for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.CollisionAPI)]
    if len(shapes) != 2069 or any(
        abs(p.GetAttribute("physxCollision:contactOffset").Get() - expected["contact_offset_m"]) > 1e-11
        or p.GetAttribute("physxCollision:restOffset").Get() != expected["rest_offset_m"] for p in shapes
    ):
        raise ValueError("Live PhysX collider offsets differ from the verified overlay")


def development_gate(checkpoint, cfg_path, config, track_sha, frames):
    files = {mode: RUN / f"dev-{suffix}.json" for mode, suffix in
             (("track", "track"), ("trim", "trim"), ("learned", "1m"))}
    deadline = time.monotonic() + 180
    while not all(path.exists() for path in files.values()):
        if time.monotonic() >= deadline:
            raise LearningLimit("DEVELOPMENT_RESULT_UNAVAILABLE")
        time.sleep(.25)
    results = {mode: read_json(path) for mode, path in files.items()}
    for mode, row in results.items():
        if (row["mode"] != mode or row["suite"] != "development" or row["episodes"] != 8
                or row["manifest_sha256"] != config["pins"]["suites"]["sha256"]
                or row["qualified"] != sum(case["passed"] for case in row["cases"].values())
                or row["trace_sha256"] != digest(files[mode].with_suffix(".npz"))):
            raise ValueError("Development result does not match the prospective native suite")
    learned = results["learned"]
    if (learned["actor"]["checkpoint_sha256"] != digest(checkpoint)
            or learned["actor"]["bridge_config_sha256"] != digest(cfg_path)
            or results["track"]["actor"]["checkpoint_sha256"] != track_sha
            or any(list(r["cases"]) != list(learned["cases"]) for r in results.values())):
        raise ValueError("Development actor or matched control provenance differs")
    counts = {mode: row["qualified"] for mode, row in results.items()}
    rule = config["development_gate"]
    gain = counts["learned"] - max(counts["track"], counts["trim"])
    proceed = gain >= rule["min_net_improvement"] and learned["safety_exits"] <= rule["max_safety_exits"]
    decision = dict(continue_training=proceed, counts=counts, net_improvement=gain,
                    safety_exits=learned["safety_exits"], frames=frames, checkpoint_sha256=digest(checkpoint),
                    result_sha256={mode: digest(path) for mode, path in files.items()})
    write_json(RUN / "development-decision.json", decision)
    return proceed


def train(cfg_path, config, pins, output):
    import torch

    from .ppo import GAE_LAMBDA, PPO, ROLLOUT_STEPS, UPSTREAM_REVISION
    from .rm27 import RM27_SIM_REVISION
    from .stage1_brake_task import Stage1BrakeStarts, Stage1BrakeTask, FRONTIER_BANDS_M
    from .stage1_sim import Stage1Sim
    from .stage1_task import Stage1Actor

    parent, assets, frozen, geometry = load_stage1c_experiment()
    settings = config["learning"]
    worlds = config["sim"]["worlds"]
    torch.manual_seed(settings["seed"])
    starts = Stage1BrakeStarts.from_witnesses(assets["witness_1p0"].parent, geometry)
    sim = Stage1Sim(worlds, pins["scene"], device="cuda:0", solver_type=0)
    assert_live_scene(config["sim"])
    if (sim.physics_dt_s, sim.control_dt_s) != (frozen["physics_dt_s"], frozen["policy_dt_s"]):
        raise ValueError("Braking pilot clocks differ from the frozen task")
    task = Stage1BrakeTask(worlds, sim.device, geometry, frozen, starts=starts, gamma=settings["gamma"],
                           goal_inset_m=parent["goal_inset_m"])
    task.set_frontier(settings["initial_frontier"])
    budget = LearningBudget(BUDGET["transitions"], BUDGET["learning_s"])
    env = make_environment(sim, task, frozen, geometry, budget, informed_resets=True)
    env.reset(env.ids)
    obs = env.observe()
    if obs["policy"].shape != (worlds, 21) or not all(
        value.is_cuda and bool(value.isfinite().all()) for value in obs.values()
    ):
        raise ValueError("Expected finite Stage 1 CUDA observations with the preserved actor schema")
    source = torch.load(assets["checkpoint"], map_location=sim.device, weights_only=True)
    if (source["format"] != 1 or source["task"] != "track" or source["actor_obs"] != 45
            or source["airframe"] != "rm27" or source["upstream_revision"] != UPSTREAM_REVISION
            or source["rm27_sim_revision"] != RM27_SIM_REVISION):
        raise ValueError("Braking bridge requires the verified Track initialization")
    actor = Stage1Actor(settings["initial_std"]).to(sim.device)
    actor.load_track_state_dict(source["actor"], initial_std=settings["initial_std"])
    ppo = PPO(obs, settings["initial_std"], actor=actor, learning_rate=settings["rate"],
              gamma=settings["gamma"], gae_lambda=math.sqrt(GAE_LAMBDA))
    config_sha, track_sha = digest(cfg_path), digest(assets["checkpoint"])
    rollout_size = worlds * ROLLOUT_STEPS
    first_gate = config["development_gate"]["nominal_transitions"] // rollout_size * rollout_size
    second_gate = 2_000_000 // rollout_size * rollout_size
    updates = frames = saved_update = consecutive = 0
    window, totals = Counter(), Counter()
    promotions = []
    status, partial_update = "ERROR", False
    evaluation_pause_s = 0.0

    def save(name):
        path = output / name
        temp = path.with_suffix(".tmp")
        torch.save(dict(format=1, task="rm27_stage1c", experiment=STAGE1C_EXPERIMENT,
                        config_sha256=digest(STAGE1C_CONFIG), bridge_experiment=EXPERIMENT,
                        bridge_config_sha256=config_sha, source_checkpoint_sha256=track_sha,
                        upstream_revision=UPSTREAM_REVISION, rm27_sim_revision=RM27_SIM_REVISION,
                        policy_dt=sim.control_dt_s, actor_obs=21, seed=settings["seed"], candidate=1,
                        goal_inset_m=parent["goal_inset_m"], frontier_index=task.frontier_index,
                        update=updates, frames=frames, actor=actor.state_dict(), critic=ppo.critic.state_dict(),
                        value_normalizer=ppo.normalizer.state_dict(), reward_config=task.pilot_reward,
                        actor_optimizer=ppo.actor_opt.state_dict(), critic_optimizer=ppo.critic_opt.state_dict()), temp)
        temp.replace(path)
        return path

    outcomes = ("episodes", "qualified", "rejected_contact", "unsafe_flight", "timeout")
    segments = ("stationary", "frontier", *(f"band_{i}" for i in range(len(FRONTIER_BANDS_M))))
    fields = ("update", "frames", "learning_s", "samples_per_s", "frontier", "reward", "position_rmse_m", "kl",
              "physical_contacts", "unsafe_contacts", *outcomes,
              *(f"{segment}_{kind}" for segment in segments for kind in outcomes),
              "mean_episode_min_error_m", "mean_saturation_fraction")

    def timeout(_signum, _frame):
        budget.stop_reason = "TIME_LIMIT"

    output.mkdir()
    write_json(output / "settings.json", dict(config_sha256=config_sha, track_sha256=track_sha,
                                              settings=settings, sim=config["sim"],
                                              band_intervals_m=FRONTIER_BANDS_M, dev_gate_frames=first_gate))
    save("initial.pt")
    from .monitoring import Monitor, training_metrics
    monitor = Monitor(output / "monitoring", output.name,
                      metadata={"kind": "isaac-training-proxy", "experiment": EXPERIMENT,
                                "config_sha256": config_sha, "source_checkpoint_sha256": track_sha})
    signal.signal(signal.SIGALRM, timeout)
    torch.cuda.synchronize()
    budget.started = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, budget.max_seconds - 1)
    print(f"PGS_BRAKING_START worlds={worlds} gate_frames={first_gate}", flush=True)
    try:
        with (output / "training.csv").open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            while budget.frames + rollout_size <= budget.max_frames:
                budget.check()
                start = time.perf_counter()
                obs, rollout = ppo.collect(env, obs)
                if not bool(torch.isfinite(rollout).all() & torch.isfinite(ppo.rollout["advantage"]).all()):
                    raise RuntimeError("Nonfinite braking pilot rollout")
                partial_update = True
                optimization = ppo.update(before_minibatch=budget.check)
                values = torch.cat((rollout, optimization)).tolist()
                if not all(map(math.isfinite, values)):
                    raise RuntimeError("Nonfinite braking pilot update")
                partial_update = False
                updates += 1
                frames = budget.frames
                stats = env.interval_stats()
                totals.update({key: value for key, value in stats.items() if isinstance(value, int)})
                window.update({key: stats.get(key, 0) for key in ("frontier_episodes", "frontier_qualified")})
                row = dict(update=updates, frames=frames, learning_s=budget.elapsed,
                           samples_per_s=rollout_size / (time.perf_counter() - start), frontier=task.frontier_index,
                           reward=values[0], position_rmse_m=values[1], kl=values[-1])
                row.update({key: stats.get(key, 0) for key in fields if key not in row})
                writer.writerow(row)
                stream.flush()
                if monitor.process is not None:
                    monitor.log(training_metrics(row, optimization=values[5:10],
                                                 log_std=ppo.actor.log_std.detach().cpu().tolist()), step=frames)

                promotion = config["promotion"]
                if task.frontier_index + 1 < task.num_frontiers and window["frontier_episodes"] >= promotion["window_episodes"]:
                    fraction = window["frontier_qualified"] / window["frontier_episodes"]
                    consecutive = consecutive + 1 if fraction >= promotion["min_fraction"] else 0
                    if consecutive >= promotion["consecutive_windows"]:
                        previous = task.frontier_index
                        task.set_frontier(previous + 1)
                        promotions.append(dict(frames=frames, previous=previous, frontier=task.frontier_index,
                                               window=dict(window)))
                        save(f"frontier-{task.frontier_index}.pt")
                        consecutive = 0
                    window.clear()

                if updates == 1 or updates % 20 == 0:
                    save("checkpoint.pt")
                    saved_update = updates
                    print(f"PGS_BRAKING u{updates} {frames:,} t={budget.elapsed:.0f}s "
                          f"band={task.frontier_index} {row['samples_per_s']:.0f}/s", flush=True)
                if frames == first_gate:
                    checkpoint = save("development-1m.pt")
                    budget.check()
                    signal.setitimer(signal.ITIMER_REAL, 0)
                    pause = time.perf_counter()
                    print("PGS_BRAKING_DEVELOPMENT_WAIT", flush=True)
                    try:
                        proceed = development_gate(checkpoint, cfg_path, config, track_sha, frames)
                    finally:
                        evaluation_pause_s = time.perf_counter() - pause
                        budget.started += evaluation_pause_s
                    if not proceed:
                        raise LearningLimit("NO_VALIDATED_FRONTIER_IMPROVEMENT")
                    budget.check()
                    signal.setitimer(signal.ITIMER_REAL, max(.001, budget.max_seconds - budget.elapsed - 1))
                if frames == second_gate:
                    save("development-2m.pt")
            status = "TRANSITION_LIMIT"
    except LearningLimit as stop:
        status = str(stop)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        learning_s = budget.elapsed
        if status != "ERROR" and updates and not partial_update and updates != saved_update:
            save("checkpoint.pt")
            saved_update = updates
        write_json(output / "learning.json", dict(status=status, frames=budget.frames, completed_frames=frames,
                   learning_s=learning_s, evaluation_pause_s=evaluation_pause_s, updates=updates,
                   checkpoint_update=saved_update, frontier_index=task.frontier_index, promotions=promotions,
                   config_sha256=config_sha, training_counts=dict(totals), failure_modes=dict(env.failure_modes),
                   reset_overlap_rejections=env.reset_rejections, representative_contacts=env.examples))
        monitor.finish()
    print(f"PGS_BRAKING_COMPLETE {status} frames={budget.frames:,} learning_s={learning_s:.2f}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    args = parser.parse_args()
    cfg_path = args.config.resolve()
    config, pins = load_config(cfg_path)
    output = RUN / "candidate"
    with (RUN / "learning.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if output.exists():
            raise FileExistsError("The single PGS braking pilot candidate slot is already occupied")
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(headless=True, device="cuda:0")
        exit_code = 0
        try:
            train(cfg_path, config, pins, output)
        except Exception:
            traceback.print_exc()
            exit_code = 2
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
