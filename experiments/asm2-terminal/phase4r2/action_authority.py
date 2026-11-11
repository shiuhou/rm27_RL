"""Matched native deterministic replays with current and pitch-expanded PPO action maps."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from itertools import count
import multiprocessing
from pathlib import Path
import time

import mujoco
import numpy as np

from rm27_asm2.flight import allocate_ctbr
from rm27_asm2.terminal_native_eval import (
    HANDOVER_CONFIG, NativeContact, digest, load_deterministic,
    load_evaluation_inputs, load_starts, read_json, recovery_indices,
    rollout, summary, write_result,
)


CONFIG = Path("configs/rm27_asm2_terminal_recovery.json")
MAPS = {
    "deterministic": None,
    "current": np.array([2.0, 2.0, 2.0]),
    "pitch_expanded": np.array([2.0, 2.75, 2.0]),
}


def initialize(plant, data, references, horizon, limits):
    global PLANT, DATA, REFERENCES, HORIZON, LIMITS
    PLANT, DATA, REFERENCES, HORIZON, LIMITS = plant, data, references, horizon, limits
    PLANT.data = mujoco.MjData(PLANT.model)


def evaluate(task):
    mode, idx = task
    case, frame = str(DATA["case_ids"][idx]), int(DATA["source_frame"][idx])
    ref, gate = REFERENCES[case]
    ticks = count(frame-gate)
    hover = PLANT.profile.hover_thrust_N
    rate_cap = MAPS[mode]
    raw = []

    def command(state):
        request = ref.command(state, next(ticks))
        raw.append(request.copy())
        if rate_cap is None:
            return request
        return np.clip(request, np.r_[.5*hover, -rate_cap], np.r_[1.5*hover, rate_cap])

    result, (states, _, applied) = rollout(
        PLANT, DATA["states"][idx], DATA["motor_thrust_N"][idx], DATA["motor_target_N"][idx],
        command, HORIZON, LIMITS,
    )
    raw = np.asarray(raw)
    targets, saturated = allocate_ctbr(PLANT.profile, applied, states[:-1, 10:13])
    assert result["saturation_controls"] == np.count_nonzero(saturated)
    result.update(case=case, source_frame=frame, requested_abs_rate_max=np.max(np.abs(raw[:, 1:]), axis=0).tolist(),
                  requested_collective_hover_range=[float(raw[:, 0].min()/hover), float(raw[:, 0].max()/hover)],
                  clipped_by_map_controls=int(np.count_nonzero(np.any(raw != applied, axis=1))),
                  clipped_rate_axis_controls=np.count_nonzero(raw[:, 1:] != applied[:, 1:], axis=0).tolist(),
                  rotor_target_N_range=[float(targets.min()), float(targets.max())],
                  peak_measured_body_rate_rad_s=float(np.linalg.norm(states[:, 10:13], axis=1).max()))
    return result, raw, applied


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=10)
    args = parser.parse_args()
    started = time.monotonic()
    cfg, profile, spec, manifest_path, manifest, _ = load_evaluation_inputs(CONFIG)
    directory = manifest_path.parent
    data = load_starts(directory, manifest["splits"]["development"], expected_count=280, profile=profile)
    indices = recovery_indices(data, cfg["selection_cases"])
    references, source = load_deterministic(directory, manifest, [("development", data, indices)], profile, spec)
    limits = read_json(HANDOVER_CONFIG)["flight_envelope"]
    plant = NativeContact(profile, spec)
    results = {}
    with ProcessPoolExecutor(
        max_workers=args.workers, mp_context=multiprocessing.get_context("fork"),
        initializer=initialize, initargs=(plant, data, references, cfg["braking_horizon_s"], limits),
    ) as pool:
        for mode in MAPS:
            episodes, requests, commands = zip(*pool.map(evaluate, ((mode, idx) for idx in indices)))
            stats = summary(episodes)
            all_requests = np.concatenate(requests)
            all_commands = np.concatenate(commands)
            stats.update(
                rate_request_abs_quantiles_rad_s=np.quantile(np.abs(all_requests[:, 1:]), [.5, .95, .99, 1.], axis=0).tolist(),
                applied_rate_abs_max_rad_s=np.max(np.abs(all_commands[:, 1:]), axis=0).tolist(),
                requested_collective_hover_range=[float(all_requests[:, 0].min()/profile.hover_thrust_N),
                                                  float(all_requests[:, 0].max()/profile.hover_thrust_N)],
                clipped_by_map_controls=sum(e["clipped_by_map_controls"] for e in episodes),
                clipped_rate_axis_controls=np.sum([e["clipped_rate_axis_controls"] for e in episodes], axis=0).tolist(),
                rotor_target_N_range=[min(e["rotor_target_N_range"][0] for e in episodes),
                                      max(e["rotor_target_N_range"][1] for e in episodes)],
                peak_measured_body_rate_rad_s=max(e["peak_measured_body_rate_rad_s"] for e in episodes),
            )
            print(mode, stats, flush=True)
            for episode in episodes:
                event = episode.pop("contact")["first_touch"]
                episode["first_touch"] = ({key: event[key] for key in (
                    "groups", "sample_time_s", "aircraft_colliders", "target_colliders",
                )} if event else None)
            results[mode] = dict(summary=stats, episodes=episodes)
    write_result(args.output, dict(
        schema="asm2_phase4r2_action_authority_diagnostic_v1", config_sha256=digest(CONFIG),
        reset_manifest_sha256=digest(manifest_path), evaluator_sha256=digest(Path(__file__)),
        scorer_sha256=spec.contract_sha256, profile_sha256=profile.profile_sha256, physics_dt_s=plant.dt,
        control_dt_s=profile.policy_dt_s, development_cases=cfg["selection_cases"], source=source,
        native_rate_cap_rad_s=profile.native_rate_limit_rad_s, rotor_cap_N=profile.max_rotor_thrust_N,
        bounded_collective_hover_range=[.5, 1.5],
        rate_caps_rad_s={name: value.tolist() if value is not None else None for name, value in MAPS.items()},
        quantiles=[.5, .95, .99, 1.], wall_s=time.monotonic()-started, training_transitions=0,
        results=results,
    ))


if __name__ == "__main__":
    main()
