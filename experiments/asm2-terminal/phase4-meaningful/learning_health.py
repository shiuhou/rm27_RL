"""Read-only PPO health measurements on archived development data."""

import json
from pathlib import Path

import numpy as np

from rm27_asm2.asset import ROOT, digest, obj_mesh, write_result
from rm27_asm2.contact_task import PATCH_GROUPS
from rm27_asm2.flight_suite import rotation_matrix
from rm27_asm2.terminal_native_eval import NativeActor, TRACK_WEIGHTS, load_evaluation_inputs

OUTPUT = Path(__file__).resolve().parent / "learning-health.json"
SOURCE = OUTPUT.parent.parent / "phase4r2" / "learning"


def measure():
    cfg, profile, spec, manifest_path, manifest, goal = load_evaluation_inputs(
        ROOT / "configs/rm27_asm2_terminal_recovery.json")
    split = manifest["splits"]["development"]
    path = manifest_path.parent / split["path"]
    if digest(path) != split["sha256"]:
        raise ValueError("Development bank changed")
    with np.load(path, allow_pickle=False) as data:
        selected = np.isin(data["case_ids"], cfg["selection_cases"])
        eval_braking = selected & (data["phase"] == "braking")
        train_braking = ~selected & (data["phase"] == "braking")
        train_cases = dict((str(case), int(data["source_frame"][train_braking & (data["case_ids"] == case)].min()))
                           for case in np.unique(data["case_ids"][train_braking]))
        states = data["states"][eval_braking | train_braking]
        eval_count = int(eval_braking.sum())
        train_count = int(train_braking.sum())
        eval_mask = eval_braking[eval_braking | train_braking]

    track, learned = (np.load(p, allow_pickle=False) for p in (TRACK_WEIGHTS, SOURCE / "development.npz"))
    if digest(SOURCE / "development.pt") != learned["checkpoint_sha256"].item():
        raise ValueError("Portable actor does not match the saved PPO checkpoint")
    mean_keys = [k for k in learned.files if k.startswith("mean.")]
    before = np.concatenate([track[k].ravel() for k in mean_keys]).astype(np.float64)
    after = np.concatenate([learned[k].ravel() for k in mean_keys]).astype(np.float64)
    rotations = np.stack([rotation_matrix(state[3:7]) for state in states])
    obs = np.concatenate((np.tile(goal-states[:, :3], (1, 10)), states[:, 7:10],
                          rotations.transpose(0, 2, 1).reshape(len(states), 9), states[:, 10:13]), axis=1)
    raw_track = NativeActor.mean(obs.astype(np.float32), NativeActor.layers(track, 45))
    raw_learned = NativeActor.mean(obs.astype(np.float32), NativeActor.layers(learned, 45))
    normalized = np.tanh(raw_learned)
    scale = np.r_[2., 2., 2., profile.hover_thrust_N*.5]
    change = (normalized-np.tanh(raw_track))*scale
    sample = np.random.default_rng(42704).standard_normal((256, len(states), 4))
    std = np.exp(learned["log_std"])
    executed = np.tanh(raw_learned + std*sample)
    stats = dict(
        mean_parameters=int(len(before)),
        changed_mean_parameters=int(np.count_nonzero(after-before)),
        mean_parameter_delta_relative_l2=float(np.linalg.norm(after-before)/np.linalg.norm(before)),
        mean_parameter_delta_rms=float(np.sqrt(np.mean((after-before)**2))),
        braking_selection_count=eval_count,
        braking_training_count=train_count,
        deterministic_action_change_rms_braking_selection=np.sqrt(np.mean(change[eval_mask]**2, axis=0)).tolist(),
        deterministic_action_change_rms_braking_training=np.sqrt(np.mean(change[~eval_mask]**2, axis=0)).tolist(),
        action_order=["p_rad_s", "q_rad_s", "r_rad_s", "thrust_N"],
        normalized_mean_action_near_bound_fraction_per_axis=(np.abs(normalized) >= .98).mean(axis=0).tolist(),
        configured_warm_start_gaussian_std=cfg["learning"]["initial_std"],
        saved_gaussian_std=std.tolist(),
        executed_noise_std_median_over_braking_resets=np.median(executed.std(axis=0)*scale, axis=0).tolist(),
    )

    patch = np.unique(np.concatenate([obj_mesh(spec.source / "mujoco" / entry["mesh"])[0]
                                     for entry in spec.entries.values() if entry["group"] in PATCH_GROUPS]), axis=0)
    terms = []
    for case, first_frame in train_cases.items():
        pin = manifest["cases"][case]["trace"]
        trace = manifest_path.parent / pin["path"]
        if digest(trace) != pin["sha256"]:
            raise ValueError(f"Archived development trace changed: {case}")
        with np.load(trace, allow_pickle=False) as data:
            state = data["states"][first_frame:, 0, :]
        rotation = np.stack([rotation_matrix(row[3:7]) for row in state])
        distance = np.linalg.norm(state[:, :3]-goal, axis=-1)
        alignment = rotation[:, :, 2] @ spec.target_normal
        local_normal = rotation.transpose(0, 2, 1) @ spec.target_normal
        projections = local_normal @ patch.T
        index = projections.argmin(axis=-1)
        gap = (state[:, :3]-spec.target_center) @ spec.target_normal + projections[np.arange(len(state)), index]
        point_rate = np.cross(state[:, 10:13], patch[index])
        inward = -(state[:, 7:10] @ spec.target_normal + (local_normal*point_rate).sum(axis=-1))
        limit = np.sqrt(.060**2 + 2.*cfg["learning"]["braking_decel_m_s2"]*np.maximum(gap, 0.))
        overspeed = .02*np.minimum((np.maximum(inward-limit, 0.)/.25)**2, 1.)
        overspeed *= (gap > 0.) & (gap <= cfg["learning"]["braking_gap_m"])
        terms.append(np.column_stack((-8.*np.diff(distance), .15*np.diff(alignment), -overspeed[:-1])))
    shaping = np.concatenate(terms)
    stats["reward_component_probe"] = dict(
        scope="Archived deterministic development witness transitions",
        components=["distance_progress", "alignment_progress", "normal_overspeed"],
        steps=int(len(shaping)),
        absolute_p50=np.median(np.abs(shaping), axis=0).tolist(),
        absolute_p95=np.quantile(np.abs(shaping), .95, axis=0).tolist(),
        absolute_max=np.abs(shaping).max(axis=0).tolist(),
        nonzero_steps=np.count_nonzero(shaping, axis=0).tolist(),
        terminal_reward=dict(qualified_range=[1., 1.2], forbidden_or_envelope=-1.25, other=-.8, timeout=-.6),
    )

    logs = [json.loads(row) for row in (SOURCE / "learning.jsonl").read_text().splitlines()]
    loss = np.asarray([row["loss"] for row in logs])
    stats["ppo_update_log"] = dict(
        updates=int(logs[-1]["updates"]), sampled_updates=[r["updates"] for r in logs],
        columns=["actor_loss", "critic_loss", "actor_grad_before_clip", "critic_grad_before_clip", "approx_kl"],
        finite=bool(np.isfinite(loss).all()), minima=loss.min(axis=0).tolist(), maxima=loss.max(axis=0).tolist(),
        final=loss[-1].tolist(), timeouts=logs[-1]["outcomes"]["TIMEOUT"],
        outcomes=logs[-1]["outcomes"])
    stats["source_hashes"] = {str(p.relative_to(ROOT)): digest(p) for p in
                             (TRACK_WEIGHTS, SOURCE/"development.npz", SOURCE/"development.pt",
                              SOURCE/"learning.jsonl", path)}
    write_result(OUTPUT, stats)
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    measure()
