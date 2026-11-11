"""Plot training metrics and replay saved evaluation trajectories on the CPU."""

import argparse
import json
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
from textwrap import fill

import matplotlib
import numpy as np

matplotlib.use("Agg")
from matplotlib import animation, pyplot as plt
from matplotlib.patches import Circle, Rectangle


def plot_curves(run_dir: Path) -> None:
    metrics = np.genfromtxt(run_dir / "training.csv", delimiter=",", names=True, ndmin=1)
    if not metrics.size:
        raise ValueError(f"No PPO updates recorded in {run_dir / 'training.csv'}")

    frames = metrics["frames"] / 1e6
    fig, axes = plt.subplots(2, 2, figsize=(9.6, 5.4), layout="constrained", sharex=True)
    fig.suptitle(f"SimpleFlight Direct-CTBR PPO | {run_dir.name}", weight="bold")
    for ax, field, title in zip(
        axes.flat,
        ("step_reward", "position_rmse_m", "samples_per_s", "gpu_used_mib"),
        ("Mean step reward", "Rollout position RMSE (m)", "Samples per wall second", "Total GPU memory (MiB)"),
    ):
        ax.plot(frames, metrics[field], ".-", lw=1.4, ms=3)
        ax.set_title(title, loc="left", fontsize=11)
        ax.grid(alpha=0.25)
    for ax in axes[-1]:
        ax.set_xlabel("Policy samples (millions)")
    output = run_dir / "curves.png"
    fig.savefig(output, dpi=140)
    plt.close(fig)
    print(output)


def representative(candidates: np.ndarray, rmse: np.ndarray, done_tick: np.ndarray) -> int:
    scored = candidates[np.isfinite(rmse[candidates])]
    if scored.size:
        median = np.median(rmse[scored])
        return int(scored[np.abs(rmse[scored] - median).argmin()])
    median = np.median(done_tick[candidates])
    return int(candidates[np.abs(done_tick[candidates] - median).argmin()])


def render_episode(
    output: Path,
    label: str,
    time_s: np.ndarray,
    position: np.ndarray,
    reference: np.ndarray,
    quat: np.ndarray,
) -> None:
    error = np.linalg.norm(position - reference, axis=1)
    w, x, y, z = quat.T  # WXYZ, body to world.
    heading_x = 1 - 2 * (y * y + z * z)
    heading_y = 2 * (x * y + w * z)
    color = "#0072b2"
    ref_color = "#b64c00"

    fig = plt.figure(figsize=(9.6, 5.4), layout="constrained")
    grid = fig.add_gridspec(2, 2, width_ratios=(1.13, 1))
    xy = fig.add_subplot(grid[:, 0], aspect="equal")
    altitude = fig.add_subplot(grid[0, 1])
    distance = fig.add_subplot(grid[1, 1], sharex=altitude)
    fig.suptitle(label, weight="bold", fontsize=13)

    xy.plot(*reference[:, :2].T, "--", color=ref_color, lw=1.5, label="Reference")
    path, = xy.plot([], [], color=color, lw=1.8, label="Flight path")
    target, = xy.plot([], [], marker="x", color=ref_color, ms=8, ls="none")
    drone, = xy.plot([], [], marker="o", color=color, ms=6, ls="none")
    heading, = xy.plot([], [], color=color, lw=2)
    xy.legend(loc="upper right", fontsize=9)
    xy.set(title="Top view", xlabel="x (m)", ylabel="y (m)")
    all_xy = np.concatenate((position[:, :2], reference[:, :2]))
    center = (all_xy.max(axis=0) + all_xy.min(axis=0)) / 2
    half_span = max(0.25, 0.6 * float(np.ptp(all_xy, axis=0).max()))
    xy.set(xlim=center[0] + half_span * np.array([-1, 1]), ylim=center[1] + half_span * np.array([-1, 1]))
    heading_len = min(0.13, half_span * 0.28)

    altitude.plot(time_s, reference[:, 2], "--", color=ref_color, label="Reference")
    altitude.plot(time_s, position[:, 2], color=color, label="Actual")
    altitude_cursor = altitude.axvline(time_s[0], color="0.4", alpha=0.6)
    altitude_now, = altitude.plot([], [], "o", color=color, ms=4)
    altitude.legend(loc="best", fontsize=8)
    altitude.set(title="Altitude", ylabel="z (m)")
    altitude.tick_params(labelbottom=False)

    distance.plot(time_s, error, color=color)
    error_cursor = distance.axvline(time_s[0], color="0.4", alpha=0.6)
    error_now, = distance.plot([], [], "o", color=color, ms=4)
    distance.set(
        title="Position error", ylabel="3D error (m)", xlabel="Time (s)",
        xlim=(0, max(0.1, time_s[-1])), ylim=(0, max(0.08, error.max() * 1.10)),
    )
    for ax in (xy, altitude, distance):
        ax.grid(alpha=0.25)

    def draw(tick: int) -> None:
        px, py, pz = position[tick]
        tx, ty, _ = reference[tick]
        stamp = time_s[tick]
        path.set_data(*position[: tick + 1, :2].T)
        target.set_data([tx], [ty])
        drone.set_data([px], [py])
        heading.set_data([px, px + heading_len * heading_x[tick]], [py, py + heading_len * heading_y[tick]])
        altitude_cursor.set_xdata([stamp, stamp])
        error_cursor.set_xdata([stamp, stamp])
        altitude_now.set_data([stamp], [pz])
        error_now.set_data([stamp], [error[tick]])
        distance.set_title(f"Position error   |   {stamp:.2f} s   ·   {error[tick]:.3f} m")

    fps = 25
    frame_times = np.arange(0.0, time_s[-1], 1 / fps)
    ticks = np.searchsorted(time_s, frame_times)
    ticks = np.concatenate((ticks, np.full(fps // 2, len(time_s) - 1)))
    write_replay(output, fig, draw, ticks, fps)


def write_replay(output: Path, fig, draw, ticks: np.ndarray, fps: int) -> None:
    encoder = shutil.which("ffmpeg")
    if encoder is None:
        import imageio_ffmpeg

        encoder = imageio_ffmpeg.get_ffmpeg_exe()
    matplotlib.rcParams["animation.ffmpeg_path"] = encoder
    # Resolve the static layout once before streaming frames to the encoder.
    fig.canvas.draw()
    fig.set_layout_engine("none")
    writer = animation.FFMpegWriter(
        fps=fps, codec="libx264", bitrate=1500,
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart", "-threads", "2"],
    )
    with TemporaryDirectory(dir=output.parent) as scratch:
        temporary_output = Path(scratch) / output.name
        with writer.saving(fig, temporary_output, dpi=100):
            for tick in ticks:
                draw(tick)
                writer.grab_frame()
        temporary_output.replace(output)
    plt.close(fig)


def replay(bundle: Path, output_dir: Path, selection: str) -> None:
    with np.load(bundle, allow_pickle=False) as data:
        dt = float(data["dt"].item())
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError(f"Invalid simulator tick duration in {bundle}")
        success = data["success"].reshape(-1).astype(bool)
        first_done = data["first_done_tick"].reshape(-1)
        rmse = data["rmse_m"].reshape(-1)
        position = data["position"]
        reference = data["reference"]
        quat = data["quat"]
        task = data["task"].item()
        profile = data["profile"].item()
        seed = data["seed"].item()

    output_dir.mkdir(parents=True, exist_ok=True)
    for outcome, mask in (("success", success), ("failure", ~success)):
        if selection not in ("both", outcome):
            continue
        # A numerical failure can make even the first tick unrenderable.
        candidates = np.flatnonzero(mask & np.isfinite(position[0]).all(axis=-1) & np.isfinite(reference[0]).all(axis=-1))
        if not candidates.size:
            print(f"{bundle.name}: no renderable {outcome} episodes")
            continue
        env = representative(candidates, rmse, first_done)
        end = int(first_done[env])
        ticks = min(end, position.shape[0]) if end > 0 else position.shape[0]
        p, r, q = position[:ticks, env], reference[:ticks, env], quat[:ticks, env]
        invalid = np.flatnonzero(~(np.isfinite(p).all(axis=1) & np.isfinite(r).all(axis=1)))
        if invalid.size:
            ticks = int(invalid[0])
        output = output_dir / f"{bundle.stem}-{outcome}.mp4"
        label = f"{task} / {profile} | {outcome} | seed {seed}, environment {env}"
        render_episode(output, label, dt * np.arange(1, ticks + 1), p[:ticks], r[:ticks], q[:ticks])
        print(output)


def stage1_replay(bundle: Path, geometry_file: Path, output: Path, case_name: str) -> None:
    result = json.loads(bundle.with_suffix(".json").read_text())
    geometry = json.loads(geometry_file.read_text())
    if result["frozen_private_scene_sha256"] != geometry["model_sha256"]:
        raise ValueError("Replay and geometry were recorded from different MuJoCo models")
    case = result["cases"][case_name]
    target = geometry["target"]
    center, rotation, half = (np.asarray(target[key]) for key in ("center_m", "target_rotation", "halfsize_m"))

    def target_cm(points):
        return 100 * (np.asarray(points) - center) @ rotation

    with np.load(bundle, allow_pickle=False) as data:
        env = data["names"].tolist().index(case_name)
        end = int(data["first_done_tick"][env])
        dt = float(data["dt"].item())
        takeover = case.get("reset", {}).get("takeover_tick", 0)
        positions = data["position"]
        if not 0 < end <= len(positions) or not np.isfinite(dt) or dt <= 0:
            raise ValueError(f"Invalid recorded episode length or timestep for {case_name}")
        if not (np.isclose((takeover + end) * dt, case["episode_elapsed_s"])
                and data["success"][env] == case["passed"]):
            raise ValueError(f"Telemetry and summary disagree for {case_name}")
        time_s = np.arange(end + 1) * dt
        position = target_cm(np.vstack((data["initial_state"][env, :3], positions[:end, env])))
        bumper = target_cm(data["bumper"][:end + 1, env])
    if not (np.isfinite(position).all() and np.isfinite(bumper).all()):
        raise ValueError(f"Incomplete telemetry for {case_name} in {bundle}")
    radius = 100 * next(item["size"][0] for item in geometry["aircraft"] if item["name"] == "drone_1_stage1_contact")
    first = case["first_contact"]
    contacts = target_cm([record["position_world_m"] for record in first["contacts"]]) if first else np.empty((0, 3))
    half *= 100

    body_color, sphere_color, contact_color = "#496a83", "#0072b2", "#b64c00"
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 6.4), layout="constrained")
    status = "Qualified contact" if case["passed"] else ("Rejected contact" if first else "No contact")
    profile = result["profile"].replace("_", " ")
    stage = {"rm27_stage1b": "Stage 1-B", "rm27_stage1c": "Stage 1-C",
             "rm27_stage1_braking_bridge": "Braking bridge"}.get(result.get("task"), "Stage 1")
    if takeover:
        profile += f" · policy takeover at {takeover * dt:.3f} s"
    if stage == "Stage 1-C" and "suite" in result:
        profile = result["suite"].replace("_", " ")
        if "nominal_standoff_m" in case:
            profile += f" · {case['nominal_standoff_m']:.3f} m nominal COM standoff"
    if "standoff_com_normal_m" in result:
        profile += f" · {result['standoff_com_normal_m']:.2f} m COM standoff"
    if "near_reset" in result:
        speed = result["near_reset"]["nominal_inward_speed_m_s"]
        profile += f" · start speed {speed:.2f} m/s inward"
    elif "frontier_index" in result and result["nominal_bumper_clearance_m"] is not None:
        clearance_mm = 1000 * result["nominal_bumper_clearance_m"]
        profile = f"Stationary frontier {result['frontier_index']} · {clearance_mm:g} mm nominal clearance"
    elif result["profile"] == "full_1m" and stage == "Stage 1-B":
        profile = "Archived 1 m start"
    prefix, separator, _ = bundle.stem.partition("-native")
    snapshot = f" ({prefix} checkpoint)" if separator else ""
    fig.suptitle(f"RM27 {stage}  |  {bundle.parent.name}{snapshot}\n{profile} / {case_name}",
                 weight="bold", fontsize=14)
    labels = [
        (0, 2, "Side view in the target frame", "Along slope (cm)", "Outward normal (cm)"),
        (0, 1, "View onto the target footprint", "Along slope (cm)", "Across target (cm)"),
    ]
    artists = []
    for ax, (i, j, title, xlabel, ylabel) in zip(axes, labels):
        ax.set(title=title, xlabel=xlabel, ylabel=ylabel, aspect="equal")
        if j == 2:
            ax.plot([-half[0], half[0]], [0, 0], color="#2b7858", lw=4, label="Target footprint")
            footprint = np.array([[-half[0], 0], [half[0], 0]])
        else:
            ax.add_patch(Rectangle(-half, *(2 * half), edgecolor="#2b7858", facecolor="#dceee4", lw=1.5,
                                   label="Target footprint"))
            footprint = np.array([-half, half])
        points = np.vstack((position[:, [i, j]], bumper[:, [i, j]], contacts[:, [i, j]], footprint))
        margin = np.maximum(2 * radius, 0.08 * np.ptp(points, axis=0))
        lo, hi = points.min(axis=0) - margin, points.max(axis=0) + margin
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]))
        ax.grid(alpha=.25)
        trail, = ax.plot([], [], color=sphere_color, lw=1.7, label="Bumper center path")
        arm, = ax.plot([], [], ".-", color=body_color, lw=1, ms=5, label="Aircraft origin to bumper")
        ball = Circle((0, 0), radius, color=sphere_color, alpha=.40)
        ax.add_patch(ball)
        hit, = ax.plot([], [], "x", color=contact_color, mew=2, ms=8, label="Recorded first contact")
        artists.append((trail, arm, ball, hit))
    axes[1].legend(loc="best", fontsize=8)
    caption = axes[0].text(.0, -.19, "", transform=axes[0].transAxes, fontsize=9, va="top")
    if first:
        pairs = sorted({name.removeprefix("rmuc2027_surface_") for record in first["contacts"]
                        for name in record["geoms"] if name.startswith("rmuc2027_surface_")})
        metrics = f"First contact at {first['time_s']:.3f} s"
        if pairs:
            metrics += f"  |  field: {', '.join(pairs)}"
        if stage in ("Stage 1-B", "Stage 1-C"):
            parts = sorted({name.removeprefix("drone_1_collision_").replace("_", " ")
                            for record in first["contacts"] for name in record["geoms"]
                            if name.startswith("drone_1_collision_")})
            if parts:
                status += f" · aircraft {', '.join(parts)}"
        if first["gates"]:
            gate = first["gates"][0]
            metrics += (f"\nNormal / tangent speed {gate['normal_speed_m_s']:.2f} / "
                        f"{gate['tangent_speed_m_s']:.2f} m/s  |  alignment {gate['bottom_normal_error_deg']:.1f}°")
    else:
        metrics = f"Recorded end at {case['episode_elapsed_s']:.3f} s"
        closest = case.get("closest_surface_target")
        if closest:
            metrics += (f"\nClosest bumper target error {100 * closest['distance_m']:.1f} cm "
                        f"at {closest['time_s']:.3f} s")
    failures = fill(", ".join(mode.lower().replace("contact_", "").replace("_", " ")
                              for mode in case["failure_modes"]), width=58)
    outcome = axes[1].text(.0, -.19, "", transform=axes[1].transAxes, fontsize=9, va="top", wrap=True)
    playback = min(1., time_s[-1] / 2.5)

    def draw(tick):
        done = tick == end
        for (i, j, *_), (trail, arm, ball, hit) in zip(labels, artists):
            trail.set_data(*bumper[:tick + 1, [i, j]].T)
            arm.set_data(*np.stack((position[tick, [i, j]], bumper[tick, [i, j]])).T)
            ball.center = bumper[tick, [i, j]]
            hit.set_data(*(contacts[:, [i, j]].T if done else ([], [])))
        caption.set_text(f"Simulation {time_s[tick] + takeover * dt:.3f} s  |  playback {playback:.2g}×\n"
                         + (metrics if done else " "))
        outcome.set_text(status + (f"\n{failures}" if failures else "") if done else "")

    fps = 25
    ticks = np.searchsorted(time_s, np.arange(0., time_s[-1], playback / fps))
    # Size the layout using the terminal text before streaming the first frame.
    draw(end)
    output.parent.mkdir(parents=True, exist_ok=True)
    write_replay(output, fig, draw, np.r_[ticks, np.full(fps, end)], fps)
    print(output)


def stage1_curves(run_dir: Path) -> None:
    candidates = sorted(run_dir.glob("candidate-*/training.csv"))
    if (run_dir / "candidate/training.csv").is_file():
        candidates.append(run_dir / "candidate/training.csv")
    if not candidates:
        raise ValueError(f"No Stage 1 training.csv files in {run_dir}")
    runs = [(candidate.parent.name, np.genfromtxt(candidate, delimiter=",", names=True, ndmin=1))
            for candidate in candidates]
    runs = [(name, rows) for name, rows in runs if rows.size]
    if not runs:
        raise ValueError(f"No Stage 1 PPO updates recorded in {run_dir}")
    handover = ["stationary_episodes" in rows.dtype.names for _, rows in runs]
    if all(handover):
        stage1c_curves(run_dir, runs)
        return
    if any(handover):
        raise ValueError(f"Stage 1-C and earlier training logs are mixed in {run_dir}")
    schemas = {"frontier_episodes" in rows.dtype.names for _, rows in runs}
    if len(schemas) != 1:
        raise ValueError(f"Stage 1-A and Stage 1-B training logs are mixed in {run_dir}")
    stage1b, = schemas
    stage = "Stage 1-B" if stage1b else "Stage 1"
    near_cohort = "frontier" if stage1b else "near"
    near_label = "stationary frontier starts" if stage1b else "near training starts"
    full_label = "archived 1 m starts" if stage1b else "≥0.8 m training starts"
    physical_label = "Physical contact | stationary frontier" if stage1b else "Physical contact"
    fig, axes = plt.subplots(2, 2, figsize=(9.6, 5.4), layout="constrained", sharex=True)
    fig.suptitle(f"RM27 {stage} Direct-CTBR PPO | 50k-sample bins with ≥5 eligible episodes", weight="bold")
    full_totals, frontier_totals = [], []
    for name, rows in runs:
        full_totals.append(f"{name} {int(rows['full_qualified'].sum()):,}/{int(rows['full_episodes'].sum()):,}")
        bin_index = rows["frames"].astype(int) // 50_000
        if stage1b:
            frontier = rows["frontier"].astype(int)
            # A promotion can split one 50k-sample bin. Keep its old and new
            # frontier starts separate in the qualified-rate series.
            bin_index = bin_index * (frontier.max(initial=0) + 1) + frontier
            for band in np.unique(frontier):
                band_rows = rows[frontier == band]
                qualified, episodes = (int(band_rows[key].sum()) for key in ("frontier_qualified", "frontier_episodes"))
                frontier_totals.append(f"Band {band}  {qualified:,}/{episodes:,}")
                if band > 0:
                    axes[0, 1].axvline(band_rows["frames"][0] / 1e6, color="0.6", ls=":", lw=1)
        samples, series = [], []
        for number in np.unique(bin_index):
            bucket = rows[bin_index == number]
            episodes = bucket["episodes"]
            has_episodes = episodes > 0
            all_count = episodes.sum()
            if all_count < 5:
                continue
            mean_error = np.average(bucket["mean_episode_min_error_m"][has_episodes], weights=episodes[has_episodes])
            near_count = bucket[f"{near_cohort}_episodes"].sum()
            near_qualified = bucket[f"{near_cohort}_qualified"].sum()
            physical_count = ((bucket["frontier_rejected_contact"].sum() + near_qualified)
                              if stage1b else bucket["physical_contacts"].sum())
            eligible = (near_count if stage1b else all_count, near_count, bucket["full_episodes"].sum())
            counts = (physical_count, near_qualified, bucket["full_qualified"].sum())
            metrics = [100 * count / total if total >= 5 else np.nan for count, total in zip(counts, eligible)]
            samples.append(bucket["frames"][-1] / 1e6)
            series.append((*metrics, 100 * mean_error))
        for ax, values in zip(axes.flat, np.asarray(series).T):
            ax.plot(samples, values, ".-", lw=1.1, ms=5, label=name)
    for ax, title, unit in zip(
        axes.flat, (physical_label, f"Qualified | {near_label}",
                    f"Qualified | {full_label}", "Mean closest bumper-to-goal distance"),
        ("Episodes (%)", "Episodes (%)", "Episodes (%)", "Distance (cm)"),
    ):
        upper = max(1., ax.get_ylim()[1])
        ax.set(title=title, ylabel=unit, ylim=(-.025 * upper, upper))
        ax.grid(alpha=.25)
    axes[1, 0].text(.98, .95, "\n".join(full_totals), transform=axes[1, 0].transAxes,
                    ha="right", va="top", fontsize=8)
    if stage1b:
        axes[0, 1].text(.98, .95, "\n".join(frontier_totals), transform=axes[0, 1].transAxes,
                        ha="right", va="top", fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Policy samples (millions)")
    axes[0, 0].legend(fontsize=8)
    _save_stage1_curves(fig, run_dir)


def stage1c_curves(run_dir: Path, runs: list) -> None:
    bridge = ["band_0_episodes" in rows.dtype.names for _, rows in runs]
    if any(bridge) and not all(bridge):
        raise ValueError(f"Braking bridge and handover training logs are mixed in {run_dir}")
    if all(bridge):
        informed = tuple(f"band_{i}" for i in range(6))
        bands = tuple(f"Gap band {i}" for i in range(6))
        stage = "Braking bridge"
    else:
        informed = ("informed_near", "informed_rotation", "informed_approach")
        bands = ("Informed near", "Informed rotation", "Informed approach", "Full stationary")
        stage = "Stage 1-C"
    fig, axes = plt.subplots(2, 2, figsize=(10, 5.8), layout="constrained", sharex=True)
    fig.suptitle(f"RM27 {stage} Direct-CTBR PPO | 50k-sample bins with ≥5 eligible episodes", weight="bold")
    rate_axes = (axes[0, 0], axes[0, 1], axes[1, 0])
    totals = [[], [], []]
    groups = (informed, ("frontier",), ("stationary",))
    for candidate, (name, rows) in enumerate(runs):
        color = f"C{candidate}"
        samples = rows["frames"] / 1e6
        frontier = rows["frontier"].astype(int)
        # Keep each curriculum phase distinct even if promotion splits a bin.
        bin_index = (rows["frames"].astype(int) // 50_000) * len(bands) + frontier
        for ax, group, annotation in zip(rate_axes, groups, totals):
            qualified = sum(rows[f"{cohort}_qualified"] for cohort in group)
            episodes = sum(rows[f"{cohort}_episodes"] for cohort in group)
            binned_x, rates = [], []
            for number in np.unique(bin_index):
                in_bin = bin_index == number
                count = episodes[in_bin].sum()
                binned_x.append(samples[in_bin][-1])
                rates.append(100 * qualified[in_bin].sum() / count if count >= 5 else np.nan)
            ax.plot(binned_x, rates, ".-", lw=1.1, ms=5, label=name, color=color)
            annotation.append(f"{name}  {int(qualified.sum()):,}/{int(episodes.sum()):,}")
        axes[1, 1].step(np.r_[0., samples], np.r_[frontier[:1], frontier], where="pre", color=color, label=name)
    for ax, title, annotation in zip(
        rate_axes, ("Qualified | informed starts", "Qualified | current frontier", "Qualified | full stationary starts"), totals,
    ):
        upper = max(1., ax.get_ylim()[1])
        ax.set(title=title, ylabel="Eligible episodes (%)", ylim=(-.025 * upper, upper))
        ax.text(.98, .95, "\n".join(annotation), transform=ax.transAxes, ha="right", va="top", fontsize=8,
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": .75})
        ax.grid(alpha=.25)
    frontier_ax = axes[1, 1]
    frontier_ax.set(title="Active curriculum frontier", yticks=np.arange(len(bands)), yticklabels=bands,
                    ylim=(-.25, len(bands) - .75))
    frontier_ax.grid(alpha=.25)
    frontier_ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Policy samples per candidate (millions)")
    _save_stage1_curves(fig, run_dir)


def _save_stage1_curves(fig, run_dir: Path) -> None:
    output = run_dir / "contact-learning.png"
    fig.savefig(output, dpi=145)
    plt.close(fig)
    print(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    curves = commands.add_parser("curves", help="Plot training.csv in a run directory")
    curves.add_argument("run_dir", type=Path)
    movie = commands.add_parser("replay", help="Render representative first episodes from eval telemetry")
    movie.add_argument("bundle", type=Path)
    movie.add_argument("--out-dir", type=Path)
    movie.add_argument("--select", choices=("both", "success", "failure"), default="both")
    contact = commands.add_parser("stage1-replay", help="Render an exact named native Stage 1 episode")
    contact.add_argument("bundle", type=Path)
    contact.add_argument("geometry", type=Path)
    contact.add_argument("output", type=Path)
    contact.add_argument("--case", default="nominal")
    contact_curves = commands.add_parser("stage1-curves", help="Compare Stage 1 training runs in 50k-sample bins")
    contact_curves.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    if args.command == "curves":
        plot_curves(args.run_dir)
    elif args.command == "stage1-replay":
        stage1_replay(args.bundle, args.geometry, args.output, args.case)
    elif args.command == "stage1-curves":
        stage1_curves(args.run_dir)
    else:
        replay(args.bundle, args.out_dir or args.bundle.parent, args.select)


if __name__ == "__main__":
    main()
