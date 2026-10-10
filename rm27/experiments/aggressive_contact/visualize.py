"""Scientific plots and deterministic recorded-motor replay verification."""
import json
from pathlib import Path

import mujoco
import numpy as np
from .recording import load_model
from scipy.spatial.transform import Rotation

from .evidence import write_json


def replay(source, output):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    x = np.load(source/"trajectory.npz")["trace"]
    model = load_model(source)
    data = mujoco.MjData(model)
    data.time = x[0, 0]
    data.qpos[:], data.qvel[:], data.act[:] = x[0, 1:8], x[0, 8:14], x[0, 14:18]
    mujoco.mj_forward(model, data)
    errors = np.zeros(3)
    for i, row in enumerate(x):
        errors = np.maximum(errors, [np.max(np.abs(data.qpos-row[1:8])), np.max(np.abs(data.qvel-row[8:14])),
                                    np.max(np.abs(data.act-row[14:18]))])
        data.ctrl[:] = row[18:22]
        if i+1 == len(x):
            break
        for _ in range(round((x[i+1, 0]-row[0])/model.opt.timestep)):
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
    result = dict(passed=bool(np.all(errors < 1e-8)), max_qpos_error=float(errors[0]),
                  max_qvel_error=float(errors[1]), max_motor_state_error=float(errors[2]), samples=len(x),
                  scope="recorded motor input deterministic physics; not an independent closed-loop reliability trial")
    write_json(output/"summary.json", result)
    return result


def plot(source, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    summary = json.loads((source/"summary.json").read_text())
    paths = []
    if "evaluations" in summary:
        groups = summary["evaluations"]
        if "arena_patch" in groups:
            groups = {f"{geometry} / {split}": methods for geometry, splits in groups.items()
                      for split, methods in splits.items()}
        nrows = (len(groups)+1)//2
        fig, axes = plt.subplots(nrows, 2, figsize=(13, 4*nrows), squeeze=False, constrained_layout=True)
        for ax, (name, methods) in zip(axes.ravel(), groups.items()):
            labels, rates, intervals = [], [], []
            for method, stats in methods.items():
                labels.append(method.replace("_", "\n"))
                rates.append(stats["success_rate"])
                intervals.append(stats["wilson_95"])
            intervals = np.array(intervals)
            ax.bar(labels, rates, color=["#7b8794", "#db9248", "#327ab7", "#529576", "#8561a5"])
            for i, stats in enumerate(methods.values()):
                ax.text(i, .03, f'{stats["successes"]}/{stats["trials"]}', ha="center", color="white")
            ax.errorbar(range(len(rates)), rates, yerr=np.maximum(0, np.array([rates-intervals[:, 0], intervals[:, 1]-rates])), fmt="none", color="black", capsize=4)
            ax.set(title=name.replace("_", " "), ylabel="Joint contact + recovery success", ylim=(0, 1.04))
        fig.suptitle("Held-out paired trials; 95% Wilson intervals; MuJoCo truth")
        fig.savefig(output/"comparison.png", dpi=180)
        plt.close(fig)
        paths.append("comparison.png")
        source = source/"demo"
    elif "grid_trials" in summary:
        fig, axes = plt.subplots(1, 3, figsize=(12, 4), constrained_layout=True)
        for ax, h in zip(axes, [.6, 1.2, 1.8]):
            rows = [r for r in summary["rows"][:75] if r["initial"]["altitude_m"] == h]
            ax.scatter([r["initial"]["pitch_deg"] for r in rows], [r["initial"]["omega_rad_s"][1] for r in rows],
                       c=["#36866a" if r["passed"] else "#c35353" for r in rows], s=180, marker="s")
            ax.set(title=f"Initial altitude {h} m", xlabel="Initial pitch (deg)", ylabel="Initial pitch rate (rad/s)")
        fig.suptitle("Empirical recovery map: green PASS / red FAIL; vx=0.4, vz=-0.3 m/s")
        fig.savefig(output/"recovery-map.png", dpi=180)
        plt.close(fig)
        write_json(output/"summary.json", dict(passed=True, plots=["recovery-map.png"]))
        return dict(passed=True, plots=[str(output/"recovery-map.png")])
    x = np.load(source/"trajectory.npz")["trace"]
    result = json.loads((source/"summary.json").read_text())
    terminal = result.get("terminal") or result
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(x[:, 1], x[:, 3], label="actual")
    ax.plot(x[:, 22], x[:, 24], "--", label="reference")
    cfg = json.loads((source/"physical.json").read_text())
    from .model import frame
    center = np.array(cfg["target"]["center_m"])
    tangent = frame(cfg["target"]["normal"])[:, 0]
    plate = np.array([center-.2*tangent, center+.2*tangent])
    ax.plot(plate[:, 0], plate[:, 2], color="black", linewidth=3, label="plate + rim")
    ax.set(xlabel="x (m)", ylabel="z (m)", title="Continuous approach and recovery")
    ax.axis("equal")
    ax.legend()
    axes[0, 1].plot(x[:, 0], np.linalg.norm(x[:, 8:11], axis=1), label="speed")
    axes[0, 1].plot(x[:, 0], x[:, 8], label="vx")
    axes[0, 1].plot(x[:, 0], x[:, 10], label="vz")
    axes[0, 1].set(title="Velocity", ylabel="m/s")
    axes[0, 1].legend()
    angles = Rotation.from_quat(x[:, [5, 6, 7, 4]]).as_euler("xyz", degrees=True)
    refangles = Rotation.from_quat(x[:, [34, 35, 36, 33]]).as_euler("xyz", degrees=True)
    axes[0, 2].plot(x[:, 0], angles[:, 1], label="pitch")
    axes[0, 2].plot(x[:, 0], refangles[:, 1], "--", label="desired pitch")
    axes[0, 2].plot(x[:, 0], angles[:, 0], label="roll")
    axes[0, 2].set(title="Force-derived attitude", ylabel="degrees")
    axes[0, 2].legend()
    axes[1, 0].plot(x[:, 0], x[:, 11:14])
    axes[1, 0].set(title="Body rates", ylabel="rad/s")
    axes[1, 1].plot(x[:, 0], x[:, 14:18])
    axes[1, 1].set(title="Motor force states (before scale)", ylabel="N")
    axes[1, 2].plot(x[:, 0], np.linalg.norm(x[:, 1:4]-x[:, 22:25], axis=1))
    axes[1, 2].set(title="Position error (anchor changes on contact)", ylabel="m")
    for ax in axes.ravel()[1:]:
        ax.set_xlabel("time (s)")
        ax.grid(alpha=.2)
        if terminal.get("first_contact"):
            ax.axvline(terminal["first_contact"]["time_s"], color="gray", linestyle=":")
    fig.suptitle(f"MuJoCo closed-loop truth teacher — {'PASS' if result['passed'] else 'FAIL'}")
    fig.savefig(output/"trajectory.png", dpi=180)
    plt.close(fig)
    paths.append("trajectory.png")
    write_json(output/"summary.json", dict(passed=True, plots=paths))
    return dict(passed=True, plots=[str(output/p) for p in paths])
