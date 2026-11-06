"""Recorded full-scene Stage 1 playback; this is not physics resimulation."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import selectors
import struct
import subprocess
import sys
import time
import zlib

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .stage1_plant import Stage1Plant


RENDER_VERSION = 2


def _episode(directory, modelpath=None):
    directory = Path(directory).resolve()
    metadata = json.loads((directory / "config.json").read_text())
    plant_config = metadata["rl"]
    if plant_config.get("task_id") == "state_flight_control_probe_v1":
        # Flight changes task/scorer identity, but instantiates the unchanged
        # contact-scene plant from this base configuration during training.
        from .paths import ROOT
        base = json.loads((ROOT / plant_config["base_task_config"]).read_text())
        if any(plant_config.get(key) != value for key, value in base.items()
               if key not in ("task_id", "observation_schema", "confirmation_scope")):
            raise ValueError("Recorded flight plant differs from its base configuration")
        plant_config = base
    plant = Stage1Plant(plant_config, model_path=modelpath or metadata["private_model_path"])
    with np.load(directory / "trajectory.npz", allow_pickle=False) as saved:
        trajectory = {name: saved[name].copy() for name in saved.files}
    times = trajectory["time_s"]
    if times.ndim != 1 or not len(times) or not np.isfinite(times).all() or np.any(np.diff(times) < 0):
        raise ValueError("Recorded trajectory needs finite monotonically increasing times")
    for name, size in (("qpos", plant.model.nq), ("qvel", plant.model.nv),
                       ("motor_state_N", plant.model.na), ("motor_commands_N", plant.model.nu),
                       ("action_ctbr", 4)):
        if trajectory[name].shape != (len(times), size) or not np.isfinite(trajectory[name]).all():
            raise ValueError(f"Invalid complete scene recording: {name}")
    return plant, trajectory


def _restore_frame(plant, trajectory, index):
    data = plant.data
    data.time = float(trajectory["time_s"][index])
    for name, key in (("qpos", "qpos"), ("qvel", "qvel"),
                      ("act", "motor_state_N"), ("ctrl", "motor_commands_N")):
        getattr(data, name)[:] = trajectory[key][index]
    mujoco.mj_forward(plant.model, data)


def _overlay(plant, trajectory, index):
    state = plant.state
    rpy = Rotation.from_matrix(state.rotation).as_euler("xyz", degrees=True)
    vector = lambda values: " ".join(f"{value:+.3f}" for value in values)
    return ("Recorded state playback\ntime [s]\nposition [m]\nvelocity [m/s]\nRPY [deg]\nCTBR [N, rad/s]",
            f"frame {index + 1}/{len(trajectory['time_s'])}\n{state.timestamp:.3f}\n"
            f"{vector(state.position)}\n{vector(state.velocity)}\n{vector(rpy)}\n"
            f"{vector(trajectory['action_ctbr'][index])}")


def _camera(plant):
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    target = np.asarray(plant.target_mapping["armor_surface_center_m"])
    position = plant.state.position
    camera.lookat[:] = (position + target) / 2
    camera.distance = max(1.2, float(np.linalg.norm(position - target)) * 1.8 + .8)
    camera.azimuth, camera.elevation = 150., -20.
    return camera


def view_episode(episode_dir, modelpath=None, *, playback_speed=1.):
    """Play every recorded frame in a passive viewer with state/control overlays."""
    import mujoco.viewer

    if not np.isfinite(playback_speed) or playback_speed <= 0:
        raise ValueError("Playback speed must be positive and finite")
    plant, trajectory = _episode(episode_dir, modelpath)
    _restore_frame(plant, trajectory, 0)
    camera = _camera(plant)
    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth, viewer.cam.elevation = camera.azimuth, camera.elevation
        times = trajectory["time_s"]
        started = time.monotonic()
        displayed = 0
        for index, stamp in enumerate(times):
            if not viewer.is_running():
                break
            deadline = started + (float(stamp) - float(times[0])) / playback_speed
            while viewer.is_running() and time.monotonic() < deadline:
                time.sleep(min(.02, max(0., deadline - time.monotonic())))
            with viewer.lock():
                _restore_frame(plant, trajectory, index)
            left, right = _overlay(plant, trajectory, index)
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,
                             mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
            viewer.sync()
            displayed += 1
    return dict(scope="recorded state playback; not independent physics resimulation",
                available_frames=len(trajectory["time_s"]), frames_displayed=displayed)


def _save_png(path, pixels):
    """Write the rendered RGB buffer without an optional imaging dependency."""
    height, width = pixels.shape[:2]
    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))
    raw = b"".join(b"\x00" + row.tobytes() for row in pixels)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def _render_worker(episode_dir, output_dir, indices, modelpath):
    plant, trajectory = _episode(episode_dir, modelpath)
    output = Path(output_dir)
    _restore_frame(plant, trajectory, indices[0])
    camera = _camera(plant)
    with mujoco.Renderer(plant.model, height=540, width=960, max_geom=plant.model.ngeom + 100) as renderer:
        print(json.dumps(dict(event="ready")), flush=True)
        for index in indices:
            started = time.monotonic()
            _restore_frame(plant, trajectory, index)
            renderer.update_scene(plant.data, camera=camera)
            pixels = renderer.render()
            left, right = _overlay(plant, trajectory, index)
            # MuJoCo 3.12 keeps the rendered framebuffer current. Draw into that
            # buffer and use the same orientation rule as Renderer.render().
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               renderer._rect, left, right, renderer._mjr_context)
            mujoco.mjr_readPixels(pixels, None, renderer._rect, renderer._mjr_context)
            if renderer._gl_context:
                pixels[:] = np.flipud(pixels)
            destination = output / f"frame_{index:05d}.png"
            _save_png(destination, pixels)
            print(json.dumps(dict(event="frame", index=index, time_s=float(trajectory["time_s"][index]),
                                  path=str(destination), overlay=True,
                                  render_wall_s=time.monotonic() - started)), flush=True)
    print(json.dumps(dict(event="done")), flush=True)


def render_episode(episode_dir, output_dir, modelpath=None, *, max_frames=3, frame_timeout_s=30.):
    """Render at most five stills, with a hard child-process setup/frame deadline."""
    if not isinstance(max_frames, int) or not 1 <= max_frames <= 5:
        raise ValueError("Headless engineering replay renders one to five frames")
    if not np.isfinite(frame_timeout_s) or frame_timeout_s <= 0:
        raise ValueError("Render timeout must be positive and finite")
    episode_dir, output = Path(episode_dir).resolve(), Path(output_dir).resolve()
    with np.load(episode_dir / "trajectory.npz", allow_pickle=False) as trajectory:
        count = len(trajectory["time_s"])
    if not count:
        raise ValueError("Cannot render an empty trajectory")
    indices = np.unique(np.linspace(0, count - 1, min(count, max_frames), dtype=int)).tolist()
    output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, "-m", __name__, "--worker", str(episode_dir), str(output),
               ",".join(map(str, indices)), str(modelpath) if modelpath else ""]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ, "stdout")
    selector.register(process.stderr, selectors.EVENT_READ, "stderr")
    result = dict(passed=False, scope="recorded state rendering; not independent physics resimulation",
                  requested_indices=indices, frames=[], frame_timeout_s=frame_timeout_s,
                  renderer=dict(version=RENDER_VERSION, mujoco_version=mujoco.__version__,
                                source=str(Path(__file__).resolve()),
                                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                overlay="MuJoCo mjr_overlay; dependency-free PNG"))
    deadline, errors, done = time.monotonic() + frame_timeout_s, [], False
    try:
        while selector.get_map():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Stage 1 rendering exceeded its setup or per-frame wall limit")
            for key, _ in selector.select(min(remaining, .5)):
                line = key.fileobj.readline()
                if not line:
                    selector.unregister(key.fileobj)
                    continue
                if key.data == "stderr":
                    if sum(map(len, errors)) + len(line) > 65536:
                        raise RuntimeError("Render diagnostics exceeded the bounded output limit")
                    errors.append(line)
                    continue
                message = json.loads(line)
                if message["event"] == "frame":
                    result["frames"].append(message)
                if message["event"] in ("ready", "frame"):
                    deadline = time.monotonic() + frame_timeout_s
                if message["event"] == "done":
                    done = True
        process.wait(timeout=max(1., deadline - time.monotonic()))
        result["passed"] = bool(process.returncode == 0 and done and len(result["frames"]) == len(indices))
        if not result["passed"]:
            raise RuntimeError("Headless recorded-state rendering did not complete")
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        selector.close()
        process.stdout.close()
        process.stderr.close()
        if errors:
            result["diagnostics"] = "".join(errors)
        (output / "render_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    if len(sys.argv) != 6 or sys.argv[1] != "--worker":
        raise SystemExit("Use the Stage 1 replay interface")
    _render_worker(sys.argv[2], sys.argv[3], [int(index) for index in sys.argv[4].split(",")], sys.argv[5] or None)
