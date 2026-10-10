"""Replay recorded physical states without rerunning the controller."""
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw


def load(run):
    run = Path(run).resolve()
    report = json.loads((run / "summary.json").read_text())
    model = mujoco.MjModel.from_binary_path(str(run / "model.mjb"))
    trace = np.load(run / "trace.npz")
    data = mujoco.MjData(model)
    return model, data, trace, report


def pose(model, data, trace, index, launched):
    data.qpos[:] = trace["qpos"][index]
    data.qvel[:] = trace["qvel"][index]
    data.act[:] = trace["act"][index]
    data.ctrl[:] = trace["ctrl"][index]
    data.time = float(trace["time"][index])
    visual = model.geom("intercept_dart_visual").id
    model.geom_rgba[visual, 3] = 1 if launched else 0
    mujoco.mj_forward(model, data)


def camera(model, data, trace, index, mode):
    cam = mujoco.MjvCamera()
    if mode == "overview":
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0., -2.5, 1.8]
        cam.distance = 29.
        cam.azimuth = -90
        cam.elevation = -25
    elif mode == "target":
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = data.qpos[:3]
        cam.lookat[2] = max(0.8, float(data.qpos[2]))
        cam.distance = 1.8
        cam.azimuth = -90
        cam.elevation = -13
    elif mode == "follow":
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        dq = model.joint("intercept_dart_free").qposadr[0]
        cam.lookat[:] = data.qpos[dq:dq+3]
        cam.distance = 1.6
        cam.azimuth = -90
        cam.elevation = -28
    else:
        raise ValueError(mode)
    return cam


def phase(report, t):
    launch = report["launch_time_s"]
    if t < report["inputs"]["config"]["takeoff_s"]:
        return "TAKEOFF"
    if t < launch:
        return "HOVER"
    if report["intercept"] and t >= report["intercept"]["time_s"]:
        return "DART / UAV CONTACT"
    if report["base_proxy_contact"] and t >= report["base_proxy_contact"]["time_s"]:
        return "BASE PROXY CONTACT"
    return "INTERCEPT" if report["active"] else "DART FLIGHT / PASSIVE UAV"


def render(run, output, video=False):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    model, data, trace, report = load(run)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    times = trace["time"]
    with mujoco.Renderer(model, height=540, width=960) as renderer:
        def frame(t, mode="target", annotate=True):
            index = int(np.argmin(abs(times-t)))
            now = float(times[index])
            pose(model, data, trace, index, now >= report["launch_time_s"])
            renderer.update_scene(data, camera=camera(model, data, trace, index, mode), scene_option=option)
            image = Image.fromarray(renderer.render())
            if annotate:
                draw = ImageDraw.Draw(image)
                draw.rectangle((0, 0, 960, 49), fill=(22, 27, 32))
                draw.text((12, 7), f"DART INTERCEPTION | {phase(report, now)} | t={now:.2f} s", fill="white")
                label = ("flight-04-apex-5m pose until contact / dynamic impact / truth-guided UAV"
                         if "dart_release" in report else
                         "MuJoCo physics / truth-guided motor control / assumed dart mass and collision proxy")
                draw.text((12, 27), label, fill=(255, 213, 119))
            return image

        crossing = report["inputs"]["config"]["intercept_dart_time_s"]
        snapshots = [("overview", 0., "overview"), ("dock_start", 0., "target"),
                     ("takeoff", .55, "target"), ("launch", report["launch_time_s"]+.12, "follow"),
                     ("dart_apex", report["launch_time_s"]+.9, "follow"),
                     ("dart_descending", report["launch_time_s"]+crossing-.02, "follow"),
                     ("approach", report["launch_time_s"]+crossing*.7, "target"),
                     ("contact", report["intercept"]["time_s"] if report["intercept"] else
                      report["launch_time_s"]+1.73, "target"),
                     ("end", float(times[-1]), "target")]
        for name, t, mode in snapshots:
            frame(t, mode).save(output / f"{name}.png")
        frames = 0
        if video:
            import cv2
            writer = cv2.VideoWriter(str(output / "flight.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                     30, (960, 540))
            if not writer.isOpened():
                raise RuntimeError("OpenCV MP4 encoder unavailable")
            try:
                for i in range(int(np.ceil((times[-1]/.5+.4)*30))):
                    t = min(i/30*.5, float(times[-1]))
                    mode = "follow" if report["launch_time_s"] <= t < report["launch_time_s"]+.7 else "target"
                    image = frame(t, mode)
                    inset = frame(t, "overview", False).resize((256, 144))
                    image.paste(inset, (692, 384))
                    writer.write(np.asarray(image)[:, :, ::-1].copy())
                    frames += 1
            finally:
                writer.release()
    (output / "render.json").write_text(json.dumps(dict(source_run=str(Path(run).resolve()),
        mode="offscreen replay of recorded simulation states", source_mode=report["mode"], frames=frames,
        fps=30 if video else None, playback_speed=.5), indent=2)+"\n")
    return output


def view(run, speed=1., camera_mode="free"):
    if speed <= 0:
        raise ValueError("speed must be positive")
    model, data, trace, report = load(run)
    import mujoco.viewer
    selected = [camera_mode]
    def key_callback(key):
        modes = {ord("1"): "overview", ord("2"): "target", ord("3"): "follow",
                 ord("4"): "auto", ord("5"): "free"}
        if key in modes:
            selected[0] = modes[key]
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.opt.geomgroup[3] = 0
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        initial = camera(model, data, trace, 0, "overview")
        viewer.cam.lookat[:] = initial.lookat
        viewer.cam.distance = initial.distance
        viewer.cam.azimuth = initial.azimuth
        viewer.cam.elevation = initial.elevation
        t0 = time.monotonic()
        while viewer.is_running():
            t = ((time.monotonic()-t0)*speed) % float(trace["time"][-1])
            index = int(np.argmin(abs(trace["time"]-t)))
            pose(model, data, trace, index, t >= report["launch_time_s"])
            mode = selected[0]
            if mode == "auto":
                mode = "follow" if report["launch_time_s"] <= t < report["launch_time_s"]+.7 else "target"
            if mode == "follow" and t < report["launch_time_s"]:
                mode = "overview"
            # Free mode leaves the viewer's mouse-controlled camera untouched.
            if mode != "free":
                cam = camera(model, data, trace, index, mode)
                with viewer.lock():
                    viewer.cam.type = cam.type
                    viewer.cam.lookat[:] = cam.lookat
                    viewer.cam.distance = cam.distance
                    viewer.cam.azimuth = cam.azimuth
                    viewer.cam.elevation = cam.elevation
            viewer.sync()
            time.sleep(.01)
