"""Read-only viser web viewer for Agile Terminal Arrival (native MuJoCo; never writes training or evaluation data).

Live mode loads a checkpoint (policy + frozen VecNormalize) exactly as evaluation does and steps the real
environment with the deterministic policy; replay mode plays stored evaluation ``trajectory.npz`` files with a
time slider and several runs overlaid. The full aircraft (including geom group 5 visuals), armor plate and
trajectory lines are drawn.

Run (viser is not in the locked environment; the viewer venv reuses .venv and adds viser only):
    .agents/runs/viewer/venv/bin/python tools/viser_viewer.py [--port 8080] [--root artifacts/agile_flight/docker]
then forward the port (VS Code "Ports") and open http://localhost:8080.
"""
import argparse
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import trimesh
import viser

from rm27_contact_rl.arrival_env import AgileTerminalArrivalEnv
from rm27_contact_rl.ppo import _load_checkpoint

ROOT = Path(__file__).resolve().parents[1]
FIXED_STARTS = [[1.2, 0, 0], [1.15, -.025, -.025], [1.15, -.025, .025], [1.15, .025, -.025], [1.15, .025, .025],
                [1.25, -.025, -.025], [1.25, -.025, .025], [1.25, .025, -.025], [1.25, .025, .025]]
RUN_COLORS = [(42, 120, 214), (235, 104, 52), (27, 175, 122), (237, 161, 0), (232, 123, 164), (98, 80, 214)]
SHOWN_GROUPS = {0, 1, 2, 5}


def geom_mesh(model, g):
    """Trimesh primitive for one MuJoCo geom in its own frame, or None for planes/unsupported types."""
    kind, size = model.geom_type[g], model.geom_size[g]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return trimesh.creation.box(extents=2 * size)
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return trimesh.creation.icosphere(subdivisions=2, radius=size[0])
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return trimesh.creation.capsule(height=2 * size[1], radius=size[0], count=[12, 12])
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return trimesh.creation.cylinder(radius=size[0], height=2 * size[1], sections=24)
    if kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=1.)
        mesh.apply_scale(size)
        return mesh
    return None


def add_body(server, model, body, prefix, tint=None, opacity=None):
    """Body frame node with every displayed geom attached at its local pose; returns the frame handle."""
    frame = server.scene.add_frame(prefix, show_axes=False)
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body or model.geom_group[g] not in SHOWN_GROUPS:
            continue
        mesh = geom_mesh(model, g)
        if mesh is None:
            continue
        rgba = model.geom_rgba[g]
        color = tuple(int(255 * c) for c in rgba[:3]) if tint is None else tint
        alpha = float(rgba[3]) if opacity is None else opacity
        server.scene.add_mesh_simple(f"{prefix}/g{g}", mesh.vertices, mesh.faces, color=color,
                                     opacity=alpha if alpha < 1 else None,
                                     position=model.geom_pos[g], wxyz=model.geom_quat[g])
    return frame


def path_segments(points):
    points = np.asarray(points, float)
    return np.stack([points[:-1], points[1:]], axis=1) if len(points) > 1 else np.zeros((0, 2, 3))


class Viewer:
    def __init__(self, server, results_root):
        self.server, self.results_root = server, Path(results_root)
        self.lock = threading.RLock()
        server.scene.set_up_direction("+z")
        server.scene.add_grid("/ground", width=4, height=4, cell_size=0.25, section_size=1.0)
        self.geometry_env = AgileTerminalArrivalEnv()
        model = self.geometry_env.plant.model
        mujoco.mj_forward(model, self.geometry_env.plant.data)
        self.model = model
        self.uav_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "uav")
        data = self.geometry_env.plant.data
        for b in range(1, model.nbody):  # static bodies (armor plate and rim)
            if b != self.uav_body:
                frame = add_body(server, model, b, f"/static/b{b}")
                frame.position, frame.wxyz = data.xpos[b].copy(), data.xquat[b].copy()
        self.target = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")].copy()

        @server.on_client_connect
        def _(client):
            client.camera.position = self.target + np.array([1.6, -1.6, 1.4])
            client.camera.look_at = self.target + np.array([0.4, 0., 0.5])

        mode = server.gui.add_dropdown("Mode", ("Live", "Replay"), initial_value="Live")
        self.status = server.gui.add_markdown("")
        self._build_live_gui()
        self._build_replay_gui()

        @mode.on_update
        def _(_):
            self.set_mode(mode.value)
        self.set_mode("Live")
        threading.Thread(target=self._live_loop, daemon=True).start()
        threading.Thread(target=self._replay_loop, daemon=True).start()

    # ---------------------------------------------------------------- live
    def _build_live_gui(self):
        gui = self.server.gui
        self.live_folder = gui.add_folder("Live policy")
        with self.live_folder:
            checkpoints = sorted(str(p.relative_to(self.results_root)) for p in
                                 self.results_root.glob("*/checkpoints/*") if (p / "checkpoint.json").is_file())
            self.ckpt = gui.add_dropdown("Checkpoint", checkpoints or ("(none found)",),
                                         initial_value=next((c for c in checkpoints if "x17_yaw3_seed1" in c
                                                             and c.endswith("final")), None))
            load = gui.add_button("Load checkpoint")
            self.start = gui.add_dropdown("Initial state", [f"fixed {i}" for i in range(9)] + ["random"],
                                          initial_value="fixed 0")
            reset = gui.add_button("Reset")
            self.live_play = gui.add_checkbox("Play", True)
            self.live_speed = gui.add_slider("Speed (x real time)", 0.05, 2.0, 0.05, 0.25)
            self.auto_restart = gui.add_checkbox("Auto restart after 1 s", False)
            self.disturb = gui.add_checkbox("Mass/thrust disturbance (applied at reset)", False)
            self.mass = gui.add_slider("Mass scale", 0.7, 1.4, 0.01, 1.0)
            self.thrust = gui.add_slider("Thrust scale", 0.7, 1.3, 0.01, 1.0)
        self.live_uav = add_body(self.server, self.model, self.uav_body, "/live/uav")
        self.live_path = None
        self.live = None  # dict(model, vec, episode, raw, points, done, result, t)

        @load.on_click
        def _(_):
            self._load(self.ckpt.value)

        @reset.on_click
        def _(_):
            self._reset()

    def _load(self, relative):
        path = self.results_root / relative
        self.status.content = f"Loading `{relative}` ..."
        try:
            model, vec, meta = _load_checkpoint(path, training=False, n_envs=1)
        except Exception as error:  # report, keep the viewer alive
            self.status.content = f"**Load failed:** {error}"
            return
        with self.lock:
            if self.live is not None:
                self.live["vec"].close()
            self.live = dict(model=model, vec=vec, episode=vec.envs[0], name=relative)
        self._reset()

    def _reset(self):
        with self.lock:
            live = self.live
            if live is None:
                self.status.content = "Load a checkpoint first."
                return
            base = live["episode"].unwrapped
            scales = (dict(mass=self.mass.value, thrust=self.thrust.value) if self.disturb.value
                      else dict(mass=1., thrust=1.))
            base.set_dynamics_scales(**scales)
            if self.start.value == "random":
                coordinates = base.np_random.uniform(base.initial_ranges[:, 0], base.initial_ranges[:, 1]).tolist()
            else:
                coordinates = FIXED_STARTS[int(self.start.value.split()[1])]
            raw, _ = live["episode"].reset(options={"initial_coordinates": coordinates})
            live.update(raw=raw, points=[base.plant.data.qpos[:3].copy()], done=False, result=None,
                        coordinates=coordinates, scales=scales, done_wall=None)
            self._draw_live(base, force=True)

    def _draw_live(self, base, force=False):
        data = base.plant.data
        self.live_uav.position = data.qpos[:3].copy()
        self.live_uav.wxyz = data.qpos[3:7].copy()
        points = self.live["points"]
        if force or len(points) % 8 == 0 or self.live["done"]:
            if self.live_path is not None:
                self.live_path.remove()
            self.live_path = self.server.scene.add_line_segments(
                "/live/path", path_segments(points), colors=(42, 120, 214), thickness=0.004)
        elapsed = data.time - base.episode_start
        lines = [f"**Live** `{self.live['name']}`",
                 f"start {np.round(self.live['coordinates'], 3).tolist()}, mass x{self.live['scales']['mass']:.2f}, "
                 f"thrust x{self.live['scales']['thrust']:.2f}",
                 f"t = {elapsed:.3f} s, speed = {np.linalg.norm(data.qvel[:3]):.2f} m/s, "
                 f"|w| = {np.linalg.norm(data.qvel[3:6]):.1f} rad/s"]
        if self.live["result"] is not None:
            r = self.live["result"]
            verdict = "SUCCESS" if r["passed"] else f"FAIL {r['termination_reason']}"
            lines.append(f"**{verdict}** at {r['episode_elapsed_s']:.3f} s, "
                         f"rho_best {r.get('best_terminal_violation_ratio', float('nan')):.2f}")
        self.status.content = "  \n".join(lines)

    def _live_loop(self):
        wall_anchor, sim_anchor, last_draw = time.time(), 0., 0.
        while True:
            if self.mode != "Live" or not self.live_play.value or self.live is None:
                time.sleep(0.05)
                wall_anchor = None
                continue
            live = self.live
            base = live["episode"].unwrapped
            if live["done"]:
                if self.auto_restart.value and time.time() - live["done_wall"] > 1.0:
                    self._reset()
                    wall_anchor = None
                else:
                    time.sleep(0.02)
                continue
            with self.lock:
                sim_now = base.plant.data.time - base.episode_start
                if wall_anchor is None:
                    wall_anchor, sim_anchor = time.time(), sim_now
                ahead = (sim_now - sim_anchor) / self.live_speed.value - (time.time() - wall_anchor)
                if ahead > 0:
                    pass
                else:
                    action, _ = live["model"].predict(live["vec"].normalize_obs(live["raw"]), deterministic=True)
                    raw, _, terminated, truncated, info = live["episode"].step(action)
                    live["raw"] = raw
                    live["points"].append(base.plant.data.qpos[:3].copy())
                    if terminated or truncated:
                        live.update(done=True, result=dict(info["result"]), done_wall=time.time())
                    if live["done"] or time.time() - last_draw > 1 / 60:
                        self._draw_live(base)
                        last_draw = time.time()
                    continue
            time.sleep(min(ahead * self.live_speed.value, 0.01))

    # -------------------------------------------------------------- replay
    def _build_replay_gui(self):
        gui = self.server.gui
        self.replay_folder = gui.add_folder("Replay")
        with self.replay_folder:
            episodes = sorted(str(p.parent.relative_to(self.results_root))
                              for p in self.results_root.glob("*/evaluation_after*/episode_*/trajectory.npz"))
            self.episode_pick = gui.add_dropdown("Episode", episodes or ("(none found)",))
            add = gui.add_button("Add to overlay")
            clear = gui.add_button("Clear overlay")
            self.slider = gui.add_slider("Time (s)", 0.0, 3.0, 0.005, 0.0)
            self.replay_play = gui.add_checkbox("Play", False)
            self.replay_speed = gui.add_slider("Replay speed", 0.05, 2.0, 0.05, 0.25)
            self.overlay_md = gui.add_markdown("")
        self.runs = []

        @add.on_click
        def _(_):
            self._add_run(self.episode_pick.value)

        @clear.on_click
        def _(_):
            for run in self.runs:
                run["frame"].remove()
                run["path"].remove()
            self.runs = []
            self._overlay_text()

        @self.slider.on_update
        def _(_):
            self._pose_runs(self.slider.value)

    def _add_run(self, relative):
        directory = self.results_root / relative
        try:
            data = np.load(directory / "trajectory.npz")
        except Exception as error:
            self.status.content = f"**Cannot read** `{relative}`: {error}"
            return
        k = len(self.runs)
        color = RUN_COLORS[k % len(RUN_COLORS)]
        t = data["time_s"] - data["time_s"][0]
        q = data["qpos"]
        frame = add_body(self.server, self.model, self.uav_body, f"/replay/r{k}/uav",
                         tint=None if k == 0 else color, opacity=None if k == 0 else 0.6)
        path = self.server.scene.add_line_segments(f"/replay/r{k}/path", path_segments(q[:, :3]), colors=color,
                                                   thickness=0.004)
        summary = directory / "summary.json"
        verdict = ""
        if summary.is_file():
            import json
            s = json.loads(summary.read_text())
            verdict = "success" if s["passed"] else f"fail {s['termination_reason']}"
        self.runs.append(dict(name=relative, t=t, q=q, frame=frame, path=path, color=color, verdict=verdict))
        self.slider.max = float(max(run["t"][-1] for run in self.runs))
        self._overlay_text()
        self._pose_runs(self.slider.value)

    def _overlay_text(self):
        rows = [f"<span style='color:rgb{run['color']}'>&#9632;</span> `{run['name']}` "
                f"({run['t'][-1]:.2f} s, {run['verdict']})" for run in self.runs]
        self.overlay_md.content = "  \n".join(rows) if rows else "No runs loaded."

    def _pose_runs(self, time_s):
        for run in self.runs:
            i = min(int(np.searchsorted(run["t"], time_s)), len(run["t"]) - 1)
            run["frame"].position = run["q"][i, :3]
            run["frame"].wxyz = run["q"][i, 3:7]
        if self.mode == "Replay":
            self.status.content = f"**Replay** t = {time_s:.3f} s, {len(self.runs)} run(s)"

    def _replay_loop(self):
        last = time.time()
        while True:
            time.sleep(1 / 60)
            now = time.time()
            if self.mode == "Replay" and self.replay_play.value and self.runs:
                value = self.slider.value + (now - last) * self.replay_speed.value
                self.slider.value = 0.0 if value > self.slider.max else round(value, 3)
            last = now

    # ---------------------------------------------------------------- mode
    def set_mode(self, mode):
        self.mode = mode
        live = mode == "Live"
        self.live_folder.visible, self.replay_folder.visible = live, not live
        self.live_uav.visible = live
        if self.live_path is not None:
            self.live_path.visible = live
        for run in self.runs:
            run["frame"].visible = run["path"].visible = not live
        if live and self.live is None:
            self.status.content = "Pick a checkpoint and press **Load checkpoint**."
        elif not live:
            self._pose_runs(self.slider.value)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--root", type=Path, default=ROOT / "artifacts/agile_flight/docker")
    args = parser.parse_args()
    server = viser.ViserServer(host=args.host, port=args.port)
    Viewer(server, args.root)
    print(f"viser viewer on http://{args.host}:{args.port}  (forward this port, then open it in a browser)")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
