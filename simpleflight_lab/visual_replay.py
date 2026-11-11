"""Interactive RM27 3D replay; recorded native states and fresh Isaac inference.

Only this viewer's USD session receives visual meshes and display styling.
All dynamics, accepted collision shapes, action decoding and scoring inputs
come from the existing frozen research implementation. No optimizer is loaded.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import time

import numpy as np

from .replay_data import ROOT, load_demos, load_manifest


class Viewer:
    DEMOS = ("contact", "witness", "failure")
    CAMERAS = ("Free orbit", "Follow drone", "Fixed target", "Bumper detail")
    SPEEDS = (.25, .5, 1.)

    def __init__(self, app, args):
        import carb
        import omni.ui as ui
        import omni.usd
        import torch
        from pxr import Gf, UsdGeom, UsdLux
        from .stage1_experiment import load_stage1c_experiment
        from .stage1_sim import Stage1Sim
        from .stage1_task import Stage1Actor, Stage1Task

        self.app, self.args = app, args
        self.clips = load_demos()
        self.manifest = load_manifest()
        parent, assets, self.frozen, self.geometry = load_stage1c_experiment()
        scene = ROOT / self.manifest["files"]["viewer_scene"]["path"]
        # Pinned production PGS settings are checked without charging any learning.
        from .stage1_brake_training import CONFIG, assert_live_scene, load_config
        from .stage1_experiment import digest
        config, _ = load_config(CONFIG)
        print("RM27_VIEWER_INPUTS_VERIFIED", flush=True)
        self.sim = Stage1Sim(1, scene, solver_type=0)
        assert_live_scene(config["sim"])
        self.task = Stage1Task(1, self.sim.device, self.geometry, self.frozen,
                               training=False, goal_inset_m=parent["goal_inset_m"])
        checkpoint = self.clips["contact"].checkpoint
        state = torch.load(checkpoint, map_location=self.sim.device, weights_only=True)
        if (state["bridge_config_sha256"] != digest(CONFIG)
                or state["frames"] != 1_998_848 or state["actor_obs"] != 21):
            raise ValueError("Viewer requires the frozen selected contact actor")
        self.actor = Stage1Actor().to(self.sim.device)
        self.actor.load_state_dict(state["actor"], strict=True)
        self.actor.eval()
        print("RM27_VIEWER_PHYSICS_ACTOR_READY", flush=True)
        self.ids = torch.zeros(1, dtype=torch.long, device=self.sim.device)
        self.policy_ticks = self.ids.clone()
        self.center = np.asarray(self.geometry["target"]["center_m"])
        self.normal = np.asarray(self.geometry["target"]["normal"])
        self.frame = np.asarray(self.geometry["target"]["target_rotation"])
        self.halfsize = np.asarray(self.geometry["target"]["halfsize_m"])
        self.offset = self.sim._origins[0].astype(float)
        self.stage = omni.usd.get_context().get_stage()
        # This RTX version supports constant curve colors; one prim per vector
        # preserves the axis colors without unsupported vertex interpolation.
        self.lines = []
        for i in range(5):
            line = UsdGeom.BasisCurves.Define(self.stage, f"/World/ReplayOverlays/Vector{i}")
            line.CreateTypeAttr("linear")
            line.CreateWrapAttr("nonperiodic")
            line.CreateWidthsAttr([.002])
            line.SetWidthsInterpolation("constant")
            self.lines.append(line)
        self.points = UsdGeom.Points.Define(self.stage, "/World/ReplayOverlays/Contacts")
        self.points.CreateWidthsAttr([.012])
        self.points.SetWidthsInterpolation("constant")
        self.points.CreateDisplayColorAttr([(1., .2, .05)])
        self.speeds = self.SPEEDS
        self.speed, self.camera, self.mode = .5, 0, 0
        self.demo, self.playing, self.loop = args.demo, not args.paused, False
        self.show_axes, self.show_normal, self.show_velocity, self.show_contacts = True, True, True, True
        self.current, self.outcome, self.contact_event = None, None, None
        self.accumulator, self.elapsed, self.physics_tick = 0., 0., 0
        self.saturated, self.terminal = 0, False
        self.last_real = time.perf_counter()
        self.checks = []

        # Retain all collision geometry for physics, render the actual source meshes.
        for prim in self.stage.Traverse():
            path = str(prim.GetPath())
            # Do not send USD notices to a live tensor-controlled rigid body.
            # Its visibility is composed before Stage1Sim initializes physics.
            if path.startswith("/World/Stage1Envs/env_0/Drone"):
                continue
            if path.startswith("/World/Stage1OverlapProbe"):
                if path == "/World/Stage1OverlapProbe":
                    UsdGeom.Imageable(prim).MakeInvisible()
                continue
            if prim.HasAttribute("stage1:geomName"):
                name = prim.GetAttribute("stage1:geomName").Get()
                image = UsdGeom.Imageable(prim)
                if name == "drone_1_stage1_contact":
                    image.MakeInvisible()  # rendered as a visual-only exact-radius twin below
                elif prim.GetAttribute("stage1:registered").Get():
                    UsdGeom.Gprim(prim).CreateDisplayColorAttr([(0.13, .9, .53)])
                    image.MakeVisible()
                else:
                    image.MakeInvisible()
        visual_file = ROOT / self.manifest["files"]["visual_usd"]["path"]
        field = UsdGeom.Xform.Define(self.stage, "/World/ReplayField")
        field.GetPrim().GetReferences().AddReference(str(visual_file), "/Visuals/Field")
        field.AddTranslateOp(opSuffix="displayOrigin").Set(Gf.Vec3d(*self.offset))
        drone = UsdGeom.Xform.Define(self.stage, "/World/ReplayAircraft")
        drone.GetPrim().GetReferences().AddReference(str(visual_file), "/Visuals/Drone")
        drone.ClearXformOpOrder()
        self.translation = drone.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self.orientation = drone.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        fixture = next(g for g in self.geometry["aircraft"] if g["name"] == "drone_1_stage1_contact")
        bumper = UsdGeom.Sphere.Define(self.stage, "/World/ReplayAircraft/UndersideBumper")
        bumper.CreateRadiusAttr(fixture["size"][0])
        bumper.AddTranslateOp().Set(Gf.Vec3d(*fixture["position_m"]))
        bumper.CreateDisplayColorAttr([(1., .62, .04)])
        self.fixture_offset = np.asarray(fixture["position_m"])
        self.fixture_radius = fixture["size"][0]
        self.translation_type, self.orientation_type = Gf.Vec3d, Gf.Quatd
        light = UsdLux.DomeLight.Define(self.stage, "/World/ReplayLighting/Dome")
        light.CreateIntensityAttr(650.)
        key = UsdLux.DistantLight.Define(self.stage, "/World/ReplayLighting/Key")
        key.CreateIntensityAttr(2000.)
        key.AddRotateXYZOp().Set(Gf.Vec3f(-35, -25, 20))
        # Visual-only coordinates use the same world translation as the solver.
        self.reset()
        self._build_ui(ui)
        carb.settings.get_settings().set_bool("/app/window/hideUi", False)
        self.set_camera(3 if self.demo == "contact" else 2)
        self.camera = 0
        self.status_update()

    def _build_ui(self, ui):
        self.window = ui.Window("RM27 | Replay", width=365, height=760, position_x=905, position_y=45)
        with self.window.frame:
            with ui.VStack(spacing=7, style={"font_size": 14}):
                ui.Label("RM27 VER2  |  2027 TARGET", height=24, style={"font_size": 19, "color": 0xFFB0E9D0})
                ui.Label("Original meshes / frozen dynamics / no learning", height=18)
                self.demo_combo = ui.ComboBox(self.DEMOS.index(self.demo), "A  PPO terminal contact", "B  Deterministic witness", "C  PPO stationary failure", height=28)
                self.demo_combo.model.add_item_changed_fn(lambda model, _: self.select_demo(self.DEMOS[model.get_item_value_model().as_int]))
                self.mode_combo = ui.ComboBox(0, "Recorded MuJoCo states", "New Isaac simulation", height=28)
                self.mode_combo.model.add_item_changed_fn(lambda model, _: self.select_mode(model.get_item_value_model().as_int))
                with ui.HStack(height=30, spacing=6):
                    self.play_button = ui.Button("Pause" if self.playing else "Play", clicked_fn=self.toggle_play)
                    ui.Button("Restart", clicked_fn=self.restart)
                with ui.HStack(height=28, spacing=6):
                    ui.Label("Speed", width=45)
                    speed = ui.ComboBox(1, "0.25 x", "0.5 x", "1.0 x")
                    speed.model.add_item_changed_fn(lambda model, _: self.set_speed(self.SPEEDS[model.get_item_value_model().as_int]))
                self.camera_combo = ui.ComboBox(0, *self.CAMERAS, height=28)
                self.camera_combo.model.add_item_changed_fn(lambda model, _: self.set_camera(model.get_item_value_model().as_int))
                ui.Label("Free orbit: Alt+left drag | pan: middle drag\nZoom: wheel | choose a preset to reframe", height=38, word_wrap=True)
                with ui.HStack(height=25):
                    for label, attr in (("Axes", "show_axes"), ("Normal", "show_normal"), ("Velocity", "show_velocity"), ("Contacts", "show_contacts")):
                        model = ui.SimpleBoolModel(True)
                        ui.CheckBox(model, width=22)
                        ui.Label(label)
                        model.add_value_changed_fn(lambda item, key=attr: setattr(self, key, item.as_bool))
                ui.Separator(height=6)
                self.identity = ui.Label("", height=58, word_wrap=True, style={"color": 0xFFB0E9D0})
                self.mode_label = ui.Label("", height=42, word_wrap=True)
                self.condition = ui.Label("", height=65, word_wrap=True)
                self.readout = ui.Label("", height=50, word_wrap=True)
                self.result = ui.Label("", height=85, word_wrap=True)
                ui.Label("Amber: exact underside fixture\nGreen: original registered contact colliders\nNo real-world force or safety equivalence claimed", height=52, word_wrap=True)
        self.window.visible = True
        # Match Kit's viewport docking lifecycle: first join the main dock,
        # then split after the UI has committed that layout on its next update.
        import asyncio
        self.window.deferred_dock_in("Viewport")
        async def dock_panel():
            import omni.kit.app
            kit_app = omni.kit.app.get_app()
            await kit_app.next_update_async()
            await kit_app.next_update_async()
            viewport_window = ui.Workspace.get_window("Viewport")
            if viewport_window is None:
                raise RuntimeError("Interactive viewport window was not created")
            self.window.dock_in(viewport_window, ui.DockPosition.RIGHT, .29)
        self.docking = asyncio.ensure_future(dock_panel())

    def toggle_play(self):
        if self.terminal and not self.playing:
            self.reset()
        self.playing = not self.playing
        self.play_button.text = "Pause" if self.playing else "Play"

    def restart(self):
        self.reset()
        self.playing = True
        self.play_button.text = "Pause"

    def select_demo(self, demo):
        if demo != self.demo:
            self.demo = demo
            self.reset()
            self.set_camera(self.camera)

    def select_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.reset()

    def set_speed(self, speed):
        self.speed = speed

    def reset(self):
        import torch
        self.clip = self.clips[self.demo]
        self.elapsed, self.accumulator, self.terminal = 0., 0., False
        self.physics_tick = self.clip.initial_tick
        self.policy_ticks[:] = self.physics_tick // 5
        self.outcome, self.contact_event, self.saturated = None, None, 0
        self.last_real = time.perf_counter()
        self.current = self.clip.sample(0.)
        if self.mode:
            state = self.clip.state[0]
            def tensor(x):
                return torch.as_tensor(np.asarray(x).copy(), dtype=torch.float32, device=self.sim.device).reshape(1, -1)
            self.sim.reset(self.ids, tensor(state[:3]), tensor(state[3:7]), tensor(state[7:10]), tensor(state[10:]),
                           tensor(self.clip.motor_state_N[0]), tensor(self.clip.initial_motor_command_N))
        self._pose(self.current)

    @staticmethod
    def rotation(quat):
        w, x, y, z = quat
        return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                         (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                         (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))

    def _pose(self, row):
        from pxr import Gf
        self.translation.Set(Gf.Vec3d(*(row["position"] + self.offset)))
        q = row["quaternion_wxyz"]
        self.orientation.Set(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:])))

    def set_camera(self, camera):
        from isaacsim.core.utils.viewports import set_camera_view
        self.camera = camera
        drone = self.current["position"] + self.offset
        target = self.center + self.offset
        if camera == 1:
            eye, look = drone + np.array([-.36, -.48, .25]), drone
        elif camera == 3:
            look = .5 * (self.current["bumper"] + self.center) + self.offset
            direction = np.array([-.22, -.36, .12])
            distance = .40 + 1.6*np.linalg.norm(self.current["bumper"] - self.center)
            eye = look + distance*direction/np.linalg.norm(direction)
        else:
            # Capture both the full stationary start and the sloped contact face.
            look = target + np.array([-.27, 0., .27])
            eye = look + np.array([-1.15, -1.75, .85])
        set_camera_view(eye=eye.tolist(), target=look.tolist(), camera_prim_path="/OmniverseKit_Persp")

    def step_isaac(self):
        import torch
        from isaaclab.utils.math import matrix_from_quat
        from .stage1_physx import classify_contacts, safety_reasons
        if self.terminal:
            return
        if self.demo == "witness":
            index = (self.physics_tick - self.clip.initial_tick) // 5
            if index >= len(self.clip.commands_ctbr):
                self.terminal, self.outcome = True, "WITNESS COMMANDS EXHAUSTED"
                return
            command = torch.as_tensor(self.clip.commands_ctbr[index], device=self.sim.device, dtype=torch.float32)[None]
        else:
            with torch.inference_mode():
                raw = self.actor(self.task.observe(self.sim.state(), self.policy_ticks)["policy"])
                if not raw.isfinite().all():
                    raise RuntimeError("Nonfinite viewer policy output")
                command = self.sim.actuator.decode_command(raw.tanh())
        self.sim.set_ctbr(command[:, 0], command[:, 1:])
        self.saturated += int(self.sim.actuator.mixer_saturated[0])
        self.policy_ticks.add_(1)
        for _ in range(5):
            tick = self.sim.step_physics()
            self.physics_tick += 1
            self.elapsed = (self.physics_tick - self.clip.initial_tick) * self.sim.physics_dt_s
            def pack(state):
                return dict(position_world_m=state.position[0].cpu().tolist(), velocity_world_m_s=state.velocity[0].cpu().tolist(),
                            rotation_body_to_world=matrix_from_quat(state.quaternion)[0].cpu().tolist(),
                            omega_body_rad_s=state.omega_body[0].cpu().tolist())
            before, after = pack(tick.before), pack(tick.after)
            reasons = safety_reasons(before, self.center, self.frame, self.frozen["safety"]) | safety_reasons(after, self.center, self.frame, self.frozen["safety"])
            contacts = [c for c in tick.contacts.get(0, []) if c["distance_m"] <= 0.]
            if contacts:
                reasons |= classify_contacts(contacts, before, self.center, self.frame, self.halfsize, self.frozen["contact_acceptance"])
                if self.physics_tick == self.clip.initial_tick + 1:
                    reasons.add("INITIAL_OVERLAP")
                self.contact_event = dict(time_s=self.physics_tick * .001, contacts=contacts)
                self.outcome = "PHYSX QUALIFIED (PROXY)" if not reasons else "PHYSX REJECTED: " + ", ".join(sorted(reasons))
            elif reasons:
                self.outcome = "ISAAC FAILURE: " + ", ".join(sorted(reasons))
            elif self.physics_tick >= round(self.frozen["horizon_s"] / .001):
                self.outcome = "ISAAC TIMEOUT"
            if self.outcome:
                self.terminal = True
                break
        state = self.sim.state()
        pos, quat, vel, omega = [field[0].cpu().numpy().astype(float) for field in state]
        self.current = dict(position=pos, quaternion_wxyz=quat, linear_velocity=vel, angular_velocity_body=omega,
                            bumper=pos + self.rotation(quat) @ self.fixture_offset,
                            time_s=self.physics_tick * .001, elapsed_s=self.elapsed, done=self.terminal,
                            contact_visible=self.contact_event is not None)

    def overlays(self):
        row = self.current
        pos = row["position"] + self.offset
        rotation = self.rotation(row["quaternion_wxyz"])
        start, end, colors = [], [], []
        def line(a, b, color):
            start.append(tuple(a)); end.append(tuple(b)); colors.append(color)
        if self.show_axes:
            for j, color in enumerate(((1., .1, .1, 1.), (.1, 1., .1, 1.), (.2, .45, 1., 1.))):
                line(pos, pos + .13 * rotation[:, j], color)
        if self.show_normal:
            p = self.center + self.offset
            line(p, p + .24*self.normal, (1., .8, .08, 1.))
        if self.show_velocity:
            line(pos, pos + .14*row["linear_velocity"], (.1, .9, 1., 1.))
        for i, curve in enumerate(self.lines):
            curve.GetPointsAttr().Set([start[i], end[i]] if i < len(start) else [])
            curve.GetCurveVertexCountsAttr().Set([2] if i < len(start) else [])
            if i < len(start):
                curve.CreateDisplayColorAttr().Set([colors[i][:3]])
                curve.GetDisplayColorPrimvar().SetInterpolation("constant")
        self.points.GetPointsAttr().Set([])
        self.points.MakeInvisible()
        contact = self.clip.first_contact if not self.mode else self.contact_event
        if self.show_contacts and row["contact_visible"] and contact:
            points = [tuple(np.asarray(c["position_world_m"]) + self.offset) for c in contact["contacts"]]
            if points:
                self.points.GetPointsAttr().Set(points)
                self.points.MakeVisible()

    def status_update(self):
        native = "MUJOCO QUALIFIED" if self.clip.qualified else "MUJOCO REJECTED: " + ", ".join(self.clip.failure_modes)
        if self.mode:
            mode = "ISAAC CLOSED-LOOP PPO - NEW SIMULATION" if self.demo != "witness" else "ISAAC DETERMINISTIC COMMAND REPLAY - NOT PPO"
            result = self.outcome or "Isaac: running; original native result is reference only"
        else:
            mode = "RECORDED MUJOCO - STATE PLAYBACK, NO INTEGRATION"
            result = "Recorded contact" if self.current["contact_visible"] else "Recorded pre-contact"
        self.identity.text = self.clip.label + "\n" + self.clip.controller
        self.mode_label.text = mode
        self.condition.text = self.clip.initial_description
        gap = float((self.current["bumper"] - self.center) @ self.normal - self.fixture_radius)
        self.readout.text = f"Original clock {self.current['time_s']:.3f} s | replay +{self.elapsed:.3f} s\nBumper normal gap {gap:.3f} m | speed {np.linalg.norm(self.current['linear_velocity']):.2f} m/s"
        self.result.text = native + "\n" + result + f"\nMixer saturated controls: {self.saturated}" + ("\nPaused at episode end; Restart replays." if self.terminal else "")

    def update(self):
        now = time.perf_counter()
        elapsed_real = min(now-self.last_real, .10)
        self.last_real = now
        if self.playing and not self.terminal:
            if self.mode:
                self.accumulator += elapsed_real*self.speed
                while self.accumulator >= .005 and not self.terminal:
                    self.step_isaac()
                    self.accumulator -= .005
            else:
                self.elapsed = min(self.elapsed + elapsed_real*self.speed, self.clip.duration_s)
                self.current = self.clip.sample(self.elapsed)
                self.terminal = self.current["done"]
            if self.terminal:
                self.playing = False
                self.play_button.text = "Play"
        self._pose(self.current)
        if self.camera == 1:
            self.set_camera(1)
        self.overlays()
        self.status_update()
        self.sim.sim.render()  # Explicit rendering disables app-driven physics.

    def self_test(self, directory):
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "viewer-check.json").unlink(missing_ok=True)
        for demo in self.DEMOS:
            for mode in (0, 1):
                self.demo, self.mode = demo, mode
                self.demo_combo.model.get_item_value_model().set_value(self.DEMOS.index(demo))
                self.mode_combo.model.get_item_value_model().set_value(mode)
                self.reset()
                if mode:
                    actual = np.concatenate([part[0].cpu().numpy() for part in self.sim.state()])
                    np.testing.assert_allclose(actual, self.clip.state[0], rtol=0., atol=2e-6)
                    while not self.terminal:
                        self.step_isaac()
                    expected = ("PHYSX QUALIFIED (PROXY)" if self.clip.qualified
                                else "ISAAC FAILURE: FLIGHT_VOLUME_EXIT")
                    if self.outcome != expected:
                        raise RuntimeError(f"{demo} live reproduction failed: {self.outcome}; expected {expected}")
                else:
                    self.elapsed = self.clip.duration_s
                    self.current = self.clip.sample(self.elapsed)
                    self.terminal = True
                    np.testing.assert_array_equal(self.current["position"], self.clip.position[-1])
                self._pose(self.current)
                self.set_camera(3 if demo == "contact" else 2)
                self.overlays()
                self.status_update()
                for _ in range(12):
                    self.sim.sim.render()
                self.docking.result()
                if not self.window.docked:
                    raise RuntimeError("Replay controls did not dock beside the viewport")
                shot = directory / f"{demo}-{'isaac' if mode else 'recorded'}.png"
                shot.unlink(missing_ok=True)
                import asyncio
                capture = capture_viewport_to_file(get_active_viewport(), str(shot))
                captured = asyncio.ensure_future(capture.wait_for_result())
                deadline = time.perf_counter() + 15
                # Kit's future acknowledges scheduling, before its file writer
                # necessarily finishes. Require a complete newly written PNG.
                while time.perf_counter() < deadline:
                    self.sim.sim.render()
                    if captured.done():
                        captured.result()
                        if shot.is_file() and shot.stat().st_size > 1000:
                            with shot.open("rb") as saved:
                                saved.seek(-12, 2)
                                if saved.read() == b"\x00\x00\x00\x00IEND\xaeB`\x82":
                                    break
                    time.sleep(.005)
                else:
                    raise RuntimeError(f"Viewport capture timed out: {shot.name}")
                self.checks.append(dict(demo=demo, mode="Isaac simulation" if mode else "Recorded MuJoCo",
                                        original_native_qualified=self.clip.qualified, current_outcome=self.outcome,
                                        physics_end_tick=self.physics_tick if mode else round(self.current["time_s"]/.001),
                                        mixer_saturated_controls=self.saturated, screenshot=shot.name,
                                        final_position=self.current["position"].tolist(),
                                        checkpoint_sha256=self.manifest["files"]["checkpoint"]["sha256"]))
        for camera in range(len(self.CAMERAS)):
            self.set_camera(camera)
            self.sim.sim.render()
        self.playing = False
        self.set_speed(.25); self.set_speed(.5); self.set_speed(1.)
        (directory / "viewer-check.json").write_text(json.dumps(dict(status="PASS", checks=self.checks,
            scope="Interactive render and inference smoke; native qualifications are archived reference, not a new Isaac success claim"), indent=2)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--livestream", type=int, choices=(0, 2), default=2)
    parser.add_argument("--demo", choices=Viewer.DEMOS, default="contact")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--self-test", type=Path)
    parser.add_argument("--verify-and-serve", action="store_true")
    args = parser.parse_args()
    import sys
    sys.argv = [sys.argv[0], "--/app/livestream/fixedHostPort=47998",
                "--/app/livestream/nvcf/quitOnSessionEnded=false",
                "--/app/livestream/nvcf/sessionResumeTimeoutSeconds=0"]
    from isaaclab.app import AppLauncher
    launcher = AppLauncher(headless=True, livestream=args.livestream, enable_cameras=False,
                           video=True, width=1280, height=800,
                           experience=str(ROOT / "configs/rm27_viewer.kit"))
    app = launcher.app
    stopped = [False]
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stopped.__setitem__(0, True))
    try:
        viewer = Viewer(app, args)
        if args.self_test or args.verify_and_serve:
            viewer.self_test(args.self_test or ROOT / ".agents/runs/rm27-viewer/verification")
            viewer.demo, viewer.mode = args.demo, 0
            viewer.demo_combo.model.get_item_value_model().set_value(viewer.DEMOS.index(args.demo))
            viewer.mode_combo.model.get_item_value_model().set_value(0)
            viewer.reset()
            viewer.set_camera(3 if args.demo == "contact" else 2)
            viewer.camera = 0
        if not args.self_test or args.verify_and_serve:
            print("RM27_VIEWER_READY | Recorded MuJoCo default | private WebRTC | no learning", flush=True)
            while app.is_running() and not stopped[0]:
                viewer.update()
                time.sleep(.005)
        viewer.sim.close()
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
