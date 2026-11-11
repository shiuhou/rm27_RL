"""Play recorded Isaac free-flight states with the original ASM2 visual meshes."""
from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys
import time
import traceback

import numpy as np

from .asset import CONFIG, ROOT, digest, read_json


def load_recording(path):
    with np.load(path, allow_pickle=False) as recording:
        states = np.asarray(recording["states"], dtype=float)
        reference = np.asarray(recording["reference"], dtype=float)
        names = recording["names"]
        dt = recording["dt"]
        times = recording["sample_times_s"] if "sample_times_s" in recording else None
        contact = None
        if "target_vertices_m" in recording:
            contact = (recording["target_vertices_m"], recording["target_faces"],
                       recording["contact_point_m"], str(recording["contact_outcome"].item()))
    if states.ndim != 3 or states.shape[2] != 13 or states.shape[0] < 2 or states.shape[1] < 1:
        raise ValueError("Expected states with shape (frames >= 2, episodes >= 1, 13)")
    if reference.shape != states.shape[:2] + (3,):
        raise ValueError("Reference positions must match the recording's frames and episodes")
    if names.shape != (states.shape[1],) or names.dtype.kind not in "US":
        raise ValueError("Expected one string name per episode")
    names = tuple(str(name) for name in names.astype(str))
    if any(not name.strip() for name in names) or len(set(names)) != len(names):
        raise ValueError("Episode names must be nonempty and distinct")
    if dt.ndim != 0 or not np.isfinite(dt) or float(dt) <= 0:
        raise ValueError("Expected a finite positive scalar policy dt in seconds")
    if not np.isfinite(states).all() or not np.isfinite(reference).all():
        raise ValueError("Recorded states and references must be finite")
    if not np.allclose(np.linalg.norm(states[:, :, 3:7], axis=-1), 1., atol=1e-4, rtol=0.):
        raise ValueError("Recorded orientations must be unit quaternions in wxyz order")
    if times is None:
        times = np.arange(len(states))*float(dt)
    if (times.shape != (len(states),) or not np.isfinite(times).all()
            or times[0] != 0 or np.any(np.diff(times) <= 0)):
        raise ValueError("Replay timestamps must increase from zero")
    if contact is not None:
        vertices, faces, point, _ = contact
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all()
                or faces.ndim != 2 or faces.shape[1] != 3 or faces.dtype.kind not in "iu"
                or np.any(faces < 0) or np.any(faces >= len(vertices))
                or point.shape not in ((0,), (3,)) or not np.isfinite(point).all()):
            raise ValueError("Invalid recorded target triangles or contact point")
    return states, reference, names, times, contact


class FlightReplay:
    CAMERAS = ("Free orbit", "Follow aircraft", "Overview")
    SPEEDS = (.25, .5, 1.)

    def __init__(self, recording, episode, paused):
        import carb
        import omni.ui as ui
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdLux

        self.states, self.reference, self.names, self.times, self.contact = recording
        self.duration = float(self.times[-1])
        self.speed, self.camera = 1., 0
        self.playing = not paused
        context = omni.usd.get_context()
        context.new_stage()
        self.stage = context.get_stage()
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        world = UsdGeom.Xform.Define(self.stage, "/World")
        self.stage.SetDefaultPrim(world.GetPrim())

        self.camera_view = UsdGeom.Camera.Define(self.stage, "/World/ReplayCamera")
        self.camera_view.CreateClippingRangeAttr(Gf.Vec2f(.01, 100.))
        self.camera_view.CreateFocalLengthAttr(24.)
        self.camera_transform = self.camera_view.AddTransformOp()
        self.camera_interest = self.camera_view.GetPrim().CreateAttribute(
            "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Vector3d, custom=True)
        self.viewport = None

        self.aircraft = UsdGeom.Xform.Define(self.stage, "/World/ASM2")
        self.visual_count = self._reference_visuals(self.aircraft.GetPath())
        self.translation = self.aircraft.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self.orientation = self.aircraft.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        self.ground = UsdGeom.Cube.Define(self.stage, "/World/Ground")
        self.ground.CreateSizeAttr(2.)
        self.ground.CreateDisplayColorAttr([(.17, .20, .25)])
        self.ground_position = self.ground.AddTranslateOp()
        self.ground_scale = self.ground.AddScaleOp()
        self.path = UsdGeom.BasisCurves.Define(self.stage, "/World/RecordedPath")
        self.guide = UsdGeom.BasisCurves.Define(self.stage, "/World/ReferencePath")
        for curve, width, color in ((self.path, .005, (1., .55, .10)),
                                    (self.guide, .003, (.09, .8, 1.))):
            curve.CreateTypeAttr("linear")
            curve.CreateWrapAttr("nonperiodic")
            curve.CreateWidthsAttr([width])
            curve.SetWidthsInterpolation("constant")
            curve.CreateDisplayColorAttr([color])
        self.target = UsdGeom.Sphere.Define(self.stage, "/World/ReferencePosition")
        self.target.CreateRadiusAttr(.014)
        self.target.CreateDisplayColorAttr([(.09, .8, 1.)])
        self.target_position = self.target.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        if self.contact is not None:
            vertices, faces, point, _ = self.contact
            surface = UsdGeom.Mesh.Define(self.stage, "/World/RegisteredTarget")
            surface.CreatePointsAttr(vertices.tolist())
            surface.CreateFaceVertexCountsAttr([3]*len(faces))
            surface.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
            surface.CreateSubdivisionSchemeAttr("none")
            surface.CreateDoubleSidedAttr(True)
            surface.CreateDisplayColorAttr([(.15, .38, .63)])
            if point.size:
                marker = UsdGeom.Sphere.Define(self.stage, "/World/RecordedContactPoint")
                marker.CreateRadiusAttr(.003)
                marker.CreateDisplayColorAttr([(.94, .25, .09)])
                marker.AddTranslateOp().Set(Gf.Vec3d(*point))
        UsdLux.DomeLight.Define(self.stage, "/World/Dome").CreateIntensityAttr(650.)
        light = UsdLux.DistantLight.Define(self.stage, "/World/Key")
        light.CreateIntensityAttr(2000.)
        light.AddRotateXYZOp().Set(Gf.Vec3f(-35., -25., 20.))
        self._build_panel(ui, episode)
        carb.settings.get_settings().set_bool("/app/window/hideUi", False)
        self.select(episode)

    def _reference_visuals(self, destination):
        from pxr import Gf, Usd, UsdGeom

        config = read_json(CONFIG)
        path = ROOT / config["asset_root"] / "usd/drone.usdc"
        if digest(path) != config["pins"]["usd/drone.usdc"]:
            raise ValueError("Pinned ASM2 visual asset changed")
        source = Usd.Stage.Open(str(path), load=Usd.Stage.LoadNone)
        drone = source.GetDefaultPrim()
        if (str(drone.GetPath()) != "/Drone" or UsdGeom.GetStageUpAxis(source) != UsdGeom.Tokens.z
                or UsdGeom.GetStageMetersPerUnit(source) != 1.
                or UsdGeom.Xformable(drone).GetLocalTransformation() != Gf.Matrix4d(1.)):
            raise ValueError("ASM2 visuals require the original metre-scale, Z-up, identity body frame")
        visuals = [prim for prim in drone.GetChildren() if prim.IsA(UsdGeom.Mesh)]
        if len(visuals) != 14 or any(prim.GetAppliedSchemas() for prim in visuals):
            raise ValueError("Expected the 14 original ASM2 visual-only meshes")
        # Reference each render mesh. Referencing the complete /Drone root
        # would also compose its rigid body and 11,874 collision pieces.
        for visual in visuals:
            prim = self.stage.OverridePrim(destination.AppendChild(visual.GetName()))
            prim.GetReferences().AddReference(str(path), visual.GetPath())
        return len(visuals)

    def _build_panel(self, ui, episode):
        import asyncio
        import omni.kit.app

        self.window = ui.Window("RM27 | ASM2 Flight", width=340, height=390)
        with self.window.frame:
            with ui.VStack(spacing=7, style={"font_size": 14}):
                ui.Label("ASM2 | FREE-FLIGHT REPLAY", height=26,
                         style={"font_size": 18, "color": 0xFFB0E9D0})
                source = "Recorded MuJoCo contact / frozen target" if self.contact else "Recorded Isaac states / 142 g CAD candidate"
                ui.Label(source, height=24, word_wrap=True)
                self.episode_menu = ui.ComboBox(episode, *self.names, height=28)
                self.episode_menu.model.add_item_changed_fn(
                    lambda model, _: self.select(model.get_item_value_model().as_int))
                with ui.HStack(height=28, spacing=6):
                    self.play_button = ui.Button("Pause" if self.playing else "Play", clicked_fn=self.toggle)
                    ui.Button("Restart", clicked_fn=self.restart)
                with ui.HStack(height=28, spacing=6):
                    ui.Label("Speed", width=55)
                    self.speed_menu = ui.ComboBox(2, "0.25x", "0.5x", "1x")
                    self.speed_menu.model.add_item_changed_fn(
                        lambda model, _: setattr(self, "speed", self.SPEEDS[model.get_item_value_model().as_int]))
                self.camera_menu = ui.ComboBox(0, *self.CAMERAS, height=28)
                self.camera_menu.model.add_item_changed_fn(
                    lambda model, _: self.set_camera(model.get_item_value_model().as_int))
                ui.Button("Reframe camera", height=26, clicked_fn=lambda: self.set_camera(self.camera))
                self.status = ui.Label("", height=54, word_wrap=True)
                ui.Label("Orange: recorded flight   Cyan: reference\nAlt+drag to orbit / wheel to zoom",
                         height=42, word_wrap=True)

        self.window.deferred_dock_in("Viewport")

        async def dock_panel():
            app = omni.kit.app.get_app()
            await app.next_update_async()
            await app.next_update_async()
            viewport = ui.Workspace.get_window("Viewport")
            if viewport is None:
                raise RuntimeError("Kit did not create the interactive viewport")
            self.window.dock_in(viewport, ui.DockPosition.RIGHT, .29)

        self.docking = asyncio.ensure_future(dock_panel())

    @staticmethod
    def _set_curve(curve, points):
        changed = np.r_[True, np.any(np.diff(points, axis=0) != 0, axis=1)]
        vertices = points[changed]
        curve.GetPointsAttr().Set([tuple(row) for row in vertices] if len(vertices) > 1 else [])
        curve.GetCurveVertexCountsAttr().Set([len(vertices)] if len(vertices) > 1 else [])

    def select(self, episode):
        from pxr import Gf

        self.episode, self.elapsed, self.frame = episode, 0., -1
        self.last_update = time.perf_counter()
        positions = self.states[:, episode, :3]
        reference = self.reference[:, episode]
        self._set_curve(self.path, positions)
        self._set_curve(self.guide, reference)
        track = np.concatenate((positions, reference))
        lo, hi = track.min(axis=0), track.max(axis=0)
        self.center = (lo + hi) * .5
        self.span = max(float(np.linalg.norm(hi - lo)), .35)
        self.ground_position.Set(Gf.Vec3d(self.center[0], self.center[1], min(0., lo[2] - .15) - .01))
        self.ground_scale.Set(Gf.Vec3f(max(self.span + .35, .8), max(self.span + .35, .8), .01))
        self.set_frame(0)
        self.set_camera(self.camera)

    def set_frame(self, index):
        from pxr import Gf

        if index == self.frame:
            return
        self.frame = index
        state = self.states[index, self.episode]
        self.translation.Set(Gf.Vec3d(*state[:3]))
        self.orientation.Set(Gf.Quatd(float(state[3]), Gf.Vec3d(*state[4:7])))
        reference = self.reference[index, self.episode]
        self.target_position.Set(Gf.Vec3d(*reference))
        error = np.linalg.norm(state[:3] - reference)
        speed = np.linalg.norm(state[7:10])
        self.status.text = (f"{self.times[index]:.2f} / {self.duration:.2f} s\n"
                            f"Position error {error * 100:.1f} cm / speed {speed:.2f} m/s")
        if self.contact is not None and index == len(self.states)-1:
            self.status.text += f"\n{self.contact[3]}"
        if self.camera == 1:
            self.set_camera(1)

    def set_camera(self, camera):
        from pxr import Gf

        self.camera = camera
        if camera == 1:
            target = self.states[self.frame, self.episode, :3]
            offset = np.array([-.29, -.39, .19])
        else:
            target = self.center
            offset = self.span * np.array([-.7, -1.3, 1.5 if camera == 2 else .65])
        eye = Gf.Vec3d(*(target + offset))
        view = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(*target), Gf.Vec3d(0., 0., 1.))
        self.camera_transform.Set(view.GetInverse())
        self.camera_interest.Set(Gf.Vec3d(0., 0., -float(np.linalg.norm(offset))))
        if self.viewport is not None:
            self.viewport.camera_path = self.camera_view.GetPath()

    def activate_camera(self):
        from omni.kit.viewport.utility import get_active_viewport

        self.viewport = get_active_viewport()
        if self.viewport is None or self.viewport.stage != self.stage:
            raise RuntimeError("Kit did not open the recorded flight stage in its viewport")
        self.viewport.camera_path = self.camera_view.GetPath()

    def toggle(self):
        if not self.playing and self.elapsed >= self.duration:
            self.restart()
            return
        self.playing = not self.playing
        self.play_button.text = "Pause" if self.playing else "Play"

    def restart(self):
        self.elapsed, self.playing = 0., True
        self.last_update = time.perf_counter()
        self.set_frame(0)
        self.play_button.text = "Pause"

    def update(self):
        now = time.perf_counter()
        elapsed_wall = now - self.last_update
        self.last_update = now
        if self.playing:
            self.elapsed = min(self.elapsed + elapsed_wall * self.speed, self.duration)
            frame = int(np.searchsorted(self.times, self.elapsed, side="right")-1)
            self.set_frame(frame)
            if self.elapsed >= self.duration:
                self.toggle()

    def self_test(self, app, screenshot):
        """Verify real stage composition, recorded poses and viewport capture."""
        import asyncio
        from omni.kit.viewport.utility import capture_viewport_to_file
        from pxr import UsdGeom, UsdPhysics

        meshes = [prim for prim in self.aircraft.GetPrim().GetChildren() if prim.IsA(UsdGeom.Mesh)]
        if len(meshes) != self.visual_count or any(not UsdGeom.Mesh(p).GetPointsAttr().Get() for p in meshes):
            raise RuntimeError("The original ASM2 visual meshes did not compose")
        if any(prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI)
               for prim in self.stage.Traverse()):
            raise RuntimeError("A physics schema was composed into the recorded playback stage")
        selected = self.episode
        for episode in range(len(self.names)):
            self.episode_menu.model.get_item_value_model().set_value(episode)
            for index in (0, len(self.states) - 1):
                self.set_frame(index)
                expected = self.states[index, episode]
                np.testing.assert_array_equal(np.asarray(self.translation.Get()), expected[:3])
                q = self.orientation.Get()
                np.testing.assert_array_equal([q.GetReal(), *q.GetImaginary()], expected[3:7])
        self.episode_menu.model.get_item_value_model().set_value(selected)
        self.restart()
        self.toggle()
        if self.playing or self.frame != 0:
            raise RuntimeError("Replay restart/pause did not return to the first frame")
        capture_frame = (len(self.states)-1 if self.contact else int(np.argmax(
            np.linalg.norm(self.states[:, selected, :3]-self.states[0, selected, :3], axis=1))))
        self.set_frame(capture_frame)
        self.set_camera(1)
        for _ in range(12):
            app.update()
        self.docking.result()
        if not self.window.docked:
            raise RuntimeError("Replay controls did not dock beside the viewport")
        if self.viewport.camera_path != self.camera_view.GetPath():
            raise RuntimeError("Replay viewport is using a different camera")
        screenshot.parent.mkdir(parents=True, exist_ok=True)
        screenshot.unlink(missing_ok=True)
        capture = capture_viewport_to_file(self.viewport, str(screenshot))
        saved = asyncio.ensure_future(capture.wait_for_result())
        deadline = time.monotonic() + 15.
        while time.monotonic() < deadline:
            app.update()
            if saved.done():
                saved.result()
                if screenshot.is_file() and screenshot.stat().st_size > 1000:
                    with screenshot.open("rb") as stream:
                        stream.seek(-12, 2)
                        if stream.read() == b"\x00\x00\x00\x00IEND\xaeB`\x82":
                            return
            time.sleep(.005)
        raise RuntimeError("ASM2 viewport screenshot timed out")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--episode", help="Initial episode name (default: first recorded episode)")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--self-test", type=Path, metavar="SCREENSHOT")
    args = parser.parse_args()
    recording = load_recording(args.replay)
    names = recording[2]
    if args.episode is not None and args.episode not in names:
        parser.error(f"Unknown episode {args.episode!r}. Choose from: {', '.join(names)}")
    episode = names.index(args.episode) if args.episode is not None else 0
    sys.argv = [sys.argv[0], "--/app/window/title=RM27 ASM2 | Recorded Simulation",
                "--/app/livestream/fixedHostPort=47998",
                "--/app/livestream/nvcf/sessionResumeTimeoutSeconds=0"]
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True, livestream=0 if args.self_test else 2, enable_cameras=False,
                      video=True, width=1280, height=800,
                      experience=str(ROOT / "configs/rm27_viewer.kit")).app
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop)
    try:
        viewer = FlightReplay(recording, episode, args.paused)
        app.update()
        viewer.activate_camera()
        if args.self_test:
            viewer.self_test(app, args.self_test)
            print(f"ASM2_VIEWER_SELF_TEST_PASS episodes={len(names)} meshes={viewer.visual_count} screenshot={args.self_test}", flush=True)
        else:
            for _ in range(12):
                app.update()
            viewer.docking.result()
            viewer.last_update = time.perf_counter()
            print(f"ASM2_VIEWER_READY episodes={len(names)} meshes={viewer.visual_count} replay={args.replay.name}", flush=True)
            while app.is_running() and not stopped:
                viewer.update()
                app.update()
                time.sleep(.005)
    except Exception:
        # Kit fast shutdown can exit successfully before Python reports a
        # failure. Let the process fail directly when replay setup fails.
        traceback.print_exc(file=sys.__stderr__)
        raise
    app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
