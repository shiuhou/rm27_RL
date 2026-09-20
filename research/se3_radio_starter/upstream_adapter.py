"""Adapter around LGQWakkk's controller/mixer, imported from a local checkout.

No original controller/model is bundled here. Do not import upstream main.py:
it launches its own viewer. This adapter is strictly for the original demo model.
"""
from __future__ import annotations

import hashlib
import importlib
import json
from pathlib import Path
import sys

import numpy as np

from core import quat_wxyz_to_xyzw, vector

EXPECTED_FILES = ("se3_controller.py", "motor_mixer.py", "geometry.py", "crazyfile/scene.xml")


def source_fingerprints(repo: Path) -> dict[str, str]:
    return {name: hashlib.sha256((repo / name).read_bytes()).hexdigest()
            for name in EXPECTED_FILES}


class UpstreamSimulation:
    # These are the demo constants in the author's main.py, not parameters of a user's UAV.
    MASS = 0.033
    GRAVITY = 9.8066
    CT = 3.25e-4
    MAX_THRUST = 0.1573
    MAX_SPEED = 22.0
    TORQUE_SCALE = 0.001
    DT = 0.001

    def __init__(self, repo: Path):
        self.repo = Path(repo).resolve()
        missing = [name for name in EXPECTED_FILES if not (self.repo / name).is_file()]
        if missing:
            raise FileNotFoundError(f"Incomplete upstream folder {self.repo}: {missing}. "
                                    "Run python fetch_upstream.py, or pass --repo <checkout>.")
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("MuJoCo is missing. Run: python -m pip install -r requirements.txt") from exc
        self.mj = mujoco
        sys.path.insert(0, str(self.repo))
        loaded = {}
        for name in ("geometry", "se3_controller", "motor_mixer"):
            module = importlib.import_module(name)
            if Path(module.__file__).resolve().parent != self.repo:
                raise RuntimeError(f"Module name collision for {name}. Start a clean Python process.")
            loaded[name] = module
        self.State = loaded["se3_controller"].State
        self.controller = loaded["se3_controller"].SE3Controller()
        self.controller.kx = 0.6
        self.controller.kv = 0.4
        self.controller.kR = 6.0
        self.controller.kw = 1.0
        self.mixer = loaded["motor_mixer"].Mixer()
        mujoco.set_mjcb_control(None)
        self.model = mujoco.MjModel.from_xml_path(str(self.repo / "crazyfile" / "scene.xml"))
        self.model.opt.timestep = self.DT
        self.data = mujoco.MjData(self.model)
        free = np.flatnonzero(self.model.jnt_type == int(mujoco.mjtJoint.mjJNT_FREE))
        if len(free) != 1:
            raise ValueError("Expected exactly one free joint in the original quadrotor demo.")
        joint = int(free[0])
        self.qadr = int(self.model.jnt_qposadr[joint])
        self.vadr = int(self.model.jnt_dofadr[joint])
        self.body_id = int(self.model.jnt_bodyid[joint])
        self.motor_ids = np.array([self.model.actuator(f"motor{i}").id for i in range(1, 5)])
        for index in self.motor_ids:
            low, high = self.model.actuator_ctrlrange[index]
            if self.model.actuator_ctrllimited[index] and (low > 0 or high < 1):
                raise ValueError("Unexpected actuator range; use the original normalized-input model.")
        self.last_control = np.zeros(4)
        self.reset()
        print(f"Original-model adapter: controller mass={self.MASS:.3f} kg, dt={self.DT}s")
        print(f"Model subtree mass={self.model.body_subtreemass[self.body_id]:.6f} kg")
        print("Uses simulator ground truth, no state estimator or hardware flight controller.")

    @property
    def position(self) -> np.ndarray:
        return self.data.qpos[self.qadr:self.qadr + 3].copy()

    def reset(self) -> None:
        self.mj.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0
        self.last_control[:] = 0
        self.mj.mj_forward(self.model, self.data)

    def step(self, target: np.ndarray, heading: np.ndarray) -> None:
        target = vector(target, 3, "target")
        heading = vector(heading, 3, "heading")
        q = self.data.qpos[self.qadr:self.qadr + 7]
        v = self.data.qvel[self.vadr:self.vadr + 6]
        if not np.isfinite(q).all() or not np.isfinite(v).all():
            raise RuntimeError("Nonfinite physics state; stopping simulation.")
        # A free joint's linear velocity is world-frame; angular velocity is body-local.
        current = self.State(q[:3].copy(), v[:3].copy(),
                             quat_wxyz_to_xyzw(q[3:7]), v[3:6].copy())
        # Keep desired velocities ZERO, like the original point-tracking main.py.
        # The reference rate is used only to integrate target position. It is NOT fed
        # into upstream goal_vel: that implementation adds a further goal.velocity term.
        desired = self.State(target.copy(), np.zeros(3), np.array([0., 0., 0., 1.]), np.zeros(3))
        command = self.controller.control_update(current, desired, self.DT, heading.copy())
        torque = np.asarray(command.angular, dtype=float) * self.TORQUE_SCALE
        thrust = float(command.thrust) * self.MASS * self.GRAVITY
        if torque.shape != (3,) or not np.isfinite(torque).all() or not np.isfinite(thrust):
            raise RuntimeError("Controller generated a nonfinite/invalid output.")
        # Protect the original mixer's collective-saturation edge case. Normal hover
        # lies far below this cap. This is an adapter guard, not an upstream code edit.
        thrust = float(np.clip(thrust, 0.0, 0.99 * 4 * self.CT * self.MAX_SPEED ** 2))
        with np.errstate(invalid="raise", divide="raise"):
            try:
                speed = np.asarray(self.mixer.calculate(thrust, *torque), dtype=float)
            except FloatingPointError as exc:
                raise RuntimeError("Original mixer hit a numeric/saturation edge case. Press restart; "
                                   "reduce motion limits rather than masking the error.") from exc
        if speed.shape != (4,) or not np.isfinite(speed).all():
            raise RuntimeError("Original mixer returned nonfinite motor speeds.")
        self.last_control = np.clip(self.CT * np.clip(speed, 0, self.MAX_SPEED) ** 2 / self.MAX_THRUST, 0, 1)
        self.data.ctrl[self.motor_ids] = self.last_control
        self.mj.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("Physics became nonfinite after stepping.")

    def add_target_marker(self, viewer, target: np.ndarray, heading: np.ndarray) -> None:
        """Call with viewer.lock held. Visual-only geometry: no mass or collision."""
        if viewer.user_scn.maxgeom < 2:
            return
        viewer.user_scn.ngeom = 2
        self.mj.mjv_initGeom(viewer.user_scn.geoms[0], self.mj.mjtGeom.mjGEOM_SPHERE,
                            np.array([0.025, 0, 0]), target,
                            np.eye(3).ravel(), np.array([1., 0.25, 0.15, 0.8]))
        self.mj.mjv_initGeom(viewer.user_scn.geoms[1], self.mj.mjtGeom.mjGEOM_SPHERE,
                            np.array([0.013, 0, 0]), target + 0.12 * heading,
                            np.eye(3).ravel(), np.array([1., 0.85, 0.15, 0.8]))
