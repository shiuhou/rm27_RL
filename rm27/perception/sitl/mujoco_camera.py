"""Headless MuJoCo camera input for the PX4 visual SITL path.

The camera is a rendering source only.  It does not expose MuJoCo truth to
the mission: callers receive an RGB frame and its timestamp, then pass that
frame through an image detector.  The camera pose follows a selected vehicle
body with a configurable body-frame offset and forward axis.  This is a
simulation camera model; it is not an OS04A10 calibration.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import mujoco
import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    """One rendered RGB frame and the source time assigned by the simulator."""

    image_rgb: np.ndarray
    timestamp_us: int
    width: int
    height: int

    def __post_init__(self):
        image = np.asarray(self.image_rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("image_rgb must be an HxWx3 uint8 array")
        if image.shape[1] != self.width or image.shape[0] != self.height:
            raise ValueError("frame dimensions do not match image_rgb")
        if type(self.timestamp_us) is not int or self.timestamp_us < 0:
            raise ValueError("timestamp_us must be a non-negative integer")


def _unit(vector: Iterable[float], name: str) -> np.ndarray:
    value = np.asarray(tuple(vector), dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain three finite values")
    norm = np.linalg.norm(value)
    if norm < 1e-9:
        raise ValueError(f"{name} must be non-zero")
    return value / norm


class MujocoCamera:
    """Render a camera that follows a MuJoCo body.

    ``forward_body`` is the optical axis in the body frame and ``up_body``
    defines the camera's vertical direction. The GL camera follows the full
    body rotation, including roll. A named model camera can be selected with
    ``camera_name`` when a scene provides one.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        *,
        width: int = 320,
        height: int = 240,
        fovy_deg: float = 60.0,
        body_name: str = "drone_1",
        camera_name: str | None = None,
        offset_body: Iterable[float] = (0.0, 0.0, 0.02),
        forward_body: Iterable[float] = (1.0, 0.0, 0.0),
        up_body: Iterable[float] = (0.0, 0.0, 1.0),
        look_ahead_m: float = 2.0,
    ):
        if int(width) <= 0 or int(height) <= 0:
            raise ValueError("camera width and height must be positive")
        if not math.isfinite(float(fovy_deg)) or not 0.0 < float(fovy_deg) < 180.0:
            raise ValueError("fovy_deg must be between 0 and 180")
        if not math.isfinite(float(look_ahead_m)) or float(look_ahead_m) <= 0:
            raise ValueError("look_ahead_m must be positive")
        self.model = model
        self.width = int(width)
        self.height = int(height)
        self.fovy_deg = float(fovy_deg)
        self.body_id = -1 if body_name is None else mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        if camera_name is not None:
            self.camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if self.camera_id < 0:
                raise ValueError(f"unknown MuJoCo camera: {camera_name}")
        else:
            self.camera_id = -1
        if self.camera_id < 0 and self.body_id < 0:
            raise ValueError(f"unknown camera body: {body_name}")
        self.offset_body = np.asarray(tuple(offset_body), dtype=float)
        if self.offset_body.shape != (3,) or not np.isfinite(self.offset_body).all():
            raise ValueError("offset_body must contain three finite values")
        self.forward_body = _unit(forward_body, "forward_body")
        self.up_body = _unit(up_body, "up_body")
        if abs(float(np.dot(self.forward_body, self.up_body))) > 1e-6:
            raise ValueError("forward_body and up_body must be perpendicular")
        self.look_ahead_m = float(look_ahead_m)
        self.renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        self._render_data = mujoco.MjData(model)
        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE

    def close(self):
        """Release the renderer's OpenGL resources."""
        self.renderer.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _follow_camera(self, data: mujoco.MjData):
        body_position = np.asarray(data.xpos[self.body_id], dtype=float)
        rotation = np.asarray(data.xmat[self.body_id], dtype=float).reshape(3, 3)
        camera_position = body_position + rotation @ self.offset_body
        forward = _unit(rotation @ self.forward_body, "world forward")
        up = rotation @ self.up_body
        self.renderer.update_scene(data, camera=self._camera)
        # The free viewer camera uses orbit coordinates and loses body roll.
        # Set both GL eyes to the rigid monocular optical pose directly.
        for camera in self.renderer.scene.camera:
            camera.pos[:] = camera_position
            camera.forward[:] = forward
            camera.up[:] = up
            camera.orthographic = 0
            camera.frustum_near = min(.002, camera.frustum_near)
            half_height = camera.frustum_near * math.tan(math.radians(self.fovy_deg) / 2)
            camera.frustum_top = half_height
            camera.frustum_bottom = -half_height
            camera.frustum_center = 0
            # MuJoCo 3.12's GL renderer interprets this as a horizontal half
            # extent. A full width silently halves fx while preserving fy.
            camera.frustum_width = half_height * self.width / self.height

    def capture(self, data: mujoco.MjData, timestamp_us: int) -> CameraFrame:
        """Render the current state and attach the caller's simulation time."""
        if type(timestamp_us) is not int or timestamp_us < 0:
            raise ValueError("timestamp_us must be a non-negative integer")
        # Render a copy so sensing never changes the integration state.
        mujoco.mj_copyData(self._render_data, self.model, data)
        data = self._render_data
        mujoco.mj_forward(self.model, data)
        if self.camera_id >= 0:
            self.renderer.update_scene(data, camera=self.camera_id)
            # Full-field scenes scale the default near plane by scene extent.
            # Keep close dock tags visible as the camera descends; this changes
            # clipping only, retaining named-camera intrinsics and occlusion.
            for camera in self.renderer.scene.camera:
                ratio=min(.002, camera.frustum_near)/camera.frustum_near
                camera.frustum_near*=ratio
                camera.frustum_top*=ratio
                camera.frustum_bottom*=ratio
                camera.frustum_center*=ratio
                camera.frustum_width*=ratio
        else:
            self._follow_camera(data)
        image = np.asarray(self.renderer.render(), dtype=np.uint8).copy()
        return CameraFrame(
            image_rgb=image,
            timestamp_us=int(timestamp_us),
            width=self.width,
            height=self.height,
        )

