"""Simulation-only RGB backend. The detector receives pixels, never truth."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import cv2
import mujoco
import numpy as np
from .perception import detect_red_sphere


class RenderedCamera:
    def __init__(self, cfg, blur_pixels=1):
        xml = f'''<mujoco><visual><global offwidth="{cfg.width}" offheight="{cfg.height}"/>
        <headlight ambient=".7 .7 .7" diffuse=".3 .3 .3" specular="0 0 0"/></visual>
        <worldbody><camera name="observer" quat="0 1 0 0"/><body mocap="true">
        <geom type="sphere" size="{cfg.target_radius_m}" rgba="1 0 0 1"/></body></worldbody></mujoco>'''
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        f = cfg.width/(2*np.tan(np.deg2rad(cfg.fov_deg/2)))
        self.model.cam_fovy[0] = np.rad2deg(2*np.arctan(cfg.height/(2*f)))
        self.renderer = mujoco.Renderer(self.model, height=cfg.height, width=cfg.width)
        self.kernel = np.ones((1, blur_pixels), np.float32)/blur_pixels
        self.frames = 0

    def measure(self, relative, basis, cfg):
        camera_relative = np.einsum('nji,nj->ni', basis, relative)
        n = len(relative)
        uv, angle, visible = np.zeros((n, 2)), np.zeros(n), np.zeros(n, bool)
        for i in range(n):
            self.data.mocap_pos[0] = camera_relative[i]
            mujoco.mj_forward(self.model, self.data)
            self.renderer.update_scene(self.data, camera='observer')
            rgb = self.renderer.render()
            rgb = cv2.filter2D(rgb, -1, self.kernel)
            detected = detect_red_sphere(rgb, cfg.fov_deg)
            self.frames += 1
            if detected is not None:
                uv[i], angle[i], visible[i] = detected['uv_px'], detected['angular_radius_rad'], True
        return uv, angle, visible

    def close(self):
        self.renderer.close()
