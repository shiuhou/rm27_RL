"""Independent MuJoCo RGB -> OpenCV -> bearing/range calibration experiment."""
import argparse
import json
import os
from pathlib import Path
import time

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import cv2
import numpy as np
from .perception import detect_red_sphere
from .geometry import unit
from .campaign import snapshot_source


def run(output, n=80, seed=27119):
    output.mkdir(parents=True, exist_ok=False)
    digest = snapshot_source(output)
    records = []
    rng = np.random.default_rng(seed)
    ranges = rng.uniform(2, 8, n)
    bearing_yaw = rng.uniform(-10, 10, n)
    bearing_pitch = rng.uniform(-7, 7, n)
    basis = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.]])  # right/down/forward to world
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.column_stack([basis[:, 0], -basis[:, 1], -basis[:, 2]]).ravel())
    xml = '''<mujoco><visual><global offwidth="640" offheight="480"/><headlight ambient=".7 .7 .7" diffuse=".3 .3 .3" specular="0 0 0"/></visual>
    <worldbody><camera name="observer" pos="0 0 2"/><body mocap="true"><geom type="sphere" size=".1" rgba="1 0 0 1"/></body></worldbody></mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    model.cam_quat[0] = quat
    started = time.monotonic()
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        for fov in (60, 90, 120):
            f = 640/(2*np.tan(np.deg2rad(fov/2)))
            model.cam_fovy[0] = np.rad2deg(2*np.arctan(480/(2*f)))
            for i in range(n):
                direction = unit(np.array([np.tan(np.deg2rad(bearing_yaw[i])), -np.tan(np.deg2rad(bearing_pitch[i])), 1.]))
                data.mocap_pos[0] = np.array([0, 0, 2])+basis@direction*ranges[i]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera='observer')
                rgb = renderer.render().copy()
                for blur in (1, 5, 11):
                    kernel = np.ones((1, blur), np.float32)/blur
                    pixels = cv2.filter2D(rgb, -1, kernel)
                    begin = time.perf_counter()
                    detected = detect_red_sphere(pixels, fov)
                    latency = (time.perf_counter()-begin)*1000
                    row = dict(episode=i, seed=seed, fov_deg=fov, horizontal_blur_px=blur, range_m=float(ranges[i]),
                               detected=detected is not None, desktop_detector_ms=latency)
                    if detected:
                        ray = unit(np.array([*np.array(detected['uv_px'])/f, 1.]))
                        row['bearing_error_deg'] = float(np.rad2deg(np.arccos(np.clip(ray@direction, -1, 1))))
                        row['range_relative_error'] = float((.1/np.sin(detected['angular_radius_rad'])-ranges[i])/ranges[i])
                    records.append(row)
                    if i == 0:
                        cv2.imwrite(str(output/f'fov{fov}_blur{blur}.png'), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
    (output/'episodes.json').write_text(json.dumps(records, indent=2)+'\n')
    summary = {}
    for fov in (60, 90, 120):
        for blur in (1, 5, 11):
            group = [r for r in records if r['fov_deg'] == fov and r['horizontal_blur_px'] == blur]
            valid = [r for r in group if r['detected']]
            summary[f'fov{fov}_blur{blur}'] = dict(n=n, detected=len(valid),
                p95_abs_relative_range_error=float(np.quantile([abs(r['range_relative_error']) for r in valid], .95)) if valid else None,
                median_relative_range_error=float(np.median([r['range_relative_error'] for r in valid])) if valid else None,
                p95_bearing_error_deg=float(np.quantile([r['bearing_error_deg'] for r in valid], .95)) if valid else None)
    report = dict(seed=seed, source_sha256=digest, mujoco=mujoco.__version__, opencv=cv2.__version__, elapsed_s=time.monotonic()-started,
                  limits='Isolated red sphere, simple lighting, ideal pinhole, no sensor noise/lens distortion/clutter; blur is a horizontal box filter. Desktop runtime is not MaixCAM latency.',
                  summary=summary)
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--n', type=int, default=80)
    args = parser.parse_args()
    run(args.output, args.n)
