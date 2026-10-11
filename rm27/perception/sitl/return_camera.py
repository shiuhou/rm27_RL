"""Image-only AprilTag position relative to the assigned dock slot.

The known camera mount and slot ID are configuration, not MuJoCo truth.  Only
the rendered image and fresh PX4 attitude enter the return estimate.
"""
from dataclasses import asdict
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .dock_tags import DockTagDetector
from .mujoco_camera import MujocoCamera


BASIS = np.diag([1., -1., -1.])
CV_TO_BODY = np.diag([1., -1., -1.])
RETURN_PHASES = frozenset({'RETURN', 'DESCEND', 'ALIGN', 'FINAL_DESCEND'})


def slot_relative_position(observation, attitude_wxyz, camera_offset_body):
    """Return aircraft XY in NWU relative to the observed slot centre."""
    if not observation.safe_for_control:
        raise ValueError('unsafe tag observation')
    q = np.asarray(attitude_wxyz, dtype=float)
    offset = np.asarray(camera_offset_body, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .02:
        raise ValueError('invalid PX4 attitude')
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('invalid camera mount')
    r_ned_frd = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    r_nwu_flu = BASIS @ r_ned_frd @ BASIS
    camera_to_tag_body = CV_TO_BODY @ np.asarray(observation.translation_camera_m)
    return -(r_nwu_flu @ (offset + camera_to_tag_body))[:2]


class DockReturnCamera:
    def __init__(self, model, output, *, config):
        self.config = dict(config)
        self.output = Path(output) / 'return_camera'
        self.output.mkdir(parents=True, exist_ok=False)
        self.output.joinpath('images').mkdir()
        self.camera = MujocoCamera(model, width=config['width'], height=config['height'],
            fovy_deg=config['fovy_deg'], camera_name=config['camera_name'])
        self.detector = DockTagDetector({int(v): config['tag_size_m'] for v in config['tag_id_by_slot'].values()},
            fovy_deg=config['fovy_deg'])
        self.assigned_id = int(config['tag_id_by_slot'][str(config['assigned_slot'])])
        self.next_image = 0.
        self.last = None
        self.rows = self.output.joinpath('observations.jsonl').open('x', buffering=1)
        self.counts = dict(images=0, assigned_valid=0, assigned_invalid=0, wrong_slot=0)

    def sense(self, data, telemetry, phase):
        if phase not in RETURN_PHASES:
            self.last = None
            return None
        t = float(data.time)
        if t + 1e-9 < self.next_image:
            return self.last if self.last and t-self.last['source_time_s']<=.2 else None
        self.next_image = t + 1. / self.config['camera_hz']
        frame = self.camera.capture(data, round(t*1e6))
        observations = self.detector.detect(frame, now_us=round(t*1e6))
        self.counts['images'] += 1
        self.counts['wrong_slot'] += sum(o.safe_for_control and o.tag_id != self.assigned_id for o in observations)
        assigned = [o for o in observations if o.tag_id == self.assigned_id]
        row = dict(source_time_s=t, phase=phase, assigned_tag_id=self.assigned_id,
            observations=[asdict(o) for o in observations], valid=False, reason='assigned tag missing')
        attitude_age=t-telemetry.get('attitude_time',-100)
        if len(assigned)==1 and assigned[0].safe_for_control and 0<=attitude_age<=.2:
            try:
                xy = slot_relative_position(assigned[0], telemetry['quaternion'],
                    self.config['position_body_flu_m'])
                row.update(valid=True, reason='valid', position_relative_slot_nwu_xy_m=xy.tolist(),
                    attitude_age_s=attitude_age)
                self.counts['assigned_valid'] += 1
            except (ValueError, KeyError) as exc:
                row['reason'] = str(exc)
        elif assigned:
            row['reason'] = 'duplicate/unsafe assigned tag or stale attitude'
        if not row['valid']:
            self.counts['assigned_invalid'] += 1
        self.rows.write(json.dumps(row, allow_nan=False)+'\n')
        if self.counts['images']<=3 or self.counts['images']%20==0:
            cv2.imwrite(str(self.output/'images'/f'{frame.timestamp_us}.png'),
                cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR))
        self.last = row
        return row if row['valid'] else None

    def close(self):
        self.rows.close()
        self.camera.close()
        self.output.joinpath('summary.json').write_text(json.dumps(dict(
            passed=self.counts['assigned_valid']>0, run_type='current-flight image return camera development',
            assigned_tag_id=self.assigned_id, counts=self.counts, flight_qualified=False), indent=2)+'\n')
