"""Image-derived task return height and goals; preserve raw PX4 EKF telemetry."""
import json
import numpy as np
from scipy.spatial.transform import Rotation

from .live_flight import VioRuntime

BASIS=np.diag([1.,-1.,-1.])


def image_height(row,quaternion,camera_offset,*,plane_above_slot_m=.0015):
    assigned=[o for o in row.get('observations',[]) if o.get('tag_id')==row.get('assigned_tag_id')
        and o.get('safe_for_control')]
    if not row.get('valid') or len(assigned)!=1:raise ValueError('fresh assigned metric tag required')
    q=np.asarray(quaternion,float);translation=np.asarray(assigned[0]['translation_camera_m'],float)
    offset=np.asarray(camera_offset,float)
    if q.shape!=(4,) or translation.shape!=(3,) or offset.shape!=(3,) or not np.isfinite(np.r_[q,translation,offset,plane_above_slot_m]).all():
        raise ValueError('invalid image height inputs')
    if abs(np.linalg.norm(q)-1)>.02:raise ValueError('invalid attitude norm')
    rotation=BASIS@Rotation.from_quat(q[[1,2,3,0]]).as_matrix()@BASIS
    relative=-rotation@(offset+BASIS@translation)
    height=float(relative[2]+plane_above_slot_m)
    if not .05<=height<=1.:raise ValueError('tag height outside reviewed return range')
    return height


class TagHeightRuntime(VioRuntime):
    """Translate task height and outgoing goals consistently into raw EKF NED."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.raw_local=None;self.height_correction=0.;self.height_source_time=None;self.height_stream=None

    def __enter__(self):
        runtime=super().__enter__()
        self.height_stream=(self.output/'tag_height_registration.jsonl').open('x',buffering=1)
        return runtime

    def _apply(self):
        if self.raw_local is None:return
        corrected=self.raw_local.copy();corrected[2]+=self.height_correction
        self.telemetry.update(local_position=corrected.tolist(),raw_local_position_ned=self.raw_local.tolist(),
            tag_height_registration_m=self.height_correction,tag_height_source_time_s=self.height_source_time)

    def receive(self):
        if self.raw_local is not None:self.telemetry['local_position']=self.raw_local.tolist()
        messages=super().receive()
        if 'local_position' in self.telemetry:self.raw_local=np.asarray(self.telemetry['local_position'],float).copy()
        self._apply()
        return messages

    def step(self,timeout=5.):
        result=super().step(timeout);row=self.return_observation
        now=float(self.plant.data.time)
        self.observe_return_height(row,now)
        self._apply()
        return result

    def observe_return_height(self,row,now):
        if row and row.get('valid') and self.raw_local is not None:
            source=row['source_time_s']
            # Cached or backwards source time cannot refresh the registration.
            # Compare image and raw EKF height at the same acquisition time.
            fresh=all(0<=now-stamp<=.2 for stamp in (source,
                self.telemetry.get('attitude_time',-100),self.telemetry.get('local_time',-100)))
            newer=self.height_source_time is None or source>self.height_source_time
            if fresh and newer and row.get('assigned_tag_id')==self.return_camera.assigned_id:
                try:
                    height=image_height(row,self.telemetry['quaternion'],self.return_camera.config['position_body_flu_m'])
                    age=source-self.telemetry['local_time']
                    raw_height=-float(self.raw_local[2])-float(self.telemetry['local_velocity'][2])*age
                    correction=raw_height-height
                    if abs(correction)>.25:raise ValueError('image height registration exceeds 25 cm')
                    self.height_correction=(correction if self.height_source_time is None else
                        .8*self.height_correction+.2*correction)
                    self.height_source_time=source
                    if self.height_stream:self.height_stream.write(json.dumps(dict(timestamp_s=now,source_time_s=source,
                        image_height_m=height,raw_ekf_height_m=raw_height,correction_m=self.height_correction,
                        source='assigned AprilTag metric pose, PX4 attitude and configured mount/plane offset'))+'\n')
                except ValueError as exc:
                    if self.height_stream:self.height_stream.write(json.dumps(dict(timestamp_s=now,rejected=str(exc)))+'\n')

    def position(self,position,yaw=0.,velocity=None,acceleration=None):
        raw_goal=np.asarray(position,float).copy();raw_goal[2]-=self.height_correction
        return super().position(raw_goal,yaw,velocity,acceleration)

    def close(self):
        super().close()
        if self.height_stream:self.height_stream.close();self.height_stream=None
