"""Flight-admissible v2 localization to MAVLink2 ODOMETRY with explicit registration."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from pymavlink.dialects.v20 import common as mavlink
from ...perception.vision.localization.schema import LocalizationEstimate, ScaleState, TrackingState

@dataclass(frozen=True)
class Registration:
    parent_frame: str
    child_frame: str
    clock_domain: str
    rotation_ned_parent: tuple
    translation_ned_parent: tuple
    rotation_child_frd: tuple

    def __post_init__(self):
        for name in ('rotation_ned_parent','rotation_child_frd'):
            r=np.asarray(getattr(self,name),float)
            if r.shape!=(3,3) or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1):
                raise ValueError('registration requires proper rotations')
        t=np.asarray(self.translation_ned_parent,float)
        if t.shape!=(3,) or not np.isfinite(t).all():raise ValueError('invalid registered translation')

class OdometryAdapter:
    def __init__(self, registration, max_age_s=.1):
        if max_age_s<=0:raise ValueError('max_age must be positive')
        self.registration=registration;self.max_age_s=max_age_s;self.previous=None;self.reset_counter=0
    def packet(self, estimate, now_ns):
        if not isinstance(estimate,LocalizationEstimate):raise ValueError('expected LocalizationEstimate v2')
        e=estimate;r=self.registration;t=e.source_time
        if not e.valid or not e.initialized or e.tracking_state!=TrackingState.TRACKING or e.relocalizing or e.scale_state!=ScaleState.METRIC:
            raise ValueError('localization is not valid metric tracking')
        if (e.parent_frame,e.child_frame)!=(r.parent_frame,r.child_frame):raise ValueError('frame registration mismatch')
        for stamp in (e.source_time,e.publish_time):
            if stamp.unit!='ns' or stamp.clock_domain!=r.clock_domain or stamp.evidence_status!='VERIFIED':raise ValueError('timestamp is not verified in registered clock')
        age=(now_ns-t.raw_value)/1e9
        if not 0<=age<=self.max_age_s or not t.raw_value<=e.publish_time.raw_value<=now_ns:raise ValueError('stale/future localization')
        if e.velocity is None or e.velocity_unit!='m/s' or e.velocity_frame!=e.parent_frame:raise ValueError('registered parent-frame velocity required')
        quality=e.quality or {}
        if quality.get('flight_qualified') is not True:raise ValueError('flight qualification evidence required')
        def covariance(name):
            c=np.asarray(quality.get(name),float)
            if c.shape!=(6,6) or not np.isfinite(c).all() or not np.allclose(c,c.T) or np.linalg.eigvalsh(c).min()<-1e-12:raise ValueError('finite PSD '+name+' required')
            # Explicit axis basis transformation for translation and orientation.
            a=np.zeros((6,6));a[:3,:3]=r.rotation_ned_parent;a[3:,3:]=r.rotation_ned_parent
            c=a@c@a.T;return c[np.triu_indices(6)].tolist()
        pose_cov=covariance('pose_covariance');vel_cov=covariance('velocity_covariance')
        if self.previous:
            if t.raw_value<=self.previous.source_time.raw_value or e.localization_epoch<self.previous.localization_epoch:raise ValueError('time/epoch reversed')
            if e.localization_epoch!=self.previous.localization_epoch:self.reset_counter=(self.reset_counter+1)%256
        parent=np.asarray(r.rotation_ned_parent);child=np.asarray(r.rotation_child_frd)
        q=np.array(e.orientation_wxyz);R=parent@Rotation.from_quat(q[[1,2,3,0]]).as_matrix()@child
        position=parent@e.translation+np.asarray(r.translation_ned_parent);velocity=parent@e.velocity
        quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
        self.previous=e
        # Child frame selects velocity basis in MAVLink ODOMETRY, independent of pose.
        return mavlink.MAVLink_odometry_message(t.raw_value//1000,mavlink.MAV_FRAME_LOCAL_NED,mavlink.MAV_FRAME_LOCAL_NED,
            *position,quat,*velocity,0.,0.,0.,pose_cov,vel_cov,self.reset_counter,mavlink.MAV_ESTIMATOR_TYPE_VISION,100)
