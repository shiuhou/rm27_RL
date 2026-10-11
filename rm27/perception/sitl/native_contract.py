"""Observable native OpenVINS state to LocalizationEstimate schema v2.

Native feature counts, initialization, state covariance and publication time
are mandatory. This adapter never converts an uninstrumented ROS pose into
tracking evidence. Flight eligibility is supplied by a separate qualification.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from rm27.perception.vision.localization.schema import LocalizationEstimate,TimestampEvidence,TrackingState,ScaleState


class NativeContract:
    def __init__(self,*,flight_qualified=False,minimum_features=8,clock_domain='mujoco_simulation'):
        self.qualified=flight_qualified;self.minimum_features=minimum_features;self.clock_domain=clock_domain
        self.previous_sequence=-1;self.previous_state=-1;self.epoch=0

    def estimate(self,row):
        sequence=row['sequence'];epoch=row['epoch']
        if type(sequence) is not int or sequence<=self.previous_sequence:raise ValueError('Native publication order reversed')
        if type(epoch) is not int or epoch<self.epoch:raise ValueError('Native localization epoch reversed')
        covariance=np.asarray(row['covariance_p_q_v'],float).reshape(9,9)
        if not np.isfinite(covariance).all() or not np.allclose(covariance,covariance.T,atol=1e-8) or np.linalg.eigvalsh(covariance).min()<-1e-8:
            raise ValueError('Native covariance invalid')
        time_ns=round(float(row['state_time_s'])*1e9)
        camera_time_ns=round(float(row['camera_time_s'])*1e9)
        pub=row['publish_monotonic_ns'];completed=row['camera_processing_completed_monotonic_ns'];started=row['camera_processing_started_monotonic_ns']
        if type(pub) is not int or not 0<=started<=completed<=pub:raise ValueError('Native processing/publication times invalid')
        initialized=row['initialized']
        if type(initialized) is not bool:raise ValueError('Native initialization must be observed boolean')
        if initialized and time_ns<=self.previous_state and epoch==self.epoch:raise ValueError('Native state time reversed')
        features=int(row['msckf_update_features'])+int(row['slam_features'])
        fresh_camera=0<=time_ns-camera_time_ns<=1_000_000
        valid=initialized and features>=self.minimum_features and fresh_camera
        state=TrackingState.TRACKING if valid else TrackingState.LOST if initialized else TrackingState.INITIALIZING
        q=row['quaternion_xyzw'];norm=np.linalg.norm(q)
        if not np.isfinite(q).all() or abs(norm-1)>1e-6:raise ValueError('Native quaternion invalid')
        # OpenVINS stores JPL G->I, whose coefficients represent Hamilton
        # I->G. Its left JPL error is a right Hamilton perturbation in I.
        # Rotate orientation error into the parent frame before the MAVLink
        # adapter applies its parent->NED transform. Preserve cross terms.
        basis=np.eye(9);basis[3:6,3:6]=Rotation.from_quat(q).as_matrix()
        covariance=basis@covariance@basis.T
        pose=covariance[:6,:6];velocity=np.zeros((6,6));velocity[:3,:3]=covariance[6:,6:];velocity[3:,3:]=np.eye(3)*1e6
        estimate=LocalizationEstimate(
            source_time=TimestampEvidence(max(0,time_ns),'ns',self.clock_domain,'NATIVE_IMU_STATE_TIME','VERIFIED'),
            publish_time=TimestampEvidence(pub,'ns','native_monotonic','NATIVE_PRE_PUBLICATION','VERIFIED'),
            parent_frame='openvins_world',child_frame='body_flu',translation=tuple(row['position']),
            orientation_wxyz=(q[3],*q[:3]),tracking_state=state,valid=valid,initialized=initialized,
            relocalizing=False,localization_epoch=epoch,scale_state=ScaleState.METRIC,translation_unit='m',
            velocity_unit='m/s',velocity=tuple(row['velocity_world']),velocity_frame='openvins_world',
            quality=dict(flight_qualified=self.qualified,pose_covariance=pose.tolist(),velocity_covariance=velocity.tolist(),
                msckf_update_features=row['msckf_update_features'],slam_features=row['slam_features'],minimum_features=self.minimum_features,
                processing_started_monotonic_ns=started,processing_completed_monotonic_ns=completed,
                processing_duration_s=(completed-started)*1e-9,sequence=sequence,
                pose_covariance_basis='parent-frame translation and left Hamilton orientation error',
                tracking_policy='native initialized, at least minimum update features, state matches current camera',
                scope='simulation host only; no M3C runtime qualification'))
        self.previous_sequence=sequence;self.epoch=epoch
        if initialized:self.previous_state=time_ns
        return estimate
