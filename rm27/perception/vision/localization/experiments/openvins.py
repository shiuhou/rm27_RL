"""Pinned OpenVINS online projection boundary, not a complete runtime adapter.

Retains partial LocalizationEstimate v2 fields via existing OnlinePoseRecord.
Neither a ROS pose nor a recorder receipt proves tracking or publication time.
"""
from decimal import Decimal
import math

from ..schema import TimestampEvidence
from .common import require, number
from .online import OnlinePoseRecord
from .trajectory import rotation_wxyz

PIN = '69488123ed9362dd44b6f28e7f4680abbff1442b'


def pose_observation(*, source_frame_id, image_ns, state_ns, recorder_ns,
                     position, quaternion_xyzw, run_id, epoch, binary_sha256,
                     raw_reference):
    """Project an already associated /poseimu sample; no time association here.

    JPL G->I xyzw equals Hamilton I->G xyzw (pinned ROS2Visualizer comment).
    The run ID and epoch belong to this adapter, not an upstream reset/map ID.
    Receipt time is kept as an unverified recorder clock, never publish time.
    """
    require(isinstance(run_id, str) and bool(run_id), 'RUN_ID_INVALID', 'run ID required')
    require(type(epoch) is int and epoch >= 0, 'EPOCH_INVALID', 'adapter-owned nonnegative epoch')
    require(isinstance(binary_sha256, str) and len(binary_sha256) == 64
            and all(c in '0123456789abcdef' for c in binary_sha256), 'BINARY_HASH_INVALID', 'SHA256')
    require(isinstance(raw_reference, dict) and raw_reference.get('topic') == '/poseimu',
            'RAW_REFERENCE_INVALID', 'visual-state stream only')
    clock = 'EuRoC:V1_01_easy:sensor_time'
    image = TimestampEvidence(image_ns, 'ns', clock, 'DATASET_CAMERA_TIMESTAMP', 'VERIFIED').to_dict()
    state = TimestampEvidence(state_ns, 'ns', clock, 'ESTIMATOR_IMU_STATE_TIME_DOUBLE_DERIVED', 'CODE_SUPPORTED').to_dict()
    receipt = TimestampEvidence(recorder_ns, 'ns', run_id+':recorder_clock_UNVERIFIED',
                                'RECORDER_RECEIPT_TIME_NOT_PUBLICATION', 'OBSERVED').to_dict()
    require(len(position) == 3 and len(quaternion_xyzw) == 4, 'POSE_INVALID', 'pose dimensions')
    p = [number(v, 'position') for v in position]
    x,y,z,w = [number(v, 'quaternion') for v in quaternion_xyzw]
    q = [w,x,y,z]
    rotation_wxyz(q)  # strict unit check, no implicit quaternion normalization
    pose = dict(parent_frame='backend_world', child_frame='imu', translation=p,
                orientation_wxyz=q, scale_state='METRIC', translation_unit='m',
                valid=False, localization_epoch=epoch, source_time=state)
    return OnlinePoseRecord(source_frame_id=source_frame_id, source_timestamp=image,
        backend='OpenVINS', backend_version=dict(commit=PIN, binary_sha256=binary_sha256),
        raw_backend_state='UNOBSERVED', tracking_state='UNKNOWN', initialization_state='UNKNOWN',
        validity=False, reason='POSE_OBSERVED_STATE_AND_PUBLICATION_EVIDENCE_UNAVAILABLE',
        encoded_pts=str(Decimal(image_ns)/Decimal(10**9)), localization_fields=pose,
        backend_timestamp=state, observation_timestamp=receipt, localization_epoch=epoch,
        backend_pose_convention='JPL_GtoI_xyzw_equivalent_Hamilton_ItoG_xyzw',
        raw_output={**raw_reference, 'epoch_owner':'adapter_run',
                    'upstream_reset_event':None, 'imu_data_coverage':None}).to_dict()
