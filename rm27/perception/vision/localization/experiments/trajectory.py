"""Offline projection of LocalizationEstimate fields; never fabricate unknown state."""
from dataclasses import fields
from decimal import Decimal
import math
from pathlib import Path

import numpy as np

from ..schema import LocalizationEstimate, TrackingState
from .common import require, number, canonical


def rotation_wxyz(q):
    require(isinstance(q, (tuple, list)) and len(q) == 4, 'POSE_INVALID', 'wxyz quaternion')
    for x in q:
        number(x, 'quaternion component')
    require(abs(math.hypot(*q)-1) <= 1e-6, 'POSE_INVALID', 'unit quaternion tolerance 1e-6; no normalization')
    w,x,y,z = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def canonical_pose(translation, orientation_wxyz, direction):
    require(len(translation)==3, 'POSE_INVALID','translation length')
    for x in translation:
        number(x,'translation')
    r = rotation_wxyz(orientation_wxyz)
    require(direction in ('T_wc','T_cw'), 'POSE_CONVENTION_UNKNOWN', direction)
    t = np.array(translation, dtype=float)
    q = list(orientation_wxyz)
    if direction == 'T_cw':
        t = -r.T@t
        q = [q[0], -q[1], -q[2], -q[3]]
    # These are fields of the existing pose contract, not a second PoseEstimate.
    return dict(parent_frame='backend_world',child_frame='camera',translation=t.tolist(),orientation_wxyz=q)


def parse_tum(path, frames, *, direction, variant, scope, tolerance_s=1e-6, timestamp_significant_digits=None):
    require(variant in ('online','final_optimized'), 'TRAJECTORY_VARIANT_INVALID', variant)
    require(direction in ('T_wc','T_cw'), 'POSE_CONVENTION_UNKNOWN', direction)
    known = [(Decimal(f.encoded_pts), f) for f in frames]
    records, used = [], set()
    previous = None
    for line_number,line in enumerate(Path(path).read_text().splitlines(),1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        items = line.split()
        require(len(items)==8, 'TRAJECTORY_PARSE_FAILED', f'line {line_number}: expected TUM timestamp xyz xyzw')
        values = [float(x) for x in items]
        for x in values:
            number(x,'trajectory component')
        timestamp=Decimal(items[0])
        require(previous is None or timestamp > previous,'TRAJECTORY_ORDER_INVALID',str(line_number))
        # Stella writes 15 significant digits, so Unix-scale seconds lose microseconds.
        # Account only for documented formatting plus IEEE double rounding; still
        # require a unique source association, and retain the actual residual.
        tolerance = Decimal(str(tolerance_s))
        if timestamp_significant_digits is not None:
            require(timestamp_significant_digits == 15, 'TRAJECTORY_FORMAT_UNSUPPORTED', 'pinned Stella precision')
            quantum = Decimal(10) ** (timestamp.adjusted() - timestamp_significant_digits + 1)
            tolerance = max(tolerance, quantum / 2 + Decimal(str(math.ulp(float(timestamp)))))
        candidates=[frame for time,frame in known if abs(time-timestamp)<=tolerance]
        require(len(candidates)==1,'TRAJECTORY_TIME_UNMATCHED',f'line {line_number}: ambiguous or absent source time')
        frame=candidates[0]
        require(frame.source_frame_id not in used,'TRAJECTORY_FRAME_DUPLICATE',str(frame.source_frame_id))
        pose=canonical_pose(values[1:4],[values[7],*values[4:7]],direction)
        # A final pose exists numerically. This says nothing about online tracking.
        projection={**pose,'valid':True,'scale_state':'ARBITRARY','translation_unit':'arbitrary'}
        require(set(projection) <= {f.name for f in fields(LocalizationEstimate)}, 'CONTRACT_FIELD_INVALID','projection')
        records.append(dict(source_frame_id=frame.source_frame_id, source_time_reference=frame.timestamp_evidence,
                            encoded_pts=frame.encoded_pts, trajectory_variant=variant, trajectory_scope=scope,
                            localization_fields=projection, contract='LocalizationEstimate', contract_complete=False,
                            scale_state='ARBITRARY',translation_unit='arbitrary',
                            backend_tracking_state=None,raw_backend_state=None,backend_map_event=None,
                            processing_time_s=None, normalized_validity='POSE_PRESENT_NOT_TRACKING_OR_FLIGHT_QUALIFICATION',
                            timestamp_association_delta_s=float(timestamp-Decimal(frame.encoded_pts)),
                            timestamp_association_tolerance_s=float(tolerance),
                            raw_output=dict(file=Path(path).name,line=line_number,text=line)))
        used.add(frame.source_frame_id)
        previous=timestamp
    require(bool(records),'TRAJECTORY_EMPTY',str(path))
    return records


def materialize_estimate(record, observed_fields):
    """Only when caller has actual missing state/timing/epoch evidence.

    Default adapters cannot call this: TUM exports omit required contract fields.
    No placeholders are synthesized to make an incomplete record appear complete.
    """
    overlap=set(record['localization_fields']) & set(observed_fields)
    require(not overlap, 'CONTRACT_FIELD_CONFLICT', str(sorted(overlap)))
    return LocalizationEstimate(**record['localization_fields'], **observed_fields)


def pose_matrix(record):
    pose=record['localization_fields']
    matrix=np.eye(4)
    matrix[:3,:3]=rotation_wxyz(pose['orientation_wxyz'])
    for x in pose['translation']:
        number(x,'translation')
    matrix[:3,3]=pose['translation']
    return matrix
