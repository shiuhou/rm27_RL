"""Runtime observations around public backend calls, not a new pose contract.

The pose is an incomplete LocalizationEstimate v2 field projection. No final
trajectory, pose absence, or state transition supplies missing backend events.
"""
import csv
from dataclasses import asdict, dataclass
from decimal import Decimal
import json
import math
from pathlib import Path

import numpy as np

from .benchmark import timing_stats
from .common import canonical, require, number
from .trajectory import canonical_pose

STATE_MAP = {
    # /poseimu alone does not report tracking/reset state. Never infer TRACKING
    # from publication or successful initialization of the upstream estimator.
    'OpenVINS': {},
    'stella_vslam': {'Initializing':'INITIALIZING', 'Tracking':'TRACKING', 'Lost':'LOST'},
    'ORB-SLAM3': {'-1':'UNINITIALIZED', '0':'UNINITIALIZED', '1':'INITIALIZING',
                  '2':'TRACKING', '3':'LOST', '4':'LOST'},
}
# ORB RECENTLY_LOST=3 retains its raw identity. OK_KLT=5 is not exercised by
# this pinned monocular implementation, so is deliberately UNKNOWN here.


def normalized_state(backend, raw):
    require(backend in STATE_MAP, 'BACKEND_UNSUPPORTED', backend)
    return STATE_MAP[backend].get(str(raw), 'UNKNOWN')


@dataclass(frozen=True)
class OnlinePoseRecord:
    source_frame_id: int
    source_timestamp: dict
    backend: str
    backend_version: dict
    raw_backend_state: str
    tracking_state: str
    initialization_state: str
    validity: bool
    reason: str
    encoded_pts: str
    localization_fields: dict | None = None
    backend_timestamp: dict | None = None
    publish_timestamp: dict | None = None
    processing_completed_timestamp: dict | None = None
    observation_timestamp: dict | None = None
    processing_time_s: float | None = None
    map_id: str | None = None
    localization_epoch: int | None = None
    reset_event: dict | None = None
    relocalization_event: dict | None = None
    raw_output: dict | None = None
    backend_pose_convention: str | None = None
    schema_version: int = 1
    trajectory_variant: str = 'online'
    pose_convention: str = 'T_parent_child'
    contract: str = 'LocalizationEstimate'
    contract_complete: bool = False

    def __post_init__(self):
        require(type(self.schema_version) is int and self.schema_version == 1,
                'ONLINE_RECORD_INVALID', 'schema version')
        require(type(self.source_frame_id) is int and self.source_frame_id >= 0,
                'ONLINE_RECORD_INVALID','source frame ID')
        require(self.trajectory_variant == 'online', 'TRAJECTORY_VARIANT_MIXED','online record')
        require(self.tracking_state == normalized_state(self.backend,self.raw_backend_state),
                'ONLINE_STATE_INVALID','raw state mapping must be preserved')
        require(self.initialization_state == {'UNINITIALIZED':'UNINITIALIZED','INITIALIZING':'INITIALIZING','TRACKING':'INITIALIZED'}.get(self.tracking_state,'UNKNOWN'),
                'ONLINE_STATE_INVALID','initialization observation mapping')
        if self.localization_fields is not None:
            require(self.localization_fields.get('valid') is self.validity,
                    'ONLINE_VALIDITY_INVALID','pose validity disagrees with observation')
        require(type(self.validity) is bool and self.validity ==
                (self.tracking_state == 'TRACKING' and self.localization_fields is not None),
                'ONLINE_VALIDITY_INVALID','current tracking pose only, not flight eligibility')
        if self.processing_time_s is not None:
            number(self.processing_time_s,'processing time')
            require(self.processing_time_s >= 0,'ONLINE_TIME_INVALID','negative processing duration')
        canonical(asdict(self))

    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return canonical(self.to_dict())

    @classmethod
    def from_json(cls, text):
        return cls(**json.loads(text))


def matrix_pose(matrix, direction):
    """Convert a checked rigid matrix; reject invalid transforms, never repair."""
    m=np.array(matrix,dtype=float)
    require(m.shape==(4,4) and np.isfinite(m).all(), 'POSE_INVALID','finite 4x4 matrix')
    r=m[:3,:3]
    require(np.allclose(m[3],[0,0,0,1],atol=1e-6,rtol=0)
            and np.allclose(r.T@r,np.eye(3),atol=1e-6,rtol=0)
            and abs(np.linalg.det(r)-1)<1e-6, 'POSE_INVALID','rigid matrix')
    # Algebraic matrix -> wxyz conversion, without unit-normalization or SVD.
    if np.trace(r)>0:
        s=2*math.sqrt(1+np.trace(r))
        q=[s/4,(r[2,1]-r[1,2])/s,(r[0,2]-r[2,0])/s,(r[1,0]-r[0,1])/s]
    else:
        i=int(np.argmax(np.diag(r)));j=(i+1)%3;k=(i+2)%3
        s=2*math.sqrt(1+r[i,i]-r[j,j]-r[k,k])
        q=[(r[k,j]-r[j,k])/s,0.,0.,0.]
        q[i+1]=s/4;q[j+1]=(r[j,i]+r[i,j])/s;q[k+1]=(r[k,i]+r[i,k])/s
    return canonical_pose(m[:3,3].tolist(),[float(x) for x in q],direction)


def parse_online(path, frames, backend, installation, run_id):
    """Associate raw input ordinals with original canonical source IDs/times."""
    records=[];previous_index=-1;previous_observed=-1
    direction='T_wc' if backend=='stella_vslam' else 'T_cw'
    columns=['input_index','backend_timestamp','processing_started_ns','processing_completed_ns',
             'observation_completed_ns','raw_state','pose_present']+[f't{i}{j}' for i in range(4) for j in range(4)]
    with Path(path).open(newline='') as stream:
        reader=csv.DictReader(stream)
        require(reader.fieldnames==columns,'ONLINE_FORMAT_INVALID','CSV v1 header')
        for line,row in enumerate(reader,2):
            index=int(row['input_index'])
            require(previous_index<index<len(frames),'ONLINE_FRAME_ORDER_INVALID',str(index))
            frame=frames[index]
            timestamp=Decimal(row['backend_timestamp']);original=Decimal(frame.encoded_pts)
            require(timestamp.is_finite() and abs(timestamp-original)<=Decimal(str(max(1e-9,math.ulp(float(original))))),
                    'ONLINE_TIMESTAMP_MISMATCH','backend double timestamp must identify input frame')
            started,completed,observed=[int(row[k]) for k in columns[2:5]]
            require(0<=started<=completed<=observed and started>=previous_observed,
                    'ONLINE_TIME_INVALID','serial processing and observation clock order')
            state=normalized_state(backend,row['raw_state'])
            require(row['pose_present'] in ('0','1'),'ONLINE_FORMAT_INVALID','pose presence')
            pose=None
            if row['pose_present']=='1':
                pose=matrix_pose([[float(row[f't{i}{j}']) for j in range(4)] for i in range(4)],direction)
                pose.update(valid=state=='TRACKING',scale_state='ARBITRARY',translation_unit='arbitrary')
            else:
                require(all(row[k]=='' for k in columns[7:]),'ONLINE_FORMAT_INVALID','absent pose has matrix values')
            valid=pose is not None and state=='TRACKING'
            def host_stamp(value,semantic):
                return dict(raw_value=value,unit='ns',clock_domain=run_id+':backend_process_steady_clock',
                            semantic=semantic,evidence_status='OBSERVED')
            record=OnlinePoseRecord(source_frame_id=frame.source_frame_id,source_timestamp=frame.timestamp_evidence,
                encoded_pts=frame.encoded_pts,backend=backend,
                backend_version={k:installation[k] for k in ('commit','binary_sha256')},
                raw_backend_state=row['raw_state'],tracking_state=state,
                initialization_state={'UNINITIALIZED':'UNINITIALIZED','INITIALIZING':'INITIALIZING','TRACKING':'INITIALIZED'}.get(state,'UNKNOWN'),
                validity=valid,reason='CURRENT_TRACKING_POSE' if valid else 'NO_POSE_RETURNED' if pose is None else 'API_RETURN_NOT_CURRENT_TRACKING',
                localization_fields=pose,backend_pose_convention=direction,
                backend_timestamp=dict(raw_value=row['backend_timestamp'],unit='s',clock_domain=frame.timestamp_evidence['clock_domain'],
                                       semantic='BACKEND_INPUT_TIMESTAMP',evidence_status='OBSERVED'),
                processing_completed_timestamp=host_stamp(completed,'PROCESSING_COMPLETION'),
                observation_timestamp=host_stamp(observed,'OBSERVATION_COMPLETION_BEFORE_SERIALIZATION'),
                processing_time_s=(completed-started)/1e9,
                raw_output=dict(file=Path(path).name,line=line,input_index=index))
            records.append(record.to_dict());previous_index=index;previous_observed=observed
    return records


def online_metrics(frames, records):
    """Sampled online behavior only; never reads final poses or fits a trajectory."""
    require(all(r['trajectory_variant']=='online' for r in records), 'TRAJECTORY_VARIANT_MIXED','online metrics')
    ids=[r['source_frame_id'] for r in records];expected=[f.source_frame_id for f in frames]
    require(ids==sorted(set(ids)) and set(ids)<=set(expected),'ONLINE_FRAME_ORDER_INVALID','online metrics')
    complete=ids==expected
    known=all(r['tracking_state']!='UNKNOWN' for r in records)
    valid=[r for r in records if r['validity']]
    tracking=[r for r in records if r['tracking_state']=='TRACKING']
    transitions=[]
    for a,b in zip(records,records[1:]):
        if a['tracking_state']!=b['tracking_state']:
            transitions.append(dict(from_frame=a['source_frame_id'],to_frame=b['source_frame_id'],
                                    from_state=a['tracking_state'],to_state=b['tracking_state'],
                                    adjacent_input=expected.index(b['source_frame_id'])==expected.index(a['source_frame_id'])+1))
    lost=None;duration=None
    if complete and known:
        lost=[];current=[];duration=0.
        for i,r in enumerate(records):
            if r['tracking_state']=='LOST':
                current.append(r)
                if i+1<len(records):
                    duration+=float(Decimal(records[i+1]['encoded_pts'])-Decimal(r['encoded_pts']))
                else: duration=None
            if current and (r['tracking_state']!='LOST' or i==len(records)-1):
                censored=r['tracking_state']=='LOST'
                lost.append(dict(first_frame=current[0]['source_frame_id'],last_frame=current[-1]['source_frame_id'],
                                 frame_count=len(current),right_censored=censored,
                                 observed_span_s=float(Decimal(current[-1]['encoded_pts'])-Decimal(current[0]['encoded_pts']))))
                current=[]
    return dict(trajectory_variant='online',input_frame_count=len(frames),observed_frame_count=len(records),
        frame_coverage=len(records)/len(frames),state_evidence='COMPLETE' if complete and known else 'PARTIAL',
        emitted_online_pose_count=sum(r['localization_fields'] is not None for r in records),
        valid_online_pose_count=len(valid),valid_pose_frame_coverage=len(valid)/len(frames),
        tracking_coverage=len(tracking)/len(frames) if complete and known else None,
        initialization_frame=tracking[0]['source_frame_id'] if complete and known and tracking else None,
        first_valid_pose_frame=valid[0]['source_frame_id'] if complete and valid else None,
        first_observed_valid_pose_frame=valid[0]['source_frame_id'] if valid else None,
        initialization_policy='first observed TRACKING sample in a complete known-state trace; not an internal event timestamp',
        first_valid_pose_source_timestamp=valid[0]['source_timestamp'] if complete and valid else None,
        first_valid_pose_elapsed_dataset_s=float(Decimal(valid[0]['encoded_pts'])-Decimal(frames[0].encoded_pts)) if complete and valid else None,
        tracking_state_transitions=transitions,lost_intervals=lost,lost_duration_s=duration,
        lost_duration_policy='sample-and-hold over original dataset times; null for unknown, missing samples or terminal LOST',
        relocalization_events=None,map_changes=None,reset_events=None,
        processing_time_statistics=timing_stats([r['processing_time_s'] for r in records if r['processing_time_s'] is not None]),
        source_to_publish_latency_s=None,source_exposure_latency_s=None,
        limits='pose presence may be an uninitialized API placeholder; validity requires observed TRACKING; no flight eligibility')
