"""Public-data scope checks use synthetic files only; not backend qualification."""
from dataclasses import replace
from types import SimpleNamespace
import math
import pytest
from slam_future_fixtures import make_dataset
from rm27.perception.vision.localization.experiments.common import ExperimentError, read_json, write_json
from rm27.perception.vision.localization.experiments.dataset import CanonicalDataset
from rm27.perception.vision.localization.experiments.calibration import validated_calibration
from rm27.perception.vision.localization.experiments.public_tum import published_calibration
from rm27.perception.vision.localization.experiments.trajectory import parse_tum, materialize_estimate, rotation_wxyz
from rm27.perception.vision.localization.schema import TimestampEvidence, TrackingState, ScaleState


def test_public_scope_cannot_relabel_rm27_calibration_required(tmp_path):
    p=make_dataset(tmp_path/'data');m=read_json(p)
    m.update(dataset_kind='TUM_RGBD_PUBLIC', status='CALIBRATION_REQUIRED')
    write_json(p,m)
    # Gate must reject before considering any provider-specific path.
    with pytest.raises(ExperimentError,match='CALIBRATION_REQUIRED'):
        validated_calibration(SimpleNamespace(manifest=m))


def test_public_scope_requires_exact_archive_identity():
    from rm27.perception.vision.localization.experiments.public_tum import DATASET_ID
    m=dict(dataset_id=DATASET_ID,purpose='VSL-3P_PUBLIC_DATASET',status='PUBLIC_REFERENCE',source={})
    with pytest.raises(ExperimentError,match='PUBLIC_DATASET_INVALID'):
        published_calibration(SimpleNamespace(manifest=m,provenance={}))


def test_public_timestamp_is_not_encoded_or_exposure(tmp_path):
    p=make_dataset(tmp_path/'data');m=read_json(p)
    m['dataset_kind']='TUM_RGBD_PUBLIC';m['source']['source_archive']=m['source'].pop('source_video')
    write_json(p,m)
    with pytest.raises(ExperimentError,match='TIMESTAMP_UNSUPPORTED'):
        CanonicalDataset(p)
    frames=p.parent/m['frames_file']
    frames.write_text(frames.read_text().replace('ENCODED_STREAM_PTS','DATASET_TIMESTAMP'))
    ds=CanonicalDataset(p)
    assert ds.frames[0].timestamp_evidence['semantic']=='DATASET_TIMESTAMP'


@pytest.mark.parametrize('backend',['stella_vslam','ORB-SLAM3'])
def test_real_adapter_twc_export_and_v2_arbitrary_scale(tmp_path,backend):
    from rm27.perception.vision.localization.experiments.backends import BackendAdapter
    ds=CanonicalDataset(make_dataset(tmp_path/'data'));q=math.sqrt(.5)
    name='frame_trajectory.txt' if backend=='stella_vslam' else 'KeyFrameTrajectory.txt'
    (tmp_path/name).write_text(f'0 1 2 3 0 0 {q:.15f} {q:.15f}\n')
    output=BackendAdapter(backend).collect_outputs(tmp_path)
    record=parse_tum(output['final_trajectory'],ds.frames,direction=output['pose_direction'],
                     variant='final_optimized',scope=output['trajectory_scope'])[0]
    pose=record['localization_fields']
    assert pose['translation']==[1,2,3]
    assert rotation_wxyz(pose['orientation_wxyz'])@ [1,0,0]==pytest.approx([0,1,0])
    estimate=materialize_estimate(record,dict(source_time=TimestampEvidence(0),publish_time=TimestampEvidence(1),
        tracking_state=TrackingState.TRACKING, initialized=True,relocalizing=False,localization_epoch=0))
    assert estimate.scale_state==ScaleState.ARBITRARY and estimate.translation_unit=='arbitrary'


def test_stella_unix_timestamp_precision_is_bounded_and_unambiguous(tmp_path):
    ds=CanonicalDataset(make_dataset(tmp_path/'data'))
    f=replace(ds.frames[0],encoded_pts='1305031102.175304')
    raw=tmp_path/'trajectory.txt';raw.write_text('1305031102.1753 1 2 3 0 0 0 1\n')
    options=dict(direction='T_wc',variant='final_optimized',scope='frames')
    with pytest.raises(ExperimentError,match='TRAJECTORY_TIME_UNMATCHED'):
        parse_tum(raw,[f],**options)
    record=parse_tum(raw,[f],**options,timestamp_significant_digits=15)[0]
    assert record['encoded_pts']==f.encoded_pts
    assert record['timestamp_association_delta_s']==pytest.approx(-.000004)
    with pytest.raises(ExperimentError,match='TRAJECTORY_TIME_UNMATCHED'):
        parse_tum(raw,[f,replace(f,source_frame_id=1,encoded_pts='1305031102.175301')],
                  **options,timestamp_significant_digits=15)


def test_orb_settings_requires_integer_nominal_fps():
    from rm27.perception.vision.localization.experiments.public_tum import PARAMETERS, CALIBRATION_ID
    from rm27.perception.vision.localization.experiments.backends import camera_config
    c=dict(PARAMETERS,calibration_id=CALIBRATION_ID)
    config=camera_config('ORB-SLAM3',c,{'sha256':'test'},30.)
    assert '\nCamera.fps: 30\n' in config
    with pytest.raises(ExperimentError,match='BACKEND_RATE_UNSUPPORTED'):
        camera_config('ORB-SLAM3',c,{'sha256':'test'},29.97)
