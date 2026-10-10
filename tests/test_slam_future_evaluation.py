import math
from pathlib import Path
import sys

import numpy as np
import pytest

from slam_future_fixtures import make_dataset, LABEL
from rm27.perception.vision.localization.experiments.common import ExperimentError, canonical, read_json, write_json, file_hash
from rm27.perception.vision.localization.experiments.dataset import CanonicalDataset
from rm27.perception.vision.localization.experiments.trajectory import canonical_pose, rotation_wxyz, parse_tum, materialize_estimate
from rm27.perception.vision.localization.experiments.evaluation import trajectory_metrics, behavior_metrics
from rm27.perception.vision.localization.experiments.benchmark import execute_process, timing_stats
from rm27.perception.vision.localization.experiments.transforms import prepare_images
from rm27.perception.vision.localization.experiments.vio import validate_imu_sample
from rm27.perception.vision.localization.schema import TimestampEvidence,TrackingState,LocalizationEstimate


def test_inverse_rotates_translation_not_just_negation():
    q=[math.sqrt(.5),0,0,math.sqrt(.5)]
    pose=canonical_pose([1,2,3],q,'T_cw')
    assert pose['translation']==pytest.approx([-2,1,-3])
    assert rotation_wxyz(pose['orientation_wxyz'])==pytest.approx(rotation_wxyz(q).T)
    back=canonical_pose(pose['translation'],pose['orientation_wxyz'],'T_cw')
    assert back['translation']==pytest.approx([1,2,3])
    with pytest.raises(ExperimentError):canonical_pose([1,2,3],[2,0,0,0],'T_wc')
    with pytest.raises(ExperimentError):canonical_pose([1,2,3],[1,0,0,0],'UNKNOWN')


def test_normalization_retains_unknown_fields_and_existing_contract(tmp_path):
    dataset=CanonicalDataset(make_dataset(tmp_path/'data'))
    raw=tmp_path/'raw.txt';raw.write_text('0 1 2 3 0 0 0 1\n')
    record=parse_tum(raw,dataset.frames,direction='T_wc',variant='final_optimized',scope='keyframes_only')[0]
    assert record['localization_fields']['translation']==[1,2,3]
    assert record['backend_tracking_state'] is None and record['backend_map_event'] is None
    assert record['contract_complete'] is False
    with pytest.raises(TypeError):materialize_estimate(record,{})
    # TEST_FIXTURE_ONLY observed state and times, no invented values in production parser.
    observed=dict(source_time=TimestampEvidence(0),publish_time=TimestampEvidence(1),tracking_state=TrackingState.TRACKING,
                  initialized=True,relocalizing=False,localization_epoch=0)
    assert isinstance(materialize_estimate(record,observed),LocalizationEstimate)


@pytest.mark.parametrize('line',['0 0 0 0 0 0 0 2','0 nan 0 0 0 0 0 1','99 0 0 0 0 0 0 1','0 0 0','0 0 0 0 0 0 0 1\n0 0 0 0 0 0 0 1'])
def test_bad_backend_outputs_rejected(tmp_path,line):
    dataset=CanonicalDataset(make_dataset(tmp_path/'data'))
    raw=tmp_path/'raw.txt';raw.write_text(line+'\n')
    with pytest.raises(ExperimentError):parse_tum(raw,dataset.frames,direction='T_wc',variant='final_optimized',scope='frames')


def trajectories():
    points=np.array([[0,0,0],[1,0,0],[1,1,0],[1,1,1],[2,1,1]],float)
    q=[math.sqrt(.5),0,0,math.sqrt(.5)];r=rotation_wxyz(q)
    estimate=[];reference=[]
    for i,p in enumerate(points):
        base=dict(source_frame_id=i,encoded_pts=str(i),trajectory_variant='final_optimized')
        estimate.append(dict(**base,localization_fields=canonical_pose(p.tolist(),[1,0,0,0],'T_wc')))
        reference.append(dict(**base,localization_fields=canonical_pose((2*r@p+[3,4,5]).tolist(),q,'T_wc')))
    return estimate,reference


def test_global_sim3_and_se3_scale_behavior():
    estimated,reference=trajectories()
    sim=trajectory_metrics(estimated,reference,'Sim3',continuity_verified=True)
    rigid=trajectory_metrics(estimated,reference,'SE3',continuity_verified=True)
    assert sim['global_scale']==pytest.approx(2)
    assert sim['ate_translation_rmse_m']<1e-10
    assert sim['rpe_translation_rmse_m']<1e-10
    assert sim['rpe_rotation_rmse_rad']<1e-6
    assert rigid['global_scale']==1 and rigid['ate_translation_rmse_m']>.1
    assert sim['duration_s']==4 and sim['reference_distance_m']==pytest.approx(8)
    with pytest.raises(ExperimentError,match='CONTINUITY_UNVERIFIED'):
        trajectory_metrics(estimated,reference)
    # A scale jump is not repaired by per-segment alignment.
    estimated[-1]['localization_fields']['translation']=[10,10,10]
    assert trajectory_metrics(estimated,reference,'Sim3',continuity_verified=True)['ate_translation_rmse_m']>.1


def test_online_final_cannot_be_mixed():
    e,r=trajectories();e[0]['trajectory_variant']='online'
    with pytest.raises(ExperimentError,match='VARIANT_MIXED'):
        trajectory_metrics(e,r,continuity_verified=True)


def test_no_gt_unknown_tracking_and_explicit_reset_events(tmp_path):
    ds=CanonicalDataset(make_dataset(tmp_path/'data',count=7))
    unknown=behavior_metrics(ds.frames,[],{'wall_clock_s':1})
    assert unknown['ground_truth_status']=='NO_GROUND_TRUTH' and unknown['tracking_coverage'] is None
    assert unknown['lost_intervals'] is None and unknown['relocalization_count'] is None
    states=['INITIALIZING','TRACKING','LOST','LOST','RELOCALIZING','RESET','TRACKING']
    events=[dict(source_frame_id=i,tracking_state=s,raw_backend_state='TEST_FIXTURE_ONLY:'+s) for i,s in enumerate(states)]
    events[5]['map_event']='TEST_FIXTURE_ONLY-new-origin'
    metrics=behavior_metrics(ds.frames,[],{'wall_clock_s':1},events)
    assert metrics['tracking_coverage']==pytest.approx(2/7)
    assert metrics['initialization_frame']==1
    assert metrics['relocalization_count']==1 and len(metrics['reset_events'])==1
    assert metrics['lost_intervals']==[dict(first_frame=2,last_frame=3,frame_count=2)]
    assert len(metrics['map_change_events'])==1
    assert behavior_metrics(ds.frames,[],{},events[:2])['tracking_coverage'] is None


def test_transforms_are_derived_and_intrinsics_follow_resize(tmp_path):
    import cv2
    ds=CanonicalDataset(make_dataset(tmp_path/'data',count=3))
    c=read_json(ds.path.parent/'calibration.json')
    before=[file_hash(f.image_path) for f in ds.frames]
    config=dict(grayscale=True,scale=.5,blur_sigma=1,brightness_gain=.8,brightness_offset=4)
    images,derived,provenance=prepare_images(ds.frames,c,tmp_path/'inputs',config)
    assert cv2.imread(str(images[0]),cv2.IMREAD_UNCHANGED).shape==(24,32)
    assert derived['fx']==28.5 and derived['cx']==15.5
    assert derived['distortion_coefficients']==c['distortion_coefficients']
    assert before==[file_hash(f.image_path) for f in ds.frames]
    assert provenance['config']==config and provenance['sensor_physics_validated'] is False
    assert all(r['encoded_pts']==f.encoded_pts for r,f in zip(provenance['frames'],ds.frames))


def test_mask_requires_explicit_validated_input(tmp_path):
    import cv2
    ds=CanonicalDataset(make_dataset(tmp_path/'data',count=3));c=read_json(ds.path.parent/'calibration.json')
    mask=tmp_path/'test-mask.png';image=np.full((48,64),255,np.uint8);image[:,:10]=0
    cv2.imwrite(str(mask),image)
    config={'mask':dict(file=str(mask),sha256=file_hash(mask),status='UNKNOWN')}
    with pytest.raises(ExperimentError,match='MASK_NOT_VALIDATED'):
        prepare_images(ds.frames,c,tmp_path/'bad',config)
    config['mask']['status']='VALIDATED'
    images,_,p=prepare_images(ds.frames,c,tmp_path/'good',config)
    assert np.all(cv2.imread(str(images[0]))[:,:10]==0)
    assert 'not backend feature exclusion' in p['mask_semantic']


@pytest.mark.parametrize('script,expected',[("print('TEST_FIXTURE_ONLY')",None),('raise SystemExit(7)','PROCESS_NONZERO_EXIT'),('import time;time.sleep(5)','TIMEOUT')])
def test_host_process_measurement_serializes(tmp_path,script,expected):
    result=execute_process([sys.executable,'-c',script],tmp_path,timeout_s=.2,sample_period_s=.02)
    assert result['failure_reason']==expected
    assert result['measurement_platform']=='HOST' and result['wall_clock_s']>0
    assert result['temperature_c'] is None and result['source_to_output_age_s'] is None
    assert result['oom']=='UNKNOWN'
    assert 'NaN' not in canonical(result)
    assert timing_stats([.001,.002,.003])['p50_s']==.002


def test_vio_contract_does_not_accept_attitude_or_guess_extrinsics():
    sample=dict(schema_version=1,purpose=LABEL,sequence=1,timestamp=TimestampEvidence(123).to_dict(),
                measurement_kind='RAW_IMU',gyro_xyz=[0.,0.,0.],accel_xyz=[0.,0.,9.8],
                gyro_unit='rad/s',accel_unit='m/s^2',device_id='TEST_FIXTURE_ONLY-imu',saturation=None)
    assert validate_imu_sample(sample) is sample
    sample['measurement_kind']='FC_ATTITUDE'
    with pytest.raises(ExperimentError):validate_imu_sample(sample)


def test_reference_evaluation_requires_bound_continuity(tmp_path):
    from slam_future_fixtures import run_config
    from rm27.perception.vision.localization.experiments.runner import run
    from rm27.perception.vision.localization.experiments.evaluation import evaluate_reference
    from rm27.perception.vision.localization.experiments.common import _TEST_FIXTURE_TOKEN
    config=run_config(tmp_path)
    result=run(config,_fixture_token=_TEST_FIXTURE_TOKEN)
    root=Path(config['output']);normalized=root/'normalized/final_optimized.json'
    records=read_json(normalized)['records']
    reference=dict(purpose=LABEL,reference_status='VERIFIED',timestamp_association='VERIFIED',
                   dataset_id=result['dataset_id'],dataset_manifest_sha256=result['comparison_identity']['dataset_manifest_sha256'],
                   translation_unit='m',records=records)
    write_json(tmp_path/'reference.json',reference)
    continuity=dict(purpose=LABEL,status='VERIFIED_SINGLE_COORDINATE_FRAME',run_config_sha256=result['config_sha256'],
                    normalized_sha256=file_hash(normalized),reference_sha256=file_hash(tmp_path/'reference.json'))
    write_json(tmp_path/'continuity.json',continuity)
    report=evaluate_reference(root/'run.json',tmp_path/'reference.json',tmp_path/'continuity.json',
                              'final_optimized','Sim3',_fixture_token=_TEST_FIXTURE_TOKEN)
    assert report['trajectory_metrics']['ate_translation_rmse_m']<1e-10
    with pytest.raises(ExperimentError,match='TRAJECTORY_UNAVAILABLE'):
        evaluate_reference(root/'run.json',tmp_path/'reference.json',tmp_path/'continuity.json',
                           'online','Sim3',_fixture_token=_TEST_FIXTURE_TOKEN)
    continuity['normalized_sha256']='0'*64;write_json(tmp_path/'continuity.json',continuity)
    with pytest.raises(ExperimentError,match='CONTINUITY_UNVERIFIED'):
        evaluate_reference(root/'run.json',tmp_path/'reference.json',tmp_path/'continuity.json',
                           'final_optimized','Sim3',_fixture_token=_TEST_FIXTURE_TOKEN)


def test_age_only_when_clocks_support_it():
    from rm27.perception.vision.localization.experiments.benchmark import source_to_output_age
    a=TimestampEvidence(1000,'us','board-steady','RECEIVE','VERIFIED').to_dict()
    b=TimestampEvidence(2,'ms','board-steady','PUBLISH','VERIFIED').to_dict()
    assert source_to_output_age(a,b)==pytest.approx(.001)
    b['clock_domain']='other-host'
    assert source_to_output_age(a,b) is None
    b['clock_domain']='board-steady';a['evidence_status']='UNVERIFIED'
    assert source_to_output_age(a,b) is None
