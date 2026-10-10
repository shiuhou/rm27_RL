"""Synthetic state/pose observations; real validation is separately retained."""
import csv
from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest

from slam_future_fixtures import make_dataset, run_config
from rm27.perception.vision.localization.experiments.common import ExperimentError, read_json, write_json, file_hash, _TEST_FIXTURE_TOKEN
from rm27.perception.vision.localization.experiments.dataset import CanonicalDataset
from rm27.perception.vision.localization.experiments.online import OnlinePoseRecord, normalized_state, parse_online, online_metrics, matrix_pose
from rm27.perception.vision.localization.experiments.runner import run
from rm27.perception.vision.localization.experiments.evaluation import evaluate_reference


def raw_csv(path, frames, states, present=None, indices=None):
    names=['input_index','backend_timestamp','processing_started_ns','processing_completed_ns',
           'observation_completed_ns','raw_state','pose_present']+[f't{i}{j}' for i in range(4) for j in range(4)]
    with path.open('w') as f:
        writer=csv.writer(f);writer.writerow(names)
        for n,state in enumerate(states):
            i=indices[n] if indices is not None else n
            has_pose=present[n] if present is not None else True
            writer.writerow([i,frames[i].encoded_pts,n*1000,n*1000+500,n*1000+600,state,int(has_pose)]
                            +(np.eye(4).ravel().tolist() if has_pose else ['']*16))
    return path


def parsed(tmp_path, states, backend='stella_vslam', present=None, count=None, indices=None):
    ds=CanonicalDataset(make_dataset(tmp_path/'data',count=count or len(states)))
    raw=raw_csv(tmp_path/'online.csv',ds.frames,states,present,indices)
    records=parse_online(raw,ds.frames,backend,dict(commit='TEST_FIXTURE_ONLY',binary_sha256='0'*64),'synthetic-run')
    return ds,records


def test_online_record_serialization_unknown_and_optional_fields():
    r=OnlinePoseRecord(source_frame_id=42,source_timestamp={'raw_value':'1.234','semantic':'DATASET_TIMESTAMP'},
       backend='stella_vslam',backend_version={'commit':'TEST_FIXTURE_ONLY'},raw_backend_state='future-state',
       tracking_state='UNKNOWN',initialization_state='UNKNOWN',validity=False,reason='NO_POSE_RETURNED',encoded_pts='1.234')
    assert OnlinePoseRecord.from_json(r.to_json())==r
    for k in ['map_id','localization_epoch','reset_event','relocalization_event','publish_timestamp','processing_time_s']:
        assert r.to_dict()[k] is None
    assert 'covariance' not in r.to_dict() and 'confidence' not in r.to_dict()
    with pytest.raises(ExperimentError,match='ONLINE_VALIDITY_INVALID'):
        replace(r,validity=True)
    with pytest.raises(ExperimentError,match='TRAJECTORY_VARIANT_MIXED'):
        replace(r,trajectory_variant='final_optimized')


@pytest.mark.parametrize('backend,raw,expected',[
 ('stella_vslam','Initializing','INITIALIZING'),('stella_vslam','Tracking','TRACKING'),
 ('stella_vslam','Lost','LOST'),('stella_vslam','','UNKNOWN'),
 ('ORB-SLAM3',-1,'UNINITIALIZED'),('ORB-SLAM3',0,'UNINITIALIZED'),('ORB-SLAM3',1,'INITIALIZING'),
 ('ORB-SLAM3',2,'TRACKING'),('ORB-SLAM3',3,'LOST'),('ORB-SLAM3',4,'LOST'),
 ('ORB-SLAM3',5,'UNKNOWN'),('ORB-SLAM3',99,'UNKNOWN')])
def test_mapping_only_exposed_states(backend,raw,expected):
    assert normalized_state(backend,raw)==expected


def test_pose_absence_never_fabricates_lost_or_relocalization(tmp_path):
    ds,records=parsed(tmp_path,['Initializing','Tracking','Lost','Tracking'],present=[False,True,False,True])
    m=online_metrics(ds.frames,records)
    assert m['tracking_coverage']==.5 and m['initialization_frame']==1
    assert m['emitted_online_pose_count']==2 and m['valid_online_pose_count']==2
    assert m['lost_intervals'][0]['first_frame']==2 and m['lost_duration_s']>0
    assert m['relocalization_events'] is None and m['reset_events'] is None
    assert records[0]['tracking_state']=='INITIALIZING' and records[2]['localization_fields'] is None
    assert records[2]['initialization_state']=='UNKNOWN'


def test_orb_initial_pose_is_not_valid_tracking(tmp_path):
    ds,records=parsed(tmp_path,['1','2','3','4'],'ORB-SLAM3')
    assert [r['validity'] for r in records]==[False,True,False,False]
    assert records[0]['localization_fields']['valid'] is False
    m=online_metrics(ds.frames,records)
    assert m['emitted_online_pose_count']==4 and m['valid_online_pose_count']==1
    assert m['lost_intervals'][0]['right_censored'] is True and m['lost_duration_s'] is None


def test_unknown_and_partial_trace_do_not_claim_complete_coverage(tmp_path):
    ds,records=parsed(tmp_path,['Tracking','unrecognized'],count=3,indices=[0,2])
    m=online_metrics(ds.frames,records)
    assert m['frame_coverage']==pytest.approx(2/3)
    assert m['state_evidence']=='PARTIAL' and m['tracking_coverage'] is None
    assert m['initialization_frame'] is None and m['lost_intervals'] is None
    assert m['tracking_state_transitions'][0]['adjacent_input'] is False


def test_source_identity_clock_semantics_and_host_durations(tmp_path):
    ds=CanonicalDataset(make_dataset(tmp_path/'data',count=2))
    f=replace(ds.frames[0],source_frame_id=37,encoded_pts='1305031102.175304',
              timestamp_evidence=dict(raw_value='1305031102.175304',unit='s',clock_domain='dataset',semantic='DATASET_TIMESTAMP'))
    p=raw_csv(tmp_path/'online.csv',[f],['Tracking'])
    r=parse_online(p,[f],'stella_vslam',dict(commit='test',binary_sha256='0'*64),'run-clock')[0]
    assert r['source_frame_id']==37 and r['source_timestamp']==f.timestamp_evidence
    assert r['encoded_pts']==f.encoded_pts and r['publish_timestamp'] is None
    assert r['processing_completed_timestamp']['clock_domain']=='run-clock:backend_process_steady_clock'
    assert r['processing_completed_timestamp']['semantic']=='PROCESSING_COMPLETION'
    assert r['processing_time_s']==pytest.approx(5e-7)
    assert r['backend_timestamp']['semantic']=='BACKEND_INPUT_TIMESTAMP'
    assert online_metrics([f],[r])['source_to_publish_latency_s'] is None
    p.write_text(p.read_text().replace('1305031102.175304','1305031103.175304'))
    with pytest.raises(ExperimentError,match='ONLINE_TIMESTAMP_MISMATCH'):
        parse_online(p,[f],'stella_vslam',dict(commit='test',binary_sha256='0'*64),'run-clock')


def test_api_pose_conventions_and_invalid_matrix():
    m=np.eye(4);m[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];m[:3,3]=[1,2,3]
    assert matrix_pose(m,'T_cw')['translation']==pytest.approx([-2,1,-3])
    assert matrix_pose(m,'T_wc')['translation']==[1,2,3]
    m[0,0]=2
    with pytest.raises(ExperimentError,match='POSE_INVALID'):matrix_pose(m,'T_wc')


def test_online_metrics_reject_final_records(tmp_path):
    ds,records=parsed(tmp_path,['Tracking','Tracking'])
    records[0]['trajectory_variant']='final_optimized'
    with pytest.raises(ExperimentError,match='TRAJECTORY_VARIANT_MIXED'):online_metrics(ds.frames,records)


def test_runner_refuses_uninstrumented_installation(tmp_path):
    c=run_config(tmp_path);c['online']=True
    r=run(c,_fixture_token=_TEST_FIXTURE_TOKEN)
    assert r['status']=='REFUSED' and r['failure_code']=='ONLINE_OBSERVATION_UNAVAILABLE'
    assert r['execution_started'] is False


def test_reference_evaluation_never_substitutes_final_for_online(tmp_path):
    c=run_config(tmp_path);r=run(c,_fixture_token=_TEST_FIXTURE_TOKEN)
    root=Path(c['output']);final=read_json(root/'normalized/final_optimized.json')
    ref=dict(purpose='TEST_FIXTURE_ONLY',reference_status='VERIFIED',timestamp_association='VERIFIED',
             dataset_id=r['dataset_id'],dataset_manifest_sha256=r['comparison_identity']['dataset_manifest_sha256'],
             translation_unit='m',records=final['records'])
    write_json(tmp_path/'reference.json',ref);write_json(tmp_path/'continuity.json',{'purpose':'TEST_FIXTURE_ONLY'})
    with pytest.raises(ExperimentError,match='TRAJECTORY_UNAVAILABLE'):
        evaluate_reference(root/'run.json',tmp_path/'reference.json',tmp_path/'continuity.json','online','Sim3',_fixture_token=_TEST_FIXTURE_TOKEN)
    # Relabeling a header cannot turn final records into online observations.
    final['trajectory_variant']='online';write_json(root/'false-online.json',final)
    r['online_trajectory']='false-online.json';write_json(root/'run.json',r)
    with pytest.raises(ExperimentError,match='TRAJECTORY_VARIANT_MIXED'):
        evaluate_reference(root/'run.json',tmp_path/'reference.json',tmp_path/'continuity.json','online','Sim3',_fixture_token=_TEST_FIXTURE_TOKEN)


def test_online_reference_filters_invalid_records_and_keeps_variant(tmp_path):
    c=run_config(tmp_path);r=run(c,_fixture_token=_TEST_FIXTURE_TOKEN)
    root=Path(c['output']);final=read_json(root/'normalized/final_optimized.json')
    records=[dict(x,trajectory_variant='online',validity=True) for x in final['records']]
    records[0].update(validity=False,localization_fields=None)
    write_json(root/'observed.json',dict(purpose='TEST_FIXTURE_ONLY',trajectory_variant='online',records=records))
    r['online_trajectory']='observed.json';write_json(root/'run.json',r)
    reference=dict(purpose='TEST_FIXTURE_ONLY',reference_status='VERIFIED',timestamp_association='VERIFIED',
      dataset_id=r['dataset_id'],dataset_manifest_sha256=r['comparison_identity']['dataset_manifest_sha256'],
      translation_unit='m',records=final['records'])
    write_json(tmp_path/'reference.json',reference)
    continuity=dict(purpose='TEST_FIXTURE_ONLY',status='VERIFIED_SINGLE_COORDINATE_FRAME',run_config_sha256=r['config_sha256'],
      normalized_sha256=file_hash(root/'observed.json'),reference_sha256=file_hash(tmp_path/'reference.json'))
    write_json(tmp_path/'continuity.json',continuity)
    e=evaluate_reference(root/'run.json',tmp_path/'reference.json',tmp_path/'continuity.json','online','Sim3',_fixture_token=_TEST_FIXTURE_TOKEN)
    assert e['trajectory_variant']=='online' and e['excluded_invalid_online_records']==1
    assert e['trajectory_metrics']['matched_frames']==len(records)-1
    assert e['trajectory_metrics']['ate_translation_rmse_m']<1e-8


@pytest.mark.parametrize('exit_code',[0,7])
def test_runner_preserves_raw_online_and_partial_failure_trace(tmp_path,exit_code):
    c=run_config(tmp_path);c['online']=True
    install=read_json(c['installation']);binary=Path(install['binary'])
    # Synthetic CSV emitted during fake processing; never real backend evidence.
    binary.write_text(binary.read_text()+'''
import csv
names=['input_index','backend_timestamp','processing_started_ns','processing_completed_ns','observation_completed_ns','raw_state','pose_present']+[f't{i}{j}' for i in range(4) for j in range(4)]
with (raw/'online.csv').open('w') as f:
    w=csv.writer(f);w.writerow(names)
    for i,row in enumerate(rows[:LIMIT]):
        w.writerow([i,row[0],i*1000,i*1000+500,i*1000+600,'Tracking',1]+[int(a==b) for a in range(4) for b in range(4)])
raise SystemExit(EXIT_CODE)
'''.replace('LIMIT','len(rows)' if exit_code==0 else '3').replace('EXIT_CODE',str(exit_code)))
    install['binary_sha256']=file_hash(binary)
    install['online_observation']=dict(format='rm27_online_csv_v1',source_files_sha256={str(binary):file_hash(binary)})
    write_json(c['installation'],install)
    r=run(c,_fixture_token=_TEST_FIXTURE_TOKEN);root=Path(c['output'])
    assert (root/'raw/online.csv').is_file()
    assert r['online_trajectory']=='online/trajectory.json'
    metrics=read_json(root/'online/evaluation.json')
    if exit_code:
        assert r['status']=='FAILED' and r['final_optimized_trajectory']=='UNAVAILABLE'
        assert metrics['observed_frame_count']==3 and metrics['tracking_coverage'] is None
    else:
        assert r['status']=='EXECUTION_COMPLETED' and metrics['frame_coverage']==1
        assert r['final_optimized_trajectory']!=r['online_trajectory']
