import json
from pathlib import Path

import pytest

from slam_future_fixtures import run_config, make_dataset, LABEL
from rm27.perception.vision.localization.experiments.backends import BackendAdapter,camera_config
from rm27.perception.vision.localization.experiments.common import ExperimentError, read_json, _TEST_FIXTURE_TOKEN, file_hash, write_json
from rm27.perception.vision.localization.experiments.runner import run
from rm27.perception.vision.localization.experiments.compare import compare_runs, markdown


@pytest.mark.parametrize('backend',['stella_vslam','ORB-SLAM3'])
def test_missing_backend_is_explicit(backend):
    with pytest.raises(ExperimentError,match='BACKEND_NOT_INSTALLED'):
        BackendAdapter(backend).describe_version()


@pytest.mark.parametrize('backend',['stella_vslam','ORB-SLAM3'])
def test_config_conversion_preserves_calibration(tmp_path,backend):
    path=make_dataset(tmp_path/'data');c=read_json(path.parent/'calibration.json')
    text=camera_config(backend,c,{'sha256':'ab'*32},30)
    assert c['calibration_id'] in text and 'ab'*32 in text
    for value in ['57.0','55.0','31.5','23.5','0.0001','64','48']:
        assert value in text
    if backend=='stella_vslam':assert 'setup: "monocular"' in text
    else:assert 'Camera.type: "PinHole"' in text and 'Camera.RGB: 0' in text
    assert text==camera_config(backend,c,{'sha256':'ab'*32},30)
    c['model']='fisheye'
    with pytest.raises(ExperimentError,match='BACKEND_CALIBRATION_UNSUPPORTED'):
        camera_config(backend,c,{'sha256':'ab'*32},30)


@pytest.mark.parametrize('backend',['stella_vslam','ORB-SLAM3'])
def test_mock_process_end_to_end(tmp_path,backend):
    config=run_config(tmp_path,backend)
    result=run(config,_fixture_token=_TEST_FIXTURE_TOKEN)
    assert result['status']=='EXECUTION_COMPLETED',result
    assert result['purpose']==LABEL and result['stage_result']=='NOT_QUALIFIED'
    out=Path(config['output'])
    data=read_json(out/'normalized/final_optimized.json')
    assert len(data['records'])==6 and all(r['backend_tracking_state'] is None for r in data['records'])
    assert result['online_trajectory']=='UNAVAILABLE'
    assert result['evaluation']['tracking_coverage'] is None
    assert result['evaluation']['ground_truth_status']=='NO_GROUND_TRUTH'
    assert result['comparison_identity']['selected_source_frame_ids']==[0,3,6,9,12,15]
    assert (out/'raw/stdout.log').is_file()
    rgb=(out/'sequence/rgb.txt').read_text()
    if backend=='stella_vslam':
        assert '--viewer' in result['command'] and '--no-sleep' in result['command']
        assert (out/'sequence/depth.txt').read_text()==rgb
        assert read_json(out/'sequence/association_note.json')['depth_data_present'] is False
        assert result['evaluation']['processed_frame_count']==6
        assert all(r['processing_time_s']==.001 for r in data['records'])
    else:
        assert len(result['command'])==4
        assert result['evaluation']['processed_frame_count'] is None
        assert result['evaluation']['trajectory_scope']=='keyframes_only'
    # POSIX uses symlinks; Windows may use byte-identical copies when symlink
    # creation is unavailable to the current user.
    assert all(p.is_symlink() or p.is_file() for p in (out/'inputs').iterdir())


def test_runner_refuses_calibration_before_binary(tmp_path):
    config=run_config(tmp_path);m=read_json(config['dataset']);m['status']='CALIBRATION_REQUIRED';write_json(config['dataset'],m)
    result=run(config,_fixture_token=_TEST_FIXTURE_TOKEN)
    assert result['status']=='REFUSED' and result['failure_code']=='CALIBRATION_REQUIRED'
    assert not result['execution_started'] and not (Path(config['output'])/'raw').exists()


def test_normal_runner_cannot_accept_synthetic_config(tmp_path):
    with pytest.raises(ExperimentError,match='TEST_FIXTURE_FORBIDDEN'):
        run(run_config(tmp_path))


def test_comparison_determinism_and_mismatches(tmp_path):
    config=run_config(tmp_path)
    first=run(config,_fixture_token=_TEST_FIXTURE_TOKEN)
    second=json.loads(json.dumps(first));second.update(backend='ORB-SLAM3',run_id='other',status='FAILED',failure_reason='fixture failure')
    report=compare_runs([first,second])
    assert report==compare_runs([second,first])
    assert 'WINNER' not in markdown(report) and 'BEST' not in markdown(report)
    assert report['runs'][0]['status']=='FAILED'
    second['comparison_identity']['selected_source_frame_ids']=[0,1]
    with pytest.raises(ExperimentError,match='COMPARISON_INPUT_MISMATCH'):
        compare_runs([first,second])


def test_native_mask_unsupported(tmp_path):
    config=run_config(tmp_path);config['native_mask']={'file':'not-a-real-mask'}
    result=run(config,_fixture_token=_TEST_FIXTURE_TOKEN)
    assert result['failure_code']=='BACKEND_MASK_UNSUPPORTED' and not result['execution_started']


def test_absent_install_can_be_compared_after_input_gate(tmp_path):
    config=run_config(tmp_path);config['installation']=str(tmp_path/'missing-install.json')
    first=run(config,_fixture_token=_TEST_FIXTURE_TOKEN)
    assert first['failure_code']=='BACKEND_NOT_INSTALLED' and first['comparison_identity']
    second=json.loads(json.dumps(first));second.update(backend='ORB-SLAM3',run_id='other')
    assert len(compare_runs([first,second])['runs'])==2
