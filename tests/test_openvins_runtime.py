"""Synthetic boundary tests; the full-sequence acceptance is a separate run."""
import importlib
import importlib.util
import math
import pytest


def module():
    name = 'rm27.perception.vision.localization.experiments.openvins_runtime'
    assert importlib.util.find_spec(name), 'runtime adapter missing'
    return importlib.import_module(name)


def test_velocity_frame_conversion():
    m = module()
    assert m.world_velocity([0,0,math.sqrt(.5),math.sqrt(.5)], [1,0,0]) == pytest.approx([0,1,0])


def test_wire_pose_convention_unknowns():
    p = module().wire_pose([1,2,3], [0,0,0,1], 1403715273262142976)
    assert p['orientation_wxyz'] == [1,0,0,0]
    assert p['source_time']['raw_value'] == 1403715273262142976
    assert p['child_frame'] == 'imu' and p['translation_unit'] == 'm'
    assert 'tracking_state' not in p and 'publish_time' not in p


def test_order_is_per_stream_and_strict():
    s = module().StreamOrder()
    s.push('camera', 10)
    s.push('imu', 5)
    s.push('camera', 11)
    with pytest.raises(ValueError):
        s.push('camera', 11)
    with pytest.raises(ValueError):
        s.push('imu', 4)


@pytest.mark.parametrize('t', [1.2, True, -1])
def test_invalid_stamp(t):
    with pytest.raises(ValueError):
        module().StreamOrder().push('imu', t)


def test_state_text_integer_precision_and_world_velocity():
    state = module().state_row('1403715273.26214 0 0 0 1 1 2 3 .1 .2 .3 0 0 0 0 0 0 .0001000 1')
    assert state['state_ns'] == 1403715273262140000
    assert state['image_ns'] == 1403715273262040000
    assert state['velocity_world'] == [.1,.2,.3]


@pytest.mark.parametrize('q,v', [([0,0,0,2],[0,0,0]), ([0,0,0,1],[float('nan'),0,0])])
def test_invalid_velocity_or_quaternion(q,v):
    with pytest.raises(ValueError):
        module().world_velocity(q,v)


def test_same_run_metric_parity_gate():
    m = module()
    baseline = dict(ate={'rmse_m':.07},rpe_1s={'translation_rmse_m':.045,'rotation_rmse_deg':.478},
                    scale_diagnostic={'path_length_ratio_est_over_ref':.98})
    assert m.metric_consistency(baseline,baseline)['pass']
    bad = {**baseline,'ate':{'rmse_m':.2}}
    assert not m.metric_consistency(bad,baseline)['pass']


def test_runtime_dispatch_rejects_fixture():
    from rm27.perception.vision.localization.experiments.runner import run
    # Must reach the dedicated VIO validation, not old monocular schema checks.
    result = run(dict(schema_version=1,backend='OpenVINS',run_id='test',purpose='TEST_FIXTURE_ONLY'))
    assert result['failure_code'] == 'VIO_REAL_RUN_REQUIRED'


def test_wrapper_preserves_ros_python_environment():
    from pathlib import Path
    script = (Path(__file__).resolve().parents[1]/'tools/openvins/run_adapter_ros2.sh').read_text()
    assert 'export PYTHONPATH=/repo:${PYTHONPATH' in script


def test_existing_output_is_never_modified(tmp_path):
    p = tmp_path/'old-run'
    p.mkdir()
    marker = p/'run.json'
    marker.write_text('original evidence')
    with pytest.raises(FileExistsError):
        module().create_output(p)
    assert marker.read_text() == 'original evidence'


def test_new_output_has_raw_directory(tmp_path):
    p = module().create_output(tmp_path/'new-run')
    assert (p/'raw').is_dir()


def test_transport_scope_rejects_other_sequences(tmp_path):
    import json
    bag=tmp_path/'bag'
    bag.mkdir()
    (bag/'source_verification.json').write_text(json.dumps(dict(
        readback_exact=True,truth_included=False,source_sha256='0'*64)))
    with pytest.raises(ValueError,match='VIO_SEQUENCE_UNSUPPORTED'):
        module().EurocVioDataset(tmp_path/'missing.zip',bag)


def test_coverage_cannot_pass_on_just_similar_errors():
    base = dict(raw_pose_count=2800,evaluated_pose_count=2780,evaluated_duration_s=138.95,
        association={'first_camera_index':112,'last_camera_index':2911},
        alignment={'type':'SE3','scale':1.0},gt_csv_sha256='reference')
    assert module().coverage_consistency(base,base)['pass']
    assert not module().coverage_consistency({**base,'evaluated_pose_count':2000},base)['pass']
    assert not module().coverage_consistency({**base,'alignment':{'type':'Sim3','scale':1.01}},base)['pass']


def test_observer_buffers_imu_bursts_and_requires_odom_readiness():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]/'tools/openvins'
    assert 'QoSProfile(depth=10000' in (root/'observe_ros2.py').read_text()
    assert 'ros2 topic info /odomimu' in (root/'run_adapter_ros2.sh').read_text()


def test_raw_stream_reconciliation_rejects_changed_velocity():
    row=dict(state_ns=10,position=[1,2,3],quaternion_xyzw=[0,0,0,1],velocity_local=[1,0,0])
    module().check_output_row(row,row,True)
    with pytest.raises(ValueError):
        module().check_output_row(row,{**row,'velocity_local':[0,1,0]},True)
