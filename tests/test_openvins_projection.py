"""Adapter boundary fixtures; not a native replay qualification."""
import importlib
import importlib.util
import math
import pytest
from rm27.perception.vision.localization.experiments.trajectory import pose_matrix


def projection(**overrides):
    name = 'rm27.perception.vision.localization.experiments.openvins'
    assert importlib.util.find_spec(name), 'OpenVINS projection missing'
    f = importlib.import_module(name).pose_observation
    args = dict(source_frame_id=112, image_ns=1403715278862142976,
                state_ns=1403715278862200000, recorder_ns=1791370000000000000,
                position=[1,2,3], quaternion_xyzw=[0,0,math.sqrt(.5),math.sqrt(.5)],
                run_id='test-run', epoch=0, binary_sha256='a'*64,
                raw_reference={'file':'online-output_0.mcap','topic':'/poseimu','ordinal':0})
    args.update(overrides)
    return f(**args)


def test_unknown_state_stays_incomplete():
    r = projection()
    assert r['contract'] == 'LocalizationEstimate'
    assert not r['contract_complete'] and not r['validity']
    assert r['tracking_state'] == r['initialization_state'] == 'UNKNOWN'
    assert r['publish_timestamp'] is None
    assert r['processing_completed_timestamp'] is None
    assert r['reset_event'] is None and r['map_id'] is None
    assert r['trajectory_variant'] == 'online'
    assert r['localization_epoch'] == 0
    assert 'velocity' not in r['localization_fields']


def test_quaternion_reorder_no_conjugation_and_metric_imu_frame():
    r = projection()
    p = r['localization_fields']
    assert p['child_frame'] == 'imu'
    assert p['translation_unit'] == 'm' and p['scale_state'] == 'METRIC'
    t = pose_matrix(r)
    assert t[1,0] == pytest.approx(1)
    assert t[0,1] == pytest.approx(-1)


def test_separate_clocks_no_latency_fabrication():
    r = projection()
    assert r['source_timestamp']['raw_value'] == 1403715278862142976
    assert r['backend_timestamp']['raw_value'] == 1403715278862200000
    assert r['observation_timestamp']['clock_domain'] != r['backend_timestamp']['clock_domain']
    assert r['processing_time_s'] is None


@pytest.mark.parametrize('kwargs', [dict(epoch=-1), dict(image_ns=1.5), dict(quaternion_xyzw=[0,0,0,2]), dict(position=[float('nan'),0,0])])
def test_invalid_evidence_rejected(kwargs):
    with pytest.raises(ValueError):
        projection(**kwargs)
