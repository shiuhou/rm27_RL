"""Synthetic IMU fixtures, never evidence of physical sensor performance."""
from copy import deepcopy

import pytest

from rm27.perception.vision.localization.experiments import vio
from rm27.perception.vision.localization.experiments.common import ExperimentError


def samples(count=64):
    return [dict(schema_version=1, sequence=i, measurement_kind='RAW_IMU',
                 device_id='TEST_FIXTURE_ONLY-imu', gyro_unit='rad/s', accel_unit='m/s^2',
                 gyro_xyz=[i % 2 * .002, 0., 0.], accel_xyz=[0., 0., 9.81], saturation=None,
                 timestamp=dict(raw_value=1400000000000000000+i*5000000, unit='ns',
                     clock_domain='TEST_FIXTURE_ONLY-clock', semantic='ACQUISITION',
                     evidence_status='VERIFIED')) for i in range(count)]


def test_sequence_rate_preserves_integer_epoch_precision():
    report = vio.validate_imu_sequence(samples())
    assert report['observed_rate_hz'] == pytest.approx(200)
    assert report['sequence_gaps'] == []
    assert report['saturation_unknown_count'] == 64
    assert report['missing_sample_count'] is None


@pytest.mark.parametrize('field,value', [('sequence',0), ('device_id','other')])
def test_sequence_identity_and_order_rejected(field, value):
    data = samples()
    data[1][field] = value
    with pytest.raises(ExperimentError):
        vio.validate_imu_sequence(data)


@pytest.mark.parametrize('field,value', [('raw_value',1400000000000000000),
    ('clock_domain','other'), ('unit','us'), ('semantic','RECEIPT')])
def test_clock_change_or_duplicate_rejected(field, value):
    data = samples()
    data[1]['timestamp'][field] = value
    with pytest.raises(ExperimentError):
        vio.validate_imu_sequence(data)


def test_gap_is_observation_not_invented_hardware_drop():
    data = samples()
    del data[5]
    data[2]['saturation'] = True
    result = vio.validate_imu_sequence(data)
    assert result['sequence_gaps'][0]['unrepresented_sequence_values'] == 1
    assert result['saturation_true_count'] == 1
    assert result['max_interval_s'] == pytest.approx(.01)
    assert result['missing_sample_count'] is None


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, '0'])
def test_nonfinite_and_non_numeric_rejected(value):
    data = samples()
    data[3]['gyro_xyz'][0] = value
    with pytest.raises(ExperimentError):
        vio.validate_imu_sequence(data)


def test_unknown_clock_allows_audit_not_noise_estimation():
    data = samples()
    for row in data:
        row['timestamp']['clock_domain'] = 'UNKNOWN'
    assert vio.validate_imu_sequence(data)['clock_qualified'] is False
    with pytest.raises(ExperimentError):
        vio.analyze_static_imu(data, stationary_confirmed=True)


def test_noise_analysis_requires_stationary_confirmation():
    with pytest.raises(ExperimentError):
        vio.analyze_static_imu(samples(), stationary_confirmed=False)


def test_static_statistics_are_not_calibrated_noise_or_accel_bias():
    report = vio.analyze_static_imu(samples(), stationary_confirmed=True)
    assert report['noise_parameters'] is None
    assert report['accel_mean_m_s2'][2] == pytest.approx(9.81)
    assert report['gyro_sample_std_rad_s'][0] > 0
    assert len(report['overlapping_allan_deviation']) > 0
    assert report['overlapping_allan_deviation'][0]['gyro_rad_s'][1] == 0
    assert report['overlapping_allan_deviation'][0]['gyro_rad_s'][0] == pytest.approx(.002/2**.5)


def test_receipt_clock_is_not_acquisition_clock_for_allan_tau():
    data = samples()
    for row in data:
        row['timestamp']['semantic'] = 'RECEIPT'
    with pytest.raises(ExperimentError):
        vio.analyze_static_imu(data, stationary_confirmed=True)


def test_jitter_is_rejected_for_uniform_sample_allan_estimator():
    data = samples()
    data[5]['timestamp']['raw_value'] += 100000
    with pytest.raises(ExperimentError):
        vio.analyze_static_imu(data, stationary_confirmed=True)


def test_cli_preserves_input_and_refuses_report_overwrite(tmp_path, monkeypatch):
    import json
    import sys
    from rm27.perception.vision.localization.experiments.common import file_hash
    source = tmp_path/'imu.jsonl'
    source.write_text('\n'.join(json.dumps(s) for s in samples()))
    digest = file_hash(source)
    output = tmp_path/'audit.json'
    monkeypatch.setattr(sys, 'argv', ['vio', '--samples', str(source), '--output', str(output)])
    vio.main()
    assert json.loads(output.read_text())['source_sha256'] == digest
    assert file_hash(source) == digest
    with pytest.raises(FileExistsError):
        vio.main()


@pytest.mark.parametrize('mode', ['gap','saturation','short'])
def test_noise_analysis_rejects_unqualified_capture(mode):
    data = deepcopy(samples())
    if mode == 'gap':
        del data[5]
    elif mode == 'saturation':
        data[5]['saturation'] = True
    else:
        data = data[:3]
    with pytest.raises(ExperimentError):
        vio.analyze_static_imu(data, stationary_confirmed=True)
