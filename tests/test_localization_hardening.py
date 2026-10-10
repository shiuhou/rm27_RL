import copy
import json
import math

import pytest

from rm27.perception.vision.localization import ScaleState, LocalizationStreamValidator, TrackingState
from rm27.perception.vision.localization.manifest import (
    validate_manifest, validate_frame_records, validate_calibration_reference, ManifestError)
from test_localization_contract import estimate, stamp


@pytest.mark.parametrize('q', [(1, 0, 0, 0), (0.5, 0.5, 0.5, 0.5), (0, 0, 0, -1)])
def test_unit_quaternion(q):
    assert estimate(orientation_wxyz=q).orientation_wxyz == q


@pytest.mark.parametrize('q', [(2, 0, 0, 0), (0, 0, 0, 0), (1e308,)*4,
                              (float('nan'), 0, 0, 0), (float('inf'), 0, 0, 0),
                              (True, 0, 0, 0), ('1', 0, 0, 0)])
def test_malformed_quaternion_rejected(q):
    with pytest.raises(ValueError):
        estimate(orientation_wxyz=q)


@pytest.mark.parametrize('changes', [dict(schema_version=True), dict(schema_version=2.0),
    dict(translation=(True, 0, 0)), dict(translation=('1', 0, 0)), dict(localization_epoch=True),
    dict(velocity=(True, 0, 0), velocity_unit='UNKNOWN', velocity_frame='map')])
def test_numeric_bools_and_strings_rejected(changes):
    with pytest.raises(ValueError):
        estimate(**changes)


@pytest.mark.parametrize('quality', [{'score': float('nan')}, {'nested': [float('inf')]}, {'bad': object()}])
def test_quality_strict_json(quality):
    with pytest.raises(ValueError, match='strict JSON'):
        estimate(quality=quality)


def test_optional_quality_and_timestamp_claim_preserved():
    e = estimate(source_time=stamp(1, 'EXPOSURE'))
    assert 'quality' not in e.to_dict()
    assert json.loads(e.to_json())['source_time']['evidence_status'] == 'UNVERIFIED'
    validate_frame_records([{'timestamp': e.source_time.to_dict()}])


@pytest.mark.parametrize('scale,unit,velocity_unit', [('METRIC', 'm', 'm/s'),
    ('ARBITRARY', 'arbitrary', 'arbitrary/s'), ('UNKNOWN', 'UNKNOWN', 'UNKNOWN')])
def test_scale_units(scale, unit, velocity_unit):
    e = estimate(scale_state=scale, translation_unit=unit, velocity=(1, 2, 3),
                 velocity_frame='map', velocity_unit=velocity_unit)
    assert e.scale_state == ScaleState(scale)
    assert type(e).from_json(e.to_json()) == e
    with pytest.raises(ValueError):
        estimate(scale_state=scale, translation_unit='centimeters')


def test_state_and_epoch_semantics():
    stream = LocalizationStreamValidator('session-A', max_translation_step=2)
    stream.push(estimate())
    stream.push(estimate(translation=(2, 2, 3)))
    with pytest.raises(ValueError, match='jump'):
        stream.push(estimate(translation=(200, 2, 3)))
    with pytest.raises(ValueError, match='epoch'):
        stream.push(estimate(parent_frame='new-origin'))
    reset = dict(tracking_state=TrackingState.RESET, valid=False, initialized=False,
                 relocalizing=False, reset_reason='new origin')
    with pytest.raises(ValueError, match='epoch'):
        stream.push(estimate(**reset))
    stream.push(estimate(**reset, localization_epoch=5))
    stream.push(estimate(localization_epoch=5, translation=(200, 2, 3)))
    with pytest.raises(ValueError, match='decreased'):
        stream.push(estimate(localization_epoch=4))
    assert estimate(tracking_state=TrackingState.LOST, valid=True).valid  # retained pose
    assert not estimate(tracking_state=TrackingState.LOST, valid=False).valid
    with pytest.raises(ValueError):
        estimate(tracking_state=TrackingState.LOST, initialized=False)
    with pytest.raises(ValueError):
        estimate(relocalizing=True)
    with pytest.raises(ValueError):
        estimate(tracking_state=TrackingState.TRACKING, valid=False)
    assert estimate(tracking_state=TrackingState.RELOCALIZING, relocalizing=True).valid
    # New session is an explicit boundary; epoch zero is then representable.
    LocalizationStreamValidator('session-B').push(estimate(localization_epoch=0))


def test_calibration_required_and_claimed_missing(tmp_path):
    validate_manifest({'schema_version': 1, 'status': 'CALIBRATION_REQUIRED',
                       'calibration': {'calibration_id': 'UNKNOWN', 'file': None, 'sha256': None}})
    with pytest.raises(ManifestError, match='requires'):
        validate_calibration_reference({'calibration_id': 'measured', 'calibration_file': 'missing'}, tmp_path)
    with pytest.raises(ManifestError, match='missing calibration file'):
        validate_calibration_reference({'calibration_id': 'measured', 'file': 'missing', 'sha256': '0'*64}, tmp_path)


@pytest.mark.parametrize('fmt,width,height,stride,extra,valid', [
    ('RGB888', 640, 480, 1920, {}, True), ('RGB888', 640, 480, 640, {}, False),
    ('GRAY8', 640, 480, 640, {}, True), ('GRAY8', 640, 480, 639, {}, False),
    ('NV21', 640, 480, 672, {'uv_stride_bytes': 672, 'frame_bytes': 483840}, True),
    ('NV21', 641, 480, 672, {}, False), ('NV21', 640, 479, 672, {}, False),
    ('NV21', 640, 480, 671, {}, False),
    ('NV21', 640, 480, 672, {'frame_bytes': 322560}, False),
    ('NV21', 640, 480, 672, {'uv_stride_bytes': 640}, False),
    ('GRAY8', True, 480, 640, {}, False)])
def test_format_stride(fmt, width, height, stride, extra, valid):
    camera = dict(pixel_format=fmt, width=width, height=height, stride_bytes=stride, **extra)
    if valid:
        validate_manifest({'schema_version': 1, 'camera': camera})
    else:
        with pytest.raises(ManifestError):
            validate_manifest({'schema_version': 1, 'camera': camera})


def test_clock_ordering_requires_comparable_evidence():
    def frame(raw, domain='A', unit='us', semantic='RECEIVE', verified='VERIFIED'):
        return {'timestamp': dict(raw_value=raw, unit=unit, clock_domain=domain, semantic=semantic,
                                  evidence_status=verified, monotonic=True)}
    validate_frame_records([frame(100), frame(1, 'B')])
    validate_frame_records([frame(100, 'UNKNOWN'), frame(1, 'UNKNOWN')])
    validate_frame_records([frame(100, verified='UNVERIFIED'), frame(1, verified='UNVERIFIED')])
    validate_frame_records([frame(100), frame(1, semantic='PUBLISH')])
    validate_frame_records([frame(100), frame(1, unit='ms')])
    with pytest.raises(ManifestError, match='monotonic'):
        validate_frame_records([frame(100), frame(99)])
    with pytest.raises(ManifestError, match='monotonic'):
        validate_frame_records([frame(100, unit='ms'), frame(99, unit='us')])


def test_manifest_bool_version_rejected():
    with pytest.raises(ManifestError):
        validate_manifest({'schema_version': True})
