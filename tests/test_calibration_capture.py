import copy
import math

import cv2
import numpy as np
import pytest

from rm27.perception.vision.localization.capture_calibration import (
    DiversitySelector, assign_holdout, validate_target, make_detector, detect,
    observation_geometry, validate_session)
from rm27.perception.vision.localization.generate_calibration_target import chessboard_svg


def target():
    return dict(type='chessboard', corners_x=9, corners_y=6, square_size_m=.025,
                physical_target_id='SYNTHETIC-TEST-ONLY')


def corners(cx, cy, scale, roll=0, skew=0):
    points = np.array([(x-4, y-2.5) for y in range(6) for x in range(9)], dtype=float)
    points[:, 0] *= scale
    points[:, 1] *= scale
    points[:, 0] += skew * points[:, 1]
    rotation = np.array([[math.cos(roll), -math.sin(roll)], [math.sin(roll), math.cos(roll)]])
    return (points @ rotation.T + [cx, cy]).astype(np.float32).reshape(-1, 1, 2)


def test_can_collect_more_than_24_and_reject_identical():
    selector = DiversitySelector(50)
    records = []
    for scale in (13, 21, 34):
        for cy in (170, 380, 590):
            for cx in (200, 510, 820, 1130):
                geometry = observation_geometry(corners(cx, cy, scale), np.arange(54), target(), 1344, 760)
                accepted, reason = selector.consider(geometry['features'])
                assert accepted
                assert selector.consider(geometry['features']) == (False, 'similar_geometry')
                records.append(dict(accepted=True, geometry=geometry))
    assert len(selector.features) == 36
    assign_holdout(records)
    assert sum(r['split'] == 'validation' for r in records) == 8
    original = copy.deepcopy(records)
    assign_holdout(records)
    assert records == original
    held = [r for r in records if r['split'] == 'validation']
    assert len({round(r['geometry']['area_fraction'], 3) for r in held}) >= 2
    assert len({round(r['geometry']['centroid_normalized'][0], 1) for r in held}) >= 3


def test_tilt_roll_are_diversity_dimensions():
    selector = DiversitySelector()
    for roll, skew in [(0, 0), (.45, 0), (0, .9)]:
        geometry = observation_geometry(corners(672, 380, 35, roll, skew), np.arange(54), target(), 1344, 760)
        assert selector.consider(geometry['features'])[0]


@pytest.mark.parametrize('n', [1, 4, 24, 30, 31, 49, 50])
def test_holdout_rounds_up(n):
    records = [dict(accepted=True, geometry={'features': [i]*12}) for i in range(n)]
    records.append(dict(accepted=False))
    assign_holdout(records)
    assert sum(r['split'] == 'validation' for r in records) == math.ceil(n*.2)
    assert records[-1]['split'] is None


@pytest.mark.parametrize('dimension', [0, -1, float('nan'), float('inf'), True, '0.025'])
def test_invalid_physical_dimensions(dimension):
    data = target()
    data['square_size_m'] = dimension
    with pytest.raises(ValueError):
        validate_target(data)


@pytest.mark.parametrize('changes', [dict(type='aprilgrid'), dict(corners_x=True), dict(corners_x=0),
                                     dict(physical_target_id='REQUIRED_OPERATOR_VALUE')])
def test_target_supported_types_and_counts(changes):
    data = target()
    data.update(changes)
    with pytest.raises(ValueError):
        validate_target(data)


def test_chessboard_actual_detection_and_generator():
    data = validate_target(target())
    # Synthetic board checks the detector only, never a camera model or calibration fit.
    board = np.full((450, 600), 255, np.uint8)
    for y in range(7):
        for x in range(10):
            if (x+y) % 2 == 0:
                board[50+y*50:50+(y+1)*50, 50+x*50:50+(x+1)*50] = 0
    points, ids = detect(board, data, make_detector(data))
    assert len(points) == 54 and ids.tolist() == list(range(54))
    svg = chessboard_svg()
    assert 'width="300.0mm"' in svg and svg.count('fill="black"') == 35


def test_symmetric_circles_grid_validation_and_detection():
    data = dict(type='circles_grid', pattern_cols=7, pattern_rows=7,
                center_distance_m=.03, physical_target_id='DOT-BOARD-01')
    validate_target(data)
    board = np.full((500, 500), 255, np.uint8)
    for y in range(7):
        for x in range(7):
            cv2.circle(board, (80 + x * 55, 80 + y * 55), 14, 0, -1)
    points, ids = detect(board, data, make_detector(data))
    assert points is not None and len(points) == 49
    assert ids.tolist() == list(range(49))
    observation_geometry(points, ids, data, 500, 500)


def test_charuco_detected_ids_and_invalid_marker():
    data = dict(type='charuco', squares_x=7, squares_y=5, square_length_m=.03,
                marker_length_m=.022, dictionary='DICT_5X5_100', physical_target_id='SYNTHETIC')
    validate_target(data)
    detector = make_detector(data)
    board = detector[0].generateImage((1000, 750), marginSize=30)
    points, ids = detect(board, data, detector)
    assert points is not None and len(ids) >= 20
    assert len(set(ids.reshape(-1).tolist())) == len(ids)
    observation_geometry(points, ids, data, 1000, 750)
    data['marker_length_m'] = .03
    with pytest.raises(ValueError):
        validate_target(data)


def test_partial_charuco_rejected():
    with pytest.raises(ValueError, match='coverage'):
        observation_geometry(corners(672, 380, 30)[:4], np.arange(4), target(), 1344, 760)


def test_session_preserves_operator_provenance():
    data = dict(platform='M3C', sensor='OS04A10', lens_id='operator-lens', focus_note='locked mark A',
                camera_mode_id='full180', width=1344, height=760, crop_resize='operator observed full180, no downstream resize',
                device_software_commit='test-only', timestamp_evidence=dict(unit='UNKNOWN', clock_domain='UNKNOWN',
                semantic='UNKNOWN', evidence_status='UNVERIFIED'), physical_measurements=dict(horizontal_span_m=.25,
                horizontal_square_count=10, vertical_span_m=.175, vertical_square_count=7))
    validate_session(data)
    assert data['lens_id'] == 'operator-lens' and data['focus_note'] == 'locked mark A'
    data['physical_measurements']['horizontal_span_m'] = float('nan')
    with pytest.raises(ValueError):
        validate_session(data)


@pytest.mark.parametrize('write_ok', [True, False])
def test_capture_manifest_and_write_failures(tmp_path, monkeypatch, write_ok):
    import json
    from rm27.perception.vision.localization import capture_calibration as capture
    source = tmp_path/'take.h264'
    source.write_bytes(b'SYNTHETIC TEST INPUT: NOT PHYSICAL CAMERA DATA')
    target_file = tmp_path/'target.json'
    target_file.write_text(json.dumps(target()))
    session_file = tmp_path/'session.json'
    session = dict(platform='M3C', sensor='OS04A10', lens_id='test-lens', focus_note='test-focus',
                   camera_mode_id='full180', width=1344, height=760, crop_resize='test-only',
                   device_software_commit='test-only', timestamp_evidence=dict(unit='UNKNOWN', clock_domain='UNKNOWN',
                   semantic='UNKNOWN', evidence_status='UNVERIFIED'), physical_measurements=dict(horizontal_span_m=.25,
                   horizontal_square_count=10, vertical_span_m=.175, vertical_square_count=7))
    session_file.write_text(json.dumps(session))
    geometry = [corners(cx, cy, scale) for scale in (13, 21, 34)
                for cy in (170, 380, 590) for cx in (200, 510, 820, 1130)]
    class Capture:
        index = 0
        def isOpened(self): return True
        def get(self, prop):
            return {cv2.CAP_PROP_FRAME_COUNT: 36, cv2.CAP_PROP_FRAME_WIDTH: 1344,
                    cv2.CAP_PROP_FRAME_HEIGHT: 760, cv2.CAP_PROP_POS_MSEC: self.index*100}[prop]
        def read(self):
            if self.index == 36: return False, None
            self.index += 1
            return True, np.full((760, 1344, 3), self.index, np.uint8)
        def release(self): pass
    stream = Capture()
    monkeypatch.setattr(capture.cv2, 'VideoCapture', lambda _: stream)
    monkeypatch.setattr(capture.cv2, 'Laplacian', lambda *a: np.array([0., 100.]))
    monkeypatch.setattr(capture, 'detect', lambda *a: (geometry[stream.index-1], np.arange(54)))
    if not write_ok:
        monkeypatch.setattr(capture.cv2, 'imwrite', lambda *a: False)
    out = tmp_path/'selected'
    result = capture.main(['--source', str(source), '--target', str(target_file), '--session', str(session_file),
                           '--out', str(out), '--step', '1'])
    manifest = json.loads((out/'capture_manifest.json').read_text())
    assert manifest['session']['lens_id'] == 'test-lens'
    assert manifest['session']['focus_note'] == 'test-focus'
    assert manifest['sources'][0]['sha256_before'] == manifest['sources'][0]['sha256_after']
    assert manifest['software']['script_sha256']
    if write_ok:
        assert result == 0
        assert manifest['summary']['accepted'] == 36
        assert manifest['summary']['validation'] == 8
        assert len(list((out/'images').glob('*.png'))) == 36
        assert all(r['corner_ids'] == list(range(54)) for r in manifest['records'])
    else:
        assert result == 1
        assert manifest['summary']['accepted'] == 0
        assert len(manifest['summary']['failures']) == 36
        assert all(r['reason'] == 'image_write_failed' for r in manifest['records'])
    assert manifest['summary']['calibration_exists'] is False
