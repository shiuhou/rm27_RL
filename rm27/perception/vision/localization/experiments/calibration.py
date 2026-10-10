"""Fail-closed calibration gate for future real experiments; never fits intrinsics."""
from .common import checked_file, read_json, resolve, require, number, fixture_mode


def validated_calibration(dataset, *, _fixture_token=None):
    m = dataset.manifest
    require(m.get('status') != 'CALIBRATION_REQUIRED', 'CALIBRATION_REQUIRED', 'dataset still needs calibration')
    if m.get('dataset_kind') == 'TUM_RGBD_PUBLIC':
        from .public_tum import published_calibration
        return published_calibration(dataset)
    reference = m.get('calibration', {})
    require(reference.get('status') == 'VALIDATED', 'CALIBRATION_NOT_VALIDATED', 'dataset calibration reference')
    require(reference.get('file') and reference.get('sha256'), 'CALIBRATION_REFERENCE_INVALID', 'file and SHA-256 required')
    path, digest = checked_file(resolve(dataset.path.parent, reference['file']), reference['sha256'])
    calibration = read_json(path)
    fixture_mode(_fixture_token, m, calibration)
    require(type(calibration.get('schema_version')) is int and calibration['schema_version'] == 1,
            'CALIBRATION_SCHEMA_UNSUPPORTED', 'expected canonical calibration version 1')
    require(calibration.get('status') == 'VALIDATED', 'CALIBRATION_NOT_VALIDATED', 'artifact status')
    identifier = calibration.get('calibration_id')
    require(isinstance(identifier, str) and identifier not in ('', 'UNKNOWN') and identifier == reference.get('calibration_id'),
            'CALIBRATION_ID_MISMATCH', 'dataset and artifact must identify same calibration')
    require(all(f.calibration_id == identifier for f in dataset.frames), 'CALIBRATION_ID_MISMATCH', 'frame references')
    g = m['observed_geometry']
    require((calibration.get('image_width'), calibration.get('image_height')) == (g['width'], g['height']),
            'CALIBRATION_GEOMETRY_MISMATCH', 'calibration and dataset image dimensions')
    require(all(type(calibration[k]) is int and calibration[k] > 0 for k in ('image_width', 'image_height')),
            'CALIBRATION_INVALID', 'image dimensions')
    for k in ('fx', 'fy'):
        number(calibration.get(k), k, True)
    for k in ('cx', 'cy'):
        number(calibration.get(k), k)
    for key in ('camera_module', 'lens_id', 'focus_setting', 'camera_mode_id'):
        value = calibration.get(key)
        require(isinstance(value, str) and value not in ('', 'UNKNOWN'), 'CALIBRATION_PROVENANCE_MISSING', key)
        require(g.get(key) == value, 'CALIBRATION_CONFIGURATION_MISMATCH', key)
    require(calibration.get('pixel_geometry') == g.get('pixel_geometry') and isinstance(g.get('pixel_geometry'), dict),
            'CALIBRATION_CONFIGURATION_MISMATCH', 'crop/resize/pixel geometry')
    for k in ('crop', 'resize'):
        require(g['pixel_geometry'].get(k) not in (None, 'UNKNOWN'), 'CALIBRATION_PROVENANCE_MISSING', k)
    coefficients = calibration.get('distortion_coefficients')
    require(isinstance(coefficients, list), 'CALIBRATION_INVALID', 'distortion coefficients must be an ordered list')
    for value in coefficients:
        number(value, 'distortion coefficient')
    # A status string alone is not enough. Require an independently retained report.
    validation = calibration.get('validation', {})
    require(validation.get('status') == 'PASSED' and validation.get('criteria_id')
            and validation.get('report_file') and validation.get('report_sha256'),
            'CALIBRATION_VALIDATION_MISSING', 'validation report, criteria and hash required')
    report_path, report_hash = checked_file(resolve(path.parent, validation['report_file']), validation['report_sha256'])
    report = read_json(report_path)
    fixture_mode(_fixture_token, report)
    require(report.get('passed') is True and report.get('calibration_id') == identifier
            and report.get('criteria_id') == validation['criteria_id'],
            'CALIBRATION_VALIDATION_FAILED', 'retained validation report disagrees')
    return calibration, {'path': str(path), 'sha256': digest, 'calibration_id': identifier,
                         'validation_report': str(report_path), 'validation_report_sha256': report_hash}
