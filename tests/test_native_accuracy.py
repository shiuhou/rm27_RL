"""Synthetic evaluator tests, not evidence of real estimator accuracy."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest


def module():
    path = Path(__file__).resolve().parents[1] / 'tools/openvins/native_accuracy.py'
    assert path.exists(), 'native accuracy implementation is missing'
    spec = importlib.util.spec_from_file_location('native_accuracy', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_integer_nanosecond_association():
    m = module()
    base = 1403715273262142976
    assert m.associate([base, base+10], [base+1, base+9], 1) == [0, 1]
    with pytest.raises(ValueError):
        m.associate([base, base+10], [base+2], 1)
    with pytest.raises(ValueError):
        m.associate([base, base+10], [base, base+1], 1)


@pytest.mark.parametrize('times', [[1, 1], [2, 1]])
def test_reject_bad_order(times):
    with pytest.raises(ValueError):
        module().associate(times, [1], 1)


def test_interpolation_overlap_gap_and_sign():
    m = module()
    base = 1403715273262142976
    p = [[0, 0, 0], [2, 0, 0]]
    q = [[1, 0, 0, 0], [-1, 0, 0, 0]]
    result = m.interpolate([base, base+10], p, q, [base-1, base+5, base+11], 10)
    assert result['indices'] == [1]
    np.testing.assert_allclose(result['positions'], [[1, 0, 0]])
    np.testing.assert_allclose(result['quaternions'], [[1, 0, 0, 0]])
    with pytest.raises(ValueError):
        m.interpolate([base, base+10], p, q, [base+5], 9)


def test_slerp_rotation_direction():
    m = module()
    out = m.interpolate([0, 10], [[0,0,0]]*2, [[1,0,0,0],[0,0,0,1]], [5], 10)
    r = m.rotations(out['quaternions'])[0]
    np.testing.assert_allclose(r @ [1,0,0], [0,1,0], atol=1e-12)
    np.testing.assert_allclose(r.T @ [0,1,0], [1,0,0], atol=1e-12)


def test_rigid_alignment_and_no_scale_correction():
    m = module()
    p = np.array([[0,0,0], [1,0,0], [1,2,0], [0,2,1]], float)
    q = np.tile([1.,0,0,0], (4,1))
    r = m.rotations([[2**-.5,0,0,2**-.5]])[0]
    transformed = p @ r.T + [3,4,5]
    out = m.metrics([0, 500000000, 1000000000, 1500000000], p, q, transformed,
                    np.tile([2**-.5,0,0,2**-.5], (4,1)))
    assert out['ate']['rmse_m'] < 1e-12
    assert out['rpe_1s']['translation_rmse_m'] < 1e-12
    assert out['rpe_1s']['rotation_rmse_deg'] < 1e-5
    scaled = m.metrics([0, 500000000, 1000000000, 1500000000], p*2, q, p, q)
    assert scaled['alignment']['scale'] == 1.0
    assert scaled['ate']['rmse_m'] > .1
    assert abs(scaled['scale_diagnostic']['path_length_ratio_est_over_ref'] - 2) < 1e-12
    assert abs(scaled['scale_diagnostic']['centered_extent_ratio_est_over_ref'] - 2) < 1e-12


@pytest.mark.parametrize('q', [[[0,0,0,0]], [[float('nan'),0,0,0]], [[2,0,0,0]]])
def test_invalid_quaternion(q):
    with pytest.raises(ValueError):
        module().rotations(q)


def test_degenerate_alignment():
    with pytest.raises(ValueError):
        module().metrics([0,1,2], [[0,0,0]]*3, [[1,0,0,0]]*3,
                         [[0,0,0]]*3, [[1,0,0,0]]*3)


def test_reference_normalization_is_explicit_and_bounded():
    m = module()
    q, report = m.prepare_reference_quaternions([[1.000021,0,0,0]])
    np.testing.assert_allclose(q, [[1,0,0,0]])
    assert report['max_abs_norm_deviation'] > 2e-5
    with pytest.raises(ValueError):
        m.prepare_reference_quaternions([[1.01,0,0,0]])
