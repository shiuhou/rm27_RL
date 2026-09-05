import numpy as np
from astra.scoring import summarize, paired_difference


def test_confidence_intervals_include_all_failures():
    rows = [dict(success=False, miss_m=2., reason='never_detected', contact_time_s=None)]*10
    result = summarize(rows)
    assert result['n'] == 10
    assert result['success_rate'] == 0
    assert result['success_ci95'][1] > 0
    assert result['miss_p95_m'] == 2


def test_paired_estimate_uses_episode_pairs():
    a = [dict(episode=i, seed=1, success=s) for i,s in enumerate([True, False, False, True])]
    b = [dict(episode=i, seed=1, success=s) for i,s in enumerate([True, True, True, True])]
    result = paired_difference(a, b)
    assert result['difference_b_minus_a'] == .5
    assert result['b_only_successes'] == 2
