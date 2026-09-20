"""All trials remain in denominators; confidence intervals are descriptive."""
from collections import Counter
import numpy as np


def summarize(rows):
    n = len(rows)
    if not n:
        raise ValueError('empty campaign')
    wins = sum(r['success'] for r in rows)
    p = wins/n
    z = 1.95996398454
    center = (p+z*z/(2*n))/(1+z*z/n)
    half = z*np.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
    result = dict(n=n, successes=wins, success_rate=p, success_ci95=[max(0., center-half), min(1., center+half)],
                  miss_p50_m=float(np.median([r['miss_m'] for r in rows])),
                  miss_p95_m=float(np.quantile([r['miss_m'] for r in rows], .95)),
                  reasons=dict(Counter(r['reason'] for r in rows)))
    for field in ('contact_time_s', 'contact_speed_m_s', 'max_speed_m_s', 'max_acceleration_m_s2', 'max_tilt_deg',
                  'control_effort_m2_s3', 'tracking_rmse_m', 'invisible_fraction', 'stale_fraction'):
        values = [r[field] for r in rows if r.get(field) is not None]
        if values:
            result[field] = {'mean': float(np.mean(values)), 'p95': float(np.quantile(values, .95)), 'max': float(max(values))}
    return result


def paired_difference(a, b):
    aa = {(r['seed'], r['episode']): r['success'] for r in a}
    bb = {(r['seed'], r['episode']): r['success'] for r in b}
    if aa.keys() != bb.keys() or len(aa) != len(a) or len(bb) != len(b):
        raise ValueError('need identical unique episode pairs')
    delta = np.array([int(bb[k])-int(aa[k]) for k in aa])
    # Paired bootstrap resamples trajectories, not individual outcomes.
    rng = np.random.default_rng(9182)
    bootstrap = np.array([delta[rng.integers(0, len(delta), len(delta))].mean() for _ in range(2000)])
    return dict(difference_b_minus_a=float(delta.mean()), ci95=np.quantile(bootstrap, [.025, .975]).tolist(),
                a_only_successes=int((delta == -1).sum()), b_only_successes=int((delta == 1).sum()))
