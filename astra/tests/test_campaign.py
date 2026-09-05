import numpy as np
from astra.scoring import summarize, paired_difference
from astra.campaign import run
import json
import hashlib


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


def test_runner_records_replayable_sources_and_all_trials(tmp_path):
    spec = dict(defaults={'duration_s': .1}, development_seed=7, development_n=4, methods=['pursuit','compensated_pursuit','lead'])
    out = tmp_path/'campaign'
    result = run(spec, out)
    assert len((out/'episodes.jsonl').read_text().splitlines()) == 12
    assert result['nominal/lead']['overall']['n'] == 4
    h = hashlib.sha256()
    for p in sorted((out/'source/astra').glob('*.py')):
        h.update(p.name.encode()); h.update(p.read_bytes())
    assert h.hexdigest() == json.loads((out/'manifest.json').read_text())['source_sha256']
