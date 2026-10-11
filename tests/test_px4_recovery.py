"""Regression cases that prevent false acceptance of a recorded recovery."""
import json

import pytest

from rm27.autopilot.px4.recovery import evaluate_rows, evaluate_batch

IMPACT = dict(time_s=1., position_world_m=[0, 0, 1], armor_normal_speed_m_s=.1)


def rows(unsafe=(), contacts=(), stop=4.1):
    result = []
    for index in range(round((stop-1.)/.01)+1):
        t = round(1. + index*.01, 6)
        result.append(dict(timestamp=t, truth=dict(position_nwu=[0,0,1], velocity_nwu=[
            .4 if any(a<=t<=b for a,b in unsafe) else 0., 0, 0], body_rates_flu=[0,0,0],
            quaternion_flu_to_nwu=[1,0,0,0]), contacts=[dict(geoms=['drone_1_guard_1','rm27_base_armor'])]
            if index == 0 or any(abs(t-c)<1e-6 for c in contacts) else []))
    return result


def test_hold_restarts_after_an_unsafe_sample():
    r = evaluate_rows(rows(unsafe=[(1.8,2.9)]), IMPACT)
    assert r['passed']
    assert r['recovery_entry_time_s'] == pytest.approx(2.91)
    assert r['recovery_confirmation_time_s'] == pytest.approx(3.91)


def test_contiguous_hold_must_verify_each_sample():
    r = evaluate_rows(rows(unsafe=[(1.8,2.9)], stop=3.5), IMPACT)
    assert not r['passed']
    assert 'recovery_hold_not_confirmed' in r['failures']


def test_recontact_after_a_gap_fails_even_immediately_after_impact():
    r = evaluate_rows(rows(contacts=[1.01,1.02,1.10]), IMPACT)
    assert not r['passed'] and len(r['secondary_contacts']) == 1
    assert r['secondary_contacts'][0]['timestamp'] == 1.10


def test_recontact_after_recovery_is_still_checked():
    r = evaluate_rows(rows(contacts=[3.8]), IMPACT)
    assert not r['passed'] and 'secondary_collision' in r['failures']


def test_missing_impact_contact_cannot_pass():
    data=rows()
    data[0]['contacts']=[]
    assert not evaluate_rows(data,IMPACT)['passed']


def test_partial_batch_does_not_reduce_required_count(tmp_path):
    source = tmp_path/'source';source.mkdir()
    (source/'summary.json').write_text(json.dumps(dict(required_count=20,attempts=[],passed=False)))
    r=evaluate_batch(source,tmp_path/'score')
    assert r['required_count']==20 and not r['passed']
