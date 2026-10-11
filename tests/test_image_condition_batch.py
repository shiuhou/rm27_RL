from rm27.experiments.image_condition_batch import report


def test_complete_gate_counts_safe_termination_separately_and_retains_all_failures():
    spec=dict(cases=list(range(50)),minimum_task_successes=48,gate_version='TEST',scope='test')
    good=dict(passed=True,task_success=True,safe_termination=False,category='varied')
    safe=dict(passed=True,task_success=False,safe_termination=True,category='varied')
    attempts=[good]*48+[safe]*2
    result=report(attempts,spec)
    assert result['passed'] and result['task_successes']==48 and result['safe_terminations']==2
    assert not report(attempts[:-1],spec)['passed']
    assert not report([good]*47+[safe]*3,spec)['passed']
    assert not report([good]*48+[safe,dict(safe,passed=False)],spec)['passed']
