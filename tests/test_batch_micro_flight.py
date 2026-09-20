import json
from rm27.experiments.batch_micro_flight import audit_trials


def test_batch_rejects_changed_inputs_and_failed_result(tmp_path):
    trials = []
    meta = dict(scene_sha256='a', binary_sha256='b', runner_sha256='c', parameter_files={'p': 'x'})
    for i in range(2):
        folder = tmp_path/str(i)
        folder.mkdir()
        (folder/'metadata.json').write_text(json.dumps(meta))
        (folder/'summary.json').write_text('{"passed": true}')
        trials.append(dict(directory=str(folder), index=i+1, returncode=0))
    assert audit_trials(trials)['passed']
    meta['binary_sha256'] = 'changed'
    (tmp_path/'1/metadata.json').write_text(json.dumps(meta))
    assert not audit_trials(trials)['passed']
    (tmp_path/'0/summary.json').write_text('{"passed": false}')
    assert len(audit_trials(trials)['failures']) == 2
