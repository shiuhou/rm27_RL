import json

import pytest

from tools.px4.resume_image_batch import completed_stdout, verify_frozen
from rm27.autopilot.px4.return_fault_batch import digest


def test_resume_rejects_frozen_input_drift_and_missing_file(tmp_path):
    source = tmp_path / 'input.json'
    source.write_text('original')
    hashes = {str(source): digest(source)}
    verify_frozen(hashes)
    source.write_text('changed')
    with pytest.raises(ValueError, match='Frozen inputs changed'):
        verify_frozen(hashes)
    source.unlink()
    with pytest.raises(ValueError, match='Frozen inputs changed'):
        verify_frozen(hashes)


def test_completed_flight_requires_matching_final_fields_and_complete_stdout(tmp_path):
    log = tmp_path / 'flight.log'
    base = dict(passed=True, events=[{'event': 'complete'}], phase='POST',
                auto_disarm={'confirmed': True}, failure=None, gate_version='BASE')
    saved = dict(base, gate_version='IMAGE', independent_recovery_score={'passed': True})
    log.write_text('startup\n' + json.dumps(base) + '\n')
    assert completed_stdout(log, saved)
    assert not completed_stdout(log, dict(saved, auto_disarm={'confirmed': False}))
    log.write_text(json.dumps(base)[:-4])
    assert not completed_stdout(log, saved)
    log.write_text(json.dumps(base) + '\nunfinished diagnostic')
    assert not completed_stdout(log, saved)
