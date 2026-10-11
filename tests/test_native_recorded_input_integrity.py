import json

import pytest

from rm27.perception.sitl.dataset import digest
from tools.openvins.replay_native_inputs import load_inputs


def test_recorded_native_replay_preserves_independent_acquisition_and_delivery(tmp_path):
    images = tmp_path / 'images'
    images.mkdir()
    image = images / '1.png'
    image.write_bytes(b'original pixels')
    rows = [dict(type='imu', source_time_ns=100),
            dict(type='camera', source_time_ns=50, received_time_ns=100,
                 filename=image.name, sha256=digest(image))]
    inputs = tmp_path / 'inputs.jsonl'
    inputs.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    loaded, _ = load_inputs(tmp_path)
    assert loaded[1]['source_time_ns'] == 50
    image.write_bytes(b'changed pixels')
    with pytest.raises(ValueError, match='Recorded image changed'):
        load_inputs(tmp_path)


def test_replay_rejects_sensor_reversal_and_impossible_delivery(tmp_path):
    inputs = tmp_path / 'inputs.jsonl'
    rows = [dict(type='imu', source_time_ns=100), dict(type='imu', source_time_ns=100)]
    inputs.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    with pytest.raises(ValueError, match='Non-increasing'):
        load_inputs(tmp_path)
    inputs.write_text(json.dumps(dict(type='camera', source_time_ns=100, received_time_ns=99))+'\n')
    with pytest.raises(ValueError, match='delivery schedule'):
        load_inputs(tmp_path)
