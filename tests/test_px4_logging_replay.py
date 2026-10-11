import importlib.util
from pathlib import Path

import pytest

from rm27.telemetry.logging_replay import run

path=Path(__file__).resolve().parents[1]/'.maixpy/imu-design-20261010/single-uart/reassembly/test_ulog_mavlink.py'
spec=importlib.util.spec_from_file_location('saved_logging_fixture',path)
fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)


@pytest.mark.parametrize('drop',[None,1])
def test_saved_log_wire_replay_preserves_source_and_rejects_loss_without_partial_output(tmp_path,drop):
    source=tmp_path/'original.ulg';raw=fixtures.fixture();source.write_bytes(raw)
    output=tmp_path/'replay';result=run(source,output,drop_packet=drop)
    assert result['passed'] and result['source_unchanged']
    assert source.read_bytes()==raw
    assert result['hardware_connected'] is False
    assert result['upstream_source_complete']=='UNKNOWN'
    if drop is None:
        assert (output/'reassembled.ulg').read_bytes()==raw
        assert result['session']['retransmissions']==1
    else:
        assert result['session']['state']=='FAILED'
        assert not (output/'reassembled.ulg').exists()
