import hashlib
import json
from types import SimpleNamespace
from rm27.experiments.log_intake import inventory


def test_readonly_inventory_keeps_instances_and_parameter_changes(tmp_path, monkeypatch):
    source = tmp_path/'test.BIN'
    source.write_bytes(b'synthetic reader fixture')
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    rows = [('FMT', {}), ('PARM', {'Name': 'P', 'Value': 1}),
            ('PARM', {'Name': 'P', 'Value': 2}),
            ('IMU', {'I': 0, 'AccX': 1, 'TimeUS': 100}),
            ('IMU', {'I': 1, 'AccX': 5, 'TimeUS': 200})]
    messages = iter([SimpleNamespace(get_type=lambda k=k: k, to_dict=lambda d=d: d) for k, d in rows])
    monkeypatch.setattr('rm27.experiments.log_intake.DFReader.DFReader_binary',
                        lambda p: SimpleNamespace(recv_msg=lambda: next(messages, None)))
    result = inventory(source, tmp_path/'result', 'sitl')
    assert result['passed'] and result['source_sha256'] == original
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original
    assert result['field_ranges']['IMU[0].AccX']['maximum'] == 1
    assert result['field_ranges']['IMU[1].AccX']['maximum'] == 5
    assert len(result['parameter_changes']) == 1
    assert json.loads((tmp_path/'result/parameters.json').read_text())['P'] == 2
