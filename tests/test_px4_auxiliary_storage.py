import gzip
import json

import pytest

from tools.px4.archive_closed_auxiliary import archive_file, restore


def test_generated_ulog_roundtrip_preserves_exact_bytes(tmp_path):
    path = tmp_path/'sample.ulg'
    original = bytes(range(256))*1024
    path.write_bytes(original)
    result = archive_file(path)
    assert result['passed'] and not path.exists()
    assert gzip.decompress((tmp_path/'sample.ulg.gz').read_bytes()) == original
    manifest = tmp_path/'sample.ulg.archive.json'
    assert restore(manifest)['passed']
    assert path.read_bytes() == original


def test_corrupt_archive_cannot_create_a_restored_recording(tmp_path):
    path = tmp_path/'sample.ulg'
    path.write_bytes(b'original log evidence'*100)
    archive_file(path)
    (tmp_path/'sample.ulg.gz').write_bytes(b'corrupt')
    with pytest.raises(ValueError, match='Archive bytes changed'):
        restore(tmp_path/'sample.ulg.archive.json')
    assert not path.exists()


def test_restoration_never_overwrites_a_changed_existing_recording(tmp_path):
    path = tmp_path/'sample.jsonl'
    path.write_text(json.dumps({'timestamp': 1})+'\n')
    archive_file(path)
    path.write_text('different evidence\n')
    with pytest.raises(ValueError, match='Existing restored file differs'):
        restore(tmp_path/'sample.jsonl.archive.json')
    assert path.read_text() == 'different evidence\n'
