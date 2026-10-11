import gzip
import hashlib
import json

import pytest

from rm27.telemetry.recording import compress_recording,open_recording,recording_sha256


def test_archive_preserves_exact_bytes_hash_and_logical_reader(tmp_path):
    path=tmp_path/'frames.jsonl'
    data=(json.dumps({'message':'回庫','position':[1.2,3.4,5.6]},ensure_ascii=False)+'\n').encode()*100
    path.write_bytes(data)
    before=recording_sha256(path)
    record=compress_recording(path)
    assert record['passed'] and before==record['original_sha256']==hashlib.sha256(data).hexdigest()
    assert not path.exists() and recording_sha256(path)==before
    with open_recording(path,'rb') as stream:assert stream.read()==data
    with open_recording(path) as stream:assert len(list(stream))==100
    with pytest.raises(FileExistsError):compress_recording(path)
    with pytest.raises(ValueError):open_recording(path,'w')


def test_archive_failed_verification_retains_original(tmp_path,monkeypatch):
    import rm27.telemetry.recording as recording
    from io import BytesIO
    path=tmp_path/'frames.jsonl';path.write_bytes(b'{"truth":1}\n')
    monkeypatch.setattr(recording.gzip,'open',lambda *a,**kw:BytesIO(b'corrupted'))
    with pytest.raises(ValueError,match='verification'):compress_recording(path)
    assert path.read_bytes()==b'{"truth":1}\n'
    assert not list(tmp_path.glob('*.gz')) and not list(tmp_path.glob('*.archive.json'))


def test_existing_archive_is_not_overwritten(tmp_path):
    path=tmp_path/'frames.jsonl';path.write_bytes(b'current')
    compressed=tmp_path/'frames.jsonl.gz'
    compressed.write_bytes(gzip.compress(b'original evidence'))
    with pytest.raises(FileExistsError):compress_recording(path)
    assert gzip.decompress(compressed.read_bytes())==b'original evidence'
    assert path.read_bytes()==b'current'
