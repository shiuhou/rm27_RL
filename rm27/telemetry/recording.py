"""Lossless storage for completed generated JSONL recordings.

Readers accept the logical original path. Hashes refer to original bytes,
regardless of whether those bytes are stored directly or inside gzip.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import uuid


def open_recording(path,mode='rt'):
    if mode not in ('r','rt','rb'):
        raise ValueError('recording reader is read-only')
    path=Path(path)
    if path.exists():
        return path.open(mode)
    compressed=path.with_name(path.name+'.gz')
    return gzip.open(compressed,'rb' if mode=='rb' else 'rt')


def recording_sha256(path):
    result=hashlib.sha256()
    with open_recording(path,'rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):
            result.update(chunk)
    return result.hexdigest()


def compress_recording(path):
    """Verify decompression before replacing a completed generated recording."""
    path=Path(path)
    if path.suffix!='.jsonl':
        raise ValueError('only generated JSONL recordings may be archived')
    target=path.with_name(path.name+'.gz')
    manifest=path.with_name(path.name+'.archive.json')
    if target.exists() or manifest.exists():
        raise FileExistsError('archive already exists: '+str(target))
    initial=path.stat()
    temporary=target.with_name(target.name+'.tmp-'+uuid.uuid4().hex)
    original=hashlib.sha256()
    try:
        with path.open('rb') as source,temporary.open('xb') as file:
            with gzip.GzipFile(filename='',mode='wb',fileobj=file,mtime=0,compresslevel=6) as zipped:
                for chunk in iter(lambda:source.read(1024*1024),b''):
                    original.update(chunk);zipped.write(chunk)
            file.flush();os.fsync(file.fileno())
        # Verify both bytes and the absence of a concurrent writer. A failure
        # retains the original file and never creates an accepted manifest.
        verified=hashlib.sha256()
        with gzip.open(temporary,'rb') as source:
            for chunk in iter(lambda:source.read(1024*1024),b''):
                verified.update(chunk)
        latest=path.stat()
        if (verified.digest()!=original.digest() or
            (initial.st_ino,initial.st_size,initial.st_mtime_ns)!=
            (latest.st_ino,latest.st_size,latest.st_mtime_ns)):
            raise ValueError('archive verification or source immutability failed')
        os.link(temporary,target)  # Atomic creation; cannot overwrite evidence.
        compressed_hash=hashlib.sha256(target.read_bytes()).hexdigest()
        record=dict(passed=True,logical_path=path.name,storage_path=target.name,
            original_sha256=original.hexdigest(),compressed_sha256=compressed_hash,
            original_bytes=initial.st_size,compressed_bytes=target.stat().st_size,
            run_type='lossless storage, not a flight or acceptance result')
        with manifest.open('x') as stream:
            stream.write(json.dumps(record,indent=2)+'\n')
        path.unlink()
        return record
    finally:
        temporary.unlink(missing_ok=True)


def archive_runs(root,output):
    root=Path(root).resolve();output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    records=[]
    candidates=[root] if (root/'metadata.json').exists() else sorted(
        p.parent for p in root.rglob('metadata.json'))
    for run in candidates:
        metadata=json.loads((run/'metadata.json').read_text())
        summary_path=run/'summary.json'
        if not summary_path.exists() or summary_path.stat().st_size==0:
            raise ValueError('completed-run summary required: '+str(run))
        summary=json.loads(summary_path.read_text())
        if summary.get('autopilot')!='px4' or 'scene_sha256' not in metadata:
            raise ValueError('only completed generated PX4 runs are supported: '+str(run))
        path=run/'frames.jsonl'
        if not path.exists():continue
        record=compress_recording(path);record['run']=str(run);records.append(record)
        print(json.dumps(dict(run=str(run),saved_bytes=record['original_bytes']-record['compressed_bytes'])),flush=True)
    result=dict(passed=bool(records) and all(r['passed'] for r in records),
        files=len(records),saved_bytes=sum(r['original_bytes']-r['compressed_bytes'] for r in records),
        input_root=str(root),records=records,
        scope='Generated frames only; original decompressed bytes and logical hashes preserved. Raw hardware evidence is untouched.')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_or_batch',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();return 0 if archive_runs(a.run_or_batch,a.output)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
