"""Losslessly archive or restore closed generated PX4 auxiliary recordings.

Use only after the experiment and its scoring processes have stopped. Frame
readers already support gzip; auxiliary native JSONL and ULog tools can require
restoration. Original byte hashes and failed flight results are retained.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import uuid

from rm27.autopilot.px4.return_fault_batch import digest
from rm27.telemetry.recording import compress_recording


def archive_file(path):
    if path.suffix == '.jsonl':
        return compress_recording(path)
    if path.suffix != '.ulg':
        raise ValueError('Only generated ULog or JSONL supported')
    target = path.with_name(path.name+'.gz')
    manifest = path.with_name(path.name+'.archive.json')
    if target.exists() or manifest.exists():
        raise FileExistsError('Archive already exists')
    temporary = target.with_name(target.name+'.tmp-'+uuid.uuid4().hex)
    before = path.stat()
    original = hashlib.sha256()
    try:
        with path.open('rb') as source, temporary.open('xb') as output:
            with gzip.GzipFile(filename='', mode='wb', fileobj=output, mtime=0) as zipped:
                for chunk in iter(lambda: source.read(1024*1024), b''):
                    original.update(chunk)
                    zipped.write(chunk)
            output.flush()
            os.fsync(output.fileno())
        check = hashlib.sha256()
        with gzip.open(temporary, 'rb') as source:
            for chunk in iter(lambda: source.read(1024*1024), b''):
                check.update(chunk)
        after = path.stat()
        if (check.digest() != original.digest() or
            (before.st_ino, before.st_size, before.st_mtime_ns) !=
            (after.st_ino, after.st_size, after.st_mtime_ns)):
            raise ValueError('Source changed or archive verification failed')
        os.link(temporary, target)
        record = dict(passed=True, logical_path=path.name, storage_path=target.name,
            original_sha256=original.hexdigest(), compressed_sha256=digest(target),
            original_bytes=before.st_size, compressed_bytes=target.stat().st_size,
            run_type='verified lossless generated storage; not flight acceptance')
        with manifest.open('x') as stream:
            stream.write(json.dumps(record, indent=2)+'\n')
        path.unlink()
        return record
    finally:
        temporary.unlink(missing_ok=True)


def restore(manifest):
    record = json.loads(manifest.read_text())
    path = manifest.parent / record['logical_path']
    archived = manifest.parent / record['storage_path']
    if path.exists():
        if digest(path) != record['original_sha256']:
            raise ValueError('Existing restored file differs')
        return dict(passed=True, path=str(path), already_present=True)
    if digest(archived) != record['compressed_sha256']:
        raise ValueError('Archive bytes changed')
    temporary = path.with_name(path.name+'.restore-'+uuid.uuid4().hex)
    try:
        with gzip.open(archived, 'rb') as source, temporary.open('xb') as output:
            for chunk in iter(lambda: source.read(1024*1024), b''):
                output.write(chunk)
            output.flush()
            os.fsync(output.fileno())
        if digest(temporary) != record['original_sha256'] or temporary.stat().st_size != record['original_bytes']:
            raise ValueError('Original bytes not restored')
        os.link(temporary, path)
        return dict(passed=True, path=str(path), original_sha256=record['original_sha256'])
    finally:
        temporary.unlink(missing_ok=True)


def run(root, output, *, restoring=False):
    root, output = Path(root).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    records = []
    runs = [root] if (root/'metadata.json').exists() else sorted(p.parent for p in root.rglob('metadata.json'))
    for recording in runs:
        summary = json.loads((recording/'summary.json').read_text())
        metadata = json.loads((recording/'metadata.json').read_text())
        if summary.get('autopilot') != 'px4' or 'scene_sha256' not in metadata:
            raise ValueError('Closed generated PX4 recordings required')
        if restoring:
            for manifest in sorted(recording.rglob('*.archive.json')):
                if manifest.name.endswith(('.ulg.archive.json', '.jsonl.archive.json')):
                    records.append(restore(manifest))
        else:
            for path in sorted(recording.rglob('*')):
                if path.is_file() and path.suffix in ('.jsonl', '.ulg') and path.stat().st_size >= 100_000:
                    record = archive_file(path)
                    record['path'] = str(path)
                    records.append(record)
        print(json.dumps(dict(run=str(recording),files=len(records))), flush=True)
    result = dict(passed=bool(records) and all(r['passed'] for r in records), records=records,
        saved_bytes=sum(r.get('original_bytes',0)-r.get('compressed_bytes',0) for r in records),
        run_type='lossless generated evidence restoration' if restoring else 'lossless generated evidence storage',
        scope='Closed local PX4 JSONL/ULog only. Camera images, summaries and user hardware inputs remain intact. Auxiliary readers may require restoration.')
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording_or_batch', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--restore', action='store_true')
    args = parser.parse_args()
    raise SystemExit(0 if run(args.recording_or_batch,args.output,restoring=args.restore)['passed'] else 1)
