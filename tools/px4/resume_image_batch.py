"""Continue an interrupted nominal batch without repeating or discarding attempts.

Original manifests and reports are retained. An existing unfinished flight is
an interrupted failure; a completed flight can finish its independent scoring.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

from rm27.autopilot.px4.image_batch import assess, report, trial
from rm27.autopilot.px4.return_fault_batch import ROOT, digest
from rm27.experiments.target_region_audit import audit_recording
from rm27.telemetry.recording import compress_recording
from rm27.telemetry.ulog import analyze


def verify_frozen(hashes):
    changed = [p for p, h in hashes.items()
               if not Path(p).is_file() or digest(p) != h]
    if changed:
        raise ValueError('Frozen inputs changed: ' + str(changed))


def completed_stdout(log, summary):
    """Verify final base-flight stdout; the mission appends independent scores.

    The flight prints its result before image_mission enriches the saved result.
    Only the known gate label may change; all printed flight fields must match.
    The process exit remains unavailable and is never invented.
    """
    contents = Path(log).read_text()
    decoder = json.JSONDecoder()
    for start in range(len(contents) - 1, -1, -1):
        if contents[start] != '{':
            continue
        try:
            value, end = decoder.raw_decode(contents[start:])
        except ValueError:
            continue
        required = {'passed', 'events', 'phase', 'auto_disarm', 'failure'}
        if (isinstance(value, dict) and required <= value.keys()
                and not contents[start + end:].strip()
                and all(v == summary.get(k) for k, v in value.items() if k != 'gate_version')):
            return True
    return False


def score_existing(job):
    command, destination, hashes, surface_configuration, options = job
    destination = Path(destination)
    verify_frozen(hashes)
    def read(name):
        p = destination / name
        return json.loads(p.read_text()) if p.is_file() else {'passed': False}
    summary, vio = read('summary.json'), read('vio/summary.json')
    completion = completed_stdout(destination.with_suffix('.log'), summary)
    logs = list((destination / 'log').rglob('*.ulg'))
    log = read('ulog_analysis/summary.json')
    if 'sha256' not in log:
        log = analyze(logs[0], destination / 'ulog_analysis_resume') if len(logs) == 1 else {'passed': False}
    elif len(logs) != 1 or log['sha256'] != digest(logs[0]):
        raise ValueError('Saved ULog analysis does not match recorded log')
    surfaces = None
    if surface_configuration:
        surfaces = audit_recording(destination, surface_configuration,
                                    destination / 'local_surface_audit_resume')
    observer = read('observer_rate.json')
    qualification = assess(summary, vio, log, surfaces=surfaces,
        require_surfaces=bool(surface_configuration), observer=observer,
        required_observer_interval_us=options.get('required_observer_interval_us'))
    archives = []
    if options.get('compress_recordings') and (destination / 'frames.jsonl').exists():
        archives.append(compress_recording(destination / 'frames.jsonl'))
    return dict(passed=bool(completion and qualification['passed']), command=command,
        returncode=None, execution_status='original process exit unavailable; base-flight stdout verified; saved mission independently scored',
        completion_stdout_verified=completion, path=str(destination), summary=summary,
        qualification=qualification, vio_summary=vio, observer_rate=observer,
        local_surface_audit_path=str(destination / 'local_surface_audit_resume/summary.json'),
        recording_archives=archives)


def dispatch(job):
    command, destination, *_ = job
    if Path(destination).exists():
        if not (Path(destination) / 'summary.json').is_file():
            return dict(passed=False, failure='Interrupted flight has no final summary; retained without retry',
                        command=command, path=str(destination), returncode=None)
        return score_existing(job)
    return trial(job)


def run(batch, output, *, workers=2, instance=50):
    batch, output = Path(batch).resolve(), Path(output).resolve()
    if not 1 <= workers <= 3 or not 0 <= instance <= 99 - workers:
        raise ValueError('1..3 independent instances required')
    manifest_path = batch / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    hashes, spec = manifest['input_sha256'], manifest['configuration']
    verify_frozen(hashes)
    source_summary = batch / 'summary.json'
    attempts = json.loads(source_summary.read_text()).get('attempts', [])
    names = [a['name'] for a in attempts]
    if names != [c['name'] for c in spec['cases'][:len(names)]]:
        raise ValueError('Original attempt order does not match reserved cases')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'original_summary.json').write_bytes(source_summary.read_bytes())
    record = dict(original_manifest=str(manifest_path), original_manifest_sha256=digest(manifest_path),
        input_sha256=hashes, original_summary_sha256=digest(source_summary),
        configuration=spec, command=sys.argv, workers=workers, first_instance=instance,
        controller_sha256=digest(__file__), git_commit=subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        policy='Never retry an existing attempt; independently score completed artifacts and launch only absent cases')
    (output / 'manifest.json').write_text(json.dumps(record, indent=2) + '\n')
    camera = json.loads((ROOT / spec['camera_configuration']).read_text())
    surface = ROOT / spec['surface_configuration'] if spec.get('surface_configuration') else None
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for offset in range(len(attempts), len(spec['cases']), workers):
            group = spec['cases'][offset:offset + workers]
            jobs = []
            for i, case in enumerate(group):
                destination = batch / case['name']
                camera_path = batch / (case['name'] + '-camera.json')
                expected = dict(camera, seed=case['camera_seed'])
                if camera_path.exists():
                    if json.loads(camera_path.read_text()) != expected:
                        raise ValueError('Existing camera input differs from reserved seed')
                else:
                    camera_path.write_text(json.dumps(expected, indent=2) + '\n')
                command = [sys.executable, '-m', 'rm27.autopilot.px4.image_mission',
                    manifest['scene'], '--audit', manifest['audit'], '--config',
                    str(ROOT / spec['mission_configuration']), '--camera-config', str(camera_path),
                    '--output', str(destination), '--instance', str(instance + i),
                    '--speedup', str(spec['speedup'])]
                if destination.exists():
                    # Preserve the original launch assignment for saved flights.
                    original_index = spec['cases'].index(case)
                    original_instance = manifest['first_instance'] + original_index % manifest['workers']
                    command[command.index('--instance') + 1] = str(original_instance)
                jobs.append((command, destination, dict(hashes, **{str(camera_path):digest(camera_path)}), surface, spec))
            for case, result in zip(group, pool.map(dispatch, jobs)):
                attempts.append(dict(name=case['name'], camera_seed=case['camera_seed'], **result))
                summary = report(attempts, spec)
                (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
                print(json.dumps(dict(name=case['name'], passed=result['passed'],
                    failure=result.get('failure') or result.get('summary', {}).get('failure'))), flush=True)
    return report(attempts, spec)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--instance', type=int, default=50)
    args = parser.parse_args()
    raise SystemExit(0 if run(args.batch, args.output, workers=args.workers, instance=args.instance)['passed'] else 1)
