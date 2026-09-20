"""Run independent S0 SITL trials, preserving every attempt and one replay."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def audit_trials(trials):
    """Compare recorded inputs, not just directory names or process exit codes."""
    keys = ('scene_sha256', 'binary_sha256', 'runner_sha256', 'parameter_files')
    reference = None
    failures = []
    for row in trials:
        folder = Path(row['directory'])
        try:
            meta = json.loads((folder/'metadata.json').read_text())
            summary = json.loads((folder/'summary.json').read_text())
            values = {k: meta[k] for k in keys}
            if reference is None:
                reference = values
            if values != reference:
                failures.append({'trial': row['index'], 'reason': 'Recorded inputs differ'})
            if summary.get('passed') is not True or row.get('returncode') != 0:
                failures.append({'trial': row['index'], 'reason': 'Trial did not pass'})
        except (OSError, KeyError, ValueError) as error:
            failures.append({'trial': row['index'], 'reason': str(error)})
    return {'passed': bool(trials) and not failures, 'failures': failures,
            'compared_keys': keys}


def run(scene, output, count=10, instance=31):
    if count < 1:
        raise ValueError('count must be positive')
    scene, output = Path(scene).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, gate='S0', count=count, trials=[],
                  mode='headless closed-loop SITL; simulation-truth guidance',
                  scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                  commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip())
    (output / 'working.diff').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT))
    # Include untracked integration sources as well as tracked files in provenance.
    result['source_hashes'] = {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for folder in ('rm27', 'configs') for p in sorted((ROOT / folder).rglob('*'))
        if p.is_file() and p.suffix in ('.py', '.json', '.parm')
    }

    def save():
        (output / 'summary.json').write_text(json.dumps(result, indent=2))

    save()
    for i in range(count):
        trial = output / f'trial-{i+1:02d}'
        command = [sys.executable, '-m', 'rm27.autopilot.micro_flight', str(scene),
                   '--output', str(trial), '--instance', str(instance)]
        row = dict(index=i+1, directory=str(trial), command=command, passed=False)
        result['trials'].append(row)
        save()
        started = time.monotonic()
        with (output / f'trial-{i+1:02d}.log').open('w') as log:
            process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        row.update(returncode=process.returncode, wall_seconds=time.monotonic()-started)
        if (trial / 'summary.json').exists():
            summary = json.loads((trial / 'summary.json').read_text())
            row.update(passed=process.returncode == 0 and summary.get('passed') is True,
                       failure=summary.get('failure'))
        else:
            row['failure'] = 'Missing trial summary'
        save()
        print(json.dumps(row), flush=True)
    successful = next((r for r in result['trials'] if r['passed']), None)
    if successful:
        command = [sys.executable, '-m', 'rm27.simulation.replay_micro_flight', successful['directory']]
        with (output / 'replay.log').open('w') as log:
            process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        replay_file = Path(successful['directory']) / 'replay.json'
        result['replay'] = json.loads(replay_file.read_text()) if replay_file.exists() else {'passed': False}
        result['replay_returncode'] = process.returncode
    result['successes'] = sum(r['passed'] for r in result['trials'])
    result['input_audit'] = audit_trials(result['trials'])
    result['passed'] = (count >= 10 and result['successes'] == count
                        and result['input_audit']['passed']
                        and result.get('replay', {}).get('passed') is True
                        and result.get('replay_returncode') == 0)
    save()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--count', type=int, default=10)
    parser.add_argument('--instance', type=int, default=31)
    args = parser.parse_args()
    result = run(args.scene, args.output, args.count, args.instance)
    print(json.dumps({'passed': result['passed'], 'successes': result['successes'], 'count': result['count']}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
