"""Run paired scenarios and store raw evidence; python -m astra.campaign --help."""
import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import itertools
import json
import platform
from pathlib import Path
import subprocess
import shutil
import time

import numpy as np
from .config import Config
from .scoring import summarize, paired_difference
from .simulation import simulate


def source_hash():
    digest = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob('*.py')):
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def snapshot_source(output):
    snapshot = output/'source'/'astra'
    snapshot.mkdir(parents=True)
    for path in Path(__file__).parent.glob('*.py'):
        shutil.copy2(path, snapshot/path.name)
    return source_hash()


def run(spec, output, split='development', full=False, n_override=None):
    output.mkdir(parents=True, exist_ok=False)
    digest = snapshot_source(output)
    (output/'spec.json').write_text(json.dumps(spec, indent=2)+'\n')
    base = Config(**spec['defaults'])
    seed = spec[f'{split}_seed']
    n = n_override or spec[f'{split}_n']
    conditions = [('nominal', base)]
    if full:
        conditions = [(f'l{latency:g}_f{fov:g}_tw{tw:g}', replace(base, latency_s=latency, fov_deg=fov, tw=tw))
                      for latency, fov, tw in itertools.product(spec['latency_s'], spec['fov_deg'], spec['tw'])]
        conditions += [(label, replace(base, **overrides)) for label, overrides in spec['stress_cases'].items()]
    try:
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL, text=True).strip()
    except subprocess.CalledProcessError:
        revision = 'uncommitted-initial-research'
    manifest = dict(utc=datetime.now(timezone.utc).isoformat(), split=split, n=n, seed=seed, specification=spec,
                    source_sha256=digest, git_revision=revision, python=platform.python_version(), numpy=np.__version__,
                    fidelity='thrust-vector surrogate + geometric camera features + abstract ego error; not hardware-validated',
                    conditions={name: asdict(c) for name, c in conditions})
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    results = {}
    started = time.monotonic()
    with (output/'episodes.jsonl').open('w') as raw:
        for name, cfg in conditions:
            pair = {}
            for method in spec['methods']:
                rows = simulate(cfg, method, n, seed)
                pair[method] = rows
                for row in rows:
                    raw.write(json.dumps({'condition': name, **row}, allow_nan=False)+'\n')
                raw.flush()
                summary = summarize(rows)
                results[f'{name}/{method}'] = {'overall': summary,
                    'by_scenario': {kind: summarize([r for r in rows if r['scenario'] == kind]) for kind in sorted(set(r['scenario'] for r in rows))}}
                print(f'{name:28} {method:8} {summary["successes"]:4}/{n} contact; miss95={summary["miss_p95_m"]:.3f}m elapsed={time.monotonic()-started:.1f}s', flush=True)
            results[f'{name}/paired'] = paired_difference(pair['pursuit'], pair['lead'])
            if 'compensated_pursuit' in pair:
                results[f'{name}/compensated_paired'] = paired_difference(pair['compensated_pursuit'], pair['lead'])
            (output/'summary.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    manifest['elapsed_s'] = time.monotonic()-started
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    write_report(results, output)
    return results


def write_report(results, output):
    lines = ['# Terminal-feasibility campaign', '', 'Geometric sensor / thrust-vector surrogate. Contact is not proof of collision survivability or visual detection.', '',
             '| Condition | Method | Contact % (95% CI) | Miss p95, m |', '|---|---|---:|---:|']
    for name, data in results.items():
        if 'overall' not in data:
            continue
        s = data['overall']
        lo, hi = s['success_ci95']
        condition, method = name.split('/')
        lines.append(f'| {condition} | {method} | {100*s["success_rate"]:.1f} ({100*lo:.1f}–{100*hi:.1f}) | {s["miss_p95_m"]:.3f} |')
    lines += ['', 'Every trial stays in the denominator. Raw episode records and complete assumptions accompany this report.',
              'Miss is minimum center separation sampled with swept segments, including the terminal segment. Contact-time statistics include successful trials only.',
              'The same scenarios appear across conditions: do not treat pooled trials as independent target trajectories.']
    (output/'report.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, default=Path(__file__).parent/'configs/campaign.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--split', choices=['development', 'evaluation'], default='development')
    parser.add_argument('--full', action='store_true')
    parser.add_argument('--n', type=int)
    args = parser.parse_args()
    if args.n is not None and args.n <= 0:
        parser.error('--n must be positive')
    run(json.loads(args.spec.read_text()), args.output, args.split, args.full, args.n)


if __name__ == '__main__':
    main()
