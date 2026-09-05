"""Investigate nonmonotonic latency effects with a controlled common initial hold."""
from dataclasses import asdict, replace
import argparse
import itertools
import json
from pathlib import Path
import shutil
from .campaign import source_hash
from .config import Config
from .simulation import simulate
from .scoring import summarize, paired_difference


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    source = output/'source'/'astra'
    source.mkdir(parents=True)
    for path in Path(__file__).parent.glob('*.py'):
        shutil.copy2(path, source/path.name)
    cfg = Config(fov_deg=60)
    (output/'manifest.json').write_text(json.dumps({'seed':28201, 'n':1000, 'base':asdict(cfg), 'source_sha256':source_hash(),
        'question':'Does the latency/contact reversal persist with a common 500 ms observation hold?',
        'protocol':'hold 0/0.5s x delay 0/0.15s x compensated pursuit/lead; fixed gains; new paired trajectories'}, indent=2)+'\n')
    summary, groups = {}, {}
    with (output/'episodes.jsonl').open('w') as raw:
        for hold, delay, method in itertools.product([0., .5], [0., .15], ['compensated_pursuit', 'lead']):
            label = f'hold{hold:g}_delay{delay:g}/{method}'
            rows = simulate(replace(cfg, guidance_start_s=hold, latency_s=delay), method, 1000, 28201)
            groups[label] = rows
            summary[label] = summarize(rows)
            for row in rows:
                raw.write(json.dumps({'condition':label, **row}, allow_nan=False)+'\n')
            print(label, summary[label]['successes'], flush=True)
    for hold, method in itertools.product([0., .5], ['compensated_pursuit', 'lead']):
        summary[f'hold{hold:g}/{method}/delay_effect'] = paired_difference(groups[f'hold{hold:g}_delay0/{method}'],groups[f'hold{hold:g}_delay0.15/{method}'])
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args().output)
