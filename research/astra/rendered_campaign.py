"""Paired geometric versus rendered-pixel closed loops on a NEW scenario seed."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time
from .campaign import snapshot_source
from .config import Config
from .rendered_camera import RenderedCamera
from .scoring import summarize, paired_difference
from .simulation import simulate


def run(output, n=24):
    output.mkdir(parents=True, exist_ok=False)
    digest = snapshot_source(output)
    # Protocol fixed after v1, no gain changes: test the sensor abstraction,
    # not an independent algorithm selection. Zero added feature noise in both.
    base = Config(pixel_noise=0, radius_noise_fraction=0)
    seed = 28111
    manifest = {'seed': seed, 'n': n, 'base': asdict(base), 'source_sha256': digest,
                'hypothesis': 'Replacing geometric features with actual RGB segmentation can change outcomes and method ranking.',
                'limitations': 'Simple sphere/lighting/pinhole and synthetic horizontal blur; same surrogate plant; not hardware transfer evidence.'}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    all_results = {}
    began = time.monotonic()
    with (output/'episodes.jsonl').open('w') as raw:
        for fov in [90, 120]:
            cfg = replace(base, fov_deg=fov)
            for method in ['pursuit', 'compensated_pursuit', 'lead']:
                reference = simulate(cfg, method, n, seed)
                label = f'f{fov}/{method}/geometric'
                for row in reference:
                    raw.write(json.dumps({'condition': label, **row}, allow_nan=False)+'\n')
                all_results[label] = summarize(reference)
                for blur in [1, 11]:
                    camera = RenderedCamera(cfg, blur)
                    trace = []
                    try:
                        rows = simulate(cfg, method, n, seed, camera=camera, trace=trace)
                    finally:
                        camera.close()
                    label = f'f{fov}/{method}/pixels_blur{blur}'
                    for row in rows:
                        raw.write(json.dumps({'condition': label, **row}, allow_nan=False)+'\n')
                    raw.flush()
                    all_results[label] = summarize(rows)
                    all_results[label]['versus_geometric'] = paired_difference(reference, rows)
                    all_results[label]['rendered_frames'] = camera.frames
                    (output/f'trace_f{fov}_{method}_blur{blur}.json').write_text(json.dumps(trace)+'\n')
                    (output/'summary.json').write_text(json.dumps(all_results, indent=2)+'\n')
                    print(label, all_results[label]['successes'], '/', n, 'elapsed_s', round(time.monotonic()-began, 1), flush=True)
    manifest['elapsed_s'] = time.monotonic()-began
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--n', default=24, type=int)
    args = parser.parse_args()
    run(args.output, args.n)
