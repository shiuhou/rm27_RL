"""Independently match every completed nominal recovery to a certified corridor."""
import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys
import time

from rm27.autopilot.px4.return_fault_batch import ROOT, digest
from rm27.experiments.recovery_membership import run as membership


def run(batch_summary, configuration, certificate, output, *, wait=False):
    batch_summary, configuration, certificate, output = map(
        lambda p:Path(p).resolve(), (batch_summary, configuration, certificate, output))
    proof = json.loads(certificate.read_text())
    if proof.get('passed') is not True or proof.get('configuration_sha256') != digest(configuration):
        raise ValueError('Passed matching continuous certificate required')
    for p, h in proof['asset_sha256'].items():
        if digest(p) != h:
            raise ValueError('Certified asset changed: ' + p)
    for key, module in [('runner_sha256', 'recovery_corridor.py'),
        ('dependency_sha256', 'recovery_region.py'), ('surface_dependency_sha256', 'target_region_audit.py')]:
        if proof[key] != digest(ROOT/'rm27/experiments'/module):
            raise ValueError('Certified geometry source changed: ' + module)
    output.mkdir(parents=True, exist_ok=False)
    (output/'manifest.json').write_text(json.dumps(dict(batch_summary=str(batch_summary),
        configuration=str(configuration), configuration_sha256=digest(configuration),
        certificate=str(certificate), certificate_sha256=digest(certificate),
        command=sys.argv, runner_sha256=digest(__file__)), indent=2)+'\n')
    results = {}
    while True:
        batch = json.loads(batch_summary.read_text())
        (output/'source_summary.json').write_bytes(batch_summary.read_bytes())
        for attempt in batch['attempts']:
            name = attempt['name']
            if name in results:
                continue
            try:
                recording = Path(attempt['path'])
                metadata = json.loads((recording/'metadata.json').read_text())
                if metadata['scene_sha256'] != proof['scene_sha256']:
                    raise ValueError('Recording differs from certified scene')
                with (output/(name+'.log')).open('x') as log, redirect_stdout(log):
                    score = membership(recording, configuration, output/name)
                results[name] = dict(name=name, passed=attempt['passed'] is True and score['passed'] is True,
                    original_task_passed=attempt['passed'], membership_passed=score['passed'],
                    samples=score['samples'], duration_s=score['duration_s'],
                    outside_count=len(score['outside']), source=str(recording))
            except Exception as error:
                results[name] = dict(name=name, passed=False, failure=f'{type(error).__name__}: {error}')
            print(json.dumps(results[name]), flush=True)
        complete = len(batch['attempts']) == batch['required_count']
        result = dict(passed=bool(complete and batch['passed'] is True and all(r['passed'] for r in results.values())),
            total_attempts=len(results), required_count=batch['required_count'],
            passed_count=sum(r['passed'] for r in results.values()),
            gate_version='PX4_NOMINAL_WITH_CONTINUOUS_RECOVERY_MEMBERSHIP_V1',
            run_type='independent recorded native-step batch audit',
            source_gate_version=batch['gate_version'], source_summary_sha256=digest(batch_summary),
            certificate_sha256=digest(certificate), attempts=list(results.values()),
            scope='Same frozen nominal tasks plus every native recovery step within the passed bounded continuous corridor. No hardware or unrestricted-contact claim.')
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        if complete or not wait:
            return result
        time.sleep(10)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_summary', type=Path)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--certificate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wait', action='store_true')
    args = parser.parse_args()
    raise SystemExit(0 if run(args.batch_summary, args.config, args.certificate, args.output, wait=args.wait)['passed'] else 1)
