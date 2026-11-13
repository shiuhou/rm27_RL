"""Matched uniform-versus-region-weighted PPO continuation; historical sources stay pinned."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from . import rotation_bridge_experiment as prior
from . import terminal_workflow as workflow
from .asset import ROOT, digest, read_json, write_result
from .terminal_curriculum import BANDS, ResetSampler

CONFIG = ROOT / 'configs/rm27_asm2_regional_update.json'


def configuration(path):
    experiment = read_json(path)
    if experiment['schema'] != 'rm27_asm2_regional_update_v1' or experiment['per_arm_transitions'] != 98_304:
        raise ValueError('Expected approved 98,304-transition regional-update pilot')
    base, _, source = prior.configuration(ROOT / experiment['source_config'])
    cfg = copy.deepcopy(base)
    mixture = experiment['reset_mixture']
    if mixture != base['bridge']['mixtures']['B'] or experiment['update_target'] != mixture:
        raise ValueError('Both arms must retain the frozen earlier-reset mixture')
    cfg['bridge']['mixtures'] = {arm: dict(mixture) for arm in ('A', 'B')}
    cfg.update(config_path=str(path.relative_to(ROOT)), output=experiment['output'], regional_update=experiment)
    output = ROOT / cfg['output']
    if output == source or not output.resolve().is_relative_to(ROOT / '.agents/runs'):
        raise ValueError('Use an isolated project run directory')
    return cfg, base, source, output


def docker_prefix():
    return (['sudo', '-n', 'docker'] if not os.access('/var/run/docker.sock', os.W_OK) else ['docker'])


def runtime_image():
    return read_json(ROOT / 'configs/rm27_asm2_collision.json')['isaac_image']


def freeze(cfg, base, source, output):
    if (output / 'freeze.json').exists():
        verify_freeze(cfg, output)
        return
    frozen = workflow.verify_freeze(base, source)
    differences = {}
    for name, expected in frozen['source_pins'].items():
        actual = digest(ROOT / name)
        if actual != expected:
            if name != 'run_asm2_terminal_pilot.sh':
                raise ValueError(f'Protected prior implementation changed: {name}')
            differences[name] = dict(archived=expected, current=actual,
                reason='Existing host runtime launcher change, reviewed before new experiment; new run uses isolated launcher.')
    workflow.legacy_inputs(cfg)
    protocol = workflow.development_module(cfg)['load_protocol'](False)[-1]
    output.mkdir(parents=True, exist_ok=False)
    for name in frozen['archives']:
        shutil.copyfile(source / name, output / name)
    bank = workflow.load_bank(output)
    effective = {}
    for arm in ('A', 'B'):
        weights = ResetSampler(bank['band'], 'fixed', cfg['seed'], workflow.sampler_settings(cfg, arm)).weights()
        effective[arm] = {band: float(weights[bank['band'] == band].sum()) for band in BANDS}
        np.testing.assert_allclose(list(effective[arm].values()),
            [cfg['regional_update']['reset_mixture'].get(b, 0.) for b in BANDS], rtol=0, atol=1e-12)
    from . import regional_update, rotation_bridge_diagnostics, rotation_bridge_report, rotation_bridge_regional_report
    sources = [Path(m.__file__) for m in (regional_update, rotation_bridge_diagnostics,
               rotation_bridge_report, rotation_bridge_regional_report)] + [Path(__file__)]
    sources += [ROOT / cfg['regional_update']['source_config'], source / 'freeze.json',
                ROOT / 'tests/test_asm2_regional_update.py', ROOT / 'configs/rm27_asm2_collision.json']
    image_id = subprocess.check_output(docker_prefix() + ['image', 'inspect', runtime_image(),
                                                          '--format', '{{.Id}}'], text=True).strip()
    write_result(output / 'freeze.json', dict(
        config_sha256=digest(ROOT / cfg['config_path']), archives=frozen['archives'],
        source_pins={str(p.relative_to(ROOT)): digest(p) for p in sources},
        historical_source_pins={n: digest(ROOT/n) for n in frozen['source_pins']},
        prior_launcher_difference=differences, runtime=dict(image=runtime_image(), image_id=image_id),
        effective_mixtures=effective, seed=cfg['seed'], budget=cfg['budget'], wall_clock_limit=None,
        legacy_development=protocol, scientific_decision=rotation_bridge_diagnostics.SCIENTIFIC_RULE,
        strict_promotion=cfg['bridge']['gate'], endpoint=98_304,
        weighting='Per rollout: w_i=N*p_region/(n_region*sum_present_p). Mean-one; no clipping. '
                  'Apply to actor surrogate and both scalar critic Huber means. '
                  'Identical random minibatches, global GAE and ValueNorm. Missing bands reported.',
        limits='One seed; same reset probabilities, not guaranteed identical realized trajectories. '
               'No held-out evaluation or generalization claim. Both arms restart after one charged update.',
        frozen_unix_s=time.time()))


def verify_freeze(cfg, output):
    frozen = workflow.verify_freeze(cfg, output)
    for name, expected in {**frozen['historical_source_pins'], **frozen['source_pins']}.items():
        if digest(ROOT / name) != expected:
            raise ValueError(f'Frozen input changed: {name}')
    return frozen


def docker_command(cfg, output, command):
    """Same reviewed runtime/mounts as the host launcher; isolated module and output."""
    if command[:2] != [str(ROOT/'run_asm2_terminal_pilot.sh'), 'workflow']:
        raise ValueError('Unexpected workflow launch command')
    cache = ROOT / '.agents/runs/asm2-contact/cache'
    for folder in (cache/'kit', cache/'user'):
        folder.mkdir(parents=True, exist_ok=True)
    uid, gid = os.getuid(), os.getgid()
    frozen = verify_freeze(cfg, output)
    args = docker_prefix() + ['run', '--rm', '--gpus', 'all', '--network', 'none',
        '--name', 'rm27-asm2-regional-update', '--shm-size=2g', '--memory=48g', '--cpus=16',
        '--ulimit', 'core=0', '--init', '--user', f'{uid}:{gid}']
    for name, value in dict(HOME='/home/ubuntu', ACCEPT_EULA='Y', PRIVACY_CONSENT='N',
            PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1', PYTHONPATH='/sf/src:/sf',
            OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='1', NVIDIA_DRIVER_CAPABILITIES='all').items():
        args += ['--env', f'{name}={value}']
    for path in ('/home/ubuntu', '/isaac-sim/kit/data', '/isaac-sim/kit/logs'):
        args += ['--tmpfs', f'{path}:uid={uid},gid={gid},mode=0700']
    for src, dst, readonly in ((cache/'kit', '/isaac-sim/kit/cache', False),
                              (cache/'user', '/home/ubuntu/.cache', False),
                              (ROOT, '/sf', True), (output, '/sf/'+str(output.relative_to(ROOT)), False)):
        args += ['--mount', f'type=bind,src={src},dst={dst}' + (',readonly' if readonly else '')]
    return args + ['--workdir', '/sf', '--entrypoint', '/isaac-sim/python.sh', frozen['runtime']['image_id'],
                   '-m', 'rm27_asm2.regional_update_experiment', *command[2:]]


def run(cfg, output):
    # Adapt only launch dispatch; reuse the pinned accounting/evaluation workflow.
    def launch(command, **kwargs):
        return subprocess.Popen(docker_command(cfg, output, command), **kwargs)
    with patch.object(workflow, 'subprocess', SimpleNamespace(Popen=launch, STDOUT=subprocess.STDOUT)):
        prior.run(cfg, output)


def train(cfg, output, arm, resume, stop_updates):
    from isaaclab.app import AppLauncher
    launcher = AppLauncher(headless=True)
    code = 0
    try:
        import torch
        from .rotation_bridge_metrics import install_metrics
        from .regional_update import install_regional_update
        folder = output / arm
        checkpoint = torch.load(folder/'resume.pt', map_location='cpu', weights_only=True) if resume else {}
        counters = dict(completed_transitions=checkpoint.get('completed_transitions', 0),
                        updates=checkpoint.get('workflow_updates', 0))
        with prior.training_budget(), install_metrics(folder, **counters), install_regional_update(
                folder, cfg['regional_update']['update_target'], balanced=(arm == 'B'), **counters):
            workflow.train(cfg, output, arm, resume, stop_updates)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workflow-config', type=Path, default=CONFIG)
    parser.add_argument('command', choices=('freeze', 'run', 'train', 'final'))
    parser.add_argument('--arm', choices=('A', 'B'))
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--stop-updates', type=int, default=0)
    args = parser.parse_args()
    cfg, base, source, output = configuration(args.workflow_config.resolve())
    if args.command == 'freeze':
        freeze(cfg, base, source, output)
        return
    verify_freeze(cfg, output)
    if args.command == 'run':
        run(cfg, output)
    elif args.command == 'train':
        if args.arm is None:
            parser.error('train requires --arm')
        train(cfg, output, args.arm, args.resume, args.stop_updates)
    else:
        from .rotation_bridge_report import finalize
        from .rotation_bridge_diagnostics import finalize_diagnostics
        finalize_diagnostics(cfg, output, finalize(cfg, output))


if __name__ == '__main__':
    main()
