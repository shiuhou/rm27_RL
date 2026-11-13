"""R3: one matched exploration-scale intervention using the frozen PPO workflow."""
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

from . import regional_update_experiment as previous
from . import rotation_bridge_experiment as instrumented
from . import terminal_workflow as workflow
from .asset import ROOT, digest, read_json, write_result

CONFIG = ROOT / 'configs/rm27_asm2_r3.json'


def configuration(path=CONFIG):
    exp = read_json(path)
    if exp['schema'] != 'asm2_r3_exploration_v1' or exp['per_arm_transitions'] != 196608:
        raise ValueError('R3 authorization is exactly 196,608 transitions per arm')
    base, _, _, source = previous.configuration(ROOT / exp['source_config'])
    cfg = copy.deepcopy(base)
    cfg.update(config_path=str(Path(path).relative_to(ROOT)), output=exp['output'], r3=exp,
               evaluate_every=98304, evaluation_wait_s=1800,
               budget=dict(per_arm_transitions=196608, total_transitions=393216))
    if exp['initial_std_B'] != .64 or exp['intermediate_transitions'] != 98304:
        raise ValueError('Unexpected R3 intervention or assessment boundary')
    return cfg, base, source, ROOT / cfg['output']


@contextmanager
def exploration_initialization(enabled, resume, std):
    """Change only fresh B's Gaussian scale; standard PPO owns sampling and density."""
    if not enabled or resume:
        yield
        return
    import torch
    from simpleflight_lab import ppo
    original = ppo.PPO

    class ExplorationPPO(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            with torch.no_grad():
                self.actor.log_std.fill_(float(np.log(std)))

    with patch.object(ppo, 'PPO', ExplorationPPO):
        yield


def fresh_cases(cfg, source, output):
    """Fresh seeded offsets, no outcome selection and no feasibility-controller runs."""
    from . import terminal_native_eval as native
    from .handover_curriculum import native_verifier
    from .rotation_bridge import RESET_KEYS
    _, profile, spec, manifest_path, manifest, _ = native.load_evaluation_inputs(ROOT/cfg['task_config'])
    old = native.load_starts(manifest_path.parent, manifest['splits']['heldout'],
                            expected_count=105, profile=profile)
    reached = native.load_starts(manifest_path.parent, manifest['heldout_handover'],
                                expected_count=4, profile=profile)
    bridge = dict(np.load(source/'bridge-validation.npz', allow_pickle=False))
    ids = np.repeat(np.flatnonzero(bridge['variant'] == 'nominal'), 3)
    parts = [{k: bridge[k][ids].copy() for k in RESET_KEYS}]
    labels = [np.full(len(ids), 'earlier_fresh')]
    for phase in ('rotation', 'braking', 'terminal'):
        chosen = []
        for case in sorted(set(old['case_ids'])):
            candidates = np.flatnonzero((old['case_ids'] == case) & (old['phase'] == phase))
            chosen.extend(candidates[np.rint(np.linspace(0, len(candidates)-1, 6)).astype(int)])
        parts.append({k: old[k][chosen].copy() for k in RESET_KEYS})
        labels.append(np.full(len(chosen), phase))
    parts.append({k: reached[k].copy() for k in RESET_KEYS})
    labels.append(np.full(4, 'reached_handover'))
    bank = {k: np.concatenate([p[k] for p in parts]) for k in RESET_KEYS}
    bank['region'] = np.concatenate(labels)
    bank['dt'] = np.asarray(profile.policy_dt_s)
    rng = np.random.default_rng(cfg['r3']['heldout_seed'])
    offsets = rng.uniform(-1., 1., (len(bank['states']), 2))
    # Existing 0.5 mm / 5 mm/s sensitivity range, independent offsets here.
    bank['states'][:, :3] += offsets[:, :1]*.0005*spec.target_rotation[:, 0]
    bank['states'][:, 7:10] += offsets[:, 1:]*.005*spec.target_rotation[:, 0]
    verify = native_verifier(native.NativeContact(profile, spec))
    checks = [verify(s, m, t) for s, m, t in zip(bank['states'], bank['motor_thrust_N'], bank['motor_target_N'])]
    if not all(c['passed'] for c in checks):
        raise ValueError('A preregistered fresh reset is invalid; do not silently replace it')
    limits = read_json(native.HANDOVER_CONFIG)['flight_envelope']
    if any(native.flight_limits(s, spec, limits) is not None for s in bank['states']):
        raise ValueError('Fresh reset outside frozen flight envelope')
    with (output/'heldout.npz').open('xb') as f:
        np.savez_compressed(f, **bank)
    return dict(count=len(checks), all_resets_valid=True, seed=cfg['r3']['heldout_seed'],
                source_manifest=str(manifest_path.relative_to(ROOT)), source_sha256=digest(manifest_path),
                counts={str(k):int(v) for k,v in zip(*np.unique(bank['region'], return_counts=True))},
                limitation='Fresh offsets on historically examined sources. Earlier anchors share development source trajectories; other sources are disjoint from training and selection. No physical uncertainty calibration.')


def freeze(cfg, base, source, output):
    from .regional_update_runtime import verify_alias
    if (output/'freeze.json').exists():
        return verify_freeze(cfg, output)
    old = verify_alias(base, source)
    workflow.legacy_inputs(cfg)
    output.mkdir(parents=True, exist_ok=True)
    if (output/'ledger.json').exists():
        raise ValueError('Cannot freeze after training starts')
    for name in old['archives']:
        if (output/name).exists():
            raise ValueError('Unfinished freeze requires inspection, not overwrite')
        shutil.copyfile(source/name, output/name)
    heldout = fresh_cases(cfg, source, output)
    inputs = {**old['historical_source_pins'], **old['source_pins'], **cfg['pins']}
    inputs.update({str(Path(__file__).relative_to(ROOT)): '',
                   'tests/test_asm2_r3.py': '',
                   str(source/'freeze.json'): '', str(source/'runtime-alias-receipt.json'): '',
                   '.agents/docs/ASM2_G312_CONTACT_TAIL.md': '',
                   'configs/rm27_asm2_flight.json': '', 'configs/rm27_asm2_contact.json': ''})
    pins = {str((ROOT/p).relative_to(ROOT)):digest(ROOT/p) for p in inputs}
    image_id = subprocess.check_output(previous.docker_prefix()+['image','inspect',previous.runtime_image(),
                                                                '--format','{{.Id}}'], text=True).strip()
    if image_id != old['runtime']['image_id']:
        raise ValueError('Training runtime changed')
    write_result(output/'freeze.json', dict(config_sha256=digest(ROOT/cfg['config_path']),
        archives={**old['archives'], 'heldout.npz':digest(output/'heldout.npz')},
        source_pins=pins, historical_source_pins={},
        runtime=dict(image=previous.runtime_image(), image_id=image_id),
        heldout=heldout, budget=cfg['budget'], reset_mixture=cfg['bridge']['mixtures'],
        parent=cfg['parent'], parent_sha256=digest(ROOT/cfg['parent']),
        intervention='Fresh B only: set actor.log_std=log(0.64); retain optimizer state, then learn std normally. A unchanged. Deterministic mean actor initially identical.',
        intermediate='Matched 98,304 transitions; stop both for technical failure or preregistered futility. Otherwise continue both to 196,608.',
        futility='At intermediate: B has zero earlier training qualifications, no gain in native new descriptor entries or bridge qualifications over A, and no new qualified anchor. Stop only if all hold.',
        selection='Primary comparison is last common positive endpoint. Heldout uses that endpoint, without best-checkpoint search. Seal actor hashes before opening heldout outcomes.',
        scientific='Require net qualified gains at >=2 distinct earlier anchors, at least one new robust 3/3 anchor, braking>=32/36 and terminal>=27/30 without retention exits; corroborate on fresh cases. A region entry alone is not success.',
        limits='One matched seed, shared sources for earlier heldout perturbations; fresh valid resets on process restart, not bitwise PhysX continuation. No official impact or hardware claims.',
        frozen_unix_s=time.time()))


def verify_freeze(cfg, output):
    return previous.verify_freeze(cfg, output)


def docker_command(cfg, output, command):
    args = previous.docker_command(cfg, output, command)
    args[args.index('rm27-asm2-regional-update')] = 'rm27-asm2-r3'
    args[args.index('rm27_asm2.regional_update_experiment')] = 'rm27_asm2.r3_experiment'
    return args


def run(cfg, output, stage):
    def launch(command, **kwargs):
        arm = command[command.index('--arm')+1]
        if stage == 'mid' and '--stop-updates' not in command:
            log = output/arm/'learning.jsonl'
            updates = json.loads(log.read_text().splitlines()[-1])['updates'] if log.exists() else 0
            if updates >= 96:
                raise ValueError('Intermediate stage already completed')
            command = [*command, '--stop-updates', str(96-updates)]
        return subprocess.Popen(docker_command(cfg, output, command), **kwargs)
    with patch.object(workflow, 'subprocess', SimpleNamespace(Popen=launch, STDOUT=subprocess.STDOUT)):
        instrumented.run(cfg, output)


def train(cfg, output, arm, resume, stop_updates):
    from isaaclab.app import AppLauncher
    launcher = AppLauncher(headless=True)
    code = 0
    try:
        import torch
        from .rotation_bridge_metrics import install_metrics
        folder = output/arm
        checkpoint = torch.load(folder/'resume.pt', map_location='cpu', weights_only=True) if resume else {}
        with instrumented.training_budget(), install_metrics(folder,
                completed_transitions=checkpoint.get('completed_transitions',0),
                updates=checkpoint.get('workflow_updates',0)), exploration_initialization(
                    arm == 'B', resume, cfg['r3']['initial_std_B']):
            workflow.train(cfg, output, arm, resume, stop_updates)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workflow-config', type=Path, default=CONFIG)
    parser.add_argument('command', choices=('freeze','run','train'))
    parser.add_argument('--stage', choices=('mid','full'), default='mid')
    parser.add_argument('--arm', choices=('A','B'))
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--stop-updates', type=int, default=0)
    args = parser.parse_args()
    cfg, base, source, output = configuration(args.workflow_config.resolve())
    if args.command == 'freeze':
        freeze(cfg, base, source, output)
        return
    verify_freeze(cfg, output)
    if args.command == 'run':
        run(cfg, output, args.stage)
    else:
        if args.arm is None:
            parser.error('train needs --arm')
        train(cfg, output, args.arm, args.resume, args.stop_updates)


if __name__ == '__main__':
    main()
