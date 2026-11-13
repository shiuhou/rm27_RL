"""Configuration-driven continuation/evaluation for terminal_training, with native sidecar."""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
import multiprocessing
import os
from pathlib import Path
import random
import runpy
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from .terminal_curriculum import BANDS, MIX, CampaignLedger, ResetSampler
from simpleflight_lab.stage1_training import LearningBudget, LearningLimit, write_json


def configuration(path):
    cfg = read_json(path)
    if cfg['schema'] != 'asm2_resumable_comparison_v1':
        raise ValueError('Unknown workflow configuration')
    if Path(path).resolve() != (ROOT/cfg['config_path']).resolve():
        raise ValueError('config_path must identify the configuration file being used')
    for name, expected in cfg['pins'].items():
        if digest(ROOT/name) != expected:
            raise ValueError(f'Pinned input changed: {name}')
    if 'bridge' in cfg:
        validate_bridge_configuration(cfg)
    return cfg, ROOT/cfg['output']


def validate_bridge_configuration(cfg):
    bridge = cfg['bridge']
    fixed = {band: mass for band, mass in zip(BANDS, MIX[0]) if mass}
    intervention = dict(fixed, rotation_late=.20, rotation_bridge=.25)
    if cfg['arms'] != {'A': 'fixed', 'B': 'fixed'} or bridge['mixtures'] != {'A': fixed, 'B': intervention}:
        raise ValueError('Phase R2 must redistribute only 25 points from late rotation')
    caps = cfg['budget']
    if not (0 < caps['per_arm_transitions'] <= 98_304 and 0 < caps['per_arm_learning_s'] <= 1350
            and 0 < caps['total_transitions'] <= 196_608 and 0 < caps['total_learning_s'] <= 2700):
        raise ValueError('Phase R2 authorization limits exceeded')
    expected_gate = dict(robust_anchors=6, anchor_delta=3, braking_qualified=32,
                         terminal_qualified=27, max_exits=0)
    if bridge['gate'] != expected_gate or bridge['gate_checkpoint'] != 'last_common_positive_transitions':
        raise ValueError('Phase R2 gate changed')


def legacy_inputs(cfg):
    # Read-only compatibility with archived validation; no archived trainer is executed.
    source = ROOT/cfg['backend_verifier']
    verifier = runpy.run_path(str(source))
    receipt = verifier['verify_receipt']()
    task = read_json(ROOT/cfg['task_config'])
    return receipt, task


def load_bank(directory):
    with np.load(directory/'training.npz', allow_pickle=False) as data:
        return dict(data)


def freeze(cfg, directory):
    from . import terminal_native_eval as native
    from .handover_curriculum import native_verifier
    if (directory/'freeze.json').exists():
        verify_freeze(cfg, directory)
        return
    if (directory/'ledger.json').exists():
        raise ValueError('Cannot freeze new cases after learning')
    receipt, task = legacy_inputs(cfg)
    if 'bridge' in cfg:
        from .rotation_bridge import freeze_bridge
        # The legacy development suites are verified before either learner starts.
        protocol = development_module(cfg)['load_protocol'](False)[-1]
        frozen = freeze_bridge(cfg, directory)
        bank = load_bank(directory)
        effective_mixtures = {}
        for arm in ('A', 'B'):
            sampler = ResetSampler(bank['band'], cfg['arms'][arm], cfg['seed'], sampler_settings(cfg, arm))
            weights = sampler.weights()
            effective_mixtures[arm] = {b: float(weights[bank['band'] == b].sum()) for b in BANDS}
        write_result(directory/'freeze.json', dict(
            config_sha256=digest(ROOT/cfg['config_path']), archives=frozen['archives'],
            bridge_bank=frozen, effective_mixtures=effective_mixtures,
            legacy_development=protocol,
            resume='Fresh valid resets; PhysX solver/contact-cache restoration is not bitwise reproducible.',
            frozen_unix_s=time.time(), backend_receipt_sha256=digest(ROOT/cfg['receipt'])))
        return
    _, profile, spec, manifest_path, manifest, _ = native.load_evaluation_inputs(ROOT/cfg['task_config'])
    source_dir = manifest_path.parent
    original = native.load_starts(source_dir, manifest['splits']['development'], expected_count=280, profile=profile)
    with np.load(ROOT/cfg['bank'], allow_pickle=False) as data:
        bank = dict(data)
    bank['phase'] = np.array(['rotation' if str(b).startswith('rotation') else str(b) for b in bank['band']])
    bank['sample_times_s'] = np.array([manifest['cases'][str(c)]['first_contact_sample_s'] for c in bank['case_ids']])-bank['witness_lead_s']
    keys = ('states', 'motor_thrust_N', 'motor_target_N', 'case_ids', 'source_frame', 'sample_times_s', 'phase')
    training_cases = sorted(set(bank['case_ids']))
    reached = np.flatnonzero(np.isin(original['case_ids'], training_cases) & (original['phase'] == 'handover'))
    joined = {k: np.concatenate((bank[k], original[k][reached])) for k in keys}
    joined['band'] = np.r_[bank['band'], np.full(len(reached), 'handover')]
    joined['dt'] = profile.policy_dt_s
    if set(joined['case_ids']) & set(task['selection_cases']):
        raise ValueError('Selection trajectories entered training')
    directory.mkdir(parents=True, exist_ok=True)
    with (directory/'training.npz').open('xb') as f:
        np.savez_compressed(f, **joined)
    # Final distribution is frozen before learning. These are new perturbations of
    # previously examined source trajectories, NOT independent new trajectories.
    old = native.load_starts(source_dir, manifest['splits']['heldout'], expected_count=105, profile=profile)
    reached_final = native.load_starts(source_dir, manifest['heldout_handover'], expected_count=4, profile=profile)
    chosen = []
    for phase in ('rotation', 'braking'):
        for case in sorted(set(old['case_ids'])):
            ids = np.flatnonzero((old['phase'] == phase) & (old['case_ids'] == case))
            chosen.extend(ids[np.linspace(0, len(ids)-1, 6, dtype=int)])
    final = {k: np.concatenate((old[k][chosen], reached_final[k])) for k in keys}
    final['region'] = np.r_[old['phase'][chosen], np.full(4, 'reached_handover')]
    final['dt'] = profile.policy_dt_s
    rng = np.random.default_rng(cfg['final_seed'])
    n = len(final['states'])
    offsets = rng.uniform(-1., 1., (n, 2))
    final['states'][:, :3] += offsets[:, :1]*cfg['final_position_tangent_m']*spec.target_rotation[:, 0]
    final['states'][:, 7:10] += offsets[:, 1:]*cfg['final_velocity_tangent_m_s']*spec.target_rotation[:, 0]
    verify = native_verifier(native.NativeContact(profile, spec))
    for data in (joined, final):
        for i, state in enumerate(data['states']):
            result = verify(state, data['motor_thrust_N'][i], data['motor_target_N'][i])
            if not result['passed']:
                raise ValueError(f'Invalid reset {data["case_ids"][i]}:{data["source_frame"][i]}: {result}')
    with (directory/'final.npz').open('xb') as f:
        np.savez_compressed(f, **final)
    write_result(directory/'freeze.json', dict(config_sha256=digest(ROOT/cfg['config_path']),
        archives={name: digest(directory/name) for name in ('training.npz', 'final.npz')},
        training_counts=dict(Counter(joined['band'])), final_counts=dict(Counter(final['region'])),
        final_source_overlap='New seeded state perturbations; all source trajectories were examined historically. No outcome selection.',
        final_shared_trajectories=sorted(set(final['case_ids'])),
        resume='Fresh valid resets; PhysX solver/contact-cache restoration is not bitwise reproducible.',
        frozen_unix_s=time.time(), backend_receipt_sha256=digest(ROOT/cfg['receipt'])))


def verify_freeze(cfg, directory):
    frozen = read_json(directory/'freeze.json')
    if frozen['config_sha256'] != digest(ROOT/cfg['config_path']):
        raise ValueError('Frozen configuration differs')
    for name, expected in frozen['archives'].items():
        if digest(directory/name) != expected:
            raise ValueError(f'Frozen suite changed: {name}')
    return frozen


def development_module(cfg):
    return runpy.run_path(str(ROOT/cfg['development_evaluator']))


def native_evaluate(cfg, directory, request):
    """Fresh portable checkpoint inference; never substitutes controller actions."""
    from . import terminal_native_eval as native
    started = time.perf_counter()
    checkpoint = ROOT/request['checkpoint']
    weights = checkpoint.with_suffix('.npz')
    if digest(checkpoint) != request['checkpoint_sha256']:
        raise ValueError('Requested checkpoint changed')
    output = ROOT/request['output']
    if output.exists():
        report = read_json(output)
        if report['checkpoint_sha256'] != digest(checkpoint):
            raise ValueError('Existing evaluation is for another actor')
        return report
    module = development_module(cfg)
    loaded = module['load_protocol'](False)
    profile, spec, manifest_path, manifest, goal, bank, indices, protocol = loaded
    devpath = output.with_name(output.stem+'-development.json')
    if not devpath.exists():
        args = argparse.Namespace(actor=weights, checkpoint=checkpoint, controls=False, workers=cfg['workers'],
                                  replay_dir=None, output=devpath)
        module['evaluate'](args, loaded)
    dev = read_json(devpath)['results']['candidate']
    report = dict(checkpoint_sha256=digest(checkpoint), weights_sha256=digest(weights),
                  development_sha256=digest(devpath), development=dev,
                  curriculum_trials=None, wall_s=None)
    if request.get('probe'):
        train = load_bank(directory)
        count = len(train['states'])
        probe = {k: np.repeat(train[k], 3, axis=0) for k in (
            'states', 'motor_thrust_N', 'motor_target_N', 'case_ids', 'source_frame', 'sample_times_s', 'phase')}
        signs = np.tile([0., -1., 1.], count)
        probe['states'][:, :3] += signs[:, None]*cfg['adaptive']['position_tangent_m']*spec.target_rotation[:, 0]
        probe['states'][:, 7:10] += signs[:, None]*cfg['adaptive']['velocity_tangent_m_s']*spec.target_rotation[:, 0]
        rows = evaluate_probe_bank(cfg, probe, checkpoint, weights)
        outcomes = np.array([r['qualified'] for r in rows]).reshape(count, 3)
        report['curriculum_trials'] = outcomes.astype(int).tolist()
        report['curriculum_episodes'] = rows
    if 'bridge' in cfg:
        with np.load(directory/'bridge-validation.npz', allow_pickle=False) as data:
            bridge_bank = dict(data)
        rows = evaluate_probe_bank(cfg, bridge_bank, checkpoint, weights)
        anchors = {}
        for row, variant in zip(rows, bridge_bank['variant'], strict=True):
            row.update(frame=row['source_frame'], variant=str(variant), lead_s=row['source_contact_lead_s'])
            variants = anchors.setdefault((row['case'], row['frame']), {})
            if row['variant'] in variants:
                raise ValueError('Duplicate bridge variant')
            variants[row['variant']] = row['qualified']
        if len(anchors) != 12 or any(set(v) != {'nominal', 'minus', 'plus'} for v in anchors.values()):
            raise ValueError('Incomplete bridge evaluation')
        counts = [sum(variants.values()) for variants in anchors.values()]
        report['bridge'] = dict(rows=rows, episodes=len(rows), qualified=sum(counts),
            robust=counts.count(3), partial=sum(n in (1, 2) for n in counts),
            exits=sum(r['safety_failure'] is not None for r in rows),
            saturation_controls=sum(r['saturation_controls'] for r in rows),
            reasons=dict(Counter(r['reason'] for r in rows)))
    report['wall_s'] = time.perf_counter()-started
    write_json(output, report)
    return report


def evaluate_probe_bank(cfg, bank, checkpoint, weights):
    from . import terminal_native_eval as native
    _, profile, spec, manifest_path, manifest, goal = native.load_evaluation_inputs(ROOT/cfg['task_config'])
    controllers = {'candidate': native.load_controller(weights, digest(checkpoint), profile, goal)}
    plant = native.NativeContact(profile, spec)
    limits = read_json(native.HANDOVER_CONFIG)['flight_envelope']
    taskcfg = read_json(ROOT/cfg['task_config'])
    horizons = dict(terminal=taskcfg['braking_horizon_s'], braking=taskcfg['braking_horizon_s'],
                    rotation=taskcfg['handover_horizon_s'], handover=taskcfg['handover_horizon_s'])
    tasks = [('candidate', str(bank.get('region', bank['phase'])[i]), bank, i,
              manifest['cases'][str(bank['case_ids'][i])].get('first_contact_sample_s')) for i in range(len(bank['states']))]
    with ProcessPoolExecutor(max_workers=cfg['workers'], mp_context=multiprocessing.get_context('fork'),
            initializer=native.init_worker, initargs=(plant, controllers, horizons, limits, False)) as pool:
        rows = [r for r, _ in pool.map(native.worker_episode, tasks)]
    return rows

def ranking(report, bridge=None):
    dev = report['development']
    c, order = dev['cohorts'], dev['rotation_order']
    if bridge is not None:
        gate = bridge['gate']
        retained = all(c[b]['qualified'] >= gate[f'{b}_qualified'] and c[b]['safety_failures'] == 0
                       for b in ('braking', 'terminal'))
        return (retained, c['rotation']['qualified'], -c['rotation']['safety_failures'],
                c['braking']['qualified'], c['terminal']['qualified'])
    retained = all(c[b]['qualified'] >= 27 and c[b]['safety_failures'] == 0 for b in ('terminal', 'braking'))
    promoted = retained and order['late']['qualified'] >= 6 and order['late']['safety_failures'] == 0
    return (promoted, retained, c['rotation']['qualified'], -c['rotation']['safety_failures'], c['braking']['qualified'])


def sampler_settings(cfg, arm):
    if 'bridge' in cfg:
        return {'fixed_mix': cfg['bridge']['mixtures'][arm]}
    return cfg['adaptive']


def train(cfg, directory, arm, resume, stop_updates):
    import torch
    from .terminal_training import CurriculumEnvironment
    from .terminal_isaac import TerminalSim
    from .flight import load_profile
    from .contact_task import load_contact_spec
    from .proxy import load_proxy
    from simpleflight_lab.ppo import Actor, PPO, ROLLOUT_STEPS
    receipt, task = legacy_inputs(cfg)
    verify_freeze(cfg, directory)
    output = directory/arm
    output.mkdir(exist_ok=True)
    latest = output/'resume.pt'
    if latest.exists() != resume:
        raise ValueError('Existing arm requires --resume; a new arm cannot resume')
    config_sha = digest(ROOT/cfg['config_path'])
    ledger = CampaignLedger(directory, config_sha, cfg['budget'])
    state = ledger.data['arms'][arm]
    if state['transitions'] and not latest.exists():
        raise ValueError('Consumed arm has no valid resume checkpoint')
    profile, spec = load_profile(), load_contact_spec()
    asset = (ROOT/cfg['overlay'], [*load_proxy()['entries'], spec.entries['collision_0_570']])
    if digest(asset[0]) != receipt['asset_sha256']:
        raise ValueError('Verified overlay changed')
    sim = TerminalSim(task['learning']['worlds'], 0, profile, spec, diagnostic=True,
                      physics_dt=receipt['physics_dt_s'], contact_offset_m=receipt['contact_offset_m'], asset_override=asset)
    bank = load_bank(directory)
    mode = cfg['arms'][arm]
    sampler = ResetSampler(bank['band'], mode, cfg['seed'], sampler_settings(cfg, arm))
    random.seed(cfg['seed']); np.random.seed(cfg['seed']); torch.manual_seed(cfg['seed'])
    source = torch.load(latest if resume else ROOT/cfg['parent'], map_location='cpu', weights_only=True)
    if resume and (source['config_sha256'] != config_sha or source['arm'] != arm):
        raise ValueError('Resume belongs to another arm or configuration')
    if resume:
        sampler.load_state_dict(source['sampler'])
        random.setstate(source['rng']['python'])
        torch.set_rng_state(source['rng']['torch'])
        torch.cuda.set_rng_state_all(source['rng']['cuda'])
        n = source['rng']['numpy']
        np.random.set_state((n[0], np.asarray(n[1], dtype=np.uint32), n[2], n[3], n[4]))
    goal = spec.target_center+(-spec.patch_face_z_m-task['goal_inset_m'])*spec.target_normal
    budget = LearningBudget(ROLLOUT_STEPS*sim.worlds, 60.)
    env = CurriculumEnvironment(sim, bank, sampler, goal, task, budget)
    obs = env.observation()
    actor = Actor(45).to(sim.device)
    actor.load_state_dict(source['actor'])
    ppo = PPO(obs, task['learning']['initial_std'], actor=actor, learning_rate=task['learning']['lr'],
              gamma=cfg['ppo']['gamma'], gae_lambda=cfg['ppo']['gae_lambda'])
    for name, obj in (('critic', ppo.critic), ('value_normalizer', ppo.normalizer),
                      ('actor_optimizer', ppo.actor_opt), ('critic_optimizer', ppo.critic_opt)):
        obj.load_state_dict(source[name])
    if resume:
        # Constructors consume RNG; restore learner RNG again after construction.
        torch.set_rng_state(source['rng']['torch']); torch.cuda.set_rng_state_all(source['rng']['cuda'])
        env.outcomes.update(source['outcomes'])
        env.completed_by_row[:] = source['completed_by_row']
        env.success_by_row[:] = source['success_by_row']
        env.saturations.fill_(source.get('saturation_controls', 0))
    history = source.get('evaluation_history', []) if resume else []
    completed = source.get('completed_transitions', 0) if resume else 0
    updates = source.get('workflow_updates', 0) if resume else 0
    pauses = source.get('evaluation_wall_s', 0.) if resume else 0.
    restarts = source.get('restart_count', 0)+1 if resume else 0
    last_eval = history[-1]['transitions'] if history else -1
    base_actor = torch.cat([v.flatten().to(sim.device) for k,v in source['actor'].items() if k != 'log_std'])
    stop_reason = 'ERROR'
    signal.signal(signal.SIGTERM, lambda *_: setattr(budget, 'stop_reason', 'INTERRUPTED'))
    signal.signal(signal.SIGALRM, lambda *_: setattr(budget, 'stop_reason', 'TIME_LIMIT'))

    def save(path):
        rng_np = np.random.get_state()
        payload = dict(format=1, task='asm2_resumable_contact', airframe='asm2', actor_obs=45,
            actor={k:v.detach().cpu() for k,v in actor.state_dict().items()}, critic=ppo.critic.state_dict(),
            value_normalizer=ppo.normalizer.state_dict(), actor_optimizer=ppo.actor_opt.state_dict(),
            critic_optimizer=ppo.critic_opt.state_dict(), sampler=sampler.state_dict(),
            rng=dict(python=random.getstate(), torch=torch.get_rng_state(), cuda=torch.cuda.get_rng_state_all(),
                     numpy=(rng_np[0], rng_np[1].tolist(), *rng_np[2:])),
            config=cfg, config_sha256=config_sha, arm=arm,
            learning_s=ledger.data['arms'][arm]['learning_s'], additional_transitions=ledger.data['arms'][arm]['transitions'],
            completed_transitions=completed, workflow_updates=updates, evaluation_history=history,
            evaluation_wall_s=pauses, restart_count=restarts, source_parent_sha256=digest(ROOT/cfg['parent']),
            policy_dt_s=profile.policy_dt_s, action_rate_rad_s=[2.,2.,2.], action_collective_hover=[.5,1.5],
            outcomes=dict(env.outcomes), saturation_controls=int(env.saturations), completed_by_row=env.completed_by_row.tolist(), success_by_row=env.success_by_row.tolist(),
            simulator_snapshot=dict(state=env.state.cpu(), filtered_thrust=sim.thrust.cpu(), held_targets=sim.targets.cpu(),
                                    active_rows=env.active_row.cpu(), ticks=env.episode_ticks.cpu()),
            simulator_restore='fresh_valid_resets; snapshot is diagnostic only; solver cache is not restored',
            rollout_state='empty_at_update_boundary', schedule=dict(kind='constant', gamma=ppo.gamma, gae_lambda=ppo.gae_lambda))
        temp = path.with_suffix('.tmp')
        torch.save(payload, temp); temp.replace(path)
        return payload

    def evaluate(complete_only=False):
        nonlocal pauses, last_eval
        start = time.perf_counter()
        frames = completed if complete_only else ledger.data['arms'][arm]['transitions']
        path = output/f'checkpoint-{frames:06d}.pt'
        if not path.exists():
            if complete_only:
                # An aborted minibatch may have modified the in-memory actor.
                temp = path.with_suffix('.tmp')
                shutil.copyfile(latest, temp)
                temp.replace(path)
            else:
                save(path)
        loaded = torch.load(path, map_location='cpu', weights_only=True)
        weights = path.with_suffix('.npz')
        if not weights.exists():
            with weights.open('xb') as f:
                np.savez_compressed(f, **{k:v.numpy() for k,v in loaded['actor'].items()}, checkpoint_sha256=digest(path))
        result_path = output/f'eval-{frames:06d}.json'
        request = dict(checkpoint=str(path.relative_to(ROOT)), checkpoint_sha256=digest(path),
                       output=str(result_path.relative_to(ROOT)), probe=(mode == 'adaptive'))
        write_json(output/'evaluation-request.json', request)
        while not result_path.exists():
            if budget.stop_reason == 'INTERRUPTED':
                raise LearningLimit('INTERRUPTED')
            if time.perf_counter()-start > cfg['evaluation_wait_s']:
                raise RuntimeError('Native development evaluation unavailable')
            time.sleep(.25)
        report = read_json(result_path)
        if report['checkpoint_sha256'] != digest(path) or report['weights_sha256'] != digest(weights):
            raise ValueError('Native evaluation actor mismatch')
        if mode == 'adaptive':
            sampler.refresh(report['curriculum_trials'])
        elif 'bridge' not in cfg:
            c, o = report['development']['cohorts'], report['development']['rotation_order']
            retained = all(c[b]['qualified'] >= 27 and c[b]['safety_failures'] == 0 for b in ('braking','terminal'))
            late = o['late']['qualified'] >= 6 and o['late']['safety_failures'] == 0
            middle = o['middle']['qualified'] >= 4 and o['middle']['safety_failures'] == 0
            if retained and late and (sampler.stage == 0 or (sampler.stage == 1 and middle)):
                sampler.stage += 1
            elif sampler.stage > 0 and not (retained and late and (sampler.stage == 1 or middle)):
                sampler.stage -= 1
        entry = dict(transitions=frames, checkpoint=str(path.relative_to(ROOT)),
                     result=str(result_path.relative_to(ROOT)), rank=ranking(report, cfg.get('bridge')),
                     cohorts=report['development']['cohorts'], sampler_stage=sampler.stage)
        if 'bridge' in cfg:
            entry['bridge'] = {k: v for k, v in report['bridge'].items() if k != 'rows'}
        history.append(entry)
        pauses += time.perf_counter()-start
        last_eval = frames
        if not complete_only:
            save(latest)
        print('WORKFLOW_DEV', arm, frames, entry['rank'], entry.get('bridge', ''), flush=True)

    try:
        if not latest.exists():
            save(latest)
        if not history:
            evaluate()
        else:
            # Restart before an evaluation finished must not skip that milestone.
            frames = ledger.data['arms'][arm]['transitions']
            if frames-last_eval >= cfg['evaluate_every']:
                evaluate()
        initial_updates = updates
        with (output/'learning.jsonl').open('a', buffering=1) as log:
            while True:
                frames_left, seconds_left = ledger.remaining(arm)
                size = ROLLOUT_STEPS*sim.worlds
                if frames_left < size or seconds_left <= .5:
                    stop_reason = 'TRANSITION_LIMIT' if frames_left < size else 'TIME_LIMIT'
                    break
                reservation = ledger.reserve(arm, size)
                budget.frames = 0; budget.max_seconds = reservation
                budget.started = time.perf_counter(); budget.stop_reason = None
                signal.setitimer(signal.ITIMER_REAL, max(.001, reservation-.1))
                try:
                    before = time.perf_counter()
                    obs, stats = ppo.collect(env, obs)
                    collected = time.perf_counter()-before
                    loss = ppo.update(before_minibatch=budget.check)
                    if not bool(torch.isfinite(loss).all() and torch.isfinite(stats).all()):
                        raise RuntimeError('Nonfinite PPO update')
                    completed += size; updates += 1
                    elapsed = budget.elapsed
                    signal.setitimer(signal.ITIMER_REAL, 0.)
                    ledger.settle(arm, elapsed, str(latest.relative_to(ROOT)), True)
                    save(latest)
                except BaseException:
                    signal.setitimer(signal.ITIMER_REAL, 0.)
                    if ledger.data['arms'][arm]['pending'] is not None:
                        ledger.settle(arm, budget.elapsed, None, False)
                    raise
                state = ledger.data['arms'][arm]
                row = dict(transitions=state['transitions'], completed_transitions=completed, updates=updates,
                    learning_s=state['learning_s'], collect_s=collected, update_s=elapsed-collected,
                    mean_reward=float(stats[0]), loss=loss.tolist(), exploration=actor.log_std.exp().tolist(),
                    actor_parameter_delta_from_process_start=float((torch.cat([v.flatten() for k,v in actor.state_dict().items() if k != 'log_std'])-base_actor).norm()),
                    episodes=dict(env.outcomes), saturation_controls=int(env.saturations),
                    region_counts={b:dict(resets=int(sampler.draws[bank['band']==b].sum()),
                                         episodes=int(env.completed_by_row[bank['band']==b].sum()),
                                         qualified=int(env.success_by_row[bank['band']==b].sum())) for b in BANDS},
                    sampler_stage=sampler.stage, sampler_refreshes=sampler.refreshes)
                log.write(json.dumps(row, allow_nan=False)+'\n')
                if updates % 8 == 0 or updates == 1:
                    print('WORKFLOW_TRAIN', arm, state['transitions'], round(state['learning_s'],1), row['region_counts'], flush=True)
                if state['transitions']-last_eval >= cfg['evaluate_every']:
                    evaluate()
                    recent = history[-3:]
                    if state['transitions'] >= 100000 and len(recent) == 3 and all(
                        h['cohorts']['braking']['qualified'] < 15 and h['cohorts']['terminal']['qualified'] < 15 for h in recent):
                        stop_reason = 'PERSISTENT_RETENTION_COLLAPSE'; break
                if stop_updates and updates-initial_updates >= stop_updates:
                    stop_reason = 'INTEGRATION_PAUSE'; break
        if stop_reason != 'INTEGRATION_PAUSE' and last_eval != ledger.data['arms'][arm]['transitions']:
            evaluate()
    except LearningLimit as error:
        stop_reason = str(error)
        # The latest completed update is retained; partial PPO buffers never saved.
        if 'bridge' in cfg and stop_reason == 'TIME_LIMIT' and completed > last_eval:
            evaluate(complete_only=True)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)
        write_json(output/'result.json', dict(stop_reason=stop_reason, accounting=ledger.data['arms'][arm],
            completed_transitions=completed, evaluations=history, selected=max(history, key=lambda h:h['rank']) if history else None,
            evaluation_wall_s=pauses, restart_count=restarts,
            resume_limit='Fresh complete resets; active episodes discarded on restart; no bitwise simulator continuation.'))
        ledger.close()
    return stop_reason


def host_run(cfg, directory):
    """One host process services native evaluation while the existing Docker launcher trains."""
    verify_freeze(cfg, directory)
    from simpleflight_lab.monitoring import Monitor, training_metrics
    for arm in ('A', 'B'):
        output = directory/arm
        output.mkdir(exist_ok=True)
        monitor = Monitor(output, 'asm2-reset-'+arm, metadata=dict(config=cfg, arm=arm))
        try:
            # A charged one-update stop and a fresh process resume in BOTH arms.
            last_line = 0
            for integration in (True, False):
                previous = read_json(output/'result.json') if (output/'result.json').exists() else None
                if previous and previous['stop_reason'] not in ('INTEGRATION_PAUSE', 'INTERRUPTED', 'ERROR'):
                    break
                if integration and (output/'resume.pt').exists():
                    continue
                command = [str(ROOT/'run_asm2_terminal_pilot.sh'), 'workflow', '--workflow-config', cfg['config_path'],
                           'train', '--arm', arm]
                if (output/'resume.pt').exists():
                    command.append('--resume')
                if integration:
                    command.extend(['--stop-updates', '1'])
                with (output/'launch.log').open('a') as stream:
                    process = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
                    while process.poll() is None:
                        request_path = output/'evaluation-request.json'
                        if request_path.exists():
                            request = read_json(request_path)
                            if not (ROOT/request['output']).exists():
                                native_evaluate(cfg, directory, request)
                        log = output/'learning.jsonl'
                        if log.exists():
                            content = log.read_text()
                            lines = content.split('\n')[:-1]
                            for line in lines[last_line:]:
                                row = json.loads(line)
                                counts = row['episodes']
                                metrics = training_metrics(dict(row, frames=row['transitions'],
                                    episodes=sum(counts.values()), qualified=counts.get('DESIGNATED_LOADED', 0)),
                                    optimization=row['loss'])
                                monitor.log(metrics, step=row['transitions'])
                            last_line = len(lines)
                        time.sleep(1.)
                    if process.returncode:
                        raise RuntimeError(f'Arm {arm} exited {process.returncode}; inspect {output}/launch.log')
                result = read_json(output/'result.json')
                if integration and result['stop_reason'] != 'INTEGRATION_PAUSE':
                    raise RuntimeError('Save/restart integration did not reach its boundary')
        finally:
            monitor.finish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workflow-config', type=Path, required=True)
    parser.add_argument('command', choices=('freeze', 'run', 'train', 'evaluate', 'final'))
    parser.add_argument('--arm', choices=('A','B'))
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--stop-updates', type=int, default=0)
    parser.add_argument('--request', type=Path)
    args = parser.parse_args()
    if args.command == 'train' and args.arm is None:
        parser.error('train requires --arm')
    if args.command == 'evaluate' and args.request is None:
        parser.error('evaluate requires --request')
    cfg, directory = configuration(args.workflow_config)
    if args.command == 'freeze':
        freeze(cfg, directory)
    elif args.command == 'run':
        host_run(cfg, directory)
    elif args.command == 'final':
        if 'bridge' in cfg:
            from .rotation_bridge_report import finalize
            finalize(cfg, directory)
        else:
            from .terminal_workflow_final import evaluate_final
            evaluate_final(cfg, directory)
    elif args.command == 'evaluate':
        verify_freeze(cfg, directory)
        native_evaluate(cfg, directory, read_json(args.request))
    else:
        from isaaclab.app import AppLauncher
        launcher = AppLauncher(headless=True)
        code = 0
        try:
            train(cfg, directory, args.arm, args.resume, args.stop_updates)
        except Exception:
            import traceback
            traceback.print_exc(); code = 2
        sys.stdout.flush(); sys.stderr.flush(); os._exit(code)


if __name__ == '__main__':
    main()
