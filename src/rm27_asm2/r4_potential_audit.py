"""Bounded, archive-first Step 1 for R4; does not launch PPO or a controller."""
from __future__ import annotations

import argparse
import json

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from scipy.stats import rankdata

from .asset import ROOT, digest, read_json, write_result
from .braking_potential import BrakingPotential
from .contact_native import NativeContact
from .terminal_native_eval import load_evaluation_inputs, restore

CONFIG = ROOT / 'configs/rm27_asm2_r4.json'


def save_once(path, value):
    value = json.loads(json.dumps(value, allow_nan=False))
    if path.exists():
        if read_json(path) != value:
            raise ValueError(f'Refusing to overwrite different evidence: {path}')
    else:
        write_result(path, value)


def auc(labels, values):
    labels, values = np.asarray(labels, bool), np.asarray(values)
    n, m = int(labels.sum()), int((~labels).sum())
    return float((rankdata(values)[labels].sum()-n*(n+1)/2)/(n*m)) if n and m else None


def inputs():
    cfg = read_json(CONFIG)
    folder = ROOT / cfg['archive']
    manifest, results = read_json(folder/'manifest.json'), read_json(folder/'results.json')
    if results['manifest_sha256'] != digest(folder/'manifest.json'):
        raise ValueError('Frontier result is not paired with the archived manifest')
    for path, sha in manifest['pins'].items():
        if digest(ROOT/path) != sha:
            raise ValueError(f'Frozen frontier input changed: {path}')
    fixed = manifest['controllers']['fixed_a']
    for path, sha in [(ROOT/fixed['checkpoint'], fixed['checkpoint_sha256']),
                      ((ROOT/fixed['checkpoint']).with_suffix('.npz'), fixed['weights_sha256'])]:
        if digest(path) != sha:
            raise ValueError(f'Fixed A changed: {path}')
    samples = {(s['case'], s['frame'], s['variant']): s for s in manifest['samples']}
    rows = [dict(samples[(r['case'], r['frame'], r['variant'])], outcome=r)
            for r in results['rows'] if r['controller'] == 'fixed_a'
            and r['case'] in cfg['training_sources']]
    if len(rows) != 120 or sum(r['outcome']['qualified'] for r in rows) != 58:
        raise ValueError('Unexpected training-only frontier support')
    task, profile, spec, source_path, source, goal = load_evaluation_inputs(
        ROOT/manifest['configuration']['experiment'])
    return cfg, manifest, rows, task, profile, spec, source_path, source


def freeze():
    cfg, manifest, rows, task, profile, spec, source_path, source = inputs()
    pins = dict(manifest['pins'])
    paths = [CONFIG, ROOT/'src/rm27_asm2/braking_potential.py',
             ROOT/'src/rm27_asm2/r4_potential_audit.py', ROOT/'tests/test_asm2_braking_potential.py',
             ROOT/cfg['archive']/'manifest.json', ROOT/cfg['archive']/'results.json',
             ROOT/cfg['conditional_training']['reset_archive'],
             ROOT/manifest['controllers']['fixed_a']['checkpoint'],
             ROOT/manifest['controllers']['phase4']['checkpoint']]
    for name in cfg['training_sources']:
        trace = source_path.parent/source['cases'][name]['trace']['path']
        if digest(trace) != source['cases'][name]['trace']['sha256']:
            raise ValueError('Saved witness trace changed')
        paths.append(trace.resolve())
    pins.update({str(p.relative_to(ROOT)): digest(p) for p in paths})
    save_once(ROOT/cfg['output']/'freeze.json', dict(schema=cfg['schema'], configuration=cfg,
        pins=pins, model=manifest['model'], date='2026-11-13',
        decision='Conditional on all offline gates and then the bounded native confirmation.',
        gate_scope='Prospective diagnostic usefulness criteria, not physical or official thresholds.',
        archived_states=120, archived_qualified=58, new_rollouts=0,
        targets='Preserved as provenance; overwritten by the next CTBR at every policy boundary.',
        limits='A failed Fixed A continuation is not proof of unreachability. Kernels outside observed states are extrapolation.'))


def compiled_envelope(plant):
    """Extract support vertices without changing the model or stepping physics."""
    m, d = plant.model, plant.data
    # Kinematics alone supplies the compiled frames. A full forward solve at
    # the coordinate origin would unnecessarily place the body in the armor.
    mujoco.mj_resetData(m, d)
    d.qpos[:] = [0., 0., 0., 1., 0., 0., 0.]
    mujoco.mj_kinematics(m, d)

    def vertices(name):
        gid = m.geom(name).id
        mid = int(m.geom_dataid[gid]); start = int(m.mesh_vertadr[mid]); n = int(m.mesh_vertnum[mid])
        return m.mesh_vert[start:start+n] @ d.geom_xmat[gid].reshape(3, 3).T + d.geom_xpos[gid]

    body = np.vstack([vertices(n) for n in plant.spec.entries])
    hull = body[ConvexHull(body).vertices]
    target_support = max(float(np.max(vertices(n) @ plant.spec.target_normal))
                         for n in plant.spec.legal_targets)
    return hull, target_support


def audit():
    cfg, manifest, rows, task, profile, spec, source_path, source = inputs()
    out = ROOT/cfg['output']; frozen = read_json(out/'freeze.json')
    pins = dict(frozen['pins'])
    correction = out/'freeze-correction.json'
    if correction.exists():
        amendment = read_json(correction)
        if amendment['freeze_sha256'] != digest(out/'freeze.json'):
            raise ValueError('Correction belongs to a different freeze')
        pins.update(amendment['corrected_pins'])
    for path, sha in pins.items():
        if digest(ROOT/path) != sha:
            raise ValueError(f'R4 frozen input changed: {path}')
    plant = NativeContact(profile, spec)
    if plant.metadata != manifest['model']:
        raise ValueError('Frozen native plant metadata mismatch')
    vertices, target_support = compiled_envelope(plant)
    limits = read_json(ROOT/task['pins']['handover_limits']['path'])['flight_envelope']
    states = np.array([r['state'] for r in rows]); motors = np.array([r['motor_thrust_N'] for r in rows])
    labels = np.array([r['outcome']['qualified'] for r in rows])
    cases = np.array([r['case'] for r in rows]); phases = np.array([r['phase'] for r in rows])
    nominal = np.array([r['variant']=='nominal' for r in rows])
    end = np.array([r['outcome']['end_s'] for r in rows])

    def fit(mask):
        return BrakingPotential(states[mask], motors[mask], end[mask], profile, spec,
                                vertices, target_support, limits, cfg['gamma'], cfg['potential_bound'])

    full = fit(labels & nominal)
    phi, hidden, best = full.evaluate(states, motors)
    cv = np.zeros(len(rows)); cv_hidden = np.zeros(len(rows)); folds = []
    for case in cfg['training_sources']:
        held = cases == case
        potential = fit(labels & nominal & ~held)
        cv[held], cv_hidden[held], _ = potential.evaluate(states[held], motors[held])
        folds.append(dict(case=case, auc=auc(labels[held], cv[held]),
            positive_median=float(np.median(cv[held & labels])),
            negative_median=float(np.median(cv[held & ~labels])), scales=potential.scales))

    # The observed clearance is a full-body separating-plane bound, not a
    # fabricated finite-mesh distance. Native restoration also checks all field
    # contacts, not merely the registered target. No dynamics are advanced.
    geometry = []
    for r, state, motor in zip(rows, states, motors):
        contacts = restore(plant, state, motor, np.array(r['motor_target_N']))
        geometry.append(dict(case=r['case'], frame=r['frame'], variant=r['variant'],
                             target_separation_lower_bound_m=float(full.clearance(state)[0]),
                             native_signed_pairs=sum(c['distance_m'] <= 0 for c in contacts)))

    directions = []; curves = []
    for case in cfg['training_sources']:
        record = source['cases'][case]
        trace = source_path.parent/record['trace']['path']
        with np.load(trace) as z:
            handover = int(z['handover_frame'])
            x, f, t = z['states'][:, 0], z['motor_thrust_N'][:, 0], z['sample_times_s']
        # Offline directional check on the already-saved successful motion,
        # ending at earliest archived nominal Fixed A-qualified entry.
        first = min(r['frame'] for r in rows if r['case']==case and r['variant']=='nominal' and r['outcome']['qualified'])
        indices = np.arange(handover, first+1)
        value, h, _ = full.evaluate(x[indices], f[indices])
        delta = np.diff(value)
        passes = bool(value[-1] > value[0] and np.mean(delta>0) >= cfg['offline_gates']['early_positive_delta_fraction_min']
                      and np.median(abs(delta)) >= cfg['offline_gates']['early_median_absolute_delta_min'])
        directions.append(dict(case=case, start_frame=handover, end_frame=first,
            samples=len(indices), start_potential=float(value[0]), end_potential=float(value[-1]),
            positive_delta_fraction=float(np.mean(delta>0)), median_absolute_delta=float(np.median(abs(delta))),
            hidden_p90=float(np.quantile(h, .9)), passed=passes,
            interpretation='Saved qualified witness motion; offline signal check, no trajectory reference supplied to an actor.'))
        curves.append(dict(case=case, time_s=(t[indices]-t[handover]).tolist(), potential=value.tolist(),
                           hidden_fraction=h.tolist()))

    transition = np.isin(phases, ['transition_pre','transition_post'])
    overall_auc, transition_auc = auc(labels, cv), auc(labels[transition], cv[transition])
    positive_fraction = float(np.mean(full.clearance(states[labels])>0))
    hidden_p90 = float(np.quantile(np.r_[cv_hidden, *[c['hidden_fraction'] for c in curves]], .9))
    g = cfg['offline_gates']
    gates = dict(leave_source_out=overall_auc >= g['leave_source_out_auc_min'],
        transition_discrimination=transition_auc >= g['transition_only_auc_min'],
        source_consistency=sum(r['positive_median'] > r['negative_median'] for r in folds)>=g['positive_median_above_negative_sources_min'],
        useful_early_direction=sum(r['passed'] for r in directions)>=g['early_direction_sources_min'],
        hidden_actuator_contribution=hidden_p90<=g['hidden_log_score_fraction_p90_max'],
        full_body_clearance=positive_fraction>=g['positive_clearance_fraction_min'] and all(r['native_signed_pairs']==0 for r in geometry))
    scored = [dict(case=r['case'], frame=r['frame'], variant=r['variant'], phase=r['phase'],
                   qualified=r['outcome']['qualified'], reason=r['outcome']['reason'],
                   end_s=r['outcome']['end_s'], potential=float(phi[i]), leave_source_out_potential=float(cv[i]),
                   hidden_fraction=float(hidden[i])) for i, r in enumerate(rows)]
    result = dict(schema=cfg['schema'], freeze_sha256=digest(out/'freeze.json'),
        decision='OFFLINE_PASS_NATIVE_CONFIRMATION_REQUIRED' if all(gates.values()) else 'STEP1_NO_GO',
        training_transitions=0, new_native_continuations=0, archived_native_continuations=len(rows),
        positives=int(labels.sum()), nominal_support_nodes=int((labels & nominal).sum()), scales=full.scales,
        support_vertices=len(vertices), registered_target_support_m=target_support,
        leave_source_out_auc=overall_auc, transition_only_auc=transition_auc,
        positive_clearance_fraction=positive_fraction, hidden_log_score_fraction_p90=hidden_p90,
        gates=gates, folds=folds, directions=directions, geometry=geometry, scores=scored, curves=curves,
        interpretation='No formal reachability claim. Failed Fixed A outcomes do not imply states are dynamically unreachable.')
    save_once(out/'potential-audit.json', result)
    save_once(out/'empirical-support.json', dict(nodes=[r for r, good in zip(rows, labels & nominal) if good],
        geometry=geometry, scales=full.scales, body_envelope_vertices_m=vertices.tolist(),
        target_support_m=target_support, clearance_kind='conservative separating-plane lower bound'))
    print(json.dumps({k:v for k,v in result.items() if k not in ['scores','curves','geometry']}, indent=2))


def plots():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cfg=read_json(CONFIG); out=ROOT/cfg['output']; data=read_json(out/'potential-audit.json')
    fig, ax=plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    order=['handover','early','rotation','transition_pre','transition_post','braking_mid','braking_late','near_contact']
    for good, color, label in [(False,'#b9453d','Fixed A failed'),(True,'#24826e','Native qualified')]:
        rows=[r for r in data['scores'] if r['qualified']==good]
        ax[0].scatter([order.index(r['phase']) for r in rows], [r['leave_source_out_potential'] for r in rows],
                      c=color, label=label, s=16, alpha=.65)
    ax[0].set_xticks(range(8), order, rotation=45, ha='right')
    ax[0].set_ylabel('Potential with source excluded from fitting');ax[0].legend(fontsize=8)
    for row in data['curves']:
        ax[1].plot(row['time_s'], row['potential'], label=row['case'])
    ax[1].set_xlabel('Seconds from saved handover');ax[1].set_ylabel('Potential on archived qualified witness motion')
    ax[1].legend(fontsize=8);fig.suptitle('R4 candidate validation — no PPO training')
    for ext in ['png','pdf']:
        fig.savefig(out/f'potential-validation.{ext}', dpi=170)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['freeze','audit','plots']);args=parser.parse_args()
    {'freeze':freeze,'audit':audit,'plots':plots}[args.command]()


if __name__ == '__main__':
    main()
