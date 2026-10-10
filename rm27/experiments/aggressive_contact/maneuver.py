"""Dock-to-impact minimum-snap trajectory with joint event/recovery evaluation.

Interior derivatives are solved together, not set to hover at a staging point.
Contact is a physical event: the flight reference is smooth before impact;
the existing event feedback handles the velocity jump. Its retreat/lift are
optimized with flight times and terminal state against time to held recovery.
"""
import copy
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from .arena import Piecewise, compose
from .control import Septic, Skill
from .evidence import snapshot, write_json
from .experiment import run_episode
from .model import ROOT, TargetState, frame, scenario, target_frame, truth_estimate, unit
from .robust import configured


def settings():
    return json.loads((ROOT/'configs/contact_maneuver.json').read_text())


class MinimumSnap(Piecewise):
    """Solve free interior v/a/j by exact quadratic integrated-snap cost."""
    def __init__(self, start, end, waypoints, times):
        states = np.zeros((len(times)+1, 4, 3))
        states[0], states[-1] = start, end
        states[1:-1, 0] = np.asarray(waypoints).reshape(-1, 3)
        n = 4*len(states)
        H = np.zeros((n, n))
        for i, T in enumerate(times):
            # Septic linear map: endpoint derivatives -> normalized coefficients.
            A = Septic(np.eye(8)[:4], np.eye(8)[4:], T).c
            Q = np.zeros((8, 8))
            for j in range(4, 8):
                for k in range(4, 8):
                    Q[j, k] = math.factorial(j)/math.factorial(j-4)*math.factorial(k)/math.factorial(k-4)/(j+k-7)/T**7
            H[4*i:4*i+8, 4*i:4*i+8] += A.T@Q@A
        free = [4*i+d for i in range(1, len(states)-1) for d in (1, 2, 3)]
        fixed = [i for i in range(n) if i not in free]
        flat = states.reshape(n, 3)
        flat[free] = np.linalg.solve(H[np.ix_(free, free)], -H[np.ix_(free, fixed)]@flat[fixed])
        self.knot_states = states
        self.integrated_snap = float(np.sum(flat*(H@flat)))
        super().__init__(states, times)


class Joined:
    def __init__(self, first, second, split):
        self.first, self.second, self.split = first, second, split

    def sample(self, t):
        return self.first.sample(t) if t < self.split else self.second.sample(t-self.split)


class ManeuverSkill(Skill):
    def __init__(self, cfg, p, initial, target, info, slot):
        self.design = p
        local = dict(settings()['terminal'], **p.get('terminal', {}))
        n, R = target.normal, frame(target.normal)
        terminal = Skill(cfg, local, initial, target, info).trajectory.sample(local['duration_s'])
        self.method = p['method']
        if self.method == 'joint':
            times = p['segment_times_s']
            waypoints = [[slot[0]-p.get('departure_dx_m', 1.), slot[1], p['cruise_height_m']],
                         [target.position[0]+p['brake_distance_m'], p.get('brake_y_m', .5), p['brake_height_m']]]
            lead = p.get('alignment_lead_s', 0.)
            if not 0 <= lead < times[-1]:
                raise ValueError('Alignment interval must fit inside final segment')
            # A fixed thrust-direction arc is part of the global p/v/a/j curve.
            # Work backward from the contact endpoint to prescribe its entry state.
            pe, ve, ae, je = terminal
            alignment = (pe-ve*lead+.5*ae*lead**2, ve-ae*lead, ae, je)
            durations = list(times)
            durations[-1] -= lead
            path = MinimumSnap((initial.position, initial.velocity, np.zeros(3), np.zeros(3)), alignment, waypoints, durations)
            if lead:
                path = Joined(path, Septic(alignment, terminal, lead), sum(durations))
            self.split = None
            total = sum(times)
        else:
            # Exact v2 staged reference, with optional time compression of transit only.
            speed = p.get('transit_time_scale', 1.)
            def state(pos, vel):
                return (np.array(pos), np.array(vel), np.zeros(3), np.zeros(3))
            path = Piecewise([state(initial.position, initial.velocity),
                              state([slot[0]-1, slot[1], 2.6], [-1.4/speed, 0, 0]),
                              state([target.position[0]+2.268, slot[1], 2.6], [-1.4/speed, -.15/speed, -.15/speed]),
                              state(target.position+cfg['start_offset_m'], [-.3, 0, 0])], np.array([3., 6., 3.])*speed)
            self.outbound, self.split = path, 12*speed
            planned_initial = type(initial)(path.sample(self.split)[0], path.sample(self.split)[1], initial.rotation, initial.omega_body, self.split)
            second = Skill(cfg, local, planned_initial, target, info).trajectory
            path = Joined(path, second, self.split)
            total = self.split+local['duration_s']
        super().__init__(cfg, dict(local, duration_s=total), initial, target, info)
        self.trajectory, self.local, self.info = path, local, info
        self.handoff_done = False

    def command(self, state, contact):
        if self.split is not None and not self.handoff_done and state.timestamp >= self.split-1e-9:
            # Preserve the old state-based terminal initialization, for a fair comparator.
            new = Skill(self.cfg, self.local, state, self.target, self.info)
            self.trajectory = Joined(self.outbound, new.trajectory, self.split)
            self.handoff_done = True
        return super().command(state, contact)


def perturb(model, data, physical):
    """Same physical draw semantics as arena.mission; never given to actor."""
    uid = model.body('uav').id
    draw = physical['draw']
    model.body_mass[uid] *= draw['mass_scale']
    model.body_inertia[uid] *= draw['mass_scale']
    model.actuator_gear[:] *= draw['thrust_scale']
    model.actuator_dynprm[:, 0] *= draw['motor_tau_scale']
    center, normal = np.array(physical['target']['center_m']), unit(physical['target']['normal'])
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(target_frame(physical['target'])).as_quat()[[3, 0, 1, 2]]
    armor = model.geom('armor').id
    model.geom_pos[armor] = center-normal*physical['target']['dimensions_m'][2]/2
    model.geom_quat[armor] = q
    rim = model.body('research_forbidden_rim').id
    model.body_pos[rim], model.body_quat[rim] = center, q
    site = model.site('rm27_base_armor_center').id
    model.site_pos[site], model.site_quat[site] = center, q
    model.geom_pos[model.geom('bumper').id] = physical['contact']['offset_body_m']
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == uid or model.geom(g).name == 'armor' or model.geom(g).name.startswith('rim_'):
            model.geom_friction[g, 0] = physical['target']['friction']
            model.geom_solref[g, :2] = [physical['target']['solref_s'], physical['target']['solref_damping_ratio']]
    mujoco.mj_setConst(model, data)
    data.qpos[:3] += draw['start_dp']
    data.qpos[3:7] = Rotation.from_rotvec(draw['start_rotvec']).as_quat()[[3, 0, 1, 2]]
    data.qvel[:3] = draw['start_dv']
    mujoco.mj_forward(model, data)


def metrics(result, trace, normal, design):
    """Time from airborne dock initial state, ending after identical stable hold."""
    first = result['first_contact']
    out = dict(time_to_contact_s=None, time_to_recovery_entry_s=None, time_to_confirmed_recovery_s=None,
               attitude_ready_lead_s=None, attitude_error_500ms_deg=None, speed_500ms_m_s=None)
    if first:
        tc = first['time_s']
        out['time_to_contact_s'] = tc
        if result['recovery_time_s'] is not None:
            out['time_to_recovery_entry_s'] = tc+result['recovery_time_s']
            done = next(e['time_s'] for e in result['events'] if e['event'] == 'RECOVERED')
            out['time_to_confirmed_recovery_s'] = done
        from scipy.spatial.transform import Rotation
        before = trace[trace[:, 0] < tc]
        rotations = Rotation.from_quat(before[:, [5, 6, 7, 4]]).as_matrix()
        angles = np.rad2deg(np.arccos(np.clip(rotations[:, :, 2]@normal, -1, 1)))
        ready = (angles <= 10.) & (np.linalg.norm(before[:, 11:14], axis=1) <= 1.)
        bad = np.flatnonzero(~ready)
        i = bad[-1]+1 if len(bad) else 0
        out['attitude_ready_lead_s'] = float(tc-before[i, 0]) if i < len(before) else 0.
        i = int(np.argmin(abs(before[:, 0]-(tc-.5))))
        out['attitude_error_500ms_deg'] = float(angles[i])
        out['speed_500ms_m_s'] = float(np.linalg.norm(before[i, 8:11]))
        # A deliberate near-stationary staging plateau is visible separately from final impact deceleration.
        moving = trace[(trace[:, 0] > .5) & (trace[:, 0] < tc-.5)]
        out['preterminal_slow_time_s'] = float(np.sum(np.linalg.norm(moving[:, 8:11], axis=1) < .4)*.005)
    out['design'] = design
    return out


def episode(cfg, design, output, seed=None, scale=1., prepared=None, view=False, speed=1., compact=False, template=None,
            finish_on_recovery=False, prefix_s=None, action_proxy=None, after_recovery=None, skill_factory=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    c = configured(cfg, 'adaptive_nominal')
    if prepared is None:
        snapshot(output, c)
        c, bundle, slot, _ = compose(c, 'full', output)
    else:
        base_c, base_bundle, slot = prepared
        c, bundle = copy.deepcopy(base_c), (copy.deepcopy(base_bundle[0]), None, base_bundle[2], base_bundle[3])
        bundle = (bundle[0], mujoco.MjData(bundle[0]), bundle[2], bundle[3])
        bundle[1].act[:] = bundle[2]['mass_kg']*9.81/4
    model, data, info, xml = bundle
    model.opt.timestep = c['physics_dt_s']
    physical = scenario(c, seed, scale)
    perturb(model, data, physical)
    initial = truth_estimate(model, data)
    target = TargetState(np.array(c['target']['center_m']), unit(c['target']['normal']), np.zeros((3, 3)), 0.)
    skill = ManeuverSkill(c, design, initial, target, info, slot)
    if skill_factory is not None:
        skill=skill_factory(skill,initial,target,info)
    if action_proxy is not None:
        from .teacher_interface import ProjectedSkill
        skill=ProjectedSkill(skill,**action_proxy)
    c['horizon_s'] = skill.p['duration_s']+4.5
    write_json(output/'design.json', design)
    live = None
    if view:
        from .viewing import LiveView
        live = LiveView(model, data, speed)
    try:
        result = run_episode(c, skill.p, seed, scale, output=output/'flight', physical=physical,
                             plant_bundle=bundle, initial_skill=skill,
                             observe=None if live is None else lambda m, d, phase:live.update(d, phase),
                             save_model=not compact, stop_on_recovery=finish_on_recovery,stop_after_contact_s=prefix_s)
    finally:
        if live is not None:
            live.close()
    trace = np.load(output/'flight/trajectory.npz')['trace']
    if compact:
        if template is None:
            raise ValueError('Compact trial requires an immutable model template')
        from .recording import save_overlay
        save_overlay(model, output/'flight', *template)
    result.update(metrics(result, trace, unit(physical['target']['normal']), design))
    gates = settings()['flight_gates']
    result['max_altitude_m'] = float(trace[:, 3].max())
    for failed, reason in ((result['max_flight_speed_m_s'] > gates['max_speed_m_s'], 'FLIGHT_SPEED_LIMIT'),
                           (result['max_tilt_deg'] > gates['max_tilt_deg'], 'FLIGHT_TILT_LIMIT'),
                           (result['max_altitude_m'] > gates['max_height_m'], 'FLIGHT_HEIGHT_LIMIT')):
        if failed:
            result['failure_modes'].append(reason)
    result['passed'] = not result['failure_modes']
    result['scope'] = 'full RMUC airborne dock-to-contact-to-held-recovery; no return/landing; truth motors'
    result['finish_on_confirmed_recovery'] = finish_on_recovery
    result['action_proxy']=action_proxy
    if after_recovery is not None and result['passed']:
        result['return']=after_recovery(c,bundle,skill,slot,output)
    write_json(output/'summary.json', result)
    return result


def validate(cfg, output):
    import hashlib
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    assets = output/'plant'; assets.mkdir()
    c, bundle, slot, _ = compose(configured(cfg, 'adaptive_nominal'), 'full', assets)
    path = assets/'model.mjb'
    mujoco.mj_saveModel(bundle[0], str(path))
    template = (path, hashlib.sha256(path.read_bytes()).hexdigest())
    checks = []
    for name in ('fast', 'early_pose'):
        design = json.loads((ROOT/f'configs/contact_maneuver_{name}.json').read_text())
        for seed in (None, 562000):
            rows = []
            for dt in (.001, .0005, .00025):
                cc = copy.deepcopy(c); cc['physics_dt_s'] = dt
                rows.append(episode(cfg, design, output/name/f'seed-{seed}-dt-{dt}', seed, 2.,
                                    (cc, bundle, slot), compact=True, template=template, finish_on_recovery=True))
            for a, b in zip(rows[:-1], rows[1:]):
                impulse = abs(a['normal_impulse_Ns']-b['normal_impulse_Ns'])/max(1e-12,b['normal_impulse_Ns'])
                force = abs(a['peak_normal_force_N']-b['peak_normal_force_N'])/max(1e-12,b['peak_normal_force_N'])
                time_error = abs(a['time_to_confirmed_recovery_s']-b['time_to_confirmed_recovery_s'])
                same = a['failure_modes'] == b['failure_modes']
                checks.append(dict(method=name, seed=seed, impulse_relative_error=impulse, force_relative_error=force,
                                   recovery_time_difference_s=time_error, same_classification=same,
                                   passed=bool(same and impulse < .05 and force < .1 and time_error < .03)))
    result = dict(passed=all(r['passed'] for r in checks), scope='12 full-field timestep trials; not hardware calibration', checks=checks)
    write_json(output/'summary.json', result)
    return result


def candidate_pool(study, count):
    from scipy.stats import qmc
    if count < 10:
        raise ValueError('At least ten joint candidates required')
    pool = []
    for scale in (.75, .85, 1.):
        for lead in (0., .25, .45):
            p = copy.deepcopy(study['initial_design'])
            p['segment_times_s'] = [v*scale for v in p['segment_times_s']]
            p['alignment_lead_s'] = lead
            p['terminal'] = dict(aim_tangent_offset_m=.006 if lead == 0 else -.012)
            pool.append(p)
    bounds = study['bounds']
    for sample in qmc.LatinHypercube(len(bounds), seed=study['search_seed']).random(count-len(pool)):
        values = {k:float(lo+x*(hi-lo)) for (k, (lo, hi)), x in zip(bounds.items(), sample)}
        p = copy.deepcopy(study['initial_design'])
        p['segment_times_s'] = [values.pop(k) for k in ('departure_time_s', 'cruise_time_s', 'arrival_time_s')]
        p['terminal'] = {k:values.pop(k) for k in ('aim_tangent_offset_m', 'normal_speed_m_s',
                        'terminal_specific_thrust_m_s2', 'retreat_m', 'recovery_lift_m')}
        p.update(values)
        pool.append(p)
    return pool


def loss(row, contact_only=False):
    if 'RUNNER_ERROR' in row['failure_modes']:
        return 1e5
    t = row['time_to_contact_s'] if contact_only else row['time_to_confirmed_recovery_s']
    if t is None:
        t = row['parameters']['duration_s']+5
    if contact_only:
        return 100*(not row['valid_contact']) + t
    first = row['first_contact']
    margin = first['region_margin_m'] if first else -.1
    return (100*(not row['passed']) + t + 40*max(0., .012-margin)
            + 2*max(0., .3-(row['attitude_ready_lead_s'] or 0.)))


def campaign(cfg, output, candidates=None, trials=None):
    import hashlib
    from .campaign import aggregate, paired_comparison
    from .evidence import save_failure
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    c = configured(cfg, 'adaptive_nominal')
    snapshot(output, c)
    study = settings()
    count, trials = candidates or study['joint_candidates'], trials or study['evaluation_trials']
    write_json(output/'study.json', study)
    assets = output/'plant'
    assets.mkdir()
    c, bundle, slot, _ = compose(c, 'full', assets)
    path = assets/'model.mjb'
    mujoco.mj_saveModel(bundle[0], str(path))
    template = (path, hashlib.sha256(path.read_bytes()).hexdigest())
    prepared = (c, bundle, slot)
    joint = candidate_pool(study, count)
    staged = [dict(method='staged', transit_time_scale=s) for s in study['staged_time_scales']]
    write_json(output/'candidates.json', dict(joint=joint, staged=staged))
    errors = []
    def run(p, seed, scale, destination):
        try:
            return episode(cfg, p, destination, seed, scale, prepared, compact=True, template=template)
        except Exception as exc:
            errors.append(str(destination))
            return save_failure(destination, exc)
    with (output/'progress.jsonl').open('w', buffering=1) as progress:
        scores, raw_training = {}, {}
        for name, pool in (('staged', staged), ('joint', joint)):
            scores[name], raw_training[name] = [], []
            for i, p in enumerate(pool):
                rows = [run(p, seed, scale, output/'training'/name/f'candidate-{i:03d}'/f'seed-{seed}')
                        for seed, scale in study['training_cases']]
                raw_training[name].append(rows)
                score = []
                for contact_only in (False, True):
                    costs = np.array([loss(r, contact_only) for r in rows])
                    score.append(float(costs.mean()+.5*costs.max()))
                scores[name].append(score)
                progress.write(json.dumps(dict(stage='training', pool=name, candidate=i, score=score,
                                              successes=sum(r['passed'] for r in rows)))+'\n')
        best_staged = int(np.argmin(np.array(scores['staged'])[:, 0]))
        selected = np.argmin(np.array(scores['joint']), axis=0).tolist()
        methods = dict(staged_v2=staged[0], compressed_staged=staged[best_staged],
                       joint_contact_only=joint[selected[1]], joint_recovery=joint[selected[0]])
        write_json(output/'selection.json', dict(methods=methods, scores=scores, best_staged=best_staged, joint_indices=selected))
        write_json(output/'selected_design.json', methods['joint_recovery'])
        evaluations, comparisons = {}, {}
        for split, scale, seed0 in (('small', 1., study['evaluation_seed_start']), ('stress', 2., study['stress_seed_start'])):
            raw, evaluations[split] = {}, {}
            for method, p in methods.items():
                rows = []
                for seed in range(seed0, seed0+trials):
                    row = run(p, seed, scale, output/'evaluation'/split/method/f'seed-{seed}')
                    rows.append(row)
                    progress.write(json.dumps(dict(stage='evaluation', split=split, method=method, seed=seed,
                                                  passed=row['passed'], failures=row['failure_modes']))+'\n')
                raw[method] = rows
                stats = aggregate(rows)
                for key in ('time_to_contact_s', 'time_to_confirmed_recovery_s', 'attitude_ready_lead_s',
                            'attitude_error_500ms_deg', 'speed_500ms_m_s', 'preterminal_slow_time_s'):
                    values = [r[key] for r in rows if r.get(key) is not None]
                    stats[key] = None if not values else dict(n=len(values), median=float(np.median(values)),
                                                            min=float(min(values)), max=float(max(values)))
                evaluations[split][method] = stats
            comparisons[split] = {m:paired_comparison(raw[m], raw['joint_recovery']) for m in methods if m != 'joint_recovery'}
        result = dict(passed=not errors, passed_scope='execution integrity only; flight success is counted per method',
                      training_episodes=(len(staged)+len(joint))*len(study['training_cases']), evaluation_episodes=8*trials,
                      methods=methods, evaluations=evaluations, comparisons=comparisons, runner_errors=errors)
        write_json(output/'summary.json', result)
        return result


def early_pose_study(cfg, source, output, trials=None):
    """Training-only constrained selection: expose time versus early-pose tradeoff."""
    import hashlib
    from .campaign import aggregate
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    study = settings()
    pool = json.loads((source/'candidates.json').read_text())['joint']
    rule = study['early_pose_selection']
    candidates = []
    for i, p in enumerate(pool):
        rows = [json.loads((source/'training/joint'/f'candidate-{i:03d}'/f'seed-{seed}'/'summary.json').read_text())
                for seed, scale in study['training_cases']]
        if (all(r['passed'] for r in rows)
                and np.mean([r['attitude_error_500ms_deg'] for r in rows]) <= rule['max_mean_error_500ms_deg']
                and min(r['attitude_ready_lead_s'] for r in rows) >= rule['min_ready_lead_s']):
            values = [r['time_to_confirmed_recovery_s'] for r in rows]
            candidates.append((float(np.mean(values)+.5*max(values)), i))
    if not candidates:
        raise ValueError('No training candidate satisfies the declared early-pose rule')
    _, index = min(candidates)
    design = pool[index]
    write_json(output/'selection.json', dict(selected_index=index, eligible_candidates=candidates, rule=rule,
                                           source=str(source), selection='training only; frozen before this study evaluates'))
    write_json(output/'selected_design.json', design)
    assets = output/'plant'; assets.mkdir()
    c, bundle, slot, _ = compose(configured(cfg, 'adaptive_nominal'), 'full', assets)
    path = assets/'model.mjb'
    mujoco.mj_saveModel(bundle[0], str(path))
    template = (path, hashlib.sha256(path.read_bytes()).hexdigest())
    evaluations = {}
    for split, scale, seed0 in (('small', 1., study['evaluation_seed_start']), ('stress', 2., study['stress_seed_start'])):
        rows = []
        for seed in range(seed0, seed0+(trials or study['evaluation_trials'])):
            row = episode(cfg, design, output/'evaluation'/split/f'seed-{seed}', seed, scale,
                          (c, bundle, slot), compact=True, template=template)
            rows.append(row)
        stats = aggregate(rows)
        for key in ('time_to_contact_s', 'time_to_confirmed_recovery_s', 'attitude_ready_lead_s', 'attitude_error_500ms_deg', 'speed_500ms_m_s'):
            values = [r[key] for r in rows if r.get(key) is not None]
            stats[key] = None if not values else dict(median=float(np.median(values)), min=float(min(values)), max=float(max(values)))
        evaluations[split] = stats
    result = dict(passed=True, passed_scope='execution integrity; see flight success and pose metrics', evaluations=evaluations, selected_index=index)
    write_json(output/'summary.json', result)
    return result
