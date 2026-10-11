"""Independent S2 scorer for recorded PX4 routes; never feeds mission control.

A recovery entry may occur within two seconds of impact, followed by one
continuous second within all bounds. Contact chatter is grouped with the
existing 30 ms gap rule, never an impact-time grace window. All subsequent
contacts in the recording are checked for new armor collisions.
"""
import argparse
import hashlib
import json
from pathlib import Path
from rm27.telemetry.recording import open_recording

import numpy as np
from scipy.spatial.transform import Rotation

GATE_VERSION = 'PX4_S2_RECOVERY_V2'
HOLD_S = 1.0
WINDOW_S = 2.0
CONTACT_GAP_S = 0.03
MAX_FRAME_GAP_S = 0.025


def _tilt(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1., atol=1e-6):
        raise ValueError('invalid recorded attitude')
    r = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return float(np.rad2deg(np.arccos(np.clip(r[2, 2], -1, 1))))


def evaluate_rows(rows, impact, *, armor_geom='rm27_base_armor'):
    """Score a chronological recording, including contacts after recovery."""
    impact_time = float(impact['time_s'])
    baseline_z = float(impact['position_world_m'][2])
    hold_start = last_time = last_touch = None
    recovery_entry = confirmation = None
    longest_hold = 0.
    initial_contacts, secondary_contacts, unintended_contacts = [], [], []
    maximum = dict(tilt_deg=0., angular_rate_deg_s=0., speed_m_s=0., height_loss_m=0.)
    frame_count = 0
    failures = []
    for row in rows:
        t = float(row['timestamp'])
        if t < impact_time - 1e-8:
            continue
        if not np.isfinite(t) or (last_time is not None and t <= last_time):
            failures.append('invalid_or_nonmonotonic_recording_time')
            break
        touches = [c for c in row.get('contacts', []) if armor_geom in c.get('geoms', [])]
        if touches:
            event = dict(timestamp=t, contacts=touches)
            if last_touch is None:
                initial_contacts.append(event)
            elif t - last_touch > CONTACT_GAP_S + 1e-9:
                secondary_contacts.append(event)
            last_touch = t
        for contact in row.get('contacts', []):
            if not (armor_geom in contact.get('geoms', []) or 'rm27_dock_collision_bottom' in contact.get('geoms', [])):
                unintended_contacts.append(dict(timestamp=t, contact=contact))
        # The hold may end after the entry deadline. Earlier code clipped at
        # two seconds and accidentally counted unsafe samples within a hold.
        if t <= impact_time + WINDOW_S + HOLD_S + 1e-8:
            truth = row['truth']
            p = np.asarray(truth['position_nwu'], dtype=float)
            v = np.asarray(truth['velocity_nwu'], dtype=float)
            w = np.asarray(truth['body_rates_flu'], dtype=float)
            if any(a.shape != (3,) or not np.isfinite(a).all() for a in (p, v, w)):
                failures.append('invalid_recorded_state')
                break
            metric = dict(tilt_deg=_tilt(truth['quaternion_flu_to_nwu']),
                          angular_rate_deg_s=float(np.rad2deg(np.linalg.norm(w))),
                          speed_m_s=float(np.linalg.norm(v)),
                          height_loss_m=float(max(0., baseline_z - p[2])))
            for key, value in metric.items():
                maximum[key] = max(maximum[key], value)
            safe = (not touches and metric['tilt_deg'] <= 10. and metric['angular_rate_deg_s'] <= 30.
                    and metric['speed_m_s'] <= .3 and metric['height_loss_m'] <= .3)
            gap = last_time is not None and t - last_time > MAX_FRAME_GAP_S + 1e-9
            if not safe or gap:
                hold_start = None
            if safe:
                if hold_start is None:
                    hold_start = t
                duration = t - hold_start
                longest_hold = max(longest_hold, duration)
                if confirmation is None and hold_start - impact_time <= WINDOW_S + 1e-8 and duration >= HOLD_S - 1e-8:
                    recovery_entry, confirmation = hold_start, t
            frame_count += 1
        last_time = t
    if not frame_count:
        failures.append('missing_post_impact_frames')
    if not initial_contacts:
        failures.append('impact_not_present_in_recording')
    if confirmation is None:
        failures.append('recovery_hold_not_confirmed')
    if maximum['height_loss_m'] > .3:
        failures.append('height_loss_exceeded')
    if secondary_contacts:
        failures.append('secondary_collision')
    if unintended_contacts:
        failures.append('unintended_contact')
    return dict(passed=not failures, gate_version=GATE_VERSION,
                run_type='recorded PX4 S2 recovery evaluation', failures=failures,
                impact_time_s=impact_time, impact_normal_speed_m_s=impact.get('armor_normal_speed_m_s'),
                recovery_entry_time_s=recovery_entry, recovery_confirmation_time_s=confirmation,
                continuous_hold_s=longest_hold, maximum_post_impact=maximum,
                initial_contacts=initial_contacts, secondary_contacts=secondary_contacts,
                unintended_contacts=unintended_contacts, post_impact_frames=frame_count,
                thresholds=dict(tilt_deg=10., angular_rate_deg_s=30., speed_m_s=.3, height_loss_m=.3,
                                continuous_hold_s=HOLD_S, entry_deadline_s=WINDOW_S, contact_gap_s=CONTACT_GAP_S))


def evaluate_run(run):
    run = Path(run).resolve()
    summary = json.loads((run / 'summary.json').read_text())
    impact = (summary.get('route') or {}).get('impact')
    if summary.get('passed') is not True or (summary.get('route') or {}).get('passed') is not True or not impact:
        return dict(passed=False, gate_version=GATE_VERSION, failures=['route_did_not_complete'],
                    flight_passed=summary.get('passed') is True)
    with open_recording(run / 'frames.jsonl') as stream:
        result = evaluate_rows((json.loads(line) for line in stream), impact)
    disarmed = (summary.get('auto_disarm') or {}).get('confirmed') is True
    if not disarmed:
        result['failures'].append('automatic_disarm_not_confirmed')
        result['passed'] = False
    result.update(flight_passed=True, auto_disarm_confirmed=disarmed,
                  evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    return result


def evaluate_batch(batch, output):
    batch, output = Path(batch).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = json.loads((batch / 'summary.json').read_text())
    required = int(source['required_count'])
    attempts = []
    for attempt in source['attempts']:
        path = Path(attempt['path']).parent
        try:
            result = evaluate_run(path)
        except (ValueError, KeyError, OSError) as exc:
            result = dict(passed=False, failures=['invalid_recording'], error=str(exc))
        attempts.append(dict(attempt=attempt['attempt'], run=str(path), summary=result))
    passed_count = sum(a['summary'].get('passed') is True for a in attempts)
    manifest = dict(passed=source.get('passed') is True and len(attempts) == required and passed_count == required,
                    gate_version=GATE_VERSION, run_type='recorded PX4 S2 recovery batch evaluation',
                    attempts=attempts, passed_count=passed_count, required_count=required,
                    evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    source_summary_sha256=hashlib.sha256((batch / 'summary.json').read_bytes()).hexdigest())
    (output / 'summary.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--batch', action='store_true')
    args = parser.parse_args()
    if args.batch:
        result = evaluate_batch(args.run, args.output)
    else:
        result = evaluate_run(args.run)
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('attempts', 'initial_contacts', 'secondary_contacts', 'unintended_contacts')}, indent=2))
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
