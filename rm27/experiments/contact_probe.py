"""Independent S1 force-sign, momentum and timestep probes; not UAV flight."""

import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from .contact_metrics import armor_contacts


def probe(dt, reverse=False, back=False):
    # A sphere impacts a frictionless fixed plate in zero gravity.
    plate = '<geom name="armor" type="box" size=".01 .5 .5"/>'
    sign = -1 if back else 1
    ball = f'<body pos="{sign*.06} 0 0"><freejoint/><geom name="ball" type="sphere" size=".02" mass=".18"/></body>'
    model = mujoco.MjModel.from_xml_string(f'''<mujoco>
      <option timestep="{dt}" integrator="RK4" gravity="0 0 0"/>
      <default><geom friction="0 0 0" solref=".01 1" solimp=".95 .99 .001"/></default>
      <worldbody>{ball+plate if reverse else plate+ball}</worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qvel[0] = -sign * .2
    initial = data.qvel[0]
    force_integral = 0.
    peak = 0.
    duration = 0.
    samples = []
    for _ in range(round(.5/dt)):
        mujoco.mj_forward(model, data)
        rows = armor_contacts(model, data, model.geom('armor').id, [1, 0, 0])
        fx = sum(r['force_on_armor_world_N'][0] for r in rows)
        force_integral += fx*dt
        peak = max(peak, sum(r['compressive_normal_force_N'] for r in rows))
        duration += dt if rows else 0
        if rows:
            samples.append({'time_s': float(data.time), 'contacts': rows})
        mujoco.mj_step(model, data)
    momentum = .18*(data.qvel[0]-initial)
    error = abs(force_integral+momentum)/max(abs(momentum), 1e-12)
    return dict(dt=dt, reverse=reverse, back=back, impulse_on_armor_x_Ns=force_integral,
                sphere_momentum_change_x_Ns=float(momentum), momentum_relative_error=error,
                peak_compressive_normal_N=peak, contact_duration_s=duration,
                passed=bool(duration > 0 and error < .05 and ((peak == 0) if back else (peak > 0))),
                samples=samples)


def run(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    runs = [probe(dt, reverse, back) for reverse in (False, True)
            for back in (False, True) for dt in (.002, .001, .0005, .00025, .000125)]
    for i, row in enumerate(runs):
        (output/f'probe-{i:02d}.json').write_text(json.dumps(row, indent=2))
        row.pop('samples')
    convergence = []
    for reverse in (False, True):
        a, b = [r for r in runs if r['reverse'] == reverse and not r['back']][-2:]
        impulse_error = abs(a['impulse_on_armor_x_Ns']/b['impulse_on_armor_x_Ns']-1)
        peak_error = abs(a['peak_compressive_normal_N']/b['peak_compressive_normal_N']-1)
        convergence.append(dict(reverse=reverse, impulse_relative_difference=impulse_error,
                                peak_relative_difference=peak_error,
                                passed=impulse_error <= .05 and peak_error <= .1))
    selected = [r for r in runs if r['dt'] == .000125]
    result = dict(passed=all(r['passed'] for r in selected+convergence), probes=runs,
                  accepted_timestep_s=.000125,
                  convergence=convergence, scope='isolated frictionless sphere; read-only force measurement',
                  s1_complete=False, limitations='No UAV recovery, corridor, referee or structural certification')
    (output/'summary.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = run(args.output)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
