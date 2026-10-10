"""Post-selection numerical convergence and declared one-factor challenges.

These measurements never feed back into the selected parameters. One-factor
sweeps are sensitivity checks, not probabilities under a hardware distribution.
"""
import copy
from pathlib import Path
import json

import numpy as np

from .evidence import snapshot, write_json
from .experiment import run_episode
from .model import scenario


def validate(cfg, source, output):
    methods = json.loads((Path(source)/"selection.json").read_text())["methods"]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    convergence, sensitivity = [], []
    for method, params in methods.items():
        for seed in (None, 101, 107):
            runs = []
            for dt in (.001, .0005, .00025):
                c = copy.deepcopy(cfg)
                c["physics_dt_s"] = dt
                row = run_episode(c, params, seed, output=output/"convergence"/method/f"{seed}-{dt}")
                runs.append(row)
            a, b = runs[-2:]
            impulse_error = abs(a["normal_impulse_Ns"]-b["normal_impulse_Ns"])/max(b["normal_impulse_Ns"], 1e-12)
            force_error = abs(a["peak_normal_force_N"]-b["peak_normal_force_N"])/max(b["peak_normal_force_N"], 1e-12)
            same = a["failure_modes"] == b["failure_modes"] and a["valid_contact"] == b["valid_contact"]
            convergence.append(dict(method=method, seed=seed, impulse_relative_error=impulse_error,
                                    peak_force_relative_error=force_error, same_classification=same,
                                    passed=bool(impulse_error < .05 and force_error < .1 and same)))
    cases = [("motor_tau_scale", x) for x in (.7, 1., 1.3, 1.6)]
    cases += [("thrust_scale", x) for x in (.85, 1., 1.15)]
    cases += [("friction", x) for x in (.1, .5, .9)]
    cases += [("target_y_error_m", x) for x in (-.015, 0., .015)]
    cases += [("contact_x_offset_m", x) for x in (-.008, 0., .008)]
    cases += [("contact_time_scale", x) for x in (.5, 1., 2.)]
    cases += [("target_tilt_deg", x) for x in (15., 45., 60.)]
    for method, params in methods.items():
        for i, (factor, value) in enumerate(cases):
            c = copy.deepcopy(cfg)
            physical = scenario(c)
            if factor in ("motor_tau_scale", "thrust_scale"):
                physical["draw"][factor] = value
            elif factor == "friction":
                physical["target"]["friction"] = value
            elif factor == "target_y_error_m":
                physical["target"]["center_m"][1] += value
            elif factor == "contact_x_offset_m":
                physical["contact"]["offset_body_m"][0] = value
            elif factor == "contact_time_scale":
                physical["target"]["solref_s"] *= value
            elif factor == "target_tilt_deg":
                normal = [float(np.sin(np.deg2rad(value))), 0., float(np.cos(np.deg2rad(value)))]
                c["target"]["normal"] = normal
                physical["target"]["normal"] = normal
            row = run_episode(c, params, physical=physical, output=output/"sensitivity"/method/f"case-{i:02d}")
            sensitivity.append(dict(method=method, factor=factor, value=value, passed=row["passed"],
                                    valid_contact=row["valid_contact"], failures=row["failure_modes"],
                                    recovery_time_s=row["recovery_time_s"], max_rate_rad_s=row["max_post_body_rate_rad_s"]))
    result = dict(passed=all(r["passed"] for r in convergence),
                  passed_scope="numerical convergence only; sensitivity failures are retained and expected",
                  convergence=convergence, sensitivity=sensitivity,
                  sensitivity_scope="nominal one-factor checks, no inferred real-world probability",
                  timestep_s=cfg["physics_dt_s"])
    write_json(output/"summary.json", result)
    return result
