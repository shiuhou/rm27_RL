"""Matched ASM2 free-flight cases, deterministic CTBR control and replay evidence."""

from dataclasses import asdict
from pathlib import Path
import time

import numpy as np

from simpleflight_lab.rm27_legacy import load_legacy_flight
from .asset import ROOT, read_json, write_result


def flight_settings():
    return read_json(ROOT / "configs/rm27_asm2_flight.json")["free_flight"]


def rotation_matrix(quaternion):
    """Body-to-world rotation for unit quaternions in wxyz order."""
    w, x, y, z = np.moveaxis(quaternion, -1, 0)
    return np.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                     2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=-1).reshape(
                         *quaternion.shape[:-1], 3, 3)


class FlightSuite:
    """Historical nominal Hover/Shift starts and two smooth moving references."""

    def __init__(self, profile):
        hover, shift = load_legacy_flight("hover"), load_legacy_flight("track")
        if (hover.policy_dt_s, hover.physics_dt_s) != (profile.policy_dt_s, profile.physics_dt_s):
            raise ValueError("Historical flight case timing does not match ASM2")
        self.legacy = hover
        self.groups = ("hover",) * len(hover.names) + ("shift",) * len(shift.names) + ("moving", "moving")
        self.names = (*hover.names, *shift.names, "moving_left_arc", "moving_right_arc")
        self.dt = profile.policy_dt_s
        self.steps = round(hover.horizon_s / self.dt)
        self.settings = flight_settings()
        self.moving = self.settings["moving_reference"]
        self.origin = np.array([0., 0., 1.])
        self.fixed_targets = np.vstack((hover.target, shift.target, np.zeros((2, 3)))) + self.origin
        self.initial = np.zeros((len(self.names), 13))
        for begin, cases in ((0, hover), (len(hover.names), shift)):
            rows = self.initial[begin:begin+len(cases.names)]
            rows[:, :3] = cases.position + self.origin
            rows[:, 3:7] = cases.quat
            rows[:, 7:10] = cases.velocity[:, :3]
            rows[:, 10:13] = np.einsum("nji,nj->ni", rotation_matrix(cases.quat), cases.velocity[:, 3:])
        position, velocity, _ = self.reference(0.)
        self.initial[-2:, :3] = position[-2:]
        self.initial[-2:, 3] = 1.
        self.initial[-2:, 7:10] = velocity[-2:]

    def reference(self, seconds):
        """Position, velocity and acceleration in world axes, including lookahead."""
        omega = 2*np.pi / self.moving["period_s"]
        a, b, c = (self.moving[key] for key in ("x_amplitude_m", "y_amplitude_m", "z_amplitude_m"))
        s, co = np.sin(omega*seconds), np.cos(omega*seconds)
        position = self.fixed_targets.copy()
        velocity, acceleration = np.zeros_like(position), np.zeros_like(position)
        side = np.array([1., -1.])
        position[-2:, 0] += a*s
        position[-2:, 1] += side*b*(1-co)
        position[-2:, 2] += c*s
        velocity[-2:, 0] = omega*a*co
        velocity[-2:, 1] = omega*side*b*s
        velocity[-2:, 2] = omega*c*co
        acceleration[-2:, 0] = -omega**2*a*s
        acceleration[-2:, 1] = omega**2*side*b*co
        acceleration[-2:, 2] = -omega**2*c*s
        return position, velocity, acceleration


class PositionController:
    """PD position feedback and geometric attitude feedback producing physical CTBR."""

    def __init__(self, profile, suite):
        self.profile, self.suite = profile, suite
        self.gains = suite.settings["controller"]
        self.metadata = {"name": "position_pd_ctbr", "gains": self.gains}

    def __call__(self, states, step):
        p_ref, v_ref, a_ref = self.suite.reference(step*self.suite.dt)
        r = rotation_matrix(states[:, 3:7])
        g = self.gains
        acc = (a_ref + (p_ref - states[:, :3])*g["position_kp_s2"]
               + (v_ref - states[:, 7:10])*g["velocity_kd_s"])
        acc[:, 2] += self.profile.gravity_m_s2
        acc[:, 2] = np.maximum(acc[:, 2], .5*self.profile.gravity_m_s2)
        horizontal = np.linalg.norm(acc[:, :2], axis=1)
        max_horizontal = acc[:, 2]*np.tan(np.deg2rad(g["max_tilt_deg"]))
        acc[:, :2] *= np.minimum(1., max_horizontal/np.maximum(horizontal, 1e-12))[:, None]
        z = acc / np.linalg.norm(acc, axis=1, keepdims=True)
        y = np.cross(z, (1., 0., 0.))
        y /= np.linalg.norm(y, axis=1, keepdims=True)
        desired = np.stack((np.cross(y, z), y, z), axis=-1)
        delta = r.transpose(0, 2, 1) @ desired
        error = .5*(delta-delta.transpose(0, 2, 1))
        rates = error[:, (2, 0, 1), (1, 2, 0)]*g["attitude_gain_s"]
        rates = np.clip(rates, -g["rate_limit_rad_s"], g["rate_limit_rad_s"])
        collective = self.profile.mass_kg*np.sum(acc*r[:, :, 2], axis=-1)
        lower, upper = self.profile.hover_thrust_N*np.array([1-g["collective_fraction"], 1+g["collective_fraction"]])
        return np.column_stack((np.clip(collective, lower, upper), rates))


def score(suite, states, reference, saturated):
    """Strict historical settled gates at the 5 ms recorded control instants."""
    legacy = suite.legacy
    gate = legacy.gate
    tail = slice(suite.steps - round(gate.window_s / suite.dt), None)
    distances = np.linalg.norm(states[:, :, :3] - reference, axis=-1)
    speed = np.linalg.norm(states[:, :, 7:10], axis=-1)
    rate = np.linalg.norm(states[:, :, 10:13], axis=-1)
    tilt = np.rad2deg(np.arccos(np.clip(1-2*np.sum(states[:, :, 4:6]**2, axis=-1), -1, 1)))
    local = (states[:, :, :3]-suite.origin) @ legacy.target_rotation
    local[..., 2] += legacy.normal_standoff_m
    inside = ((local >= legacy.safety_low) & (local <= legacy.safety_high)).all(axis=(0, 2))
    rows = {}
    for i, (name, group) in enumerate(zip(suite.names, suite.groups)):
        error = distances[:, i]
        settle = error[tail]
        full_rmse = float(np.sqrt(np.mean(error**2)))
        settled = {"position_peak_m": float(settle.max()), "position_rmse_m": float(np.sqrt(np.mean(settle**2))),
                   "speed_peak_m_s": float(speed[tail, i].max()), "tilt_peak_deg": float(tilt[tail, i].max()),
                   "rate_peak_rad_s": float(rate[tail, i].max())}
        qualified = (full_rmse <= suite.moving["rmse_m"] and error.max() <= suite.moving["peak_error_m"] if group == "moving"
                     else settled["position_peak_m"] <= gate.position_m and settled["speed_peak_m_s"] <= gate.speed_m_s
                     and settled["tilt_peak_deg"] <= gate.tilt_deg and settled["rate_peak_rad_s"] <= gate.body_rate_rad_s)
        in_bounds = bool(inside[i] and speed[:, i].max() <= legacy.safety_max_speed_m_s
                         and rate[:, i].max() <= legacy.safety_max_body_rate_rad_s and states[:, i, 2].min() > .1)
        rows[name] = {"group": group, "pass": bool(qualified and in_bounds), "in_bounds": in_bounds,
                      "full_rmse_m": full_rmse, "peak_error_m": float(error.max()), "final_error_m": float(error[-1]),
                      "peak_tilt_deg": float(tilt[:, i].max()), "min_altitude_m": float(states[:, i, 2].min()),
                      "settled": settled, "saturated_controls": int(saturated[:, i].sum())}
    return rows


def run_suite(backend_factory, profile, output, backend_name, controller_factory=None):
    """Execute closed-loop evaluation. Backends must advance real physics per step."""
    started = time.perf_counter()
    output = Path(output)
    json_path, npz_path = output.with_suffix(".json"), output.with_suffix(".npz")
    if json_path.exists() or npz_path.exists():
        raise FileExistsError(f"Choose a fresh evidence prefix: {output}")
    suite = FlightSuite(profile)
    n = len(suite.names)
    backend = backend_factory(profile, n)
    controller = (controller_factory or PositionController)(profile, suite)
    backend.reset(suite.initial)
    states = np.empty((suite.steps+1, n, 13))
    reference = np.stack([suite.reference(t*suite.dt)[0] for t in range(suite.steps+1)])
    commands = np.empty((suite.steps, n, 4))
    saturated = np.empty((suite.steps, n), dtype=bool)
    motors = np.empty((suite.steps+1, n, 4))
    states[0], motors[0] = backend.state(), backend.motor_thrust_N
    np.testing.assert_allclose(states[0], suite.initial, atol=3e-6, rtol=0)
    rollout_start = time.perf_counter()
    for tick in range(suite.steps):
        commands[tick] = controller(states[tick], tick)
        if not np.isfinite(commands[tick]).all():
            raise RuntimeError(f"Nonfinite CTBR command at control tick {tick}")
        saturated[tick] = backend.step(commands[tick])
        states[tick+1], motors[tick+1] = backend.state(), backend.motor_thrust_N
        if not np.isfinite(states[tick+1]).all() or not np.isfinite(motors[tick+1]).all():
            raise RuntimeError(f"Nonfinite flight or motor state at control tick {tick+1}")
    rollout_s = time.perf_counter()-rollout_start
    np.testing.assert_allclose(np.linalg.norm(states[:, :, 3:7], axis=-1), 1., atol=2e-6, rtol=0)
    cases = score(suite, states, reference, saturated)
    groups = {}
    for group in ("hover", "shift", "moving"):
        values = [row for row in cases.values() if row["group"] == group]
        groups[group] = {"pass_count": sum(row["pass"] for row in values), "cases": len(values),
                         "mean_full_rmse_m": float(np.mean([row["full_rmse_m"] for row in values])),
                         "mean_settled_rmse_m": float(np.mean([row["settled"]["position_rmse_m"] for row in values])),
                         "worst_peak_error_m": max(row["peak_error_m"] for row in values),
                         "worst_final_error_m": max(row["final_error_m"] for row in values),
                         "saturated_controls": sum(row["saturated_controls"] for row in values)}
    result = {"backend": backend_name, "plant": backend.metadata, "controller": controller.metadata,
              "profile_sha256": profile.profile_sha256, "physics_dt_s": profile.physics_dt_s, "policy_dt_s": suite.dt,
              "pass": all(row["pass"] for row in cases.values()), "groups": groups, "cases": cases,
              "gate": asdict(suite.legacy.gate), "moving_gate": suite.moving,
              "suite_scope": "Legacy starts and settled thresholds in isolated free flight, sampled at control rate; two moving arcs added",
              "rollout_wall_s": rollout_s, "total_wall_s": time.perf_counter()-started,
              "simulated_control_transitions": n*suite.steps, "training_transitions": 0, "learning_wall_s": 0.}
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    with npz_path.open("xb") as stream:
        np.savez_compressed(stream, states=states, reference=reference, names=np.asarray(suite.names), dt=suite.dt,
                            ctbr_N_rad_s=commands, mixer_saturated=saturated, motor_thrust_N=motors)
    write_result(json_path, result)
    counts = " ".join(f"{g}={v['pass_count']}/{v['cases']}" for g, v in groups.items())
    print(f"ASM2_FLIGHT {backend_name} {controller.metadata['name']} {counts} {rollout_s:.2f}s", flush=True)
    return result
