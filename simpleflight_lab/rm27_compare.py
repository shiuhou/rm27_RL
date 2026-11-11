"""Matched native MuJoCo and Isaac RM27 free-flight response probes."""

import argparse
import hashlib
import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np


_ROOT = Path(__file__).resolve().parents[1]
_SIM_SOURCE = _ROOT / "upstream/rm27_sim"
_PROFILE = _SIM_SOURCE / "configs/impact_recovery/dynamics_v2_150g.json"
_VEHICLE = _SIM_SOURCE / "configs/drone_v2.json"
_TASK = _ROOT / "upstream/rm27_RL/configs/stage1/task_stage1a_seed0.json"
_SIM_REVISION = "6ec552b4a836c601295b7657c78c4bd3af977600"
_RL_REVISION = "ed4ebc550800baf032348f67af5699b44e7953f2"
_G = 9.81
_DURATION_S = 0.6
_LIMITS = {
    "position_m": 0.002,
    "velocity_m_s": 0.01,
    "attitude_deg": 0.5,
    "body_rate_rad_s": 0.05,
    "motor_command_N": 0.01,
    "motor_force_N": 0.01,
}


def read_sources():
    return tuple(json.loads(path.read_text()) for path in (_PROFILE, _VEHICLE, _TASK))


def inputs():
    profile, vehicle, task = read_sources()
    values = profile["values"]
    mg = values["mass_kg"] * _G
    policy_dt, physics_dt = task["policy_dt_s"], task["physics_dt_s"]
    n_controls = round(_DURATION_S / policy_dt)
    names = (
        "hover", "collective_up", "collective_down", "roll_positive", "roll_negative",
        "pitch_positive", "pitch_negative", "yaw_positive", "yaw_negative", "mixed_initial",
        "lower_saturation", "upper_saturation",
    )
    n = len(names)
    initial = np.zeros((n, 13), dtype=np.float64)
    initial[:, 2:4] = [2.0, 1.0]
    # Tilt and body rate force the comparison to exercise both frame
    # conversions and the gyroscopic term.
    rotvec = np.deg2rad([8.0, -4.0, 15.0])
    angle = np.linalg.norm(rotvec)
    initial[9, 3:7] = np.r_[np.cos(angle / 2), np.sin(angle / 2) * rotvec / angle]
    initial[9, 7:] = [0.2, -0.15, 0.1, 0.25, -0.3, 0.4]
    command = np.zeros((n, n_controls, 4), dtype=np.float64)
    command[:, :, 0] = mg
    normal = slice(round(0.10 / policy_dt), round(0.32 / policy_dt))
    pulse = slice(round(0.10 / policy_dt), round(0.17 / policy_dt))
    command[1, normal, 0] *= 1.25
    command[2, normal, 0] *= 0.75
    for axis in range(3):
        command[3 + 2 * axis, normal, axis + 1] = 0.75
        command[4 + 2 * axis, normal, axis + 1] = -0.75
    command[9, normal, 1:] = [0.9, -0.65, 0.4]
    command[10, pulse] = [0.04 * mg, 3.0, -2.5, 2.0]
    command[11, pulse] = [2.98 * mg, -3.0, 2.5, -2.0]
    return names, initial, command, profile, vehicle, task


def empty_trace(n, controls, substeps):
    ticks = controls * substeps
    return {
        "state": np.empty((n, ticks + 1, 13), dtype=np.float64),
        "motor_command_N": np.empty((n, controls, 4), dtype=np.float64),
        "motor_force_N": np.empty((n, ticks, 4), dtype=np.float64),
        "motor_state_N": np.empty((n, ticks + 1, 4), dtype=np.float64),
        "saturated": np.empty((n, controls), dtype=bool),
    }


def _isolated_mjcf(values, vehicle, dt):
    arm = vehicle["rotor_center_offset_m"]
    sites, motors = [], []
    for i, (x, y) in enumerate(((arm, arm), (arm, -arm), (-arm, -arm), (-arm, arm))):
        sites.append(f'<site name="rotor_{i}" pos="{x} {y} 0" size=".001"/>')
        yaw = (-1) ** i * values["yaw_torque_per_thrust_m"]
        motors.append(
            f'<general name="motor_{i}" site="rotor_{i}" gear="0 0 1 0 0 {yaw}" '
            f'ctrlrange="0 {values["max_rotor_thrust_N"]}" dyntype="filterexact" '
            f'dynprm="{values["motor_time_constant_s"]}" actearly="false"/>'
        )
    inertia = " ".join(map(str, values["inertia_kg_m2"]))
    w, h = vehicle["outer_width_m"], vehicle["outer_height_m"]
    # Retain a floor and monitor actual contacts. The envelope is a guard for
    # this airborne test. The CAD, field and contact scorer are separate inputs.
    return f"""<mujoco model="rm27_ver2_free_flight_comparison">
      <compiler autolimits="true"/>
      <option timestep="{dt}" integrator="implicitfast" gravity="0 0 -{_G}"/>
      <worldbody>
        <geom name="ground" type="plane" size="0 0 .1"/>
        <body name="rm27" pos="0 0 2">
          <freejoint name="rm27_free"/>
          <inertial pos="0 0 0" mass="{values['mass_kg']}" diaginertia="{inertia}"/>
          <geom type="box" size="{w / 2} {w / 2} {h / 2}" mass="0"/>
          {''.join(sites)}
        </body>
      </worldbody>
      <actuator>{''.join(motors)}</actuator>
    </mujoco>"""


class NativeVer2:
    """Read-only model and controller shared by both native flight probes."""

    def __init__(self, profile, vehicle, task):
        import mujoco

        # Import the pinned mixer and rate proxy from their source checkout.
        sys.path.insert(0, str(_SIM_SOURCE / "packages/physics/src"))
        from rm27_contact_physics.calibration import verify_compiled
        from rm27_contact_physics.control import motor_mixer
        from rm27_contact_physics.teacher_interface import RateThrust, apply_proxy

        values = profile["values"]
        self.dt = task["physics_dt_s"]
        self.substeps = round(task["policy_dt_s"] / self.dt)
        xml = _isolated_mjcf(values, vehicle, self.dt)
        self.model = model = mujoco.MjModel.from_xml_string(xml)
        motors = np.array([model.actuator(f"motor_{i}").id for i in range(4)])
        verify_compiled(model, model.body("rm27").id, motors, profile)
        if np.any(model.actuator_actearly[motors]) or not np.all(
            model.actuator_dyntype[motors] == mujoco.mjtDyn.mjDYN_FILTEREXACT
        ):
            raise RuntimeError("Unexpected MuJoCo actuator integration contract")
        self.inverse = np.linalg.inv(motor_mixer(vehicle["rotor_center_offset_m"], values["yaw_torque_per_thrust_m"]))
        self.inertia = np.asarray(values["inertia_kg_m2"])
        self.hover_N = values["mass_kg"] * _G
        self.max_rotor_N = values["max_rotor_thrust_N"]
        self.rate_gain = task["rate_gain"]
        self.metadata = {"mujoco_version": mujoco.__version__, "fixture_mjcf_sha256": hashlib.sha256(xml.encode()).hexdigest()}
        self._action_type, self._apply_proxy = RateThrust, apply_proxy

    def reset(self, data, state):
        import mujoco

        mujoco.mj_resetData(self.model, data)
        data.qpos[:] = state[:7]
        data.qvel[:] = state[7:]
        data.act[:] = data.ctrl[:] = self.hover_N / 4
        mujoco.mj_forward(self.model, data)

    def apply_ctbr(self, data, ctbr):
        data.ctrl[:], saturated = self._apply_proxy(
            self._action_type(ctbr[1:], ctbr[0]), data.qvel[3:], self.inertia, self.inverse, self.rate_gain,
            self.max_rotor_N, return_saturation=True,
        )
        return saturated


def run_mujoco(initial, command, profile, vehicle, task):
    import mujoco

    plant = NativeVer2(profile, vehicle, task)
    model, substeps, dt = plant.model, plant.substeps, plant.dt
    data = mujoco.MjData(model)
    trace = empty_trace(*command.shape[:2], substeps)
    for i, reset in enumerate(initial):
        plant.reset(data, reset)
        trace["state"][i, 0] = reset
        trace["motor_state_N"][i, 0] = data.act
        tick = 0
        for j, ctbr in enumerate(command[i]):
            saturated = plant.apply_ctbr(data, ctbr)
            trace["motor_command_N"][i, j] = data.ctrl
            trace["saturated"][i, j] = saturated
            for _ in range(substeps):
                mujoco.mj_step(model, data)
                if data.ncon or any(w.number for w in data.warning):
                    raise RuntimeError(f"MuJoCo contact or warning in airborne probe {i}, tick {tick}")
                trace["motor_force_N"][i, tick] = data.actuator_force
                tick += 1
                trace["state"][i, tick, :7] = data.qpos
                trace["state"][i, tick, 7:] = data.qvel
                trace["motor_state_N"][i, tick] = data.act
        if not np.isclose(data.time, _DURATION_S, atol=1e-12, rtol=0):
            raise RuntimeError("MuJoCo comparison clock mismatch")
    # Verify the integration convention on the first nonzero collective
    # step. Native MuJoCo uses pre-update force on that interval.
    tick = round(0.1 / dt)
    if not (
        np.array_equal(trace["motor_force_N"][1, tick], trace["motor_state_N"][1, tick])
        and np.any(trace["motor_state_N"][1, tick + 1] > trace["motor_state_N"][1, tick])
    ):
        raise RuntimeError("MuJoCo actuator force ordering differs from the frozen plant")
    return trace, plant.metadata


def run_isaac(initial, command, task):
    env = None
    try:
        import torch
        from isaaclab.utils.math import quat_apply

        from .env import FlightEnv, FlightEnvCfg

        cfg = FlightEnvCfg()
        cfg.airframe = "rm27"
        cfg.scene.num_envs = len(initial)
        cfg.sim.device = "cuda:0"
        env = FlightEnv(cfg)
        policy_dt, physics_dt = task["policy_dt_s"], task["physics_dt_s"]
        substeps = round(policy_dt / physics_dt)
        if env.physics_dt != physics_dt or env.step_dt != policy_dt:
            raise RuntimeError("Isaac RM27 comparison clock differs from source")
        env.reset(seed=0)
        ids = torch.arange(len(initial), device=env.device)
        reset = torch.as_tensor(initial, dtype=torch.float32, device=env.device)
        actions = torch.as_tensor(command, dtype=torch.float32, device=env.device)
        pos, quat = reset[:, :3], reset[:, 3:7]
        vel_w, omega_b = reset[:, 7:10], reset[:, 10:13]
        env._robot.write_root_pose_to_sim(torch.cat((pos + env.scene.env_origins, quat), dim=-1), ids)
        env._robot.write_root_velocity_to_sim(torch.cat((vel_w, quat_apply(quat, omega_b)), dim=-1), ids)
        env.sim.forward()
        actuator = env._actuator
        actuator.reset(ids)
        trace = {
            key: torch.empty(value.shape, device=env.device, dtype=torch.bool if value.dtype == bool else torch.float32)
            for key, value in empty_trace(*command.shape[:2], substeps).items()
        }
        tick = 0
        trace["state"][:, tick] = torch.cat(env.state(), dim=-1)
        trace["motor_state_N"][:, tick] = actuator.motor_thrust_N
        for j, ctbr in enumerate(actions.unbind(dim=1)):
            actuator.set_ctbr(ctbr, env.state()[3])
            trace["motor_command_N"][:, j] = actuator.motor_command_N
            trace["saturated"][:, j] = actuator.mixer_saturated
            for _ in range(substeps):
                trace["motor_force_N"][:, tick] = actuator.motor_thrust_N
                env._apply_wrench(*actuator.step_wrench())
                env.scene.write_data_to_sim()
                env.sim.step(render=False)
                env.scene.update(dt=physics_dt)
                tick += 1
                trace["state"][:, tick] = torch.cat(env.state(), dim=-1)
                trace["motor_state_N"][:, tick] = actuator.motor_thrust_N
        return {key: value.cpu().numpy() for key, value in trace.items()}, {"backend": "Isaac Lab 2.1.0 / Isaac Sim 4.5"}
    finally:
        if env is not None:
            env.close()


def write_summary(path, names, expected, actual, metadata):
    p, q = expected["state"], actual["state"]
    p_rot = p[..., 3:7] / np.linalg.norm(p[..., 3:7], axis=-1, keepdims=True)
    q_rot = q[..., 3:7] / np.linalg.norm(q[..., 3:7], axis=-1, keepdims=True)
    # Chord length gives the correct small-angle resolution and is invariant
    # to the sign chosen by either quaternion representation.
    chord = np.minimum(np.linalg.norm(p_rot - q_rot, axis=-1), np.linalg.norm(p_rot + q_rot, axis=-1))
    differences = {
        "position_m": np.linalg.norm(p[..., :3] - q[..., :3], axis=-1),
        "velocity_m_s": np.linalg.norm(p[..., 7:10] - q[..., 7:10], axis=-1),
        "attitude_deg": np.rad2deg(4 * np.arcsin(np.clip(chord / 2, 0, 1))),
        "body_rate_rad_s": np.linalg.norm(p[..., 10:] - q[..., 10:], axis=-1),
        "motor_command_N": np.abs(expected["motor_command_N"] - actual["motor_command_N"]).max(axis=-1),
        "motor_force_N": np.abs(expected["motor_force_N"] - actual["motor_force_N"]).max(axis=-1),
    }
    rows = {}
    for i, name in enumerate(names):
        maxima = {key: float(value[i].max()) for key, value in differences.items()}
        mismatch = int(np.count_nonzero(expected["saturated"][i] != actual["saturated"][i]))
        min_z = float(min(p[i, :, 2].min(), q[i, :, 2].min()))
        rows[name] = {
            "maximum_difference": maxima,
            "saturation_mujoco_controls": int(expected["saturated"][i].sum()),
            "saturation_isaac_controls": int(actual["saturated"][i].sum()),
            "saturation_mismatch_controls": mismatch,
            "minimum_height_m": min_z,
            "pass": all(value <= _LIMITS[key] for key, value in maxima.items()) and mismatch == 0 and min_z > 0.5,
        }
    passed = all(row["pass"] for row in rows.values())
    summary = {
        "pass": passed,
        "scope": "Isolated nominal Ver2 free-flight response. No arena or collision equivalence claimed.",
        "duration_s_per_probe": _DURATION_S,
        "limits": _LIMITS,
        "probes": rows,
        **metadata,
    }
    path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print("RM27_MATCHED_RESPONSE " + json.dumps({
        "pass": passed, "probes": len(names), "output": str(path),
        "maximum_difference": {key: float(value.max()) for key, value in differences.items()},
    }), flush=True)
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("backend", choices=("mujoco", "isaac"))
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    destination = args.output_dir / f"response-{args.backend}.npz"
    if destination.exists():
        parser.error(f"Result already exists: {destination}")
    names, initial, command, profile, vehicle, task = inputs()
    identity = {
        "sim_revision": _SIM_REVISION, "rl_revision": _RL_REVISION,
        "profile_sha256": hashlib.sha256(_PROFILE.read_bytes()).hexdigest(),
        "vehicle_config_sha256": hashlib.sha256(_VEHICLE.read_bytes()).hexdigest(),
        "task_config_sha256": hashlib.sha256(_TASK.read_bytes()).hexdigest(),
        "policy_dt_s": task["policy_dt_s"], "physics_dt_s": task["physics_dt_s"],
    }

    def save_trace(trace, runtime):
        if not all(np.isfinite(value).all() for value in trace.values()):
            raise FloatingPointError(f"{args.backend} response contains nonfinite data")
        np.savez_compressed(destination, **trace, names=names, initial=initial, command=command,
                            metadata=json.dumps({"identity": identity, "runtime": runtime}))

    if args.backend == "mujoco":
        save_trace(*run_mujoco(initial, command, profile, vehicle, task))
        print(f"RM27_MUJOCO_PROBES_COMPLETE {len(names)} {destination}")
        return 0

    with np.load(args.output_dir / "response-mujoco.npz") as data:
        expected = {key: data[key] for key in data.files}
    reference_metadata = json.loads(expected["metadata"].item())
    if not (
        reference_metadata["identity"] == identity
        and np.array_equal(expected["names"], names)
        and np.allclose(expected["command"], command, atol=1e-14, rtol=0)
        and np.allclose(expected["initial"], initial, atol=1e-14, rtol=0)
    ):
        raise ValueError("MuJoCo response uses different inputs")
    initial, command = expected["initial"], expected["command"]

    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True, device="cuda:0").app
    try:
        trace, runtime = run_isaac(initial, command, task)
        save_trace(trace, runtime)
        passed = write_summary(
            args.output_dir / "response-summary.json", names, expected, trace,
            {"identity": identity, "mujoco": reference_metadata["runtime"], "isaac": runtime},
        )
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    # Kit's fast shutdown can exit the process itself. Preserve the gate's
    # failure code after the full diagnostic has been written.
    if not passed:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(2)
    app.close(wait_for_replicator=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
