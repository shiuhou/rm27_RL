"""Independent MuJoCo plant for the ASM2 free-flight benchmark."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .asset import ROOT
from .flight import FlightProfile, allocate_ctbr, load_profile


def _flight_mjcf(profile: FlightProfile) -> str:
    source = ROOT / "assets/asm2/source/mujoco/model.xml"
    inertial = ET.parse(source).find("worldbody/body[@name='drone']/inertial")
    if inertial is None:
        raise ValueError("ASM2 source MJCF is missing the drone inertial element")
    xx, yy, zz, xy, xz, yz = map(float, inertial.attrib.pop("fullinertia").split())
    tensor = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    np.testing.assert_allclose(tensor, profile.inertia_kg_m2, rtol=0, atol=1e-12)
    # Explicit principal axes retain the pinned tensor to its source precision.
    # MuJoCo's fullinertia conversion introduces additional eigensolver error.
    moments, axes = np.linalg.eigh(tensor)
    if np.linalg.det(axes) < 0.0:
        axes[:, 0] *= -1.0
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, axes.ravel())
    inertial.set("diaginertia", " ".join(map(str, moments)))
    inertial.set("quat", " ".join(map(str, quat)))

    root = ET.Element("mujoco", model="asm2_free_flight")
    ET.SubElement(root, "option", integrator="implicitfast",
                  timestep=str(profile.physics_dt_s), gravity=f"0 0 {-profile.gravity_m_s2}")
    world = ET.SubElement(root, "worldbody")
    body = ET.SubElement(world, "body", name="drone")
    ET.SubElement(body, "freejoint", name="asm2_free")
    body.append(inertial)
    actuators = ET.SubElement(root, "actuator")
    for index, (hub, axis, yaw) in enumerate(zip(profile.hubs_m, profile.axes, profile.yaw_torque_per_thrust_m)):
        site = f"rotor_{index}"
        ET.SubElement(body, "site", name=site, pos=" ".join(map(str, hub)), size=".001")
        # The site axes coincide with FLU. MuJoCo applies thrust at the hub
        # and the signed reaction torque about the rotor axis.
        gear = " ".join(map(str, np.r_[axis, yaw * axis]))
        ET.SubElement(actuators, "general", name=f"motor_{index}", site=site, gear=gear,
                      ctrllimited="true", ctrlrange=f"0 {profile.max_rotor_thrust_N}",
                      dyntype="filterexact", dynprm=str(profile.motor_time_constant_s), actearly="false")
    return ET.tostring(root, encoding="unicode")


def _verify_compiled(model: mujoco.MjModel, profile: FlightProfile) -> dict[str, float]:
    if (model.nq, model.nv, model.nu, model.na, model.ngeom) != (7, 6, 4, 4, 0):
        raise RuntimeError("Expected one free body, four filtered motors and no contact geometry")
    if np.any(model.actuator_actearly) or np.any(model.actuator_dyntype != mujoco.mjtDyn.mjDYN_FILTEREXACT):
        raise RuntimeError("Native motors must apply current thrust before their exact lag update")

    body = model.body("drone")
    principal_axes = np.empty(9)
    mujoco.mju_quat2Mat(principal_axes, body.iquat)
    principal_axes = principal_axes.reshape(3, 3)
    inertia = principal_axes @ np.diag(body.inertia) @ principal_axes.T
    np.testing.assert_allclose(body.mass, profile.mass_kg, rtol=0, atol=1e-12)
    np.testing.assert_allclose(body.ipos, 0.0, rtol=0, atol=1e-12)
    # The original MJCF rounds its inertia coefficients to ten significant digits.
    np.testing.assert_allclose(inertia, profile.inertia_kg_m2, rtol=0, atol=1e-12)

    data = mujoco.MjData(model)
    axis = np.array([1.0, -2.0, 3.0]) / math.sqrt(14.0)
    mujoco.mju_axisAngle2Quat(data.qpos[3:7], axis, math.radians(35.0))
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, data.qpos[3:7])
    rotation = rotation.reshape(3, 3)
    omega = np.array([0.8, -0.65, 1.1])
    data.qvel[3:6] = omega
    thrust = profile.max_rotor_thrust_N / 2.0
    force_error = torque_error = acceleration_error = angular_acceleration_error = 0.0
    for index, (hub, rotor_axis, yaw) in enumerate(zip(profile.hubs_m, profile.axes, profile.yaw_torque_per_thrust_m)):
        data.act[:] = 0.0
        data.act[index] = thrust
        mujoco.mj_forward(model, data)
        force_world = rotation @ (rotor_axis * thrust)
        torque_body = (np.cross(hub, rotor_axis) + yaw * rotor_axis) * thrust
        acceleration_world = force_world / profile.mass_kg - [0.0, 0.0, profile.gravity_m_s2]
        angular_acceleration_body = np.linalg.solve(inertia, torque_body - np.cross(omega, inertia @ omega))
        force_error = max(force_error, float(np.max(np.abs(data.qfrc_actuator[:3] - force_world))))
        torque_error = max(torque_error, float(np.max(np.abs(data.qfrc_actuator[3:6] - torque_body))))
        acceleration_error = max(acceleration_error, float(np.max(np.abs(data.qacc[:3] - acceleration_world))))
        angular_acceleration_error = max(angular_acceleration_error,
                                         float(np.max(np.abs(data.qacc[3:6] - angular_acceleration_body))))
    if max(force_error, torque_error, acceleration_error, angular_acceleration_error) > 1e-9:
        raise RuntimeError("MuJoCo rotor site transmission or full-inertia dynamics disagree with the ASM2 model")

    mujoco.mj_resetData(model, data)
    trim = profile.hover_motor_thrust_N
    data.act[:] = data.ctrl[:] = trim
    mujoco.mj_forward(model, data)
    trim_acceleration = float(np.max(np.abs(data.qacc)))
    if trim_acceleration > 1e-9:
        raise RuntimeError("Native hover trim does not balance gravity and moment")
    target = np.full(4, 0.8 * profile.max_rotor_thrust_N)
    data.ctrl[:] = target
    mujoco.mj_step(model, data)
    alpha = -math.expm1(-profile.physics_dt_s / profile.motor_time_constant_s)
    lag_error = float(np.max(np.abs(data.act - (trim + alpha * (target - trim)))))
    force_order_error = float(np.max(np.abs(data.actuator_force - trim)))
    if max(lag_error, force_order_error) > 1e-12:
        raise RuntimeError("MuJoCo motor lag differs from the pre-update force contract")

    return {
        "compiled_mass_kg": float(body.mass[0]),
        "compiled_inertia_max_error_kg_m2": float(np.max(np.abs(inertia - profile.inertia_kg_m2))),
        "rotor_force_max_error_N": force_error,
        "rotor_torque_max_error_N_m": torque_error,
        "linear_acceleration_max_error_m_s2": acceleration_error,
        "angular_acceleration_max_error_rad_s2": angular_acceleration_error,
        "hover_trim_max_acceleration_abs": trim_acceleration,
        "motor_lag_one_step_max_error_N": lag_error,
        "pre_update_motor_force_max_error_N": force_order_error,
    }


class NativeFlight:
    """Native MuJoCo integration with a held CTBR allocation at each policy tick.

    State order is [position W, body-to-world wxyz, linear velocity W, body rates FLU].
    Commands are [collective N, p, q, r rad/s].
    """

    def __init__(self, profile: FlightProfile, num_envs: int):
        if num_envs < 1:
            raise ValueError("At least one native world is required")
        self.profile = profile
        self.num_envs = num_envs
        self.substeps = round(profile.policy_dt_s / profile.physics_dt_s)
        if self.substeps < 1 or not math.isclose(self.substeps * profile.physics_dt_s, profile.policy_dt_s):
            raise ValueError("Control period must be a positive integer multiple of the physics step")
        xml = _flight_mjcf(profile)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.metadata = {
            "mujoco_version": mujoco.__version__,
            "native_fixture_mjcf_sha256": hashlib.sha256(xml.encode()).hexdigest(),
            "integrator": "implicitfast",
            "motor_integration": "MuJoCo filterexact, actearly=false",
            "collision_geoms": self.model.ngeom,
            "native_plant_probes": _verify_compiled(self.model, profile),
        }
        self.data = [mujoco.MjData(self.model) for _ in range(num_envs)]

    def reset(self, states: np.ndarray) -> None:
        states = np.asarray(states, dtype=np.float64)
        if states.shape != (self.num_envs, 13) or not np.isfinite(states).all():
            raise ValueError("Expected finite initial states with shape (num_envs, 13)")
        np.testing.assert_allclose(np.linalg.norm(states[:, 3:7], axis=1), 1.0, atol=1e-6, rtol=0)
        for data, state in zip(self.data, states):
            mujoco.mj_resetData(self.model, data)
            data.qpos[:] = state[:7]
            data.qvel[:] = state[7:]
            data.act[:] = data.ctrl[:] = self.profile.hover_motor_thrust_N
            mujoco.mj_forward(self.model, data)

    def state(self) -> np.ndarray:
        return np.stack([np.r_[data.qpos, data.qvel] for data in self.data])

    @property
    def motor_thrust_N(self) -> np.ndarray:
        return np.stack([data.act for data in self.data])

    def step(self, ctbr: np.ndarray) -> np.ndarray:
        body_rates = np.stack([data.qvel[3:6] for data in self.data])
        targets, saturated = allocate_ctbr(self.profile, ctbr, body_rates)
        for world, (data, motor_target) in enumerate(zip(self.data, targets)):
            endpoint = data.time + self.profile.policy_dt_s
            data.ctrl[:] = motor_target
            mujoco.mj_step(self.model, data, nstep=self.substeps)
            if (data.ncon or any(w.number for w in data.warning)
                    or not np.isfinite(np.r_[data.qpos, data.qvel, data.act]).all()
                    or not math.isclose(data.time, endpoint, rel_tol=1e-10, abs_tol=1e-12)):
                raise RuntimeError(f"Native free-flight fault in world {world} at {endpoint:.3f} s")
        return saturated


def main() -> int:
    from .flight_suite import run_suite

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Fresh output prefix for JSON and NPZ evidence")
    parser.add_argument("--policy-weights", type=Path, help="Portable Ver2 actor weights for direct transfer evaluation")
    args = parser.parse_args()
    controller_factory = None
    if args.policy_weights is not None:
        from .flight_policy import portable_controller_factory

        controller_factory = portable_controller_factory(args.policy_weights)
    result = run_suite(NativeFlight, load_profile(), args.output, f"MuJoCo {mujoco.__version__}", controller_factory)
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
