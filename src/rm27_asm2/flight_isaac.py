"""GPU PhysX ASM2 free-flight backend and closed-loop evaluation entry point."""

import argparse
import os
from pathlib import Path
import sys
import traceback

import numpy as np

from .flight import Actuator, load_profile
from .flight_suite import FlightSuite, rotation_matrix, run_suite


class IsaacFlight:
    """Batched rigid-body flight using the full ASM2 inertia, without colliders."""

    def __init__(self, profile, num_envs):
        import torch
        from isaaclab import sim as sim_utils
        import omni.usd
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        self.profile, self.num_envs = profile, num_envs
        self.torch = torch
        self.dt = profile.physics_dt_s
        self.substeps = round(profile.policy_dt_s / self.dt)
        self.actuator = Actuator(profile, num_envs)
        self.sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            device="cuda:0", dt=self.dt, render_interval=self.substeps,
            gravity=(0., 0., -profile.gravity_m_s2), enable_scene_query_support=False,
        ))
        stage = omni.usd.get_context().get_stage()
        inertia, basis = np.linalg.eigh(profile.inertia_kg_m2)
        if np.linalg.det(basis) < 0.:
            basis[:, -1] *= -1.
        q = Gf.Matrix3d(*basis.T.ravel().tolist()).ExtractRotation().GetQuat()
        axes = Gf.Quatf(q.GetReal(), Gf.Vec3f(*q.GetImaginary()))
        paths = [f"/World/ASM2/body_{i:03d}" for i in range(num_envs)]
        for path in paths:
            prim = UsdGeom.Xform.Define(stage, path).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            m = UsdPhysics.MassAPI.Apply(prim)
            m.CreateMassAttr(profile.mass_kg)
            m.CreateCenterOfMassAttr(Gf.Vec3f(0.))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia.tolist()))
            m.CreatePrincipalAxesAttr(axes)
            b = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            b.CreateLinearDampingAttr(0.)
            b.CreateAngularDampingAttr(0.)
            b.CreateSleepThresholdAttr(0.)
            b.CreateEnableGyroscopicForcesAttr(True)
        self.sim.reset()
        self.view = self.sim.physics_sim_view.create_rigid_body_view("/World/ASM2/body_*")
        if self.view.count != num_envs or self.view.max_shapes != 0:
            raise RuntimeError(f"Expected {num_envs} no-contact bodies, found {self.view.count}/{self.view.max_shapes}")
        self.device = self.view.get_transforms().device
        if self.device.type != "cuda":
            raise RuntimeError(f"PhysX view is not on CUDA: {self.device}")
        order = {str(path): i for i, path in enumerate(self.view.prim_paths)}
        self.order = np.array([order[path] for path in paths])
        self.indices = torch.as_tensor(self.order, dtype=torch.int32, device=self.device)
        masses = self.view.get_masses().cpu().numpy()
        inertias = self.view.get_inertias().cpu().numpy().reshape(num_envs, 3, 3)
        np.testing.assert_allclose(masses, profile.mass_kg, atol=1e-8, rtol=0)
        np.testing.assert_allclose(inertias, np.broadcast_to(profile.inertia_kg_m2, inertias.shape), atol=2e-10, rtol=0)
        np.testing.assert_allclose(self.view.get_coms().cpu().numpy()[:, :3], 0., atol=1e-8, rtol=0)
        self.metadata = {"device": str(self.device), "gpu": torch.cuda.get_device_name(self.device), "colliders": 0,
                         "body_mass_kg": float(masses.flat[0]),
                         "max_inertia_error_kg_m2": float(np.max(np.abs(inertias-profile.inertia_kg_m2))),
                         "rotor_basis_probe": self._probe_wrench()}

    @property
    def motor_thrust_N(self):
        return self.actuator.motor_thrust_N

    def _tensor(self, data):
        return self.torch.as_tensor(data, dtype=self.torch.float32, device=self.device)

    def reset(self, states):
        if states.shape != (self.num_envs, 13):
            raise ValueError("Expected ASM2 reset states in xyz/wxyz/world-v/body-omega order")
        transforms = self.view.get_transforms()
        velocities = self.view.get_velocities()
        transforms[self.indices] = self._tensor(states[:, (0, 1, 2, 4, 5, 6, 3)])
        omega_world = np.einsum("nij,nj->ni", rotation_matrix(states[:, 3:7]), states[:, 10:13])
        velocities[self.indices] = self._tensor(np.column_stack((states[:, 7:10], omega_world)))
        self.view.set_transforms(transforms, self.indices)
        self.view.set_velocities(velocities, self.indices)
        self.actuator.reset()
        self._state = self._read_state()

    def _read_state(self):
        transform = self.view.get_transforms().cpu().numpy()[self.order]
        velocity = self.view.get_velocities().cpu().numpy()[self.order]
        q = transform[:, (6, 3, 4, 5)].astype(np.float64)
        omega_body = np.einsum("nji,nj->ni", rotation_matrix(q), velocity[:, 3:])
        return np.column_stack((transform[:, :3], q, velocity[:, :3], omega_body))

    def state(self):
        return self._state.copy()

    def _wrench_tick(self, force, torque):
        # Same local-frame wrench path as the legacy RM27 Isaac environment.
        self.view.apply_forces_and_torques_at_position(force_data=self._tensor(force), torque_data=self._tensor(torque),
                                                       position_data=None, indices=self.indices, is_global=False)
        self.sim.step(render=False)

    def step(self, ctbr):
        saturated = self.actuator.set_ctbr(ctbr, self._state[:, 10:13])
        for _ in range(self.substeps):
            self._wrench_tick(*self.actuator.step_wrench())
        self._state = self._read_state()
        return saturated

    def _probe_wrench(self):
        """Measure one PhysX tick of each rotor basis at a tilted body pose."""
        n = self.num_envs
        states = np.zeros((n, 13))
        states[:, 2] = 1.
        states[:, 3:5] = (np.cos(.18), np.sin(.18))
        self.reset(states)
        rotor = np.eye(4)[np.arange(n) % 4] * .35
        self.actuator.motor_thrust_N[:] = rotor
        self.actuator.motor_command_N[:] = rotor
        self._wrench_tick(*self.actuator.step_wrench())
        observed = self._read_state()
        expected_force = np.einsum("nij,nj->ni", rotation_matrix(states[:, 3:7]), rotor @ self.profile.axes)
        measured_force = self.profile.mass_kg*(observed[:, 7:10]/self.dt + (0., 0., self.profile.gravity_m_s2))
        omega = observed[:, 10:13]
        angular_momentum = omega @ self.profile.inertia_kg_m2.T
        # The gyroscopic solve uses the updated rates. Recover the applied
        # torque from Euler's equation, including the resulting cross term.
        acceleration_term = angular_momentum/self.dt
        measured_torque = acceleration_term + np.cross(omega, angular_momentum)
        expected_torque = rotor @ self.profile.mixer[1:].T
        np.testing.assert_allclose(measured_force, expected_force, atol=2e-6, rtol=0)
        np.testing.assert_allclose(measured_torque, expected_torque, atol=2e-7, rtol=0)
        return {"tilt_rad": .36, "test_thrust_N": .35,
                "max_force_error_N": float(np.max(np.abs(measured_force-expected_force))),
                "max_uncorrected_torque_residual_Nm": float(np.max(np.abs(acceleration_term-expected_torque))),
                "max_torque_error_Nm": float(np.max(np.abs(measured_torque-expected_torque)))}


def run(args):
    profile = load_profile()
    suite = FlightSuite(profile)
    flight = IsaacFlight(profile, len(suite.names))

    def factory(_profile, n):
        if n != flight.num_envs:
            raise ValueError("Flight suite size changed")
        return flight

    baseline = run_suite(factory, profile, args.output, "Isaac Lab 2.1 / PhysX GPU")
    if args.ver2_output:
        from .flight_policy import export_weights, portable_controller_factory

        weights = args.ver2_output.with_suffix(".weights.npz")
        export_weights(weights, profile, suite)
        run_suite(factory, profile, args.ver2_output, "Isaac Lab 2.1 / PhysX GPU", portable_controller_factory(weights))
    return 0 if baseline["pass"] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="Unused evidence prefix (.json/.npz)")
    parser.add_argument("--ver2-output", type=Path, help="Also screen the pinned Ver2 Track actor")
    args = parser.parse_args()
    if args.ver2_output and (args.ver2_output.with_suffix(".json").resolve() == args.output.with_suffix(".json").resolve()
                             or args.ver2_output.with_suffix(".weights.npz").exists()):
        parser.error("Choose a distinct, unused Ver2 screen prefix")
    for output in (args.output, args.ver2_output):
        if output and (output.with_suffix(".json").exists() or output.with_suffix(".npz").exists()):
            parser.error(f"Choose an unused evidence prefix: {output}")
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True)
    try:
        code = run(args)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    # Kit teardown can stall; the bounded collision diagnostics use this exit
    # after simulation and evidence writes have finished.
    os._exit(code)


if __name__ == "__main__":
    main()
