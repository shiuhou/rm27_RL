"""Deterministic low-speed ASM2 contact witness on the provisional flight plant."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from numpy.polynomial import polynomial as poly

from .asset import write_result
from .flight import load_profile
from .flight_suite import rotation_matrix


def endpoint_polynomial(duration, boundary):
    """Fit the two position/velocity/acceleration/jerk/snap endpoints."""
    matrix, values = [], []
    basis = np.eye(10)
    for time, derivatives in enumerate(boundary):
        for order, value in enumerate(derivatives):
            matrix.append(poly.polyval(time, poly.polyder(basis, m=order)))
            values.append(value * duration**order)
    coefficients = np.linalg.solve(matrix, values)
    return tuple(poly.polyder(coefficients, m=i)/duration**i for i in range(5))


class ContactReference:
    """Smooth position reference with gravity-consistent terminal orientation.

    Both endpoints prescribe position and four derivatives. At the final
    endpoint, horizontal acceleration makes the body thrust axis coincide
    with the target normal. Jerk and snap vanish there. The contact test runs
    the actual mixer, motor lag, flight integration and collision response.
    """

    def __init__(self, profile, center, normal, *, face_height, standoff, duration, inward_speed, terminal_depth):
        self.profile = profile
        self.duration = duration
        self.hold_s = .20
        self.normal = np.asarray(normal, dtype=float)
        if (abs(self.normal[1]) > 1e-10 or self.normal[2] < .5
                or not np.isclose(np.linalg.norm(self.normal), 1.)):
            raise ValueError("Witness requires the registered inclined target normal in the XZ plane")
        self.gravity = np.array([0., 0., profile.gravity_m_s2])
        self.start = center + (face_height + standoff)*self.normal
        final = center + (face_height - terminal_depth)*self.normal
        end_accel = profile.gravity_m_s2*self.normal/self.normal[2] - self.gravity
        zeros = np.zeros(3)
        boundary = [(self.start, zeros, zeros, zeros, zeros),
                    (final, -inward_speed*self.normal, end_accel, zeros, zeros)]
        self.derivatives = endpoint_polynomial(duration, boundary)
        self.alpha = -math.expm1(-profile.policy_dt_s/profile.motor_time_constant_s)

    def kinematics(self, seconds):
        time = max(0., seconds-self.hold_s)/self.duration
        return tuple(poly.polyval(time, c) for c in self.derivatives)

    def feedforward(self, seconds):
        position, velocity, acceleration, jerk, snap = self.kinematics(seconds)
        specific_force = acceleration+self.gravity
        x, z = specific_force[[0, 2]]
        denominator = x*x+z*z
        numerator = z*jerk[0]-x*jerk[2]
        omega = np.array([0., numerator/denominator, 0.])
        angular_accel = np.array([0., (z*snap[0]-x*snap[2])/denominator
                                  - 2.*numerator*(x*jerk[0]+z*jerk[2])/denominator**2, 0.])
        inertia = self.profile.inertia_kg_m2
        torque = inertia @ angular_accel + np.cross(omega, inertia @ omega)
        wrench = np.r_[self.profile.mass_kg*np.linalg.norm(specific_force), torque]
        return position, velocity, specific_force, omega, wrench

    def command(self, state, tick):
        time = tick*self.profile.policy_dt_s
        position, velocity, force, omega, wrench = self.feedforward(time)
        next_wrench = self.feedforward(time+self.profile.policy_dt_s)[-1]
        # M is constant. Lead the exact motor filter in wrench coordinates.
        target = wrench + (next_wrench-wrench)/self.alpha
        correction = 8.*(position-state[:3]) + 5.5*(velocity-state[7:10])
        r = rotation_matrix(state[3:7])
        z = force + correction
        z /= np.linalg.norm(z)
        y = np.cross(z, (1., 0., 0.))
        y /= np.linalg.norm(y)
        desired = np.column_stack((np.cross(y, z), y, z))
        delta = r.T @ desired
        error = .5*(delta-delta.T)
        att_error = error[(2, 0, 1), (1, 2, 0)]
        inertia = self.profile.inertia_kg_m2
        rates = state[10:13]
        target[0] += self.profile.mass_kg * correction @ r[:, 2]
        target[1:] += inertia @ (90.*att_error+18.*(omega-rates))
        angular_accel = np.linalg.solve(inertia, target[1:]-np.cross(rates, inertia @ rates))
        return np.r_[target[0], rates+angular_accel/self.profile.rate_gain_per_s]

    def initial_state(self):
        return np.r_[self.start, 1., np.zeros(9)]


def simulate(plant, scorer, reference, profile):
    """Advance continuously from stationary trim through the contact event."""
    initial = plant.reset(reference.initial_state())
    outcome = scorer.observe(initial, plant.state(), 0)
    states, targets = [plant.state()], [reference.start]
    commands, saturation = [], []
    substeps = round(profile.policy_dt_s/plant.dt)
    if not math.isclose(substeps*plant.dt, profile.policy_dt_s, abs_tol=1e-12):
        raise ValueError("Witness control interval must span integer physics steps")
    steps = math.ceil((reference.hold_s+reference.duration+.06)/profile.policy_dt_s)
    for tick in range(steps):
        if outcome is not None:
            break
        ctbr = reference.command(plant.state(), tick)
        commands.append(ctbr)
        saturation.append(plant.set_ctbr(ctbr))
        for _ in range(substeps):
            rows = plant.step_physics()
            outcome = scorer.observe(rows, plant.pre_state, plant.tick)
            if outcome is not None:
                break
        states.append(plant.state())
        targets.append(reference.kinematics(min(plant.tick*plant.dt, reference.hold_s+reference.duration))[0])
    return outcome or scorer.finish(), np.asarray(states), np.asarray(targets), np.asarray(commands), saturation


def target_mesh(plant, legal_names):
    """World-space triangles of the registered geoms actually used in evaluation."""
    model, data = plant.model, plant.data
    points, triangles, offset = [], [], 0
    for name in sorted(legal_names):
        geom = model.geom(name).id
        mesh = int(model.geom_dataid[geom])
        if mesh < 0:
            raise ValueError(f"Registered replay collider has no mesh: {name}")
        vi, vn = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        fi, fn = int(model.mesh_faceadr[mesh]), int(model.mesh_facenum[mesh])
        vertices = model.mesh_vert[vi:vi+vn] @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        points.append(vertices)
        triangles.append(model.mesh_face[fi:fi+fn]+offset)
        offset += vn
    return np.concatenate(points), np.concatenate(triangles)


def crop_coverage(plant, spec, profile, states):
    """Check the witness's swept bound against the archived crop envelope."""
    selection = spec.field_geometry["selection"]
    padding = selection["padding_m_excluding_static_margin"]
    lower = np.asarray(selection["safety_lower_target_frame_m"])-padding
    upper = np.asarray(selection["safety_upper_target_frame_m"])+padding
    model = plant.model
    aircraft = ((model.geom_bodyid == model.body("drone").id)
                & ((model.geom_contype != 0) | (model.geom_conaffinity != 0)))
    indices = np.flatnonzero(aircraft)
    rotations = rotation_matrix(model.geom_quat[indices])
    reach = 0.
    for geom, rotation in zip(indices, rotations):
        mesh = model.geom_dataid[geom]
        if mesh < 0:
            raise ValueError("Expected the detailed ASM2 mesh colliders")
        begin, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        points = model.mesh_vert[begin:begin+count] @ rotation.T + model.geom_pos[geom]
        reach = max(reach, float(np.linalg.norm(points, axis=1).max()))
    # Include a full control interval of travel from every recorded sample.
    # Before impact the rotor cap and gravity bound linear acceleration.
    max_speed = np.linalg.norm(states[:, 7:10], axis=1).max()
    acceleration_bound = 4.*profile.max_rotor_thrust_N/profile.mass_kg+profile.gravity_m_s2
    travel = max_speed*profile.policy_dt_s + .5*acceleration_bound*profile.policy_dt_s**2
    radius = reach + travel + model.geom_margin[aircraft].max()
    com_local = (states[:, :3]-spec.target_center) @ spec.target_rotation
    swept_lower = com_local.min(axis=0)-radius
    swept_upper = com_local.max(axis=0)+radius
    clearance = float(min((swept_lower-lower).min(), (upper-swept_upper).min()))
    if clearance < 0:
        raise RuntimeError(f"Witness exceeds the frozen Field crop envelope by {-clearance:.6f} m")
    return dict(maximum_aircraft_vertex_radius_m=reach, minimum_envelope_clearance_m=clearance,
                swept_lower_target_m=swept_lower.tolist(), swept_upper_target_m=swept_upper.tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--standoff", type=float, default=.22, help="Initial normal clearance above nominal aligned contact, m")
    parser.add_argument("--duration", type=float, default=1.60, help="Motion time after the 0.2 s hover hold")
    parser.add_argument("--inward-speed", type=float, default=.040)
    parser.add_argument("--terminal-depth", type=float, default=.000020)
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0 for value in
           (args.standoff, args.duration, args.inward_speed, args.terminal_depth)):
        parser.error("Witness distances, duration and inward speed must be positive and finite")
    json_path, trace_path = args.output.with_suffix(".json"), args.output.with_suffix(".npz")
    if json_path.exists() or trace_path.exists():
        parser.error("Choose a fresh evidence prefix")
    from .contact_task import ContactScorer, load_contact_spec
    from .contact_native import NativeContact

    profile, spec = load_profile(), load_contact_spec()
    reference = ContactReference(profile, spec.target_center, spec.target_normal, face_height=-spec.patch_face_z_m,
                                 standoff=args.standoff,
                                 duration=args.duration, inward_speed=args.inward_speed, terminal_depth=args.terminal_depth)
    plant, scorer = NativeContact(profile, spec), ContactScorer(spec)
    outcome, states, targets, commands, saturation = simulate(plant, scorer, reference, profile)
    vertices, faces = target_mesh(plant, spec.legal_targets)
    manifold = outcome["first_loaded"] or outcome["first_touch"]
    contact_point = (max(manifold["records"], key=lambda row: row["normal_impulse_N_s"])["position_world_m"]
                     if manifold else np.empty(0))
    end_time = plant.tick*plant.dt
    sample_times = np.arange(len(states))*profile.policy_dt_s
    sample_times[-1] = end_time
    summary = dict(witness="stationary-hover deterministic CTBR", result=outcome,
                   model=plant.metadata, crop_coverage=crop_coverage(plant, spec, profile, states),
                   start=states[0].tolist(), end=states[-1].tolist(), motion_duration_s=args.duration,
                   initial_standoff_m=args.standoff, initial_hold_s=reference.hold_s,
                   requested_contact_speed_m_s=args.inward_speed, terminal_depth_m=args.terminal_depth,
                   physics_dt_s=plant.dt, control_dt_s=profile.policy_dt_s,
                   exact_end_time_s=end_time, saturated_controls=int(np.count_nonzero(saturation)),
                   profile_sha256=profile.profile_sha256, training_transitions=0)
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    with trace_path.open("xb") as output:
        np.savez_compressed(output, states=states[:, None, :], reference=targets[:, None, :],
                            names=np.array(["asm2_deterministic_contact"]), dt=profile.policy_dt_s,
                            ctbr_N_rad_s=commands.reshape(-1, 1, 4), sample_times_s=sample_times,
                            target_vertices_m=vertices, target_faces=faces, contact_point_m=contact_point,
                            contact_outcome=outcome["reason"])
    write_result(json_path, summary)
    print("ASM2_WITNESS", outcome["qualified"], outcome["reason"], f"end={plant.tick*plant.dt:.4f}s", flush=True)
    return 0 if outcome["qualified"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
