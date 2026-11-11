"""Bounded ASM2 navigation-to-contact evaluation with an unchanged native scorer."""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import time

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from .contact_native import NativeContact
from .contact_task import ContactScorer, load_contact_spec
from .contact_witness import ContactReference, crop_coverage, endpoint_polynomial, target_mesh
from .flight import load_profile
from .flight_native import NativeFlight
from .flight_suite import rotation_matrix


CONFIG = ROOT / "configs/rm27_asm2_handover.json"


def unit_derivatives(vector, first, second):
    """Differentiate a unit vector through second order."""
    length = np.linalg.norm(vector)
    unit = vector/length
    length_rate = unit @ first
    rate = (first-length_rate*unit)/length
    length_accel = unit @ second + length*(rate @ rate)
    accel = (second-length_accel*unit-2.*length_rate*rate)/length
    return unit, rate, accel


def attitude_derivatives(force, jerk, snap):
    """Heading convention from Phase 1B with thrust, rate and acceleration axes."""
    heading = (1., 0., 0.)
    z, dz, ddz = unit_derivatives(force, jerk, snap)
    y, dy, ddy = unit_derivatives(np.cross(z, heading), np.cross(dz, heading), np.cross(ddz, heading))
    rotation = np.column_stack((np.cross(y, z), y, z))
    first = np.column_stack((np.cross(dy, z)+np.cross(y, dz), dy, dz))
    second = np.column_stack((np.cross(ddy, z)+2.*np.cross(dy, dz)+np.cross(y, ddz), ddy, ddz))
    body_rate = rotation.T @ first
    body_accel = first.T @ first + rotation.T @ second
    indices = ((2, 0, 1), (1, 2, 0))
    return (.5*(body_rate-body_rate.T))[indices], (.5*(body_accel-body_accel.T))[indices]


class MovingReference(ContactReference):
    """Phase 2 endpoint planning with the live handover's force and motion."""

    def __init__(self, profile, spec, state, motors, held_targets, duration, depth, normal_speed):
        self.profile = profile
        self.duration, self.hold_s = duration, 0.
        self.start = state[:3].copy()
        gravity = np.array([0., 0., profile.gravity_m_s2])
        rotation = rotation_matrix(state[3:7])
        specific_force = motors @ profile.axes / profile.mass_kg
        force_rate = (held_targets-motors) @ profile.axes / (profile.mass_kg*profile.motor_time_constant_s)
        start_accel = rotation @ specific_force - gravity
        start_jerk = rotation @ (force_rate+np.cross(state[10:13], specific_force))
        normal = spec.target_normal
        final = spec.target_center + (-spec.patch_face_z_m-depth)*normal
        end_accel = profile.gravity_m_s2*normal/normal[2] - gravity
        zero = np.zeros(3)
        self.derivatives = endpoint_polynomial(duration, [
            (self.start, state[7:10], start_accel, start_jerk, zero),
            (final, -normal_speed*normal, end_accel, zero, zero),
        ])
        self.gravity = gravity
        self.alpha = -math.expm1(-profile.policy_dt_s/profile.motor_time_constant_s)

    def feedforward(self, seconds):
        position, velocity, acceleration, jerk, snap = self.kinematics(seconds)
        force = acceleration+self.gravity
        omega, angular_accel = attitude_derivatives(force, jerk, snap)
        inertia = self.profile.inertia_kg_m2
        torque = inertia @ angular_accel + np.cross(omega, inertia @ omega)
        return position, velocity, force, omega, np.r_[self.profile.mass_kg*np.linalg.norm(force), torque]


def flight_limits(state, spec, limits):
    speed = float(np.linalg.norm(state[7:10]))
    rate = float(np.linalg.norm(state[10:13]))
    tilt = float(np.rad2deg(np.arccos(np.clip(1.-2.*np.sum(state[4:6]**2), -1., 1.))))
    local = (state[:3]-spec.target_center) @ spec.target_rotation
    selection = spec.field_geometry["selection"]
    if not np.isfinite(state).all():
        return "NONFINITE_STATE"
    if np.any(local < selection["safety_lower_target_frame_m"]) or np.any(local > selection["safety_upper_target_frame_m"]):
        return "FLIGHT_VOLUME_EXIT"
    if speed > limits["max_com_speed_m_s"]:
        return "SPEED_LIMIT"
    if rate > limits["max_body_rate_rad_s"]:
        return "BODY_RATE_LIMIT"
    if tilt > limits["max_tilt_deg"]:
        return "TILT_LIMIT"
    return None


def command_valid(command, profile):
    return (np.isfinite(command).all() and 0. <= command[0] <= 4.*profile.max_rotor_thrust_N
            and np.max(np.abs(command[1:])) <= profile.native_rate_limit_rad_s)


def support_points(plant):
    """Convex support samples used only to rank the free-flight previews."""
    from scipy.spatial import ConvexHull

    model = plant.model
    groups = {"body": [], "patch": []}
    for name, entry in plant.spec.entries.items():
        geom = model.geom(name).id
        mid = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        rotation = rotation_matrix(model.geom_quat[geom])
        vertices = model.mesh_vert[start:start+count] @ rotation.T + model.geom_pos[geom]
        groups["body" if entry["group"] == "body" else "patch"].append(vertices)
    cloud = np.concatenate(groups["body"])
    return cloud[ConvexHull(cloud).vertices], np.concatenate(groups["patch"])


def preview(plant, reference, initial, motors, held_targets, spec, supports, config):
    """Rank a candidate with a virtual plane crossing and a safety envelope."""
    profile = plant.profile
    plant.reset(initial[None, :])
    plant.data[0].act[:] = motors
    plant.data[0].ctrl[:] = held_targets
    normal = spec.target_normal
    body, patches = supports
    closest, prediction = float("inf"), None
    steps = math.ceil((reference.hold_s+reference.duration+config["terminal_extension_s"])/profile.policy_dt_s)
    for tick in range(steps):
        state = plant.state()[0]
        failure = flight_limits(state, spec, config["flight_envelope"])
        if failure:
            return dict(safe=False, reason=failure)
        rot = rotation_matrix(state[3:7])
        local_normal = rot.T @ normal
        offsets = patches @ local_normal
        point = patches[int(np.argmin(offsets))]
        center_gap = float(normal @ (state[:3]-spec.target_center))
        patch_gap = center_gap+float(offsets.min())
        body_gap = center_gap+float((body @ local_normal).min())
        closest = min(closest, patch_gap)
        if min(patch_gap, body_gap) <= 0.:
            velocity = state[7:10]+rot @ np.cross(state[10:13], point)
            inward = float(-normal @ velocity)
            alignment = float(np.rad2deg(np.arccos(np.clip(local_normal[2], -1., 1.))))
            lateral_speed = float(np.linalg.norm(velocity + inward*normal))
            # A sampled crossing can include both surfaces. Rank by relative
            # support and kinematics, then let the real manifold decide.
            cost = (40.*max(0., (patch_gap-body_gap)*1000.) + 6.*alignment
                    + 100.*abs(inward-config["terminal_normal_speed_m_s"])
                    + 40.*lateral_speed + 4.*np.linalg.norm(state[10:13]))
            prediction = dict(safe=True, reason="VIRTUAL_PLANE_CROSSING", rank_cost=float(cost),
                              time_s=tick*profile.policy_dt_s, body_gap_m=body_gap, patch_gap_m=patch_gap,
                              normal_speed_m_s=inward, alignment_deg=alignment)
            break
        command = reference.command(state, tick)
        if not command_valid(command, profile):
            return dict(safe=False, reason="CTBR_LIMIT")
        if plant.step(command[None, :])[0]:
            return dict(safe=False, reason="MIXER_SATURATION")
    if prediction:
        return prediction
    return dict(safe=True, reason="VIRTUAL_PLANE_MISS", closest_patch_gap_m=closest,
                rank_cost=1000.+1000.*max(0., closest))


def advance(plant, scorer, command, spec, limits):
    if not command_valid(command, plant.profile):
        return None, "CTBR_LIMIT", False
    saturated = plant.set_ctbr(command)
    if saturated and limits["max_saturated_controls"] == 0:
        return None, "MIXER_SATURATION", True
    substeps = round(plant.profile.policy_dt_s/plant.dt)
    outcome = None
    for _ in range(substeps):
        rows = plant.step_physics()
        outcome = scorer.observe(rows, plant.pre_state, plant.tick)
        if outcome is not None:
            break
    return outcome, flight_limits(plant.state(), spec, limits), saturated


def evaluate(plant, preview_plant, spec, config, supports, mesh, case, output, budget):
    """Run navigation and the chosen witness on one uninterrupted native plant."""
    from .handover_navigation import measure_handover

    profile = plant.profile
    limits = config["flight_envelope"]
    name, split, kind, start, nav_case = case
    nav_ticks = nav_case.nav_ticks if nav_case else 0
    nav_controller = nav_case.controller(profile) if nav_case else None
    scorer = ContactScorer(spec)
    outcome = scorer.observe(plant.reset(start), plant.state(), 0)
    states, motors, targets = [plant.state()], [plant.motor_thrust_N], [plant.data.ctrl.copy()]
    times, references, commands = [0.], [start[:3]], []
    saturation, failure = 0, None
    preview_rows = []

    def tick(command, target):
        nonlocal outcome, failure, saturation
        before = plant.tick
        outcome, failure, saturated = advance(plant, scorer, command, spec, limits)
        saturation += int(saturated)
        if plant.tick != before:
            commands.append(command.copy())
            states.append(plant.state())
            motors.append(plant.motor_thrust_N)
            targets.append(plant.data.ctrl.copy())
            times.append(plant.tick*plant.dt)
            references.append(np.asarray(target).copy())

    for index in range(nav_ticks):
        if outcome is not None or scorer.first_touch is not None or failure:
            failure = failure or "CONTACT_DURING_NAVIGATION"
            break
        if time.monotonic() >= budget["deadline"]:
            failure = "WALL_BUDGET"
            break
        tick(nav_controller(plant.state()[None, :], index)[0], nav_case.reference((index+1)*profile.policy_dt_s)[0][0])
    gate = len(states)-1
    if nav_ticks:
        navigation = measure_handover(nav_case, states[-1], spec)
        navigation["completed"] = gate == nav_ticks and scorer.first_touch is None
        navigation["passed"] = bool(navigation["navigation_tracking_pass"] and navigation["completed"] and not failure)
        navigation["recipe_sha256"] = nav_case.recipe_sha256
        if not navigation["passed"]:
            failure = failure or "NAVIGATION_GATE"
    else:
        navigation = dict(passed=True, completed=True, source="stationary_reference")

    selected = None
    if not failure and outcome is None:
        for duration in config["duration_grid_s"]:
            for depth in config["terminal_depth_grid_m"]:
                if budget["previews"] >= config["max_preview_rollouts"] or time.monotonic() >= budget["deadline"]:
                    break
                reference = MovingReference(profile, spec, states[-1], motors[-1], targets[-1], duration, depth,
                                            config["terminal_normal_speed_m_s"])
                row = preview(preview_plant, reference, states[-1], motors[-1], targets[-1], spec, supports, config)
                row.update(duration_s=duration, terminal_depth_m=depth)
                preview_rows.append(row)
                budget["previews"] += 1
        valid = [row for row in preview_rows if row["safe"]]
        if valid:
            selected = min(valid, key=lambda row: row["rank_cost"])
        else:
            failure = "NO_SAFE_PREVIEW" if preview_rows else "SEARCH_BUDGET"

    if selected is not None:
        reference = MovingReference(profile, spec, states[-1], motors[-1], targets[-1], selected["duration_s"],
                                    selected["terminal_depth_m"], config["terminal_normal_speed_m_s"])
        steps = math.ceil((reference.duration+config["terminal_extension_s"])/profile.policy_dt_s)
        for index in range(steps):
            if outcome is not None or failure:
                break
            if time.monotonic() >= budget["deadline"]:
                failure = "WALL_BUDGET"
                break
            target = reference.kinematics(min(reference.duration, (index+1)*profile.policy_dt_s))[0]
            tick(reference.command(plant.state(), index), target)

    outcome = outcome or scorer.finish()
    if outcome["reason"] in {"FORBIDDEN_AIRFRAME_FIRST_TOUCH", "FORBIDDEN_TARGET_FIRST_TOUCH", "INITIAL_OVERLAP"}:
        failure = failure or outcome["reason"]
    states, motors, targets = np.asarray(states), np.asarray(motors), np.asarray(targets)
    coverage = None
    try:
        coverage = crop_coverage(plant, spec, profile, states)
    except RuntimeError as error:
        failure = failure or "CROP_ENVELOPE_EXIT"
        navigation["crop_error"] = str(error)
    qualified = bool(outcome["qualified"] and not failure and navigation["passed"])
    contact = outcome["first_loaded"] or outcome["first_touch"]
    point = (max(contact["records"], key=lambda row: row["normal_impulse_N_s"])["position_world_m"]
             if contact else np.empty(0))
    normal = spec.target_normal
    tilt = np.rad2deg(np.arccos(np.clip(1.-2.*np.sum(states[:, 4:6]**2, axis=1), -1., 1.)))
    rotations = rotation_matrix(states[:, 3:7])
    pitch = np.rad2deg(np.arctan2(rotations[:, 0, 2], rotations[:, 2, 2]))
    safety = dict(passed=failure is None, failure=failure, crop_coverage=coverage,
                  mixer_saturated_controls=saturation,
                  peak_speed_m_s=float(np.linalg.norm(states[:, 7:10], axis=1).max()),
                  peak_body_rate_rad_s=float(np.linalg.norm(states[:, 10:13], axis=1).max()),
                  peak_tilt_deg=float(tilt.max()), pitch_excursion_after_handover_deg=float(np.ptp(pitch[gate:])))
    trace = output / f"{name}.npz"
    with trace.open("xb") as stream:
        np.savez_compressed(stream, states=states[:, None, :], reference=np.asarray(references)[:, None, :],
                            names=np.array([name]), dt=profile.policy_dt_s, handover_frame=gate,
                            sample_times_s=np.asarray(times), motor_thrust_N=motors[:, None, :],
                            motor_target_N=targets[:, None, :], ctbr_N_rad_s=np.asarray(commands).reshape(-1, 1, 4),
                            target_vertices_m=mesh[0], target_faces=mesh[1], contact_point_m=point,
                            contact_outcome=failure or outcome["reason"], target_center_m=spec.target_center,
                            target_rotation=spec.target_rotation)
    result = dict(case=name, split=split, kind=kind, model=plant.metadata, result=outcome,
                  qualified=qualified, reason=failure or outcome["reason"], safety=safety,
                  navigation=navigation, handover_frame=gate, handover_state=states[gate].tolist(),
                  handover_com_normal_m=float((states[gate, :3]-spec.target_center) @ normal),
                  handover_inward_speed_m_s=float(-states[gate, 7:10] @ normal),
                  handover_time_s=times[gate], end_time_s=times[-1], selected_preview=selected,
                  previews=preview_rows, trace_sha256=digest(trace), training_transitions=0)
    write_result(output / f"{name}.json", result)
    print(f"ASM2_HANDOVER {name} {result['reason']} gate={gate} end={times[-1]:.4f}s", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Fresh run directory")
    parser.add_argument("--case", help="Optional single frozen case for engineering diagnostics")
    args = parser.parse_args()
    from .handover_navigation import CONFIG as NAV_CONFIG, navigation_cases

    config = read_json(CONFIG)
    if config["training_allowed"] is not False:
        raise ValueError("Phase 3 only evaluates deterministic feasibility")
    start = time.monotonic()
    budget = {"deadline": start+config["wall_budget_s"], "previews": 0}
    profile, spec = load_profile(), load_contact_spec()
    cases = []
    for case in navigation_cases(profile, spec):
        cases.append((case.name, case.split, case.kind, case.start, case))
    for index, standoff in enumerate(config["stationary_com_normal_m"]):
        state = np.r_[spec.target_center+standoff*spec.target_normal, 1., np.zeros(9)]
        cases.append((f"stationary_{index}", "development", "stationary", state, None))
    if args.case:
        cases = [case for case in cases if case[0] == args.case]
        if not cases:
            parser.error("Unknown frozen case name")
    if len(cases) > config["max_contact_cases"]:
        raise ValueError("Frozen cases exceed the physical rollout budget")
    args.output.mkdir(parents=True, exist_ok=False)
    plant = NativeContact(profile, spec)
    proxy = NativeFlight(replace(profile, physics_dt_s=spec.dt), 1)
    supports, mesh = support_points(plant), target_mesh(plant, spec.legal_targets)
    results, unrun = [], []
    for case in cases:
        if time.monotonic() >= budget["deadline"]:
            unrun.append(case[0])
            continue
        results.append(evaluate(plant, proxy, spec, config, supports, mesh, case, args.output, budget))
    write_result(args.output / "summary.json", dict(
        schema="rm27_asm2_handover_results_v1", config_sha256=digest(CONFIG),
        navigation_config_sha256=digest(NAV_CONFIG),
        contact_contract_sha256=spec.contract_sha256, profile_sha256=profile.profile_sha256,
        budget=config, evaluated=len(results), qualified=sum(row["qualified"] for row in results),
        unrun=unrun, previews=budget["previews"], wall_s=time.monotonic()-start, training_transitions=0,
        cases=[{key: row[key] for key in ("case", "split", "kind", "qualified", "reason", "handover_com_normal_m",
                                        "handover_inward_speed_m_s", "navigation", "safety")} for row in results],
    ))
    return 0 if any(row["qualified"] for row in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
