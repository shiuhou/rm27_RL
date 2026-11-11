"""Check the five frozen Stage 1 engineering fixtures in real PhysX.

This is a geometry and contact preflight. It compares classifications with the
saved MuJoCo results and performs no policy optimization or native simulation.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback

import numpy as np


_ROOT = Path(__file__).resolve().parents[1]
_CONFIGS = _ROOT / "upstream/rm27_RL/configs/stage1"
_RUN = _ROOT / ".agents/runs/rm27-stage1"
_BOTTOM = "drone_1_stage1_contact"
_FIXTURE_LIMITS = {
    "center_contact": 25,
    "outside_footprint": 110,
    "wrong_aircraft_component": 130,
    "initial_overlap": 0,
    "proximity_without_contact": 5,
}


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def load_contract(native_path: Path):
    config = read_json(_CONFIGS / "task_contact_geometry.json")
    source = (_CONFIGS / "contact_surface_registration.json").read_bytes()
    if hashlib.sha256(source).hexdigest() != config["expected_contact_surface_registration_sha256"]:
        raise ValueError("Frozen contact registration changed")
    registration = json.loads(source)
    native = read_json(native_path)
    if not native["pass"] or native["private_model_sha256"] != registration["physics_model_sha256"]:
        raise ValueError("Saved native results do not validate the registered archive")
    if set(native["fixtures"]) != set(_FIXTURE_LIMITS) or not all(x["pass"] for x in native["fixtures"].values()):
        raise ValueError("Expected all five saved native fixtures to pass")
    target = registration["target_reference"]
    frame = np.column_stack((-np.array(target["slope_axis"]), target["across_axis"], target["armor_normal"]))
    frame /= np.linalg.norm(frame, axis=0)
    if not np.allclose(frame.T @ frame, np.eye(3), atol=1e-5, rtol=0) or not np.isclose(np.linalg.det(frame), 1):
        raise ValueError("Frozen target frame is not a proper rotation")
    legal = set(registration["original_registered_contact_geoms"]) | {
        r["geom"] for r in registration["additional_contact_geoms"]
    }
    return config, target, frame, legal, native


def fixture_poses(center, frame, fixture):
    """Copy the starts from tests/test_stage1_task.py's five frozen fixtures."""
    normal = frame[:, 2]
    offset = np.array(fixture.bumper_offset_body_m)
    radius = fixture.bumper_radius_m

    def contact_pose(gap, across, speed):
        position = center + across * frame[:, 1] + normal * (radius + gap) - frame @ offset
        return position, frame, -speed * normal

    return {
        "center_contact": contact_pose(.001, 0.0, .2),
        "outside_footprint": contact_pose(.030, .060, .5),
        "wrong_aircraft_component": (
            center + .12 * normal,
            frame @ np.diag([1., -1., -1.]),
            -.5 * normal,
        ),
        "initial_overlap": contact_pose(.001, .060, .2),
        "proximity_without_contact": contact_pose(.002, 0.0, 0.0),
    }


def safety_reasons(state, center, frame, limits):
    pos = np.array(state["position_world_m"])
    target_position = frame.T @ (pos - center)
    speed = np.linalg.norm(state["velocity_world_m_s"])
    rate = np.linalg.norm(state["omega_body_rad_s"])
    return {
        reason
        for condition, reason in (
            (not all(np.isfinite(x).all() for x in state.values()), "NUMERICAL_FAILURE"),
            (np.any(target_position < limits["position_min_target_frame_m"]) or
             np.any(target_position > limits["position_max_target_frame_m"]), "FLIGHT_VOLUME_EXIT"),
            (speed > limits["max_speed_m_s"], "EXCESSIVE_SPEED"),
            (rate > limits["max_body_rate_rad_s"], "EXCESSIVE_BODY_RATE"),
        )
        if condition
    }


def classify_contacts(contacts, before, center, frame, halfsize, limits):
    """Apply frozen per-contact kinematic gates to actual completed-interval reports."""
    normal = frame[:, 2]
    rotation = np.asarray(before["rotation_body_to_world"])
    omega = np.array(before["omega_body_rad_s"])
    angle = np.degrees(np.arccos(np.clip(rotation[:, 2] @ normal, -1.0, 1.0)))
    reasons = set()
    for contact in contacts:
        point = np.array(contact["position_world_m"])
        local = frame.T @ (point - center)
        velocity = before["velocity_world_m_s"] + np.cross(rotation @ omega, point - before["position_world_m"])
        vn = -float(velocity @ normal)
        vt = float(np.linalg.norm(velocity + vn * normal))
        checks = (
            (not contact["designated"], "UNDESIGNATED_CONTACT"),
            (-contact["distance_m"] > limits["max_penetration_m"], "EXCESSIVE_PENETRATION"),
            (np.any(np.abs(local[:2]) > halfsize), "CONTACT_OUTSIDE_REGION"),
            (abs(local[2]) > limits["max_plane_error_m"], "CONTACT_OUTSIDE_SURFACE_PLANE"),
            (np.asarray(contact["surface_outward_world"]) @ normal <
             np.cos(np.deg2rad(limits["max_surface_normal_error_deg"])), "WRONG_TARGET_FACE"),
            (not limits["normal_speed_range_m_s"][0] <= vn <= limits["normal_speed_range_m_s"][1],
             "CONTACT_NORMAL_SPEED"),
            (vt > limits["max_tangent_speed_m_s"], "CONTACT_TANGENT_SPEED"),
            (angle > limits["max_bottom_normal_error_deg"], "CONTACT_BOTTOM_NORMAL_ALIGNMENT"),
            (np.linalg.norm(omega) > limits["max_body_rate_rad_s"], "CONTACT_BODY_RATE"),
            (contact["normal_force_N"] <= 0.0, "NO_COMPRESSIVE_CONTACT_FORCE"),
        )
        reasons.update(reason for failure, reason in checks if failure)
        contact["gates"] = {
            "local_coordinates_m": local.tolist(),
            "normal_speed_m_s": vn,
            "tangent_speed_m_s": vt,
            "bottom_normal_error_deg": float(angle),
        }
    return reasons


def pair_set(contacts):
    return {tuple(sorted(row["geoms"])) for row in contacts}


def compare_fixture(name, result, native):
    event, reference = result["first_contact"], native["first_contact"]
    own_pairs = pair_set(event["contacts"]) if event else set()
    ref_pairs = pair_set(reference["contacts"]) if reference else set()
    classification_agrees = (
        result["scorer_pass"] == native["scorer_pass"]
        and (event is not None) == (reference is not None)
        and set(native["failure_modes"]) <= set(result["failure_modes"])
    )
    if name == "outside_footprint" and event is not None:
        classification_agrees &= "UNDESIGNATED_CONTACT" not in result["failure_modes"]
    if event is not None:
        classification_agrees &= any(point["normal_force_N"] > 0 for point in event["contacts"])
    if name == "proximity_without_contact":
        classification_agrees &= result["observed_intervals"] == _FIXTURE_LIMITS[name]
    return {
        "classification_agrees": bool(classification_agrees),
        "native_time_s": None if reference is None else reference["time_s"],
        "first_contact_time_delta_s": None if event is None or reference is None
        else event["time_s"] - reference["time_s"],
        "first_contact_pair_set_agrees": own_pairs == ref_pairs,
        "native_first_contact_pairs": sorted(ref_pairs),
        "native_failure_modes": native["failure_modes"],
    }


def run_physx(scene_path, native_path, *, device="cuda:0", check_starts=False):
    import carb
    import omni.physx
    import omni.usd
    import torch
    from isaaclab import sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.utils.math import matrix_from_quat, quat_from_matrix
    from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics

    from .rm27 import RM27Actuator, load_rm27_profile
    from .rm27_contact_scene import reference_scene

    config, target, frame, legal, native = load_contract(native_path)
    profile = load_rm27_profile()
    dt = profile.physics_dt_s
    center = np.asarray(target["armor_surface_center_m"])
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        device=device, dt=dt, render_interval=1, enable_scene_query_support=True,
    ))
    settings = carb.settings.get_settings()
    settings.set_bool("/physics/disableContactProcessing", False)
    settings.set_bool("/physics/suppressReadback", False)
    stage = omni.usd.get_context().get_stage()
    field, active = reference_scene(stage, scene_path, "/World")
    # The query twin supplies exact geometry poses to overlap_shape before any
    # integration. It has no enabled colliders and cannot affect dynamics.
    query_root = stage.DefinePrim("/World/OverlapProbe", "Xform")
    if not query_root.GetReferences().AddReference(str(scene_path.resolve()), "/Stage1Scene/Drone"):
        raise RuntimeError("Cannot reference the exact active geometry for reset overlap queries")
    query_root.RemoveAPI(UsdPhysics.RigidBodyAPI)
    query_xform = UsdGeom.Xformable(query_root)
    query_xform.ClearXformOpOrder()
    query_transform = query_xform.AddTransformOp(opSuffix="overlap")
    probes = []
    for prim in Usd.PrimRange(query_root):
        attr = prim.GetAttribute("stage1:geomName")
        if attr and attr.HasAuthoredValueOpinion():
            probes.append((prim.GetPath(), attr.Get(), prim.GetAttribute("stage1:geomId").Get()))
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
    report = PhysxSchema.PhysxContactReportAPI.Apply(active)
    report.CreateThresholdAttr().Set(0.0)

    identities = {}
    for prim in [*Usd.PrimRange(field), *Usd.PrimRange(active)]:
        attr = prim.GetAttribute("stage1:geomName")
        if not attr or not attr.HasAuthoredValueOpinion():
            continue
        geom_name = attr.Get()
        geom_id = prim.GetAttribute("stage1:geomId").Get()
        if geom_id is None or not prim.HasAPI(UsdPhysics.CollisionAPI):
            raise ValueError(f"Missing archived collider identity: {prim.GetPath()}")
        identities[str(prim.GetPath())] = (geom_name, geom_id)
    if len({geom_id for _, geom_id in identities.values()}) != len(identities):
        raise ValueError("Duplicate archived collider IDs in the loaded USD")
    if not (legal | {_BOTTOM}) <= {name for name, _ in identities.values()} or len(probes) != 247:
        raise ValueError("USD is missing registered geoms or authentic aircraft colliders")

    contact_points = []
    callback_errors = []
    report_counts = {"all_headers": 0, "active_headers": 0}
    active_path = str(active.GetPath())

    def record_reports(headers, data):
        try:
            for header in headers:
                report_counts["all_headers"] += 1
                paths = [str(PhysicsSchemaTools.intToSdfPath(key)) for key in (header.collider0, header.collider1)]
                owner = [path.startswith(active_path + "/") for path in paths]
                actors = [str(PhysicsSchemaTools.intToSdfPath(key)) for key in (header.actor0, header.actor1)]
                if not any(owner) and active_path not in actors:
                    continue
                report_counts["active_headers"] += 1
                if not header.num_contact_data:
                    continue
                names, ids = zip(*(identities[path] for path in paths), strict=True)
                for point_index in range(header.contact_data_offset,
                                         header.contact_data_offset + header.num_contact_data):
                    point = data[point_index]
                    normal = np.array(tuple(point.normal))
                    impulse = np.array(tuple(point.impulse))
                    # PhysX reports normals from shape 1 toward shape 0.
                    surface_normal = normal if owner[0] else -normal
                    contact_points.append({
                        "geom_ids": list(ids), "geoms": list(names), "collider_paths": paths,
                        "position_world_m": list(point.position),
                        "distance_m": float(point.separation),
                        "surface_outward_world": surface_normal.tolist(),
                        "impulse_world_N_s": impulse.tolist(),
                        "normal_force_N": float(normal @ impulse) / dt,
                        "designated": bool(_BOTTOM in names and legal.intersection(names)),
                    })
        except Exception as error:
            callback_errors.append(str(error))

    rigid = RigidObject(RigidObjectCfg(prim_path=active_path))
    sim_iface = omni.physx.get_physx_simulation_interface()
    subscription = sim_iface.subscribe_contact_report_events(record_reports)
    sim.reset()
    view = rigid.root_physx_view
    contact_view = sim.physics_sim_view.create_rigid_contact_view(active_path)
    torch.testing.assert_close(view.get_masses().reshape(-1), torch.tensor([profile.mass_kg]))
    inertia = view.get_inertias().reshape(3, 3).diag()
    torch.testing.assert_close(inertia, inertia.new_tensor(profile.inertia_kg_m2), atol=1e-8, rtol=1e-5)
    actuator = RM27Actuator(profile, 1, sim.device)
    indices = torch.arange(1, device=sim.device)
    wrench = torch.tensor([[actuator.hover_force_N, 0.0, 0.0, 0.0]], device=sim.device)
    query = omni.physx.get_physx_scene_query_interface()

    def state():
        data = rigid.data
        pos, quat = data.root_pos_w, data.root_quat_w
        return {
            "position_world_m": pos[0].tolist(),
            "velocity_world_m_s": data.root_lin_vel_w[0].tolist(),
            "rotation_body_to_world": matrix_from_quat(quat)[0].tolist(),
            "omega_body_rad_s": data.root_ang_vel_b[0].tolist(),
        }

    def reset(position, rotation, velocity):
        quat = quat_from_matrix(torch.from_numpy(rotation)).numpy()
        pose = torch.tensor([np.r_[position, quat].tolist()], dtype=torch.float32, device=sim.device)
        speed = torch.tensor([np.r_[velocity, np.zeros(3)].tolist()], dtype=torch.float32, device=sim.device)
        rigid.write_root_pose_to_sim(pose, indices)
        rigid.write_root_velocity_to_sim(speed, indices)
        actuator.reset(indices)
        sim.forward()
        contact_points.clear()
        callback_errors.clear()
        report_counts.update(all_headers=0, active_headers=0)

    def overlap(position, rotation):
        w, x, y, z = quat_from_matrix(torch.from_numpy(rotation)).tolist()
        transform = Gf.Matrix4d(1.0)
        transform.SetRotate(Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(x, y, z))))
        transform.SetTranslateOnly(Gf.Vec3d(*position))
        query_transform.Set(transform)
        pairs = {}
        for probe_path, probe_name, probe_id in probes:
            def collect(hit):
                hit_path = str(hit.collision)
                if hit_path.startswith(active_path + "/") or hit_path.startswith("/World/OverlapProbe/"):
                    return True
                if hit_path not in identities:
                    raise RuntimeError(f"Overlap query found unmapped collider {hit_path}")
                name, geom_id = identities[hit_path]
                pairs[(probe_id, geom_id)] = {
                    "geom_ids": [probe_id, geom_id], "geoms": [probe_name, name],
                    "collider_paths": [str(probe_path), hit_path],
                }
                return True

            query.overlap_shape(*PhysicsSchemaTools.encodeSdfPath(probe_path), collect, False)
        return list(pairs.values())

    results = {}
    period_ticks = round(profile.policy_dt_s / dt)
    for name, pose in fixture_poses(center, frame, profile).items():
        reset(*pose)
        initial = state()
        overlaps = overlap(pose[0], pose[1])
        event = None
        speculative = {"interval_count": 0, "first": None, "first_compressive": None}
        trajectory = [initial]
        reasons = safety_reasons(initial, center, frame, config["safety"])
        if overlaps:
            reasons.add("INITIAL_OVERLAP")
        saturated = False
        observed = 0
        maximum_net_force = 0.0
        first_nonzero_net_force_s = None
        for tick in range(_FIXTURE_LIMITS[name]):
            if reasons:
                break
            before = state()
            if tick % period_ticks == 0:
                _, flags = actuator.set_ctbr(wrench, rigid.data.root_ang_vel_b)
                saturated = bool(flags[0])
            force_b, torque_b = actuator.step_wrench()
            view.apply_forces_and_torques_at_position(
                force_data=force_b, torque_data=torque_b, position_data=None, indices=indices, is_global=False,
            )
            contact_points.clear()
            n_before = report_counts["all_headers"]
            sim.step(render=False)
            rigid.update(dt)
            after = state()
            trajectory.append(after)
            observed += 1
            if n_before == report_counts["all_headers"]:
                headers, data, *_ = sim_iface.get_contact_report()
                record_reports(headers, data)
            if callback_errors:
                raise RuntimeError(f"PhysX contact report could not be decoded: {callback_errors[0]}")
            net_force = float(contact_view.get_net_contact_forces(dt=dt).norm())
            maximum_net_force = max(maximum_net_force, net_force)
            if net_force > 1e-6 and first_nonzero_net_force_s is None:
                first_nonzero_net_force_s = observed * dt
            reasons |= safety_reasons(after, center, frame, config["safety"])
            positives = [point for point in contact_points if point["distance_m"] > 0]
            solids = [point for point in contact_points if point["distance_m"] <= 0]
            if positives:
                speculative["interval_count"] += 1
                if speculative["first"] is None:
                    speculative["first"] = {"time_s": observed * dt, "contacts": positives}
                if speculative["first_compressive"] is None and any(p["normal_force_N"] > 0 for p in positives):
                    speculative["first_compressive"] = {"time_s": observed * dt, "contacts": positives}
            if solids:
                reasons |= classify_contacts(
                    solids, before, center, frame, np.asarray(target["contact_region_halfsize_slope_across_m"]),
                    config["contact_acceptance"],
                )
                event = {
                    "time_s": observed * dt, "interval_start_s": (observed - 1) * dt,
                    "contacts": solids, "pre_state": before, "post_state": after,
                    "proxy_saturated": saturated, "accepted": not reasons,
                }
                break
        row = {
            "scorer_pass": event is not None and not reasons,
            "failure_modes": sorted(reasons), "first_contact": event,
            "observed_intervals": observed, "initial_overlap_pairs": overlaps,
            "initial_state": initial, "positive_separation_contacts": speculative,
            "final_state": state(), "reported_contact_headers": report_counts.copy(),
            "maximum_net_contact_force_N": maximum_net_force,
            "first_nonzero_net_contact_force_time_s": first_nonzero_net_force_s,
            "trajectory": {"state_at_every_interval_boundary": trajectory,
                           "held_ctbr_T_N_p_q_r_rad_s": wrench[0].tolist()},
        }
        row["comparison"] = compare_fixture(name, row, native["fixtures"][name])
        results[name] = row
        print(f"STAGE1_PHYSX {name}: {row['comparison']['classification_agrees']} "
              f"intervals={observed} initial_overlap={len(overlaps)} reasons={sorted(reasons)}", flush=True)

    starts = {}
    if check_starts:
        start = config["initial_state"]
        nominal = center + start["normal_standoff_m"] * frame[:, 2]
        halfwidth = np.r_[start["position_perturbation_halfwidth_target_frame_m"],
                          start["velocity_perturbation_halfwidth_world_m_s"],
                          np.deg2rad(start["rotation_vector_halfwidth_body_deg"])]
        deltas = np.zeros((19, 9))
        deltas[1::2] = -np.diag(halfwidth)
        deltas[2::2] = np.diag(halfwidth)
        for label, delta in zip(native["starts"], deltas, strict=True):
            position = nominal + frame @ delta[:3]
            angle = np.linalg.norm(delta[6:])
            rotation = np.eye(3)
            if angle:
                axis = delta[6:] / angle
                cross = np.cross(axis, np.eye(3)).T
                rotation += np.sin(angle) * cross + (1 - np.cos(angle)) * cross @ cross
            starts[label] = overlap(position, rotation)
    contact_settings = {key: settings.get(key) for key in
                        ("/physics/disableContactProcessing", "/physics/suppressReadback")}
    del subscription, contact_view, view, rigid
    passed = all(r["comparison"]["classification_agrees"] for r in results.values()) and all(
        not pairs for pairs in starts.values()
    )
    return {
        "pass": passed, "device": device, "learning_updates": 0, "physics_dt_s": dt, "policy_dt_s": profile.policy_dt_s,
        "scope": "Real PhysX fixture contacts, compared with stored native evidence. No native or learned-policy rerun.",
        "scene_sha256": hashlib.sha256(scene_path.read_bytes()).hexdigest(),
        "native_evidence_sha256": hashlib.sha256(native_path.read_bytes()).hexdigest(),
        "collider_identity_count": len(identities), "contact_settings": contact_settings,
        "fixtures": results, "reset_overlaps": starts,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=_RUN / "contact-scene.usda")
    parser.add_argument("--native-evidence", type=Path, default=_RUN / "native-preflight.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check-starts", action="store_true")
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a new evidence path")
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True, device=args.device)
    try:
        result = run_physx(args.scene, args.native_evidence, device=args.device, check_starts=args.check_starts)
        with args.output.open("x") as evidence:
            evidence.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
            evidence.flush()
            os.fsync(evidence.fileno())
        print("STAGE1_PHYSX_COMPLETE", "PASS" if result["pass"] else "FAIL", flush=True)
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(2)
    # This one-shot process has completed every simulation and evidence write.
    # Kit 4.5 can hang while shutting down a scene with contact reporting.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if result["pass"] else 2)


if __name__ == "__main__":
    main()
