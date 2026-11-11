"""Matched detailed/proxy PhysX contact diagnostics and bounded benchmarks."""
from __future__ import annotations

import argparse
from collections import Counter
import os
from pathlib import Path
import resource
import subprocess
import sys
import traceback

import numpy as np

from .asset import ROOT, RUN, digest, load_inputs, obj_mesh, read_json, write_result
from .contacts import scorer_input, summarize
from .flight import Actuator, load_profile
from .flight_suite import rotation_matrix
from .isaac import ASM2Sim
from .proxy import load_proxy


EVIDENCE = ROOT / ".agents/runs/asm2-contact"
PROXY_MANIFEST = ROOT / "assets/asm2/proxy/manifest.json"
CONTACT_CONFIG = ROOT / "configs/rm27_asm2_contact.json"


def proxy_asset(manifest_path, output):
    """Build an isolated USD from manifest body hulls and exact reference patches."""
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils, Vt

    config, source, original, _ = load_inputs()
    manifest = load_proxy(manifest_path)
    entries = manifest["entries"]
    patches = {entry["name"]: entry["group"] for entry in original["entries"] if entry["group"] != "body"}
    if not len(patches) < len(entries) < len(original["entries"]):
        raise ValueError("Expected a reduced ASM2 body proxy and every original contact patch identity")
    detailed = ROOT / config["candidate_usd"]
    if digest(detailed) != config["candidate_sha256"]:
        raise ValueError("Detailed ASM2 USD differs from its verified pin")
    source_usd = source / "usd/drone.usdc"
    source_stage = Usd.Stage.Open(str(source_usd), load=Usd.Stage.LoadNone)
    source_drone = source_stage.GetDefaultPrim()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.)
    drone = UsdGeom.Xform.Define(stage, "/Drone").GetPrim()
    stage.SetDefaultPrim(drone)
    UsdPhysics.RigidBodyAPI.Apply(drone)
    mass = UsdPhysics.MassAPI.Apply(drone)
    source_mass = UsdPhysics.MassAPI(source_drone)
    mass.CreateMassAttr(source_mass.GetMassAttr().Get())
    mass.CreateCenterOfMassAttr(source_mass.GetCenterOfMassAttr().Get())
    mass.CreateDiagonalInertiaAttr(source_mass.GetDiagonalInertiaAttr().Get())
    mass.CreatePrincipalAxesAttr(source_mass.GetPrincipalAxesAttr().Get())
    for prim in source_drone.GetChildren():
        if prim.IsA(UsdGeom.Mesh):
            stage.OverridePrim(prim.GetPath()).GetReferences().AddReference(str(source_usd), prim.GetPath())
    stage.DefinePrim("/Drone/Collision", "Scope")
    scale = config["cook_coordinate_scale"]
    max_world_coordinate_error_m = 0.
    body_vertex_counts = []
    for entry in entries:
        name = entry["name"]
        path = Sdf.Path("/Drone/Collision").AppendChild(name)
        mesh_path = (manifest_path.parent / entry["mesh"]).resolve()
        if name in patches:
            # The detailed candidate carries the already validated thin-hull
            # cooking and exact physical patch coordinates.
            stage.OverridePrim(path).GetReferences().AddReference(str(detailed), path)
            continue
        vertices, faces = obj_mesh(mesh_path)
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or not 4 <= len(vertices) <= config["convex_hull_vertex_limit"]
                or not np.isfinite(vertices).all() or faces.ndim != 2 or faces.shape[1] != 3
                or len(faces) < 4 or np.any(faces < 0) or np.any(faces >= len(vertices))):
            raise ValueError(f"Invalid triangular hull in proxy manifest: {name}")
        body_vertex_counts.append(len(vertices))
        points = (vertices * scale).astype(np.float32)
        error = float(np.max(np.abs(points.astype(np.float64) / scale - vertices)))
        max_world_coordinate_error_m = max(max_world_coordinate_error_m, error)
        if error > 1e-8:
            raise ValueError(f"Cook normalization altered physical coordinates: {name}")
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
        mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0).tolist()), Gf.Vec3f(*points.max(0).tolist())])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.AddScaleOp().Set(Gf.Vec3f(1. / scale))
        prim = mesh.GetPrim()
        prim.SetCustomDataByKey("contact_group", entry["group"])
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        hull = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
        hull.CreateMinThicknessAttr(config["convex_min_thickness_m"])
        hull.CreateHullVertexLimitAttr(config["convex_hull_vertex_limit"])
        collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision.CreateContactOffsetAttr(config["diagnostic_contact_offset_m"])
        collision.CreateRestOffsetAttr(config["rest_offset_m"])
    layer = stage.GetRootLayer()
    layer.customLayerData = dict(scope="ASM2_PROXY_CONTACT_DIAGNOSTIC", training_allowed=False,
                                 manifest_sha256=digest(manifest_path), detailed_usd_sha256=digest(detailed))
    # Rebase the exported reference paths without changing the in-memory source.
    portable = Sdf.Layer.CreateAnonymous("asm2-proxy.usda")
    portable.TransferContent(layer)
    UsdUtils.ModifyAssetPaths(portable, lambda path: os.path.relpath(path, output.parent))
    recipe = portable.ExportToString()
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with output.open("x") as stream:
            stream.write(recipe)
    except FileExistsError:
        if output.read_text() != recipe:
            raise ValueError(f"Existing proxy USD differs from the rebuilt recipe: {output}")
    provenance = dict(manifest_sha256=digest(manifest_path), detailed_usd_sha256=digest(detailed),
                      collider_groups=dict(Counter(entry["group"] for entry in entries)),
                      body_max_vertices=max(body_vertex_counts),
                      normalization_max_world_coordinate_error_m=max_world_coordinate_error_m)
    return (output, entries), provenance


class ContactComparison(ASM2Sim):
    """Preserve notification provenance for the short dynamic fixture runs."""

    def __init__(self, *args, **kwargs):
        self.report_packets = None
        super().__init__(*args, **kwargs)

    def _contacts(self, headers, points):
        if self.report_packets is not None:
            packet = []
            for header in headers:
                if header.num_contact_data:
                    packet.append(dict(type=str(header.type), contact_data_offset=int(header.contact_data_offset),
                                       points=int(header.num_contact_data),
                                       colliders=[str(self._path_decoder(header.collider0)),
                                                  str(self._path_decoder(header.collider1))]))
            self.report_packets.append(packet)
        super()._contacts(headers, points)

    def compare(self, case, registered_targets=(), profile=None, *, command_tape=None,
                initial_motor_thrust=None, stop_at_signed_touch=False):
        self.reset(case)
        pose, velocity = self.state()
        case_tick = self.tick
        normal = np.asarray(case["rotation"])[:, 2] if case["fixture"] == "registered_field" else np.array([0., 0., 1.])
        normal = np.asarray(case.get("normal_reference_world", normal))
        steps = case.get("steps", 1)
        mass = float(self.config["expected_mass_kg"])
        trajectories = [[] for _ in range(self.worlds)]
        contacts = [[] for _ in range(self.worlds)]
        packets = {}
        actuator = Actuator(profile, self.worlds) if profile else None
        indices = self.torch.as_tensor(self.order, dtype=self.torch.int32, device=self.view_device)
        if actuator is None and (command_tape is not None or initial_motor_thrust is not None):
            raise ValueError("Recorded actuation requires the existing ASM2 flight profile")
        if profile:
            if not np.isclose(profile.physics_dt_s, self.dt, rtol=0., atol=1e-12):
                raise ValueError("Dynamic fixture requires the same physics interval as the actuator")
            command = np.broadcast_to([profile.hover_thrust_N, 0., 0., 0.], (self.worlds, 4))
            control_steps = round(profile.policy_dt_s / self.dt)
            if command_tape is not None and (command_tape.ndim != 3
                                             or command_tape.shape[1:] != (self.worlds, 4)
                                             or len(command_tape) * control_steps < steps):
                raise ValueError("CTBR command tape does not cover the simulated intervals")
            if initial_motor_thrust is not None:
                if initial_motor_thrust.shape != actuator.motor_thrust_N.shape:
                    raise ValueError("Initial motor state does not match the actor batch")
                actuator.motor_thrust_N[:] = actuator.motor_command_N[:] = initial_motor_thrust
        try:
            for index in range(steps):
                self.report_packets = []
                applied_force = np.zeros((self.worlds, 3))
                saturated = np.zeros(self.worlds, dtype=bool)
                if actuator:
                    rotation = rotation_matrix(pose[:, (6, 3, 4, 5)])
                    rates = np.einsum("nji,nj->ni", rotation, velocity[:, 3:])
                    if index % control_steps == 0:
                        controls = command if command_tape is None else command_tape[index // control_steps]
                        actuator.set_ctbr(controls, rates)
                    saturated = actuator.mixer_saturated.copy()
                    force, torque = actuator.step_wrench()
                    applied_force = np.einsum("nij,nj->ni", rotation, force)
                    applied_force[:, 2] -= mass * profile.gravity_m_s2
                    applied_torque = np.einsum("nij,nj->ni", rotation, torque)
                    self.view.apply_forces_and_torques_at_position(
                        force_data=self.torch.as_tensor(applied_force, dtype=self.torch.float32, device=self.view_device),
                        torque_data=self.torch.as_tensor(applied_torque, dtype=self.torch.float32, device=self.view_device),
                        position_data=None, indices=indices, is_global=True)
                self.step()
                next_pose, next_velocity = self.state()
                if any(self.report_packets):
                    packets[str(index + 1)] = self.report_packets
                for world, ledger in enumerate(self.ledgers):
                    before = self.pre[world]
                    summary = summarize(ledger.records)
                    contacts[world].extend(ledger.records)
                    reported = np.asarray(summary["impulse_on_drone_world_Ns"])
                    momentum = mass * (next_velocity[world, :3] - before[1]) - self.dt * applied_force[world]
                    trajectories[world].append(dict(
                        local_tick=index + 1, sample_time_s=index * self.dt,
                        interval_end_s=(index + 1) * self.dt, pre_pose_xyzw=pose[world].tolist(),
                        pre_velocity_world=velocity[world].tolist(), post_pose_xyzw=next_pose[world].tolist(),
                        post_velocity_world=next_velocity[world].tolist(), mixer_saturated=bool(saturated[world]),
                        measured_contact_momentum_Ns=momentum.tolist(), external_force_N=applied_force[world].tolist(),
                        reported_normal_impulse_Ns=float(normal @ reported),
                        measured_normal_momentum_Ns=float(normal @ momentum),
                        normal_impulse_momentum_residual_Ns=float(normal @ (reported - momentum)),
                        **summary))
                    if command_tape is not None:
                        trajectories[world][-1]["post_motor_thrust_N"] = actuator.motor_thrust_N[world].tolist()
                pose, velocity = next_pose, next_velocity
                if stop_at_signed_touch and any(rows[-1]["touching_points"] for rows in trajectories):
                    break
        finally:
            self.report_packets = None

        worlds = []
        for world, (records, trajectory) in enumerate(zip(contacts, trajectories)):
            overall = summarize(records)
            initial, touching, loaded = {}, {}, {}
            for predicate, first in ((lambda row: True, initial),
                                     (lambda row: row["distance_m"] <= 0, touching),
                                     (lambda row: row["normal_force_N"] > 0, loaded)):
                tick = next((row["tick"] for row in records if predicate(row)), None)
                if tick is not None:
                    points = [row for row in records if row["tick"] == tick]
                    interval = trajectory[tick - case_tick - 1]
                    raw_packets = packets.get(str(tick - case_tick), [])
                    prefix = self.env_paths[world] + "/"
                    first.update(interval=interval, points=points,
                                 geometric_inputs=[scorer_input(row, registered_targets) for row in points],
                                 raw_notification_packets=[[header for header in packet
                                                            if header["colliders"][0].startswith(prefix)]
                                                           for packet in raw_packets])
                    reference = interval["measured_normal_momentum_Ns"]
                    first["reported_to_measured_normal_ratio"] = (
                        interval["reported_normal_impulse_Ns"] / reference if abs(reference) > 1e-8 else None)
            touched = touching["interval"]["local_tick"] if touching else len(trajectory) + 1
            following = trajectory[touched - 1:]
            worlds.append(dict(
                **overall, classification_pass=overall["touching_groups"] == case["expected_groups"],
                first_report=initial or None, first_touching=touching or None, first_positive_load=loaded or None,
                peak_rebound_normal_speed_m_s=max((max(0., float(normal @ row["post_velocity_world"][:3]))
                                                   for row in following), default=None),
                max_abs_normal_impulse_momentum_residual_Ns=max(abs(row["normal_impulse_momentum_residual_Ns"])
                                                                for row in trajectory),
                finite=all(np.isfinite(row["post_pose_xyzw"] + row["post_velocity_world"]).all() for row in trajectory),
                trajectory=trajectory))
        actuation = "held provisional hover CTBR plus gravity" if actuator else "none, zero gravity"
        if command_tape is not None:
            actuation = "recorded CTBR commands, live rate feedback and gravity"
        return dict(case=case, normal_reference_world=normal.tolist(), actuation=actuation, worlds=worlds)


def gpu_memory():
    raw = subprocess.check_output(["nvidia-smi", "--query-compute-apps=pid,used_gpu_memory",
                                   "--format=csv,noheader,nounits"], text=True)
    values = []
    for line in raw.splitlines():
        pid, _, memory = line.partition(",")
        if pid.strip() == str(os.getpid()) and memory.strip().isdigit():
            values.append(int(memory))
    total = subprocess.check_output(["nvidia-smi", "--query-gpu=memory.used",
                                     "--format=csv,noheader,nounits"], text=True)
    return dict(process_pid=os.getpid(), process_gpu_memory_MiB=max(values, default=None),
                device_memory_used_MiB=[int(line) for line in total.splitlines() if line.strip().isdigit()])


def run(args):
    config = read_json(CONTACT_CONFIG)
    if config["training_allowed"] is not False:
        raise ValueError("This entry point is a contact diagnostic only")
    for key in ("field_fixture", "diagnostic_fixtures"):
        item = config[key]
        if digest(ROOT / item["path"]) != item["sha256"]:
            raise ValueError(f"Frozen contact fixture changed: {key}")
    override = provenance = None
    if args.geometry == "proxy":
        manifest = args.manifest.resolve()
        override, provenance = proxy_asset(manifest, EVIDENCE / f"proxy-{digest(manifest)[:12]}.usda")
    sim = ContactComparison(args.worlds, "candidate", args.layout, field=args.suite == "field",
                            asset_override=override, export_recipe=False)
    if sim.dt != config["comparison_dt_s"]:
        raise ValueError("Isaac comparison timestep differs from the prospective contact config")
    result = dict(backend="Isaac Lab 2.1 / Isaac Sim 4.5 / GPU PhysX", solver="PGS",
                  geometry=args.geometry, worlds=args.worlds, layout=args.layout, physics_dt_s=sim.dt,
                  asset=sim.asset_path, asset_sha256=sim.asset_sha256, proxy=provenance,
                  contact_config_sha256=digest(CONTACT_CONFIG), contact_offsets_m=dict(sim.authored_offsets),
                  shapes_per_aircraft=sim.view.max_shapes, stage_author_s=sim.stage_author_s, load_s=sim.load_s,
                  body_mass_kg=float(sim.mass_actual[0]), com_pose_xyzw=sim.com_actual[0].tolist(),
                  impulse_scope="Raw report impulse versus external-force-adjusted momentum. No scaling correction.",
                  task_qualification="NATIVE_SCORER_ONLY", training_transitions=0)
    result["gpu_after_load"] = gpu_memory()
    if args.suite == "field":
        case = read_json(RUN / "field-case.json")
        profile = load_profile()
        field = config["field_fixture"]
        normal = np.asarray(case["rotation"])[:, 2]
        dynamic = case | dict(name="field_clear_approach",
                              position=(np.asarray(case["position"]) + field["clearance_shift_m"] * normal).tolist(),
                              velocity=(-field["normal_approach_speed_m_s"] * normal).tolist(),
                              steps=round(field["duration_s"] / sim.dt))
        result["registered_target_count"] = len(sim.field_registered)
        result["cases"] = {case["name"]: sim.compare(case, sim.field_registered),
                           dynamic["name"]: sim.compare(dynamic, sim.field_registered, profile)}
    elif args.suite == "fixtures":
        cases = read_json(RUN / "fixtures.json")["fixtures"]
        result["cases"] = {case["name"]: sim.compare(case) for case in cases}
        result["guard"] = sim.guard_rays()
    else:
        result["benchmark"] = sim.benchmark(args.steps)
    result["duplicates_removed"] = sum(ledger.duplicates for ledger in sim.ledgers)
    result["gpu_after_suite"] = gpu_memory()
    result["max_rss_MiB"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    write_result(args.output, result)
    print(f"ASM2_CONTACT_ISAAC_COMPLETE {args.geometry} {args.suite} worlds={args.worlds} {args.output}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", choices=("detailed", "proxy"), required=True)
    parser.add_argument("--suite", choices=("fixtures", "field", "bench"), required=True)
    parser.add_argument("--manifest", type=Path, default=PROXY_MANIFEST)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--worlds", type=int, choices=(1, 8, 32), default=1)
    parser.add_argument("--layout", type=int, choices=(0, 1), default=0)
    parser.add_argument("--steps", type=int, choices=range(1, 201), default=200, metavar="1-200")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a new evidence path")
    if args.suite != "bench" and args.worlds > 8:
        parser.error("Full dynamic contact diagnostics are capped at eight worlds")
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True)
    code = 0
    try:
        run(args)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # Match the existing bounded Kit contact diagnostics.


if __name__ == "__main__":
    main()
