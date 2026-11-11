"""Isolated Isaac Lab ASM2 collider tests and bounded parallel benchmarks."""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import resource
import sys
import time
import traceback

import numpy as np

from .asset import ROOT, RUN, digest, load_inputs, read_json, write_result
from .contacts import ContactLedger, contact_record, scorer_input, summarize


class ASM2Sim:
    def __init__(self, worlds, mode, layout=0, field=False, *, asset_override=None, export_recipe=True,
                 physics_dt=None, contact_offset_m=None):
        import carb
        import omni.physx
        import omni.usd
        import torch
        from isaaclab import sim as sim_utils
        from isaacsim.core.cloner import Cloner
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

        self.config, source, geometry, self.mass = load_inputs()
        self.worlds, self.mode, self.tick = worlds, mode, 0
        self.dt = self.config["physics_dt_s"] if physics_dt is None else float(physics_dt)
        if contact_offset_m is not None and not 0. < contact_offset_m <= self.config["diagnostic_contact_offset_m"]:
            raise ValueError("The contact offset override must be positive and at most the verified offset")
        self.torch, self.Gf = torch, Gf
        self._path_decoder = PhysicsSchemaTools.intToSdfPath
        entries = geometry["entries"] if asset_override is None else asset_override[1]
        self._entries = {entry["name"]: entry for entry in entries}
        if len(self._entries) != len(entries):
            raise ValueError("ASM2 collider identities must be unique")
        self._paths, self.callback_s, self.callback_error = {}, 0., None
        self.ledgers = [ContactLedger() for _ in range(worlds)]
        self._subscription = None
        self.collection_mode = "full"
        self.raw_headers = self.empty_headers = 0
        self.pre = {}
        started = time.perf_counter()
        config = sim_utils.SimulationCfg(device="cuda:0", dt=self.dt, render_interval=1,
                                         gravity=(0., 0., 0.), enable_scene_query_support=True)
        config.physx.solver_type = 0  # PGS, scoped to this candidate only.
        self.sim = sim_utils.SimulationContext(config)
        settings = carb.settings.get_settings()
        settings.set_bool("/physics/disableContactProcessing", False)
        settings.set_bool("/physics/suppressReadback", False)
        stage = omni.usd.get_context().get_stage()
        self.stage = stage
        asset_path = source / "usd/drone.usdc"
        if asset_override is not None:
            if mode != "candidate":
                raise ValueError("An alternate ASM2 asset requires the candidate cooking mode")
            asset_path = asset_override[0]
        elif mode == "candidate":
            asset_path = ROOT / self.config["candidate_usd"]
            if digest(asset_path) != self.config["candidate_sha256"]:
                raise ValueError("Standalone ASM2 candidate changed after export validation")
        self.asset_path, self.asset_sha256 = str(asset_path.relative_to(ROOT)), digest(asset_path)
        base = "/World/ASM2Envs"
        cloner = Cloner(stage=stage)
        cloner.define_base_env(base)
        paths = cloner.generate_paths(base + "/env", worlds)
        template = paths[0]
        UsdGeom.Xform.Define(stage, template)
        drone = stage.DefinePrim(template + "/Drone", "Xform")
        if not drone.GetReferences().AddReference(str(asset_path), "/Drone"):
            raise ValueError("Cannot compose the supplied ASM2 USD")
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(drone)
        body.CreateSleepThresholdAttr().Set(0.)
        body.CreateLinearDampingAttr().Set(0.)
        body.CreateAngularDampingAttr().Set(0.)
        body.CreateEnableGyroscopicForcesAttr().Set(True)
        PhysxSchema.PhysxContactReportAPI.Apply(drone).CreateThresholdAttr().Set(0.)
        defaults = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(stage.DefinePrim("/World/ASM2SchemaDefaults", "Xform"))
        self.cooking_defaults = dict(vertex_limit=defaults.GetHullVertexLimitAttr().Get(),
                                    min_thickness_m=defaults.GetMinThicknessAttr().Get())
        self.authored_offsets = Counter()
        for prim in Usd.PrimRange(drone):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            group = prim.GetCustomDataByKey("contact_group")
            if self._entries[prim.GetName()]["group"] != group:
                raise ValueError(f"Invalid ASM2 collider identity: {prim.GetPath()}")
            cooking = PhysxSchema.PhysxConvexHullCollisionAPI(prim)
            offset = PhysxSchema.PhysxCollisionAPI(prim)
            self.authored_offsets[f"{offset.GetContactOffsetAttr().Get():.9g}"] += 1
            if mode != "source":
                cooking = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
                cooking.CreateMinThicknessAttr().Set(self.config["convex_min_thickness_m"])
                cooking.CreateHullVertexLimitAttr().Set(self.config["convex_hull_vertex_limit"])
            if mode in ("pgs1um", "normalized"):
                offset.CreateContactOffsetAttr().Set(self.config["diagnostic_contact_offset_m"])
                offset.CreateRestOffsetAttr().Set(self.config["rest_offset_m"])
            if contact_offset_m is not None:
                offset.CreateContactOffsetAttr().Set(contact_offset_m)
            if mode == "normalized":
                # PhysX QuickHull's plane tolerance is in mesh-local units.
                # Normalize ONLY the mesh representation, then undo it in the
                # collider transform. World geometry, body mass/frame and
                # simulation units remain metres, with no contact enlargement.
                mesh = UsdGeom.Mesh(prim)
                vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
                cook_scale = self.config["cook_coordinate_scale"]
                scaled = (vertices * cook_scale).astype(np.float32)
                mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(scaled))
                mesh.GetExtentAttr().Set([Gf.Vec3f(*scaled.min(0).astype(float).tolist()),
                                         Gf.Vec3f(*scaled.max(0).astype(float).tolist())])
                mesh.AddScaleOp().Set(Gf.Vec3f(1./cook_scale))
                if np.max(np.abs(scaled.astype(float) / cook_scale - vertices)) > 1e-8:
                    raise ValueError("Cook normalization changed physical mesh coordinates")
        if field:
            field_prim = stage.DefinePrim(template + "/Field", "Xform")
            field_prim.GetReferences().AddReference(str(ROOT / self.config["field_usd"]), "/Stage1Scene/Field")
            UsdGeom.Xformable(field_prim).AddTranslateOp().Set(Gf.Vec3d(*read_json(RUN / "field-case.json")["field_origin_translation_m"]))
            field_shapes = [p for p in Usd.PrimRange(field_prim) if p.HasAPI(UsdPhysics.CollisionAPI)]
            if contact_offset_m is not None:
                for shape in field_shapes:
                    PhysxSchema.PhysxCollisionAPI(shape).CreateContactOffsetAttr().Set(contact_offset_m)
            self.field_registered = {p.GetAttribute("stage1:geomName").Get() for p in field_shapes
                                     if p.GetAttribute("stage1:registered").Get()}
            if len(field_shapes) != 1822 or len(self.field_registered) != 25:
                raise ValueError("Candidate Field lost authentic static or registered colliders")
            normal = np.asarray(read_json(RUN / "field-case.json")["rotation"])[:, 2]
            UsdGeom.Xformable(drone).AddTranslateOp().Set(Gf.Vec3d(*normal.tolist()))
        else:
            UsdGeom.Xform.Define(stage, template + "/Field")
            for name, kind in (("test_plane", UsdGeom.Plane), ("test_box", UsdGeom.Cube)):
                geom = kind.Define(stage, template + "/Field/" + name)
                geom.AddTranslateOp().Set(Gf.Vec3d(0., 0., -10.))
                if name == "test_plane":
                    geom.CreateAxisAttr("Z")
                else:
                    geom.CreateSizeAttr(2.)
                    geom.AddScaleOp().Set(Gf.Vec3f(.0002, .0002, .002))
                UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
                shape = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
                shape.CreateContactOffsetAttr().Set(self.config["diagnostic_contact_offset_m"])
                shape.CreateRestOffsetAttr().Set(0.)
        # Explicit same friction assumption on both source and diagnostic layers.
        material = UsdShade.Material.Define(stage, template + "/PhysicsMaterial")
        mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        mat.CreateStaticFrictionAttr(.6)
        mat.CreateDynamicFrictionAttr(.6)
        mat.CreateRestitutionAttr(0.)
        for prim in Usd.PrimRange(stage.GetPrimAtPath(template)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                if not field or "/Drone/" in str(prim.GetPath()):
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
        side = math.ceil(math.sqrt(worlds))
        grid = np.arange(worlds)
        self.origins = np.column_stack((grid % side, grid // side, np.zeros(worlds))) * .5
        if layout:
            self.origins[:, :2] = self.origins[:, [1, 0]] + np.array([.037, -.023])
        cloner.clone(template, paths, positions=self.origins, replicate_physics=True, enable_env_ids=True)
        self.env_paths = paths
        self._path_to_world = {p: i for i, p in enumerate(paths)}
        self.stage_author_s = time.perf_counter() - started
        if export_recipe:
            recipe = RUN / f"isaac-{mode}-{worlds}-layout{layout}-{'field' if field else 'fixtures'}.usda"
            stage.GetRootLayer().Export(str(recipe))
            from pxr import Sdf, UsdUtils
            layer = Sdf.Layer.FindOrOpen(str(recipe))
            UsdUtils.ModifyAssetPaths(layer, lambda p: os.path.relpath(p, recipe.parent) if p.startswith(str(ROOT) + "/") else p)
            layer.Save()
        self.iface = omni.physx.get_physx_simulation_interface()
        self.query = omni.physx.get_physx_scene_query_interface()
        self.sim.reset()
        self.view = self.sim.physics_sim_view.create_rigid_body_view(base + "/env_*/Drone")
        if self.view.count != worlds or self.view.max_shapes != len(self._entries):
            raise ValueError(f"PhysX body/shape count mismatch: {self.view.count}/{self.view.max_shapes}")
        self.view_device = self.view.get_transforms().device
        self.indices = torch.arange(worlds, device=self.view_device)
        view_paths = {str(p): i for i, p in enumerate(self.view.prim_paths)}
        self.order = np.array([view_paths[p + "/Drone"] for p in paths])
        self.load_s = time.perf_counter() - started
        self.mass_actual = self.view.get_masses().cpu().numpy().reshape(-1)
        np.testing.assert_allclose(self.mass_actual, .142, atol=1e-8, rtol=0)
        self.inertia_actual = self.view.get_inertias().cpu().numpy().reshape(worlds, 3, 3)
        np.testing.assert_allclose(self.inertia_actual, np.broadcast_to(self.mass["inertia_body_kg_m2"], (worlds, 3, 3)),
                                   atol=2e-10, rtol=1e-5)
        self.com_actual = self.view.get_coms().cpu().numpy()
        # Reports are interval evidence: subscribe only once the actor views
        # exist. SimulationContext.reset may itself publish startup overlaps.
        self.enable_reports(True)

    def cooked_mesh_audit(self):
        import omni.physx
        from pxr import UsdGeom

        cooking = omni.physx.get_physx_cooking_interface()
        names = [name for name, e in self._entries.items() if e["group"] != "body"]
        names += ["collision_0_1987", "collision_0_350", "collision_0_569", "collision_11_3135"]
        rows = {}
        for name in names:
            path = self.env_paths[0] + "/Drone/Collision/" + name
            count = cooking.get_nb_convex_mesh_data(path)
            hulls = [cooking.get_convex_mesh_data(path, i) for i in range(count)]
            points = np.concatenate([np.asarray(h["vertices"], dtype=float) for h in hulls])
            prim = self.stage.GetPrimAtPath(path)
            transform = np.asarray(UsdGeom.XformCache().ComputeRelativeTransform(prim, self.stage.GetPrimAtPath(self.env_paths[0]+"/Drone"))[0])
            points = (np.c_[points, np.ones(len(points))] @ transform)[:, :3]
            source = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
            source = (np.c_[source, np.ones(len(source))] @ transform)[:, :3]
            rows[name] = dict(hulls=count, cooked_vertices=len(points), source_vertices=len(source),
                              source_bounds_m=[source.min(0).tolist(), source.max(0).tolist()],
                              cooked_bounds_m=[points.min(0).tolist(), points.max(0).tolist()],
                              max_axis_support_error_m=float(np.max(np.abs(np.array([points.min(0), points.max(0)]) -
                                                                                     np.array([source.min(0), source.max(0)])))))
        return rows

    def enable_reports(self, enabled, collection="full"):
        self._subscription = None
        self.collection_mode = collection
        if enabled:
            self._subscription = self.iface.subscribe_contact_report_events(self._contacts)

    def decode(self, key):
        if key not in self._paths:
            path = str(self._path_decoder(key))
            world_path, part, name = path.rsplit("/", 2)
            # Aircraft paths contain Drone/Collision/name; field paths contain Field/name.
            if part == "Collision":
                world_path = world_path.removesuffix("/Drone")
                entry = self._entries[name]
                group = entry["group"]
            elif part == "Field":
                group = None
            else:
                raise ValueError(f"Unexpected collider path: {path}")
            self._paths[key] = self._path_to_world[world_path], name, group, path
        return self._paths[key]

    def _contacts(self, headers, points):
        started = time.perf_counter()
        try:
            for h in headers:
                self.raw_headers += 1
                if self.collection_mode == "count":
                    continue
                if not h.num_contact_data:
                    self.empty_headers += 1
                    continue
                a, b = self.decode(h.collider0), self.decode(h.collider1)
                if a[0] != b[0] or (a[2] is None) == (b[2] is None):
                    raise ValueError(f"Cross-world or non-aircraft pair: {a}/{b}")
                aircraft, other, sign = (a, b, 1.) if a[2] else (b, a, -1.)
                world, name, group, path = aircraft
                before = self.pre[world]
                for index in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                    p = points[index]
                    point = np.asarray(tuple(p.position)) - self.origins[world]
                    record = contact_record(tick=self.tick, dt=self.dt, group=group, geom=name, other_geom=other[1],
                                            point=point, normal=sign*np.asarray(tuple(p.normal)),
                                            impulse=sign*np.asarray(tuple(p.impulse)), separation=float(p.separation),
                                            pre_position=before[0], pre_linear=before[1], pre_angular_world=before[2],
                                            collider_paths=[path, other[3]])
                    self.ledgers[world].add(record)
        except Exception as error:
            self.callback_error = error
        self.callback_s += time.perf_counter() - started

    def state(self):
        transforms = self.view.get_transforms().cpu().numpy()[self.order].copy()
        velocities = self.view.get_velocities().cpu().numpy()[self.order].copy()
        transforms[:, :3] -= self.origins
        return transforms, velocities

    def reset(self, case):
        from pxr import Gf, UsdGeom

        for path in self.env_paths if case["fixture"] != "registered_field" else []:
            for name in ("test_plane", "test_box"):
                selected = name == "test_" + case["fixture"]
                prim = self.stage.GetPrimAtPath(path + "/Field/" + name)
                translate = next(op for op in UsdGeom.Xformable(prim).GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate)
                translate.Set(Gf.Vec3d(*(case["fixture_position"] if selected else [0., 0., -10.])))
        matrix = np.asarray(case["rotation"])
        # Gf matrices use row vectors, USD pose tensors use xyzw.
        q = Gf.Matrix3d(*matrix.T.reshape(-1).tolist()).ExtractRotation().GetQuat()
        poses = self.view.get_transforms()
        velocities = self.view.get_velocities()
        for env, view_id in enumerate(self.order):
            poses[view_id] = poses.new_tensor([*(np.asarray(case["position"]) + self.origins[env]), *q.GetImaginary(), q.GetReal()])
            velocities[view_id] = velocities.new_tensor([*case["velocity"], *(matrix @ case["omega"])])
        self.view.set_transforms(poses, self.indices)
        self.view.set_velocities(velocities, self.indices)

    def step(self, force=False, readback=True):
        self.tick += 1
        if readback:
            pose, velocity = self.state()
            self.pre = {env: (pose[env, :3], velocity[env, :3], velocity[env, 3:]) for env in range(self.worlds)}
            for ledger in self.ledgers:
                ledger.begin(self.tick)
        if force:
            forces = self.torch.zeros((self.worlds, 3), device=self.view_device)
            forces[:, 2] = -.142 * 9.81
            self.view.apply_forces_and_torques_at_position(force_data=forces, torque_data=None, position_data=None,
                                                         indices=self.indices, is_global=True)
        self.sim.step(render=False)
        if self.callback_error:
            raise RuntimeError("Collider-level report decoding failed") from self.callback_error

    def fixture(self, case, registered_targets=()):
        self.reset(case)
        per_world = [[] for _ in range(self.worlds)]
        trajectories = [[] for _ in range(self.worlds)]
        for _ in range(case.get("steps", 1)):
            self.step()
            pose, velocity = self.state()
            for index, ledger in enumerate(self.ledgers):
                per_world[index].extend(ledger.records)
                before = self.pre[index]
                impulse = np.sum([r["impulse_on_drone_world_Ns"] for r in ledger.records], axis=0) if ledger.records else np.zeros(3)
                momentum = .142 * (velocity[index, :3] - before[1])
                trajectories[index].append(dict(tick=self.tick, pre_position=before[0].tolist(),
                                               pre_velocity=before[1].tolist(), post_pose_xyzw=pose[index].tolist(),
                                               post_velocity=velocity[index].tolist(),
                                               impulse_momentum_residual_Ns=(impulse-momentum).tolist()))
        rows = []
        for index, records in enumerate(per_world):
            summary = summarize(records)
            first = next((r["tick"] for r in records if r["distance_m"] <= 0), None)
            first_records = [r for r in records if r["tick"] == first and r["distance_m"] <= 0]
            summary.update(classification_pass=summary["touching_groups"] == case["expected_groups"], records=records,
                           first_touching_tick=first, first_touching=summarize(first_records),
                           scorer_input_at_first_touch=[scorer_input(r, registered_targets) for r in first_records],
                           trajectory=trajectories[index],
                           maximum_impulse_momentum_residual_Ns=max(np.linalg.norm(t["impulse_momentum_residual_Ns"])
                                                                    for t in trajectories[index]))
            rows.append(summary)
        return dict(expected_groups=case["expected_groups"], worlds=rows)

    def guard_rays(self):
        case = dict(fixture="plane", fixture_position=[0., 0., -10.], position=[0., 0., 0.],
                    rotation=np.eye(3).tolist(), velocity=[0., 0., 0.], omega=[0., 0., 0.])
        self.reset(case)
        self.step()
        native = read_json(RUN / "guard-rays.json")
        results = []
        for row in native["rays"]:
            hits = []

            def collect(hit):
                collider = str(hit.collision)
                if collider.startswith(self.env_paths[0] + "/Drone/Collision/"):
                    hits.append(dict(collider=collider.rsplit("/", 1)[-1], distance_m=float(hit.distance),
                                     position=list(hit.position), normal=list(hit.normal)))
                return True

            origin = np.asarray(row["origin_m"]) + self.origins[0]
            self.query.raycast_all(tuple(origin.tolist()), tuple(row["direction"]), .16, collect)
            first = min(hits, key=lambda h: h["distance_m"]) if hits else None
            results.append(dict(index=row["index"], visual_open=row["visual_distance_m"] < 0,
                                native_collision_open=row["collision_distance_m"] < 0,
                                physx_open=first is None, first_hit=first,
                                distance_error_m=None if first is None or row["collision_distance_m"] < 0
                                else first["distance_m"] - row["collision_distance_m"]))
        return dict(probes=len(results), visual_open=sum(r["visual_open"] for r in results),
                    blocked_visual_open=sum(r["visual_open"] and not r["physx_open"] for r in results),
                    native_physx_open_disagreements=sum(r["native_collision_open"] != r["physx_open"] for r in results),
                    max_abs_hit_distance_error_m=max((abs(r["distance_error_m"]) for r in results if r["distance_error_m"] is not None), default=None),
                    rays=results)

    def benchmark(self, steps):
        fixtures = {r["name"]: r for r in read_json(RUN / "fixtures.json")["fixtures"]}
        results = {}
        for name, contact, reporting in (("free", False, "off"), ("contact_no_reports", True, "off"),
                                          ("contact_headers", True, "count"), ("contact_full_reports", True, "full")):
            case = dict(fixtures["both"])
            case.update(velocity=[0., 0., 0.], omega=[0., 0., 0.])
            if not contact:
                case["fixture_position"] = [0., 0., -10.]
            self.reset(case)
            self.enable_reports(reporting != "off", reporting)
            for _ in range(50):
                self.step(force=True)
            callback_start, headers_start = self.callback_s, self.raw_headers
            started = time.perf_counter()
            for _ in range(steps):
                self.step(force=True)
            seconds = time.perf_counter() - started
            pose, velocity = self.state()
            results[name] = dict(steps=steps, wall_s=seconds, world_steps_s=self.worlds*steps/seconds,
                                 simulated_seconds_per_wall_second=steps*self.dt/seconds,
                                 callback_s=self.callback_s-callback_start, contact_headers=self.raw_headers-headers_start,
                                 finite=bool(np.isfinite(pose).all() and np.isfinite(velocity).all()),
                                 max_speed_m_s=float(np.linalg.norm(velocity[:, :3], axis=1).max()),
                                 max_displacement_m=float(np.linalg.norm(pose[:, :3]-case["position"], axis=1).max()))
            print("ASM2_BENCH",self.worlds,name,round(results[name]["world_steps_s"],1),"world_steps/s",flush=True)
        return results


def run(args):
    sim = ASM2Sim(args.worlds, args.mode, args.layout, field=args.suite == "field")
    result = dict(backend="Isaac Lab 2.1 / Isaac Sim 4.5 / PhysX GPU PGS", mode=args.mode, worlds=args.worlds,
                  layout=args.layout, physics_dt_s=sim.dt, stage_author_s=sim.stage_author_s, load_s=sim.load_s,
                  shapes_per_aircraft=sim.view.max_shapes, mass_kg=sim.mass_actual.tolist(),
                  inertia_tensor_raw=sim.inertia_actual[0].tolist(), com_pose_xyzw=sim.com_actual[0].tolist(),
                  installed_cooking_defaults=sim.cooking_defaults, supplied_contact_offsets_m=dict(sim.authored_offsets),
                  candidate_cooking_override=None if args.mode == "source" else dict(min_thickness_m=0., vertex_limit=255),
                  contact_impulse_scope="PhysX p.impulse / dt; not a validated total contact-force sensor; tangential completeness unverified",
                  task_qualification="NOT_DEFINED_FOR_ASM2", training_transitions=0)
    result.update(asset=sim.asset_path, asset_sha256=sim.asset_sha256)
    result["cooked_mesh_sample"] = sim.cooked_mesh_audit()
    if args.suite == "fixtures":
        results = {}
        for case in read_json(RUN / "fixtures.json")["fixtures"]:
            results[case["name"]] = sim.fixture(case)
            rows = results[case["name"]]["worlds"]
            print("ASM2_PHYSX", case["name"], rows[0]["touching_groups"], sum(r["classification_pass"] for r in rows), "/",sim.worlds,flush=True)
        result["fixtures"] = results
        result["callback_s"] = sim.callback_s
        result["duplicate_samples_removed"] = sum(l.duplicates for l in sim.ledgers)
    if args.suite == "field":
        result["field"] = sim.fixture(read_json(RUN / "field-case.json"), sim.field_registered)
        result["registered_target_count"] = len(sim.field_registered)
    if args.suite in ("fixtures", "bench"):
        result["guard"] = sim.guard_rays()
    if args.suite == "bench":
        result["benchmark"] = sim.benchmark(args.steps)
    result["max_rss_MiB"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    import subprocess
    result["gpu_memory_snapshot"] = subprocess.check_output(["nvidia-smi", "--query-gpu=memory.used,utilization.gpu", "--format=csv,noheader,nounits"], text=True).strip()
    result["gpu_process_memory_snapshot"] = subprocess.check_output(["nvidia-smi", "--query-compute-apps=pid,process_name,used_gpu_memory", "--format=csv,noheader,nounits"], text=True).strip()
    write_result(args.output, result)
    print("ASM2_ISAAC_COMPLETE", args.output,flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("source", "cooked", "pgs1um", "normalized", "candidate"), default="candidate")
    parser.add_argument("--worlds", type=int, choices=range(1, 65), default=1)
    parser.add_argument("--layout", type=int, choices=(0, 1), default=0)
    parser.add_argument("--suite", choices=("load", "fixtures", "bench", "field"), default="fixtures")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=200)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a new evidence path")
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
    # Same one-shot exit as the verified legacy contact tools: Kit contact
    # teardown can stall. All simulation/evidence is complete before this exit.
    os._exit(code)


if __name__ == "__main__":
    main()
