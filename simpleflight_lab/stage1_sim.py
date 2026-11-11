"""Batched RM27 Stage 1 PhysX state and collider-level contact reports."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import NamedTuple

import numpy as np
import torch

from .rm27 import RM27Actuator, load_rm27_profile


_ROOT = Path(__file__).resolve().parents[1]
_CONFIGS = _ROOT / "upstream/rm27_RL/configs/stage1"
_DEFAULT_SCENE = _ROOT / ".agents/runs/rm27-stage1/contact-scene.usda"
_ENVS = "/World/Stage1Envs"
_BOTTOM = "drone_1_stage1_contact"


class Stage1State(NamedTuple):
    """Environment-local world position/velocity, wxyz attitude, and body rates."""

    position: torch.Tensor
    quaternion: torch.Tensor
    velocity: torch.Tensor
    omega_body: torch.Tensor


@dataclass(frozen=True)
class Stage1Tick:
    before: Stage1State
    after: Stage1State
    contacts: dict[int, list[dict]]


class Stage1Sim:
    """Exact authored Stage 1 colliders, with one dynamic aircraft per GPU world.

    Call reset before the first tick, allocate CTBR once per 5 ms control
    interval, and step PhysX at 1 ms. Contact positions use the frozen local
    world frame, identical to the state frame. Reports include speculative
    positive separation points so the caller can enforce the frozen gate.
    """

    def __init__(self, num_envs: int, scene_path: Path = _DEFAULT_SCENE, device: str = "cuda:0", *,
                 solver_type: int | None = None):
        import carb
        import omni.physx
        import omni.usd
        from isaaclab import sim as sim_utils
        from isaacsim.core.cloner import Cloner
        from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics

        from .rm27_contact_scene import SCENE_ROOT, reference_scene

        if num_envs < 1 or torch.device(device).type != "cuda":
            raise ValueError("Stage1Sim requires at least one GPU PhysX environment")
        self.num_envs = num_envs
        self.device = device
        self.profile = load_rm27_profile()
        self.physics_dt_s = self.profile.physics_dt_s
        self.control_dt_s = self.profile.policy_dt_s
        config = json.loads((_CONFIGS / "task_contact_geometry.json").read_text())
        registry_bytes = (_CONFIGS / "contact_surface_registration.json").read_bytes()
        registry_sha = hashlib.sha256(registry_bytes).hexdigest()
        registry = json.loads(registry_bytes)
        source_stage = Usd.Stage.Open(str(scene_path.resolve(strict=True)))
        source = source_stage.GetPrimAtPath(SCENE_ROOT)
        if (
            registry_sha != config["expected_contact_surface_registration_sha256"]
            or source.GetCustomDataByKey("stage1:registrySha256") != registry_sha
            or source.GetCustomDataByKey("stage1:modelSha256") != registry["physics_model_sha256"]
        ):
            raise ValueError("Stage 1 scene does not match the frozen contact registration")
        legal = set(registry["original_registered_contact_geoms"]) | {
            row["geom"] for row in registry["additional_contact_geoms"]
        }
        target = registry["target_reference"]
        center = np.array(target["armor_surface_center_m"])
        normal = np.array(target["armor_normal"])
        nominal = center + config["initial_state"]["normal_standoff_m"] * normal / np.linalg.norm(normal)

        sim_config = sim_utils.SimulationCfg(
            device=device, dt=self.physics_dt_s, render_interval=1, enable_scene_query_support=True,
        )
        if solver_type is not None:
            if solver_type not in (0, 1):
                raise ValueError("Expected PhysX solver 0 (PGS) or 1 (TGS)")
            sim_config.physx.solver_type = solver_type
        self.sim = sim_utils.SimulationContext(sim_config)
        settings = carb.settings.get_settings()
        settings.set_bool("/physics/disableContactProcessing", False)
        settings.set_bool("/physics/suppressReadback", False)
        stage = omni.usd.get_context().get_stage()
        self._cloner = Cloner(stage=stage)
        self._cloner.define_base_env(_ENVS)
        paths = self._cloner.generate_paths(_ENVS + "/env", num_envs)
        template = paths[0]
        UsdGeom.Xform.Define(stage, template)
        field, drone = reference_scene(stage, scene_path, template)
        UsdGeom.Xformable(drone).GetOrderedXformOps()[0].Set(Gf.Vec3d(*nominal))

        # Keep finite cropped fields spatially separated for the broadphase.
        # PhysX env IDs additionally isolate the infinite floor of each copy.
        side = math.ceil(math.sqrt(num_envs))
        grid = np.arange(num_envs)
        origins = np.column_stack((grid // side, grid % side, np.zeros(num_envs))) * 3.0
        origins[:, :2] -= center[:2]
        self._cloner.clone(template, paths, positions=origins, replicate_physics=True, enable_env_ids=True)
        self._origins = origins.astype(np.float32)
        self._offset = torch.tensor(self._origins, device=device)
        self._env_path_to_index = {path: index for index, path in enumerate(paths)}
        self._path_from_key = PhysicsSchemaTools.intToSdfPath
        self._shape_by_suffix = {}
        for root in (field, drone):
            for prim in Usd.PrimRange(root):
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                name = prim.GetAttribute("stage1:geomName").Get()
                geom_id = prim.GetAttribute("stage1:geomId").Get()
                registered = prim.GetAttribute("stage1:registered").Get()
                if name is None or geom_id is None or registered != (name in legal):
                    raise ValueError(f"Invalid archived collider identity at {prim.GetPath()}")
                suffix = str(prim.GetPath())[len(template) + 1:]
                self._shape_by_suffix[suffix] = (name, geom_id, registered, root == drone)
        names = {name for name, _, _, _ in self._shape_by_suffix.values()}
        if not (legal | {_BOTTOM}) <= names or len(self._shape_by_suffix) != 2069:
            raise ValueError("Stage 1 USD is missing authentic cropped field or aircraft colliders")

        # A disabled twin gives overlap queries exact aircraft geometry without
        # moving a live actor or relying on its post-step collision manifolds.
        self._query = omni.physx.get_physx_scene_query_interface()
        self._query_field = str(field.GetPath()) + "/"
        probe = UsdGeom.Xform.Define(stage, "/World/Stage1OverlapProbe").GetPrim()
        if not probe.GetReferences().AddReference(str(scene_path.resolve()), f"{SCENE_ROOT}/Drone"):
            raise RuntimeError("Cannot reference the Stage 1 aircraft for overlap queries")
        probe.RemoveAPI(UsdPhysics.RigidBodyAPI)
        xform = UsdGeom.Xformable(probe)
        xform.ClearXformOpOrder()
        self._probe_xform = xform.AddTransformOp(opSuffix="overlap")
        self._probe_shapes = []
        for prim in Usd.PrimRange(probe):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
                geom_id = prim.GetAttribute("stage1:geomId").Get()
                self._probe_shapes.append((geom_id, PhysicsSchemaTools.encodeSdfPath(prim.GetPath())))
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeUntransformedBound(drone)
        bounds = bbox.ComputeAlignedRange()
        self._query_radius = float(np.linalg.norm(np.maximum(np.abs(bounds.GetMin()), np.abs(bounds.GetMax()))))
        if len(self._probe_shapes) != 247 or not math.isfinite(self._query_radius) or self._query_radius <= 0:
            raise RuntimeError("Invalid Stage 1 overlap probe geometry")

        self._decoded_paths = {}
        self._contacts = {}
        self._callback_error = None
        self._sim_iface = omni.physx.get_physx_simulation_interface()
        self._subscription = self._sim_iface.subscribe_contact_report_events(self._record_contacts)
        self.sim.reset()
        self._view = self.sim.physics_sim_view.create_rigid_body_view(_ENVS + "/env_*/Drone")
        if self._view.count != num_envs or self._view.max_shapes != 247:
            raise RuntimeError("PhysX did not instantiate every Stage 1 aircraft and its shapes")
        indices = {str(path): i for i, path in enumerate(self._view.prim_paths)}
        # Kit 4.5 uses a CPU tensor view when GPU contact readback is enabled.
        # Keep the solver on GPU and bridge its small state and wrench batches.
        self._view_device = self._view.get_transforms().device
        self._view_order = torch.tensor([indices[path + "/Drone"] for path in paths], device=self._view_device)
        self._env_order = self._view_order.argsort().to(device)
        self._all = torch.arange(num_envs, device=self._view_device)
        mass = self._view.get_masses()
        torch.testing.assert_close(mass, torch.full_like(mass, self.profile.mass_kg))
        inertia = self._view.get_inertias().reshape(-1, 3, 3).diagonal(dim1=1, dim2=2)
        torch.testing.assert_close(inertia, inertia.new_tensor(self.profile.inertia_kg_m2).expand_as(inertia),
                                   atol=1e-8, rtol=1e-5)
        self.actuator = RM27Actuator(self.profile, num_envs, device)
        self._state = self._read_state()

    def _decode_path(self, key):
        decoded = self._decoded_paths.get(key)
        if decoded is None:
            path = str(self._path_from_key(key))
            env_path, suffix = path[len(_ENVS) + 1:].split("/", 1)
            env = self._env_path_to_index[_ENVS + "/" + env_path]
            name, geom_id, registered, aircraft = self._shape_by_suffix[suffix]
            decoded = env, name, geom_id, registered, aircraft, path
            self._decoded_paths[key] = decoded
        return decoded

    def _record_contacts(self, headers, data):
        try:
            for header in headers:
                if header.num_contact_data == 0:
                    continue
                env, name0, id0, registered0, aircraft0, path0 = self._decode_path(header.collider0)
                other, name1, id1, registered1, aircraft1, path1 = self._decode_path(header.collider1)
                if env != other:
                    raise RuntimeError(f"Cross-environment contact: {path0} and {path1}")
                if aircraft0 == aircraft1:
                    raise RuntimeError(f"Expected a single aircraft side: {path0} and {path1}")
                rows = self._contacts.setdefault(env, [])
                outward_sign = 1.0 if aircraft0 else -1.0
                designated = (name0 == _BOTTOM and registered1) or (name1 == _BOTTOM and registered0)
                origin = self._origins[env]
                for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                    point = data[i]
                    normal, impulse = tuple(point.normal), tuple(point.impulse)
                    rows.append({
                        "geom_ids": [id0, id1], "geoms": [name0, name1], "collider_paths": [path0, path1],
                        "position_world_m": [x - float(offset) for x, offset in zip(point.position, origin)],
                        "distance_m": float(point.separation),
                        "surface_outward_world": [outward_sign * x for x in normal],
                        "impulse_world_N_s": list(impulse),
                        "normal_force_N": sum(n * j for n, j in zip(normal, impulse)) / self.physics_dt_s,
                        "designated": designated,
                    })
        except Exception as error:
            self._callback_error = error

    def _read_state(self) -> Stage1State:
        from isaaclab.utils.math import quat_rotate_inverse

        pose = self._view.get_transforms().index_select(0, self._view_order).to(self.device)
        velocity = self._view.get_velocities().index_select(0, self._view_order).to(self.device)
        quat = torch.cat((pose[:, 6:7], pose[:, 3:6]), dim=1)
        return Stage1State(pose[:, :3] - self._offset, quat, velocity[:, :3], quat_rotate_inverse(quat, velocity[:, 3:]))

    def state(self) -> Stage1State:
        return self._state

    def initial_overlaps(self, positions, quaternions) -> dict[int, list[tuple[int, int]]]:
        """Check candidate reset poses against the same field before taking a step.

        Keys index the supplied rows, with pairs in (aircraft, field) ID order.
        Worlds have identical static fields, so all candidates query field 0.
        """
        from pxr import Gf

        data = torch.cat((positions, quaternions), dim=1).cpu().numpy()
        collisions = {}
        for row, (x, y, z, w, qx, qy, qz) in enumerate(data.tolist()):
            position = tuple(float(a + b) for a, b in zip((x, y, z), self._origins[0]))
            field_hit = False

            def record_field(hit):
                nonlocal field_hit
                if str(hit.collision).startswith(self._query_field):
                    field_hit = True
                    return False
                return True

            self._query.overlap_sphere(self._query_radius, position, record_field, False)
            if not field_hit:
                continue
            transform = Gf.Matrix4d(1.0)
            transform.SetRotate(Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(qx, qy, qz))))
            transform.SetTranslateOnly(Gf.Vec3d(*position))
            self._probe_xform.Set(transform)
            pairs = set()
            for geom_id, shape_key in self._probe_shapes:
                def record_overlap(hit):
                    path = str(hit.collision)
                    if path.startswith(self._query_field):
                        suffix = "Field/" + path[len(self._query_field):]
                        pairs.add((geom_id, self._shape_by_suffix[suffix][1]))
                    return True

                self._query.overlap_shape(*shape_key, record_overlap, False)
            if pairs:
                collisions[row] = sorted(pairs)
        return collisions

    def reset(self, env_ids, pos_env_m, quat_wxyz, vel_env_m_s, omega_body_rad_s,
              motor_thrust_N=None, motor_command_N=None) -> None:
        """Restore a policy-boundary state, with separate activation and held motor command."""
        from isaaclab.utils.math import quat_rotate

        if len(env_ids) == 0:
            return
        if (motor_thrust_N is None) != (motor_command_N is None):
            raise ValueError("Provide motor activation and command together for an in-flight reset")
        if motor_thrust_N is not None and (
            motor_thrust_N.shape != (len(env_ids), 4) or motor_command_N.shape != motor_thrust_N.shape
        ):
            raise ValueError("Motor activation and command must have one row of four rotors per reset")
        view_ids = self._view_order[env_ids.to(self._view_device)]
        pose = self._view.get_transforms()
        speed = self._view.get_velocities()
        pose[view_ids] = torch.cat((pos_env_m + self._offset[env_ids], quat_wxyz[:, 1:], quat_wxyz[:, :1]), dim=1).to(
            self._view_device
        )
        speed[view_ids] = torch.cat((vel_env_m_s, quat_rotate(quat_wxyz, omega_body_rad_s)), dim=1).to(self._view_device)
        self._view.set_transforms(pose, indices=view_ids)
        self._view.set_velocities(speed, indices=view_ids)
        self.actuator.reset(env_ids)
        if motor_thrust_N is not None:
            self.actuator.motor_thrust_N[env_ids] = motor_thrust_N
            self.actuator.motor_command_N[env_ids] = motor_command_N
        self._state = self._read_state()

    def set_ctbr(self, thrust_N, rates_rad_s) -> None:
        command = torch.cat((thrust_N.reshape(-1, 1), rates_rad_s), dim=1)
        self.actuator.set_ctbr(command, self._state.omega_body)

    def step_physics(self) -> Stage1Tick:
        before = self._state
        force, torque = self.actuator.step_wrench()
        self._view.apply_forces_and_torques_at_position(
            force_data=force.index_select(0, self._env_order).to(self._view_device),
            torque_data=torque.index_select(0, self._env_order).to(self._view_device),
            position_data=None, indices=self._all, is_global=False,
        )
        self._contacts = {}
        self._callback_error = None
        self.sim.step(render=False)
        if self._callback_error is not None:
            raise RuntimeError("Unable to decode batched PhysX contact reports") from self._callback_error
        self._state = self._read_state()
        return Stage1Tick(before, self._state, self._contacts)

    def close(self) -> None:
        self._subscription = None
