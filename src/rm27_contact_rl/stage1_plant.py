"""Named-aircraft adapter for the frozen RMUC2027 scene, without changing Physics.

The private scene fixes the nominal base rail and adds the existing S1c assumed
underside sphere. Original scene, assets, motor dynamics and inertial parameters
remain evidence inputs. Scene compilation is an explicit bounded caller action.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from rm27_contact_physics.calibration import load_profile, verify_compiled
from rm27_contact_physics.control import motor_mixer
from rm27_contact_physics.model import VehicleEstimate
from rm27_contact_physics.teacher_interface import RateThrust, apply_proxy

from .paths import ROOT

BOTTOM_GEOM = "drone_1_stage1_contact"


def _path(value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else ROOT / path).resolve()


def _sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _snapshot_file(directory, relative):
    path = (directory / relative).resolve()
    if directory not in path.parents:
        raise ValueError("Frozen snapshot references must stay inside the snapshot")
    return path


def _contact_material(path):
    target = json.loads(Path(path).read_text())["target"]
    return dict(friction=[float(target["friction"]), .005, .0001],
                solref=[float(target["solref_s"]), float(target["solref_damping_ratio"])],
                solimp=[.95, .99, .001])


def _inputs(cfg):
    directory = _path(cfg["scene_directory"])
    summary_path = directory / "summary.json"
    summary = json.loads(summary_path.read_text())
    if not summary.get("passed") or summary.get("status") != "PASS":
        raise ValueError("The extracted scene does not have a passing source audit")
    if summary.get("mujoco_version") != mujoco.__version__ or not mujoco.__version__.startswith("3.12."):
        raise ValueError("Use the frozen MuJoCo 3.12 scene with its matching runtime version")
    source = _snapshot_file(directory, summary["scene"])
    expected = cfg["expected_scene_sha256"]
    if summary["scene_sha256"] != expected or _sha(source) != expected:
        raise ValueError("Frozen extracted scene hash mismatch")
    if cfg.get("expected_scene_summary_sha256") and _sha(summary_path) != cfg["expected_scene_summary_sha256"]:
        raise ValueError("Frozen scene audit hash mismatch")
    compiled = _snapshot_file(directory, summary["compiled_model"])
    if _sha(compiled) != summary["compiled_model_sha256"]:
        raise ValueError("Frozen compiled source scene hash mismatch")
    for relative, record in summary["copied_assets"].items():
        if _sha(_snapshot_file(directory, relative)) != record["sha256"]:
            raise ValueError(f"Frozen scene asset hash mismatch: {relative}")
    paths = {name: _path(cfg[name]) for name in
             ("target_mapping", "vehicle_config", "dynamics_profile", "contact_fixture_config")}
    if "target_registration_source" in cfg:
        paths["target_registration_source"] = _path(cfg["target_registration_source"])
        if _sha(paths["target_registration_source"]) != summary["source_runtime_sha256"]["collision_geoms.xml"]:
            raise ValueError("Target registration source differs from the frozen field collision geometry")
    target = json.loads(paths["target_mapping"].read_text())
    if target["field_sha256"] != summary["source_runtime_sha256"]["model.xml"]:
        raise ValueError("Target registration belongs to a different source field")
    if not target.get("registered_contact_geoms"):
        raise ValueError("Explicit registered physical target surfaces are required")
    contact = json.loads(paths["contact_fixture_config"].read_text())["contact"]
    if not contact.get("bumper") or float(contact["radius_m"]) <= 0:
        raise ValueError("Expected the existing explicitly assumed S1c contact sphere")
    if not np.isclose(float(cfg.get("physics_dt_s", .001)), .001, rtol=0, atol=1e-12):
        raise ValueError("Stage 1 uses one millisecond physical intervals")
    if not np.isclose(float(cfg.get("policy_dt_s", .005)), .005, rtol=0, atol=1e-12):
        raise ValueError("Stage 1 uses five millisecond policy actions")
    sources = dict(paths, source_scene=source, source_scene_summary=summary_path,
                   source_scene_binary=compiled)
    identity = {name: dict(path=str(path), sha256=_sha(path)) for name, path in sources.items()}
    identity["assets"] = {name: record["sha256"] for name, record in summary["copied_assets"].items()}
    return directory, summary, target, contact, sources, identity


def _resolve_target_registration(root, summary, target, sources):
    """Keep the selected physical plate, auditing unchanged source registration."""
    configured = list(target["registered_contact_geoms"])
    available = {geom.get("name") for geom in root.iter("geom")}
    retained = [name for name in configured if name in available]
    missing = [name for name in configured if name not in available]
    excluded = []
    if missing:
        if "target_registration_source" not in sources:
            raise ValueError("Missing registered surfaces require the explicitly frozen full-field registration source")
        full = ET.parse(sources["target_registration_source"]).getroot()
        surfaces = full.find("body[@name='rmuc2027_collision_surfaces']")
        if (surfaces is None or not np.allclose(np.fromstring(surfaces.get("pos", "0 0 0"), sep=" "), 0.)
                or not np.allclose(np.fromstring(surfaces.get("quat", "1 0 0 0"), sep=" "), [1., 0., 0., 0.])):
            raise ValueError("Full-field registration classification requires its reviewed world-aligned surface body")
        positions = {geom.get("name"): np.fromstring(geom.get("pos", "0 0 0"), sep=" ")
                     for geom in surfaces.findall("geom")}
        center = np.asarray(target["armor_surface_center_m"])
        regions = np.asarray(summary["config"]["regions"])
        for name in missing:
            position = positions.get(name)
            if position is None or position.shape != (3,):
                raise ValueError(f"Cannot classify absent registered target surface: {name}")
            opposite_base = position[0] * center[0] < 0
            inside_roi = np.any(np.all(position >= regions[:, 0], axis=1) & np.all(position <= regions[:, 1], axis=1))
            if not opposite_base or inside_roi:
                raise ValueError(f"Selected-side registered target geometry was omitted: {name}")
            excluded.append(dict(geom=name, source_position_world_m=position.tolist(),
                                 reason="opposite-X red base outside the frozen subset; selected target is positive-X blue"))
    if not retained:
        raise ValueError("The extracted scene has no retained selected target surfaces")
    model = mujoco.MjModel.from_binary_path(str(sources["source_scene_binary"]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    center = np.asarray(target["armor_surface_center_m"], float)
    normal = np.asarray(target["armor_normal"], float)
    normal /= np.linalg.norm(normal)
    slope, across = np.asarray(target["slope_axis"]), np.asarray(target["across_axis"])
    half = target["contact_region_halfsize_slope_across_m"]
    alpha = model.geom_rgba[:, 3].copy()
    records = []
    try:
        # MuJoCo rays skip alpha-zero collision meshes. This only changes an
        # audit model's visibility temporarily, following the existing Sim audit.
        model.geom_rgba[:, 3] = 1.
        for a in np.linspace(-half[0], half[0], 21):
            for b in np.linspace(-half[1], half[1], 21):
                origin = center + a * slope + b * across + .15 * normal
                hits = []
                for group in (2, 3):
                    mask = np.zeros(6, np.uint8)
                    mask[group] = 1
                    identifier = np.array([-1], np.int32)
                    distance = float(mujoco.mj_ray(model, data, origin, -normal, mask, 1, -1, identifier))
                    name = model.geom(int(identifier[0])).name if identifier[0] >= 0 else None
                    hits.append((distance, name))
                (visual_distance, visual), (collision_distance, collision) = hits
                error = collision_distance - visual_distance
                passed = (visual == "visual_115" and collision in retained and visual_distance >= 0
                          and collision_distance >= 0 and abs(error) <= .005)
                records.append(dict(offset_slope_across_m=[float(a), float(b)], visual=visual,
                                    collision=collision, visual_distance_m=visual_distance,
                                    collision_distance_m=collision_distance, collision_minus_visual_m=error,
                                    passed=bool(passed)))
    finally:
        model.geom_rgba[:, 3] = alpha
    audit = dict(passed=all(record["passed"] for record in records),
                 configured_source_geoms=configured, retained_selected_geoms=retained,
                 excluded_opposite_base_geoms=excluded, samples=records,
                 footprint_halfsize_slope_across_m=half, sample_grid=[21, 21],
                 visual_collision_tolerance_m=.005,
                 diagnosis="Initial preparation rejected all-33 presence: source registration also lists the opposite red base. "
                           "Initial rays with original alpha-zero colliders returned NULL; audit temporarily enables visibility "
                           "on its private loaded model and restores it. No geometry or valid region changes.")
    if not audit["passed"]:
        raise ValueError("Retained target surfaces do not cover the unchanged registered contact footprint")
    resolved = copy.deepcopy(target)
    resolved["registered_contact_geoms"] = retained
    return resolved, audit


def prepare_stage1_scene(cfg, output_dir):
    """Verify source evidence and write a private XML variant; do not compile it."""
    directory, summary, target, contact, sources, identity = _inputs(cfg)
    output = _path(output_dir)
    if output == directory or directory in output.parents:
        raise ValueError("The private scene must be outside the protected source snapshot")
    manifest_path = output / "stage1_scene.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest["inputs"] != identity or _sha(output / "scene.xml") != manifest["scene_sha256"]:
            raise ValueError("Private scene cache does not match the frozen inputs")
        return output / "scene.xml"
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Private scene output must be new or have a verified Stage 1 manifest")
    root = ET.parse(sources["source_scene"]).getroot()
    target, registration_audit = _resolve_target_registration(root, summary, target, sources)
    root.set("model", "rm27_fixed_surface_stage1")
    root.find("option").set("timestep", "0.001")
    removed_joints = removed_actuators = 0
    for element in root.iter():
        for child in list(element):
            if child.tag == "joint" and child.get("name") == "blue_base_slide":
                element.remove(child)
                removed_joints += 1
            elif child.tag in ("position", "general") and child.get("name") == "blue_base_drive":
                element.remove(child)
                removed_actuators += 1
    if (removed_joints, removed_actuators) != (1, 1):
        raise ValueError("Expected exactly the reviewed independent platform rail and drive")
    for keyframe in root.findall("keyframe"):
        root.remove(keyframe)  # Source key vectors include the removed rail coordinate.
    for element in root.iter():
        if element.get("file"):
            element.set("file", str(_snapshot_file(directory, element.get("file"))))
    body = root.find("worldbody/body[@name='drone_1']")
    if body is None or body.find("freejoint[@name='drone_1_free']") is None:
        raise ValueError("Frozen scene is missing the named active aircraft")
    material = _contact_material(sources["contact_fixture_config"])
    ET.SubElement(body, "geom", name=BOTTOM_GEOM, type="sphere", mass="0",
                  pos=" ".join(map(str, contact["offset_body_m"])), size=str(contact["radius_m"]),
                  contype="1", conaffinity="1", group="3", rgba=".1 .9 .3 1",
                  **{name: " ".join(map(str, value)) for name, value in material.items()})
    output.mkdir(parents=True, exist_ok=True)
    destination = output / "scene.xml"
    ET.ElementTree(root).write(destination, encoding="unicode")
    manifest = dict(inputs=identity, scene_sha256=_sha(destination), physics_dt_s=.001,
                    policy_dt_s=.005, fixed_platform_joint_removed="blue_base_slide",
                    original_inputs_preserved=True, compiled=False,
                    contact_fixture=dict(geom=BOTTOM_GEOM, material=material, **contact),
                    resolved_target_mapping=target, target_registration_audit=registration_audit,
                    modifications=["fix original rail at nominal q=0", "physics timestep 1 ms",
                                   "add massless S1c assumed underside sphere", "remove incompatible source keyframes"],
                    limitations=summary["limitations"] + [contact.get("status", "Engineering assumed contact sphere")])
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return destination


def compile_stage1_scene(cfg, output_dir):
    """Compile once when the caller has allocated the scene-build wall budget."""
    scene = prepare_stage1_scene(cfg, output_dir)
    manifest_path = scene.parent / "stage1_scene.json"
    manifest = json.loads(manifest_path.read_text())
    binary = scene.parent / "scene.mjb"
    if manifest.get("compiled"):
        if (manifest.get("mujoco_version") != mujoco.__version__
                or not binary.is_file() or _sha(binary) != manifest["compiled_model_sha256"]):
            raise ValueError("Private compiled scene cache hash mismatch")
        return mujoco.MjModel.from_binary_path(str(binary)), manifest
    started = time.monotonic()
    model = mujoco.MjModel.from_xml_path(str(scene))
    mujoco.mj_saveModel(model, str(binary), None)
    manifest.update(compiled=True, compile_wall_s=time.monotonic() - started,
                    compiled_model_sha256=_sha(binary), mujoco_version=mujoco.__version__,
                    nq=model.nq, nv=model.nv, nu=model.nu, ngeom=model.ngeom)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return model, manifest


def _apply_contact_surface_registration(cfg, target, sources, model, data):
    """Classify audited front shells without changing the frozen physical model.

    The opt-in registry is separate from scene preparation so historical task
    configs and their already compiled caches keep the original classification.
    Geometry, the allowed footprint and the scorer's signed face gates remain
    independent requirements; a registered geom is not an unconditional success.
    """
    path = cfg.get("contact_surface_registration")
    expected = cfg.get("expected_contact_surface_registration_sha256")
    if path is None and expected is None:
        return target
    if (not isinstance(path, str) or not path.strip() or not isinstance(expected, str)
            or len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected)):
        raise ValueError("Contact surface registration requires a path and explicit SHA-256")
    path = _path(path)
    if _sha(path) != expected:
        raise ValueError("Contact surface registration hash mismatch")
    registry = json.loads(path.read_text())
    if (registry.get("schema_version") != 1
            or registry.get("registration_id") != "fixed_surface_contact_geometry_v1"
            or registry.get("task_id") != cfg["task_id"]
            or registry.get("physics_model_sha256") != _sha(sources["private_scene_binary"])
            or registry.get("original_target_mapping_sha256") != _sha(sources["target_mapping"])
            or registry.get("original_registered_contact_geoms") != target["registered_contact_geoms"]):
        raise ValueError("Contact registration does not match the frozen model and original target")
    reference = registry["target_reference"]
    reference_keys = ("armor_surface_center_m", "armor_normal", "slope_axis", "across_axis",
                      "contact_region_halfsize_slope_across_m")
    if set(reference) != set(reference_keys) or any(reference[key] != target[key] for key in reference_keys):
        raise ValueError("Contact registration must retain the original target frame and allowed region")
    registered_sources = {}
    for name, record in registry["sources"].items():
        source = _path(record["path"])
        if _sha(source) != record["sha256"]:
            raise ValueError(f"Contact registration source hash mismatch: {name}")
        registered_sources[name] = source
    geoms = {geom.get("name"): geom for geom in ET.parse(registered_sources["collision_geoms"]).iter("geom")}
    assets = {mesh.get("name"): mesh for mesh in ET.parse(registered_sources["collision_assets"]).iter("mesh")}
    records = registry["additional_contact_geoms"]
    if (len(records) != 8 or sorted(record["source_partition_index"] for record in records) != list(range(8))
            or len({record["geom"] for record in records}) != len(records)):
        raise ValueError("Contact registration requires the eight audited field-facing plate partitions")
    normal = np.asarray(target["armor_normal"], float)
    normal /= np.linalg.norm(normal)
    center = np.asarray(target["armor_surface_center_m"], float)
    additions = []
    for record in records:
        name = record["geom"]
        if (record["source_visual"] != "visual_117"
                or record["source_plane_group_face_index_ranges_inclusive"] != [[24, 363]]
                or record["source_plane_group_face_count"] != 340
                or name in target["registered_contact_geoms"]):
            raise ValueError("Only the audited selected plate front-source group may be added")
        if name not in geoms or record["mesh"] not in assets:
            raise ValueError(f"Contact registration names an unknown source collider: {name}")
        geom = geoms[name]
        if geom.attrib != record["source_geom_attributes"] or geom.get("mesh") != record["mesh"]:
            raise ValueError(f"Contact registration collider source mismatch: {name}")
        points = np.asarray(record["source_polygon_world_m"], float)
        source_normal = np.asarray(record["source_normal_world"], float)
        shift = np.asarray(record["shell_shift_world_m"], float)
        thickness = float(record["shell_thickness_m"])
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 3
                or not np.isfinite(points).all() or not np.isfinite(source_normal).all()
                or source_normal.shape != (3,) or shift.shape != (3,)
                or not np.isclose(np.linalg.norm(source_normal), 1., rtol=0, atol=1e-9)
                or source_normal @ normal < .9999 or not np.array_equal(shift, np.zeros(3))
                or thickness != .005 or np.max(np.abs((points-center) @ normal + .0025)) > .0001):
            raise ValueError(f"Contact registration is not the audited front skin shell: {name}")
        quaternion = np.fromstring(geom.get("quat"), sep=" ")
        rotation = Rotation.from_quat(quaternion[[1, 2, 3, 0]]).as_matrix()
        local = np.fromstring(assets[record["mesh"]].get("vertex"), sep=" ").reshape(-1, 3)
        source_vertices = local @ rotation.T + np.fromstring(geom.get("pos"), sep=" ")
        reconstructed = np.vstack((points + source_normal*thickness/2,
                                   points - source_normal*thickness/2))
        expected_vertices = np.asarray(record["expected_world_vertices_m"], float)
        if expected_vertices.shape != source_vertices.shape or not np.allclose(
                source_vertices, expected_vertices, rtol=0, atol=1e-12):
            raise ValueError(f"Contact registration vertex evidence mismatch: {name}")
        distances = np.linalg.norm(reconstructed[:, None]-source_vertices[None, :], axis=2)
        reconstruction_error = float(max(distances.min(axis=0).max(), distances.min(axis=1).max()))
        # The original generator shares meshes using a 10 micrometre rounding
        # grid. Its reconstruction error is unrelated to scorer acceptance.
        if (reconstruction_error > 1e-5 or not np.isclose(
                reconstruction_error, record["reconstructed_xml_vertex_hausdorff_m"], rtol=0, atol=1e-12)):
            raise ValueError(f"Contact registration source reconstruction mismatch: {name}")
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if (identifier < 0 or model.geom_type[identifier] != mujoco.mjtGeom.mjGEOM_MESH
                or model.geom_bodyid[identifier] != 0 or not model.geom_contype[identifier]
                or not model.geom_conaffinity[identifier]):
            raise ValueError(f"Contact registration geom is absent or not the unchanged fixed collider: {name}")
        mesh = int(model.geom_dataid[identifier])
        if model.mesh(mesh).name != record["mesh"]:
            raise ValueError(f"Contact registration compiled mesh identity mismatch: {name}")
        first, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        compiled = (model.mesh_vert[first:first+count].astype(float)
                    @ data.geom_xmat[identifier].reshape(3, 3).T + data.geom_xpos[identifier])
        distances = np.linalg.norm(compiled[:, None]-source_vertices[None, :], axis=2)
        # Mesh storage is float32; this is an identity check, not contact tolerance.
        if (compiled.shape != source_vertices.shape
                or max(distances.min(axis=0).max(), distances.min(axis=1).max()) > 2e-7):
            raise ValueError(f"Contact registration compiled collider vertex mismatch: {name}")
        additions.append(name)
    resolved = copy.deepcopy(target)
    resolved["registered_contact_geoms"] = target["registered_contact_geoms"] + additions
    resolved["contact_surface_registration_id"] = registry["registration_id"]
    sources["contact_surface_registration"] = path
    sources.update({f"contact_registration_{name}": source for name, source in registered_sources.items()})
    return resolved


class Stage1Plant:
    """One active named Ver2 vehicle, retaining the rest of the extracted scene."""

    bottom_geom_name = BOTTOM_GEOM

    def __init__(self, cfg, *, output_dir=None, model_path=None, model=None):
        self.cfg = copy.deepcopy(cfg)
        _, _, self.target_mapping, self.contact, self.sources, identity = _inputs(cfg)
        self.contact_spec = self.contact
        self.target = self.target_mapping
        self.physical = dict(target=dict(center_m=self.target_mapping["armor_surface_center_m"],
                                         normal=self.target_mapping["armor_normal"]),
                             contact=copy.deepcopy(self.contact_spec),
                             draw=dict(mass_scale=1., thrust_scale=1., motor_tau_scale=1.))
        self.vehicle_config = json.loads(self.sources["vehicle_config"].read_text())
        self.profile = load_profile(self.sources["dynamics_profile"])
        cache = output_dir or cfg.get("scene_cache_directory")
        if model_path is not None:
            cache = _path(model_path).parent
        if cache is None:
            raise ValueError("Supply the prepared private scene directory for source provenance")
        cache = _path(cache)
        self.sources.update(private_scene_xml=cache / "scene.xml",
                            private_scene_binary=cache / "scene.mjb",
                            private_scene_manifest=cache / "stage1_scene.json")
        manifest = json.loads(self.sources["private_scene_manifest"].read_text())
        if (manifest["inputs"] != identity or not manifest.get("compiled")
                or manifest.get("mujoco_version") != mujoco.__version__
                or _sha(self.sources["private_scene_binary"]) != manifest["compiled_model_sha256"]
                or _sha(self.sources["private_scene_xml"]) != manifest["scene_sha256"]):
            raise ValueError("Private compiled scene does not match its source, runtime and compilation manifest")
        self.target_mapping = copy.deepcopy(manifest["resolved_target_mapping"])
        self.target = self.target_mapping
        if model is None:
            if model_path is None:
                model_path = self.sources["private_scene_binary"]
            model_path = _path(model_path)
            if model_path.suffix != ".mjb":
                raise ValueError("Use the explicitly compiled private Stage 1 model")
            model = mujoco.MjModel.from_binary_path(str(model_path))
        self.model = model
        self.data = mujoco.MjData(model)
        self.body_id = model.body("drone_1").id
        self.joint_id = model.joint("drone_1_free").id
        if (model.jnt_type[self.joint_id] != mujoco.mjtJoint.mjJNT_FREE
                or model.jnt_bodyid[self.joint_id] != self.body_id):
            raise ValueError("The named active joint must be the active aircraft free joint")
        self.qpos_address = self.qa = int(model.jnt_qposadr[self.joint_id])
        self.dof_address = self.va = int(model.jnt_dofadr[self.joint_id])
        self.qpos_slice = slice(self.qa, self.qa + 7)
        self.qvel_slice = slice(self.va, self.va + 6)
        self.motor_ids = self.actuator_ids = np.array([model.actuator(f"drone_1_motor_{i}").id for i in range(4)])
        self.actuator_state_ids = model.actuator_actadr[self.motor_ids].astype(int)
        if np.any(self.actuator_state_ids < 0) or not np.all(model.actuator_actnum[self.motor_ids] == 1):
            raise ValueError("Each active motor must retain its one-dimensional lag state")
        verify_compiled(model, self.body_id, self.motor_ids, self.profile)
        if not np.isclose(model.opt.timestep, .001, rtol=0, atol=1e-12):
            raise ValueError("Private scene must use one millisecond intervals")
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "blue_base_slide") >= 0:
            raise ValueError("Stage 1 target platform must be fixed")
        self.bottom_geom_id = model.geom(BOTTOM_GEOM).id
        if model.geom_bodyid[self.bottom_geom_id] != self.body_id:
            raise ValueError("Designated contact fixture must belong to the active aircraft")
        if (not np.allclose(model.geom_pos[self.bottom_geom_id], self.contact["offset_body_m"], atol=1e-12)
                or not np.isclose(model.geom_size[self.bottom_geom_id, 0], self.contact["radius_m"], atol=1e-12)):
            raise ValueError("Private contact fixture differs from the frozen S1c assumption")
        material = _contact_material(self.sources["contact_fixture_config"])
        if (not np.allclose(model.geom_friction[self.bottom_geom_id], material["friction"])
                or not np.allclose(model.geom_solref[self.bottom_geom_id], material["solref"])
                or not np.allclose(model.geom_solimp[self.bottom_geom_id, :3], material["solimp"])):
            raise ValueError("Private contact material differs from the S1c engineering assumption")
        descendants = {self.body_id}
        for body in range(self.body_id + 1, model.nbody):
            if int(model.body_parentid[body]) in descendants:
                descendants.add(body)
        self.active_geom_ids = set(np.flatnonzero(np.isin(model.geom_bodyid, list(descendants))).tolist())
        self.inertia = model.body_inertia[self.body_id].copy()
        self.maximum = float(model.actuator_ctrlrange[self.motor_ids[0], 1])
        if not np.allclose(model.actuator_ctrlrange[self.motor_ids, 1], self.maximum):
            raise ValueError("The shared CTBR proxy requires the reviewed identical motor limits")
        sites = model.actuator_trnid[self.motor_ids, 0]
        arm = float(model.site_pos[sites[0], 0])
        expected_positions = np.array([[arm, arm], [arm, -arm], [-arm, -arm], [-arm, arm]])
        if (np.any(model.actuator_trntype[self.motor_ids] != mujoco.mjtTrn.mjTRN_SITE)
                or np.any(model.site_bodyid[sites] != self.body_id)
                or not np.allclose(model.site_pos[sites, :2], expected_positions, atol=1e-12)
                or not np.allclose(model.site_quat[sites], [1., 0., 0., 0.])
                or not np.allclose(model.actuator_gear[self.motor_ids, :3], [0., 0., 1.])):
            raise ValueError("Named Ver2 rotor geometry differs from the shared mixer contract")
        self.mixer = motor_mixer(arm, float(model.actuator_gear[self.motor_ids[0], 5]))
        self.inverse_mixer = np.linalg.inv(self.mixer)
        self.rate_gain = float(cfg.get("rate_gain", 17.))
        if not np.isfinite(self.rate_gain) or self.rate_gain <= 0:
            raise ValueError("CTBR rate gain must be positive and finite")
        self.hover_force = float(model.body_mass[self.body_id] * -model.opt.gravity[2])
        self.info = dict(mass_kg=float(model.body_mass[self.body_id]), inertia_kg_m2=self.inertia.tolist(),
                         rotor_arm_m=arm, yaw_torque_per_thrust_m=float(model.actuator_gear[self.motor_ids[0], 5]),
                         max_rotor_thrust_N_assumed=self.maximum)
        mujoco.mj_forward(model, self.data)
        self.target_mapping = _apply_contact_surface_registration(
            cfg, self.target_mapping, self.sources, model, self.data)
        self.target = self.target_mapping

    @property
    def state(self):
        return VehicleEstimate(self.data.qpos[self.qa:self.qa + 3].copy(),
                               self.data.qvel[self.va:self.va + 3].copy(),
                               self.data.xmat[self.body_id].reshape(3, 3).copy(),
                               self.data.qvel[self.va + 3:self.va + 6].copy(), float(self.data.time),
                               source="MUJOCO_TRUTH_STAGE1")

    def reset(self, *, position=None, velocity=None, rotation=None, omega_body=None):
        values = ((position, (3,)), (velocity, (3,)), (rotation, (3, 3)), (omega_body, (3,)))
        if any(value is not None and (np.shape(value) != shape or not np.isfinite(value).all())
               for value, shape in values):
            raise ValueError("Invalid active aircraft initial state")
        if rotation is not None and (not np.allclose(np.asarray(rotation).T @ rotation, np.eye(3), atol=1e-9)
                                     or not np.isclose(np.linalg.det(rotation), 1., atol=1e-9)):
            raise ValueError("Initial attitude must belong to SO(3)")
        mujoco.mj_resetData(self.model, self.data)
        if position is not None:
            self.data.qpos[self.qa:self.qa + 3] = position
        if velocity is not None:
            self.data.qvel[self.va:self.va + 3] = velocity
        if rotation is not None:
            quaternion = Rotation.from_matrix(rotation).as_quat()
            self.data.qpos[self.qa + 3:self.qa + 7] = quaternion[[3, 0, 1, 2]]
        if omega_body is not None:
            self.data.qvel[self.va + 3:self.va + 6] = omega_body
        self.data.act[self.actuator_state_ids] = self.hover_force / 4
        self.data.ctrl[self.motor_ids] = self.hover_force / 4
        mujoco.mj_forward(self.model, self.data)
        return self.state

    def apply_ctbr(self, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (4,) or not np.isfinite(command).all():
            raise ValueError("Expected four finite physical CTBR values")
        action = RateThrust(command[1:], float(command[0]))
        motors, saturated = apply_proxy(action, self.state.omega_body, self.inertia,
                                        self.inverse_mixer, self.rate_gain, self.maximum,
                                        return_saturation=True)
        self.data.ctrl[self.motor_ids] = motors
        return motors.copy(), saturated

    def snapshot(self):
        kind = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(self.model, kind))
        mujoco.mj_getState(self.model, self.data, state, kind)
        return state

    def restore(self, snapshot):
        kind = mujoco.mjtState.mjSTATE_INTEGRATION
        snapshot = np.asarray(snapshot, dtype=float)
        if snapshot.shape != (mujoco.mj_stateSize(self.model, kind),) or not np.isfinite(snapshot).all():
            raise ValueError("Invalid complete scene integration snapshot")
        mujoco.mj_setState(self.model, self.data, snapshot, kind)
        mujoco.mj_forward(self.model, self.data)
        return self.state
