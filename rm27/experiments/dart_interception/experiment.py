"""Truth-guided UAV versus the original dart replay, released at first contact."""
from __future__ import annotations

import hashlib
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from research.Darts.trajectory import BallisticPath, basis, target_frame
from rm27.integration.drone import add as add_drone
from rm27.experiments.aggressive_contact.control import Septic, Skill
from rm27.experiments.aggressive_contact.model import TargetState, truth_estimate

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = ROOT / "configs/dart_interception.json"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fmt(values):
    return " ".join(f"{v:.12g}" for v in values)


def quat(rotation):
    value = np.zeros(4)
    mujoco.mju_mat2Quat(value, np.asarray(rotation, float).reshape(9))
    return fmt(value)


def prescribed_dart_state(path, elapsed):
    """Original nose path and velocity-aligned attitude, with consistent rates.

    The freejoint is located at the CAD nose, not the offset center of mass.
    Its translation rate is world-frame nose velocity; rotation rate is local.
    Differentiating the same basis also preserves the original roll convention.
    This is a prescribed replay rule, not an aerodynamic model.
    """
    position, velocity = path.state(elapsed)
    rotation = basis(velocity)
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, rotation.reshape(9))
    epsilon = 1e-5
    derivative = (basis(velocity + epsilon*path.gravity)
                  - basis(velocity - epsilon*path.gravity)) / (2*epsilon)
    skew = rotation.T @ derivative
    omega = .5*np.array([skew[2, 1]-skew[1, 2], skew[0, 2]-skew[2, 0],
                        skew[1, 0]-skew[0, 1]])
    return np.r_[position, quaternion], np.r_[velocity, omega]


def build(output, config=DEFAULT_CONFIG):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = Path(config).resolve()
    cfg = json.loads(config.read_text())
    scene = ROOT / cfg["source_scene"]
    run = ROOT / cfg["source_dart_run"]
    dart_report = json.loads((run / "summary.json").read_text())
    if not dart_report["passed"] or digest(run / "dart_visual.obj") != dart_report["dart_mesh_sha256"]:
        raise ValueError("Original dart run or mesh does not match its evidence")
    source_report = json.loads((scene.parent / "summary.json").read_text())
    if not source_report["passed"]:
        raise ValueError("Isolated scene is not a validated build")
    root = ET.parse(scene).getroot()
    root.set("model", "rm27_dart_interception")
    for mesh in root.findall(".//asset/mesh"):
        path = mesh.get("file")
        if path:
            mesh.set("file", str((scene.parent / path).resolve()))
    root.find("option").set("timestep", str(cfg["physics_dt_s"]))
    root.find("option").set("integrator", "implicitfast")
    root.find("option").set("iterations", "60")
    ET.SubElement(root.find("asset"), "mesh", name="intercept_dart_cad", file=str((run / "dart_visual.obj").resolve()), maxhullvert="4")
    world = root.find("worldbody")
    restored = []
    if cfg.get("restore_hangar_scene"):
        hangar_scene = ROOT / cfg["restore_hangar_scene"]
        for geom in ET.parse(hangar_scene).getroot().findall("./worldbody/geom"):
            name = geom.get("name", "")
            if "hangar_red_" in name or name == "test_column_footing":
                world.append(copy.deepcopy(geom))
                restored.append(name)
    if cfg.get("start_slot"):
        slot = world.find(f"site[@name='{cfg['start_slot']}']")
        if slot is None:
            raise ValueError("Requested dock slot is absent")
        start = np.fromstring(slot.get("pos"), sep=" ")
        start[2] += cfg["start_slot_center_clearance_m"]
        cfg["start_center_m"] = start.tolist()
    vehicle_cfg = json.loads((ROOT / cfg["vehicle_config"]).read_text())
    initial_quat = [np.cos(np.pi/8), 0., 0., np.sin(np.pi/8)] if cfg.get("start_slot") else [1, 0, 0, 0]
    vehicle = add_drone(root, "uav", cfg["start_center_m"], initial_quat, vehicle_cfg)
    dart = ET.SubElement(world, "body", name="intercept_dart", pos=fmt(dart_report["launch_m"]))
    ET.SubElement(dart, "freejoint", name="intercept_dart_free")
    mass = cfg["dart_mass_kg_assumed"]
    length = cfg["dart_proxy_length_m_assumed"]
    radius = cfg["dart_proxy_radius_m_assumed"]
    ET.SubElement(dart, "inertial", pos=fmt([-length/2, 0, 0]), mass=str(mass),
                  diaginertia=fmt([mass*radius**2/2, mass*(length**2/12+radius**2/4), mass*(length**2/12+radius**2/4)]))
    ET.SubElement(dart, "geom", name="intercept_dart_proxy", type="capsule", size=str(radius),
                  fromto=fmt([-length, 0, 0, 0, 0, 0]), rgba="1 .24 .05 .25", group="3")
    ET.SubElement(dart, "geom", name="intercept_dart_visual", type="mesh", mesh="intercept_dart_cad",
                  contype="0", conaffinity="0", group="2", rgba="1 .4 .12 1")
    ET.SubElement(dart, "site", name="intercept_dart_nose", pos="0 0 0", size=".008", rgba="1 .2 .1 1")
    center = np.array(dart_report["target_m"])
    normal = np.array(dart_report["target_normal"])
    target = ET.SubElement(world, "body", name="intercept_target", pos=fmt(center), quat=quat(target_frame(normal)))
    width, height = dart_report["config"]["target_width_height_m"]
    ET.SubElement(target, "geom", name="intercept_target_proxy", type="box",
                  size=fmt([width/2, height/2, dart_report["config"]["target_thickness_m"]/2]),
                  contype="1" if cfg["target_proxy_collision"] else "0",
                  conaffinity="1" if cfg["target_proxy_collision"] else "0",
                  rgba=".1 .8 .2 .5", group="3")
    ET.SubElement(target, "site", name="intercept_target_center", size=".01", rgba=".1 1 .2 1")
    ET.SubElement(world, "camera", name="intercept_side", pos="-11 -6.2 3.0", xyaxes="1 0 0 0 .33 .94", fovy="48")
    ET.SubElement(world, "camera", name="intercept_overview", pos="-1 -14 10", xyaxes="1 0 0 0 .57 .82", fovy="62")
    xml = output / "scene.xml"
    ET.ElementTree(root).write(xml, encoding="unicode", xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(xml))
    model.opt.timestep = cfg["physics_dt_s"]
    mujoco.mj_saveModel(model, str(output / "model.mjb"))
    metadata = dict(config=cfg, source_scene=str(scene), source_scene_sha256=digest(scene),
                    original_dart_run=str(run), original_dart_summary_sha256=digest(run / "summary.json"),
                    original_dart_mesh_sha256=digest(run / "dart_visual.obj"),
                    config_sha256=digest(config),
                    experiment_sha256=digest(Path(__file__)),
                    controller_sha256=digest(ROOT / "rm27/experiments/aggressive_contact/control.py"),
                    vehicle_model_sha256=digest(ROOT / "rm27/integration/drone.py"),
                    ballistic_path_sha256=digest(ROOT / "research/Darts/trajectory.py"),
                    restored_hangar_geoms=restored,
                    restored_hangar_source_sha256=digest(hangar_scene) if restored else None,
                    vehicle=vehicle)
    (output / "build.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return model, cfg, dart_report, metadata


def controller(model, data, cfg, vehicle, dart_report):
    skill_cfg = json.loads((ROOT / cfg["controller_config"]).read_text())
    skill_cfg["control_dt_s"] = cfg["control_dt_s"]
    skill_cfg["controller"].update(cfg.get("controller_overrides", {}))
    target = TargetState(np.array(dart_report["target_m"]), np.array(dart_report["target_normal"]),
                         np.eye(3)*1e-6, 0.)
    estimate = truth_estimate(model, data)
    info = dict(vehicle, rotor_arm_m=json.loads((ROOT / cfg["vehicle_config"]).read_text())["rotor_center_offset_m"],
                yaw_torque_per_thrust_m=json.loads((ROOT / cfg["vehicle_config"]).read_text())["engineering_estimates"]["yaw_torque_per_thrust_m"])
    skill = Skill(skill_cfg, skill_cfg["nominal_skill"], estimate, target, info)
    skill.set_heading_reference(np.pi/4 if cfg.get("start_slot") else 0.)
    return skill


def simulate(output, config=DEFAULT_CONFIG, active=True):
    model, cfg, dart_report, metadata = build(output, config)
    output = Path(output).resolve()
    data = mujoco.MjData(model)
    start = np.array(cfg["start_center_m"], float)
    launch = np.array(dart_report["launch_m"], float)
    velocity = np.array(dart_report["launch_velocity_m_s"], float)
    flight = BallisticPath(launch, velocity, np.array([0., 0., -dart_report["config"]["gravity_m_s2"]]),
                           dart_report["flight_time_s"])
    intercept, _ = flight.state(cfg["intercept_dart_time_s"])
    intercept = intercept + np.array([cfg["intercept_lead_m"], 0., 0.])
    hover = start.copy()
    hover[2] = cfg["hover_height_m"]
    dart_joint = model.joint("intercept_dart_free")
    dq, dv = dart_joint.qposadr[0], dart_joint.dofadr[0]
    data.qpos[dq:dq+3] = [0., 0., -100.]
    data.qpos[dq+3:dq+7] = [1., 0., 0., 0.]
    proxy_id = model.geom("intercept_dart_proxy").id
    visual_id = model.geom("intercept_dart_visual").id
    model.geom_contype[proxy_id] = model.geom_conaffinity[proxy_id] = 0
    model.geom_rgba[visual_id, 3] = 0
    mujoco.mj_forward(model, data)
    skill = controller(model, data, cfg, metadata["vehicle"], dart_report)
    takeoff = Septic((start, np.zeros(3), np.zeros(3), np.zeros(3)),
                     (hover, np.zeros(3), np.zeros(3), np.zeros(3)), cfg["takeoff_s"])
    duration = cfg["intercept_dart_time_s"]
    terminal_velocity = (intercept-hover)/duration*cfg.get("intercept_terminal_velocity_scale", 0.)
    approach = Septic((hover, np.zeros(3), np.zeros(3), np.zeros(3)),
                      (intercept, terminal_velocity, np.zeros(3), np.zeros(3)), duration)
    dt = cfg["physics_dt_s"]
    control_steps = round(cfg["control_dt_s"] / dt)
    if control_steps < 1 or abs(control_steps*dt-cfg["control_dt_s"]) > 1e-10:
        raise ValueError("Control period must be a multiple of physics timestep")
    launch_step = round(cfg["launch_time_s"] / dt)
    total_steps = round((cfg["launch_time_s"] + cfg["post_launch_s"]) / dt)
    contacts = []
    base_contacts = []
    records = []
    uav_body = model.body("uav").id
    dart_body = model.body("intercept_dart").id
    target_body = model.body("intercept_target").id
    first_move = None
    takeoff_height = None
    max_prelaunch_horizontal_motion = 0.
    release = None
    guided_steps = 0
    max_position_error = 0.
    max_rotation_matrix_error = 0.
    contact_impulse = 0.
    base_impulse = 0.
    for step in range(total_steps+1):
        t = step*dt
        if step == launch_step:
            model.geom_contype[proxy_id] = model.geom_conaffinity[proxy_id] = 1
            model.geom_rgba[visual_id, 3] = 1
            takeoff_height = float(data.qpos[2])
        if step >= launch_step and release is None:
            q, v = prescribed_dart_state(flight, t-cfg["launch_time_s"])
            data.qpos[dq:dq+7] = q
            data.qvel[dv:dv+6] = v
            guided_steps += 1
        # mj_step leaves position-dependent quantities at the preceding time.
        # Refresh at the current prescribed pose BEFORE deciding on handoff.
        mujoco.mj_forward(model, data)
        if step >= launch_step and release is None:
            p, v = flight.state(t-cfg["launch_time_s"])
            max_position_error = max(max_position_error, float(np.linalg.norm(data.xpos[dart_body]-p)))
            max_rotation_matrix_error = max(max_rotation_matrix_error,
                float(np.linalg.norm(data.xmat[dart_body].reshape(3, 3)-basis(v))))
        if step % control_steps == 0:
            state = truth_estimate(model, data)
            if t < cfg["takeoff_s"]:
                desired = takeoff.sample(t)
            elif t < cfg["launch_time_s"] or not active:
                desired = (hover, np.zeros(3), np.zeros(3), np.zeros(3))
            elif t < cfg["launch_time_s"]+duration:
                desired = approach.sample(t-cfg["launch_time_s"])
            else:
                # Brake after the scheduled crossing, rather than snap to a hover reference.
                h = t-cfg["launch_time_s"]-duration
                brake = .65
                decay = np.exp(-h/brake)
                desired = (intercept+terminal_velocity*brake*(1-decay),
                           terminal_velocity*decay, -terminal_velocity*decay/brake,
                           terminal_velocity*decay/brake**2)
            data.ctrl[:] = skill.track(state, desired)[0]
        if step >= launch_step:
            for i in range(data.ncon):
                contact = data.contact[i]
                if contact.efc_address < 0 or contact.dist > 0:
                    continue
                bodies = {model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]}
                if dart_body in bodies and release is None:
                    # Latch before the contact solve. Never overwrite this body's
                    # pose/rates again, including after it separates from contact.
                    release = dict(time_s=t, dart_time_s=t-cfg["launch_time_s"],
                                   contact_geoms=[model.geom(contact.geom1).name, model.geom(contact.geom2).name],
                                   qpos=data.qpos[dq:dq+7].copy().tolist(),
                                   qvel=data.qvel[dv:dv+6].copy().tolist())
                if bodies == {uav_body, dart_body} and not contacts:
                    contacts.append(dict(time_s=t, position_m=np.array(contact.pos).tolist(),
                                         uav_center_m=data.qpos[:3].copy().tolist(),
                                         dart_nose_m=data.qpos[dq:dq+3].copy().tolist(),
                                         dart_speed_m_s=float(np.linalg.norm(data.qvel[dv:dv+3]))))
                if bodies == {target_body, dart_body} and not base_contacts:
                    base_contacts.append(dict(time_s=t, position_m=np.array(contact.pos).tolist()))
        event_frame = ((contacts and contacts[0]["time_s"] == t)
                       or (base_contacts and base_contacts[0]["time_s"] == t)
                       or (release is not None and release["time_s"] == t))
        if step % 10 == 0 or step in (launch_step, total_steps) or event_frame:
            mode = "waiting" if step < launch_step else "prescribed" if release is None else "dynamic"
            records.append((t, data.qpos.copy(), data.qvel.copy(), data.act.copy(), data.ctrl.copy(), mode))
        if first_move is None and step >= launch_step and abs(data.qpos[1]-hover[1]) > .05:
            first_move = t
        if step < launch_step:
            max_prelaunch_horizontal_motion = max(max_prelaunch_horizontal_motion,
                                                  float(np.linalg.norm(data.qpos[:2]-start[:2])))
        if step < total_steps:
            mujoco.mj_step(model, data)
            # Forces here belong to the solve that actually advanced the plant.
            for i in range(data.ncon):
                contact = data.contact[i]
                bodies = {model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]}
                if dart_body in bodies and contact.efc_address >= 0:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, i, force)
                    if bodies == {uav_body, dart_body}:
                        contact_impulse += max(0., float(force[0]))*dt
                    if bodies == {target_body, dart_body}:
                        base_impulse += max(0., float(force[0]))*dt
    np.savez_compressed(output / "trace.npz", time=np.array([r[0] for r in records]),
                        qpos=np.array([r[1] for r in records]), qvel=np.array([r[2] for r in records]),
                        act=np.array([r[3] for r in records]), ctrl=np.array([r[4] for r in records]),
                        dart_mode=np.array([r[5] for r in records]))
    interception_succeeded = bool(contacts and not base_contacts and first_move is not None
                                  and takeoff_height > start[2]+.3
                                  and max_prelaunch_horizontal_motion < .005 and contact_impulse > 0.)
    passed = interception_succeeded if active else bool(base_contacts and not contacts and base_impulse > 0.)
    summary = dict(passed=passed, interception_succeeded=interception_succeeded,
                   mode="headless hybrid replay: prescribed dart flight until first contact, then MuJoCo rigid-body physics; truth-guided UAV motors; no SITL",
                   dart_release=release,
                   prescribed_flight_validation=dict(steps=guided_steps,
                       max_nose_position_error_m=max_position_error,
                       max_rotation_matrix_error=max_rotation_matrix_error),
                   uav_dart_normal_impulse_Ns=contact_impulse,
                   base_dart_normal_impulse_Ns=base_impulse,
                   active=active, launch_time_s=cfg["launch_time_s"],
                   takeoff_center_height_at_launch_m=takeoff_height,
                   max_prelaunch_horizontal_motion_m=max_prelaunch_horizontal_motion,
                   first_post_launch_lateral_move_s=first_move,
                   post_launch_lateral_displacement_at_end_m=float(abs(data.qpos[1]-hover[1])),
                   intercept=contacts[0] if contacts else None,
                   base_proxy_contact=base_contacts[0] if base_contacts else None,
                   final_uav_center_m=data.qpos[:3].tolist(),
                   final_dart_nose_m=data.qpos[dq:dq+3].tolist(),
                   physics_steps=total_steps, recorded_frames=len(records),
                   assumptions=["Dart mass, rigid capsule and target collision proxy are engineering assumptions.",
                                "Before first contact, dart nose position and attitude follow flight-04-apex-5m's prescribed rule, not aerodynamics.",
                                "At first contact the same free body is released with matching nose velocity and body angular velocity; subsequent motion has gravity and contact, no aerodynamics.",
                                "Flight control observes MuJoCo truth and knows the nominal launch path; no camera detection or SITL.",
                                "Base visual mesh has no certified collision or damage semantics."],
                   inputs=metadata)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary
