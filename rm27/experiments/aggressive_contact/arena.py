"""Compose the existing RM27 field/dock assets and execute a single-body mission."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from rm27.integration.base_target import add as add_target
from rm27.integration.drone import add as add_drone
from .control import Septic, Skill
from .evidence import snapshot, write_json
from .experiment import run_episode
from .model import ROOT, TargetState, scenario, truth_estimate, unit, target_frame


def compose(cfg, scene_kind, output, *, source_path=None, asset_root_map=None):
    """Only rewrite an output copy; hash all original assets before loading."""
    settings = json.loads((ROOT/"configs/contact_arena.json").read_text())
    src = settings["sources"][scene_kind]
    source = Path(source_path).resolve() if source_path else ROOT/src["path"]
    if hashlib.sha256(source.read_bytes()).hexdigest() != src["sha256"]:
        raise ValueError("Arena source changed: audit geometry before updating its hash")
    field = ROOT/"rm27/simulation/field/rmuc2026_battlefield_mjcf"
    if subprocess.check_output(["git", "-C", str(field), "rev-parse", "HEAD"], text=True).strip() != settings["field_commit"]:
        raise ValueError("Field revision differs from reviewed composition")
    root = ET.parse(source).getroot()
    compiler = root.find("compiler")
    assets, remapped = {}, []
    for asset in root.findall("asset/*"):
        filename = asset.get("file")
        if not filename:
            continue
        directory = compiler.get("meshdir" if asset.tag == "mesh" else "texturedir", compiler.get("assetdir", "")) if compiler is not None else ""
        old = (source.parent/directory/filename).resolve()
        new = old
        # Explicit known repository migration, never search by basename or substitute geometry.
        prefix = str(ROOT/"rm_uav_lab_m600/rmuc2026_battlefield_mjcf")+"/"
        if str(old).startswith(prefix):
            new = field/str(old)[len(prefix):]
            remapped.append(dict(old=str(old), new=str(new)))
        for before,after in (asset_root_map or {}).items():
            prefix=str(Path(before))+"/"
            if str(old).startswith(prefix):
                new=Path(after)/str(old)[len(prefix):]
                remapped.append(dict(old=str(old),new=str(new)))
                break
        assets[str(new)] = hashlib.sha256(new.read_bytes()).hexdigest()
        asset.set("file", str(new))
    if compiler is not None:
        for key in ("meshdir", "texturedir", "assetdir"):
            compiler.attrib.pop(key, None)
    original = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    original_data = mujoco.MjData(original)
    mujoco.mj_forward(original, original_data)
    slot = original_data.site_xpos[original.site("slot_1").id].copy()
    world = root.find("worldbody")
    for body in list(world.findall("body")):
        if body.get("name", "").startswith("drone_") or body.get("name") == "rm27_base_frame":
            world.remove(body)
    for act in root.findall("actuator"):
        root.remove(act)
    target_cfg = json.loads((ROOT/"configs/base_target.json").read_text())
    add_target(root, target_cfg)
    target_body = world.find("body[@name='rm27_base_frame']")
    armor = target_body.find("geom[@name='rm27_base_armor']")
    armor.set("name", "armor")  # explicit research evaluator alias
    c = copy.deepcopy(cfg)
    c["target"]["center_m"] = target_cfg["armor_surface_center_m"]
    c["target"]["normal"] = target_cfg["armor_normal"]
    c["target"]["dimensions_m"] = target_cfg["armor_dimensions_m"]
    c["target"]["in_plane_rotation_deg"] = 90.
    origin_delta = np.array(c["target"]["center_m"])-cfg["target"]["center_m"]
    for key in ("recovery_volume_min_m", "recovery_volume_max_m"):
        c["gates"][key] = (np.array(cfg["gates"][key])+origin_delta).tolist()
    contact_attrs = dict(friction=f'{c["target"]["friction"]} .005 .0001',
                         solref=f'{c["target"]["solref_s"]} {c["target"]["solref_damping_ratio"]}', solimp=".95 .99 .001")
    armor.attrib.update(contact_attrs)
    rim = ET.SubElement(world, "body", name="research_forbidden_rim", pos=" ".join(map(str, c["target"]["center_m"])), quat=armor.get("quat"))
    x, y, thick = np.array(c["target"]["dimensions_m"])/2
    ox, oy = c["target"]["rim_outer_halfsize_m"]
    for sign in (-1, 1):
        for axis, pos, size in (("x", [sign*(ox+x)/2, 0, -thick], [(ox-x)/2, oy, thick]),
                                ("y", [0, sign*(oy+y)/2, -thick], [x, (oy-y)/2, thick])):
            ET.SubElement(rim, "geom", name=f"rim_{axis}_{sign}", type="box", pos=" ".join(map(str, pos)),
                          size=" ".join(map(str, size)), rgba=".5 .3 .3 .25", **contact_attrs)
    vcfg = json.loads((ROOT/c["vehicle_config"]).read_text())
    initial = slot + [0, 0, settings["start_height_above_slot_m"]]
    info = add_drone(root, "uav", initial, [1, 0, 0, 0], vcfg)
    info.update(rotor_arm_m=vcfg["rotor_center_offset_m"], yaw_torque_per_thrust_m=vcfg["engineering_estimates"]["yaw_torque_per_thrust_m"])
    uav = world.find("body[@name='uav']")
    ET.SubElement(uav, "geom", name="bumper", type="sphere", pos=" ".join(map(str, c["contact"]["offset_body_m"])),
                  size=str(c["contact"]["radius_m"]), rgba=".1 .9 .3 1", **contact_attrs)
    # Match the research plant's vehicle contact material without changing field colliders.
    for geom in uav.findall("geom"):
        geom.attrib.update(contact_attrs)
    root.find("option").attrib.update(timestep=str(c["physics_dt_s"]), integrator="implicitfast", iterations="60", tolerance="1e-10")
    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    data.act[:] = vcfg["mass_kg"]*9.81/4
    mujoco.mj_forward(model, data)
    uid = model.body("uav").id
    if any(uid in model.geom_bodyid[contact.geom] for contact in data.contact):
        raise ValueError("Initial vehicle collision; no flight allowed")
    np.testing.assert_allclose(data.geom_xmat[model.geom("armor").id].reshape(3, 3), target_frame(c["target"]), atol=1e-7)
    # Verify every retained source geom has exactly the original pose, size and collision masks.
    verified = 0
    for i in range(original.ngeom):
        name = original.geom(i).name
        if name.startswith(("drone_", "rm27_base_")):
            continue
        j = model.geom(name).id
        for a, b in ((original_data.geom_xpos[i], data.geom_xpos[j]), (original_data.geom_xmat[i], data.geom_xmat[j]),
                     (original.geom_size[i], model.geom_size[j])):
            np.testing.assert_allclose(a, b, atol=1e-8, rtol=0)
        assert original.geom_contype[i] == model.geom_contype[j] and original.geom_conaffinity[i] == model.geom_conaffinity[j]
        verified += 1
    evidence = dict(passed=True, scene_kind=scene_kind, source=str(source), source_sha256=src["sha256"],
                    field_commit=settings["field_commit"], assets_sha256=assets, migrated_asset_paths=remapped,
                    retained_geometries_verified=verified, slot_world_m=slot.tolist(), aircraft=info,
                    target=target_cfg, research_contact=c["target"], settings=settings,
                    limitations=["Airborne start; no takeoff/landing certification", "Base tower and marker remain visual-only",
                                 "Forbidden rim is an added assumed collision proxy", "Existing field collision masks and poses preserved"])
    (output/"scene.xml").write_text(xml)
    write_json(output/"composition.json", evidence)
    return c, (model, data, info, xml), slot, settings


class Piecewise:
    def __init__(self, states, times):
        self.times = np.cumsum([0.]+list(times))
        self.segments = [Septic(a, b, t) for a, b, t in zip(states[:-1], states[1:], times)]

    def sample(self, t):
        i = min(len(self.segments)-1, max(0, int(np.searchsorted(self.times, t, side="right")-1)))
        return self.segments[i].sample(t-self.times[i])


def transfer_controller(source, dest):
    for key in ("motor_estimate", "previous_motor_command", "last_Rd", "last_time",
                "specific_thrust_ratio", "previous_velocity", "previous_rotation",
                "heading_yaw", "heading_rate"):
        setattr(dest, key, copy.deepcopy(getattr(source, key)))


def mission(cfg, parameters, output, scene_kind="full", view=False, speed=1., seed=None, scale=1.):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    c, bundle, slot, settings = compose(cfg, scene_kind, output)
    model, data, info, xml = bundle
    physical = scenario(c, seed, scale)
    if seed is not None:
        uid = model.body("uav").id
        model.body_mass[uid] *= physical["draw"]["mass_scale"]
        model.body_inertia[uid] *= physical["draw"]["mass_scale"]
        model.actuator_gear[:] *= physical["draw"]["thrust_scale"]
        model.actuator_dynprm[:, 0] *= physical["draw"]["motor_tau_scale"]
        normal = unit(physical["target"]["normal"])
        center_physical = np.array(physical["target"]["center_m"])
        q = Rotation.from_matrix(target_frame(physical["target"])).as_quat()[[3, 0, 1, 2]]
        armor_id = model.geom("armor").id
        model.geom_pos[armor_id] = center_physical-normal*physical["target"]["dimensions_m"][2]/2
        model.geom_quat[armor_id] = q
        model.body_pos[model.body("research_forbidden_rim").id] = center_physical
        model.body_quat[model.body("research_forbidden_rim").id] = q
        model.site_pos[model.site("rm27_base_armor_center").id] = center_physical
        model.site_quat[model.site("rm27_base_armor_center").id] = q
        model.geom_pos[model.geom("bumper").id] = physical["contact"]["offset_body_m"]
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == uid or model.geom(g).name == "armor" or model.geom(g).name.startswith("rim_"):
                model.geom_friction[g, 0] = physical["target"]["friction"]
                model.geom_solref[g, :2] = [physical["target"]["solref_s"], physical["target"]["solref_damping_ratio"]]
        mujoco.mj_setConst(model, data)
        data.qpos[:3] += physical["draw"]["start_dp"]
        data.qpos[3:7] = Rotation.from_rotvec(physical["draw"]["start_rotvec"]).as_quat()[[3, 0, 1, 2]]
        data.qvel[:3] = physical["draw"]["start_dv"]
        mujoco.mj_forward(model, data)
    write_json(output/"physical.json", physical)
    write_json(output/"resolved_config.json", c)
    write_json(output/"parameters.json", parameters)
    center = np.array(c["target"]["center_m"])
    target = TargetState(center, unit(c["target"]["normal"]), np.zeros((3, 3)), 0.)
    controller = Skill(c, parameters, truth_estimate(model, data), target, info)
    live = None
    if view:
        from .viewing import LiveView
        live = LiveView(model, data, speed=speed)
    uid = model.body("uav").id
    rows, phases, failures, route_errors, events = [], [], set(), [], []
    def observe(m, d, phase):
        if live is not None:
            return live.update(d, phase)
    def state(p, v):
        return (np.array(p), np.array(v), np.zeros(3), np.zeros(3))
    def fly(path, phase, hold=0.):
        start = float(data.time)
        stable_time = 0.
        events.append(dict(time_s=start, phase=phase))
        for k in range(round((path.times[-1]+hold)/c["physics_dt_s"])):
            elapsed = data.time-start
            s = truth_estimate(model, data)
            if k % round(c["control_dt_s"]/c["physics_dt_s"]) == 0:
                ref = path.sample(elapsed)
                data.ctrl[:], sat = controller.track(s, ref)
                err = float(np.linalg.norm(ref[0]-s.position))
                route_errors.append(err)
                rows.append(np.r_[data.time, data.qpos.copy(), data.qvel.copy(), data.act.copy(), data.ctrl.copy(),
                                  ref[0], data.ncon, sat, controller.last_reference_state])
                phases.append(phase)
                if observe(model, data, phase) is False:
                    failures.add("VIEWER_CLOSED")
                    return
            for contact in data.contact:
                if uid in model.geom_bodyid[contact.geom] and contact.dist <= 0:
                    failures.add("UNEXPECTED_ROUTE_CONTACT")
            if err > settings["maximum_route_tracking_error_m"]:
                failures.add("ROUTE_TRACKING_ERROR")
            if failures:
                return
            if elapsed >= path.times[-1] and hold:
                stable = (np.linalg.norm(s.position-return_goal) < settings["return_position_tolerance_m"]
                          and np.linalg.norm(s.velocity) < settings["return_speed_tolerance_m_s"]
                          and s.rotation[2, 2] > np.cos(np.deg2rad(10)) and np.linalg.norm(s.omega_body) < .5236)
                stable_time = stable_time+c["physics_dt_s"] if stable else 0.
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
        if hold and stable_time < hold-2*c["physics_dt_s"]:
            failures.add("RETURN_NOT_STABLE")
    entry = center+np.array(c["start_offset_m"])
    z = settings["cruise_height_m"]
    outbound = Piecewise([state(data.qpos[:3], data.qvel[:3]),
                          state([slot[0]-1., slot[1], z], [-1.4, 0, 0]),
                          state([center[0]+2.268, slot[1], z], [-1.4, -.15, -.15]),
                          state(entry, settings["terminal_entry_velocity_m_s"])], settings["outbound_segment_times_s"])
    terminal_result = None
    return_goal = slot+[0, 0, settings["return_height_above_slot_m"]]
    try:
        fly(outbound, "OUTBOUND")
        if not failures:
            target = TargetState(center, unit(c["target"]["normal"]), np.zeros((3, 3)), float(data.time))
            terminal = Skill(c, parameters, truth_estimate(model, data), target, info)
            transfer_controller(controller, terminal)
            terminal_result = run_episode(c, parameters, physical=physical, output=output/"terminal", plant_bundle=bundle,
                                          initial_skill=terminal, observe=observe)
            trace = np.load(output/"terminal/trajectory.npz")["trace"]
            rows.extend(trace)
            phases.extend(["TERMINAL"]*len(trace))
            events.extend(terminal_result["events"])
            failures.update(terminal_result["failure_modes"])
            transfer_controller(terminal, controller)
            controller.contact_time = None
        if not failures:
            inbound = Piecewise([state(data.qpos[:3], data.qvel[:3]),
                                 state([center[0]+1.468, .5, z], [1.4, .1, 0]),
                                 state([slot[0]-1.15, slot[1], z], [1.2, 0, -.1]),
                                 state(return_goal, [0, 0, 0])], settings["return_segment_times_s"])
            fly(inbound, "RETURN", settings["return_hold_s"])
    finally:
        if live is not None:
            live.close()
        np.savez_compressed(output/"trajectory.npz", trace=np.asarray(rows), phases=np.array(phases))
        mujoco.mj_saveModel(model, str(output/"model.mjb"))
    distance = float(np.linalg.norm(data.qpos[:3]-return_goal))
    if distance > settings["return_position_tolerance_m"] or np.linalg.norm(data.qvel[:3]) > settings["return_speed_tolerance_m_s"]:
        failures.add("RETURN_NOT_REACHED")
    if any(w.number for w in data.warning):
        failures.add("SIMULATOR_WARNING")
    result = dict(passed=not failures, scene_kind=scene_kind, mode="LIVE_VIEW_TRUTH_MOTOR_MISSION" if view else "HEADLESS_TRUTH_MOTOR_MISSION",
                  failure_modes=sorted(failures), terminal=terminal_result, duration_s=float(data.time),
                  return_error_m=distance, return_speed_m_s=float(np.linalg.norm(data.qvel[:3])),
                  max_route_tracking_error_m=max(route_errors, default=0), events=events,
                  start="airborne above slot_1", finish="return-near-dock; no landing/disarm", s2_passed=False,
                  parameters=parameters)
    result.update(seed=seed, uncertainty_scale=scale)
    write_json(output/"summary.json", result)
    return result
