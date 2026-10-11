"""Run from repo root: python -m rm27 --help."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from .integration import dock, drone

ROOT = Path(__file__).resolve().parents[1]


def build(config, output, dynamics=None):
    cfg = json.loads(Path(config).read_text())
    drone_cfg = json.loads((ROOT / cfg["drone_config"]).read_text())
    if dynamics is not None:
        from .experiments.calibration import with_profile
        drone_cfg = with_profile(drone_cfg, dynamics)
    field = ROOT / cfg["field"]["path"]
    revision = subprocess.check_output(
        ["git", "-C", str(field.parent), "rev-parse", "HEAD"], text=True
    ).strip()
    if revision != cfg["field"]["commit"]:
        raise ValueError(
            "Field revision changed: explicitly review and update scene configuration"
        )
    subprocess.run(
        [
            "git",
            "-C",
            str(field.parent),
            "diff",
            "--quiet",
            "HEAD",
            "--",
            "model.xml",
            "assets",
        ],
        check=True,
    )
    source = ROOT / cfg["dock"]["source"]
    if hashlib.sha256(source.read_bytes()).hexdigest() != cfg["dock"]["source_sha256"]:
        raise ValueError(
            "Dock CAD changed: audit geometry and update the source hash explicitly"
        )
    report = dock.export(ROOT / cfg["dock"]["source"], output)
    if dynamics is not None:
        report['calibration'] = drone_cfg['calibration']
    root = ET.parse(field).getroot()
    for asset in root.find("asset"):
        if "file" in asset.attrib:
            asset.set("file", str((field.parent / asset.get("file")).resolve()))
    m = mujoco.MjModel.from_xml_path(str(field))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    support = mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_GEOM, cfg["dock"]["support_geom"]
    )
    platform = mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_BODY, cfg["dock"]["platform_body"]
    )
    if support < 0 or platform < 0 or m.geom_bodyid[support] != platform:
        raise ValueError("Platform/support names invalid")
    rotation = d.geom_xmat[support].reshape(3, 3)
    if not np.allclose(rotation[:, 2], [0, 0, 1]):
        raise ValueError("Support must be horizontal")
    yaw = np.deg2rad(cfg["dock"]["yaw_degrees"])
    footprint = 0.15 * (abs(np.cos(yaw)) + abs(np.sin(yaw)))
    if np.any(m.geom_size[support, :2] < footprint):
        raise ValueError("Dock footprint extends beyond platform")
    position = d.geom_xpos[support] + rotation[:, 2] * m.geom_size[support, 2]
    base_yaw = np.arctan2(rotation[1, 0], rotation[0, 0]) + yaw
    quat = [np.cos(base_yaw / 2), 0, 0, np.sin(base_yaw / 2)]
    dock.add_to_world(root, output / "dock_visual.stl", position, quat)
    option = root.find("option")
    option.set("integrator", "RK4")
    option.set("timestep", ".002")
    # Four physical vehicles, initially unarmed. No autopilot/mission is attached.
    c, s = np.cos(base_yaw), np.sin(base_yaw)
    dock_rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    vehicles = []
    for i, local in enumerate(dock.slot_centers().values()):
        spawn = position + dock_rotation @ (np.array(local) + [0, 0, 0.021])
        vehicles.append(drone.add(root, f"drone_{i+1}", spawn, quat, drone_cfg))
    path = output / "scene.xml"
    ET.ElementTree(root).write(path, encoding="unicode")
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    report.update(
        {
            "field_commit": revision,
            "dock_world_position_m": position.tolist(),
            "dock_world_quaternion": quat,
            "scene_geoms": model.ngeom,
            "placement": "red H platform by default; configurable, not a competition-side assignment",
            "slots_world_m": {
                name: data.site_xpos[
                    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
                ].tolist()
                for name in dock.slot_centers()
            },
            "vehicles": vehicles,
            "static_slot_side_clearance_m": (0.1425 - drone_cfg["outer_width_m"]) / 2,
            "mission_status": "not implemented; dock and four unarmed provisional vehicles only",
        }
    )
    (output / "audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    from .autopilot.px4.simulation import add_parser, dispatch
    add_parser(sub)
    p = sub.add_parser('build-cad-drone', help='Convert aircraft STEP and place one static aircraft in the isolated dock')
    p.add_argument('--scene', type=Path, default=ROOT/'artifacts/isolated-dock-base-20260916-v6/scene.xml')
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('build-isolated-dart', help='Derive v6 with the red H pedestal removed and red dart station added')
    p.add_argument('--config', type=Path, default=ROOT/'configs/isolated_red_dart.json')
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('build-isolated', help='Build an empty test world with dock, column and current blue base')
    p.add_argument('--scene', type=Path, default=ROOT/'artifacts/dock-final/scene.xml')
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser(
        "build-dock",
        help="Audit CAD, export mesh and place dock on configured H platform",
    )
    p.add_argument("--config", type=Path, default=ROOT / "configs/scene.json")
    p.add_argument("--dynamics", type=Path, help="Explicit simulation dynamics profile; does not change flight-controller parameters")
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("view", help="Open a generated scene in the MuJoCo viewer")
    p.add_argument("scene", type=Path)
    p = sub.add_parser(
        "micro-flight",
        help="Run the single-vehicle SITL takeoff, hover, return and dock landing gate",
    )
    p.add_argument("scene", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--autopilot", choices=("ardupilot", "px4"), default="ardupilot")
    p.add_argument(
        "--profile",
        choices=("diagnostic", "ekf", "flow", "mtf", "combined"),
        default="diagnostic",
        help="PX4 diagnostic guidance or EKF telemetry using external vision, flow, or combined sensors",
    )
    p.add_argument("--instance", type=int, default=31)
    p.add_argument("--dynamics", type=Path, help="Verify simulation dynamics against this profile")
    p.add_argument("--view", action="store_true", help="Watch the live flight")
    p.add_argument(
        "--speedup",
        type=float,
        help="Requested speed: default 1 with --view, otherwise 5",
    )
    p = sub.add_parser('route-flight', help='Fly from the red dock, touch blue armor and return within the configured gate')
    p.add_argument('scene', type=Path)
    p.add_argument('--audit', type=Path, required=True)
    p.add_argument('--mission', type=Path, default=ROOT/'configs/fast_armor_mission.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--instance', type=int, default=43)
    p.add_argument('--view', action='store_true')
    p.add_argument('--speedup', type=float)
    p.add_argument('--dynamics', type=Path)
    p.add_argument('--autopilot', choices=('ardupilot', 'px4'), default='ardupilot')
    p.add_argument(
        '--profile',
        choices=('diagnostic', 'ekf', 'flow', 'mtf', 'combined'),
        default='diagnostic',
        help='PX4 diagnostic guidance or EKF telemetry using external vision, flow, or combined sensors',
    )
    args = parser.parse_args()
    if args.command == 'px4-sim':
        raise SystemExit(dispatch(args))
    elif args.command == 'build-cad-drone':
        from .integration.cad_drone import build as build_cad_drone
        build_cad_drone(args.output, args.scene)
    elif args.command == 'build-isolated-dart':
        from .integration.isolated_red_dart import build as build_isolated_dart
        build_isolated_dart(args.output, args.config)
    elif args.command == 'build-isolated':
        from .integration.isolated_scene import build as build_isolated
        build_isolated(args.scene, args.output)
    elif args.command == "build-dock":
        build(args.config, args.output.resolve(), args.dynamics)
    elif args.command == "micro-flight":
        if args.autopilot == 'px4':
            from .autopilot.px4.flight import run
        else:
            from .autopilot.micro_flight import run

        kwargs = dict(view=args.view, speedup=args.speedup, dynamics=args.dynamics)
        if args.autopilot == 'px4':
            kwargs['profile'] = args.profile
        return run(args.scene, args.output, args.instance, **kwargs)
    elif args.command == 'route-flight':
        from .experiments.route_experiment import RouteExperiment
        if args.autopilot == 'px4':
            from .autopilot.px4.flight import run
        else:
            from .autopilot.micro_flight import run
        kwargs = dict(
            route=RouteExperiment(args.scene, args.audit, args.mission),
            view=args.view, speedup=args.speedup, dynamics=args.dynamics,
        )
        if args.autopilot == 'px4':
            kwargs['profile'] = args.profile
        return run(args.scene, args.output, args.instance, **kwargs)
    else:
        import mujoco.viewer

        mujoco.viewer.launch(
            model=mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
        )


if __name__ == "__main__":
    raise SystemExit(main())
