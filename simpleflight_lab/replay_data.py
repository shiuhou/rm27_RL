"""Verified MuJoCo histories and original render meshes for the RM27 viewer.

This module never integrates dynamics or evaluates a policy. Recorded playback
uses original 1 ms samples, including the initial state and terminal interval.
Viewer-only render meshes are exported separately from collision geometry.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "configs/rm27_visual_replay.json"


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        h = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
        return h.hexdigest()


def load_manifest(path: Path = MANIFEST) -> dict:
    manifest = json.loads(Path(path).read_text())
    if manifest.get("schema") != "rm27_visual_replay_v1":
        raise ValueError("Unsupported replay manifest")
    for item in manifest["files"].values():
        source = ROOT / item["path"]
        if digest(source) != item["sha256"]:
            raise ValueError(f"Viewer source integrity mismatch: {item['path']}")
    return manifest


@dataclass(frozen=True)
class ReplayEpisode:
    demo_id: str
    case: str
    label: str
    controller: str
    initial_description: str
    checkpoint: Path | None
    qualified: bool
    failure_modes: tuple[str, ...]
    first_contact: dict | None
    summary: dict
    time_s: np.ndarray
    position: np.ndarray
    quaternion_wxyz: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity_body: np.ndarray
    bumper: np.ndarray
    motor_state_N: np.ndarray
    commands_ctbr: np.ndarray
    policy_dt_s: float
    source_sha256: str
    initial_motor_command_N: np.ndarray
    previous_ctbr_N_rad_s: np.ndarray

    @property
    def state(self) -> np.ndarray:
        return np.column_stack((self.position, self.quaternion_wxyz, self.linear_velocity, self.angular_velocity_body))

    @property
    def start_time_s(self) -> float:
        return self.takeover_time_s

    @property
    def initial_tick(self) -> int:
        return round(self.takeover_time_s / .001)

    @property
    def velocity(self) -> np.ndarray:
        return self.linear_velocity

    @property
    def omega_body(self) -> np.ndarray:
        return self.angular_velocity_body

    @property
    def path_hash(self) -> str:
        return self.source_sha256

    @property
    def elapsed_s(self) -> np.ndarray:
        return self.time_s - self.time_s[0]

    @property
    def takeover_time_s(self) -> float:
        return float(self.time_s[0])

    @property
    def duration_s(self) -> float:
        return float(self.time_s[-1] - self.time_s[0])

    def sample(self, elapsed_s: float) -> dict:
        """Return nearest recorded tick; never extrapolate or reintegrate."""
        tick = int(np.clip(round(elapsed_s / (self.time_s[1] - self.time_s[0])), 0, len(self.time_s) - 1))
        command = min(int(self.elapsed_s[tick] / self.policy_dt_s), len(self.commands_ctbr) - 1)
        return dict(index=tick, time_s=float(self.time_s[tick]), elapsed_s=float(self.elapsed_s[tick]),
                    position=self.position[tick], quaternion_wxyz=self.quaternion_wxyz[tick],
                    linear_velocity=self.linear_velocity[tick], angular_velocity_body=self.angular_velocity_body[tick],
                    bumper=self.bumper[tick], motor_state_N=self.motor_state_N[tick],
                    ctbr_N_rad_s=self.commands_ctbr[command],
                    contact_visible=bool(self.first_contact and self.time_s[tick] + 1e-9 >= self.first_contact["time_s"]),
                    done=tick == len(self.time_s) - 1)


def load_episode(demo_id: str, case_name: str | None = None, manifest_path: Path = MANIFEST) -> ReplayEpisode:
    manifest = load_manifest(manifest_path)
    demo = manifest["demos"][demo_id]
    source = manifest["files"][demo["trace"]]
    summary_path = ROOT / manifest["files"][demo["summary"]]["path"]
    summary = json.loads(summary_path.read_text())
    case_name = case_name or demo["case"]
    case = summary["cases"][case_name]
    if summary["frozen_private_scene_sha256"] != manifest["files"]["native_scene"]["sha256"]:
        raise ValueError("Replay result refers to a different full-field scene")
    if demo_id != "witness":
        actor = summary.get("actor")
        checkpoint_sha = actor["checkpoint_sha256"] if isinstance(actor, dict) else summary["checkpoint_sha256"]
        if checkpoint_sha != manifest["files"]["checkpoint"]["sha256"]:
            raise ValueError("Replay actor differs from the selected checkpoint")
    elif summary["frozen_scorer_sha256"] != manifest["files"]["scorer"]["sha256"]:
        raise ValueError("Witness scorer differs from the frozen scorer")
    with np.load(ROOT / source["path"], allow_pickle=False) as z:
        indices = np.flatnonzero(z["names"] == case_name)
        if len(indices) != 1:
            raise ValueError("Replay case must have exactly one state history")
        index = int(indices[0])
        n = int(z["first_done_tick"][index])
        dt = float(z["dt"])
        if dt != .001 or n < 1 or bool(z["success"][index]) != bool(case["passed"]):
            raise ValueError("Replay ticks, timing or result differs from the original evaluator")
        initial = z["initial_state"][index]
        arrays = {}
        for key, first in (("position", initial[:3]), ("quat", initial[3:7]),
                           ("linear_velocity", initial[7:10]), ("angular_velocity", initial[10:13])):
            arrays[key] = np.concatenate((first[None], z[key][:n, index]), axis=0)
        bumper = z["bumper"][:n+1, index].copy()
        motors_key = "motor_state_N" if "motor_state_N" in z else "motor_state"
        motors = z[motors_key][:n+1, index].copy()
        command_key = "ctbr_N_rad_s" if "ctbr_N_rad_s" in z else "ctbr_command"
        commands = z[command_key].copy()
        if commands.ndim == 3:
            commands = commands[:, index]
        commands = commands[:int(np.ceil(n / 5))]
        if any(not np.isfinite(a).all() for a in (*arrays.values(), bumper, motors, commands)):
            raise ValueError("Non-finite data before the authoritative replay terminal interval")
        if not np.allclose(np.linalg.norm(arrays["quat"], axis=1), 1, atol=1e-9, rtol=0):
            raise ValueError("Replay quaternion normalization differs")
    start = case.get("reset", {}).get("takeover_tick", 0) * dt
    time_s = start + np.arange(n + 1) * dt
    if abs(time_s[-1] - case["episode_elapsed_s"]) > 1e-9:
        raise ValueError("Replay clock does not preserve the original evaluation clock")
    if start:
        with np.load(ROOT / manifest["files"]["prefixes"]["path"], allow_pickle=False) as bank:
            bank_index = int(np.flatnonzero(bank["names"] == case_name)[0])
            if not np.array_equal(bank["initial_state"][bank_index], initial):
                raise ValueError("Replay initial state differs from the validated dynamic snapshot")
            if not np.array_equal(bank["initial_motor_state"][bank_index], motors[0]):
                raise ValueError("Replay initial actuator differs from the validated dynamic snapshot")
            initial_motor_command = bank["initial_motor_command"][bank_index].copy()
            previous_ctbr = bank["previous_ctbr"][bank_index].copy()
    else:
        initial_motor_command = motors[0].copy()
        previous_ctbr = np.array((float(motors[0].sum()), 0., 0., 0.))
    return ReplayEpisode(demo_id, case_name, demo["label"], demo["controller"], demo["initial_description"],
                         None if demo_id == "witness" else ROOT / manifest["files"]["checkpoint"]["path"],
                         bool(case["passed"]), tuple(case["failure_modes"]), case["first_contact"], case,
                         time_s, arrays["position"], arrays["quat"], arrays["linear_velocity"],
                         arrays["angular_velocity"], bumper, motors, commands, .005, source["sha256"],
                         initial_motor_command, previous_ctbr)


def load_demos(manifest_path: Path = MANIFEST) -> dict[str, ReplayEpisode]:
    return {name: load_episode(demo_id, manifest_path=manifest_path)
            for name, demo_id in (("contact", "ppo_contact"), ("witness", "witness"), ("failure", "ppo_failure"))}


ReplayClip = ReplayEpisode


def load_visual_geometry(manifest_path: Path = MANIFEST) -> tuple[dict, dict[str, np.ndarray]]:
    manifest = load_manifest(manifest_path)
    meta = json.loads((ROOT / manifest["files"]["visual_geometry"]["path"]).read_text())
    if meta["model_sha256"] != manifest["files"]["native_scene"]["sha256"]:
        raise ValueError("Visual geometry came from a different compiled scene")
    with np.load(ROOT / manifest["files"]["visual_meshes"]["path"], allow_pickle=False) as z:
        meshes = {key: z[key].copy() for key in z.files}
    return meta, meshes


def export_visual_geometry(output: Path, scene: Path) -> None:
    """Decode original rendering triangles, retaining compiled mesh/geom frames."""
    import mujoco

    if mujoco.__version__ != "3.12.0":
        raise ValueError("Frozen compiled render scene requires MuJoCo 3.12.0")
    if any(output.with_suffix(suffix).exists() for suffix in (".json", ".npz")):
        raise FileExistsError("Visual geometry export is exclusive")
    expected = "8023b223fdfdf7829ba2478c580e8f81d82dcd497f564b28a5afdf9dbf156b9a"
    if digest(scene) != expected:
        raise ValueError("Native scene integrity mismatch")
    model = mujoco.MjModel.from_binary_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    active = model.body("drone_1").id
    visual = (model.geom_group == 2) & (model.geom_rgba[:, 3] > 0)
    aircraft_ids = np.flatnonzero(visual & (model.body_rootid[model.geom_bodyid] == active))
    static_ids = np.flatnonzero(visual & (model.body_weldid[model.geom_bodyid] == 0))
    arrays = {}

    def row(index, dynamic):
        if model.geom_type[index] != mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError("Unexpected nonmesh source visual")
        mid = int(model.geom_dataid[index])
        key = f"mesh_{mid}"
        if f"{key}_vertices" not in arrays:
            va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
            fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
            arrays[f"{key}_vertices"] = model.mesh_vert[va:va+vn].copy()
            arrays[f"{key}_faces"] = model.mesh_face[fa:fa+fn].copy()
        quat = model.geom_quat[index].copy()
        if not dynamic:
            quat = np.empty(4)
            mujoco.mju_mulQuat(quat, data.xquat[model.geom_bodyid[index]], model.geom_quat[index])
        return dict(name=model.geom(int(index)).name, id=int(index), type="mesh", mesh=key,
                    position_m=(model.geom_pos[index] if dynamic else data.geom_xpos[index]).tolist(),
                    quaternion_wxyz=quat.tolist(), rgba=model.geom_rgba[index].tolist())

    meta = dict(schema="rm27_original_render_meshes_v1", model_sha256=expected,
                scope="Viewer-only original compiled render triangles; no collision, dynamics or geometry changes",
                coordinates="NWU world / FLU body. Compiled mesh vertices include asset scale and mesh frame; compose only geom pose.",
                aircraft=[row(i, True) for i in aircraft_ids], static=[row(i, False) for i in static_ids])
    if len(meta["aircraft"]) != 8 or len(meta["static"]) != 35:
        raise ValueError("Unexpected original visual mesh count")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.with_suffix(".npz").open("xb") as stream:
        np.savez_compressed(stream, **arrays)
    with output.with_suffix(".json").open("x") as stream:
        stream.write(json.dumps(meta, indent=2, allow_nan=False) + "\n")


def export_visual_usd(output: Path, geometry_prefix: Path) -> None:
    """Create a visual-only USD; all transforms retain native world/body axes."""
    from pxr import Gf, Sdf, Tf, Usd, UsdGeom, Vt

    if output.exists():
        raise FileExistsError("Visual USD export is exclusive")
    meta = json.loads(geometry_prefix.with_suffix(".json").read_text())
    with np.load(geometry_prefix.with_suffix(".npz"), allow_pickle=False) as z:
        arrays = {key: z[key].copy() for key in z.files}
    stage = Usd.Stage.CreateNew(str(output.resolve()))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    root = UsdGeom.Xform.Define(stage, "/Visuals")
    root.GetPrim().SetCustomDataByKey("rm27:sourceMjbSha256", meta["model_sha256"])
    root.GetPrim().SetCustomDataByKey("rm27:visualOnly", True)
    stage.SetDefaultPrim(root.GetPrim())
    for category, name in (("aircraft", "Drone"), ("static", "Field")):
        UsdGeom.Xform.Define(stage, f"/Visuals/{name}")
        for row in meta[category]:
            mesh = UsdGeom.Mesh.Define(stage, f"/Visuals/{name}/{Tf.MakeValidIdentifier(row['name'])}")
            points = arrays[f"{row['mesh']}_vertices"]
            faces = arrays[f"{row['mesh']}_faces"]
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(points, dtype=np.float32)))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(faces.reshape(-1), dtype=np.int32)))
            mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
            mesh.CreateDoubleSidedAttr(False)
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*row["rgba"][:3])])
            mesh.CreateDisplayOpacityAttr([float(row["rgba"][3])])
            mesh.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0).tolist()), Gf.Vec3f(*points.max(axis=0).tolist())])
            mesh.AddTranslateOp().Set(Gf.Vec3d(*row["position_m"]))
            q = row["quaternion_wxyz"]
            mesh.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
            mesh.GetPrim().CreateAttribute("rm27:originalGeomName", Sdf.ValueTypeNames.String).Set(row["name"])
    stage.GetRootLayer().Save()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-visuals", type=Path)
    parser.add_argument("--export-usd", type=Path)
    parser.add_argument("--geometry-prefix", type=Path, default=ROOT / ".agents/runs/rm27-viewer/visual-geometry")
    parser.add_argument("--scene", type=Path, default=ROOT / ".agents/runs/rm27-stage1/recovered/private/scene/scene.mjb")
    args = parser.parse_args()
    if args.export_visuals:
        export_visual_geometry(args.export_visuals, args.scene)
    elif args.export_usd:
        export_visual_usd(args.export_usd, args.geometry_prefix)
    else:
        for demo_id in load_manifest()["demos"]:
            replay = load_episode(demo_id)
            print(f"{demo_id}: {replay.label}; {len(replay.time_s)} original states; "
                  f"clock {replay.time_s[0]:.3f}–{replay.time_s[-1]:.3f}s; native qualified={replay.qualified}")


if __name__ == "__main__":
    main()
