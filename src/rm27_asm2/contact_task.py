"""Prospective ASM2 gentle-contact gates over collider-level physical evidence."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .asset import ROOT, digest, load_inputs, read_json
from .flight_suite import rotation_matrix


CONFIG = ROOT / "configs/rm27_asm2_contact.json"
PATCH_GROUPS = frozenset(("contact_front", "contact_rear"))


@dataclass(frozen=True)
class ContactSpec:
    config: dict
    contract_sha256: str
    source: Path
    entries: dict
    field_geometry: dict
    legal_targets: frozenset[str]
    target_center: np.ndarray
    target_rotation: np.ndarray
    region_halfsize_m: np.ndarray
    field_translation: np.ndarray

    @property
    def target_normal(self) -> np.ndarray:
        return self.target_rotation[:, 2]

    @property
    def dt(self) -> float:
        return self.config["native_dt_s"]

    @property
    def patch_face_z_m(self) -> float:
        return self.config["patch_face_z_m"]


def load_contact_spec() -> ContactSpec:
    config = read_json(CONFIG)
    if (config["schema"] != "rm27_asm2_contact_v1"
            or config["scope"] != "simulation_only_provisional_gentle_contact"
            or config["training_allowed"] is not False):
        raise ValueError("Unknown or non-diagnostic ASM2 contact contract")
    for key in ("field_geometry", "registration", "field_fixture", "diagnostic_fixtures"):
        record = config[key]
        if digest(ROOT / record["path"]) != record["sha256"]:
            raise ValueError(f"Frozen ASM2 contact input changed: {key}")

    collision_config, source, geometry, _ = load_inputs()
    if digest(ROOT / collision_config["candidate_usd"]) != collision_config["candidate_sha256"]:
        raise ValueError("Preserved detailed ASM2 collision candidate changed")
    entries = {entry["name"]: entry for entry in geometry["entries"]}
    field = read_json(ROOT / config["field_geometry"]["path"])
    registry = read_json(ROOT / config["registration"]["path"])
    targets = frozenset(row["name"] for row in field["static"] if row["legal"])
    registered = frozenset(registry["original_registered_contact_geoms"]) | frozenset(
        row["geom"] for row in registry["additional_contact_geoms"]
    )
    if targets != registered or len(targets) != 25 or field["registry_sha256"] != config["registration"]["sha256"]:
        raise ValueError("ASM2 contact scorer requires the original registered 25 Field colliders")
    target = field["target"]
    center = np.asarray(target["center_m"], dtype=float)
    rotation = np.asarray(target["target_rotation"], dtype=float)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0)
    if np.linalg.det(rotation) < 0:
        raise ValueError("Expected a right-handed Field target frame")
    halfsize = np.asarray(target["halfsize_m"], dtype=float)
    np.testing.assert_array_equal(center, registry["target_reference"]["armor_surface_center_m"])
    np.testing.assert_array_equal(halfsize, registry["target_reference"]["contact_region_halfsize_slope_across_m"])
    return ContactSpec(config, digest(CONFIG), source, entries, field, targets, np.zeros(3), rotation, halfsize, -center)


def _angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.rad2deg(np.arccos(np.clip(first @ second, -1.0, 1.0))))


class ContactScorer:
    """Score the first signed manifold and require timely load on its collider pair."""

    def __init__(self, spec: ContactSpec, *, dt: float | None = None):
        self.spec = spec
        self.dt = spec.dt if dt is None else dt
        self.first_touch = None
        self.first_loaded = None
        self.result = None
        self.last_tick = -1
        self.positive_separation_records = 0
        self.positive_separation_compressive_records = 0

    def _point(self, row: dict, state: np.ndarray, body_rotation: np.ndarray) -> dict:
        normal = np.asarray(row["normal_other_to_drone_world"])
        velocity = np.asarray(row["relative_velocity_world_m_s"])
        normal_velocity = float(normal @ velocity)
        position = np.asarray(row["position_world_m"])
        target_position = (position - self.spec.target_center) @ self.spec.target_rotation
        body_position = (position - state[:3]) @ body_rotation
        impulse = np.asarray(row["impulse_on_drone_world_Ns"])
        return dict(row, target_registered=row["other_geom"] in self.spec.legal_targets,
                    normal_approach_speed_m_s=-normal_velocity,
                    tangential_speed_m_s=float(np.linalg.norm(velocity - normal_velocity * normal)),
                    surface_normal_error_deg=_angle_deg(normal, self.spec.target_normal),
                    patch_normal_error_deg=_angle_deg(normal, body_rotation[:, 2]),
                    body_to_target_alignment_deg=_angle_deg(body_rotation[:, 2], self.spec.target_normal),
                    target_position_m=target_position.tolist(),
                    body_position_m=body_position.tolist(),
                    patch_face_offset_m=float(body_position[2] - self.spec.patch_face_z_m),
                    normal_impulse_N_s=float(normal @ impulse))

    @staticmethod
    def _manifold(points: list[dict], tick: int, dt: float, body_rate: float) -> dict:
        return {
            "tick": tick,
            "sample_time_s": max(tick - 1, 0) * dt,
            "interval_end_s": tick * dt,
            "groups": sorted({point["group"] for point in points}),
            "aircraft_colliders": sorted({point["geom"] for point in points}),
            "target_colliders": sorted({point["other_geom"] for point in points}),
            "signed_points": len(points),
            "body_rate_rad_s": body_rate,
            "max_penetration_m": max(-point["distance_m"] for point in points),
            "compressive_normal_impulse_N_s": sum(max(point["normal_impulse_N_s"], 0.0) for point in points),
            "records": points,
        }

    def _reject_or_accept(self, reason: str) -> dict:
        self.result = {
            "qualified": reason == "QUALIFIED",
            "reason": reason,
            "contract_sha256": self.spec.contract_sha256,
            "physics_dt_s": self.dt,
            "first_touch": self.first_touch,
            "first_loaded": self.first_loaded,
            "positive_separation_records": self.positive_separation_records,
            "positive_separation_compressive_records": self.positive_separation_compressive_records,
        }
        return self.result

    def observe(self, rows: list[dict], pre_state: np.ndarray, tick: int) -> dict | None:
        if self.result is not None:
            return self.result
        if tick <= self.last_tick:
            raise ValueError("Contact observations must arrive in increasing physics-tick order")
        self.last_tick = tick
        touching = []
        for row in rows:
            if row["distance_m"] <= 0:
                touching.append(row)
            else:
                self.positive_separation_records += 1
                self.positive_separation_compressive_records += int(row["normal_force_N"] > 0)

        limits = self.spec.config["thresholds"]
        first_touch = self.first_touch is None
        if not first_touch and (tick - self.first_touch["tick"]) * self.dt > limits["load_delay_max_s"]:
            return self._reject_or_accept("UNLOADED_FIRST_TOUCH")
        if not touching:
            return None
        pre_state = np.asarray(pre_state, dtype=float)
        if pre_state.shape != (13,) or not np.isfinite(pre_state).all():
            raise ValueError("Expected finite world pose, linear velocity and body rates at signed contact")
        body_rotation = rotation_matrix(pre_state[3:7])
        body_rate = float(np.linalg.norm(pre_state[10:13]))
        points = [self._point(row, pre_state, body_rotation) for row in touching]
        if first_touch:
            self.first_touch = self._manifold(points, tick, self.dt, body_rate)
        if tick == 0:
            return self._reject_or_accept("INITIAL_OVERLAP")
        if any(point["group"] not in PATCH_GROUPS for point in points):
            return self._reject_or_accept("FORBIDDEN_AIRFRAME_FIRST_TOUCH")
        if any(not point["target_registered"] for point in points):
            return self._reject_or_accept("FORBIDDEN_TARGET_FIRST_TOUCH")
        if first_touch:
            if any(np.any(np.abs(point["target_position_m"][:2]) > self.spec.region_halfsize_m)
                   or abs(point["target_position_m"][2]) > limits["target_plane_error_max_m"] for point in points):
                return self._reject_or_accept("OUTSIDE_REGISTERED_FOOTPRINT")
            if any(not limits["normal_speed_min_m_s"] <= point["normal_approach_speed_m_s"] <= limits["normal_speed_max_m_s"]
                   or point["tangential_speed_m_s"] > limits["tangent_speed_max_m_s"] for point in points):
                return self._reject_or_accept("CONTACT_POINT_SPEED")
            if any(max(point["body_to_target_alignment_deg"], point["surface_normal_error_deg"],
                       point["patch_normal_error_deg"]) > limits["alignment_max_deg"] for point in points):
                return self._reject_or_accept("CONTACT_ALIGNMENT")
            if body_rate > limits["body_rate_max_rad_s"]:
                return self._reject_or_accept("CONTACT_BODY_RATE")
        if any(-point["distance_m"] > limits["penetration_max_m"] for point in points):
            return self._reject_or_accept("CONTACT_PENETRATION")

        first_pairs = {(point["geom"], point["other_geom"]) for point in self.first_touch["records"]}
        loaded = [point for point in points if point["normal_impulse_N_s"] > 0
                  and (point["geom"], point["other_geom"]) in first_pairs]
        if sum(point["normal_impulse_N_s"] for point in loaded) >= limits["normal_impulse_min_N_s"]:
            self.first_loaded = self._manifold(loaded, tick, self.dt, body_rate)
            return self._reject_or_accept("QUALIFIED")
        return None

    def finish(self) -> dict:
        if self.result is not None:
            return self.result
        return self._reject_or_accept("UNLOADED_FIRST_TOUCH" if self.first_touch is not None else "NO_SIGNED_TOUCH")
