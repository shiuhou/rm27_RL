"""Collider-level physical evidence shared by native and PhysX diagnostics.

These records classify geometry, never qualify the new task. In particular,
the old sphere evaluator is neither patched nor used as a shell evaluator.
"""
from __future__ import annotations

import numpy as np

GROUPS = {"contact_front", "contact_rear", "body"}


def point_velocity(position, linear, angular_world, point):
    return np.asarray(linear) + np.cross(angular_world, np.asarray(point) - position)


def contact_record(*, tick, dt, group, geom, other_geom, point, normal, impulse, separation,
                   pre_position, pre_linear, pre_angular_world, collider_paths=None):
    """Record the supplied report/solver vector; do not infer a full contact wrench.

    Native callers use the resolved contact wrench. PhysX callers pass its
    reported point impulse, whose friction completeness is not established.
    Dividing by dt changes units only; it does not validate force accounting.
    """
    if group not in GROUPS or dt <= 0:
        raise ValueError("Unknown collider group or invalid physical interval")
    point, normal, impulse = (np.asarray(x, dtype=float) for x in (point, normal, impulse))
    if not np.isfinite(np.r_[point, normal, impulse, separation, pre_position, pre_linear, pre_angular_world]).all() or abs(np.linalg.norm(normal) - 1) > 1e-4:
        raise ValueError("Non-finite or non-unit contact evidence")
    velocity = point_velocity(np.asarray(pre_position), pre_linear, pre_angular_world, point)
    return dict(tick=int(tick), interval_start_s=(tick - 1) * dt, interval_end_s=tick * dt,
                group=group, geom=geom, other_geom=other_geom, collider_paths=collider_paths,
                position_world_m=point.tolist(), normal_other_to_drone_world=normal.tolist(),
                relative_velocity_world_m_s=velocity.tolist(), normal_approach_speed_m_s=float(-normal @ velocity),
                force_on_drone_world_N=(impulse / dt).tolist(), impulse_on_drone_world_Ns=impulse.tolist(),
                normal_force_N=float(normal @ impulse / dt), physics_dt_s=dt, distance_m=float(separation),
                zero_impulse=bool(np.linalg.norm(impulse) == 0), positive_separation=bool(separation > 0),
                qualification="NOT_DEFINED_FOR_ASM2")


def scorer_input(record, registered_targets):
    """Existing event field names plus new semantics, with no success decision.

    `designated` describes a front/rear-vs-registered-target pair only. It is
    not the legacy sphere scorer's `accepted`, nor a new acceptance threshold.
    """
    return dict(geoms=[record["geom"], record["other_geom"]],
                collider_paths=record["collider_paths"], contact_group=record["group"],
                position_world_m=record["position_world_m"], distance_m=record["distance_m"],
                normal_force_N=record["normal_force_N"], surface_outward_world=record["normal_other_to_drone_world"],
                relative_velocity_world_m_s=record["relative_velocity_world_m_s"],
                impulse_world_N_s=record["impulse_on_drone_world_Ns"],
                designated=record["group"] != "body" and record["other_geom"] in registered_targets,
                task_qualification="NOT_DEFINED_FOR_ASM2")


class ContactLedger:
    """Consume each reported manifold sample once per completed physics tick.

    Equal locations on distinct convex patches are retained. Only exact
    repeats of the same pair/sample/impulse in the same tick are duplicates.
    Zero-force and positive-separation samples remain visible as evidence.
    """

    def __init__(self):
        self.tick = 0
        self.records = []
        self.seen = set()
        self.duplicates = 0

    def begin(self, tick):
        if tick <= self.tick:
            raise ValueError("Contact ledger requires strictly increasing physics ticks")
        self.tick = tick
        self.records.clear()
        self.seen.clear()

    def add(self, record):
        if record["tick"] != self.tick:
            raise ValueError("Contact sample belongs to another interval")
        key = (self.tick, record["geom"], record["other_geom"], tuple(record["position_world_m"]),
               tuple(record["normal_other_to_drone_world"]), record["distance_m"],
               tuple(record["impulse_on_drone_world_Ns"]))
        if key in self.seen:
            self.duplicates += 1
            return False
        self.seen.add(key)
        self.records.append(record)
        return True


def summarize(records):
    touching = [r for r in records if r["distance_m"] <= 0]
    return dict(reported_groups=sorted({r["group"] for r in records}),
                touching_groups=sorted({r["group"] for r in touching}),
                contacting_colliders=sorted({r["geom"] for r in touching}),
                points=len(records), touching_points=len(touching),
                zero_impulse_points=sum(r["zero_impulse"] for r in records),
                positive_separation_points=sum(r["positive_separation"] for r in records),
                min_separation_m=min((r["distance_m"] for r in records), default=None),
                impulse_on_drone_world_Ns=np.sum([r["impulse_on_drone_world_Ns"] for r in records], axis=0).tolist()
                if records else [0., 0., 0.])
