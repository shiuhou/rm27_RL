"""Independent first-contact scoring; rewards and policy inputs live elsewhere."""
from __future__ import annotations

import copy

import mujoco
import numpy as np

from rm27_contact_physics.model import unit


def stage1_geometry(plant):
    mapping, contact = plant.target_mapping, plant.contact_spec
    normal = unit(mapping["armor_normal"])
    slope, across = unit(mapping["slope_axis"]), unit(mapping["across_axis"])
    # The registry lists slope/across with a left-handed normal convention.
    # Negating the slope keeps rectangular |coordinates| unchanged and gives
    # the proper SO(3) frame needed for aircraft attitude and perturbations.
    rotation = np.column_stack((-slope, across, normal))
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)):
        raise ValueError("Registered target axes must be orthonormal")
    return dict(center=np.asarray(mapping["armor_surface_center_m"], float), normal=normal,
                target_rotation=rotation, offset_body=np.asarray(contact["offset_body_m"], float),
                radius=float(contact["radius_m"]),
                halfsize=np.asarray(mapping["contact_region_halfsize_slope_across_m"], float))


def state_record(state):
    return dict(timestamp_s=float(state.timestamp), position_world_m=state.position.tolist(),
                velocity_world_m_s=state.velocity.tolist(), rotation_body_to_world=state.rotation.tolist(),
                omega_body_rad_s=state.omega_body.tolist())


class Stage1Evaluator:
    """Stop at the first completed interval containing contact or a safety event.

    Contact pairs and forces are read from the interval solve before mj_forward.
    Kinematic contact gates use the pre-impact state; safety gates apply to both
    boundaries and take priority over a simultaneous valid contact.
    """

    protocol = "stage1_completed_interval_first_contact_v1"

    def __init__(self, config, plant):
        self.config, self.plant = config, plant
        self.geometry = stage1_geometry(plant)
        self.acceptance, self.safety = config["contact_acceptance"], config["safety"]
        self.active_geoms = set(map(int, plant.active_geom_ids))
        self.bottom = int(plant.model.geom(plant.bottom_geom_name).id)
        self.surfaces = {int(plant.model.geom(name).id)
                         for name in plant.target_mapping["registered_contact_geoms"]}
        self.reasons, self.events = set(), []
        self.first_contact = None
        self.done = False
        self.completed_intervals = 0
        self.elapsed_s = 0.
        self.saturated_s = 0.
        self.last_metrics = None
        self.start_time = self.interval_end_time = float(plant.data.time)
        self.last_interval = None

    def metrics(self, state):
        g = self.geometry
        point = state.position + state.rotation @ g["offset_body"] - g["radius"] * g["normal"]
        error = point - g["center"]
        return dict(contact_point_world_m=point.tolist(), position_error_m=float(np.linalg.norm(error)),
                    target_coordinates_m=(g["target_rotation"].T @ (state.position-g["center"])).tolist(),
                    contact_alignment_cosine=float(np.clip(state.rotation[:, 2] @ g["normal"], -1., 1.)),
                    speed_m_s=float(np.linalg.norm(state.velocity)),
                    body_rate_rad_s=float(np.linalg.norm(state.omega_body)))

    def _safety_state(self, state):
        values = np.r_[state.position, state.velocity, state.rotation.ravel(), state.omega_body,
                       state.timestamp, self.plant.data.qpos, self.plant.data.qvel, self.plant.data.act]
        if not np.isfinite(values).all():
            self.reasons.add("NUMERICAL_FAILURE")
            return
        self.last_metrics = self.metrics(state)
        local = np.asarray(self.last_metrics["target_coordinates_m"])
        if (np.any(local < self.safety["position_min_target_frame_m"])
                or np.any(local > self.safety["position_max_target_frame_m"])):
            self.reasons.add("FLIGHT_VOLUME_EXIT")
        if self.last_metrics["speed_m_s"] > self.safety["max_speed_m_s"]:
            self.reasons.add("EXCESSIVE_SPEED")
        if self.last_metrics["body_rate_rad_s"] > self.safety["max_body_rate_rad_s"]:
            self.reasons.add("EXCESSIVE_BODY_RATE")
        if np.any(self.plant.data.warning.number):
            self.reasons.add("MUJOCO_WARNING")

    def _contacts(self, *, initial=False):
        model, data = self.plant.model, self.plant.data
        records = []
        for index in range(data.ncon):
            contact = data.contact[index]
            pair = tuple(map(int, contact.geom))
            if not self.active_geoms.intersection(pair) or contact.dist > 0:
                continue
            force = np.zeros(6)
            if contact.efc_address >= 0:
                mujoco.mj_contactForce(model, data, index, force)
            if not initial and contact.efc_address < 0:
                continue
            surface = next((geom for geom in pair if geom in self.surfaces), None)
            outward = contact.frame.reshape(3, 3)[0] * (1 if pair[0] == surface else -1)
            records.append(dict(geom_ids=list(pair), geoms=[model.geom(i).name for i in pair],
                                position_world_m=contact.pos.tolist(), distance_m=float(contact.dist),
                                normal_force_N=float(force[0]), wrench_contact_frame=force.tolist(),
                                surface_outward_world=outward.tolist(),
                                designated=bool(self.bottom in pair and surface is not None)))
        return records

    def initialize(self):
        self._safety_state(self.plant.state)
        contacts = self._contacts(initial=True)
        if contacts:
            self.reasons.add("INITIAL_OVERLAP")
            self.events.append(dict(event="INITIAL_OVERLAP", time_s=self.start_time, contacts=contacts))
        self.done = bool(self.reasons)

    def _consume(self, model, data, start, saturated):
        dt = float(data.time)-float(start)
        if (not np.isclose(start, self.interval_end_time, atol=1e-12, rtol=0)
                or not np.isclose(dt, model.opt.timestep, atol=1e-12, rtol=0)):
            self.reasons.add("NUMERICAL_FAILURE")
        self.completed_intervals += 1
        self.elapsed_s += float(model.opt.timestep)
        self.saturated_s += float(model.opt.timestep) if saturated else 0.
        self.interval_end_time = float(data.time)
        contacts = self._contacts()
        self.last_interval = dict(start_time_s=float(start), end_time_s=float(data.time),
                                  contacts=contacts, proxy_saturated=bool(saturated))
        if not contacts:
            return
        if any(not contact["designated"] for contact in contacts):
            self.reasons.add("UNDESIGNATED_CONTACT")
        if any(-contact["distance_m"] > self.acceptance["max_penetration_m"] for contact in contacts):
            self.reasons.add("EXCESSIVE_PENETRATION")
        before = self._before
        g = self.geometry
        first = dict(time_s=float(data.time), interval_start_s=float(start), contacts=contacts,
                     pre_state=state_record(before),
                     pre_motor_state_N=self._pre_motor.tolist(),
                     pre_motor_commands_N=self._pre_commands.tolist(),
                     valid_geometry=False, accepted=False)
        gate_records = []
        for contact in contacts:
            point = np.asarray(contact["position_world_m"])
            local = g["target_rotation"].T @ (point-g["center"])
            point_velocity = before.velocity + np.cross(before.rotation @ before.omega_body,
                                                        point-before.position)
            vn = -float(point_velocity @ g["normal"])
            vt = float(np.linalg.norm(point_velocity+vn*g["normal"]))
            angle = float(np.rad2deg(np.arccos(np.clip(before.rotation[:, 2] @ g["normal"], -1., 1.))))
            checks = (
                (np.any(np.abs(local[:2]) > g["halfsize"]), "CONTACT_OUTSIDE_REGION"),
                (abs(local[2]) > self.acceptance["max_plane_error_m"], "CONTACT_OUTSIDE_SURFACE_PLANE"),
                (float(np.asarray(contact["surface_outward_world"]) @ g["normal"])
                 < np.cos(np.deg2rad(self.acceptance["max_surface_normal_error_deg"])), "WRONG_TARGET_FACE"),
                (not self.acceptance["normal_speed_range_m_s"][0] <= vn
                 <= self.acceptance["normal_speed_range_m_s"][1], "CONTACT_NORMAL_SPEED"),
                (vt > self.acceptance["max_tangent_speed_m_s"], "CONTACT_TANGENT_SPEED"),
                (angle > self.acceptance["max_bottom_normal_error_deg"], "CONTACT_BOTTOM_NORMAL_ALIGNMENT"),
                (np.linalg.norm(before.omega_body) > self.acceptance["max_body_rate_rad_s"], "CONTACT_BODY_RATE"),
                (contact["normal_force_N"] <= 0., "NO_COMPRESSIVE_CONTACT_FORCE"))
            for failed, reason in checks:
                if failed:
                    self.reasons.add(reason)
            gate_records.append(dict(local_coordinates_m=local.tolist(), normal_speed_m_s=vn,
                                     tangent_speed_m_s=vt, bottom_normal_error_deg=angle))
        first.update(gates=gate_records, valid_geometry=all(c["designated"] for c in contacts))
        self.first_contact = first

    def advance(self, before, saturated=False):
        if self.done:
            raise RuntimeError("Cannot advance a finished Stage 1 evaluator")
        self._safety_state(before)
        if self.reasons:
            self.done = True
            return before
        if not np.isclose(before.timestamp, self.interval_end_time, atol=1e-12, rtol=0):
            raise ValueError("Stage 1 interval must start at the last completed boundary")
        self._before = before
        self._pre_motor, self._pre_commands = self.plant.data.act.copy(), self.plant.data.ctrl.copy()
        # The common helper finishes by looking up a body named "uav" and
        # qpos[:3]. This full-field aircraft is "drone_1" with other joints
        # before it. Preserve the same interval ordering without that lookup.
        mujoco.mj_step(self.plant.model, self.plant.data)
        self._consume(self.plant.model, self.plant.data, before.timestamp, saturated)
        mujoco.mj_forward(self.plant.model, self.plant.data)
        state = self.plant.state
        self._safety_state(state)
        if self.elapsed_s >= self.config["horizon_s"]-1e-12 and self.first_contact is None:
            self.reasons.add("CONTACT_TIMEOUT")
        if self.first_contact is not None:
            self.first_contact.update(post_state=state_record(state),
                                      post_motor_state_N=self.plant.data.act.tolist(),
                                      post_motor_commands_N=self.plant.data.ctrl.tolist(),
                                      accepted=not bool(self.reasons))
            self.events.append(dict(event="FIRST_CONTACT", **copy.deepcopy(self.first_contact)))
        self.done = bool(self.reasons or self.first_contact is not None)
        return state

    def summary(self):
        return dict(task_id=self.config["task_id"], observation_schema=self.config["observation_schema"],
                    passed=bool(self.first_contact and not self.reasons),
                    failure_modes=sorted(self.reasons), first_contact=copy.deepcopy(self.first_contact),
                    episode_elapsed_s=self.elapsed_s, events=copy.deepcopy(self.events),
                    metrics=copy.deepcopy(self.last_metrics), completed_physics_intervals=self.completed_intervals,
                    saturated_time_s=self.saturated_s, evaluation_protocol=self.protocol,
                    parameter_status=self.config["parameter_status"],
                    task_success_status="PROVISIONAL_ENGINEERING_TASK_ONLY",
                    orientation_acceptance="BOTTOM_NORMAL_ALIGNMENT; YAW_UNCONSTRAINED",
                    recovery_evaluated=False)
