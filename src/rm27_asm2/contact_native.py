"""Independent ASM2 detailed-geometry contact simulation and qualification fixtures."""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .asset import ROOT, read_json, write_result
from .contact_task import ContactScorer, ContactSpec, load_contact_spec
from .flight import FlightProfile, allocate_ctbr, load_profile
from .flight_native import _flight_mjcf
from .flight_suite import rotation_matrix
from .native import fixture_model, records, reset as reset_fixture
from .native_resolution import _before, _field_model, _native_options, _verify_frozen_geometry


class NativeContact:
    """One 142 g ASM2 on the verified, translated detailed 2027 Field subset."""

    def __init__(self, profile: FlightProfile, spec: ContactSpec | None = None, *, dt: float | None = None):
        if mujoco.__version__ != "3.12.0":
            raise RuntimeError("Use the pinned native-env MuJoCo 3.12.0 runtime")
        self.profile = profile
        self.spec = load_contact_spec() if spec is None else spec
        self.dt = self.spec.dt if dt is None else dt
        if self.dt not in (self.spec.dt, self.spec.config["comparison_dt_s"]):
            raise ValueError("Select the primary or comparison timestep from the contact contract")
        field_contact_parameters = {}
        geometry, meshes, provenance = _verify_frozen_geometry(contact_parameters=field_contact_parameters)
        if geometry != self.spec.field_geometry:
            raise ValueError("Native Field does not match the pinned contact task")
        flight = ET.fromstring(_flight_mjcf(profile))

        def configure(root):
            option = root.find("option")
            option.set("gravity", flight.find("option").get("gravity"))
            option.set("integrator", flight.find("option").get("integrator"))
            body = root.find("worldbody/body[@name='drone']")
            flight_body = flight.find("worldbody/body[@name='drone']")
            body.remove(body.find("inertial"))
            body.insert(0, flight_body.find("inertial"))
            body.extend(flight_body.findall("site"))
            root.append(flight.find("actuator"))
            for geom in root.findall("worldbody/geom"):
                params = field_contact_parameters[geom.get("name")]
                geom.set("priority", str(params["priority"]))
                geom.set("solmix", str(params["solmix"]))

        self.model, self.data, vertex_error = _field_model(self.spec.source, geometry, meshes,
                                                          self.spec.field_translation, self.dt, configure)
        actual = self.model
        if (actual.nq, actual.nv, actual.nu, actual.na) != (7, 6, 4, 4):
            raise RuntimeError("Expected the single detailed ASM2 and four filtered site actuators")
        if any(actual.geom(name).id < 0 for name in self.spec.entries):
            raise RuntimeError("An original ASM2 collider is absent from the native model")
        for name, params in field_contact_parameters.items():
            geom_id = actual.geom(name).id
            if (actual.geom_priority[geom_id] != params["priority"]
                    or actual.geom_solmix[geom_id] != params["solmix"]):
                raise RuntimeError(f"Native contact material mixing changed for {name}")
        if np.any(actual.actuator_actearly) or np.any(actual.actuator_dyntype != mujoco.mjtDyn.mjDYN_FILTEREXACT):
            raise RuntimeError("Native motors must apply current thrust before the lag update")
        body = actual.body("drone")
        principal = rotation_matrix(body.iquat)
        inertia = principal @ np.diag(body.inertia) @ principal.T
        np.testing.assert_allclose(body.mass, profile.mass_kg, atol=1e-12, rtol=0)
        np.testing.assert_allclose(body.ipos, 0., atol=1e-12, rtol=0)
        np.testing.assert_allclose(inertia, profile.inertia_kg_m2, atol=1e-12, rtol=0)
        self.metadata = dict(backend="MuJoCo", version=mujoco.__version__, physics=_native_options(actual),
                             field=provenance, field_recompiled_max_vertex_error_m=vertex_error,
                             verified_field_material_blend_geoms=len(field_contact_parameters),
                             detailed_aircraft_colliders=len(self.spec.entries),
                             profile_sha256=profile.profile_sha256, contract_sha256=self.spec.contract_sha256)
        self.tick = 0
        self.pre_state = self.state()

    @property
    def motor_thrust_N(self) -> np.ndarray:
        return self.data.act.copy()

    def state(self) -> np.ndarray:
        return np.r_[self.data.qpos, self.data.qvel]

    def reset(self, state: np.ndarray) -> list[dict]:
        state = np.asarray(state, dtype=float)
        if state.shape != (13,) or not np.isfinite(state).all() or not np.isclose(np.linalg.norm(state[3:7]), 1.):
            raise ValueError("Expected a finite state with a unit wxyz quaternion")
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = state[:7]
        self.data.qvel[:] = state[7:]
        self.data.act[:] = self.data.ctrl[:] = self.profile.hover_motor_thrust_N
        mujoco.mj_forward(self.model, self.data)
        self.tick = 0
        self.pre_state = self.state()
        return records(self.model, self.data, self.spec.entries, 0, _before(self.data))

    def set_ctbr(self, command: np.ndarray) -> bool:
        targets, saturated = allocate_ctbr(self.profile, np.asarray(command)[None, :], self.data.qvel[None, 3:6])
        self.data.ctrl[:] = targets[0]
        return bool(saturated[0])

    def step_physics(self) -> list[dict]:
        self.pre_state = self.state()
        self.tick += 1
        mujoco.mj_step(self.model, self.data)
        if (any(w.number for w in self.data.warning) or not np.isfinite(self.state()).all()
                or not np.isfinite(self.data.act).all()
                or not math.isclose(self.data.time, self.tick * self.dt, rel_tol=1e-10, abs_tol=1e-12)):
            raise RuntimeError(f"Native detailed-contact simulation failed at tick {self.tick}")
        if not self.data.ncon:
            return []
        pre = self.pre_state
        before = pre[:3], pre[7:10], rotation_matrix(pre[3:7]) @ pre[10:13]
        return records(self.model, self.data, self.spec.entries, self.tick, before)


def run_field_fixture(plant: NativeContact) -> dict:
    spec = plant.spec
    settings = spec.config["field_fixture"]
    case = read_json(ROOT / settings["path"])
    np.testing.assert_array_equal(case["field_origin_translation_m"], spec.field_translation)
    np.testing.assert_allclose(case["rotation"], spec.target_rotation, atol=1e-12, rtol=0)
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, spec.target_rotation.ravel())
    position = np.asarray(case["position"]) + settings["clearance_shift_m"] * spec.target_normal
    velocity = -settings["normal_approach_speed_m_s"] * spec.target_normal
    state = np.r_[position, quaternion, velocity, np.zeros(3)]
    scorer = ContactScorer(spec, dt=plant.dt)
    outcome = scorer.observe(plant.reset(state), state, 0)
    saturated = plant.set_ctbr(np.r_[plant.profile.hover_thrust_N, np.zeros(3)])
    decision_state = state.copy() if outcome is not None else None
    trace = []
    for _ in range(round(settings["duration_s"] / plant.dt)):
        motors = plant.motor_thrust_N
        rows = plant.step_physics()
        post = plant.state()
        if outcome is None:
            outcome = scorer.observe(rows, plant.pre_state, plant.tick)
            if outcome is not None:
                decision_state = post.copy()
        # Fixed-duration follow-through is diagnostic only. The event outcome
        # stays latched and does not depend on post-contact recovery.
        normal = spec.target_normal
        pre = plant.pre_state
        force = rotation_matrix(pre[3:7]) @ (motors @ plant.profile.axes)
        force[2] -= plant.profile.hover_thrust_N
        projected_impulse = sum(float(normal @ row["impulse_on_drone_world_Ns"]) for row in rows)
        momentum = plant.profile.mass_kg * float(normal @ (post[7:10] - pre[7:10]))
        external = plant.dt * float(normal @ force)
        trace.append(dict(tick=plant.tick, interval_end_s=plant.tick*plant.dt,
                          signed_groups=sorted({row["group"] for row in rows if row["distance_m"] <= 0}),
                          normal_post_velocity_m_s=float(normal @ post[7:10]),
                          target_normal_contact_impulse_N_s=projected_impulse,
                          normal_impulse_momentum_residual_N_s=projected_impulse + external - momentum))
    outcome = outcome or scorer.finish()
    if decision_state is None:
        decision_state = plant.state()
    first_tick = outcome["first_touch"]["tick"] if outcome["first_touch"] else plant.tick + 1
    return dict(initial_state=state.tolist(), end_state=decision_state.tolist(),
                final_diagnostic_state=plant.state().tolist(), held_trim_saturated=saturated, result=outcome,
                peak_rebound_normal_speed_m_s=max([0.]+[row["normal_post_velocity_m_s"] for row in trace if row["tick"] >= first_tick]),
                max_abs_normal_impulse_momentum_residual_N_s=max(abs(row["normal_impulse_momentum_residual_N_s"]) for row in trace),
                trajectory=trace)


def run_diagnostic_fixtures(spec: ContactSpec) -> dict:
    """Exercise positive/negative gates using physical ASM2 contacts on small fixtures."""
    settings = spec.config["diagnostic_fixtures"]
    source = read_json(ROOT / settings["path"])["fixtures"]
    cases = {case["name"]: case for case in source}
    definitions = (
        ("legal_front", "front", "QUALIFIED"),
        ("legal_rear", "rear", "QUALIFIED"),
        ("legal_both", "both", "QUALIFIED"),
        ("forbidden_body", "body_0_-1", "FORBIDDEN_AIRFRAME_FIRST_TOUCH"),
        ("forbidden_target", "front", "FORBIDDEN_TARGET_FIRST_TOUCH"),
        ("initial_overlap", "front", "INITIAL_OVERLAP"),
        ("near_miss", "clear_20um", "NO_SIGNED_TOUCH"),
        ("unloaded_touch", "front", "UNLOADED_FIRST_TOUCH"),
        ("excess_speed", "front", "CONTACT_POINT_SPEED"),
        ("coarse_both", "both", "FORBIDDEN_AIRFRAME_FIRST_TOUCH"),
    )
    model = fixture_model(spec.source, spec.dt)
    data = mujoco.MjData(model)
    box = model.geom("test_box").id
    original_margin, original_gap = model.geom_margin[box], model.geom_gap[box]
    results = {}
    for name, source_name, expected_reason in definitions:
        dt = spec.config["comparison_dt_s"] if name == "coarse_both" else spec.dt
        model.opt.timestep = dt
        case = dict(cases[source_name])
        if name not in ("initial_overlap", "near_miss"):
            case["position"] = np.asarray(case["position"]) + [0., 0., settings["clearance_shift_m"]]
            speed = settings["high_approach_speed_m_s" if name == "excess_speed" else "normal_approach_speed_m_s"]
            if name == "unloaded_touch":
                # Maintain legal approach speed while staying inside the 50 um
                # patch skin through the no-load deadline.
                speed = 1.5 * spec.config["thresholds"]["normal_speed_min_m_s"]
            case["velocity"], case["omega"] = [0., 0., -speed], [0., 0., 0.]
        model.geom_margin[box], model.geom_gap[box] = original_margin, original_gap
        if name == "unloaded_touch":
            # Isolated detector/solver fault injection. MuJoCo 3.12 detects
            # at margin + gap, and constrains below margin. Keep the first
            # signed touch inactive through the entire load deadline.
            model.geom_margin[box] = -2 * settings["normal_approach_speed_m_s"] * spec.config["thresholds"]["load_delay_max_s"]
            model.geom_gap[box] = -model.geom_margin[box] + 2 * settings["normal_approach_speed_m_s"] * spec.dt
        reset_fixture(model, data, case)
        target_center = np.array([0., 0., case["fixture_position"][2]])
        if case["fixture"] == "box":
            target_center[2] += case["fixture_halfsize"][2]
        legal = frozenset() if name == "forbidden_target" else frozenset({"test_" + case["fixture"]})
        diagnostic = replace(spec, target_center=target_center, target_rotation=np.eye(3), legal_targets=legal)
        scorer = ContactScorer(diagnostic, dt=dt)
        initial_state = np.r_[data.qpos, data.qvel]
        rows = records(model, data, spec.entries, 0, _before(data))
        outcome = scorer.observe(rows, initial_state, 0)
        for tick in range(1, round(settings["duration_s"] / dt) + 1):
            if outcome is not None:
                break
            pre = np.r_[data.qpos, data.qvel]
            before = _before(data)
            mujoco.mj_step(model, data)
            rows = records(model, data, spec.entries, tick, before)
            outcome = scorer.observe(rows, pre, tick)
        outcome = outcome or scorer.finish()
        first_groups = outcome["first_touch"]["groups"] if outcome["first_touch"] else []
        groups_match = name not in ("legal_front", "legal_rear", "legal_both") or first_groups == case["expected_groups"]
        finite = np.isfinite(np.r_[data.qpos, data.qvel]).all() and not any(w.number for w in data.warning)
        results[name] = dict(expected_reason=expected_reason, actual_groups=first_groups,
                             pass_fixture=bool(finite and groups_match and outcome["reason"] == expected_reason),
                             initial_state=initial_state.tolist(), result=outcome)
        print("ASM2_CONTACT_DIAGNOSTIC", name, outcome["reason"], first_groups, flush=True)
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="Fresh JSON evidence path")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose an unused evidence path")
    start = time.perf_counter()
    profile, spec = load_profile(), load_contact_spec()
    fixtures = run_diagnostic_fixtures(spec)
    plant = NativeContact(profile, spec)
    primary = run_field_fixture(plant)
    metadata = plant.metadata
    # One geometry compile serves both resolutions. Reset clears all state
    # and the compiled motor's filterexact uses the current simulation step.
    plant.dt = spec.config["comparison_dt_s"]
    plant.model.opt.timestep = plant.dt
    coarse = run_field_fixture(plant)
    passed = all(case["pass_fixture"] for case in fixtures.values()) and primary["result"]["qualified"]
    result = dict(pass_fixture_suite=passed, diagnostics=fixtures, detailed_field_primary=primary,
                  detailed_field_coarse=coarse, model=metadata,
                  wall_s=time.perf_counter()-start, training_transitions=0)
    write_result(args.output, result)
    print("ASM2_NATIVE_CONTACT", "PASS" if passed else "FAIL",
          f"primary={primary['result']['reason']}", f"coarse={coarse['result']['reason']}", flush=True)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
