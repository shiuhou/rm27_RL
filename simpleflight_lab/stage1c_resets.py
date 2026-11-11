"""Training-only policy-boundary state from the qualified RM27 handover witnesses."""

from pathlib import Path
from typing import NamedTuple

import numpy as np

from .stage1_experiment import ROOT, digest, load_stage1c_experiment, read_json


WITNESS_TAPES = {
    "native-first-1p0": (1.0, "witness_1p0"),
    "native-first-1p25": (1.25, "witness_1p25"),
}
BANDS = ("near", "rotation", "approach")


class InformedReset(NamedTuple):
    state: tuple
    motor_activation_N: object
    motor_command_N: object
    previous_ctbr: object
    elapsed_physics_ticks: object
    control_phase: object
    source_standoff_m: object


def _tape_arrays(path: Path, receipt: dict, dt: float, substeps: int):
    case = receipt["cases"]["nominal"]
    done = case["completed_physics_intervals"]
    if (
        receipt["status"] != "QUALIFIED" or not case["passed"] or not case["first_contact"]["accepted"]
        or receipt["initial_failure_modes"] or receipt["saturation_policy_ticks"]
        or not np.isclose(case["first_contact"]["time_s"], done * dt, atol=1e-9, rtol=0)
    ):
        raise ValueError(f"Not a qualified nominal continuous witness: {path}")
    with np.load(path, allow_pickle=False) as a:
        expected = {"position": (done, 1, 3), "quat": (done, 1, 4), "linear_velocity": (done, 1, 3),
                    "angular_velocity": (done, 1, 3), "motor_state_N": (done + 1, 1, 4),
                    "motor_command_N": ((done + substeps - 1) // substeps, 4),
                    "ctbr_N_rad_s": ((done + substeps - 1) // substeps, 4),
                    "bumper": (done + 1, 1, 3), "initial_state": (1, 13)}
        values = {key: a[key] for key in expected}
        if (any(values[key].shape != shape or not np.isfinite(values[key]).all() for key, shape in expected.items())
                or a["first_done_tick"].tolist() != [done] or a["success"].tolist() != [True] or a["dt"].item() != dt):
            raise ValueError(f"Malformed qualified witness telemetry: {path}")
    state = np.concatenate((values["initial_state"], np.concatenate((
        values["position"][:, 0], values["quat"][:, 0], values["linear_velocity"][:, 0],
        values["angular_velocity"][:, 0],
    ), axis=1)), axis=0)
    for index, label in ((done - 1, "pre_state"), (done, "post_state")):
        row = case["first_contact"][label]
        np.testing.assert_array_equal(state[index, :3], row["position_world_m"])
        np.testing.assert_array_equal(state[index, 7:10], row["velocity_world_m_s"])
        np.testing.assert_array_equal(state[index, 10:13], row["omega_body_rad_s"])
    return state, values["bumper"][:, 0], values["motor_state_N"][:, 0], values["motor_command_N"], values["ctbr_N_rad_s"]


class Stage1CStarts:
    """Read-only training starts, with no future command or schedule available to the policy."""

    def __init__(self, rows: dict, substeps: int):
        self._rows = rows
        self._substeps = substeps
        self._devices = {}

    @classmethod
    def from_witnesses(cls, handover_dir: Path):
        _, assets, task, geometry = load_stage1c_experiment()
        contract = read_json(assets["handover_contract"])
        source_sha = contract["source_sha256"]
        dt = task["physics_dt_s"]
        substeps = round(task["policy_dt_s"] / dt)
        if dt != .001 or substeps != 5 or task["policy_dt_s"] != .005:
            raise ValueError("Informed resets require exact 1 ms / 5 ms timing")
        profile = read_json(ROOT / "upstream/rm27_sim/configs/impact_recovery/dynamics_v2_150g.json")["values"]
        alpha = -np.expm1(-dt / profile["motor_time_constant_s"])
        maximum = profile["max_rotor_thrust_N"]
        hover = geometry["body"]["mass_kg"] * 9.81
        target = geometry["target"]
        center, normal, frame = map(np.asarray, (target["center_m"], target["normal"], target["target_rotation"]))
        radius = next(g["size"][0] for g in geometry["aircraft"] if g["name"] == "drone_1_stage1_contact")
        parts = {band: [] for band in BANDS}
        for name, (standoff, asset_key) in WITNESS_TAPES.items():
            prefix = handover_dir / name
            receipt_path, tape_path = prefix.with_suffix(".json"), prefix.with_suffix(".npz")
            if (tape_path.resolve() != assets[asset_key].resolve()
                    or digest(receipt_path) != contract["feasibility_evidence_sha256"][receipt_path.name]):
                raise ValueError(f"Unverified handover witness: {prefix}")
            receipt = read_json(receipt_path)
            if (receipt["standoff_com_normal_m"] != standoff
                    or receipt["frozen_private_scene_sha256"] != source_sha["private_scene_mjb"]
                    or receipt["frozen_scorer_sha256"] != source_sha["stage1_evaluation_py"]):
                raise ValueError(f"Witness physics or scorer differs from the Pilot: {prefix}")
            state, bumper, motors, held, ctbr = _tape_arrays(tape_path, receipt, dt, substeps)
            np.testing.assert_allclose(state[0, :3], center + standoff * normal, atol=1e-12, rtol=0)
            np.testing.assert_array_equal(state[0, 3:], np.r_[1., np.zeros(9)])
            np.testing.assert_allclose(motors[0], hover / 4, atol=1e-12, rtol=0)
            np.testing.assert_allclose(np.linalg.norm(state[:, 3:7], axis=1), 1., atol=1e-12, rtol=0)
            lag_target = held[np.arange(len(state) - 1) // substeps]
            np.testing.assert_allclose(motors[1:], motors[:-1] + alpha * (lag_target - motors[:-1]), atol=1e-12, rtol=0)
            if (np.any(motors < 0) or np.any(motors > maximum) or np.any(held < 0) or np.any(held > maximum)
                    or np.any(ctbr[:, 0] < .5 * hover) or np.any(ctbr[:, 0] > 1.5 * hover)
                    or np.any(np.abs(ctbr[:, 1:]) > 2.0)):
                raise ValueError(f"Witness actuator state or command exceeds the approved limits: {prefix}")
            # A tape terminates at first contact. Every selected earlier
            # policy boundary is pre-contact, with an actual previous hold.
            tick = np.arange(substeps, len(state) - 1, substeps, dtype=np.int64)
            previous = tick // substeps - 1
            gap = (bumper[tick] - center) @ normal - radius
            local = (state[tick, :3] - center) @ frame
            safety = task["safety"]
            if (np.any(local < safety["position_min_target_frame_m"]) or np.any(local > safety["position_max_target_frame_m"])
                    or np.any(np.linalg.norm(state[tick, 7:10], axis=1) > safety["max_speed_m_s"])
                    or np.any(np.linalg.norm(state[tick, 10:13], axis=1) > safety["max_body_rate_rad_s"])):
                raise ValueError(f"Witness includes unsafe intermediate states: {prefix}")
            band_masks = ((.008 <= gap) & (gap <= .055), (.055 < gap) & (gap <= .35),
                          (.35 < gap) & (tick >= round(.20 / dt)))
            inputs = np.column_stack((state[tick], motors[tick], held[previous], ctbr[previous],
                                      np.full(len(tick), standoff))).astype(np.float32)
            for band, selected in zip(BANDS, band_masks, strict=True):
                if not selected.any():
                    raise ValueError(f"Witness has no {band} reset states: {prefix}")
                parts[band].append((inputs[selected], tick[selected]))
        return cls({band: (np.concatenate([x for x, _ in items]), np.concatenate([t for _, t in items]))
                    for band, items in parts.items()}, substeps)

    def sample(self, count: int, band: str, device) -> InformedReset:
        """Sample only the chosen training band on the caller's RNG/device."""
        import torch

        device = torch.device(device)
        if device not in self._devices:
            self._devices[device] = {key: (torch.as_tensor(x, device=device), torch.as_tensor(t, device=device))
                                     for key, (x, t) in self._rows.items()}
        states, ticks = self._devices[device][band]
        ids = torch.randint(len(states), (count,), device=device)
        x, t = states[ids], ticks[ids]
        return InformedReset((x[:, :3], x[:, 3:7], x[:, 7:10], x[:, 10:13]), x[:, 13:17], x[:, 17:21],
                             x[:, 21:25], t, t.remainder(self._substeps), x[:, 25])
