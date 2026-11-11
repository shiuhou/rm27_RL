"""Replay continuous native CTBR command recordings in the verified Isaac proxy."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import traceback

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from .contact_isaac import ContactComparison, EVIDENCE, PROXY_MANIFEST
from .contact_task import load_contact_spec
from .flight import load_profile
from .flight_suite import rotation_matrix
from .proxy import load_proxy


def load_tape(path, profile):
    with np.load(path, allow_pickle=False) as archive:
        states = np.asarray(archive["states"], dtype=float)
        commands = np.asarray(archive["ctbr_N_rad_s"], dtype=float)
        motors = np.asarray(archive["motor_thrust_N"], dtype=float)
        reference = np.asarray(archive["reference"], dtype=float)
        times = np.asarray(archive["sample_times_s"], dtype=float)
        dt = float(archive["dt"].item())
        handover = archive["handover_frame"]
        outcome = str(archive["contact_outcome"].item()) if "contact_outcome" in archive else None
    frames = len(states)
    if (states.shape != (frames, 1, 13) or frames < 2
            or commands.shape != (frames - 1, 1, 4) or motors.shape != (frames, 1, 4)
            or reference.shape != (frames, 1, 3) or times.shape != (frames,)):
        raise ValueError("Expected a single continuous recording of states, CTBR, motor states and reference")
    if not all(np.isfinite(array).all() for array in (states, commands, motors, reference, times)):
        raise ValueError("The recorded state and command arrays must be finite")
    if not np.allclose(np.linalg.norm(states[:, 0, 3:7], axis=1), 1., atol=1e-6, rtol=0):
        raise ValueError("Expected unit wxyz orientation samples")
    if (dt != profile.policy_dt_s or not 0 < times[-1] <= 20.
            or not np.allclose(times[:-1], np.arange(frames - 1) * dt, atol=1e-9, rtol=0)
            or not 0 < times[-1] - times[-2] <= dt + 1e-9):
        raise ValueError("Expected a bounded 5 ms CTBR tape with an optional partial final interval")
    if handover.ndim != 0 or handover.dtype.kind not in "iu" or not 0 <= int(handover) < frames:
        raise ValueError("Handover frame is outside the recording")
    if np.any(motors[0] < 0.) or np.any(motors[0] > profile.max_rotor_thrust_N):
        raise ValueError("Initial motor thrust must be inside the frozen actuator limits")
    return states[:, 0], commands, motors[:, 0], times, int(handover), outcome


def terminal_tape(path, tape, lookback_ms):
    states, commands, motors, times, handover, outcome = tape
    report = read_json(path.with_suffix(".json"))
    if report["trace_sha256"] != digest(path) or report["result"]["first_touch"] is None:
        raise ValueError("Terminal reset requires a matching native trace with signed contact")
    first_touch_s = float(report["result"]["first_touch"]["sample_time_s"])
    reset = int(np.searchsorted(times, first_touch_s - lookback_ms * .001 + 1e-12, side="right")) - 1
    if not 0 <= handover <= reset < len(commands) or not 0 < first_touch_s <= times[-1]:
        raise ValueError("No recorded post-handover control sample meets the terminal lookback")
    source_offset = float(times[reset])
    sliced = states[reset:], commands[reset:], motors[reset:], times[reset:] - source_offset, 0, outcome
    metadata = dict(source_frame=reset, source_start_s=source_offset, requested_lookback_ms=lookback_ms,
                    native_first_signed_source_s=first_touch_s,
                    native_first_signed_elapsed_s=first_touch_s - source_offset)
    return sliced, metadata


def state_history(rows):
    poses = np.asarray([rows[0]["pre_pose_xyzw"], *[row["post_pose_xyzw"] for row in rows]])
    velocities = np.asarray([rows[0]["pre_velocity_world"], *[row["post_velocity_world"] for row in rows]])
    quaternions = poses[:, (6, 3, 4, 5)]
    rates = np.einsum("nji,nj->ni", rotation_matrix(quaternions), velocities[:, 3:])
    return np.column_stack((poses[:, :3], quaternions, velocities[:, :3], rates))


def state_errors(native, isaac):
    q, r = native[..., 3:7], isaac[..., 3:7]
    cosine = np.abs(np.sum(q * r, axis=-1)) / (np.linalg.norm(q, axis=-1) * np.linalg.norm(r, axis=-1))
    return dict(position_m=np.linalg.norm(isaac[..., :3] - native[..., :3], axis=-1),
                attitude_deg=np.rad2deg(2. * np.arccos(np.clip(cosine, 0., 1.))),
                velocity_m_s=np.linalg.norm(isaac[..., 7:10] - native[..., 7:10], axis=-1),
                body_rate_rad_s=np.linalg.norm(isaac[..., 10:13] - native[..., 10:13], axis=-1))


def first_touch_evidence(event, live, spec):
    if event is None:
        return None
    interval = event["interval"]
    signed = [point for point in event["points"] if point["distance_m"] <= 0.]
    pairs = sorted({(point["geom"], point["other_geom"], point["group"]) for point in signed})
    velocity = np.asarray([point["relative_velocity_world_m_s"] for point in signed])
    normal = np.asarray([point["normal_other_to_drone_world"] for point in signed])
    inward = np.asarray([point["normal_approach_speed_m_s"] for point in signed])
    tangent = np.linalg.norm(velocity + inward[:, None] * normal, axis=1)
    pre = live[interval["local_tick"] - 1]
    alignment = float(np.clip(rotation_matrix(pre[3:7])[:, 2] @ spec.target_normal, -1., 1.))
    return dict(
        interval_s=[interval["sample_time_s"], interval["interval_end_s"]],
        signed_points=len(signed), touching_groups=interval["touching_groups"],
        pairs=[dict(aircraft=aircraft, field=field, group=group, registered=field in spec.legal_targets,
                    designated=group != "body" and field in spec.legal_targets)
               for aircraft, field, group in pairs],
        min_separation_m=min(point["distance_m"] for point in signed),
        inward_point_speed_range_m_s=[float(inward.min()), float(inward.max())],
        max_tangent_point_speed_m_s=float(tangent.max()), pre_state=pre.tolist(),
        body_rate_norm_rad_s=float(np.linalg.norm(pre[10:13])),
        body_alignment_to_target_normal_deg=math.degrees(math.acos(alignment)),
        measured_normal_momentum_Ns=interval["measured_normal_momentum_Ns"],
        reported_normal_impulse_Ns=interval["reported_normal_impulse_Ns"],
        raw_to_measured_normal_ratio=event["reported_to_measured_normal_ratio"],
        raw_notification_packets=event["raw_notification_packets"], points=signed)


def replay(sim, path, tape, profile, spec, terminal_reset=None):
    native, commands, motors, times, handover, native_outcome = tape
    initial = native[0]
    steps = math.ceil(times[-1] / sim.dt - 1e-9)
    case = dict(name=path.stem, fixture="registered_field", position=initial[:3].tolist(),
                rotation=rotation_matrix(initial[3:7]).tolist(), velocity=initial[7:10].tolist(),
                omega=initial[10:13].tolist(), steps=steps,
                normal_reference_world=spec.target_normal.tolist(), expected_groups=["contact_front", "contact_rear"])
    result = sim.compare(case, sim.field_registered, profile, command_tape=commands,
                         initial_motor_thrust=motors[0][None, :], stop_at_signed_touch=True)["worlds"][0]
    if not result["finite"]:
        raise RuntimeError(f"Non-finite Isaac trajectory: {path.name}")
    rows = result["trajectory"]
    live = state_history(rows)
    motor_history = np.asarray([motors[0], *[row["post_motor_thrust_N"] for row in rows]])
    touch = first_touch_evidence(result["first_touching"], live, spec)
    source_offset = 0. if terminal_reset is None else terminal_reset["source_start_s"]
    if touch is not None:
        touch["source_interval_s"] = [value + source_offset for value in touch["interval_s"]]

    # The native final sample can already include the contact solve. Match
    # only regular policy boundaries through the last pre-contact state.
    pre_touch_steps = len(rows) - 1 if touch else len(rows)
    matched_frames = np.flatnonzero(np.round(times[:-1] / sim.dt).astype(int) <= pre_touch_steps)
    matched_ticks = np.round(times[matched_frames] / sim.dt).astype(int)
    errors = state_errors(native[matched_frames], live[matched_ticks])
    path_errors = {}
    phases = (("navigation", matched_frames <= handover), ("approach", matched_frames > handover))
    if terminal_reset is not None:
        phases = (("terminal", np.ones(len(matched_frames), dtype=bool)),)
    for phase, mask in phases:
        path_errors[phase] = dict(samples=int(mask.sum()),
                                  maximum={key: float(np.max(value[mask])) for key, value in errors.items()} if mask.any() else None,
                                  rms_position_m=float(np.sqrt(np.mean(errors["position_m"][mask] ** 2))) if mask.any() else None)

    handover_tick = round(times[handover] / sim.dt)
    handover_result = dict(frame=handover, time_s=float(times[handover]),
                           native_state=native[handover].tolist(), native_motor_thrust_N=motors[handover].tolist(),
                           native_com_normal_m=float((native[handover, :3] - spec.target_center) @ spec.target_normal),
                           native_inward_speed_m_s=float(-native[handover, 7:10] @ spec.target_normal))
    if (handover_tick <= pre_touch_steps
            and math.isclose(handover_tick * sim.dt, times[handover], abs_tol=1e-9, rel_tol=0.)):
        measured = live[handover_tick]
        handover_result.update(isaac_state=measured.tolist(), isaac_motor_thrust_N=motor_history[handover_tick].tolist(),
                               isaac_com_normal_m=float((measured[:3] - spec.target_center) @ spec.target_normal),
                               isaac_inward_speed_m_s=float(-measured[7:10] @ spec.target_normal),
                               errors={key: float(value) for key, value in state_errors(native[handover], measured).items()},
                               motor_error_max_N=float(np.max(np.abs(motor_history[handover_tick] - motors[handover]))))
    else:
        handover_result["isaac_state"] = None
    if terminal_reset is not None:
        handover_result = None

    def interval(event):
        return None if event is None else [event["interval"]["sample_time_s"], event["interval"]["interval_end_s"]]

    return dict(trace=str(path.relative_to(ROOT)), trace_sha256=digest(path), native_outcome_as_recorded=native_outcome,
                command_interval_s=profile.policy_dt_s, native_recorded_end_s=float(times[-1]),
                isaac_end_s=rows[-1]["interval_end_s"], steps=len(rows),
                native_recorded_source_end_s=float(times[-1] + source_offset),
                isaac_source_end_s=rows[-1]["interval_end_s"] + source_offset,
                terminal_reset=terminal_reset,
                final_bracket_overrun_s=max(0., rows[-1]["interval_end_s"] - float(times[-1])),
                initial_errors={key: float(value) for key, value in state_errors(native[0], live[0]).items()},
                initial_motor_thrust_N=motors[0].tolist(), handover=handover_result, path_errors=path_errors,
                matched_pre_touch_times_s=times[matched_frames].tolist(),
                isaac_pre_touch_positions_m=live[matched_ticks, :3].tolist(),
                first_report_interval_s=interval(result["first_report"]),
                first_reported_positive_load_interval_s=interval(result["first_positive_load"]),
                first_signed_touch=touch, saturated_physics_steps=sum(row["mixer_saturated"] for row in rows),
                isaac_contact_qualification="NOT_SCORED", finite=True)


def run(args):
    profile = load_profile()
    spec = load_contact_spec()
    tapes = [load_tape(path, profile) for path in args.traces]
    resets = [None] * len(tapes)
    if args.terminal_lookback_ms is not None:
        tapes, resets = zip(*(terminal_tape(path, tape, args.terminal_lookback_ms)
                             for path, tape in zip(args.traces, tapes)))
    baseline = read_json(EVIDENCE / "isaac-proxy-field.json")
    manifest_hash = digest(PROXY_MANIFEST)
    asset = EVIDENCE / f"proxy-{manifest_hash[:12]}.usda"
    if (baseline["geometry"] != "proxy" or baseline["proxy"]["manifest_sha256"] != manifest_hash
            or baseline["contact_config_sha256"] != spec.contract_sha256
            or baseline["asset_sha256"] != digest(asset)):
        raise ValueError("Expected the verified Phase 2 proxy USD and Field contract")
    entries = load_proxy()["entries"]
    sim = ContactComparison(1, "candidate", field=True, asset_override=(asset, entries), export_recipe=False)
    if sim.dt != spec.config["comparison_dt_s"] or sim.field_registered != spec.legal_targets:
        raise ValueError("The live PhysX clock or Field registration differs from Phase 2")
    results = [replay(sim, path, tape, profile, spec, reset)
               for path, tape, reset in zip(args.traces, tapes, resets)]
    scope = ("Terminal snapshot reset, existing CTBR command suffix, independent Isaac integration"
             if args.terminal_lookback_ms is not None
             else "Actual command tape, same initial state and motors, independent Isaac integration")
    write_result(args.output, dict(
        backend="Isaac Lab 2.1 / Isaac Sim 4.5 / GPU PhysX PGS", physics_dt_s=sim.dt,
        scope=scope,
        proxy_manifest_sha256=manifest_hash, proxy_usd_sha256=sim.asset_sha256, shapes=sim.view.max_shapes,
        contact_config_sha256=spec.contract_sha256, profile_sha256=profile.profile_sha256,
        qualification_authority="Independent native detailed model", training_transitions=0, traces=results))
    for row in results:
        touch = row["first_signed_touch"]
        print("ASM2_HANDOVER_ISAAC", Path(row["trace"]).name,
              touch["touching_groups"] if touch else "NO_SIGNED_TOUCH", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--terminal-lookback-ms", type=int,
                        help="Reset at an exact recorded control sample at least this far before native signed touch")
    args = parser.parse_args()
    args.traces = [path.resolve() for path in args.traces]
    if len(args.traces) > 2 or len(set(args.traces)) != len(args.traces):
        parser.error("Provide one or two distinct native traces")
    if args.terminal_lookback_ms is not None and args.terminal_lookback_ms <= 0:
        parser.error("Terminal lookback must be positive")
    if any(not path.is_relative_to(ROOT) or not path.is_file() for path in args.traces):
        parser.error("Choose recorded NPZ traces inside this workspace")
    if args.output.exists():
        parser.error("Choose a new evidence path")
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True)
    code = 0
    try:
        run(args)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # Same bounded contact diagnostic exit as Phase 2.


if __name__ == "__main__":
    main()
