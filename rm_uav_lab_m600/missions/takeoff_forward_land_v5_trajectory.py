#!/usr/bin/env python3

import math
import time

from pymavlink import mavutil


def wait_prearm_ok(master, timeout=60):
    print("Waiting for ArduPilot pre-arm checks...")

    prearm_bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["SYS_STATUS", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            if is_from_target(
                msg,
                master.target_system,
                master.target_component,
            ):
                print(f"AP: {msg.text}")
            continue

        if not is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            continue

        if msg.onboard_control_sensors_health & prearm_bit:
            print("PreArm checks PASS")
            return

    raise TimeoutError("PreArm checks did not pass within timeout")

# ============================================================
# Configuration
# ============================================================

CONNECTION = "tcp:127.0.0.1:5760"

TAKEOFF_ALT = 1.5

TRAJECTORY_BODY_WAYPOINTS = [
    ("WP1_FORWARD_3M", 3.0, 0.0, 1.0),
    ("WP2_RIGHT_2M",   3.0, 2.0, 1.0),
    ("WP3_FORWARD_2M", 5.0, 2.0, 3.0),
]

TRAJECTORY_MAX_SPEED = 1.0
WAYPOINT_STOP_ERROR = 0.05
WAYPOINT_SETTLED_SPEED = 0.20
WAYPOINT_TIMEOUT = 25
WAYPOINT_ACCEPTANCE_ERROR = 0.10
V5_MAX_SPEED = 1.20
V5_MAX_ALTITUDE_ERROR = 0.10

# V5 return-to-start settings
RETURN_MAX_SPEED = 0.80
RETURN_STOP_ERROR = 0.08
RETURN_SETTLED_SPEED = 0.20
RETURN_HOLD_DURATION = 2.0
RETURN_TIMEOUT = 30
RETURN_ACCEPTANCE_ERROR = 0.10


# SET_POSITION_TARGET_LOCAL_NED velocity-only mask:
# ignore position xyz, use velocity xyz, ignore acceleration, yaw, yaw-rate.
VELOCITY_ONLY_TYPE_MASK = 0x0DC7

CONNECT_TIMEOUT = 20
READY_TIMEOUT = 30
PREARM_TIMEOUT = 60
MODE_TIMEOUT = 8
COMMAND_ACK_TIMEOUT = 8
ARM_TIMEOUT = 10
TAKEOFF_TIMEOUT = 20
FORWARD_TIMEOUT = 30
LAND_TIMEOUT = 40

# MAVLink ESTIMATOR_STATUS_FLAGS:
# 1  = attitude estimate good
# 2  = horizontal velocity estimate good
# 8  = relative horizontal position estimate good
# 32 = absolute vertical position estimate good
REQUIRED_EKF_FLAGS = 1 | 2 | 8 | 32


# ============================================================
# Small pure helpers
# ============================================================

def is_from_target(msg, target_system, target_component):
    return (
        msg.get_srcSystem() == target_system
        and msg.get_srcComponent() == target_component
    )


def ekf_flags_ready(flags):
    return (flags & REQUIRED_EKF_FLAGS) == REQUIRED_EKF_FLAGS


def body_offset_to_local_ned(forward_m, right_m, yaw_rad):
    """Convert initial-body-frame forward/right offsets to LOCAL_NED XY."""
    north = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
    east = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
    return north, east


def project_body_displacement(dx, dy, yaw_rad):
    """Project LOCAL_NED displacement into initial forward/right axes."""
    forward = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    right = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    return forward, right


def velocity_for_xy_error(error_x, error_y):
    """Return a LOCAL_NED velocity vector pointing toward the waypoint."""
    distance = math.hypot(error_x, error_y)

    if distance <= WAYPOINT_STOP_ERROR:
        return 0.0, 0.0

    if distance > 1.0:
        speed = TRAJECTORY_MAX_SPEED
    elif distance > 0.4:
        speed = 0.60
    elif distance > 0.15:
        speed = 0.30
    else:
        speed = 0.15

    scale = speed / distance
    return error_x * scale, error_y * scale


def waypoint_is_settled(distance, speed):
    return (
        distance <= WAYPOINT_STOP_ERROR
        and speed <= WAYPOINT_SETTLED_SPEED
    )


def return_velocity_for_error(error_x, error_y):
    """Return LOCAL_NED XY velocity pointing toward the saved mission origin."""
    distance = math.hypot(error_x, error_y)

    if distance <= RETURN_STOP_ERROR:
        return 0.0, 0.0
    if distance > 0.80:
        speed = RETURN_MAX_SPEED
    elif distance > 0.25:
        speed = 0.40
    else:
        speed = 0.20

    scale = speed / distance
    return error_x * scale, error_y * scale


def return_is_settled(distance, speed):
    return (
        distance <= RETURN_STOP_ERROR
        and speed <= RETURN_SETTLED_SPEED
    )


def evaluate_v5_pass(waypoint_metrics, return_metrics):
    waypoints_ok = all(
        item["final_error"] <= WAYPOINT_ACCEPTANCE_ERROR
        and item["max_speed"] <= V5_MAX_SPEED
        and item["max_altitude_error"] <= V5_MAX_ALTITUDE_ERROR
        for item in waypoint_metrics
    )
    return_ok = return_metrics["return_error"] <= RETURN_ACCEPTANCE_ERROR
    return waypoints_ok and return_ok


# ============================================================
# MAVLink connection / target selection
# ============================================================

def connect_vehicle():
    print("[1] CONNECTING")

    master = mavutil.mavlink_connection(
        CONNECTION,
        source_system=255,
        source_component=190,
    )

    deadline = time.monotonic() + CONNECT_TIMEOUT

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            print("Waiting for heartbeat...")
            continue

        # We want the actual ArduPilot autopilot heartbeat,
        # not a GCS / gimbal / camera component heartbeat.
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            print(
                "Ignoring non-ArduPilot heartbeat "
                f"from {msg.get_srcSystem()}:{msg.get_srcComponent()}"
            )
            continue

        master.target_system = msg.get_srcSystem()
        master.target_component = msg.get_srcComponent()

        print(
            "Connected to ArduPilot: "
            f"system={master.target_system}, "
            f"component={master.target_component}"
        )

        if master.target_component == 0:
            raise RuntimeError(
                "Autopilot heartbeat resolved to component 0; "
                "expected the actual flight-controller component."
            )

        return master

    raise TimeoutError("No ArduPilot heartbeat received")


# ============================================================
# Message-rate helpers
# ============================================================

def request_message_interval(master, message_id, frequency_hz):
    interval_us = int(1_000_000 / frequency_hz)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0,
    )


def configure_message_rates(master):
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        2,
    )

    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        10,
    )

    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        5,
    )

    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        2,
    )

    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        10,
    )


# ============================================================
# Vehicle readiness
# ============================================================

def wait_vehicle_ready(master):
    print("\n[2] WAIT READY")
    print("Waiting for EKF + LOCAL_POSITION_NED...")

    configure_message_rates(master)

    ekf_ok = False
    local_position_ok = False

    deadline = time.monotonic() + READY_TIMEOUT

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=[
                "EKF_STATUS_REPORT",
                "LOCAL_POSITION_NED",
                "STATUSTEXT",
            ],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            if is_from_target(
                msg,
                master.target_system,
                master.target_component,
            ):
                print(f"AP: {msg.text}")
            continue

        if not is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            continue

        if msg.get_type() == "EKF_STATUS_REPORT":
            flags = int(msg.flags)

            print(
                f"EKF flags={flags} "
                f"required={REQUIRED_EKF_FLAGS}"
            )

            ekf_ok = ekf_flags_ready(flags)

        elif msg.get_type() == "LOCAL_POSITION_NED":
            if not local_position_ok:
                print(
                    "LOCAL_POSITION_NED received: "
                    f"x={msg.x:.2f}, "
                    f"y={msg.y:.2f}, "
                    f"z={msg.z:.2f}"
                )
            local_position_ok = True

        if ekf_ok and local_position_ok:
            print("Vehicle navigation state READY")
            return

    raise TimeoutError(
        "Vehicle did not become ready "
        f"(ekf_ok={ekf_ok}, "
        f"local_position_ok={local_position_ok})"
    )


# ============================================================
# Mode handling
# ============================================================

def wait_mode(master, mode_name):
    mode_map = master.mode_mapping()

    if mode_name not in mode_map:
        raise RuntimeError(f"Unknown mode: {mode_name}")

    mode_id = mode_map[mode_name]
    deadline = time.monotonic() + MODE_TIMEOUT

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if not is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            continue

        if msg.custom_mode == mode_id:
            print(f"Mode confirmed: {mode_name}")
            return

    raise TimeoutError(f"Failed to enter {mode_name}")


def set_mode(master, mode_name):
    mode_map = master.mode_mapping()

    if mode_name not in mode_map:
        raise RuntimeError(f"Unknown mode: {mode_name}")

    mode_id = mode_map[mode_name]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    wait_mode(master, mode_name)


# ============================================================
# COMMAND_ACK handling
# ============================================================

def collect_statustext(master, duration=2.0):
    deadline = time.monotonic() + duration

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="STATUSTEXT",
            blocking=True,
            timeout=0.5,
        )

        if msg is None:
            continue

        if is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            print(f"AP: {msg.text}")


def wait_command_ack(master, command):
    deadline = time.monotonic() + COMMAND_ACK_TIMEOUT

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            if is_from_target(
                msg,
                master.target_system,
                master.target_component,
            ):
                print(f"AP: {msg.text}")
            continue

        # Critical diagnostic:
        # only trust ACKs from the actual ArduPilot FC component.
        if not is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            print(
                "Ignoring ACK from other component: "
                f"command={msg.command}, "
                f"result={msg.result}, "
                f"from={msg.get_srcSystem()}:"
                f"{msg.get_srcComponent()}"
            )
            continue

        if msg.command != command:
            continue

        print(
            f"ACK command={msg.command}, "
            f"result={msg.result}, "
            f"from={msg.get_srcSystem()}:"
            f"{msg.get_srcComponent()}"
        )

        if msg.result in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            return

        print("Command rejected; collecting ArduPilot diagnostics...")
        collect_statustext(master)

        raise RuntimeError(
            f"Command {command} rejected, result={msg.result}"
        )

    raise TimeoutError(f"No ACK for command {command}")


# ============================================================
# Arm / disarm
# ============================================================

def wait_armed(master):
    deadline = time.monotonic() + ARM_TIMEOUT

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if not is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            continue

        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("ARM confirmed")
            return

    raise TimeoutError("Vehicle failed to arm")


def wait_disarmed(master):
    deadline = time.monotonic() + LAND_TIMEOUT

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if not is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            continue

        armed = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        if not armed:
            print("DISARM confirmed")
            return

    raise TimeoutError("Vehicle failed to disarm")


def vehicle_is_armed(master):
    return bool(master.motors_armed())


def arm_vehicle(master):
    print("\n[4] WAIT PREARM")
    wait_prearm_ok(master, timeout=PREARM_TIMEOUT)

    print("\n[5] ARM")
    print(
        "ARM target: "
        f"system={master.target_system}, "
        f"component={master.target_component}"
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,      # param1: arm
        0,      # param2: normal arm, NOT force-arm
        0, 0, 0, 0, 0,
    )

    wait_command_ack(
        master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    )

    wait_armed(master)


# ============================================================
# Position helpers
# ============================================================

def get_local_position(master, timeout=5):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="LOCAL_POSITION_NED",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            return msg

    raise TimeoutError("No LOCAL_POSITION_NED from flight controller")


def get_vehicle_yaw(master, timeout=5):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="ATTITUDE",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if is_from_target(
            msg,
            master.target_system,
            master.target_component,
        ):
            return msg.yaw

    raise TimeoutError("No ATTITUDE message from flight controller")


# ============================================================
# Takeoff
# ============================================================

def takeoff(master):
    ground = get_local_position(master)
    ground_z = ground.z

    print(f"\n[6] TAKEOFF -> {TAKEOFF_ALT:.1f} m")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        TAKEOFF_ALT,
    )

    wait_command_ack(
        master,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    )

    deadline = time.monotonic() + TAKEOFF_TIMEOUT

    while time.monotonic() < deadline:
        pos = get_local_position(master, timeout=2)

        # NED: +Z points downward.
        altitude_gain = ground_z - pos.z

        print(
            f"Altitude gain={altitude_gain:.2f} m   "
            f"vz={pos.vz:.2f} m/s"
        )

        if (
            altitude_gain >= TAKEOFF_ALT - 0.15
            and abs(pos.vz) < 0.35
        ):
            print("Takeoff complete")
            return altitude_gain

    raise TimeoutError("Takeoff altitude timeout")


# ============================================================
# V5 multi-waypoint trajectory
# ============================================================

def fly_to_local_waypoint(
    master,
    label,
    target_x,
    target_y,
    hold_duration,
    altitude_reference_z,
):
    print(f"\n    {label}")
    print(
        f"    LOCAL_NED target: x={target_x:.2f}, y={target_y:.2f} | "
        f"hold={hold_duration:.1f} s"
    )

    deadline = time.monotonic() + WAYPOINT_TIMEOUT
    hold_started = None
    max_speed = 0.0
    max_altitude_error = 0.0
    last_print = 0.0
    final_error = float("inf")
    final_x = float("nan")
    final_y = float("nan")

    while time.monotonic() < deadline:
        pos = get_local_position(master, timeout=2)

        error_x = target_x - pos.x
        error_y = target_y - pos.y
        distance = math.hypot(error_x, error_y)
        speed = math.hypot(pos.vx, pos.vy)
        altitude_error = abs(pos.z - altitude_reference_z)

        max_speed = max(max_speed, speed)
        max_altitude_error = max(max_altitude_error, altitude_error)
        final_error = distance
        final_x = pos.x
        final_y = pos.y

        cmd_vx, cmd_vy = velocity_for_xy_error(error_x, error_y)

        if waypoint_is_settled(distance, speed):
            cmd_vx = 0.0
            cmd_vy = 0.0
            if hold_started is None:
                hold_started = time.monotonic()
                print(
                    f"    Waypoint window reached; starting "
                    f"{hold_duration:.1f} s hold"
                )
        else:
            hold_started = None

        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            VELOCITY_ONLY_TYPE_MASK,
            0, 0, 0,
            cmd_vx, cmd_vy, 0,
            0, 0, 0,
            0, 0,
        )

        now = time.monotonic()
        if now - last_print >= 0.25:
            print(
                f"    error={distance:.2f} m   "
                f"cmd=({cmd_vx:+.2f},{cmd_vy:+.2f}) m/s   "
                f"speed={speed:.2f} m/s   "
                f"alt_err={altitude_error:.2f} m"
            )
            last_print = now

        if hold_started is not None:
            hold_elapsed = now - hold_started
            if hold_elapsed >= hold_duration:
                print(f"    {label} held for {hold_duration:.1f} s")
                return {
                    "label": label,
                    "target_x": target_x,
                    "target_y": target_y,
                    "final_x": final_x,
                    "final_y": final_y,
                    "final_error": final_error,
                    "max_speed": max_speed,
                    "max_altitude_error": max_altitude_error,
                    "hold_duration": hold_elapsed,
                }

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_ONLY_TYPE_MASK,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )

    raise TimeoutError(
        f"{label} timeout (final XY error={final_error:.2f} m)"
    )


def fly_trajectory(master, origin_x, origin_y, initial_yaw):
    print("\n[7] V5 MULTI-WAYPOINT TRAJECTORY")
    print(
        f"Mission origin: x={origin_x:.2f}, y={origin_y:.2f}, "
        f"initial yaw={math.degrees(initial_yaw):.1f} deg"
    )
    print(f"Trajectory max speed: {TRAJECTORY_MAX_SPEED:.2f} m/s")

    altitude_reference = get_local_position(master).z
    results = []

    for index, (label, forward_m, right_m, hold_s) in enumerate(
        TRAJECTORY_BODY_WAYPOINTS,
        start=1,
    ):
        offset_x, offset_y = body_offset_to_local_ned(
            forward_m,
            right_m,
            initial_yaw,
        )
        target_x = origin_x + offset_x
        target_y = origin_y + offset_y

        print(
            f"\n[7.{index}] {label}: "
            f"body(forward={forward_m:.2f}, right={right_m:.2f})"
        )

        result = fly_to_local_waypoint(
            master,
            label,
            target_x,
            target_y,
            hold_s,
            altitude_reference,
        )

        dx = result["final_x"] - origin_x
        dy = result["final_y"] - origin_y
        achieved_forward, achieved_right = project_body_displacement(
            dx,
            dy,
            initial_yaw,
        )
        result["target_forward"] = forward_m
        result["target_right"] = right_m
        result["achieved_forward"] = achieved_forward
        result["achieved_right"] = achieved_right
        results.append(result)

    print("\nAll V5 waypoints completed")
    return results


# ============================================================
# V5 return to saved takeoff XY before landing
# ============================================================

def return_to_start(master, origin_x, origin_y):
    print("\n[8] RETURN TO START")
    print(
        f"Saved origin: x={origin_x:.2f}, y={origin_y:.2f} | "
        f"max speed={RETURN_MAX_SPEED:.2f} m/s"
    )

    deadline = time.monotonic() + RETURN_TIMEOUT
    hold_started = None
    max_speed = 0.0
    last_print = 0.0
    final_error = float("inf")

    while time.monotonic() < deadline:
        pos = get_local_position(master, timeout=2)
        error_x = origin_x - pos.x
        error_y = origin_y - pos.y
        distance = math.hypot(error_x, error_y)
        speed = math.hypot(pos.vx, pos.vy)
        max_speed = max(max_speed, speed)
        final_error = distance

        cmd_vx, cmd_vy = return_velocity_for_error(error_x, error_y)

        if return_is_settled(distance, speed):
            cmd_vx = 0.0
            cmd_vy = 0.0
            if hold_started is None:
                hold_started = time.monotonic()
                print(
                    f"Return window reached; starting "
                    f"{RETURN_HOLD_DURATION:.1f} s hold"
                )
        else:
            hold_started = None

        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            VELOCITY_ONLY_TYPE_MASK,
            0, 0, 0,
            cmd_vx, cmd_vy, 0,
            0, 0, 0,
            0, 0,
        )

        now = time.monotonic()
        if now - last_print >= 0.25:
            print(
                f"Return error={distance:.2f} m   "
                f"cmd=({cmd_vx:+.2f},{cmd_vy:+.2f}) m/s   "
                f"speed={speed:.2f} m/s"
            )
            last_print = now

        if hold_started is not None:
            hold_elapsed = now - hold_started
            if hold_elapsed >= RETURN_HOLD_DURATION:
                print(
                    f"Return target held for "
                    f"{RETURN_HOLD_DURATION:.1f} s"
                )
                return {
                    "return_error": final_error,
                    "return_max_speed": max_speed,
                    "return_hold_duration": hold_elapsed,
                }

    # Stop horizontal motion before handing control to the exception/LAND path.
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_ONLY_TYPE_MASK,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )

    raise TimeoutError(
        f"Return-to-start timeout (final XY error={final_error:.2f} m)"
    )


# ============================================================
# Landing
# ============================================================

def land(master):
    print("\n[9] LAND")
    set_mode(master, "LAND")
    wait_disarmed(master)


# ============================================================
# Main mission
# ============================================================

def main():
    master = None

    try:
        master = connect_vehicle()
        wait_vehicle_ready(master)

        print("\n[3] GUIDED")
        set_mode(master, "GUIDED")

        arm_vehicle(master)

        mission_origin = get_local_position(master)
        print(
            "Saved mission origin: "
            f"x={mission_origin.x:.2f}, y={mission_origin.y:.2f}"
        )

        takeoff_achieved = takeoff(master)

        initial_yaw = get_vehicle_yaw(master)
        print(
            f"Saved initial trajectory yaw: "
            f"{math.degrees(initial_yaw):.1f} deg"
        )

        waypoint_metrics = fly_trajectory(
            master,
            mission_origin.x,
            mission_origin.y,
            initial_yaw,
        )

        return_metrics = return_to_start(
            master,
            mission_origin.x,
            mission_origin.y,
        )

        land(master)

        passed = evaluate_v5_pass(
            waypoint_metrics,
            return_metrics,
        )

        print("\n========== V5 TRAJECTORY REPORT ==========")
        print(f"Takeoff target       : {TAKEOFF_ALT:.2f} m")
        print(f"Takeoff achieved     : {takeoff_achieved:.2f} m")

        for item in waypoint_metrics:
            print()
            print(f"{item['label']}")
            print(
                f"  Body target        : "
                f"forward={item['target_forward']:.2f}, "
                f"right={item['target_right']:.2f} m"
            )
            print(
                f"  Body achieved      : "
                f"forward={item['achieved_forward']:.2f}, "
                f"right={item['achieved_right']:.2f} m"
            )
            print(f"  Final XY error     : {item['final_error']:.2f} m")
            print(f"  Maximum speed      : {item['max_speed']:.2f} m/s")
            print(
                f"  Maximum altitude err: "
                f"{item['max_altitude_error']:.2f} m"
            )
            print(f"  Hold duration      : {item['hold_duration']:.2f} s")
            print(
                f"  Result             : "
                f"{'PASS' if item['final_error'] <= WAYPOINT_ACCEPTANCE_ERROR else 'FAIL'}"
            )

        overall_max_speed = max(
            item["max_speed"] for item in waypoint_metrics
        )
        overall_max_alt_error = max(
            item["max_altitude_error"] for item in waypoint_metrics
        )

        print()
        print(f"Overall max speed    : {overall_max_speed:.2f} m/s")
        print(f"Overall max alt err  : {overall_max_alt_error:.2f} m")
        print(f"Return XY error      : {return_metrics['return_error']:.2f} m")
        print(f"Return max speed     : {return_metrics['return_max_speed']:.2f} m/s")
        print(f"Return hold duration : {return_metrics['return_hold_duration']:.2f} s")
        print(f"MISSION RESULT       : {'PASS' if passed else 'FAIL'}")
        print("==========================================")

        print("\n======================")
        print("MISSION COMPLETE")
        print("======================")

    except KeyboardInterrupt:
        print("\nMISSION INTERRUPTED BY USER")
        if master is not None and vehicle_is_armed(master):
            print("Vehicle is armed; attempting LAND...")
            try:
                set_mode(master, "LAND")
            except Exception as land_error:
                print(f"WARNING: LAND failed: {land_error}")
        raise

    except Exception as error:
        print("\nMISSION ERROR:")
        print(error)
        if master is not None and vehicle_is_armed(master):
            print("Vehicle is armed; attempting LAND...")
            try:
                set_mode(master, "LAND")
            except Exception as land_error:
                print(f"WARNING: Unable to switch to LAND: {land_error}")
        else:
            print("Vehicle is not armed; LAND not required.")
        raise

    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()