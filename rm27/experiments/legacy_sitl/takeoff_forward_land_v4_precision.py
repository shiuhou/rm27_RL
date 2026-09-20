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
FORWARD_DISTANCE = 5.0

# V4 precision-control settings
FORWARD_MAX_SPEED = 1.0
FORWARD_STOP_ERROR = 0.05
FORWARD_HOLD_DURATION = 3.0
FORWARD_SETTLED_SPEED = 0.20

# V4 acceptance thresholds
V4_MAX_FORWARD_ERROR = 0.10
V4_MAX_SPEED = 1.20
V4_MAX_ALTITUDE_ERROR = 0.10
V4_MAX_HOLD_ERROR = 0.10

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


def forward_speed_for_error(error_m):
    """Return signed body-forward velocity from signed target error."""
    magnitude = abs(error_m)

    if magnitude <= FORWARD_STOP_ERROR:
        speed = 0.0
    elif magnitude > 1.0:
        speed = FORWARD_MAX_SPEED
    elif magnitude > 0.4:
        speed = 0.6
    elif magnitude > 0.15:
        speed = 0.3
    else:
        speed = 0.15

    return math.copysign(speed, error_m) if speed else 0.0


def project_body_displacement(dx, dy, yaw_rad):
    """Project NED XY displacement into the initial body-forward/lateral axes."""
    forward = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    lateral = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    return forward, lateral


def evaluate_v4_pass(metrics):
    return (
        abs(metrics["forward_error"]) <= V4_MAX_FORWARD_ERROR
        and metrics["max_speed"] <= V4_MAX_SPEED
        and metrics["max_altitude_error"] <= V4_MAX_ALTITUDE_ERROR
        and metrics["final_hold_error"] <= V4_MAX_HOLD_ERROR
    )


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
# V4 precision forward motion: closed-loop body velocity
# ============================================================

def fly_forward(master):
    start = get_local_position(master)
    start_x = start.x
    start_y = start.y
    start_z = start.z

    # Capture the initial heading so LOCAL_POSITION_NED displacement can
    # be projected onto the original body-forward axis.
    yaw_deadline = time.monotonic() + 5
    start_yaw = None

    while time.monotonic() < yaw_deadline:
        attitude = master.recv_match(
            type="ATTITUDE",
            blocking=True,
            timeout=1,
        )

        if attitude is None:
            continue

        if not is_from_target(
            attitude,
            master.target_system,
            master.target_component,
        ):
            continue

        start_yaw = attitude.yaw
        break

    if start_yaw is None:
        raise TimeoutError("No ATTITUDE message for forward-axis reference")

    print(f"\n[7] PRECISION FORWARD -> {FORWARD_DISTANCE:.1f} m")
    print(
        "Start position: "
        f"x={start_x:.2f}, y={start_y:.2f}, "
        f"yaw={math.degrees(start_yaw):.1f} deg"
    )
    print(f"Commanded max speed: {FORWARD_MAX_SPEED:.2f} m/s")

    deadline = time.monotonic() + FORWARD_TIMEOUT
    hold_started = None
    max_speed = 0.0
    max_altitude_error = 0.0
    max_lateral_error = 0.0
    last_print = 0.0
    final_progress = 0.0
    final_lateral = 0.0

    while time.monotonic() < deadline:
        pos = get_local_position(master, timeout=2)

        dx = pos.x - start_x
        dy = pos.y - start_y
        progress, lateral = project_body_displacement(dx, dy, start_yaw)
        error = FORWARD_DISTANCE - progress
        speed = math.hypot(pos.vx, pos.vy)
        altitude_error = abs(pos.z - start_z)

        max_speed = max(max_speed, speed)
        max_altitude_error = max(max_altitude_error, altitude_error)
        max_lateral_error = max(max_lateral_error, abs(lateral))
        final_progress = progress
        final_lateral = lateral

        command_vx = forward_speed_for_error(error)

        # Enter the hold timer only once position and speed are both settled.
        if abs(error) <= FORWARD_STOP_ERROR and speed <= FORWARD_SETTLED_SPEED:
            command_vx = 0.0
            if hold_started is None:
                hold_started = time.monotonic()
                print("Target window reached; starting 3 s hold")
        else:
            hold_started = None

        # Velocity-only BODY_NED command.  Re-sent continuously so Guided
        # velocity control never times out.  vz=0 requests altitude hold.
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            VELOCITY_ONLY_TYPE_MASK,
            0, 0, 0,
            command_vx, 0, 0,
            0, 0, 0,
            0, 0,
        )

        now = time.monotonic()
        if now - last_print >= 0.25:
            print(
                f"Forward={progress:.2f} m   "
                f"error={error:+.2f} m   "
                f"cmd_vx={command_vx:+.2f} m/s   "
                f"speed={speed:.2f} m/s   "
                f"lateral={lateral:+.2f} m   "
                f"alt_err={altitude_error:.2f} m"
            )
            last_print = now

        if hold_started is not None:
            hold_elapsed = now - hold_started
            if hold_elapsed >= FORWARD_HOLD_DURATION:
                forward_error = final_progress - FORWARD_DISTANCE
                final_hold_error = math.hypot(forward_error, final_lateral)

                print("Forward target held for 3.0 s")
                return {
                    "forward_achieved": final_progress,
                    "forward_error": forward_error,
                    "lateral_error": final_lateral,
                    "final_hold_error": final_hold_error,
                    "max_speed": max_speed,
                    "max_altitude_error": max_altitude_error,
                    "max_lateral_error": max_lateral_error,
                    "hold_duration": hold_elapsed,
                }

    # Send one final zero-velocity command before raising.
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        VELOCITY_ONLY_TYPE_MASK,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )

    raise TimeoutError(
        "Precision forward movement timeout "
        f"(progress={final_progress:.2f} m, lateral={final_lateral:.2f} m)"
    )


# ============================================================
# Landing
# ============================================================

def land(master):
    print("\n[8] LAND")
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

        takeoff_achieved = takeoff(master)

        metrics = fly_forward(master)

        land(master)

        metrics["takeoff_achieved"] = takeoff_achieved
        passed = evaluate_v4_pass(metrics)

        print("\n========== V4 FLIGHT REPORT ==========")
        print(f"Takeoff target       : {TAKEOFF_ALT:.2f} m")
        print(f"Takeoff achieved     : {takeoff_achieved:.2f} m")
        print(f"Forward target       : {FORWARD_DISTANCE:.2f} m")
        print(f"Forward achieved     : {metrics['forward_achieved']:.2f} m")
        print(f"Forward error        : {metrics['forward_error']:+.2f} m")
        print(f"Lateral error        : {metrics['lateral_error']:+.2f} m")
        print(f"Final 2D hold error  : {metrics['final_hold_error']:.2f} m")
        print(f"Maximum speed        : {metrics['max_speed']:.2f} m/s")
        print(f"Maximum altitude err : {metrics['max_altitude_error']:.2f} m")
        print(f"Maximum lateral err  : {metrics['max_lateral_error']:.2f} m")
        print(f"Final hold duration  : {metrics['hold_duration']:.2f} s")
        print(f"MISSION RESULT       : {'PASS' if passed else 'FAIL'}")
        print("======================================")

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