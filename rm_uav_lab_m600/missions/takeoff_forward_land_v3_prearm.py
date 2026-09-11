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

CONNECT_TIMEOUT = 20
READY_TIMEOUT = 30
PREARM_TIMEOUT = 60
MODE_TIMEOUT = 8
COMMAND_ACK_TIMEOUT = 8
ARM_TIMEOUT = 10
TAKEOFF_TIMEOUT = 20
FORWARD_TIMEOUT = 25
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
            return

    raise TimeoutError("Takeoff altitude timeout")


# ============================================================
# Forward BODY_OFFSET_NED motion
# ============================================================

def fly_forward(master):
    start = get_local_position(master)

    start_x = start.x
    start_y = start.y

    print(
        f"\n[7] FORWARD -> {FORWARD_DISTANCE:.1f} m"
    )
    print(
        "Start position: "
        f"x={start_x:.2f}, "
        f"y={start_y:.2f}"
    )

    # Position-only setpoint:
    # x/y/z are enabled; velocity, acceleration, yaw and yaw-rate ignored.
    type_mask = 0x0DF8

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask,

        # BODY_OFFSET_NED:
        # +X = forward from current body heading
        # Y=0 = no lateral offset
        # Z=0 = maintain current altitude
        FORWARD_DISTANCE,
        0,
        0,

        0, 0, 0,      # velocity
        0, 0, 0,      # acceleration
        0, 0,          # yaw, yaw-rate
    )

    deadline = time.monotonic() + FORWARD_TIMEOUT

    while time.monotonic() < deadline:
        pos = get_local_position(master, timeout=2)

        dx = pos.x - start_x
        dy = pos.y - start_y

        distance = math.hypot(dx, dy)
        speed = math.hypot(pos.vx, pos.vy)

        print(
            f"Distance={distance:.2f} m   "
            f"speed={speed:.2f} m/s"
        )

        if (
            distance >= FORWARD_DISTANCE - 0.30
            and speed < 0.30
        ):
            print("Forward target reached")
            return

    raise TimeoutError("Forward movement timeout")


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

        takeoff(master)

        fly_forward(master)

        land(master)

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