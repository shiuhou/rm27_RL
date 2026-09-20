from pymavlink import mavutil
import math
import time


TAKEOFF_ALT = 1.5
FORWARD_DISTANCE = 5.0


print("Connecting to ArduPilot...")

master = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5760"
)

master.wait_heartbeat()

print(
    f"Connected: system={master.target_system}, "
    f"component={master.target_component}"
)


# --------------------------------------------------
# 1. GUIDED mode
# --------------------------------------------------

mode_id = master.mode_mapping()["GUIDED"]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id,
)

print("Switching to GUIDED...")
time.sleep(2)


# --------------------------------------------------
# 2. ARM
# --------------------------------------------------

print("Arming...")

master.arducopter_arm()
master.motors_armed_wait()

print("ARMED")


# --------------------------------------------------
# 3. TAKEOFF
# --------------------------------------------------

print(f"Taking off to {TAKEOFF_ALT} m")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0,
    TAKEOFF_ALT,
)


while True:
    msg = master.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=2,
    )

    if msg is None:
        continue

    alt = msg.relative_alt / 1000.0

    print(f"Altitude: {alt:.2f} m")

    if alt >= TAKEOFF_ALT - 0.15:
        break


print("Takeoff complete")


# --------------------------------------------------
# 4. Get current local position
# --------------------------------------------------

msg = master.recv_match(
    type="LOCAL_POSITION_NED",
    blocking=True,
)

start_x = msg.x
start_y = msg.y

print(
    f"Start local position: "
    f"x={start_x:.2f}, y={start_y:.2f}"
)


# --------------------------------------------------
# 5. Move 5 m FORWARD relative to the vehicle
# --------------------------------------------------

print(f"Moving forward {FORWARD_DISTANCE} m")

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,

    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,

    0x0DF8,      # use position x/y/z only

    FORWARD_DISTANCE,
    0,
    0,

    0, 0, 0,
    0, 0, 0,
    0, 0,
)


# --------------------------------------------------
# 6. Monitor travelled distance
# --------------------------------------------------

while True:
    msg = master.recv_match(
        type="LOCAL_POSITION_NED",
        blocking=True,
        timeout=2,
    )

    if msg is None:
        continue

    dx = msg.x - start_x
    dy = msg.y - start_y

    distance = math.sqrt(dx * dx + dy * dy)

    print(f"Travelled: {distance:.2f} m")

    if distance >= 4.7:
        break


print("Forward movement complete")


# --------------------------------------------------
# 7. LAND
# --------------------------------------------------

print("Landing...")

land_id = master.mode_mapping()["LAND"]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    land_id,
)

master.motors_disarmed_wait()

print("DISARMED")
print("MISSION COMPLETE")
