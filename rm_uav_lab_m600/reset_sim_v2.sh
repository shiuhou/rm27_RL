#!/usr/bin/env bash
set -euo pipefail

# RM UAV clean benchmark reset v2
# Gazebo stays open. We reset model state only, restart SITL, then
# require a real ArduPilot MAVLink HEARTBEAT before success.

LAB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="${LAB_DIR}/ardupilot"
PYTHON="${PYTHON:-${HOME}/venvs/ardupilot/bin/python}"
LOG_DIR="${LAB_DIR}/logs"
PID_FILE="${LAB_DIR}/.sitl_reset.pid"

PORT=5760
WAIT_SECONDS=60

mkdir -p "${LOG_DIR}"

if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: 'gz' command not found." >&2
    exit 1
fi

if [[ ! -d "${ARDUPILOT_DIR}" ]]; then
    echo "ERROR: ArduPilot directory not found: ${ARDUPILOT_DIR}" >&2
    exit 1
fi

if [[ ! -x "${PYTHON}" ]]; then
    echo "ERROR: venv Python not found: ${PYTHON}" >&2
    exit 1
fi

echo "[1/5] Stopping old ArduPilot SITL..."
pkill -INT -f 'Tools/autotest/sim_vehicle.py.*ArduCopter' 2>/dev/null || true
pkill -INT -f 'build/sitl/bin/arducopter.*--model JSON' 2>/dev/null || true
sleep 2

if ss -ltnp 2>/dev/null | grep -q ":${PORT}[[:space:]]"; then
    echo "TCP ${PORT} still busy; terminating remaining ArduCopter..."
    pkill -TERM -f 'build/sitl/bin/arducopter' 2>/dev/null || true
    sleep 2
fi

if ss -ltnp 2>/dev/null | grep -q ":${PORT}[[:space:]]"; then
    echo "ERROR: TCP ${PORT} is still occupied." >&2
    exit 1
fi

echo "[2/5] Finding Gazebo world-control service..."
CONTROL_SERVICE="$(gz service -l | awk '/^\/world\/.*\/control$/ {print; exit}')"

if [[ -z "${CONTROL_SERVICE}" ]]; then
    echo "ERROR: No Gazebo /world/<name>/control service found." >&2
    echo "Keep Terminal A running: gz sim -v4 -r iris_runway.sdf" >&2
    exit 1
fi

echo "Using: ${CONTROL_SERVICE}"
echo "Resetting MODEL state only; simulation time remains monotonic..."

RESET_REPLY="$(
    gz service \
        -s "${CONTROL_SERVICE}" \
        --reqtype gz.msgs.WorldControl \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req 'reset: {model_only: true}'
)"
echo "${RESET_REPLY}"

if ! grep -q 'data: true' <<<"${RESET_REPLY}"; then
    echo "ERROR: Gazebo model-only reset did not return success." >&2
    exit 1
fi

sleep 1

echo "[3/5] Restarting ArduPilot SITL (--no-mavproxy)..."
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="${LOG_DIR}/sitl-reset-v2-${TIMESTAMP}.log"

(
    cd "${ARDUPILOT_DIR}"
    # This machine's gnome-terminal (snap) is broken; run SITL detached
    # instead of in a terminal window.
    export SITL_RITW_TERMINAL="setsid"
    exec Tools/autotest/sim_vehicle.py \
        -v ArduCopter \
        -f gazebo-iris \
        --model JSON \
        --no-mavproxy
) >"${LOG_FILE}" 2>&1 &

SITL_PID=$!
echo "${SITL_PID}" > "${PID_FILE}"

echo "SITL launcher PID: ${SITL_PID}"
echo "SITL log: ${LOG_FILE}"

echo "[4/5] Waiting for TCP ${PORT}..."
tcp_ready=0
for ((i=1; i<=WAIT_SECONDS; i++)); do
    if ss -ltn 2>/dev/null | grep -q ":${PORT}[[:space:]]"; then
        tcp_ready=1
        echo
        echo "TCP ${PORT} is listening."
        break
    fi

    if ! kill -0 "${SITL_PID}" 2>/dev/null; then
        echo
        echo "ERROR: sim_vehicle.py exited before TCP ${PORT} opened." >&2
        tail -n 50 "${LOG_FILE}" >&2 || true
        exit 1
    fi

    printf '.'
    sleep 1
done

if [[ "${tcp_ready}" -ne 1 ]]; then
    echo
    echo "ERROR: TCP ${PORT} did not open within ${WAIT_SECONDS}s." >&2
    tail -n 50 "${LOG_FILE}" >&2 || true
    exit 1
fi

echo "[5/5] Waiting for real ArduPilot MAVLink HEARTBEAT..."
heartbeat_ready=0
for ((i=1; i<=WAIT_SECONDS; i++)); do
    if "${PYTHON}" - <<'PY'
from pymavlink import mavutil
import sys

connection = None
try:
    connection = mavutil.mavlink_connection(
        "tcp:127.0.0.1:5760",
        source_system=254,
        source_component=190,
    )
    heartbeat = connection.wait_heartbeat(timeout=2)

    if heartbeat is None:
        raise RuntimeError("no heartbeat")

    if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise RuntimeError("heartbeat was not from ArduPilot")

    print(
        "HEARTBEAT OK: "
        f"{heartbeat.get_srcSystem()}:{heartbeat.get_srcComponent()}"
    )
except Exception:
    sys.exit(1)
finally:
    if connection is not None:
        try:
            connection.close()
        except Exception:
            pass
PY
    then
        heartbeat_ready=1
        break
    fi

    if ! kill -0 "${SITL_PID}" 2>/dev/null; then
        echo
        echo "ERROR: sim_vehicle.py exited while waiting for heartbeat." >&2
        tail -n 50 "${LOG_FILE}" >&2 || true
        exit 1
    fi

    printf '.'
    sleep 1
done

if [[ "${heartbeat_ready}" -ne 1 ]]; then
    echo
    echo "ERROR: TCP ${PORT} opened but no ArduPilot heartbeat arrived." >&2
    echo "Gazebo <-> JSON SITL is not actually ready." >&2
    echo "Launcher log: ${LOG_FILE}" >&2
    exit 1
fi

sleep 1

echo
echo "RESET COMPLETE: model reset + fresh SITL + MAVLink heartbeat verified."
echo "Run V5:"
echo "  cd ${LAB_DIR}/missions"
echo "  source ${HOME}/venvs/ardupilot/bin/activate"
echo "  python takeoff_forward_land_v5_trajectory.py"
echo
echo "Launcher log:"
echo "  ${LOG_FILE}"
