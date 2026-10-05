#!/usr/bin/env bash
# ==============================================================================
# launch_swarm_2d.sh — One-shot launcher: 3 ArduCopter SITL drones in 2D map
#
# Opens 3 MAVProxy map windows (one per drone, each showing an arrowhead).
# Also forwards all 3 to UDP 14550 so QGroundControl can see all at once.
#
# Usage:  bash sitl/launch_swarm_2d.sh
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
PARAMS="${SCRIPT_DIR}/swarm_params.parm"
VENV_ACTIVATE="${HOME}/venv-ardupilot/bin/activate"

# --- Pre-flight sanity checks ---
if [ ! -f "${VENV_ACTIVATE}" ]; then
    echo "[PRE-FLIGHT ERROR] Virtualenv activate script not found at: ${VENV_ACTIVATE}" >&2
    echo "Please ensure the ArduPilot Python environment is set up at ~/venv-ardupilot." >&2
    exit 1
fi

if ! command -v xterm >/dev/null 2>&1; then
    echo "[PRE-FLIGHT ERROR] 'xterm' is not installed or not in PATH." >&2
    echo "Please install it via: sudo apt-get install -y xterm" >&2
    exit 1
fi

SIM_VEHICLE="${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py"
if [ ! -f "${SIM_VEHICLE}" ]; then
    echo "[PRE-FLIGHT ERROR] sim_vehicle.py not found at: ${SIM_VEHICLE}" >&2
    echo "Please check ARDUPILOT_HOME (current value: ${ARDUPILOT_HOME})." >&2
    exit 1
fi

if [ ! -f "${PARAMS}" ]; then
    echo "[PRE-FLIGHT ERROR] swarm_params.parm not found at: ${PARAMS}" >&2
    exit 1
fi

# Activate venv in current shell
source "${VENV_ACTIVATE}"

# Kill any leftover SITL processes cleanly
echo "[LAUNCH] Cleaning up any existing SITL instances..."
pkill -9 -f "arducopter" 2>/dev/null
pkill -9 -f "sim_vehicle" 2>/dev/null
pkill -9 -f "Drone.*SITL" 2>/dev/null
sleep 2

cd "${ARDUPILOT_HOME}/ArduCopter" || { echo "ERROR: ArduCopter directory not found at ${ARDUPILOT_HOME}/ArduCopter"; exit 1; }

# --- Drone 1: Apex Leader (Instance 0) ---
# Map window title: "Drone 1 - Apex Leader"
# UDP 14552 -> swarm controller  |  UDP 14550 -> QGC
echo "[LAUNCH] Starting Drone 1 (Apex Leader)..."
xterm -hold -T "Drone 1 - Apex Leader [SITL]" -geometry 100x25+0+0 -e /bin/bash -c "
    source '${VENV_ACTIVATE}' || exit 1
    cd '${ARDUPILOT_HOME}/ArduCopter' || exit 1
    exec python3 '${SIM_VEHICLE}' \
        -v ArduCopter \
        -I 0 \
        -N \
        --auto-sysid \
        --custom-location=-35.363261,149.165230,584,0 \
        --out=udp:127.0.0.1:14550 \
        --out=udp:127.0.0.1:14552 \
        --add-param-file='${PARAMS}' \
        --map
" &

sleep 5

# --- Drone 2: Left Wing (Instance 1) ---
echo "[LAUNCH] Starting Drone 2 (Left Wing)..."
xterm -hold -T "Drone 2 - Left Wing [SITL]" -geometry 100x25+0+450 -e /bin/bash -c "
    source '${VENV_ACTIVATE}' || exit 1
    cd '${ARDUPILOT_HOME}/ArduCopter' || exit 1
    exec python3 '${SIM_VEHICLE}' \
        -v ArduCopter \
        -I 1 \
        -N \
        --auto-sysid \
        --custom-location=-35.363261,149.165285,584,0 \
        --out=udp:127.0.0.1:14550 \
        --out=udp:127.0.0.1:14562 \
        --add-param-file='${PARAMS}' \
        --map
" &

sleep 5

# --- Drone 3: Right Wing (Instance 2) ---
echo "[LAUNCH] Starting Drone 3 (Right Wing)..."
xterm -hold -T "Drone 3 - Right Wing [SITL]" -geometry 100x25+750+0 -e /bin/bash -c "
    source '${VENV_ACTIVATE}' || exit 1
    cd '${ARDUPILOT_HOME}/ArduCopter' || exit 1
    exec python3 '${SIM_VEHICLE}' \
        -v ArduCopter \
        -I 2 \
        -N \
        --auto-sysid \
        --custom-location=-35.363261,149.165180,584,0 \
        --out=udp:127.0.0.1:14550 \
        --out=udp:127.0.0.1:14572 \
        --add-param-file='${PARAMS}' \
        --map
" &

disown -a 2>/dev/null || true

echo ""
echo "================================================================="
echo "  3 SITL instances launching in separate xterm windows."
echo "  Each window shows a 2D MAVProxy map with an arrowhead drone."
echo ""
echo "  Waiting 60s for EKF/GPS to converge before swarm controller..."
echo "  Watch for: 'APM: EKF2 IMU0 using GPS' or EKF3 in each window."
echo "================================================================="
echo ""
echo "  Once all 3 show GPS lock, run in a NEW terminal:"
echo "    cd ~/swarm_drones_fyp"
echo "    python3 sitl/interactive_flight_console.py --standalone"
echo "================================================================="
