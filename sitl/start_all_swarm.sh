#!/usr/bin/env bash
# ==============================================================================
# start_all_swarm.sh - Master Single-Command Swarm Launcher
# Starts QGroundControl, 5 SITL Drones (2D or Gazebo 3D) and the Self-Healing
# Console. Every daemon runs in its own session (setsid) with its own log and
# PID file, and readiness is probed instead of using fixed sleeps.
#
# Usage: start_all_swarm.sh [--gazebo|-3d]
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_PYTHON="/home/drone/venv-ardupilot/bin/python3"
QGC_LAUNCHER="/home/drone/Documents/gemini work/launch_qgroundcontrol.sh"
RUN_DIR="${HOME}/.swarm"
mkdir -p "${RUN_DIR}"

echo "================================================================="
echo "   AUTONOMOUS SWARM DRONES - ALL-IN-ONE ONE-CLICK LAUNCHER       "
echo "================================================================="

# start_daemon <name> <command...>
# Fully detached (new session, no tty, no SIGHUP), logs to ~/.swarm/<name>.log,
# PID recorded in ~/.swarm/<name>.pid.
start_daemon() {
    local name="$1"; shift
    setsid -f bash -c 'echo $$ > "$1"; shift; exec "$@"' _ \
        "${RUN_DIR}/${name}.pid" "$@" >"${RUN_DIR}/${name}.log" 2>&1 </dev/null
}

# stop_daemon <name>: SIGTERM the whole session/process group, then SIGKILL.
stop_daemon() {
    local pidfile="${RUN_DIR}/$1.pid"
    [[ -f "${pidfile}" ]] || return 0
    local pid
    pid="$(cat "${pidfile}")"
    if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null
        for _ in 1 2 3 4 5 6; do kill -0 "${pid}" 2>/dev/null || break; sleep 0.5; done
        kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null
    fi
    rm -f "${pidfile}"
}

# wait_for <description> <timeout_s> <command...>: poll until the command succeeds.
wait_for() {
    local desc="$1" timeout="$2"; shift 2
    local t0=$SECONDS
    while ! "$@" >/dev/null 2>&1; do
        if (( SECONDS - t0 >= timeout )); then
            echo "!! Timed out after ${timeout}s waiting for: ${desc}"
            return 1
        fi
        sleep 1
    done
    echo "   ready: ${desc} ($((SECONDS - t0))s)"
}

mavlink_ready() {  # heartbeat on all five control ports
    "${VENV_PYTHON}" - <<'EOF'
import sys
from pymavlink import mavutil
for port in (14552, 14562, 14572, 14582, 14592):
    c = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    ok = c.wait_heartbeat(timeout=3)
    c.close()
    if not ok:
        sys.exit(1)
EOF
}

gazebo_ready() { gz topic -l 2>/dev/null | grep -q "/world/"; }

# 1. Clean up stale simulator processes (QGC is preserved if alive)
echo ">> [1/4] Sweeping prior flight simulator processes..."
for name in console gazebo sitl; do stop_daemon "${name}"; done
for pat in "sim_vehicle.py" "arducopter" "mavproxy" "swarm_console" "gz sim"; do
    pkill -TERM -f "${pat}" 2>/dev/null || true
done
sleep 2
for pat in "arducopter" "gz sim"; do
    pkill -KILL -f "${pat}" 2>/dev/null || true
done

# 2. Ensure QGroundControl is active (match the binary name exactly)
if pgrep -x "QGroundControl" >/dev/null 2>&1 || pgrep -f "qgroundcontrol-app" >/dev/null 2>&1; then
    echo ">> [2/4] QGroundControl is already active and running."
else
    echo ">> [2/4] Launching QGroundControl on DISPLAY=:1..."
    DISPLAY=:1 start_daemon qgc "${QGC_LAUNCHER}"
fi

# 3. Launch 5-drone SITL swarm daemons
if [[ "$1" == "--gazebo" || "$1" == "-3d" ]]; then
    echo ">> [3/4] Launching Gazebo 3D Simulation & SITL daemon..."
    DISPLAY=:1 start_daemon gazebo bash "${PROJECT_DIR}/sitl/launch_gazebo_5drones.sh"
    wait_for "Gazebo world" 120 gazebo_ready || exit 1
    DISPLAY=:1 start_daemon sitl bash "${PROJECT_DIR}/sitl/start_swarm_5drones_3d.sh"
    CONSOLE_FLAG="--gazebo"
else
    echo ">> [3/4] Launching 5-Drone 2D SITL Daemon..."
    DISPLAY=:1 start_daemon sitl bash "${PROJECT_DIR}/sitl/start_swarm_5drones_2d.sh"
    CONSOLE_FLAG="--standalone"
fi

echo ">> Waiting for MAVLink heartbeats from all 5 drones..."
wait_for "5/5 drone heartbeats" 180 mavlink_ready || { echo "See logs in ${RUN_DIR}/"; exit 1; }

# 4. Launch the Swarm Pilot Console in a visible xterm window.
#    The console itself waits for EKF convergence (EKF_STATUS_REPORT) before arming.
echo ">> [4/4] Opening Swarm Console Terminal..."
DISPLAY=:1 start_daemon console xterm -geometry 102x36+360+70 \
    -bg "#1e1e2e" -fg "#cdd6f4" \
    -T "Swarm Pilot Console (5 drones)" \
    -e bash -c "
        source /home/drone/venv-ardupilot/bin/activate
        cd '${PROJECT_DIR}'
        python3 -u sitl/swarm_console.py --drones 5 ${CONSOLE_FLAG} 2>&1 | tee -a '${RUN_DIR}/console.log'
        echo ''
        echo \"Console exited (code \${PIPESTATUS[0]}). Full log: ${RUN_DIR}/console.log\"
        echo 'Press Enter to close this window...'
        read
    "

echo "================================================================="
echo "   ALL 5 SWARM SYSTEMS ARE ONLINE. LOGS: ${RUN_DIR}/*.log        "
echo "   (3D mode: the console now waits for EKF lock before arming.)  "
echo "================================================================="
