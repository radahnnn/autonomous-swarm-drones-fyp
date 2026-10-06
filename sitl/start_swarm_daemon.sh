#!/usr/bin/env bash
# ==============================================================================
# start_swarm.sh - Persistent daemon managing all 3 SITL drones
# ==============================================================================

ARDUPILOT_DIR="/home/drone/ardupilot"
SWARM_PARAMS="/home/drone/swarm_drones_fyp/sitl/swarm_params.parm"
VENV_PYTHON="/home/drone/venv-ardupilot/bin/python3"

# Cleanup function on exit
cleanup() {
    echo "[SWARM] Terminating all swarm drone processes..."
    kill $(jobs -p) 2>/dev/null || true
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    exit 0
}
trap cleanup SIGTERM SIGINT EXIT

echo "[SWARM] Cleaning up previous processes..."
pkill -9 -f "arducopter" 2>/dev/null || true
pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
pkill -9 -f "mavproxy.py" 2>/dev/null || true
sleep 2

echo "[SWARM] Starting Drone 1 (Apex Leader, SysID 1)..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 0 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165230,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14552 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM] Starting Drone 2 (Left Wing, SysID 2)..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 1 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165285,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14560 \
    --out=udp:127.0.0.1:14562 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM] Starting Drone 3 (Right Wing, SysID 3)..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 2 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165180,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14570 \
    --out=udp:127.0.0.1:14572 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

echo "[SWARM] All 3 drones running in daemon mode. Swarm is active."
# Keep script alive indefinitely to keep child jobs running
while true; do
    sleep 5
done
