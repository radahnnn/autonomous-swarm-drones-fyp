#!/usr/bin/env bash
# ==============================================================================
# start_swarm_5drones_2d.sh - Persistent daemon managing all 5 SITL drones (2D)
# ==============================================================================

ARDUPILOT_DIR="/home/drone/ardupilot"
SWARM_PARAMS="/home/drone/swarm_drones_fyp/sitl/swarm_params.parm"
VENV_PYTHON="/home/drone/venv-ardupilot/bin/python3"

# Cleanup function on exit
cleanup() {
    echo "[SWARM-5D] Terminating all 5 swarm drone processes..."
    kill $(jobs -p) 2>/dev/null || true
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    exit 0
}
trap cleanup SIGTERM SIGINT EXIT

echo "[SWARM-5D] Cleaning up previous processes..."
pkill -9 -f "arducopter" 2>/dev/null || true
pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
pkill -9 -f "mavproxy.py" 2>/dev/null || true
sleep 2

echo "[SWARM-5D] Starting Drone 1 (Apex Leader, SysID 1)..."
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

echo "[SWARM-5D] Starting Drone 2 (Left Inner, SysID 2)..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 1 \
    -N \
    --auto-sysid \
    --custom-location=-35.363283,149.165208,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14560 \
    --out=udp:127.0.0.1:14562 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM-5D] Starting Drone 3 (Right Inner, SysID 3)..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 2 \
    -N \
    --auto-sysid \
    --custom-location=-35.363283,149.165252,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14570 \
    --out=udp:127.0.0.1:14572 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM-5D] Starting Drone 4 (Left Outer, SysID 4)..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 3 \
    -N \
    --auto-sysid \
    --custom-location=-35.363306,149.165186,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14580 \
    --out=udp:127.0.0.1:14582 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM-5D] Starting Drone 5 (Right Outer, SysID 5)..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 4 \
    -N \
    --auto-sysid \
    --custom-location=-35.363306,149.165274,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14590 \
    --out=udp:127.0.0.1:14592 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

echo "================================================================="
echo "  [SWARM-5D] All 5 drones running in daemon mode!"
echo "  Telemetry stream forwarded to QGroundControl (UDP 14550)."
echo "  Swarm controller channels active on UDP 14552, 14562, 14572, 14582, 14592."
echo "================================================================="

while true; do
    sleep 5
done
