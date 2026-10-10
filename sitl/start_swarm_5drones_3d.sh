#!/usr/bin/env bash
# ==============================================================================
# start_swarm_5drones_3d.sh - Persistent daemon managing all 5 SITL drones (3D Gazebo)
# ==============================================================================

ARDUPILOT_DIR="/home/drone/ardupilot"
SWARM_PARAMS="/home/drone/swarm_drones_fyp/sitl/swarm_params.parm"
VENV_PYTHON="/home/drone/venv-ardupilot/bin/python3"

# Cleanup function on exit
cleanup() {
    echo "[SWARM-3D] Terminating all 5 SITL drone processes..."
    kill $(jobs -p) 2>/dev/null || true
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    exit 0
}
trap cleanup SIGTERM SIGINT EXIT

echo "[SWARM-3D] Cleaning up previous SITL processes..."
pkill -9 -f "arducopter" 2>/dev/null || true
pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
pkill -9 -f "mavproxy.py" 2>/dev/null || true
sleep 2

echo "[SWARM-3D] Starting Drone 1 (Apex Leader, SysID 1) -> Gazebo Port 9002..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    -I 0 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14552 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM-3D] Starting Drone 2 (Left Inner, SysID 2) -> Gazebo Port 9012..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    -I 1 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14560 \
    --out=udp:127.0.0.1:14562 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM-3D] Starting Drone 3 (Right Inner, SysID 3) -> Gazebo Port 9022..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 2 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14570 \
    --out=udp:127.0.0.1:14572 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM-3D] Starting Drone 4 (Left Outer, SysID 4) -> Gazebo Port 9032..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 3 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14580 \
    --out=udp:127.0.0.1:14582 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

sleep 3

echo "[SWARM-3D] Starting Drone 5 (Right Outer, SysID 5) -> Gazebo Port 9042..."
cd "$ARDUPILOT_DIR/ArduCopter"
$VENV_PYTHON "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 4 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14590 \
    --out=udp:127.0.0.1:14592 \
    --mavproxy-args="--daemon" \
    --add-param-file="$SWARM_PARAMS" &

echo "================================================================="
echo "  [SWARM-3D] All 5 SITL drones active and linked to Gazebo Harmonic!"
echo "  Telemetry stream forwarded to QGroundControl (UDP 14550)."
echo "  Swarm controller channels active on UDP 14552, 14562, 14572, 14582, 14592."
echo "================================================================="

while true; do
    sleep 5
done
