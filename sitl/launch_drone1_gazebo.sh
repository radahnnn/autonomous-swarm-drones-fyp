#!/usr/bin/env bash
# ==============================================================================
# Launch ArduCopter SITL Instance 0 (Drone 1) connected to Gazebo 3D
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/ArduCopter

echo "================================================================="
echo "   Starting Drone 1 (SYSID 1) connected to Gazebo JSON Model     "
echo "   FDM Port: 9002 <-> Gazebo                                    "
echo "   MAVLink Out: UDP 14550 (QGC) & UDP 14552 (Controller)       "
echo "================================================================="

python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    -I 0 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14552 \
    --add-param-file="${SCRIPT_DIR}/swarm_params.parm" \
    --map
