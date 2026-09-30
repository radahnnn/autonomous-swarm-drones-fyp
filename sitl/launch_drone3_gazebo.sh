#!/usr/bin/env bash
# ==============================================================================
# Launch ArduCopter SITL Instance 2 (Drone 3 / Follower 2) for Gazebo 3D
# ==============================================================================

source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/ArduCopter

echo "================================================================="
echo "   Starting Drone 3 (SYSID 3 - Right Wing) for Gazebo 3D        "
echo "   FDM Port: 9022 <-> Gazebo                                    "
echo "   MAVLink Out: UDP 14550 (QGC) & UDP 14572 (Controller)       "
echo "================================================================="

python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    -I 2 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14572 \
    --add-param-file=/home/drone/swarm_drones_fyp/sitl/swarm_params.parm \
    --map
