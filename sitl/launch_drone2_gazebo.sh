#!/usr/bin/env bash
# ==============================================================================
# Launch ArduCopter SITL Instance 1 (Drone 2 / Follower 1) for Gazebo 3D
# ==============================================================================

source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/ArduCopter

echo "================================================================="
echo "   Starting Drone 2 (SYSID 2 - Left Wing) for Gazebo 3D         "
echo "   FDM Port: 9012 <-> Gazebo                                    "
echo "   MAVLink Out: UDP 14550 (QGC) & UDP 14562 (Controller)       "
echo "================================================================="

python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    -I 1 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14562 \
    --add-param-file=/home/drone/swarm_drones_fyp/sitl/swarm_params.parm \
    --map
