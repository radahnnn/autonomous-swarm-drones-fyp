#!/usr/bin/env bash
# Launch ArduCopter SITL Instance 1 (Drone 2 / SYSID 2)
# Spawned ~5 meters East of Drone 1
# Communication output on UDP 127.0.0.1:14560

source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/ArduCopter
echo "Starting Drone 2 (SYSID 2) on UDP port 14560..."
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -I 1 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14562 \
    --add-param-file=/home/drone/swarm_drones_fyp/sitl/swarm_params.parm \
    --map
