#!/usr/bin/env bash
# Launch ArduCopter SITL Instance 0 (Drone 1 / SYSID 1)
# Communication output on UDP 127.0.0.1:14550

source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/ArduCopter
echo "Starting Drone 1 (SYSID 1) on UDP port 14550..."
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -I 0 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165230,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14552 \
    --add-param-file=/home/drone/swarm_drones_fyp/sitl/swarm_params.parm \
    --map
