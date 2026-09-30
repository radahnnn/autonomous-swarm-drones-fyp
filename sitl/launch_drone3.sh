#!/usr/bin/env bash
# Launch ArduCopter SITL Instance 2 (Drone 3 / SYSID 3)
# Spawned ~5 meters West of Drone 1
# Telemetry forwarded to QGroundControl (14550) and script (14570 / TCP 5782)

source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/ArduCopter
echo "Starting Drone 3 (SYSID 3) on Instance 2..."
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -I 2 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165180,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14570 \
    --add-param-file=/home/drone/swarm_drones_fyp/sitl/swarm_params.parm \
    --map
