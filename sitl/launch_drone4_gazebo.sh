#!/usr/bin/env bash
# ==============================================================================
# Launch ArduCopter SITL Instance 3 (Drone 4) connected to Gazebo 3D
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"

if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

cd "${ARDUPILOT_HOME}/ArduCopter" || exit 1

echo "================================================================="
echo "   Starting Drone 4 (SYSID 4) connected to Gazebo JSON Model     "
echo "   FDM Port: 9032 <-> Gazebo                                    "
echo "   MAVLink Out: UDP 14550 (QGC) & UDP 14582 (Controller)       "
echo "================================================================="

python3 "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    -I 3 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14582 \
    --add-param-file="${SCRIPT_DIR}/swarm_params.parm" \
    --map
