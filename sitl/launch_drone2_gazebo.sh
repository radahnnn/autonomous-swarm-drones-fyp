#!/usr/bin/env bash
# ==============================================================================
# Launch ArduCopter SITL Instance 1 (Drone 2) connected to Gazebo 3D
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"

if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

cd "${ARDUPILOT_HOME}/ArduCopter" || exit 1

echo "================================================================="
echo "   Starting Drone 2 (SYSID 2) connected to Gazebo JSON Model     "
echo "   FDM Port: 9012 <-> Gazebo                                    "
echo "   MAVLink Out: UDP 14550 (QGC) & UDP 14562 (Controller)       "
echo "================================================================="

python3 "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    -I 1 \
    -N \
    --auto-sysid \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14562 \
    --add-param-file="${SCRIPT_DIR}/swarm_params.parm" \
    --map
