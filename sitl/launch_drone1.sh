#!/usr/bin/env bash
# Launch ArduCopter SITL Instance 0 (Drone 1 / SYSID 1)
# Communication output on UDP 127.0.0.1:14550

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"

if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

cd "${ARDUPILOT_HOME}/ArduCopter" || exit 1
echo "Starting Drone 1 (SYSID 1) on UDP port 14550..."
python3 "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 0 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165230,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14552 \
    --add-param-file="${SCRIPT_DIR}/swarm_params.parm" \
    --map
