#!/usr/bin/env bash
# Launch ArduCopter SITL Instance 1 (Drone 2 / SYSID 2)
# Spawned ~5 meters East of Drone 1
# Communication output on UDP 127.0.0.1:14562

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"

if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

cd "${ARDUPILOT_HOME}/ArduCopter" || exit 1
echo "Starting Drone 2 (SYSID 2) on UDP port 14562..."
python3 "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 1 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165285,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14562 \
    --add-param-file="${SCRIPT_DIR}/swarm_params.parm" \
    --map
