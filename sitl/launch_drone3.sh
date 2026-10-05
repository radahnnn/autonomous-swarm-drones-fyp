#!/usr/bin/env bash
# Launch ArduCopter SITL Instance 2 (Drone 3 / SYSID 3)
# Spawned ~5 meters West of Drone 1
# Telemetry forwarded to QGroundControl (14550) and script (14572)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"

if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

cd "${ARDUPILOT_HOME}/ArduCopter" || exit 1
echo "Starting Drone 3 (SYSID 3) on Instance 2..."
python3 "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -I 2 \
    -N \
    --auto-sysid \
    --custom-location=-35.363261,149.165180,584,0 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14572 \
    --add-param-file="${SCRIPT_DIR}/swarm_params.parm" \
    --map
