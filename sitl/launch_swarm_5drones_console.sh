#!/usr/bin/env bash
# ==============================================================================
# launch_swarm_5drones_console.sh - Open interactive Swarm Pilot Console (5 Drones)
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$DISPLAY" ]; then
    export DISPLAY=":0"
fi

if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="Swarm Pilot Console (5 Drones)" -- bash -c "
        source /home/drone/venv-ardupilot/bin/activate
        cd /home/drone/swarm_drones_fyp
        python3 sitl/interactive_flight_console_5drones.py
        echo ''
        echo 'Console closed. Press Enter to exit.'
        read
    "
elif command -v xterm >/dev/null 2>&1; then
    xterm -geometry 100x32 -T "Swarm Pilot Console (5 Drones)" -e bash -c "
        source /home/drone/venv-ardupilot/bin/activate
        cd /home/drone/swarm_drones_fyp
        python3 sitl/interactive_flight_console_5drones.py
        echo ''
        echo 'Console closed. Press Enter to exit.'
        read
    "
else
    source /home/drone/venv-ardupilot/bin/activate
    cd /home/drone/swarm_drones_fyp
    python3 sitl/interactive_flight_console_5drones.py
fi
