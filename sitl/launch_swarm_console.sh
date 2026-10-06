#!/usr/bin/env bash
# ==============================================================================
# launch_swarm_console.sh - Open interactive Swarm Pilot Console in a new terminal
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="Swarm Pilot Console (3 Drones)" -- bash -c "
        source /home/drone/venv-ardupilot/bin/activate
        cd /home/drone/swarm_drones_fyp
        python3 sitl/interactive_flight_console.py
        echo ''
        echo 'Console closed. Press Enter to exit.'
        read
    "
elif command -v xterm >/dev/null 2>&1; then
    xterm -geometry 90x30 -T "Swarm Pilot Console (3 Drones)" -e bash -c "
        source /home/drone/venv-ardupilot/bin/activate
        cd /home/drone/swarm_drones_fyp
        python3 sitl/interactive_flight_console.py
        echo ''
        echo 'Console closed. Press Enter to exit.'
        read
    "
else
    source /home/drone/venv-ardupilot/bin/activate
    cd /home/drone/swarm_drones_fyp
    python3 sitl/interactive_flight_console.py
fi
