#!/usr/bin/env bash
# ==============================================================================
# launch_console.sh - open the swarm flight console in its own terminal window
#   usage: sitl/launch_console.sh [--drones N] [--gazebo|--standalone] [--restart] [--auto-launch]
# All arguments are passed straight to sitl/swarm_console.py. Paths are relative to this
# repo; set VENV_ACTIVATE if your ArduPilot virtualenv lives somewhere else.
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
source "${SCRIPT_DIR}/consolidated_env.sh"
VENV_ACTIVATE="${VENV_ACTIVATE:-}"
export DISPLAY="${DISPLAY:-:0}"

RUN="source '${SCRIPT_DIR}/consolidated_env.sh'; [ -f '${VENV_ACTIVATE}' ] && source '${VENV_ACTIVATE}'; cd '${PROJECT_DIR}'; python3 sitl/swarm_console.py $*; echo; echo 'Console closed. Press Enter to exit.'; read"

if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="Swarm Flight Console" -- bash -c "${RUN}"
elif command -v xterm >/dev/null 2>&1; then
    xterm -geometry 105x38 -T "Swarm Flight Console" -e bash -c "${RUN}"
else
    bash -c "${RUN}"
fi
