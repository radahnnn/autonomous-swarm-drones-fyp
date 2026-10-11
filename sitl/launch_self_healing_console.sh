#!/usr/bin/env bash
# ==============================================================================
# launch_self_healing_console.sh - Launch Self-Healing Swarm Console in Xterm
# ==============================================================================

if [ -d "/home/drone/.gemini/antigravity/scratch/swarm_drones_fyp" ]; then
    PROJECT_DIR="/home/drone/.gemini/antigravity/scratch/swarm_drones_fyp"
elif [ -d "/home/drone/swarm_drones_fyp" ]; then
    PROJECT_DIR="/home/drone/swarm_drones_fyp"
else
    PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
fi
VENV_PYTHON="/home/drone/venv-ardupilot/bin/python3"

echo "[SWARM-PILOT] Launching Self-Healing Swarm Console..."

# Determine mode based on arguments or Gazebo detection
MODE_FLAG=""
if [[ "$1" == "--gazebo" || "$1" == "-g" ]]; then
    MODE_FLAG="--gazebo"
elif [[ "$1" == "--standalone" || "$1" == "-2d" ]]; then
    MODE_FLAG="--standalone"
fi

DISPLAY=:1 xterm -geometry 105x38+1050+30 \
    -T "Self-Healing Swarm Pilot Console (Hybrid Fault-Tolerant)" \
    -e bash -c "
        source /home/drone/venv-ardupilot/bin/activate
        cd '${PROJECT_DIR}'
        python3 sitl/self_healing_swarm_console.py ${MODE_FLAG}
        echo ''
        echo 'Press Enter to exit...'
        read
    " &
