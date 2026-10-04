#!/usr/bin/env bash
# ==============================================================================
# Launch Gazebo Harmonic 3D Simulation with 3 Custom Cinewhoop Swarm Drones
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [ -z "${ARDUPILOT_GAZEBO_DIR}" ]; then
    if [ -d "$HOME/ardupilot_gazebo" ]; then
        ARDUPILOT_GAZEBO_DIR="$HOME/ardupilot_gazebo"
    elif [ -d "$HOME/.gemini/antigravity/scratch/ardupilot_gazebo" ]; then
        ARDUPILOT_GAZEBO_DIR="$HOME/.gemini/antigravity/scratch/ardupilot_gazebo"
    else
        ARDUPILOT_GAZEBO_DIR="$HOME/ardupilot_gazebo"
    fi
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_DIR}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="${REPO_ROOT}/simulator/gazebo/models:${REPO_ROOT}/simulator/gazebo/worlds:${ARDUPILOT_GAZEBO_DIR}/models:${GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="${REPO_ROOT}/simulator/gazebo/models:${ARDUPILOT_GAZEBO_DIR}/models:${SDF_PATH}"

WORLD_FILE="${REPO_ROOT}/simulator/gazebo/worlds/cinewhoop_3drones.sdf"

echo "================================================================="
echo "   Launching Gazebo Harmonic 3D Swarm World (3 Cinewhoops)      "
echo "   World: ${WORLD_FILE}"
echo "   - Drone 1 (Leader):     Port 9002 (Pose: 0, 0, 0.05)         "
echo "   - Drone 2 (Left Wing):  Port 9012 (Pose: -3, -3, 0.05)       "
echo "   - Drone 3 (Right Wing): Port 9022 (Pose:  3, -3, 0.05)       "
echo "================================================================="

if [ ! -f "${WORLD_FILE}" ]; then
    echo "[ERROR] Gazebo world file not found at: ${WORLD_FILE}"
    exit 1
fi

gz sim -v4 -r "${WORLD_FILE}"
