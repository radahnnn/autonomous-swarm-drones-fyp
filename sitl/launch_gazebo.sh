#!/usr/bin/env bash
# ==============================================================================
# Launch Gazebo Harmonic 3D Simulation with 3 Custom Cinewhoop Swarm Drones
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "${SCRIPT_DIR}")"

# Location of the built ardupilot_gazebo plugin checkout (override if installed elsewhere)
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"

# Repo-tracked Cinewhoop models/world first, then the ardupilot_gazebo ones (e.g. model://runway)
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_DIR}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="${REPO_DIR}/simulator/gazebo/models:${REPO_DIR}/simulator/gazebo/worlds:${ARDUPILOT_GAZEBO_DIR}/models:${ARDUPILOT_GAZEBO_DIR}/worlds:${GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="${REPO_DIR}/simulator/gazebo/models:${ARDUPILOT_GAZEBO_DIR}/models:${SDF_PATH}"

echo "================================================================="
echo "   Launching Gazebo Harmonic 3D Swarm World (3 Cinewhoops)      "
echo "   World: cinewhoop_3drones.sdf                                 "
echo "   - Drone 1 (Leader):     Port 9002 (Pose: 0, 0, 0.05)         "
echo "   - Drone 2 (Left Wing):  Port 9012 (Pose: -4, 3.5, 0.05)      "
echo "   - Drone 3 (Right Wing): Port 9022 (Pose: -4, -3.5, 0.05)     "
echo "================================================================="

gz sim -v4 -r "${REPO_DIR}/simulator/gazebo/worlds/cinewhoop_3drones.sdf"

