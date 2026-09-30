#!/usr/bin/env bash
# ==============================================================================
# Launch Gazebo Harmonic 3D Simulation with 3 Custom Cinewhoop Swarm Drones
# ==============================================================================

export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=/home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/models:/home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
export SDF_PATH=/home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/models:${SDF_PATH}

echo "================================================================="
echo "   Launching Gazebo Harmonic 3D Swarm World (3 Cinewhoops)      "
echo "   World: cinewhoop_3drones.sdf                                 "
echo "   - Drone 1 (Leader):     Port 9002 (Pose: 0, 0, 0.05)         "
echo "   - Drone 2 (Left Wing):  Port 9012 (Pose: -4, 3.5, 0.05)      "
echo "   - Drone 3 (Right Wing): Port 9022 (Pose: -4, -3.5, 0.05)     "
echo "================================================================="

gz sim -v4 -r /home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/worlds/cinewhoop_3drones.sdf

