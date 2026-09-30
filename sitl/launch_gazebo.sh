#!/usr/bin/env bash
# ==============================================================================
# Launch Gazebo Harmonic 3D Simulation with Iris Quadcopter
# ==============================================================================

export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=/home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/models:/home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

echo "================================================================="
echo "   Launching Gazebo Harmonic 3D Simulation World                "
echo "   Plugin Path:   /home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/build"
echo "   Resource Path: /home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/models"
echo "================================================================="

gz sim -v4 -r /home/drone/.gemini/antigravity/scratch/ardupilot_gazebo/worlds/iris_runway.sdf
