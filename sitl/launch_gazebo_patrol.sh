#!/usr/bin/env bash
# The survey-patrol world (6 drones with cameras, poles, markers, trees) at a 2.5 ms physics step.
#   bash sitl/launch_gazebo_patrol.sh              # with the GUI
#   HEADLESS=1 bash sitl/launch_gazebo_patrol.sh   # server only
# Then: python3 -m sitl.fleet_ctl scenarios/gazebo_patrol.txt --drones 6 --gazebo --out runs/independent/<name> \
#         --extra "--ekf-timeout 1800 --time-scale 0.2 $(python3 -m sitl.patrol_layout console-args)"
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# FORMATION_SCALE must match the console (--realistic uses 1.4) so spawn layout and markers line up with the slots.
export FYP_FORMATION_SCALE="${FORMATION_SCALE:-1.0}"
eval "$(python3 -m sitl.patrol_layout env)"
export STEP_MS="${STEP_MS:-1.25}"
exec bash sitl/launch_gazebo_world.sh 6
