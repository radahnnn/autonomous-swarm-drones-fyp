#!/usr/bin/env bash
# ==============================================================================
# Cleanly terminate all ArduPilot SITL, QGroundControl, Swarm controllers, and Gazebo
# ==============================================================================

echo "Stopping all ArduPilot SITL, QGroundControl, and Gazebo processes..."

pkill -9 -f "QGroundControl" 2>/dev/null || true
pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
pkill -9 -f "arducopter" 2>/dev/null || true
pkill -9 -f "mavproxy" 2>/dev/null || true
pkill -9 -f "swarm_3_drones.py" 2>/dev/null || true
pkill -9 -f "autonomous_wingman.py" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true

sleep 1

echo "================================================================="
echo "   All flight processes have been cleanly terminated!            "
echo "   All ports (14550, 14560, 5760-5790, 9002) are now free.      "
echo "================================================================="
