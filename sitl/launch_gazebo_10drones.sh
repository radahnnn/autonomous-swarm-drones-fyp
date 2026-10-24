#!/usr/bin/env bash
# Generate a ten-vehicle Gazebo world and run it with the checked-in airframe model.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
source "${SCRIPT_DIR}/consolidated_env.sh"

if ! command -v gz >/dev/null 2>&1; then
    echo "[ERROR] Gazebo Sim (gz) is not installed or not on PATH." >&2
    exit 1
fi
if [[ ! -f "${ARDUPILOT_GAZEBO_DIR}/build/ArduPilotPlugin.so" &&
      ! -f "${ARDUPILOT_GAZEBO_DIR}/build/libArduPilotPlugin.so" ]]; then
    echo "[ERROR] ArduPilot Gazebo plugin was not found under ${ARDUPILOT_GAZEBO_DIR}/build." >&2
    exit 1
fi

TEMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/swarm-gazebo-10.XXXXXX")"
cleanup() { rm -rf -- "${TEMP_ROOT}"; }
trap cleanup EXIT INT TERM

export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_DIR}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${TEMP_ROOT}/models:${REPO_ROOT}/simulator/gazebo/models:${REPO_ROOT}/simulator/gazebo/worlds:${ARDUPILOT_GAZEBO_DIR}/models:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="${TEMP_ROOT}/models:${REPO_ROOT}/simulator/gazebo/models:${ARDUPILOT_GAZEBO_DIR}/models:${SDF_PATH:-}"

python3 -m sitl.generate_gazebo_world --drones 10 --output-dir "${TEMP_ROOT}"
WORLD_FILE="${TEMP_ROOT}/worlds/cinewhoop_10drones.sdf"

if command -v ss >/dev/null 2>&1; then
    for index in {0..9}; do
        port=$((9002 + 10 * index))
        if ss -H -lun | grep -Eq ":${port}[[:space:]]"; then
            echo "[ERROR] Gazebo FDM port ${port} is already in use. Close the existing world first." >&2
            exit 1
        fi
    done
fi

echo "Launching ten-drone Gazebo world: ${WORLD_FILE}"
echo "After Gazebo is ready, run: python3 -m sitl.swarm_console --drones 10 --gazebo --ekf-timeout 900"
gz sim -v4 -r "${WORLD_FILE}"
