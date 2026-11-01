#!/usr/bin/env bash
# Generate an N-vehicle (6-10) Gazebo world and run it with the checked-in airframe model.
#   bash sitl/launch_gazebo_world.sh 6              # with the GUI (needs DISPLAY)
#   HEADLESS=1 bash sitl/launch_gazebo_world.sh 6   # server only (use for timing measurements)
#   STEP_MS=2.5 bash sitl/launch_gazebo_world.sh 6  # larger physics step: fewer lockstep exchanges, faster
#   CAMERA=1 OBSTACLES="12,-4,0.6 0,9,0.4" bash sitl/launch_gazebo_world.sh 6   # cameras + poles (north,east,radius)
set -euo pipefail

COUNT="${1:-6}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
source "${SCRIPT_DIR}/consolidated_env.sh"

command -v gz >/dev/null 2>&1 || { echo "[ERROR] Gazebo Sim (gz) is not installed or not on PATH." >&2; exit 1; }
if [[ ! -f "${ARDUPILOT_GAZEBO_DIR}/build/ArduPilotPlugin.so" &&
      ! -f "${ARDUPILOT_GAZEBO_DIR}/build/libArduPilotPlugin.so" ]]; then
    echo "[ERROR] ArduPilot Gazebo plugin was not found under ${ARDUPILOT_GAZEBO_DIR}/build." >&2
    exit 1
fi

TEMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/swarm-gazebo-${COUNT}.XXXXXX")"
trap 'rm -rf -- "${TEMP_ROOT}"' EXIT INT TERM

export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_DIR}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${TEMP_ROOT}/models:${REPO_ROOT}/simulator/gazebo/models:${REPO_ROOT}/simulator/gazebo/worlds:${ARDUPILOT_GAZEBO_DIR}/models:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="${TEMP_ROOT}/models:${REPO_ROOT}/simulator/gazebo/models:${ARDUPILOT_GAZEBO_DIR}/models:${SDF_PATH:-}"

cd "${REPO_ROOT}"
EXTRA=()
[[ -n "${CAMERA:-}" ]] && EXTRA+=(--camera)
for pole in ${OBSTACLES:-}; do EXTRA+=("--obstacle=${pole}"); done
for marker in ${MARKERS:-}; do EXTRA+=("--marker=${marker}"); done
for tree in ${TREES:-}; do EXTRA+=("--tree=${tree}"); done
python3 -m sitl.generate_gazebo_world --drones "${COUNT}" --output-dir "${TEMP_ROOT}" --step-ms "${STEP_MS:-1}" "${EXTRA[@]}"
WORLD_FILE="${TEMP_ROOT}/worlds/cinewhoop_${COUNT}drones.sdf"

for ((index = 0; index < COUNT; index++)); do
    port=$((9002 + 10 * index))
    if ss -H -lun | grep -Eq ":${port}[[:space:]]"; then
        echo "[ERROR] Gazebo FDM port ${port} is already in use. Close the existing world first." >&2
        exit 1
    fi
done

echo "Launching ${COUNT}-drone Gazebo world: ${WORLD_FILE}"
echo "After Gazebo is ready: python3 -m sitl.swarm_console --drones ${COUNT} --gazebo --agent-runtime independent ..."
if [[ -n "${HEADLESS:-}" ]]; then
    gz sim -v3 -s -r "${WORLD_FILE}"
else
    gz sim -v3 -r "${WORLD_FILE}"
fi
