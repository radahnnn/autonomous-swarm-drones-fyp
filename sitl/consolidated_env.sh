#!/usr/bin/env bash
# Resolve simulator dependencies from the consolidated folder while keeping
# optional environment overrides available for other machines.
_FYP_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FYP_PROJECT_DIR="$(cd "${_FYP_SCRIPT_DIR}/.." && pwd)"
export FYP_CONSOLIDATED_DIR="$(cd "${FYP_PROJECT_DIR}/../.." && pwd)"

export ARDUPILOT_HOME="${ARDUPILOT_HOME:-${FYP_CONSOLIDATED_DIR}/dependencies/ardupilot}"
export ARDUCOPTER_BIN="${ARDUCOPTER_BIN:-${ARDUPILOT_HOME}/build/sitl/bin/arducopter}"
export ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${FYP_CONSOLIDATED_DIR}/dependencies/ardupilot_gazebo}"
export QGC_LAUNCHER="${QGC_LAUNCHER:-${FYP_CONSOLIDATED_DIR}/dependencies/QGroundControl/launch_qgroundcontrol.sh}"

if [[ -x "${FYP_PROJECT_DIR}/.venv/bin/python" ]]; then
    export VENV_PYTHON="${VENV_PYTHON:-${FYP_PROJECT_DIR}/.venv/bin/python}"
    export VENV_ACTIVATE="${VENV_ACTIVATE:-${FYP_PROJECT_DIR}/.venv/bin/activate}"
elif [[ -x "$HOME/venv-ardupilot/bin/python3" ]]; then
    export VENV_PYTHON="${VENV_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
    export VENV_ACTIVATE="${VENV_ACTIVATE:-$HOME/venv-ardupilot/bin/activate}"
fi

unset _FYP_SCRIPT_DIR
