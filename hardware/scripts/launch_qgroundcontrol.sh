#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONSOLIDATED_DIR="$(cd "${PROJECT_DIR}/../.." && pwd)"
echo "Starting QGroundControl..."

if [ -n "$WAYLAND_DISPLAY" ]; then
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
fi

QGC_BIN="${QGC_BIN:-${CONSOLIDATED_DIR}/dependencies/QGroundControl/qgroundcontrol-app/AppRun}"
if [ -x "$QGC_BIN" ]; then
    exec "$QGC_BIN" "$@"
elif [ -x "${CONSOLIDATED_DIR}/dependencies/QGroundControl/QGroundControl.AppImage" ]; then
    exec "${CONSOLIDATED_DIR}/dependencies/QGroundControl/QGroundControl.AppImage" "$@"
else
    echo "ERROR: QGroundControl executable not found!" >&2
    exit 1
fi
