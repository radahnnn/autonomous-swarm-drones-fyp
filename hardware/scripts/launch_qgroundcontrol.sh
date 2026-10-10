#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Starting QGroundControl..."

if [ -n "$WAYLAND_DISPLAY" ]; then
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
fi

QGC_BIN="/home/drone/Documents/gemini work/qgroundcontrol-app/AppRun"
if [ -x "$QGC_BIN" ]; then
    exec "$QGC_BIN" "$@"
elif [ -x "$SCRIPT_DIR/qgroundcontrol-app/AppRun" ]; then
    exec "$SCRIPT_DIR/qgroundcontrol-app/AppRun" "$@"
elif [ -x "/home/drone/Documents/gemini work/QGroundControl.AppImage" ]; then
    exec "/home/drone/Documents/gemini work/QGroundControl.AppImage" "$@"
else
    echo "ERROR: QGroundControl executable not found!" >&2
    exit 1
fi
