#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Starting QGroundControl..."

if [ -n "$WAYLAND_DISPLAY" ]; then
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
fi

exec "$SCRIPT_DIR/qgroundcontrol-app/AppRun" "$@"
