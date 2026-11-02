#!/bin/sh
# Run on Uno Q's VNC desktop. Close remote viewers first; no processes are killed.
set -eu
cd "$(dirname "$0")"
# Uno Q VNC does not support the tested DRI3/OpenGL path. Laptop uses its GPU.
export LIDAR_CPU_RENDER="${LIDAR_CPU_RENDER:-1}"
export DISPLAY="${DISPLAY:-:1}"
export PYTHONPATH="$HOME/lidar-python${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -B lidar_gui.py "$@"
