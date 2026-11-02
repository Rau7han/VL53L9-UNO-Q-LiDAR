#!/bin/sh
# Sensor remains connected to Uno Q; all processing/rendering runs on this laptop.
set -eu
HOST="${1:-arduino@192.168.1.21}"
PORT="${LIDAR_PORT:-7000}"
cd "$(dirname "$0")"
# The dedicated relay lives in persistent storage and owns the sensor exclusively.
ssh -tt -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
  -L "127.0.0.1:$PORT:127.0.0.1:$PORT" "$HOST" \
  "PYTHONPATH=\$HOME/lidar-python python3 -B \$HOME/lidar-dashboard/relay_sensor.py --port $PORT" &
TUNNEL=$!
trap 'kill "$TUNNEL" 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM
sleep 1
kill -0 "$TUNNEL" 2>/dev/null || { echo 'SSH relay failed to start'; exit 1; }
python3 -B lidar_gui.py --usb "socket://127.0.0.1:$PORT" --log
