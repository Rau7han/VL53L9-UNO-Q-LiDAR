#!/usr/bin/env bash
# USB DFU flash/verify/start of current firmware; optional image and --port USB1.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$HERE/flash-usb.py" "$@"
