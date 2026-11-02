#!/usr/bin/env bash
# Build, flash and verify the same ELF, then read completed diagnostics over SWD.
set -euo pipefail
exec python3 "$(dirname "$0")/check_sensors.py" "$@"
