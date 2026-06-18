#!/usr/bin/env bash
# Launch the DaMiao motor dashboard.
set -euo pipefail
cd "$(dirname "$0")"

# Point the DaMiao DM_Device SDK at the runtime fetched by
# `motorbridge-install-dm-device --download` (cached in ~/.cache).
LIB="$HOME/.cache/motorbridge/dm_device/v1.1.0/linux/x86_64/libdm_device.so"
if [ -f "$LIB" ]; then
  export MOTOR_DM_DEVICE_LIB="$LIB"
fi

export HOST="${HOST:-127.0.0.1}"
export PORT="${PORT:-5000}"

echo "Starting dashboard on http://$HOST:$PORT"
exec ./venv/bin/python app.py
