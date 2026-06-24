#!/usr/bin/env bash
# Launch the OpenArm bimanual teleoperation dashboard.
#
# Hardware: two OpenArm units (4 arms total) on two dual-channel gs_usb CAN-FD
# adapters -> can0/can1 (unit A) and can2/can3 (unit B). The dashboard brings the
# buses up (sudo), shows which motors are connected, and runs unilateral
# leader -> follower teleop. Always run inside the project venv.
set -euo pipefail
cd "$(dirname "$0")"

export HOST="${HOST:-127.0.0.1}"
export PORT="${PORT:-5002}"
export OA_RATE_HZ="${OA_RATE_HZ:-120}"

PY="${OA_PYTHON:-./venv/bin/python}"

# The dashboard runs `sudo ip link ...` to bring CAN buses up. For that to work
# without a password prompt, allow passwordless sudo for `ip` once, e.g.:
#   echo "$USER ALL=(root) NOPASSWD: /usr/sbin/ip, /sbin/ip, /bin/ip" \
#     | sudo tee /etc/sudoers.d/openarm-can
# (Otherwise launch this script itself with sudo, or bring buses up by hand.)
if ! sudo -n true 2>/dev/null; then
  echo "NOTE: passwordless sudo is not available. The 'Bring up all buses'"
  echo "      button will fail. Either add a sudoers rule for 'ip' (see"
  echo "      run_openarm.sh), or pre-bring-up the buses, e.g.:"
  for c in can0 can1 can2 can3; do
    echo "        sudo ip link set $c type can bitrate 1000000 dbitrate 5000000 fd on && sudo ip link set $c up"
  done
fi

echo "Using interpreter: $PY"
echo "Starting OpenArm teleop dashboard on http://$HOST:$PORT"
exec "$PY" oa_dashboard.py
