#!/usr/bin/env bash
# Launch the DaMiao motor dashboard.
#
# The CAN adapter on this machine is a candleLight/gs_usb device (USB id
# 1d50:606f) which the kernel exposes as SocketCAN interfaces (can0/can1), NOT a
# DaMiao DM-USB2FDCAN in USB mode (vendor 34b7). So the dashboard talks to it
# through motorbridge's SocketCAN-FD transport. Override DM_DEVICE_TYPE /
# DM_CHANNEL if you actually have a DaMiao USB adapter instead.
set -euo pipefail
cd "$(dirname "$0")"

# Transport: SocketCAN-FD over the gs_usb adapter, scanning both channels.
export DM_DEVICE_TYPE="${DM_DEVICE_TYPE:-socketcanfd}"
export DM_CHANNEL="${DM_CHANNEL:-can0,can1}"
# CAN-FD bitrates (DaMiao default: 1 Mbit nominal / 5 Mbit data).
CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_DBITRATE="${CAN_DBITRATE:-5000000}"

# Point the DaMiao DM_Device SDK at the runtime fetched by
# `motorbridge-install-dm-device --download` (cached in ~/.cache). Only needed
# for the DM_Device (USB-mode) transport, harmless otherwise.
LIB="$HOME/.cache/motorbridge/dm_device/v1.1.0/linux/x86_64/libdm_device.so"
if [ -f "$LIB" ]; then
  export MOTOR_DM_DEVICE_LIB="$LIB"
fi

# Bring any SocketCAN interface we intend to use UP first. A SocketCAN bus must
# be configured + UP before motorbridge can open it; doing it here means a fresh
# plug-in "just works" without remembering the ip-link incantation.
if [ "${DM_DEVICE_TYPE}" = "socketcanfd" ] || [ "${DM_DEVICE_TYPE}" = "socketcan" ]; then
  IFS=',' read -ra _CAN_IFACES <<< "$DM_CHANNEL"
  for iface in "${_CAN_IFACES[@]}"; do
    iface="$(echo "$iface" | tr -d '[:space:]')"
    [ -z "$iface" ] && continue
    if ! ip link show "$iface" >/dev/null 2>&1; then
      echo "WARNING: SocketCAN interface '$iface' not found (is the USB-CAN adapter plugged in?)" >&2
      continue
    fi
    if ip link show "$iface" | grep -q "state UP"; then
      continue
    fi
    echo "Bringing up $iface (bitrate ${CAN_BITRATE}, dbitrate ${CAN_DBITRATE}, fd on)..."
    sudo ip link set "$iface" type can bitrate "$CAN_BITRATE" \
      dbitrate "$CAN_DBITRATE" fd on
    sudo ip link set "$iface" up
  done
fi

export HOST="${HOST:-127.0.0.1}"
export PORT="${PORT:-5000}"

echo "Transport: $DM_DEVICE_TYPE  channel(s): $DM_CHANNEL"
echo "Starting dashboard on http://$HOST:$PORT"
exec ./venv/bin/python app.py
