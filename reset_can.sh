#!/usr/bin/env bash
# Recover a wedged SocketCAN / gs_usb (DM-USB2FDCAN) adapter.
#
# Symptom: an arm suddenly shows all motors "offline" and the bus stops moving
# frames (TX/RX counters frozen) even though the interface still reads "UP".
# This is usually a soft firmware wedge after an ENOBUFS burst. Cycling the
# interface down -> reconfigure -> up almost always recovers it without a
# physical USB replug.
#
# Usage:
#   ./reset_can.sh                # reset can0 and can1 (default)
#   ./reset_can.sh can0           # reset just one bus
#   ./reset_can.sh can0 can1 can2 can3   # reset several
#
# After running this, restart the dashboard (./run.sh) and re-scan; if the
# motors still don't show up, physically unplug/replug the USB-CAN adapter.
set -uo pipefail

CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_DBITRATE="${CAN_DBITRATE:-5000000}"
CAN_TXQUEUELEN="${CAN_TXQUEUELEN:-1000}"

# Default to both buses on the single adapter if none are given.
IFACES=("$@")
if [ "${#IFACES[@]}" -eq 0 ]; then
  IFACES=(can0 can1)
fi

for iface in "${IFACES[@]}"; do
  if ! ip link show "$iface" >/dev/null 2>&1; then
    echo "WARNING: '$iface' not found (is the USB-CAN adapter plugged in?) -- skipping." >&2
    continue
  fi

  echo "Resetting $iface ..."
  sudo ip link set "$iface" down 2>/dev/null || true
  sleep 0.3

  # Re-apply CAN-FD config; fall back to classic CAN if FD isn't supported.
  if ! sudo ip link set "$iface" type can bitrate "$CAN_BITRATE" \
       dbitrate "$CAN_DBITRATE" fd on 2>/dev/null; then
    echo "  CAN-FD config failed on $iface; trying classic CAN (no FD)..." >&2
    sudo ip link set "$iface" type can bitrate "$CAN_BITRATE" 2>/dev/null || \
      echo "  WARNING: could not configure $iface." >&2
  fi

  # Bigger TX queue avoids ENOBUFS ("No buffer space available") under bursts.
  sudo ip link set "$iface" txqueuelen "$CAN_TXQUEUELEN" 2>/dev/null || true

  if sudo ip link set "$iface" up 2>/dev/null; then
    state="$(ip -details link show "$iface" | grep -oE 'state [A-Z-]+' | head -1)"
    echo "  $iface up ($state)"
  else
    echo "  WARNING: could not bring $iface up." >&2
  fi
done

echo
echo "Done. Now restart the dashboard (./run.sh) and re-scan."
echo "If an arm is still missing, unplug and replug the USB-CAN adapter."
