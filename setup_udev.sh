#!/usr/bin/env bash
# One-time setup: grant the DM-USB2FDCAN adapter (VID 34b7, PID 6632) to the
# 'plugdev' group so the dashboard can talk to it WITHOUT sudo.
#
# Run once:  ./setup_udev.sh
set -euo pipefail

RULE_FILE=/etc/udev/rules.d/99-damiao.rules
RULE='SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6632", MODE="0660", GROUP="plugdev"'

echo "Installing udev rule -> $RULE_FILE"
echo "$RULE" | sudo tee "$RULE_FILE" >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger --attr-match=idVendor=34b7

# Make sure your user is in plugdev (log out/in if this is newly added).
if ! id -nG "$USER" | grep -qw plugdev; then
  echo "Adding $USER to plugdev (re-login required to take effect)"
  sudo usermod -aG plugdev "$USER"
fi

echo "Done. If the device was already plugged in, replug it (or it should be"
echo "re-permissioned by the trigger above). Verify with:"
echo "    ls -l /dev/bus/usb/\$(lsusb | awk '/34b7:6632/{printf \"%s/%s\",\$2,substr(\$4,1,3)}')"
