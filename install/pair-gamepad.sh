#!/usr/bin/env bash
# Pairs a PS4 (DualShock 4) or PS5 (DualSense) controller with the Raspberry Pi
# over Bluetooth. Only needed once: afterwards pressing PS reconnects it.
#
#   1. Put the controller in pairing mode: hold SHARE + PS (PS5: Create + PS)
#      until the light bar double-flashes.
#   2. ./install/pair-gamepad.sh            (or ./install/pair-gamepad.sh <MAC>)
#
# The Bailey server picks the controller up by itself; the web UI shows
# "pad: connected" at the top.
set -euo pipefail

MAC="${1:-}"

sudo rfkill unblock bluetooth 2>/dev/null || true
sudo systemctl enable --now bluetooth >/dev/null
bluetoothctl power on >/dev/null

if [ -z "$MAC" ]; then
  echo "Scanning for 20 s, controller in pairing mode..."
  bluetoothctl --timeout 20 scan on >/dev/null || true
  MAC="$(bluetoothctl devices | grep -Ei 'wireless controller|dualsense|dualshock' | awk '{print $2}' | head -n 1)"
  if [ -z "$MAC" ]; then
    echo "No controller found. Hold SHARE + PS until the light bar double-flashes and try again." >&2
    exit 1
  fi
fi

echo "Pairing $MAC"
bluetoothctl pair "$MAC"
bluetoothctl trust "$MAC"       # lets it reconnect by itself when PS is pressed
bluetoothctl connect "$MAC" || true

sleep 2
if grep -qiE 'wireless controller|dualsense' /sys/class/input/event*/device/name 2>/dev/null; then
  echo "Paired. The light bar stays on while connected."
  id -nG | grep -qw input || echo "Log out and in (or reboot) so the server can read it (\"input\" group)."
else
  echo "Paired, but no input device yet: press PS on the controller." >&2
fi
