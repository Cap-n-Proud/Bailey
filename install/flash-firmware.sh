#!/usr/bin/env bash
# Builds the firmware and uploads it to the Arduino Mega over USB, from the Pi.
#
#   ./install/flash-firmware.sh [serial port]      (default /dev/ttyACM0)
#
# Installs arduino-cli, the AVR core and the libraries the first time. The
# Bailey server is stopped during the upload because it holds the serial port.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${1:-/dev/ttyACM0}"
FQBN="arduino:avr:mega:cpu=atmega2560"
BIN="$HOME/.local/bin"
export PATH="$BIN:$PATH"

if ! command -v arduino-cli >/dev/null; then
  echo "== Installing arduino-cli"
  mkdir -p "$BIN"
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$BIN" sh
fi

echo "== AVR core and libraries"
arduino-cli core update-index
arduino-cli core install arduino:avr
# Versions the firmware was built and tested with.
arduino-cli lib install "MPU6050@1.4.5" "FastAccelStepper@2.0.0" "Encoder@1.4.4"

echo "== Compiling"
arduino-cli compile --fqbn "$FQBN" "$REPO/bailey_FW"

echo "== Uploading to $PORT"
restart=false
if systemctl is-active --quiet bailey; then
  sudo systemctl stop bailey
  restart=true
fi
trap '$restart && sudo systemctl start bailey' EXIT
arduino-cli upload --fqbn "$FQBN" -p "$PORT" "$REPO/bailey_FW"
echo "Done. Settings in EEPROM are kept unless the settings layout changed (then defaults are loaded)."
