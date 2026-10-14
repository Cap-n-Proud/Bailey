#!/usr/bin/env bash
# Builds the firmware and uploads it over USB, from the Pi.
#
#   ./install/flash-firmware.sh [serial port]
#
# For the Waveshare ESP32-S3-Pico or any ESP32-S3 board; the port defaults to
# /dev/ttyACM0. (Firmware 3 is ESP32-S3 only: the Arduino Mega can't build
# Eigen and TinyMPC.)
#
# Installs arduino-cli, the board core and the libraries the first time. The
# Bailey server is stopped during the upload because it holds the serial port.
#
# ESP32-S3: no programmer needed, it uploads over its own USB port. If an upload
# fails (first time, or after a crash), hold BOOT, press RESET (or replug the
# cable), release BOOT and run this again.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${1:-/dev/ttyACM0}"
BIN="$HOME/.local/bin"
export PATH="$BIN:$PATH"

CORE="esp32:esp32"
# USB CDC On Boot: the firmware's Serial is the USB port the Pi talks to.
# Flash/PSRAM match the ESP32-S3-Pico (16 MB flash, 2 MB quad PSRAM).
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc,USBMode=hwcdc,FlashSize=16M,PSRAM=enabled"

if ! command -v arduino-cli >/dev/null; then
  echo "== Installing arduino-cli"
  mkdir -p "$BIN"
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$BIN" sh
fi

echo "== Board core ($CORE) and libraries"
arduino-cli config init --overwrite >/dev/null 2>&1 || true
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json >/dev/null 2>&1 || true
arduino-cli core update-index
arduino-cli core install "$CORE"
# Versions the firmware was built and tested with. "Eigen" is hideakitai's
# ArduinoEigen (Eigen 3.4.0); it pulls in ArxTypeTraits. TinyMPC is in the
# sketch (bailey_FW/src/tinympc).
arduino-cli lib install "MPU6050@1.4.5" "FastAccelStepper@2.0.0" "Eigen@0.3.2"

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
