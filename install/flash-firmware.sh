#!/usr/bin/env bash
# Builds the firmware and uploads it over USB, from the Pi.
#
#   ./install/flash-firmware.sh [esp32s3|mega] [serial port]
#
#   esp32s3  Waveshare ESP32-S3-Pico or any ESP32-S3 board (default)
#   mega     Arduino Mega 2560 (the original board)
#   port     default /dev/ttyACM0
#
# Installs arduino-cli, the board core and the libraries the first time. The
# Bailey server is stopped during the upload because it holds the serial port.
#
# ESP32-S3: no programmer needed, it uploads over its own USB port. If an upload
# fails (first time, or after a crash), hold BOOT, press RESET (or replug the
# cable), release BOOT and run this again.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
BOARD="${1:-esp32s3}"
PORT="${2:-/dev/ttyACM0}"
BIN="$HOME/.local/bin"
export PATH="$BIN:$PATH"

case "$BOARD" in
  esp32s3)
    CORE="esp32:esp32"
    # USB CDC On Boot: the firmware's Serial is the USB port the Pi talks to.
    # Flash/PSRAM match the ESP32-S3-Pico (16 MB flash, 2 MB quad PSRAM).
    FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc,USBMode=hwcdc,FlashSize=16M,PSRAM=enabled"
    ;;
  mega)
    CORE="arduino:avr"
    FQBN="arduino:avr:mega:cpu=atmega2560"
    ;;
  *)
    echo "unknown board '$BOARD' (use esp32s3 or mega)" >&2
    exit 2
    ;;
esac

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
# Versions the firmware was built and tested with.
arduino-cli lib install "MPU6050@1.4.5" "FastAccelStepper@2.0.0"
[ "$BOARD" = mega ] && arduino-cli lib install "Encoder@1.4.4"

echo "== Compiling for $BOARD"
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
