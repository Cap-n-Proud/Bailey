#!/usr/bin/env bash
# Installs the Bailey server on the robot's Raspberry Pi (Raspberry Pi OS
# Bookworm or later). Run from a clone of the repository, as the normal user:
#
#   git clone https://github.com/Cap-n-Proud/Bailey.git && cd Bailey && ./install/install.sh
#
# Safe to run again (e.g. after an update).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
USER_NAME="${SUDO_USER:-$USER}"
NODE_MAJOR=22

sed -n '/<<"EOT"/,/^EOT/p' "$REPO/install/ASCII_art.txt" 2>/dev/null | sed '1d;$d' | head -n 40 || true

echo "== System packages"
sudo apt-get update
sudo apt-get install -y git curl ca-certificates rpicam-apps g++ bluez

if ! command -v node >/dev/null || [ "$(node -p 'process.versions.node.split(".")[0]')" -lt 20 ]; then
  echo "== Node.js $NODE_MAJOR (NodeSource)"
  curl -fsSL "https://deb.nodesource.com/setup_${NODE_MAJOR}.x" | sudo -E bash -
  sudo apt-get install -y nodejs
fi

echo "== Access to the robot's serial port, the camera and a Bluetooth controller"
sudo usermod -aG dialout,video,input "$USER_NAME"

echo "== Server dependencies"
(cd "$REPO/server" && npm ci --omit=dev)

echo "== Video: MediaMTX (camera over WebRTC)"
# Pinned release, checked against the release's published SHA-256 sums.
MTX_VERSION=v1.21.0
case "$(uname -m)" in
  aarch64) MTX_ARCH=linux_arm64 ;;
  armv7l)  MTX_ARCH=linux_armv7 ;;
  armv6l)  MTX_ARCH=linux_armv6 ;;
  x86_64)  MTX_ARCH=linux_amd64 ;;
  *) MTX_ARCH="" ;;
esac
if [ -z "$MTX_ARCH" ]; then
  echo "no MediaMTX build for $(uname -m): set camera.mode to \"mjpeg\" in server/config.local.json"
elif ! /usr/local/bin/mediamtx --version 2>/dev/null | grep -q "$MTX_VERSION"; then
  MTX_TMP="$(mktemp -d)"
  MTX_FILE="mediamtx_${MTX_VERSION}_${MTX_ARCH}.tar.gz"
  MTX_URL="https://github.com/bluenviron/mediamtx/releases/download/$MTX_VERSION"
  curl -fsSL -o "$MTX_TMP/$MTX_FILE" "$MTX_URL/$MTX_FILE"
  curl -fsSL -o "$MTX_TMP/checksums.sha256" "$MTX_URL/checksums.sha256"
  (cd "$MTX_TMP" && grep "$MTX_FILE" checksums.sha256 | sha256sum --check --quiet)
  tar -xzf "$MTX_TMP/$MTX_FILE" -C "$MTX_TMP" mediamtx
  sudo install -m 0755 "$MTX_TMP/mediamtx" /usr/local/bin/mediamtx
  rm -rf "$MTX_TMP"
fi
if [ -n "$MTX_ARCH" ]; then
  sudo install -D -m 0644 "$REPO/install/mediamtx.yml" /etc/mediamtx/bailey.yml
  sed -e "s|@USER@|$USER_NAME|g" "$REPO/install/mediamtx.service" | sudo tee /etc/systemd/system/mediamtx.service >/dev/null
fi

echo "== Shutdown/reboot from the web UI (these two commands only, no password)"
SUDOERS=/etc/sudoers.d/bailey
TMP="$(mktemp)"
{
  echo "# Bailey web UI: allow $USER_NAME to shut down and reboot without a password"
  for sd in /sbin/shutdown /usr/sbin/shutdown; do
    echo "$USER_NAME ALL=(root) NOPASSWD: $sd -h now, $sd -r now"
  done
} > "$TMP"
sudo visudo -cf "$TMP" >/dev/null
sudo install -m 0440 "$TMP" "$SUDOERS"
rm -f "$TMP"

echo "== Service"
if [ -f /etc/init.d/robotserver ]; then
  echo "removing the old init.d robotserver service"
  sudo update-rc.d robotserver remove || true
  sudo rm -f /etc/init.d/robotserver
fi
sed -e "s|@USER@|$USER_NAME|g" -e "s|@REPO@|$REPO|g" "$REPO/install/bailey.service" \
  | sudo tee /etc/systemd/system/bailey.service >/dev/null
sudo systemctl daemon-reload
if [ -n "$MTX_ARCH" ]; then
  sudo systemctl enable mediamtx
  sudo systemctl restart mediamtx
fi
sudo systemctl enable bailey
sudo systemctl restart bailey

IP="$(hostname -I | awk '{print $1}')"
echo
echo "Done. Open http://${IP:-<pi address>}:54321"
echo "Logs: journalctl -u bailey -f"
echo "If this was the first install, log out and in again (or reboot) for the serial port permission."
echo "PS4/PS5 controller: ./install/pair-gamepad.sh"
