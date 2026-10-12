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
sudo apt-get install -y git curl ca-certificates rpicam-apps g++

if ! command -v node >/dev/null || [ "$(node -p 'process.versions.node.split(".")[0]')" -lt 20 ]; then
  echo "== Node.js $NODE_MAJOR (NodeSource)"
  curl -fsSL "https://deb.nodesource.com/setup_${NODE_MAJOR}.x" | sudo -E bash -
  sudo apt-get install -y nodejs
fi

echo "== Access to the Arduino's serial port and the camera"
sudo usermod -aG dialout,video "$USER_NAME"

echo "== Server dependencies"
(cd "$REPO/server" && npm ci --omit=dev)

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
sudo systemctl enable bailey
sudo systemctl restart bailey

IP="$(hostname -I | awk '{print $1}')"
echo
echo "Done. Open http://${IP:-<pi address>}:54321"
echo "Logs: journalctl -u bailey -f"
echo "If this was the first install, log out and in again (or reboot) for the serial port permission."
