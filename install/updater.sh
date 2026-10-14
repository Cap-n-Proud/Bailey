#!/usr/bin/env bash
# Updates the Bailey server from git and restarts it. The firmware is updated
# separately with install/flash-firmware.sh.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
before="$(git rev-parse HEAD)"
git pull --ff-only
if [ "$before" = "$(git rev-parse HEAD)" ]; then
  echo "Already up to date."
  exit 0
fi
(cd server && npm ci --omit=dev)
sudo systemctl restart bailey
echo "Updated to $(git log -1 --format='%h %s')."
if ! git diff --quiet "$before" HEAD -- install; then
  echo "The installation changed (services, MediaMTX): run install/install.sh again."
fi
if ! git diff --quiet "$before" HEAD -- bailey_FW; then
  echo "The firmware changed too: run install/flash-firmware.sh to update the ESP32-S3."
fi
