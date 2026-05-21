#!/usr/bin/env bash
#
# Reset the Argus daemon to clear stale CSI/Argus state.
#
# When the camera node is killed mid-stream (Ctrl-C while Capture is blocked,
# pkill -9, OOM, etc.) nvargus-daemon force-destroys the CameraProvider after
# 1500 ms of pending requests and logs:
#
#   "Forced destruction will now proceed, which may leave the libargus
#    server in a bad state."
#
# In that bad state, subsequent launches of the camera node either fail to
# open the IMX219 or open it but stop receiving frames. A device reboot fixes
# it, but so does just restarting nvargus-daemon — much faster.
#
# Usage:
#   bash scripts/reset_camera.sh
#
# Requires sudo for the systemctl restart.

set -euo pipefail

if ! systemctl is-active --quiet nvargus-daemon; then
    echo "nvargus-daemon is not active — starting it."
    sudo systemctl start nvargus-daemon
    exit 0
fi

echo "Restarting nvargus-daemon to clear any stale Argus state..."
sudo systemctl restart nvargus-daemon

# Daemon takes ~1s to come back up and re-enumerate sensors.
for i in {1..10}; do
    if systemctl is-active --quiet nvargus-daemon; then
        echo "nvargus-daemon is up."
        exit 0
    fi
    sleep 0.5
done

echo "WARNING: nvargus-daemon did not come back up cleanly." >&2
exit 1
