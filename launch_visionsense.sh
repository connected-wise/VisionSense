#!/bin/bash
# VisionSense Launch Script
# This script launches the VisionSense autonomous vehicle vision system

# Set up terminal window title
echo -ne "\033]0;VisionSense\007"

# Make sure GTK / Qt subprocesses (gui, preview, etc.) inherit a usable X11
# session. When this script runs from a .desktop launcher the parent env
# already has DISPLAY+XAUTHORITY; when it runs from a fresh terminal or via
# `sudo`, XAUTHORITY may be missing — node_gui then dies with
#   OpenCV error: Can't initialize GTK backend in function 'cvInitSystem'
# Setting reasonable defaults here is harmless if the values are already set.
export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ]; then
    # Try the standard locations in order of likelihood.
    for xauth in "$HOME/.Xauthority" "/run/user/$(id -u)/gdm/Xauthority" "/run/user/$(id -u)/mutter/Xauthority"; do
        if [ -f "$xauth" ]; then
            export XAUTHORITY="$xauth"
            break
        fi
    done
fi

# FastDDS shared-memory transport profile. Required to avoid right-eye packet
# loss when both /camera_stereo/{left,right}/image_raw (4.32 MB each @ 30 Hz)
# flow simultaneously — the default 512 KB SHM segment can't hold two stereo
# images, and Humble's FastDDS (2.6.x) does NOT support the `LARGE_DATA?...`
# query-string env-var syntax used by newer ROS2 distros (logs "Wrong value
# for environment variable 'FASTDDS_BUILTIN_TRANSPORTS'" and falls back to
# DEFAULT, which is exactly the bug we're trying to avoid).
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/VisionSense/fastdds_profile.shm.xml

# Stale /dev/shm/fastrtps_* segments from previously-killed runs choke DDS
# discovery — subscribers log "ready" but their callbacks never fire. Wipe
# them when nothing is currently running so this launch starts clean.
if ! pgrep -af "lib/visionconnect|stereo_depth_lightstereo" >/dev/null 2>&1; then
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
fi

# Change to workspace directory
cd /home/jetson/VisionSense

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch VisionSense
echo "================================================"
echo "         VisionSense Vision System"
echo "================================================"
echo ""
echo "Starting VisionSense nodes..."
echo "  - Camera (stereo auto-crop)"
echo "  - Object Detection"
echo "  - Traffic Sign Classification"
echo "  - Lane Detection"
echo "  - ADAS (Advanced Driver Assistance)"
echo "  - Dashboard (Web UI at http://localhost:8080)"
echo "  - GUI (Data Fusion)"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "================================================"
echo ""

# Re-kick the persistent IMX219 service once VisionSense subscribers come up.
# Reason: with the SHM-only FastDDS profile, a publisher that started before
# any subscribers existed (which is what visionsense-imx219.service is doing
# from boot) is not always rediscovered by late-joining subscribers. Restarting
# the service after `ros2 launch` is up forces a fresh participant
# announcement that the now-existing subscribers do see. Passwordless sudo
# for this exact command is configured in /etc/sudoers.d/visionsense-imx219.
(
    for _ in $(seq 1 30); do
        if pgrep -f "driver_monitor|camera_stereo" >/dev/null 2>&1; then
            sleep 3
            sudo -n systemctl restart visionsense-imx219.service 2>&1 \
                | sed 's/^/[imx219-restart] /' || true
            break
        fi
        sleep 1
    done
) &
IMX219_KICK_PID=$!
trap 'kill $IMX219_KICK_PID 2>/dev/null; wait $IMX219_KICK_PID 2>/dev/null' EXIT

# Launch the main launch file. Window closes cleanly on exit.
ros2 launch visionconnect visionsense.launch.py
