#!/bin/bash
# VisionSense Launcher Script
# Launches the complete VisionSense autonomous vehicle vision system

# Set display for GUI
export DISPLAY=:0

# Fast-DDS shared-memory transport profile. Required to avoid right-eye packet
# loss when both /camera_stereo/{left,right}/image_raw (4.32 MB each @ 30 Hz)
# flow simultaneously. SHM is same-host only — for cross-host ROS2 switch to
# fastdds_profile.udp.xml (the UDP variant), which also needs the kernel buffer
# settings in /etc/sysctl.d/99-ros2-fastdds.conf.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/VisionSense/fastdds_profile.shm.xml

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the VisionSense workspace
cd /home/jetson/VisionSense
source install/setup.bash

# Launch VisionSense
echo "Starting VisionSense..."
ros2 launch visionconnect visionsense.launch.py
