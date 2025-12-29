#!/bin/bash
# VisionSense Launcher Script
# Launches the complete VisionSense autonomous vehicle vision system

# Set display for GUI
export DISPLAY=:0

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the VisionSense workspace
cd /home/jetson/VisionSense
source install/setup.bash

# Launch VisionSense
echo "Starting VisionSense..."
ros2 launch visionconnect visionsense.launch.py
