#!/bin/bash
# VisionSense Launch Script
# This script launches the VisionSense autonomous vehicle vision system

# Set up terminal window title
echo -ne "\033]0;VisionSense\007"

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

# Launch the main launch file
ros2 launch visionconnect visionsense.launch.py

# Keep terminal open on exit
echo ""
echo "VisionSense has stopped."
read -p "Press Enter to close this window..."
