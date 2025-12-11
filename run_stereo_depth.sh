#!/bin/bash
# Launch the CUDA-accelerated stereo depth system
#
# Components:
#  - camera_stereo: Captures left/right stereo images from Arducam
#  - stereo_depth: GPU-accelerated disparity computation (CUDA StereoSGM)
#  - image_view: Displays disparity output
#
# All stereo matching operations run on GPU for maximum performance

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch visionconnect stereo_depth_system.launch.py
