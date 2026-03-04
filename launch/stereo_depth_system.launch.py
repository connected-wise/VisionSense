#!/usr/bin/env python3
"""
Stereo Depth System Launch File

Launches:
1. camera_stereo - Captures and publishes left/right rectified images
2. stereo_depth  - Computes depth from stereo disparity (TensorRT)
3. display       - Shows colorized depth map (OpenCV window)

Usage:
    ros2 launch visionconnect stereo_depth_system.launch.py
"""

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Load config
    config_path = os.path.join(
        get_package_share_directory("visionconnect"),
        "config",
        "config.yaml"
    )

    with open(config_path, "r") as fp:
        config = yaml.safe_load(fp)

    cam_params = config["camera_stereo"]["ros__parameters"]
    depth_params = config["stereo_depth"]["ros__parameters"]

    # Stereo Camera Node
    # Publishes to:
    #   /camera_stereo/left/image_raw
    #   /camera_stereo/right/image_raw
    camera_stereo_node = Node(
        package="visionconnect",
        name="camera_stereo",
        executable="camera_stereo",
        output="screen",
        parameters=[cam_params]
    )
    ld.add_action(camera_stereo_node)

    # Stereo Depth Node
    # Subscribes to:
    #   /camera_stereo/left/image_raw
    #   /camera_stereo/right/image_raw
    # Publishes to:
    #   /stereo_depth/depth         (32FC1, meters)
    #   /stereo_depth/depth_color   (bgr8, colorized)
    stereo_depth_node = Node(
        package="visionconnect",
        namespace="stereo_depth",
        name="stereo_depth",
        executable="stereo_depth",
        output="screen",
        remappings=[
            ("left/image_raw", "/camera_stereo/left/image_raw"),
            ("right/image_raw", "/camera_stereo/right/image_raw")
        ],
        parameters=[{
            **depth_params,
            "baseline_mm":      cam_params.get("baseline_mm", 100.0),
            "focal_length_mm":  cam_params.get("focal_length_mm", 3.6),
            "sensor_width_mm":  cam_params.get("sensor_width_mm", 5.76),
        }]
    )
    ld.add_action(stereo_depth_node)

    # Depth Display Node (lightweight OpenCV window)
    display_node = Node(
        package="visionconnect",
        executable="display_disparity.py",
        name="depth_display",
        output="screen",
        additional_env={"DISPLAY": ":0"},
    )
    ld.add_action(display_node)

    return ld
