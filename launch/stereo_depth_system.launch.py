#!/usr/bin/env python3
"""
Stereo Depth System Launch File

Launches:
1. camera_stereo - Captures and publishes left/right rectified images
2. stereo_depth - Subscribes to images and computes depth map
3. preview - Displays the disparity image output
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

    # Stereo Camera Node
    # Publishes to:
    #   /camera_stereo/left/image_raw
    #   /camera_stereo/right/image_raw
    camera_stereo_node = Node(
        package="visionconnect",
        name="camera_stereo",
        executable="camera_stereo",
        output="screen",
        parameters=[config["camera_stereo"]["ros__parameters"]]
    )
    ld.add_action(camera_stereo_node)

    # Stereo Depth Node
    # Subscribes to:
    #   /camera_stereo/left/image_raw
    #   /camera_stereo/right/image_raw
    # Publishes to:
    #   /stereo_depth/disparity
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
            "min_disparity": 0,
            "num_disparities": 64,
            "block_size": 9,
            "p1_multiplier": 8,
            "p2_multiplier": 32,
            "uniqueness_ratio": 10,
            "median_filter_size": 5
        }]
    )
    ld.add_action(stereo_depth_node)

    # Preview Node for Disparity Visualization
    # Subscribe to disparity and display
    preview_node = Node(
        package="visionconnect",
        name="disparity_preview",
        executable="preview",
        output="screen",
        parameters=[config["preview"]["ros__parameters"]],
        remappings=[
            ("disparity_in", "/stereo_depth/disparity")
        ]
    )
    ld.add_action(preview_node)

    return ld
