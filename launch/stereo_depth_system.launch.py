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
        parameters=[config["stereo_depth"]["ros__parameters"]]
    )
    ld.add_action(stereo_depth_node)

    # Preview Node for Stereo Cameras and Disparity
    preview_node = Node(
        package="visionconnect",
        name="preview_stereo",
        executable="preview",
        output="screen",
        additional_env={"DISPLAY": ":0"},
        parameters=[{
            'output': 'display://0'
        }],
        remappings=[
            ('left/image_in', '/camera_stereo/left/image_raw'),
            ('right/image_in', '/camera_stereo/right/image_raw'),
            ('disparity_in', '/stereo_depth/disparity')
        ]
    )
    ld.add_action(preview_node)

    return ld
