#!/usr/bin/env python3

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config_path = os.path.join(
        get_package_share_directory("visionconnect"),
        "config",
        "config.yaml"
    )
    
    with open(config_path, "r") as fp:
        config = yaml.safe_load(fp)
    
    # Camera Node
    # Filter out None values from parameters
    camera_params = {k: v for k, v in config["camera"]["ros__parameters"].items() if v is not None}
    camera_node = Node(
        package="visionconnect",
        name="camera",
        executable="camera",
        output="screen",
        parameters=[camera_params]
    )
    ld.add_action(camera_node)
    
    # Object Detection Node
    detect_params = {k: v for k, v in config["detect"]["ros__parameters"].items() if v is not None}
    detect_node = Node(
        package="visionconnect",
        name="detect",
        executable="detect",
        output="screen",
        parameters=[detect_params],
        remappings=[
            ("/detect/image_in", "/camera/raw")
        ]
    )
    ld.add_action(detect_node)
    
    # Preview Node - Display output
    preview_params = {k: v for k, v in config["preview"]["ros__parameters"].items() if v is not None}
    preview_node = Node(
        package="visionconnect",
        name="preview",
        executable="preview",
        output="screen",
        parameters=[preview_params],
        remappings=[
            ("/preview/image_in", "/camera/raw"),
            ("/preview/detect_in", "/detect/detections")
        ]
    )
    ld.add_action(preview_node)
    
    return ld