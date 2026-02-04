# test_camera_latency.launch.py
# Minimal launch file to test camera-to-display latency
# Only runs: stereo camera (no processing nodes)
#
# Usage:
#   Terminal 1: ros2 launch visionconnect test_camera_latency.launch.py
#   Terminal 2: python3 scripts/measure_latency.py
#   Terminal 3: python3 scripts/simple_display.py /camera_stereo/left/image_raw
#
# This isolates camera capture latency from processing latency.

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

    # Stereo Camera only - minimal configuration
    camera_stereo_node = Node(
        package="visionconnect",
        name="camera_stereo",
        executable="camera_stereo",
        output="screen",
        parameters=[config["camera_stereo"]["ros__parameters"]]
    )
    ld.add_action(camera_stereo_node)

    # Preview node using jetson-utils display
    preview_node = Node(
        package="visionconnect",
        name="preview",
        executable="preview",
        output="screen",
        parameters=[config.get("preview", {}).get("ros__parameters", {})],
        remappings=[
            ("left/image_in", "/camera_stereo/left/image_raw"),
            ("right/image_in", "/camera_stereo/right/image_raw")
        ]
    )
    ld.add_action(preview_node)

    return ld
