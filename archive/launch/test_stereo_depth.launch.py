# test_stereo_depth.launch.py
#
# Runs the two-topic stereo pipeline end-to-end:
#   camera_stereo  →  /camera_stereo/{left,right}/image_raw  →  stereo_depth  →  /stereo_depth/depth_color
#                                                          ↘
#                                                            preview (left, right, disparity windows)
#
# Verifies the stereo pipeline without bringing up the rest of VisionSense
# (no detect / lanedet / gui / dashboard). preview uses cv::imshow with one
# named window per topic.

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
        "config.yaml",
    )
    with open(config_path, "r") as fp:
        config = yaml.safe_load(fp)

    # Stereo camera — publishes /camera_stereo/{left,right}/image_raw
    # (each 1440×900 when rotated_lenses=false; 1200×1200 legacy when true).
    camera_stereo_node = Node(
        package="visionconnect",
        name="camera_stereo",
        executable="camera_stereo",
        output="screen",
        parameters=[config["camera_stereo"]["ros__parameters"]],
    )
    ld.add_action(camera_stereo_node)

    # Stereo depth — subscribes to the synchronized L/R pair, publishes
    # /stereo_depth/depth and /stereo_depth/depth_color.
    stereo_depth_node = Node(
        package="visionconnect",
        namespace="stereo_depth",
        name="stereo_depth",
        executable="stereo_depth",
        output="screen",
        parameters=[config["stereo_depth"]["ros__parameters"]],
        remappings=[
            ("left/image_raw",  "/camera_stereo/left/image_raw"),
            ("right/image_raw", "/camera_stereo/right/image_raw"),
        ],
    )
    ld.add_action(stereo_depth_node)

    # Preview — three OpenCV windows: left eye, right eye, colorized disparity.
    preview_node = Node(
        package="visionconnect",
        name="preview",
        executable="preview",
        output="screen",
        parameters=[config.get("preview", {}).get("ros__parameters", {})],
        remappings=[
            ("left/image_in",  "/camera_stereo/left/image_raw"),
            ("right/image_in", "/camera_stereo/right/image_raw"),
            ("disparity_in",   "/stereo_depth/depth_color"),
        ],
    )
    ld.add_action(preview_node)

    return ld
