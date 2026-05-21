# test_stereo_depth_monster.launch.py
#
# Minimal end-to-end test of the RT-MonSter++ stereo-depth node — isolates
# the monster pipeline from the rest of VisionSense (no detect / lanedet /
# classify / gui / dashboard / bev), so any slowdown you observe is in the
# camera→rectify→TRT→depth path itself, not GPU contention with everything
# else.
#
#   camera_stereo  →  /camera_stereo/{l,r}/{image_raw,camera_info}
#                            ↓ (calibrated K/D/R/P)
#                            stereo_depth_monster.py
#                            ↓
#                            /stereo_depth/{depth, depth_color}
#                            ↓
#                            preview (left, right, depth_color windows)
#
# Requires stereo_calib.yaml in config/ (compute_stereo_calib.py output)
# and the RT-MonSter++ engine at config-specified path.

import os
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config_dir = os.path.join(
        get_package_share_directory("visionconnect"),
        "config",
    )
    with open(os.path.join(config_dir, "config.yaml"), "r") as fp:
        config = yaml.safe_load(fp)

    # Resolve camera_stereo's calibration_file (just a filename in config.yaml)
    # against the installed config dir, so camera_stereo's CameraInfo carries
    # calibrated K/D/R/P and stereo_depth_monster gets the same calibration.
    cs_params = dict(config["camera_stereo"]["ros__parameters"])
    calib_name = cs_params.get("calibration_file", "")
    if calib_name:
        cs_params["calibration_file"] = os.path.join(config_dir, calib_name)

    camera_stereo_node = Node(
        package="visionconnect",
        name="camera_stereo",
        executable="camera_stereo",
        output="screen",
        parameters=[cs_params],
    )
    ld.add_action(camera_stereo_node)

    # MonSter Python node. calibration_file is the same absolute path the
    # camera node uses; engine_file_path comes from the stereo_depth_monster
    # config section.
    monster_params = dict(config.get("stereo_depth_monster", {}).get("ros__parameters", {}))
    monster_params["calibration_file"] = cs_params["calibration_file"]
    stereo_depth_monster_node = Node(
        package="visionconnect",
        namespace="stereo_depth",
        name="stereo_depth_monster",
        executable="stereo_depth_monster.py",
        output="screen",
        parameters=[monster_params],
        remappings=[
            ("left/image_raw",  "/camera_stereo/left/image_raw"),
            ("right/image_raw", "/camera_stereo/right/image_raw"),
        ],
    )
    ld.add_action(stereo_depth_monster_node)

    # Preview — three OpenCV windows (left, right, depth_color).
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
