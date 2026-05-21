# test_stereo_depth_lightstereo.launch.py
#
# Minimal end-to-end test of the LightStereo-S stereo-depth node — isolates
# the pipeline from the rest of VisionSense (no detect / lanedet / classify /
# gui / dashboard / bev), so any slowdown observed here is in the
# camera→rectify→TRT→depth path, not GPU contention with the full stack.
#
#   camera_stereo  →  /camera_stereo/{l,r}/{image_raw,camera_info}
#                            ↓ (calibrated K/D/R/P)
#                            stereo_depth_lightstereo.py
#                            ↓
#                            /stereo_depth/{depth, depth_color}
#                            ↓
#                            preview (left, right, depth_color windows)
#
# Requires stereo_calib.yaml in config/ (compute_stereo_calib.py output)
# and the LightStereo engine at the config-specified path.

import os
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory("visionconnect")
    config_dir = os.path.join(pkg_share, "config")
    with open(os.path.join(config_dir, "config.yaml"), "r") as fp:
        config = yaml.safe_load(fp)

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

    # LightStereo Python node. engine_file_path in config is relative to
    # the installed graphs dir (src/graphs/stereo-depth); resolve to absolute
    # here so the node can open it as-is.
    ls_params = dict(config.get("stereo_depth_lightstereo", {}).get("ros__parameters", {}))
    ls_params["calibration_file"] = cs_params["calibration_file"]
    engine_name = ls_params.get("engine_file_path", "")
    if engine_name and not os.path.isabs(engine_name):
        ls_params["engine_file_path"] = os.path.join(pkg_share, "graphs", "stereo-depth", engine_name)

    stereo_depth_lightstereo_node = Node(
        package="visionconnect",
        namespace="stereo_depth",
        name="stereo_depth_lightstereo",
        executable="stereo_depth_lightstereo.py",
        output="screen",
        parameters=[ls_params],
        remappings=[
            ("left/image_raw",  "/camera_stereo/left/image_raw"),
            ("right/image_raw", "/camera_stereo/right/image_raw"),
        ],
    )
    ld.add_action(stereo_depth_lightstereo_node)

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
