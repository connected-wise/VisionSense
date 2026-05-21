# test_stereo_depth_ess.launch.py
#
# A/B counterpart to test_stereo_depth.launch.py — runs the Isaac ROS NITROS
# ESS pipeline (with proper NITROS RectifyNode in front) instead of FFS.
#
#   camera_stereo  →  /camera_stereo/{l,r}/{image_raw,camera_info}
#                                  ↓                (calibrated K/D/R/P from `calibration_file`)
#                                  RectifyNode → ResizeNode → ESS → DisparityToDepth → depth_colorize
#                                                                                       ↓
#                                                                  /stereo_depth/depth + depth_color
#                                  ↓
#                                  preview (left, right, depth_color windows)
#
# Run scripts/capture_stereo_calib.py + scripts/compute_stereo_calib.py first
# to produce the calibration YAML; this launch then points camera_stereo at it.

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    config_dir = os.path.join(
        get_package_share_directory('visionconnect'),
        'config',
    )
    with open(os.path.join(config_dir, 'config.yaml'), 'r') as fp:
        config = yaml.safe_load(fp)

    # Resolve camera_stereo's calibration_file (a filename in config.yaml) to
    # the installed config dir. RectifyNode in the ESS pipeline needs the
    # resulting K/D/R/P in CameraInfo; empty string falls back to focal math.
    cs_params = dict(config['camera_stereo']['ros__parameters'])
    calib_name = cs_params.get('calibration_file', '')
    if calib_name:
        cs_params['calibration_file'] = os.path.join(config_dir, calib_name)

    # Stereo camera — CameraInfo carries calibrated K/D/R/P, required for ESS.
    camera_stereo_node = Node(
        package='visionconnect',
        name='camera_stereo',
        executable='camera_stereo',
        output='screen',
        parameters=[cs_params],
    )
    ld.add_action(camera_stereo_node)

    # ESS pipeline — rectify + resize + ESS + disparity_to_depth + colorize.
    ess_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('visionconnect'),
                'launch', 'stereo_depth_ess.launch.py',
            )
        )
    )
    ld.add_action(ess_pipeline)

    # Preview — left / right / depth_color windows (cv::imshow).
    preview_node = Node(
        package='visionconnect',
        name='preview',
        executable='preview',
        output='screen',
        parameters=[config.get('preview', {}).get('ros__parameters', {})],
        remappings=[
            ('left/image_in',  '/camera_stereo/left/image_raw'),
            ('right/image_in', '/camera_stereo/right/image_raw'),
            ('disparity_in',   '/stereo_depth/depth_color'),
        ],
    )
    ld.add_action(preview_node)

    return ld
