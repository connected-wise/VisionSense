"""
Test launch file for stereo depth node with real camera.
Launches stereo camera, stereo_depth processing, and disparity display.

Usage:
    ros2 launch visionconnect test_stereo_depth.launch.py

For file-based testing, use:
    ros2 launch visionconnect test_stereo_depth.launch.py use_files:=true
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    pkg_share = get_package_share_directory('visionconnect')

    # Load config
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')
    with open(config_file, 'r') as fp:
        config = yaml.safe_load(fp)

    # Test images paths
    testing_dir = '/home/jetson/VisionSense/testing'
    left_image = os.path.join(testing_dir, '000003-left.png')
    right_image = os.path.join(testing_dir, '000003-right.png')

    # Launch argument for choosing file vs camera mode
    use_files_arg = DeclareLaunchArgument(
        'use_files',
        default_value='false',
        description='Use file images instead of camera (true/false)'
    )

    # Stereo Camera Node (real camera)
    camera_stereo_node = Node(
        package='visionconnect',
        executable='camera_stereo',
        name='camera_stereo',
        output='screen',
        parameters=[config['camera_stereo']['ros__parameters']],
        condition=UnlessCondition(LaunchConfiguration('use_files'))
    )

    # File publisher node (for testing with images)
    file_publisher_node = Node(
        package='visionconnect',
        executable='test_stereo_from_files.py',
        name='stereo_file_publisher',
        namespace='camera_stereo',
        parameters=[{
            'left_image': left_image,
            'right_image': right_image,
            'rate': 2.0,
            'encoding': 'rgb8',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_files'))
    )

    # Stereo depth node
    stereo_depth_node = Node(
        package='visionconnect',
        executable='stereo_depth',
        name='stereo_depth',
        namespace='stereo_depth',
        remappings=[
            ('left/image_raw', '/camera_stereo/left/image_raw'),
            ('right/image_raw', '/camera_stereo/right/image_raw'),
        ],
        parameters=[config['stereo_depth']['ros__parameters']],
        output='screen',
    )

    # Disparity display node
    disparity_display_node = Node(
        package='visionconnect',
        executable='display_disparity.py',
        name='disparity_display',
        output='screen',
    )

    return LaunchDescription([
        use_files_arg,
        camera_stereo_node,
        file_publisher_node,
        stereo_depth_node,
        disparity_display_node,
    ])
