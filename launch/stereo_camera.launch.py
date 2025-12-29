#!/usr/bin/env python3
"""
Launch file for Arducam stereo camera with Y16 format support
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    resource_arg = DeclareLaunchArgument(
        'resource',
        default_value='/dev/video1',
        description='Video device path for Arducam stereo camera'
    )

    width_arg = DeclareLaunchArgument(
        'width',
        default_value='3840',
        description='Full stereo width (1920 left + 1920 right)'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1200',
        description='Camera height'
    )

    # Stereo camera node
    stereo_camera_node = Node(
        package='visionconnect',
        executable='camera_stereo',
        name='camera_stereo',
        output='screen',
        parameters=[{
            'resource': LaunchConfiguration('resource'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'framerate': 30,
            'flip': 'vertical',
            'latency': 0,       # Minimize GStreamer latency
            'num_buffers': 4    # Keep at 4 for stability under load
        }],
        remappings=[
            ('left/image_raw', '/stereo/left/image_raw'),
            ('right/image_raw', '/stereo/right/image_raw'),
            ('framerate', '/stereo/framerate'),
            ('timestamp', '/stereo/timestamp')
        ]
    )
    
    # Optional: Detection nodes for each camera
    detect_left_node = Node(
        package='visionconnect',
        executable='detect',
        name='detect_left',
        output='screen',
        remappings=[
            ('/camera/raw', '/stereo/left/image_raw'),
            ('/detect/detections', '/detect_left/detections'),
            ('/detect/signs', '/detect_left/signs')
        ]
    )
    
    detect_right_node = Node(
        package='visionconnect',
        executable='detect',
        name='detect_right',
        output='screen',
        remappings=[
            ('/camera/raw', '/stereo/right/image_raw'),
            ('/detect/detections', '/detect_right/detections'),
            ('/detect/signs', '/detect_right/signs')
        ]
    )
    
    # Preview node to display both stereo cameras
    preview_node = Node(
        package='visionconnect',
        executable='preview',
        name='preview_stereo',
        output='screen',
        additional_env={"DISPLAY": ":0"},
        parameters=[{
            'output': 'display://0',
            'width': 960,
            'height': 540
        }],
        remappings=[
            ('left/image_in', '/stereo/left/image_raw'),
            ('right/image_in', '/stereo/right/image_raw')
        ]
    )
    
    return LaunchDescription([
        resource_arg,
        width_arg,
        height_arg,
        stereo_camera_node,
        preview_node,
        # Uncomment to enable detection on both cameras
        # detect_left_node,
        # detect_right_node,
    ])
