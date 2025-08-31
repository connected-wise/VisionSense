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
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='Video device path'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='3200',
        description='Full stereo width (both cameras)'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1300',
        description='Camera height'
    )
    
    y16_format_arg = DeclareLaunchArgument(
        'y16_format',
        default_value='true',
        description='Use Y16 format (16-bit grayscale)'
    )
    
    use_gstreamer_arg = DeclareLaunchArgument(
        'use_gstreamer',
        default_value='false',
        description='Use GStreamer pipeline instead of V4L2'
    )
    
    # Stereo camera node
    stereo_camera_node = Node(
        package='visionconnect',
        executable='stereo_camera',
        name='stereo_camera',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'y16_format': LaunchConfiguration('y16_format'),
            'use_gstreamer': LaunchConfiguration('use_gstreamer'),
            'enable_left': True,
            'enable_right': True
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
    
    # Optional: Preview node to display stereo images
    preview_node = Node(
        package='visionconnect',
        executable='preview',
        name='preview_stereo',
        output='screen',
        parameters=[{
            'output': 'display://0',
            'width': 960,
            'height': 540
        }],
        remappings=[
            ('/camera/raw', '/stereo/left/image_raw')
        ]
    )
    
    return LaunchDescription([
        device_arg,
        width_arg,
        height_arg,
        y16_format_arg,
        use_gstreamer_arg,
        stereo_camera_node,
        # Uncomment to enable detection on both cameras
        # detect_left_node,
        # detect_right_node,
        # Uncomment to enable preview
        # preview_node
    ])