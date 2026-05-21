# stereo_depth_ess.launch.py
#
# Isaac ROS NITROS ESS stereo-depth pipeline, now with proper rectification
# via NITROS RectifyNode in front of the resize+ESS chain. This makes the
# pipeline geometrically correct as long as camera_stereo is publishing
# CameraInfo with calibrated K/D/R/P (set its `calibration_file` parameter
# to the YAML produced by scripts/compute_stereo_calib.py).
#
# Pipeline (all composable, single ComponentContainerMt, IPC zero-copy):
#
#   /camera_stereo/{left,right}/image_raw     (1440x900 rgb8)
#   /camera_stereo/{left,right}/camera_info   (CameraInfo with calibrated K/D/R/P)
#                       │
#                       ▼  RectifyNode (per eye)
#   /stereo_depth/{left,right}/image_rect     (rectified, undistorted)
#   /stereo_depth/{left,right}/camera_info_rect  (K = P[:3,:3], D = zeros)
#                       │
#                       ▼  ResizeNode (rectified → 960x576, rescales K/P)
#   /stereo_depth/{left,right}/image_resize
#   /stereo_depth/{left,right}/camera_info_resize
#                       │
#                       ▼  ESSDisparityNode  (full ESS, fp16 engine, threshold)
#   /stereo_depth/disparity                   (stereo_msgs/DisparityImage)
#                       │
#                       ▼  DisparityToDepthNode
#   /stereo_depth/depth                       (sensor_msgs/Image 32FC1, m)
#                       │
#                       ▼  depth_colorize (small Python helper)
#   /stereo_depth/depth_color                 (sensor_msgs/Image bgr8, TURBO)

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


DEFAULT_ESS_ENGINE = (
    '/home/jetson/isaac_ros_assets/models/dnn_stereo_disparity/'
    'dnn_stereo_disparity_v4.1.0_onnx/ess.engine'
)


def generate_launch_description():
    engine_file_path   = LaunchConfiguration('engine_file_path')
    threshold          = LaunchConfiguration('threshold')
    input_layer_width  = LaunchConfiguration('input_layer_width')
    input_layer_height = LaunchConfiguration('input_layer_height')

    decl_engine = DeclareLaunchArgument(
        'engine_file_path', default_value=DEFAULT_ESS_ENGINE,
        description='Absolute path to the ESS .engine file built by install_ess_models.sh',
    )
    decl_threshold = DeclareLaunchArgument(
        'threshold', default_value='0.4',
        description='ESS confidence threshold (disparity below this confidence -> 0)',
    )
    decl_in_w = DeclareLaunchArgument('input_layer_width',  default_value='960')
    decl_in_h = DeclareLaunchArgument('input_layer_height', default_value='576')

    # ---- Rectify (per eye) --------------------------------------------------
    # Subscribes to /camera_stereo/{l,r}/{image_raw,camera_info}. Uses K + D + R
    # from camera_info to undistort + rectify; publishes /stereo_depth/{l,r}/
    # {image_rect,camera_info_rect}. camera_info_rect has K = P[:3,:3] and D = 0
    # so downstream stages can treat the image as pinhole-projected.

    left_rectify = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='left_rectify',
        namespace='stereo_depth',
        parameters=[{
            'output_width':  1440,
            'output_height': 900,
        }],
        remappings=[
            ('image_raw',         '/camera_stereo/left/image_raw'),
            ('camera_info',       '/camera_stereo/left/camera_info'),
            ('image_rect',        'left/image_rect'),
            ('camera_info_rect',  'left/camera_info_rect'),
        ],
    )

    right_rectify = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='right_rectify',
        namespace='stereo_depth',
        parameters=[{
            'output_width':  1440,
            'output_height': 900,
        }],
        remappings=[
            ('image_raw',         '/camera_stereo/right/image_raw'),
            ('camera_info',       '/camera_stereo/right/camera_info'),
            ('image_rect',        'right/image_rect'),
            ('camera_info_rect',  'right/camera_info_rect'),
        ],
    )

    # ---- Resize rectified → ESS input (960×576) ----------------------------

    left_resize = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='left_resize',
        namespace='stereo_depth',
        parameters=[{
            'output_width':  960,
            'output_height': 576,
            'keep_aspect_ratio': False,
        }],
        remappings=[
            ('image',                'left/image_rect'),
            ('camera_info',          'left/camera_info_rect'),
            ('resize/image',         'left/image_resize'),
            ('resize/camera_info',   'left/camera_info_resize'),
        ],
    )

    right_resize = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='right_resize',
        namespace='stereo_depth',
        parameters=[{
            'output_width':  960,
            'output_height': 576,
            'keep_aspect_ratio': False,
        }],
        remappings=[
            ('image',                'right/image_rect'),
            ('camera_info',          'right/camera_info_rect'),
            ('resize/image',         'right/image_resize'),
            ('resize/camera_info',   'right/camera_info_resize'),
        ],
    )

    # ---- ESS ---------------------------------------------------------------

    ess_disparity = ComposableNode(
        package='isaac_ros_ess',
        plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
        name='ess_disparity',
        namespace='stereo_depth',
        parameters=[{
            'engine_file_path':   engine_file_path,
            'threshold':          threshold,
            'input_layer_width':  input_layer_width,
            'input_layer_height': input_layer_height,
        }],
        remappings=[
            ('left/image_rect',   'left/image_resize'),
            ('right/image_rect',  'right/image_resize'),
            ('left/camera_info',  'left/camera_info_resize'),
            ('right/camera_info', 'right/camera_info_resize'),
        ],
    )

    disparity_to_depth = ComposableNode(
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
        name='disparity_to_depth',
        namespace='stereo_depth',
    )

    container = ComposableNodeContainer(
        name='stereo_depth_ess_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            left_rectify,
            right_rectify,
            left_resize,
            right_resize,
            ess_disparity,
            disparity_to_depth,
        ],
        output='screen',
    )

    depth_colorize = Node(
        package='visionconnect',
        executable='depth_colorize.py',
        name='depth_colorize',
        output='screen',
        remappings=[
            ('depth_in',        '/stereo_depth/depth'),
            ('depth_color_out', '/stereo_depth/depth_color'),
        ],
    )

    return LaunchDescription([
        decl_engine,
        decl_threshold,
        decl_in_w,
        decl_in_h,
        container,
        depth_colorize,
    ])
