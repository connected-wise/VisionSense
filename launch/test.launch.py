# test.launch.py

import os
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

	# All created nodes will be added to this object
	# to be launched from
	ld = LaunchDescription()

	config_path = os.path.join(
		get_package_share_directory("visionconnect"),
		"config",
		"config.yaml"
		)
		
	with open(config_path, "r") as fp:
		config = yaml.safe_load(fp)
		
	# Camera
	camera_node = Node(
		package="visionconnect",
		name="camera",
		executable="camera",
		output="screen",
		parameters=[config["camera"]["ros__parameters"]]
	)
	ld.add_action(camera_node)
		

	# Object Detection
	detect_node = Node(
		package="visionconnect",
		name="detect",
		executable="detect",
		output="screen",
		parameters=[config["detect"]["ros__parameters"]],
		remappings=[
			("/detect/image_in", "/camera/raw")
		]
	)
	ld.add_action(detect_node)

	# Traffic Sign and Light Classification
	classify_node = Node(
		package="visionconnect",
		name="classify",
		executable="classify",
		output="screen",
		parameters=[config["classify"]["ros__parameters"]],
		remappings=[
			("/classify/signs_in", "/detect/signs")
		]
	)
	ld.add_action(classify_node)

	# Lane Detection
	lanedet_node = Node(
		package="visionconnect",
		name="lanedet",
		executable="lanedet",
		output="screen",
		parameters=[config["lanedet"]["ros__parameters"]],
		remappings=[
			("/lanedet/image_in", "/camera/raw")
		]
	)
	ld.add_action(lanedet_node)

	# dashboard 
	dashboard_node = Node(
		package="visionconnect",
		name="dashboard",
		executable="dashboard",
		output="screen",
		remappings=[
			("/dashboard/image_in", "/camera/raw"),
			("/dashboard/detect_in", "/detect/detections"),
			("/dashboard/signs_in", "/detect/signs"),
			("/dashboard/lanes_in", "/lanedet/lanes"),
			("/dashboard/gui_in", "/gui/fusion")
		]
	)
	ld.add_action(dashboard_node)

	# GUI / Data Fusion 
	gui_node = Node(
		package="visionconnect",
		name="gui",
		executable="gui",
		output="screen",
		remappings=[
			("/gui/image_in", "/camera/raw"),
			("/gui/detect_in", "/detect/detections"),
			("/gui/signs_in", "/classify/signs"),
			("/gui/track_in", "/detect/track"),
			("/gui/lanes_in", "/lanedet/lanes")
		]
	)
	ld.add_action(gui_node)

	# # Preview images
	# preview_node = Node(
	# 	package="visionconnect",
	# 	name="preview",
	# 	executable="preview",
	# 	output="screen",
	# 	parameters=[config["preview"]["ros__parameters"]],
	# 	remappings=[
	# 		("/preview/image_in", "/camera/raw"),
	# 		("/preview/detect_in", "/detect/detections"),
	# 		("/preview/signs_in", "/classify/signs"),
	# 		("/preview/lanes_in", "/lanedet/lanes"),
	# 		("/preview/gui_in", "/gui/fusion")

	# 	]
	# )
	# ld.add_action(preview_node)
	
	
	return ld
