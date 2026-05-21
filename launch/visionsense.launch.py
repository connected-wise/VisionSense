# visionsense.launch.py
# Main launch file for VisionSense autonomous vehicle vision system.
# Stereo depth backend: LightStereo-S TensorRT (scripts/stereo_depth_lightstereo.py).
# ICRA 2025, OpenStereo authors — ~16ms GPU compute at 320×512 on Orin NX.

import os
import subprocess
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():

	# All created nodes will be added to this object
	# to be launched from
	ld = LaunchDescription()

	config_dir = os.path.join(
		get_package_share_directory("visionconnect"),
		"config",
	)
	config_path = os.path.join(config_dir, "config.yaml")

	with open(config_path, "r") as fp:
		config = yaml.safe_load(fp)

	# Resolve camera_stereo's calibration_file (just a filename in config.yaml)
	# to a full path under the installed config dir, so the camera node can
	# open it as-is. NITROS RectifyNode needs the resulting K/D/R/P in the
	# camera's CameraInfo. Empty string keeps the focal/pixel-pitch fallback.
	cs_params = dict(config["camera_stereo"]["ros__parameters"])
	calib_name = cs_params.get("calibration_file", "")
	if calib_name:
		cs_params["calibration_file"] = os.path.join(config_dir, calib_name)

	# Resolve which eye of the stereo pair single-image consumers (detect,
	# lanedet, gui, dashboard) read from. Defaults to "left" if `stereo.main_eye`
	# is absent or malformed. stereo_depth is unaffected — it still subscribes
	# to both eyes.
	main_eye = str(config.get("stereo", {}).get("main_eye", "left")).lower()
	if main_eye not in ("left", "right"):
		print(f"[visionsense.launch] WARNING: stereo.main_eye='{main_eye}' invalid; defaulting to 'left'")
		main_eye = "left"
	main_image_topic = f"/camera_stereo/{main_eye}/image_raw"

	# UI toggles. `display` controls the on-screen cv::imshow window (passed to
	# gui as a ROS parameter). `dashboard` controls whether we open Chromium
	# in kiosk mode to the dashboard URL after node_dashboard is up.
	ui = config.get("ui", {}) or {}
	ui_display = bool(ui.get("display", True))
	ui_dashboard = bool(ui.get("dashboard", False))

	# Mono IMX219 driver-monitor camera — NOT started here.
	#
	# The Jetson Argus stack wedges its CSI/RTCPU state on Producer close;
	# every second `nvarguscamerasrc` session fails with INVALID_SETTINGS
	# until reboot (bare gst-launch reproduces it, so it's an NVIDIA bug, not
	# ours). To avoid that, the IMX219 is owned by a persistent systemd unit
	# — `scripts/visionsense-imx219.service`, installed once via
	# `sudo systemctl enable --now visionsense-imx219` — which opens the
	# camera at boot and never closes it. driver_monitor + anyone else just
	# subscribes to /camera/raw as a normal ROS topic, oblivious to who's
	# publishing it. Closing VisionSense no longer touches Argus.
	#
	# If the systemd unit isn't installed yet, /camera/raw simply won't be
	# published and driver_monitor will idle — log a hint so it's obvious.
	try:
		_active = subprocess.run(
			["systemctl", "is-active", "--quiet", "visionsense-imx219.service"],
			timeout=2,
		).returncode == 0
	except Exception:
		_active = False
	if not _active:
		print("[visionsense.launch] WARNING: visionsense-imx219.service is not active — "
		      "/camera/raw will be silent. Install with: "
		      "sudo cp scripts/visionsense-imx219.service /etc/systemd/system/ && "
		      "sudo systemctl daemon-reload && "
		      "sudo systemctl enable --now visionsense-imx219.service")



	# Object Detection
	detect_node = Node(
		package="visionconnect",
		name="detect",
		executable="detect",
		output="screen",
		parameters=[config["detect"]["ros__parameters"]],
		remappings=[
			("/detect/image_in", main_image_topic)
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
			("/lanedet/image_in", main_image_topic)
		]
	)
	ld.add_action(lanedet_node)

	# ADAS (Advanced Driver Assistance System)
	# image_width/height must match the frame fed to lanedet (= main_image_topic
	# = camera_stereo per-eye crop). With rotated_lenses=false (default config)
	# each eye is 1440x900; with rotated_lenses=true it's 1200x1200 (square,
	# legacy). ADAS normalizes lane points by these to produce 0..1 coords
	# consumed by gui/bev — wrong dims here = tilted lane regions on screen.
	rotated_lenses = bool(cs_params.get("rotated_lenses", False))
	adas_img_w, adas_img_h = (1200, 1200) if rotated_lenses else (1440, 900)
	adas_node = Node(
		package="visionconnect",
		name="adas",
		executable="adas",
		output="screen",
		parameters=[{
			"image_width":  adas_img_w,
			"image_height": adas_img_h,
		}],
		remappings=[
			("/adas/lanes_in", "/lanedet/lanes")
		]
	)
	ld.add_action(adas_node)

	# Driver Monitor - TensorRT accelerated face detection + gaze estimation
	driver_monitor_node = Node(
		package="visionconnect",
		name="driver_monitor",
		executable="driver_monitor",
		output="screen",
		parameters=[config["driver_monitor"]["ros__parameters"]]
	)
	ld.add_action(driver_monitor_node)

	# IMU and GPS
	imu_gps_node = Node(
		package="visionconnect",
		name="imu_gps",
		executable="imu_gps",
		output="screen"
	)
	ld.add_action(imu_gps_node)

	# Stereo Camera (for depth) — calibration_file resolved above into cs_params
	# so its CameraInfo carries calibrated K/D/R/P (consumed by downstream
	# rectification + the LightStereo node).
	camera_stereo_node = Node(
		package="visionconnect",
		name="camera_stereo",
		executable="camera_stereo",
		output="screen",
		parameters=[cs_params],
	)
	ld.add_action(camera_stereo_node)

	# Stereo Depth Processing — LightStereo-S / FFS TensorRT (C++ node).
	#   Subscribes /camera_stereo/{l,r}/image_raw → fused CUDA kernel that does
	#   rectify (model-resolution remap) + resize to 320×512 + ImageNet norm +
	#   HWC→CHW directly into the TRT input bindings → enqueueV3 inference →
	#   publishes /stereo_depth/depth (32FC1, metres, native eye res) and
	#   /stereo_depth/depth_color (BGR8 INFERNO, downsized for cheap viz).
	# Python fallback (scripts/stereo_depth_lightstereo.py) is still installed
	# and can be re-pointed here as `executable="stereo_depth_lightstereo.py"`
	# if the C++ port has issues.
	# `calibration_file` is the absolute path resolved above (cs_params).
	# `engine_file_path` in config.yaml is relative to the installed
	# graphs/stereo-depth dir — resolve it here.
	ls_params = dict(config.get("stereo_depth_lightstereo", {}).get("ros__parameters", {}))
	ls_params["calibration_file"] = cs_params["calibration_file"]
	engine_name = ls_params.get("engine_file_path", "")
	if engine_name and not os.path.isabs(engine_name):
		ls_params["engine_file_path"] = os.path.join(
			get_package_share_directory("visionconnect"),
			"graphs", "stereo-depth", engine_name,
		)
	stereo_depth_node = Node(
		package="visionconnect",
		namespace="stereo_depth",
		name="stereo_depth_lightstereo",
		executable="stereo_depth",
		output="screen",
		parameters=[ls_params],
		remappings=[
			("left/image_raw",  "/camera_stereo/left/image_raw"),
			("right/image_raw", "/camera_stereo/right/image_raw"),
		],
	)
	ld.add_action(stereo_depth_node)

	# Bird's Eye View mapping
	bev_node = Node(
		package="visionconnect",
		name="bev",
		executable="bev",
		output="screen",
		parameters=[config["bev"]["ros__parameters"]],
		remappings=[
			("/bev/detect_in", "/detect/detections"),
			("/bev/lanes_in", "/lanedet/lanes"),
			("/bev/adas_in", "/adas/adas_alerts")
		]
	)
	ld.add_action(bev_node)

	# Dashboard — single binary that embeds an HTTP + WebSocket server
	# (libwebsockets) on port 8080. Subscribes to image and telemetry topics,
	# JPEG-encodes images, and pushes everything directly to browser clients.
	# No rosbridge, no separate http.server — open http://<host>:8080/ from
	# any device (PC, Mac, Android, iPhone) on the same network.
	dashboard_node = Node(
		package="visionconnect",
		name="dashboard",
		executable="dashboard",
		output="screen",
		remappings=[
			# image_in / image_in_right feed the raw-camera stream. The
			# dashboard subscribes to BOTH so the browser can toggle L/R
			# at runtime via {"t":"camera-side","side":"..."}.
			("/dashboard/image_in",       "/camera_stereo/left/image_raw"),
			("/dashboard/image_in_right", "/camera_stereo/right/image_raw"),
			("/dashboard/detect_in", "/detect/detections"),
			("/dashboard/signs_in",  "/detect/signs"),
			("/dashboard/lanes_in",  "/lanedet/lanes"),
			("/dashboard/gui_in",    "/gui/fusion"),
			("/dashboard/depth_in",  "/stereo_depth/depth_color"),
			("/dashboard/driver_in", "/driver_monitor/image"),
			("/dashboard/bev_in",    "/bev/image"),
			("/dashboard/gps_in",    "/imu_gps/gps/fix"),
			("/dashboard/imu_in",    "/imu_gps/imu/data"),
			("/dashboard/dms_in",    "/driver_monitor/state"),
		]
	)
	ld.add_action(dashboard_node)

	# GUI / Data Fusion. gui uses GTK via OpenCV's HighGUI — without DISPLAY
	# AND XAUTHORITY the GTK backend init fails with
	#   OpenCV error: Can't initialize GTK backend in function 'cvInitSystem'
	# and the node dies on startup. Forward both from the parent env (set by
	# launch_visionsense.sh) so this works whether launched from a .desktop
	# entry, a graphical terminal, or `bash launch_visionsense.sh`.
	gui_env = {"DISPLAY": os.environ.get("DISPLAY", ":0")}
	if os.environ.get("XAUTHORITY"):
		gui_env["XAUTHORITY"] = os.environ["XAUTHORITY"]
	# Forward the ui.display flag into the gui node as a ROS parameter so it
	# can skip namedWindow/imshow when display=false (still publishes
	# /gui/fusion for the dashboard).
	gui_params = dict(config["gui"]["ros__parameters"])
	gui_params["display"] = ui_display
	gui_node = Node(
		package="visionconnect",
		name="gui",
		executable="gui",
		output="screen",
		additional_env=gui_env,
		parameters=[gui_params],
		remappings=[
			("/gui/image_in", main_image_topic),
			("/gui/detect_in", "/detect/detections"),
			("/gui/signs_in", "/classify/signs"),
			("/gui/lanes_in", "/lanedet/lanes"),
			("/gui/adas_in", "/adas/adas_alerts")
		]
	)
	ld.add_action(gui_node)

	# If ui.dashboard=true, open the dashboard in a fullscreen Chromium kiosk
	# AFTER actively confirming http://localhost:8080/ responds. Three things
	# bit us with the naïve "TimerAction + chromium --app= URL" approach:
	#  1. A fixed delay races against node_dashboard binding port 8080 → the
	#     kiosk loads ERR_CONNECTION_REFUSED and sits on it.
	#  2. --app= URL attaches to an EXISTING chromium instance (if the user
	#     has one open). That instance ignores --kiosk, opens an "app window"
	#     with the user's normal session profile, and often shows a blank
	#     page because the cached service worker / state mismatches.
	#  3. Some chromium builds gate WebRTC <video autoplay> on a user gesture
	#     unless --autoplay-policy is overridden.
	# Fix: poll the server, use a dedicated --user-data-dir so we always get a
	# fresh kiosk instance, and force the autoplay policy.
	if ui_dashboard:
		kiosk_cmd = (
			"for bin in chromium-browser chromium google-chrome chrome; do "
			"  if command -v \"$bin\" >/dev/null 2>&1; then BROWSER=\"$bin\"; break; fi; "
			"done; "
			"if [ -z \"$BROWSER\" ]; then "
			"  echo '[visionsense] no chromium binary found — skip ui.dashboard'; exit 0; "
			"fi; "
			# Wait up to ~30s for node_dashboard to start serving.
			"for i in $(seq 1 60); do "
			"  curl -sf -o /dev/null --max-time 1 http://localhost:8080/dashboard.html && break; "
			"  sleep 0.5; "
			"done; "
			"if ! curl -sf -o /dev/null --max-time 1 http://localhost:8080/dashboard.html; then "
			"  echo '[visionsense] dashboard never came up on :8080 — skip kiosk'; exit 0; "
			"fi; "
			# Dedicated user-data-dir → fresh chromium instance, never reuses
			# an already-running profile (which would ignore --kiosk and load
			# stale state). Cleared every launch keeps things deterministic.
			"DATA_DIR=\"$HOME/.cache/visionsense-kiosk\"; "
			"rm -rf \"$DATA_DIR\" 2>/dev/null; mkdir -p \"$DATA_DIR\"; "
			"exec \"$BROWSER\" "
			"  --user-data-dir=\"$DATA_DIR\" "
			"  --kiosk "
			"  --no-first-run --noerrdialogs --disable-infobars "
			"  --disable-session-crashed-bubble --disable-translate "
			"  --autoplay-policy=no-user-gesture-required "
			"  --disable-features=TranslateUI,InterestFeedContentSuggestions "
			"  http://localhost:8080/"
		)
		dashboard_browser = TimerAction(
			period=0.5,  # tiny head-start; the curl-poll inside does the real wait
			actions=[ExecuteProcess(
				cmd=["bash", "-lc", kiosk_cmd],
				additional_env=gui_env,
				output="screen",
			)],
		)
		ld.add_action(dashboard_browser)

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
	# 		("/preview/signs_in", "/detect/signs"),
	# 		("/preview/lanes_in", "/lanedet/lanes")
	# 	]
	# )
	# ld.add_action(preview_node)
	
	
	return ld
