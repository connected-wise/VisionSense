#!/bin/bash
# Simple script to run VisionConnect-Plus nodes without full ROS2 package installation

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Set working directory
cd /home/cw-orin/VisionConnect-Plus

# Kill any existing nodes
pkill -f camera
pkill -f detect
pkill -f classify
pkill -f lanedet
pkill -f dashboard
pkill -f gui
pkill -f preview

echo "Starting VisionConnect-Plus nodes..."
echo ""

# Function to check if executable exists
check_node() {
    if [ -f "build/visionconnect/$1" ]; then
        return 0
    else
        echo "WARNING: $1 node not found (not built)"
        return 1
    fi
}

# Start camera node
if check_node "camera"; then
    echo "Starting camera node..."
    ./build/visionconnect/camera --ros-args \
        -p resource:="file:///home/cw-orin/Videos/videoplayback.mp4" \
        -p width:=1920 \
        -p height:=1080 \
        -p flip:="rotate-180" &
    sleep 3
fi

# Start detect node
if check_node "detect"; then
    echo "Starting detect node..."
    ./build/visionconnect/detect --ros-args \
        -p model:="yolov8s-detect.engine" \
        -p labels:="labels_detect.txt" \
        -p thresholds:=[0.40,0.45,0.45,0.6,0.55,0.45,0.5,0.35,0.55] \
        --remap /detect/image_in:=/camera/raw &
    sleep 2
fi

# Start classify node
if check_node "classify"; then
    echo "Starting classify node..."
    ./build/visionconnect/classify --ros-args \
        -p model:="yolov8n-TSR.engine" \
        -p labels:="labels_classify.txt" \
        -p thresholds:=[0.30,0.75] \
        --remap /classify/signs_in:=/detect/signs &
    sleep 2
fi

# Start lanedet node
if check_node "lanedet"; then
    echo "Starting lanedet node..."
    ./build/visionconnect/lanedet --ros-args \
        -p model:="lane_detect.engine" \
        --remap /lanedet/image_in:=/camera/raw &
    sleep 2
fi

# Start dashboard node
if check_node "dashboard"; then
    echo "Starting dashboard node..."
    ./build/visionconnect/dashboard --ros-args \
        --remap /dashboard/image_in:=/camera/raw \
        --remap /dashboard/detect_in:=/detect/detections \
        --remap /dashboard/signs_in:=/detect/signs \
        --remap /dashboard/lanes_in:=/lanedet/lanes \
        --remap /dashboard/gui_in:=/gui/fusion &
    sleep 2
fi

# Start GUI node
if check_node "gui"; then
    echo "Starting GUI node..."
    ./build/visionconnect/gui --ros-args \
        --remap /gui/image_in:=/camera/raw \
        --remap /gui/detect_in:=/detect/detections \
        --remap /gui/signs_in:=/classify/signs \
        --remap /gui/track_in:=/detect/track \
        --remap /gui/lanes_in:=/lanedet/lanes &
fi

# Start preview node (if you want visual output)
if check_node "preview"; then
    echo ""
    echo "Starting preview node for visual output..."
    ./build/visionconnect/preview --ros-args \
        -p output:="display://0" \
        -p width:=960 \
        -p height:=540 \
        --remap /preview/image_in:=/camera/raw &
fi

echo ""
echo "Available nodes started. Press Ctrl+C to stop."
echo "Dashboard should be available at http://localhost:8080"
echo ""
echo "Run ./check_build.sh to see which nodes were built successfully."

# Wait for interrupt
wait