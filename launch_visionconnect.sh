#!/bin/bash

# VisionConnect-Plus Launch Script
# This script starts rosbridge websocket server first, then launches the vision system

echo "Starting VisionConnect-Plus System..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
fi

echo "Checking for existing ROSbridge WebSocket processes..."
# Kill any existing rosbridge websocket processes
pkill -f "rosbridge_websocket_launch.xml" || true
pkill -f "rosbridge_websocket" || true

# Wait for processes to terminate gracefully
sleep 2

# Force kill if any still exist
pkill -9 -f "rosbridge_websocket" || true

# Wait longer for processes to fully terminate
sleep 3

echo "Verifying all ROSbridge processes are stopped..."
# Wait up to 10 seconds for processes to fully terminate
for i in {1..10}; do
    if ! pgrep -f "rosbridge_websocket" > /dev/null; then
        echo "All ROSbridge processes stopped successfully."
        break
    fi
    echo "Waiting for ROSbridge processes to terminate... ($i/10)"
    sleep 1
    if [ $i -eq 10 ]; then
        echo "Warning: Some ROSbridge processes may still be running."
    fi
done

echo "Starting ROSbridge WebSocket server..."
# Launch rosbridge websocket server in background
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Wait a moment for rosbridge to initialize
sleep 3

echo "Starting VisionConnect-Plus nodes..."
# Launch the main vision system
ros2 launch visionconnect test.launch.py

# When test.launch.py exits, clean up rosbridge
echo "Shutting down ROSbridge WebSocket server..."
kill $ROSBRIDGE_PID

echo "VisionConnect-Plus system stopped."