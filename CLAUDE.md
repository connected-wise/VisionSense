# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

VisionConnect-Plus is a ROS2-based computer vision system for autonomous vehicles, specifically designed for NVIDIA Jetson platforms. It provides real-time object detection, lane detection, traffic sign classification, and object tracking capabilities.

## Building and Running

### Build Commands
```bash
# Build the package (from workspace root)
colcon build --packages-select visionconnect

# Source the workspace
source install/setup.bash
```

### Running the System
```bash
# Launch all nodes with test configuration
ros2 launch visionconnect test.launch.py

# Launch individual nodes
ros2 run visionconnect camera     # Camera input node
ros2 run visionconnect detect     # Object detection node
ros2 run visionconnect classify   # Traffic sign classifier
ros2 run visionconnect lanedet    # Lane detection node
ros2 run visionconnect gui        # GUI interface
ros2 run visionconnect dashboard  # Web dashboard
```

### Testing Components
```bash
# Run unit tests (if built)
colcon test --packages-select visionconnect

# Test individual components
./build/visionconnect/classifier-test  # Test sign classifier
./build/visionconnect/lanedet-test     # Test lane detection
```

## Architecture

### Core Components

1. **ROS2 Nodes** (7 main nodes):
   - `camera`: Handles video input from various sources (V4L2, CSI, file)
   - `detect`: YOLOv8-based object detection with TensorRT acceleration
   - `classify`: Traffic sign classification using neural networks
   - `lanedet`: Lane detection with neural network models
   - `preview`: Video output and visualization
   - `dashboard`: Web-based monitoring interface
   - `gui`: Native GUI with data fusion capabilities

2. **Common Libraries** (`src/common/`):
   - Image conversion utilities
   - TensorRT utilities for neural network inference
   - BYTE tracker for multi-object tracking
   - Kalman filter implementation
   - ROS2 compatibility layer

3. **Neural Network Models** (`src/graphs/`):
   - Lane detection models (ONNX → TensorRT)
   - YOLOv8 object detection models
   - Traffic sign classification models

### Data Flow

1. Camera node publishes raw images to `/camera/raw`
2. Detection nodes subscribe to camera feed:
   - Object detector publishes to `/detect/detections` and `/detect/signs`
   - Lane detector publishes to `/lanedet/lanes`
3. Classifier refines traffic signs from detector
4. GUI/Dashboard nodes aggregate all detection results for visualization

### Configuration

Main configuration in `config/config.yaml`:
- Sensor enable/disable flags
- Camera settings (source, resolution, orientation)
- Model paths and detection thresholds
- Object class configurations

### Custom Messages

Located in `/msg/`:
- `Box.msg`: Bounding box coordinates
- `Detect.msg`: Detection results with boxes and confidence
- `Lanes.msg`: Lane detection polylines
- `Signs.msg`: Traffic sign classification results
- `Track.msg`: Object tracking data with ID and state

## Key Dependencies

- ROS2 (Humble or newer)
- OpenCV 4 with CUDA support
- NVIDIA Jetson libraries (jetson-utils, jetson-inference)
- TensorRT for neural network inference
- CUDA for GPU acceleration
- Qt5 for GUI components
- VPI 2 (Vision Programming Interface)
- Eigen3 for linear algebra

## Development Notes

- C++17 standard with GPU acceleration via CUDA
- All nodes use TensorRT for optimized inference
- Web dashboard available at `http://localhost:8080` when running
- GPS integration via Python script in `/scripts/node_gps.py`