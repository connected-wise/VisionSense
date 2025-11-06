# VisionSense

A ROS2-based computer vision system for autonomous vehicles on NVIDIA Jetson platforms with JetPack 6.2.

## Overview

VisionSense provides real-time:
- Object detection using YOLOv8 with TensorRT
- Lane detection with neural networks
- Traffic sign classification
- Multi-object tracking with BYTE tracker
- Web dashboard for monitoring
- Native GUI with data fusion

## System Requirements

- **Hardware**: NVIDIA Jetson Orin or Xavier
- **OS**: Ubuntu 22.04 (JetPack 6.2)
- **ROS2**: Humble or newer
- **CUDA**: 12.6 (included in JetPack 6.2)
- **TensorRT**: 10.x (included in JetPack 6.2)

## Prerequisites Installation

### 1. Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-vision-msgs python3-colcon-common-extensions
```

### 2. Install System Dependencies

```bash
# Update package list
sudo apt update

# Install build tools and libraries
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-numpy \
    libeigen3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5webkit5-dev \
    libqt5opengl5-dev \
    libqt5svg5-dev \
    libopencv-dev \
    v4l-utils

# Install Python ROS2 dependencies for message generation
sudo apt install -y \
    python3-rosidl-generator-py \
    ros-humble-rosidl-typesupport-introspection-cpp \
    ros-humble-rosidl-typesupport-introspection-c
```

### 3. Install Jetson Libraries

```bash
# Clone and build jetson-inference (includes jetson-utils)
cd ~
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build && cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 4. Install Python Dependencies

```bash
cd /path/to/VisionSense
pip3 install -r requirements.txt
```

## Building VisionSense

1. Clone the repository:
```bash
git clone https://github.com/your-repo/VisionSense.git
cd VisionSense
```

2. Source ROS2 environment:
```bash
source /opt/ros/humble/setup.bash
```

3. Build the package:
```bash
colcon build --packages-select visionconnect --cmake-args -DBUILD_TESTING=OFF
```

## Configuration

Edit `config/config.yaml` to configure:
- Camera source (USB, CSI, or video file)
- Model paths and detection thresholds
- Sensor enable/disable flags

Example camera configurations:
```yaml
camera:
    ros__parameters:
        # For USB camera:
        resource: "v4l2:///dev/video0"
        
        # For CSI camera:
        # resource: "csi://0"
        
        # For video file:
        # resource: "file:///path/to/video.mp4"
        
        width: 1920
        height: 1080
        flip: ""  # Options: "", "rotate-180", "flip-horizontal", etc.
```

## Running the System

### Option 1: Using the Launch Script (Recommended)
```bash
# Make sure executables exist
cd build/visionconnect
ls camera detect classify lanedet dashboard gui

# If executables exist, run:
cd ../..
./run_nodes.sh
```

### Option 2: Running Individual Nodes
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Run camera node
./build/visionconnect/camera --ros-args \
    -p resource:="file:///path/to/video.mp4" \
    -p width:=1920 \
    -p height:=1080

# In separate terminals, run other nodes...
```

### Option 3: Using ROS2 Launch (if Python bindings are installed)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch visionconnect test.launch.py
```

## Neural Network Models

Place the following TensorRT engine files in the project root:
- `yolov8s-detect.engine` - Object detection model
- `yolov8n-TSR.engine` - Traffic sign classification model  
- `lane_detect.engine` - Lane detection model

Also required:
- `labels_detect.txt` - Object class labels
- `labels_classify.txt` - Traffic sign class labels

## Troubleshooting

### Missing Executables
If nodes like `detect`, `classify`, etc. are missing:
1. Check build logs for errors
2. Ensure all dependencies are installed
3. Try a clean rebuild:
   ```bash
   rm -rf build install log
   colcon build --packages-select visionconnect
   ```

### Camera Issues
- **USB Camera**: Check with `v4l2-ctl --list-devices`
- **CSI Camera**: Ensure camera is properly connected
- **Video File**: Use absolute paths starting with `file:///`

### Library Loading Errors
```bash
# Add library paths
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
sudo ldconfig
```

### OpenCV CUDA Support
This build includes fallback implementations if OpenCV CUDA is not available. For better performance, consider building OpenCV with CUDA support.

## Web Dashboard

Once running, access the dashboard at: http://localhost:8080

## Project Structure

```
VisionSense/
├── src/
│   ├── nodes/          # ROS2 node implementations
│   ├── common/         # Shared libraries and utilities
│   └── graphs/         # Neural network model files
├── config/             # Configuration files
├── launch/             # ROS2 launch files
├── msg/                # Custom message definitions
└── scripts/            # Utility scripts
```

## Known Issues on JetPack 6.2

1. **VPI Version**: Project requires VPI 3 (included in JetPack 6.2)
2. **TensorRT API**: Uses TensorRT 10.x APIs
3. **Python Bindings**: May require manual installation of ROS2 Python packages

## License

VisionSense is licensed for **non-commercial research and educational use only**. 

✅ **Allowed:** Research, education, testing, and developing your own distinct technologies  
❌ **Not Allowed:** Direct commercial use, integration into commercial products, or offering as a service  
💼 **Commercial License:** Contact licensing@connectedwise.com

See [![License: Non-Commercial](https://img.shields.io/badge/License-Non--Commercial-blue.svg)](LICENSE) for full terms.

## Contributing

We welcome research contributions! To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes (`git commit -m 'feat: add feature'`)
4. Push to the branch (`git push origin feature/my-feature`)
5. Open a Pull Request

Please follow existing code style, test your changes, and use [conventional commits](https://www.conventionalcommits.org/). 

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.
