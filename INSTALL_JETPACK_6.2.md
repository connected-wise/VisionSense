# Installation Guide for JetPack 6.2

This guide provides step-by-step instructions for installing VisionConnect-Plus on NVIDIA Jetson devices running JetPack 6.2.

## Pre-Installation Checklist

- [ ] JetPack 6.2 installed and verified
- [ ] Internet connection available
- [ ] At least 10GB free disk space
- [ ] sudo/admin access

## Step 1: Verify JetPack Installation

```bash
# Check JetPack version
cat /etc/nv_tegra_release

# Verify CUDA installation
nvcc --version  # Should show CUDA 12.6

# Check TensorRT
dpkg -l | grep tensorrt  # Should show TensorRT 10.x
```

## Step 2: Install Core Dependencies

```bash
# Create installation script
cat > install_deps.sh << 'EOF'
#!/bin/bash
set -e

echo "Installing VisionConnect-Plus dependencies for JetPack 6.2..."

# Update system
sudo apt update

# Install ROS2 Humble
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-vision-msgs python3-colcon-common-extensions

# Install build dependencies
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

# Install Python ROS2 dependencies
sudo apt install -y \
    python3-rosidl-generator-py \
    ros-humble-rosidl-typesupport-introspection-cpp \
    ros-humble-rosidl-typesupport-introspection-c

echo "Core dependencies installed successfully!"
EOF

chmod +x install_deps.sh
./install_deps.sh
```

## Step 3: Install Jetson Libraries

```bash
# Install jetson-inference (includes jetson-utils)
cd ~
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build && cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

## Step 4: Clone and Build VisionConnect-Plus

```bash
# Clone repository
cd ~
git clone https://github.com/your-repo/VisionConnect-Plus.git
cd VisionConnect-Plus

# Install Python dependencies
pip3 install numpy pyserial

# Build the project
source /opt/ros/humble/setup.bash
colcon build --packages-select visionconnect --cmake-args -DBUILD_TESTING=OFF
```

## Step 5: Verify Installation

```bash
# Check if executables were built
ls build/visionconnect/camera
ls build/visionconnect/dashboard

# Test camera node
source /opt/ros/humble/setup.bash
./build/visionconnect/camera --ros-args -p resource:="v4l2:///dev/video0" --ros-args --log-level debug
```

## Common Issues and Solutions

### Issue: Missing libvisionconnect__rosidl_typesupport_*.so

**Solution**: The Python bindings failed to build. You can still run the C++ nodes directly:
```bash
# Use the provided run_nodes.sh script
./run_nodes.sh
```

### Issue: GStreamer Critical Errors

**Solution**: These are warnings and can usually be ignored. The video pipeline should still work.

### Issue: No cameras available

**Solution**: 
1. For USB cameras: Check `ls /dev/video*`
2. For CSI cameras: Verify connection and use `csi://0`
3. Use a video file for testing: Update config.yaml with a valid video path

### Issue: CUDA/TensorRT errors

**Solution**: 
```bash
# Set library paths
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH
sudo ldconfig
```

### Issue: Qt5 related errors

**Solution**:
```bash
# Install additional Qt5 packages
sudo apt install -y qt5-default libqt5multimediawidgets5
```

## Post-Installation Setup

1. **Download Models**: Place TensorRT engine files in the project root:
   - yolov8s-detect.engine
   - yolov8n-TSR.engine
   - lane_detect.engine

2. **Configure Camera**: Edit `config/config.yaml`:
   ```yaml
   camera:
       ros__parameters:
           resource: "v4l2:///dev/video0"  # or your camera source
   ```

3. **Test the System**:
   ```bash
   ./run_nodes.sh
   ```

4. **Access Dashboard**: Open http://localhost:8080 in a web browser

## Environment Variables

Add to your `~/.bashrc`:
```bash
# ROS2
source /opt/ros/humble/setup.bash

# CUDA
export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH

# Jetson libraries
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH
```

## Verification Commands

```bash
# Check all dependencies
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
ros2 pkg list | grep vision_msgs
nvcc --version
pkg-config --modversion tensorrt
```

## Support

If you encounter issues not covered here:
1. Check the build logs in `log/latest_build/`
2. Verify all dependencies with the verification commands
3. Try a clean rebuild: `rm -rf build install log && colcon build`