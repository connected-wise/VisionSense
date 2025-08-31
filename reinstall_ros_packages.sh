#!/bin/bash

# Script to reinstall ROS2 packages that may have been removed during OpenCV build
# For JetPack 6.2 with ROS2 Humble

echo "=== Reinstalling ROS2 Packages ==="
echo "This will reinstall ROS2 packages that may have been removed during OpenCV build"
echo ""

# Update package list
sudo apt-get update

# Core ROS2 packages
echo "Installing core ROS2 packages..."
sudo apt-get install -y \
    ros-humble-ros-base \
    ros-humble-ros-core \
    ros-humble-desktop

# Development tools
echo "Installing ROS2 development tools..."
sudo apt-get install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Ament build system packages
echo "Installing Ament build system packages..."
sudo apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-auto \
    ros-humble-ament-cmake-core \
    ros-humble-ament-cmake-python \
    ros-humble-ament-cmake-ros \
    ros-humble-ament-index-cpp \
    ros-humble-ament-index-python \
    ros-humble-ament-package

# Message generation packages
echo "Installing message generation packages..."
sudo apt-get install -y \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rosidl-generator-c \
    ros-humble-rosidl-generator-cpp \
    ros-humble-rosidl-generator-py \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-rosidl-typesupport-introspection-c \
    ros-humble-rosidl-typesupport-introspection-cpp \
    ros-humble-rosidl-typesupport-fastrtps-c \
    ros-humble-rosidl-typesupport-fastrtps-cpp

# Common ROS2 packages
echo "Installing common ROS2 packages..."
sudo apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-image-geometry

# Additional packages for VisionConnect
echo "Installing additional packages for VisionConnect..."
sudo apt-get install -y \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py

# Image processing packages
echo "Installing image processing packages..."
sudo apt-get install -y \
    ros-humble-image-common \
    ros-humble-image-pipeline \
    ros-humble-image-transport-plugins \
    ros-humble-vision-opencv

# Qt packages for GUI
echo "Installing Qt packages..."
sudo apt-get install -y \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools

# Fix any broken packages
echo "Fixing any broken packages..."
sudo apt-get install -f -y

# Update rosdep
echo "Updating rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

echo ""
echo "=== ROS2 Package Reinstallation Complete ==="
echo ""

# Verify installation
echo "Verifying ROS2 installation..."
source /opt/ros/humble/setup.bash

echo "ROS2 version:"
ros2 --version

echo ""
echo "Checking critical packages:"
echo -n "ament-index-cpp: "
dpkg -l | grep ros-humble-ament-index-cpp > /dev/null && echo "✓ Installed" || echo "✗ Missing"

echo -n "cv-bridge: "
dpkg -l | grep ros-humble-cv-bridge > /dev/null && echo "✓ Installed" || echo "✗ Missing"

echo -n "rosidl-generators: "
dpkg -l | grep ros-humble-rosidl-default-generators > /dev/null && echo "✓ Installed" || echo "✗ Missing"

echo ""
echo "You can now rebuild VisionConnect-Plus:"
echo "  cd ~/VisionConnect-Plus"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select visionconnect --cmake-args -DCMAKE_BUILD_TYPE=Release"