#!/bin/bash
# Comprehensive installation script for VisionConnect-Plus on JetPack 6.2
# Run with: bash install_all_deps.sh

set -e  # Exit on error

echo "======================================"
echo "VisionConnect-Plus Dependency Installer"
echo "For JetPack 6.2 on NVIDIA Jetson"
echo "======================================"

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "Warning: This doesn't appear to be a Jetson device."
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Function to check if a package is installed
check_package() {
    dpkg -l | grep -q "^ii  $1 " && echo "✓ $1 already installed" || return 1
}

# Function to check if ROS2 is sourced
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        if [ -f /opt/ros/humble/setup.bash ]; then
            source /opt/ros/humble/setup.bash
        fi
    fi
}

echo -e "\n1. Updating package lists..."
sudo apt update

echo -e "\n2. Installing ROS2 Humble..."
if ! check_package ros-humble-desktop; then
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop
fi

echo -e "\n3. Installing ROS2 additional packages..."
sudo apt install -y \
    ros-humble-vision-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

echo -e "\n4. Installing build dependencies..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    unzip

echo -e "\n5. Installing Python dependencies..."
sudo apt install -y \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-opencv \
    python3-rosidl-generator-py \
    ros-humble-rosidl-typesupport-introspection-cpp \
    ros-humble-rosidl-typesupport-introspection-c

echo -e "\n6. Installing system libraries..."
sudo apt install -y \
    libeigen3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    v4l-utils

echo -e "\n7. Installing Qt5 dependencies..."
sudo apt install -y \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5webkit5-dev \
    libqt5opengl5-dev \
    libqt5svg5-dev \
    libqt5multimedia5 \
    libqt5multimediawidgets5

echo -e "\n8. Installing OpenCV (should be pre-installed with JetPack)..."
if ! check_package libopencv-dev; then
    sudo apt install -y libopencv-dev
fi

echo -e "\n9. Setting up environment variables..."
cat >> ~/.bashrc << 'EOL'

# ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# CUDA (JetPack 6.2)
export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH

# Jetson libraries
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
EOL

echo -e "\n10. Installing Python packages..."
pip3 install --upgrade pip
pip3 install numpy pyserial

echo -e "\n11. Checking jetson-inference installation..."
if [ ! -d "/usr/local/include/jetson-utils" ]; then
    echo "jetson-inference not found. Installing..."
    cd ~
    if [ ! -d "jetson-inference" ]; then
        git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
    fi
    cd jetson-inference
    mkdir -p build && cd build
    cmake ../
    make -j$(nproc)
    sudo make install
    sudo ldconfig
else
    echo "✓ jetson-inference already installed"
fi

echo -e "\n12. Verifying installations..."
echo "Checking CUDA..."
nvcc --version || echo "✗ CUDA not found"

echo -e "\nChecking TensorRT..."
dpkg -l | grep tensorrt || echo "✗ TensorRT not found"

echo -e "\nChecking OpenCV..."
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)" || echo "✗ OpenCV Python not found"

echo -e "\nChecking ROS2..."
check_ros2
ros2 pkg list | grep -q vision_msgs && echo "✓ vision_msgs found" || echo "✗ vision_msgs not found"

echo -e "\n======================================"
echo "Installation complete!"
echo "======================================"
echo ""
echo "Next steps:"
echo "1. Close and reopen your terminal (or run: source ~/.bashrc)"
echo "2. Navigate to VisionConnect-Plus directory"
echo "3. Build with: colcon build --packages-select visionconnect"
echo "4. Run with: ./run_nodes.sh"
echo ""
echo "If you encounter any issues, check INSTALL_JETPACK_6.2.md"