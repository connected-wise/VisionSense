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
    python3-ament-package \
    ros-humble-rosidl-generator-py \
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

echo -e "\n7. Checking OpenCV installation..."
# Check if OpenCV from source is installed and reinstall headers if needed
if [ -f "/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake" ]; then
    OPENCV_VER=$(grep 'SET(OpenCV_VERSION ' /usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake | grep -oP '\d+\.\d+\.\d+')
    echo "OpenCV libraries version: $OPENCV_VER"

    # Check if headers match libraries (system packages may overwrite headers)
    if [ -f "/usr/include/opencv4/opencv2/core/version.hpp" ]; then
        HEADER_MINOR=$(grep '#define CV_VERSION_MINOR' /usr/include/opencv4/opencv2/core/version.hpp | grep -oP '\d+')
        LIB_MINOR=$(echo $OPENCV_VER | cut -d. -f2)
        if [ "$HEADER_MINOR" != "$LIB_MINOR" ]; then
            echo "Warning: Header/library version mismatch detected (headers: 4.$HEADER_MINOR, libs: $OPENCV_VER)"
            if [ -d ~/opencv/build ]; then
                echo "Reinstalling OpenCV headers from source build..."
                cd ~/opencv/build && sudo make install > /dev/null 2>&1
                echo "✓ OpenCV headers reinstalled"
            else
                echo "Please rebuild OpenCV from source to fix header mismatch."
            fi
        else
            echo "✓ OpenCV version: $OPENCV_VER (headers and libraries match)"
        fi
    fi
else
    echo "OpenCV not found. Please run install_opencv_cuda_orin.sh first to build OpenCV with CUDA support."
    echo "Run: bash install_opencv_cuda_orin.sh"
fi

echo -e "\n8. Setting up environment variables..."
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

echo -e "\n9. Installing Python packages..."
pip3 install --upgrade pip
pip3 install numpy pyserial

echo -e "\n10. Fixing npymath library for newer NumPy versions..."
# NumPy >= 1.24 moved/removed npymath from standard paths, causing jetson-inference build to fail
NPYMATH_SRC="/usr/lib/python3/dist-packages/numpy/core/lib/libnpymath.a"
if [ -f "$NPYMATH_SRC" ] && [ ! -f /usr/lib/libnpymath.a ]; then
    echo "Creating symlink for npymath library..."
    sudo ln -sf "$NPYMATH_SRC" /usr/lib/libnpymath.a
    echo "✓ npymath symlink created"
elif [ -f /usr/lib/libnpymath.a ]; then
    echo "✓ npymath symlink already exists"
else
    echo "Warning: npymath library not found at $NPYMATH_SRC"
fi

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

echo -e "\n12. Installing camera device tree overlay..."
# Combined overlay for AR0234 stereo camera + IMX219 rear camera
OVERLAY_SRC="$(dirname "$0")/overlays/tegra234-p3767-camera-p3768-arducam-imx219-combined.dts"
OVERLAY_DST="/boot/arducam/dts/tegra234-p3767-camera-p3768-arducam-imx219-combined.dtbo"

if [ -f "$OVERLAY_SRC" ]; then
    echo "Compiling camera overlay..."
    OVERLAY_TMP="/tmp/tegra234-p3767-camera-p3768-arducam-imx219-combined.dtbo"
    dtc -I dts -O dtb -o "$OVERLAY_TMP" "$OVERLAY_SRC" 2>/dev/null

    if [ -f "$OVERLAY_TMP" ]; then
        # Check if Arducam directory exists
        if [ -d "/boot/arducam/dts" ]; then
            sudo cp "$OVERLAY_TMP" "$OVERLAY_DST"
            echo "✓ Camera overlay installed to $OVERLAY_DST"

            # Update extlinux.conf if ArducamIMX219 entry doesn't exist
            if ! grep -q "ArducamIMX219" /boot/extlinux/extlinux.conf; then
                echo "Adding boot entry for combined camera overlay..."

                # Determine kernel and FDT paths
                if [ -f "/boot/arducam/Image" ]; then
                    KERNEL_PATH="/boot/arducam/Image"
                    FDT_PATH="/boot/arducam/dts/dtb/tegra234-p3768-0000+p3767-0005-nv-super.dtb"
                else
                    KERNEL_PATH="/boot/Image"
                    FDT_PATH="/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb"
                fi

                sudo tee -a /boot/extlinux/extlinux.conf > /dev/null << BOOTENTRY

LABEL ArducamIMX219
	MENU LABEL Custom Header Config: <AR0234 + IMX219 Combined>
	LINUX ${KERNEL_PATH}
	FDT ${FDT_PATH}
	INITRD /boot/initrd
	APPEND \${cbootargs} root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0
	OVERLAYS /boot/arducam/dts/tegra234-p3767-camera-p3768-arducam-imx219-combined.dtbo
BOOTENTRY
                echo "✓ Boot entry added"
            else
                echo "✓ Boot entry already exists"
            fi
        else
            echo "Warning: /boot/arducam/dts not found. Arducam drivers may not be installed."
            echo "Install Arducam drivers first, then re-run this script."
        fi
        rm -f "$OVERLAY_TMP"
    else
        echo "✗ Failed to compile camera overlay"
    fi
else
    echo "Warning: Camera overlay source not found at $OVERLAY_SRC"
fi

echo -e "\n13. Verifying installations..."
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
echo "2. Navigate to VisionSense directory: cd ~/VisionSense"
echo "3. Build with: colcon build --packages-select visionconnect"
echo "4. Run with: ros2 launch visionconnect visionsense.launch.py"
echo ""
echo "If you encounter any issues, check the README.md"
