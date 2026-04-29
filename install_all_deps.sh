#!/bin/bash
# Comprehensive installation script for VisionSense on JetPack 6.2
# Run with: bash install_all_deps.sh

set -e  # Exit on error

# Resolve the directory containing this script regardless of how it's invoked.
# We `cd` later, so this must be captured up front and from $BASH_SOURCE
# (which works under sourcing too) rather than $0.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gpsd \
    gpsd-clients \
    libgps-dev \
    v4l-utils \
    libopencv-dev

echo -e "\n7. Verifying OpenCV..."
# VisionSense uses the system OpenCV that ships with JetPack 6.2 (no CUDA
# modules). The cv::cuda::* ops the older codebase relied on were replaced
# with raw CUDA kernels in src/cuda/preprocess.cu and src/cuda/stereo_preproc.cu,
# so an OpenCV-from-source build is no longer required.
if [ -f "/usr/include/opencv4/opencv2/core/version.hpp" ]; then
    OPENCV_VER=$(grep '#define CV_VERSION_' /usr/include/opencv4/opencv2/core/version.hpp \
        | awk '{print $3}' | head -3 | paste -sd. | tr -d '"')
    echo "✓ OpenCV $OPENCV_VER (system, libopencv-dev)"
else
    echo "✗ libopencv-dev did not install OpenCV headers — aborting"
    exit 1
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

echo -e "\n9. Configuring gpsd for GPS module..."
if [ -f /etc/default/gpsd ]; then
    # Configure gpsd to use the Jetson UART connected to GPS module
    if ! grep -q "ttyTHS1" /etc/default/gpsd; then
        echo "Configuring gpsd for /dev/ttyTHS1..."
        sudo tee /etc/default/gpsd > /dev/null << 'GPSDCONF'
# Devices gpsd should collect to at boot time.
# They need to be read/writeable, either by user gpsd or the group dialout.
DEVICES="/dev/ttyTHS1"

# Other options you want to pass to gpsd
GPSD_OPTIONS=""

# Automatically hot add/remove USB GPS devices via gpsdctl
USBAUTO="true"
GPSDCONF
    else
        echo "✓ gpsd already configured for /dev/ttyTHS1"
    fi
    sudo systemctl enable gpsd gpsd.socket || echo "  (gpsd enable failed — continuing; safe if no GPS hardware)"
    sudo systemctl restart gpsd gpsd.socket || echo "  (gpsd restart failed — continuing; safe if no GPS hardware)"
    echo "✓ gpsd configured (service may be inactive without GPS hardware)"
else
    echo "Warning: /etc/default/gpsd not found. gpsd may not be installed correctly."
fi

echo -e "\n10. Installing Python packages..."
pip3 install --upgrade pip
pip3 install numpy pyserial

echo -e "\n11. Fixing npymath library for newer NumPy versions..."
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

echo -e "\n12. Checking jetson-inference installation..."
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

echo -e "\n13. Installing Arducam camera drivers..."
if [ ! -d "/boot/arducam" ]; then
    echo "Arducam drivers not found. Installing Jetvariety driver..."
    ARDUCAM_TMP="/tmp/arducam_install"
    rm -rf "$ARDUCAM_TMP"
    mkdir -p "$ARDUCAM_TMP"
    cd "$ARDUCAM_TMP"
    wget -q https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh -O install_full.sh || {
        echo "Primary download failed, trying git clone..."
        cd /tmp
        rm -rf MIPI_Camera
        git clone --depth=1 https://github.com/ArduCAM/MIPI_Camera.git
        cp MIPI_Camera/Jetson/Jetvariety/install_driver.sh "$ARDUCAM_TMP/install_full.sh"
        cd "$ARDUCAM_TMP"
    }
    chmod +x install_full.sh
    # Arducam's installer asks "Continue?" interactively. Pipe "n" so it
    # doesn't block; the kernel/dtb pieces it offers to swap may already be
    # in place and skipping that prompt is the right default.
    echo "n" | sudo ./install_full.sh -m arducam_4lane || \
        echo "  (Arducam installer returned non-zero — may be a pre-existing kernel mismatch; check /boot/arducam manually)"
    cd "$SCRIPT_DIR"
    rm -rf "$ARDUCAM_TMP" /tmp/MIPI_Camera
    echo "✓ Arducam drivers installed (reboot required after script completes)"
else
    echo "✓ Arducam drivers already installed"
fi

echo -e "\n14. Installing camera device tree overlay..."
# Combined overlay for AR0234 stereo camera + IMX219 rear camera.
# Works on every Orin variant the AR0234+IMX219 combo runs on:
#   - Orin Nano 4GB / 8GB / 8GB Super, Orin NX 8GB / 16GB
#   - Rootfs on either SD card (/dev/mmcblk0p1) or NVMe (root=PARTUUID=...)
# The trick is: don't hardcode root=, kernel flags, or FDT. Derive everything
# from the primary boot entry that's already working on this machine.
OVERLAY_SRC="${SCRIPT_DIR}/overlays/tegra234-p3767-camera-p3768-arducam-imx219-combined.dts"

if [ -f "$OVERLAY_SRC" ]; then
    echo "Compiling camera overlay..."
    OVERLAY_TMP="/tmp/tegra234-p3767-camera-p3768-arducam-imx219-combined.dtbo"
    dtc -I dts -O dtb -o "$OVERLAY_TMP" "$OVERLAY_SRC" 2>/dev/null

    if [ -f "$OVERLAY_TMP" ]; then
        # Determine overlay destination - prefer Arducam dir, fall back to /boot
        if [ -d "/boot/arducam/dts" ]; then
            OVERLAY_DST="/boot/arducam/dts/tegra234-p3767-camera-p3768-arducam-imx219-combined.dtbo"
        else
            sudo mkdir -p /boot/overlays
            OVERLAY_DST="/boot/overlays/tegra234-p3767-camera-p3768-arducam-imx219-combined.dtbo"
        fi

        sudo cp "$OVERLAY_TMP" "$OVERLAY_DST"
        echo "✓ Camera overlay installed to $OVERLAY_DST"

        # Always back up extlinux.conf before any edit. Past install runs of
        # this script wrote entries that prevented boot — hardcoded
        # /dev/mmcblk0p1 on NVMe-rooted systems, missing nvme.use_threaded_interrupts=1,
        # wrong-module FDT. Cheap insurance.
        BACKUP="/boot/extlinux/extlinux.conf.bak.$(date +%Y%m%d-%H%M%S)"
        sudo cp /boot/extlinux/extlinux.conf "$BACKUP"
        echo "  Backup: $BACKUP"

        # Add ArducamIMX219 entry if it doesn't exist
        if ! grep -q "ArducamIMX219" /boot/extlinux/extlinux.conf; then
            echo "Adding boot entry for combined camera overlay..."

            # Inherit the APPEND line from the "primary" entry verbatim. This
            # is what makes the script work on both SD-rooted and NVMe-rooted
            # devices: primary's APPEND already contains the right root= for
            # this filesystem (root=/dev/mmcblk0p1 on SD, root=PARTUUID=... on
            # NVMe) and the right kernel flags (NVMe-rooted boards also need
            # nvme.use_threaded_interrupts=1 + nv-auto-config to init reliably).
            # Hand-rolling these per-board is how installs break.
            PRIMARY_APPEND=$(awk '
                /^LABEL primary/ { in_primary=1; next }
                /^LABEL / { in_primary=0 }
                in_primary && /^[[:space:]]*APPEND/ {
                    sub(/^[[:space:]]*APPEND[[:space:]]*/, "")
                    print
                    exit
                }' /boot/extlinux/extlinux.conf)

            # FDT is intentionally NOT set: we rely on the bootloader's
            # EEPROM-based DTB auto-detection so the entry works on NX 16GB,
            # NX 8GB, Nano 8GB Super, etc. without any per-module fork.
            # (Hardcoding p3767-0005-nv-super.dtb on NX hardware silently
            # mis-identifies the SoC as Orin Nano and caps GPU clocks.)
            if [ -f "/boot/arducam/Image" ]; then
                KERNEL_PATH="/boot/arducam/Image"
            else
                KERNEL_PATH="/boot/Image"
            fi

            if [ -z "$PRIMARY_APPEND" ]; then
                # Fallback for unusual extlinux layouts where there's no
                # LABEL primary or its APPEND line is empty. Use the
                # documented Jetson default that works on both rootfs types
                # (cbootargs supplies root= via the bootloader cmdline).
                echo "  Note: could not extract APPEND from primary; using \${cbootargs} fallback."
                PRIMARY_APPEND='${cbootargs} rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0'
            fi

            sudo tee -a /boot/extlinux/extlinux.conf > /dev/null << BOOTENTRY

LABEL ArducamIMX219
	MENU LABEL Custom Header Config: <AR0234 + IMX219 Combined>
	LINUX ${KERNEL_PATH}
	INITRD /boot/initrd
	APPEND ${PRIMARY_APPEND}
	OVERLAYS ${OVERLAY_DST}
BOOTENTRY
            echo "✓ Boot entry added (APPEND inherited from primary, FDT auto-detected)"
        else
            echo "✓ Boot entry already exists"
        fi

        # Set ArducamIMX219 as default boot entry
        if grep -q "^DEFAULT " /boot/extlinux/extlinux.conf; then
            sudo sed -i 's/^DEFAULT .*/DEFAULT ArducamIMX219/' /boot/extlinux/extlinux.conf
            echo "✓ Default boot entry set to ArducamIMX219"
        fi

        rm -f "$OVERLAY_TMP"
    else
        echo "✗ Failed to compile camera overlay"
    fi
else
    echo "Warning: Camera overlay source not found at $OVERLAY_SRC"
fi

echo -e "\n15. Verifying installations..."
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
echo "1. REBOOT to load the camera device tree overlay"
echo "2. Close and reopen your terminal (or run: source ~/.bashrc)"
echo "3. Navigate to VisionSense directory: cd ~/VisionSense"
echo "4. Regenerate TensorRT engines for this device's GPU+TRT version:"
echo "     bash scripts/regenerate_engines.sh"
echo "   (~20 min total; required because checked-in .engine files are not"
echo "    portable across Jetsons. Backups of the previous engines are kept"
echo "    as <name>.prev.<TIMESTAMP> so a partial run never leaves you broken.)"
echo "5. Build with: colcon build --packages-select visionconnect"
echo "6. Run with: ros2 launch visionconnect visionsense.launch.py"
echo ""
echo "If you encounter any issues, check the README.md"
