#!/bin/bash
# Comprehensive installation script for VisionSense on JetPack 6.2 (Ubuntu
# 22.04, ROS 2 Humble) and JetPack 7.2 (Ubuntu 24.04, ROS 2 Jazzy).
# Run with: bash install_all_deps.sh
#
# The driver-monitor camera on CAM0 picks the device-tree overlay:
#   VS_DRIVER_CAM=imx219 (default) or VS_DRIVER_CAM=imx477, e.g.
#   VS_DRIVER_CAM=imx477 bash install_all_deps.sh

set -e  # Exit on error

# Resolve the directory containing this script regardless of how it's invoked.
# We `cd` later, so this must be captured up front and from $BASH_SOURCE
# (which works under sourcing too) rather than $0.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "======================================"
echo "VisionConnect-Plus Dependency Installer"
echo "For JetPack 6.2 / 7.2 on NVIDIA Jetson"
echo "======================================"

# The ROS 2 distro follows the Ubuntu release JetPack ships.
UBUNTU_CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
case "$UBUNTU_CODENAME" in
    jammy) ROS_DISTRO=humble ;;   # JetPack 6.2, L4T R36
    noble) ROS_DISTRO=jazzy ;;    # JetPack 7.2, L4T R39
    *)
        echo "Unsupported Ubuntu release '$UBUNTU_CODENAME' (expected jammy or noble)"
        exit 1
        ;;
esac
echo "Ubuntu $UBUNTU_CODENAME -> ROS 2 $ROS_DISTRO"

VS_DRIVER_CAM="${VS_DRIVER_CAM:-imx219}"
case "$VS_DRIVER_CAM" in
    imx219|imx477) ;;
    *)
        echo "VS_DRIVER_CAM must be imx219 or imx477 (got '$VS_DRIVER_CAM')"
        exit 1
        ;;
esac
echo "Driver-monitor camera on CAM0: $VS_DRIVER_CAM"

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
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        # ROS setup scripts reference unset variables; don't let set -u-style
        # callers or our own set -e trip on them.
        set +e
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        set -e
    fi
}

echo -e "\n1. Updating package lists..."
sudo apt update

echo -e "\n2. Installing ROS2 ${ROS_DISTRO^}..."
if ! check_package ros-$ROS_DISTRO-desktop; then
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-desktop
fi

echo -e "\n3. Installing ROS2 additional packages..."
sudo apt install -y \
    ros-$ROS_DISTRO-vision-msgs \
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
    python3-serial \
    python3-ament-package \
    ros-$ROS_DISTRO-rosidl-generator-py \
    ros-$ROS_DISTRO-rosidl-typesupport-introspection-cpp \
    ros-$ROS_DISTRO-rosidl-typesupport-introspection-c

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
    libopencv-dev \
    libwebsockets-dev \
    libyaml-cpp-dev \
    device-tree-compiler

echo -e "\n7. Verifying OpenCV..."
# VisionSense uses the system OpenCV that ships with JetPack (4.8, no CUDA
# modules). The cv::cuda::* ops the older codebase relied on were replaced
# with raw CUDA kernels in src/cuda/preprocess.cu and src/cuda/stereo_rotate.cu,
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
# /usr/local/cuda points at the CUDA JetPack installed (12.6 on JetPack 6.2,
# 13.x on JetPack 7.2). Guarded so re-running the installer doesn't stack
# duplicate blocks in ~/.bashrc.
if ! grep -q "# VisionSense environment" ~/.bashrc; then
cat >> ~/.bashrc << EOL

# VisionSense environment
if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# CUDA
export PATH=/usr/local/cuda/bin:\$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH

# Jetson libraries
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:\$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH
EOL
else
    echo "✓ ~/.bashrc already has the VisionSense environment block"
fi

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
# numpy and pyserial come from apt (step 5: python3-numpy, python3-serial).
# Ubuntu 24.04 marks the system Python as externally managed, so a system-wide
# `pip3 install` fails there; apt works on both JetPack releases.
python3 -c "import numpy, serial; print('✓ numpy', numpy.__version__, '/ pyserial', serial.VERSION)"

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
    if [ "$UBUNTU_CODENAME" = "noble" ]; then
        # JetPack 7.2 (Ubuntu 24.04, GCC 13, CUDA 13) needs a few build fixes;
        # see the script for the list.
        bash "${SCRIPT_DIR}/scripts/patch_jetson_inference_cuda13.sh" "$PWD"
    fi
    mkdir -p build && cd build
    cmake -DBUILD_INTERACTIVE=NO ../
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

    # On L4T R39 (JetPack 7.2) Arducam's package list has arducam_4lane builds
    # only under Orin NX, so install_full.sh rejects an Orin Nano. The Orin NX
    # and Orin Nano share the T234 SoC and the L4T kernel, and the package is
    # keyed by the exact nvidia-l4t-kernel version, so install that build
    # directly. (On R36 the Orin Nano entry already points at the same
    # t234-nx package.)
    if [ ! -f /boot/arducam/Image ]; then
        L4T_KERNEL=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-kernel)
        wget -q -O arducam_4lane_links.txt \
            https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/arducam_4lane_links.txt || true
        ARDUCAM_DEB_URL=$(grep -F "orin_nx_download_links[\"$L4T_KERNEL\"]" arducam_4lane_links.txt 2>/dev/null \
            | head -1 | sed -E 's/.*="([^"]+)".*/\1/')
        if [ -n "$ARDUCAM_DEB_URL" ]; then
            echo "  Installing Arducam's T234 build for kernel $L4T_KERNEL:"
            echo "    $ARDUCAM_DEB_URL"
            wget -q "$ARDUCAM_DEB_URL" -O arducam-4lane.deb && sudo dpkg -i arducam-4lane.deb || \
                echo "  ✗ Arducam package install failed"
        else
            echo "  ✗ No Arducam arducam_4lane package for kernel $L4T_KERNEL"
        fi
    fi
    cd "$SCRIPT_DIR"
    rm -rf "$ARDUCAM_TMP" /tmp/MIPI_Camera
    echo "✓ Arducam drivers installed (reboot required after script completes)"
else
    echo "✓ Arducam drivers already installed"
fi

echo -e "\n14. Installing camera device tree overlay..."
# Combined overlay for the AR0234 stereo camera (CAM1) + the driver-monitor
# camera on CAM0, picked with VS_DRIVER_CAM:
#   imx219 -> overlays/tegra234-p3767-camera-p3768-arducam-imx219-combined.dts
#   imx477 -> overlays/tegra234-p3767-camera-p3768-arducam-imx477-combined.dts
# Works on every Orin variant the combo runs on:
#   - Orin Nano 4GB / 8GB / 8GB Super, Orin NX 8GB / 16GB
#   - Rootfs on either SD card (/dev/mmcblk0p1) or NVMe (root=PARTUUID=...)
# The trick is: don't hardcode root= or kernel flags. Derive them from the
# primary boot entry that's already working on this machine.
OVERLAY_NAME="tegra234-p3767-camera-p3768-arducam-${VS_DRIVER_CAM}-combined"
OVERLAY_SRC="${SCRIPT_DIR}/overlays/${OVERLAY_NAME}.dts"
BOOT_LABEL="Arducam${VS_DRIVER_CAM^^}"     # ArducamIMX219 / ArducamIMX477

if [ -f "$OVERLAY_SRC" ]; then
    echo "Compiling camera overlay..."
    OVERLAY_TMP="/tmp/${OVERLAY_NAME}.dtbo"
    dtc -I dts -O dtb -o "$OVERLAY_TMP" "$OVERLAY_SRC" 2>/dev/null

    if [ -f "$OVERLAY_TMP" ]; then
        # Determine overlay destination - prefer Arducam dir, fall back to /boot
        if [ -d "/boot/arducam/dts" ]; then
            OVERLAY_DST="/boot/arducam/dts/${OVERLAY_NAME}.dtbo"
        else
            sudo mkdir -p /boot/overlays
            OVERLAY_DST="/boot/overlays/${OVERLAY_NAME}.dtbo"
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

        # Add the boot entry if it doesn't exist
        if ! grep -q "^LABEL ${BOOT_LABEL}\$" /boot/extlinux/extlinux.conf; then
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

            # FDT: the UEFI L4TLauncher only applies OVERLAYS on top of an
            # explicit FDT; without one it boots the DTB from the bootloader
            # and silently skips the overlay (seen on L4T R36.5 and R39.2).
            # Use the DTB that flashing installed under /boot/dtb/ for this
            # exact module: it's always the right one for this board, so this
            # doesn't hardcode a Nano DTB onto an NX (which would mis-identify
            # the SoC and cap GPU clocks).
            FDT_LINE=""
            BOARD_DTBS=(/boot/dtb/kernel_*.dtb)
            if [ ${#BOARD_DTBS[@]} -eq 1 ] && [ -f "${BOARD_DTBS[0]}" ]; then
                FDT_LINE="FDT ${BOARD_DTBS[0]}"
            else
                echo "  Note: expected one /boot/dtb/kernel_*.dtb, found ${#BOARD_DTBS[@]}; leaving FDT unset."
            fi

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

LABEL ${BOOT_LABEL}
	MENU LABEL Custom Header Config: <AR0234 + ${VS_DRIVER_CAM^^} Combined>
	LINUX ${KERNEL_PATH}
	${FDT_LINE}
	INITRD /boot/initrd
	APPEND ${PRIMARY_APPEND}
	OVERLAYS ${OVERLAY_DST}
BOOTENTRY
            echo "✓ Boot entry ${BOOT_LABEL} added (APPEND inherited from primary)"
        else
            echo "✓ Boot entry ${BOOT_LABEL} already exists"
        fi

        # Set the combined entry as default boot entry
        if grep -q "^DEFAULT " /boot/extlinux/extlinux.conf; then
            sudo sed -i "s/^DEFAULT .*/DEFAULT ${BOOT_LABEL}/" /boot/extlinux/extlinux.conf
            echo "✓ Default boot entry set to ${BOOT_LABEL}"
        fi

        rm -f "$OVERLAY_TMP"
    else
        echo "✗ Failed to compile camera overlay"
    fi
else
    echo "Warning: Camera overlay source not found at $OVERLAY_SRC"
fi

echo "Installing persistent camera names (/dev/video-mono, /dev/video-stereo)..."
sudo install -m 0644 "${SCRIPT_DIR}/scripts/99-camera-persistent.rules" \
    /etc/udev/rules.d/99-camera-persistent.rules
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=video4linux || true
echo "✓ udev rules installed"

echo -e "\n15. Configuring kernel UDP buffers for Fast-DDS..."
# A single 1200×1200×3 stereo image is 4.32 MB. With Fast-DDS LARGE_DATA mode
# (used by launch_visionsense.sh) or any UDP fallback, this fragments into
# ~67 UDP packets and immediately overflows the kernel's default ~208 KB
# socket buffer — the symptom is /camera_stereo/right/image_raw dropping to
# ~2 Hz while left holds 30 Hz. Bump rmem/wmem to 16 MB.
FASTDDS_SYSCTL=/etc/sysctl.d/99-ros2-fastdds.conf
DESIRED_SYSCTL_CONTENT=$(cat <<'SYSCTLCONF'
# Larger UDP socket buffers for ROS2/Fast-DDS image streaming.
# Default is 208 KB; a single 1200x1200x3 stereo image is 4.32 MB,
# which fragments into ~67 UDP packets and overflows the default buffer.
# Required for Fast-DDS LARGE_DATA mode (set in launch_visionsense.sh)
# and any UDP-transport fallback.
net.core.rmem_max = 16777216
net.core.rmem_default = 16777216
net.core.wmem_max = 16777216
net.core.wmem_default = 16777216
SYSCTLCONF
)

if [ -f "$FASTDDS_SYSCTL" ] && [ "$(cat "$FASTDDS_SYSCTL")" = "$DESIRED_SYSCTL_CONTENT" ]; then
    echo "✓ $FASTDDS_SYSCTL already up to date"
else
    echo "Writing $FASTDDS_SYSCTL ..."
    echo "$DESIRED_SYSCTL_CONTENT" | sudo tee "$FASTDDS_SYSCTL" > /dev/null
    sudo sysctl --system > /dev/null 2>&1 || echo "  (sysctl --system reload failed; values will apply at next boot)"
    echo "✓ Kernel UDP buffers set to 16 MB"
fi

echo -e "\n16. Configuring nvargus-daemon for clean camera teardown..."
# Workaround for L4T R36.x bug where nvargus force-destroys the CameraProvider
# after 1500 ms of pending requests on disconnect. That force-destruction leaks
# vb2 buffers in the VI channel and wedges /dev/video0 until reboot. Setting
# enableCamInfiniteTimeout=1 disables the timeout so the userspace shutdown
# sequence finishes cleanly. See:
# https://forums.developer.nvidia.com/t/imx219-camera-works-the-first-time-then-fails-thereafter/363291
NVARGUS_DROPIN_DIR=/etc/systemd/system/nvargus-daemon.service.d
sudo mkdir -p "$NVARGUS_DROPIN_DIR"
sudo tee "$NVARGUS_DROPIN_DIR/override.conf" > /dev/null << 'NVARGUS_OVR'
[Service]
Environment="enableCamInfiniteTimeout=1"
NVARGUS_OVR
sudo systemctl daemon-reload
sudo systemctl restart nvargus-daemon || echo "  (nvargus-daemon restart failed — continuing)"
echo "✓ nvargus-daemon override installed"

echo -e "\n17. Installing persistent IMX219 driver-monitor camera service..."
# The Jetson Argus stack wedges on Producer close on this rig — every second
# nvarguscamerasrc session fails until reboot (bare gst-launch reproduces it).
# Workaround: keep one long-lived camera session alive for the whole uptime via
# systemd, and have VisionSense subscribe to /camera/raw as a normal ROS topic
# instead of opening Argus itself. Stopping/starting VisionSense never touches
# the camera. See scripts/visionsense-imx219.service for the unit and
# scripts/imx219-csi-warmup.sh for the boot-time Arducam V4L2 warmup that keeps
# Argus' sensor-1 probe from poisoning IMX219 capture on fresh boot.
IMX219_UNIT_SRC="${SCRIPT_DIR}/scripts/visionsense-imx219.service"
IMX219_UNIT_DST=/etc/systemd/system/visionsense-imx219.service
IMX219_WARMUP_SRC="${SCRIPT_DIR}/scripts/imx219-csi-warmup.sh"
if [ -f "$IMX219_UNIT_SRC" ] && [ -f "$IMX219_WARMUP_SRC" ]; then
    # The unit in the repo is written for user jetson, /home/jetson/VisionSense
    # and Humble; fill in this machine's user, checkout and ROS distro.
    VS_USER="$(id -un)"
    VS_GROUP="$(id -gn)"
    sed -e "s#/home/jetson/VisionSense#${SCRIPT_DIR}#g" \
        -e "s#^User=jetson#User=${VS_USER}#" \
        -e "s#^Group=jetson#Group=${VS_GROUP}#" \
        -e "s#/opt/ros/humble/#/opt/ros/${ROS_DISTRO}/#g" \
        "$IMX219_UNIT_SRC" > /tmp/visionsense-imx219.service
    sudo install -m 0644 /tmp/visionsense-imx219.service "$IMX219_UNIT_DST"
    rm -f /tmp/visionsense-imx219.service
    sudo chmod +x "$IMX219_WARMUP_SRC"
    sudo systemctl daemon-reload
    sudo systemctl enable visionsense-imx219.service || true
    echo "✓ visionsense-imx219.service installed and enabled (starts at next boot)"
    # Passwordless sudo for the launch script's re-kick of this one unit.
    SUDOERS_SRC="${SCRIPT_DIR}/scripts/sudoers.visionsense-imx219"
    SUDOERS_DST=/etc/sudoers.d/visionsense-imx219
    if [ -f "$SUDOERS_SRC" ]; then
        sed "s#^jetson #${VS_USER} #" "$SUDOERS_SRC" > /tmp/sudoers.visionsense-imx219
        sudo install -m 0440 -o root -g root /tmp/sudoers.visionsense-imx219 "$SUDOERS_DST"
        rm -f /tmp/sudoers.visionsense-imx219
        if sudo visudo -c -f "$SUDOERS_DST" >/dev/null 2>&1; then
            echo "✓ /etc/sudoers.d/visionsense-imx219 installed (passwordless re-kick)"
        else
            sudo rm -f "$SUDOERS_DST"
            echo "✗ sudoers file failed visudo -c — removed; launch script will prompt for password"
        fi
    fi
else
    echo "  (scripts/visionsense-imx219.service or imx219-csi-warmup.sh missing — skipping)"
fi

echo -e "\n18. Verifying installations..."
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
echo "3. Navigate to VisionSense directory: cd $SCRIPT_DIR"
echo "4. Rebuild the TensorRT engines for this Jetson's TensorRT version (needed"
echo "   after every flash / JetPack change; engines are not portable):"
echo "     bash scripts/regenerate_engines.sh"
echo "5. Build with: colcon build --packages-select visionconnect"
echo "6. Run with: bash launch_visionsense.sh   (or double-click the desktop icon)"
echo "   The launch script exports FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA, which is"
echo "   required to keep both stereo eyes at 30 Hz; running 'ros2 launch ...'"
echo "   directly will skip that and one eye will drop to ~2 Hz."
echo ""
echo "If you encounter any issues, check the README.md"
