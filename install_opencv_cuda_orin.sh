#!/bin/bash

# OpenCV with CUDA installation script for Jetson Orin - JetPack 6.2
# Based on: https://qengineering.eu/install-opencv-on-orin-nano.html
# Updated for JetPack 6.2 (L4T 36.4) with CUDA 12.6
# Downloads the latest OpenCV version from GitHub

set -e

echo "=== OpenCV CUDA Installation for Jetson Orin - JetPack 6.2 ==="
echo "System: JetPack 6.2 with CUDA 12.6"
echo "This will build the latest OpenCV with CUDA support"
echo "Expected build time: 2-3 hours"
echo ""

# Step 1: Reveal the CUDA location
echo "Step 1: Setting up CUDA paths..."
sudo sh -c "echo '/usr/local/cuda/lib64' >> /etc/ld.so.conf.d/nvidia-tegra.conf" 2>/dev/null || true
sudo ldconfig

# Step 2: Install third-party libraries
echo "Step 2: Installing third-party libraries..."
sudo apt-get update

# Install dependencies exactly as specified on qengineering.eu
sudo apt-get install -y cmake
sudo apt-get install -y libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev
sudo apt-get install -y libpng-dev libtiff-dev libglew-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgtk2.0-dev libgtk-3-dev libcanberra-gtk*
sudo apt-get install -y python3-pip
sudo apt-get install -y libxvidcore-dev libx264-dev
sudo apt-get install -y libtbb-dev libdc1394-dev libxine2-dev
sudo apt-get install -y libv4l-dev v4l-utils qv4l2
sudo apt-get install -y libtesseract-dev libpostproc-dev
sudo apt-get install -y libswresample-dev libvorbis-dev
sudo apt-get install -y libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install -y libopenblas-dev libatlas-base-dev libblas-dev
sudo apt-get install -y liblapack-dev liblapacke-dev libeigen3-dev gfortran
sudo apt-get install -y libhdf5-dev libprotobuf-dev protobuf-compiler
sudo apt-get install -y libgoogle-glog-dev libgflags-dev

# Step 3: Remove existing OpenCV if present
echo "Step 3: Checking for existing OpenCV..."
if command -v opencv_version &> /dev/null; then
    CURRENT_VERSION=$(opencv_version)
    echo "Found OpenCV version: $CURRENT_VERSION"
    read -p "Remove existing OpenCV? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get remove -y '*opencv*' || true
        sudo apt-get autoremove -y
    fi
fi

# Step 4: Download OpenCV sources
echo "Step 4: Downloading the latest OpenCV version..."
cd ~
# Remove old versions if they exist
if [ -d "opencv" ]; then
    echo "Removing existing opencv directory..."
    rm -rf opencv
fi
if [ -d "opencv_contrib" ]; then
    echo "Removing existing opencv_contrib directory..."
    rm -rf opencv_contrib
fi

# Download the latest version
git clone --depth=1 https://github.com/opencv/opencv.git
git clone --depth=1 https://github.com/opencv/opencv_contrib.git

echo "Downloaded OpenCV version:"
cd opencv && git describe --tags && cd ..

# Step 5: Configure build
echo "Step 5: Configuring OpenCV build with CUDA..."
cd ~/opencv
rm -rf build
mkdir build
cd build

# Configure with CUDA support for Orin (compute capability 8.7)
# Using exact configuration from qengineering.eu for JetPack 6.2
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
    -D WITH_OPENCL=OFF \
    -D CUDA_ARCH_BIN=8.7 \
    -D CUDA_ARCH_PTX="sm_87" \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D WITH_CUBLAS=ON \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_NEON=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENMP=ON \
    -D BUILD_TIFF=ON \
    -D WITH_FFMPEG=ON \
    -D WITH_GSTREAMER=ON \
    -D WITH_TBB=ON \
    -D BUILD_TBB=ON \
    -D BUILD_TESTS=OFF \
    -D WITH_EIGEN=ON \
    -D WITH_V4L=ON \
    -D WITH_LIBV4L=ON \
    -D WITH_PROTOBUF=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=OFF \
    ..

# Step 6: Build OpenCV
echo "Step 6: Building OpenCV (this will take 2-3 hours)..."
echo "Using 4 cores for compilation to avoid memory issues"
make -j4

# Step 7: Install OpenCV
echo "Step 7: Installing OpenCV..."
sudo make install
sudo ldconfig

# Step 8: Verify installation
echo "Step 8: Verifying installation..."
echo "OpenCV version:"
opencv_version

echo ""
echo "Checking CUDA support in Python:"
python3 -c "
import cv2
print('OpenCV version:', cv2.__version__)
print('CUDA devices:', cv2.cuda.getCudaEnabledDeviceCount())
print('CUDA available:', cv2.cuda.getCudaEnabledDeviceCount() > 0)
"

echo ""
echo "=== OpenCV with CUDA installation complete! ==="
echo "You may need to rebuild VisionConnect-Plus to use the new OpenCV."