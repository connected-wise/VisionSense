#!/bin/bash

# Install OpenCV with CUDA support for JetPack 6.2 on Jetson Orin
# This script builds OpenCV 4.8.0 with CUDA, cuDNN, and TensorRT support

set -e

echo "=== OpenCV with CUDA Installation Script for JetPack 6.2 ==="
echo "This will build OpenCV 4.8.0 with CUDA support"
echo ""

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "Warning: This doesn't appear to be a Jetson device"
fi

# Set variables
OPENCV_VERSION=4.8.0
INSTALL_PREFIX=/usr/local
BUILD_DIR=$HOME/opencv_build
CUDA_ARCH_BIN="8.7"  # For Jetson Orin (compute capability 8.7)
NUM_JOBS=$(nproc)

echo "Configuration:"
echo "- OpenCV Version: $OPENCV_VERSION"
echo "- Install Prefix: $INSTALL_PREFIX"
echo "- CUDA Architecture: $CUDA_ARCH_BIN"
echo "- Parallel Jobs: $NUM_JOBS"
echo ""

# Install dependencies
echo "Installing dependencies..."
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libjpeg-dev \
    libtiff-dev \
    libpng-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    python3-dev \
    python3-numpy \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev \
    libopenexr-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libopenblas-dev \
    liblapack-dev \
    liblapacke-dev \
    libeigen3-dev \
    libhdf5-dev \
    protobuf-compiler \
    libprotobuf-dev \
    libgoogle-glog-dev \
    libgflags-dev

# Create build directory
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# Clone OpenCV and OpenCV contrib
echo ""
echo "Cloning OpenCV repositories..."
if [ ! -d "opencv" ]; then
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout $OPENCV_VERSION
    cd ..
fi

if [ ! -d "opencv_contrib" ]; then
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv_contrib
    git checkout $OPENCV_VERSION
    cd ..
fi

# Create build directory
cd opencv
mkdir -p build
cd build

# Configure OpenCV with CUDA support
echo ""
echo "Configuring OpenCV with CUDA support..."
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D ENABLE_FAST_MATH=1 \
      -D CUDA_FAST_MATH=1 \
      -D CUDA_ARCH_BIN=$CUDA_ARCH_BIN \
      -D WITH_CUBLAS=1 \
      -D WITH_LIBV4L=ON \
      -D WITH_V4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_TBB=ON \
      -D WITH_OPENMP=ON \
      -D WITH_IPP=ON \
      -D WITH_EIGEN=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_DOCS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_opencv_java=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=ON \
      -D PYTHON3_EXECUTABLE=$(which python3) \
      -D CMAKE_CXX_FLAGS="-march=native" \
      -D CMAKE_C_FLAGS="-march=native" \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      ..

# Show configuration summary
echo ""
echo "Configuration Summary:"
echo "====================="
grep -E "CUDA|cuDNN|TensorRT" CMakeCache.txt | grep -v "^//" | sort | uniq

echo ""
echo "OpenCV modules to be built:"
grep "To be built" ../cmake_output.txt 2>/dev/null || true

# Ask for confirmation
echo ""
read -p "Do you want to proceed with the build? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Build cancelled."
    exit 1
fi

# Build OpenCV
echo ""
echo "Building OpenCV (this will take 1-2 hours)..."
make -j$NUM_JOBS

# Install OpenCV
echo ""
echo "Installing OpenCV (requires sudo)..."
sudo make install
sudo ldconfig

# Update Python path
echo ""
echo "Updating Python path..."
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
if [ -d "$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages/cv2" ]; then
    echo "export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:\$PYTHONPATH" >> ~/.bashrc
fi

# Verify installation
echo ""
echo "Verifying installation..."
opencv_version -v | grep -i cuda || echo "Warning: CUDA support not detected"

# Create a test script
cat > $BUILD_DIR/test_opencv_cuda.py << 'EOF'
#!/usr/bin/env python3
import cv2
import numpy as np

print(f"OpenCV Version: {cv2.__version__}")
print(f"CUDA Available: {cv2.cuda.getCudaEnabledDeviceCount() > 0}")
print(f"CUDA Device Count: {cv2.cuda.getCudaEnabledDeviceCount()}")

# Test CUDA functionality
if cv2.cuda.getCudaEnabledDeviceCount() > 0:
    # Create a test image
    cpu_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Upload to GPU
    gpu_img = cv2.cuda_GpuMat()
    gpu_img.upload(cpu_img)
    
    # Perform a simple operation
    gpu_result = cv2.cuda.resize(gpu_img, (320, 240))
    
    # Download result
    result = gpu_result.download()
    
    print(f"CUDA resize test: {'PASSED' if result.shape == (240, 320, 3) else 'FAILED'}")
else:
    print("No CUDA devices found!")

# Print build information
print("\nBuild Information:")
print(cv2.getBuildInformation())
EOF

chmod +x $BUILD_DIR/test_opencv_cuda.py

echo ""
echo "=== Installation Complete ==="
echo ""
echo "To test the installation, run:"
echo "  python3 $BUILD_DIR/test_opencv_cuda.py"
echo ""
echo "To use OpenCV in your project:"
echo "  - C++: pkg-config --cflags --libs opencv4"
echo "  - Python: import cv2"
echo ""
echo "Note: You may need to log out and log back in for all changes to take effect."
echo ""

# Return to original directory
cd $HOME/VisionConnect-Plus

# Offer to rebuild the project
echo "Now you can rebuild VisionConnect-Plus with CUDA support:"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select visionconnect"