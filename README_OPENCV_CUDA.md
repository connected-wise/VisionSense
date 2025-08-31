# OpenCV with CUDA Support Required

This project requires OpenCV with CUDA support for GPU-accelerated image processing on Jetson devices.

## Current Status
The OpenCV installation on this system (4.8.0) does not have CUDA support enabled, which causes compilation errors for the following modules:
- opencv_cudaarithm
- opencv_cudawarping  
- opencv_cudaimgproc

## Solutions for JetPack 6.2

### Option 1: Use NVIDIA's Pre-built OpenCV (Recommended)
JetPack 6.2 should include OpenCV with CUDA support. Check if it's installed:
```bash
# Check for NVIDIA's OpenCV packages
dpkg -l | grep nvidia | grep opencv

# If found, you may need to switch from the standard OpenCV to NVIDIA's version
```

### Option 2: Build OpenCV from Source
If pre-built packages aren't available, build OpenCV with CUDA support:

```bash
# 1. Clone OpenCV and contrib modules
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
git checkout 4.8.0  # Match your current version

# 2. Create build directory
mkdir build && cd build

# 3. Configure with CUDA support
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D CUDA_ARCH_BIN="8.7" \  # For Jetson Orin
      -D WITH_CUBLAS=ON \
      -D BUILD_opencv_cudacodec=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=ON \
      -D CMAKE_CXX_FLAGS="-march=native" \
      ..

# 4. Build and install
make -j$(nproc)
sudo make install
sudo ldconfig
```

### Option 3: Use JetsonHacks Scripts
The JetsonHacks repository provides scripts specifically for building OpenCV on Jetson:
```bash
git clone https://github.com/jetsonhacks/buildOpenCVJetson.git
cd buildOpenCVJetson
./buildOpenCV.sh
```

## Temporary Workaround
Until OpenCV with CUDA is installed, the project will not compile successfully. The code heavily relies on GPU acceleration for:
- Image resizing and preprocessing
- Color space conversions
- Matrix operations

## Verification
After installation, verify CUDA support:
```bash
opencv_version -v | grep CUDA
# Should show CUDA: YES
```

## Note for JetPack 6.2
JetPack 6.2 is relatively new (2024), so community support and pre-built packages may still be evolving. Check NVIDIA Developer Forums for the latest information on OpenCV CUDA support for JetPack 6.2.