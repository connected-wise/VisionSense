# OpenCV CUDA Installation Steps

## Important: Run these commands in your terminal

The installation script requires sudo access. Please run the following command in your terminal:

```bash
./install_opencv_cuda_orin.sh
```

When prompted for password, enter your sudo password.

## What the script will do:

1. **Setup CUDA paths** - Configure system to find CUDA libraries
2. **Install dependencies** - Install all required libraries (may take 10-15 minutes)
3. **Download OpenCV** - Get latest OpenCV and opencv_contrib from GitHub
4. **Configure build** - Setup CMake with CUDA support for Jetson Orin
5. **Compile OpenCV** - Build with 4 cores (will take 2-3 hours)
6. **Install OpenCV** - Install to /usr directory
7. **Verify installation** - Check that CUDA support is enabled

## Alternative: Run steps manually

If you prefer to run steps manually or if the script fails:

### Step 1: Setup CUDA
```bash
sudo sh -c "echo '/usr/local/cuda/lib64' >> /etc/ld.so.conf.d/nvidia-tegra.conf"
sudo ldconfig
```

### Step 2: Install dependencies
```bash
sudo apt-get update
sudo apt-get install -y cmake libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev
# (run all apt-get commands from the script)
```

### Step 3: Download OpenCV
```bash
cd ~
git clone --depth=1 https://github.com/opencv/opencv.git
git clone --depth=1 https://github.com/opencv/opencv_contrib.git
```

### Step 4: Build OpenCV
```bash
cd ~/opencv
mkdir build && cd build
cmake [all the flags from the script]
make -j4
sudo make install
sudo ldconfig
```

### Step 5: Verify
```bash
opencv_version
python3 -c "import cv2; print('CUDA:', cv2.cuda.getCudaEnabledDeviceCount() > 0)"
```

## After Installation

Once OpenCV is installed with CUDA support:

1. Go back to VisionConnect-Plus directory:
   ```bash
   cd ~/VisionConnect-Plus
   ```

2. Rebuild the ROS package:
   ```bash
   colcon build --packages-select visionconnect --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. Test the application with preview:
   ```bash
   source install/setup.bash
   ros2 launch visionconnect simple_test.launch.py
   ```

The preview window should now display with object detection overlays!