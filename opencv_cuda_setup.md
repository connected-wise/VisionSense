# OpenCV CUDA Setup for VisionConnect-Plus on JetPack 6.2

## Quick Start (Choose One Option)

### Option 1: Use NVIDIA L4T ML Container (Fastest - 5 minutes)
```bash
# Pull and run NVIDIA's ML container which includes OpenCV with CUDA
docker pull nvcr.io/nvidia/l4t-ml:r36.2.0-py3

# Run container with access to cameras and display
docker run -it --rm --runtime nvidia --network host \
    -v /home/cw-orin/VisionConnect-Plus:/workspace/VisionConnect-Plus \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --device /dev/video0 \
    nvcr.io/nvidia/l4t-ml:r36.2.0-py3

# Inside container, build the project
cd /workspace/VisionConnect-Plus
source /opt/ros/humble/setup.bash
colcon build --packages-select visionconnect
```

### Option 2: Install Pre-built OpenCV (If Available - 30 minutes)
```bash
# Remove current OpenCV
sudo apt remove -y libopencv libopencv-dev
sudo apt autoremove -y

# Add NVIDIA's apt repository (if not already added)
sudo apt update
sudo apt install -y nvidia-jetpack

# Install OpenCV with CUDA from JetPack
sudo apt install -y nvidia-opencv nvidia-opencv-dev

# Verify
pkg-config --modversion opencv4
```

### Option 3: Build from Source (Most Reliable - 2 hours)
```bash
# Run the automated build script
./install_opencv_cuda.sh
```

## Verification
After installation, verify CUDA support:

```python
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}, CUDA: {cv2.cuda.getCudaEnabledDeviceCount() > 0}')"
```

## Rebuild VisionConnect-Plus
Once OpenCV with CUDA is installed:

```bash
# Clean previous build
rm -rf build/visionconnect install/visionconnect

# Rebuild
source /opt/ros/humble/setup.bash
colcon build --packages-select visionconnect

# Source and run
source install/setup.bash
ros2 launch visionconnect test.launch.py
```

## Troubleshooting

### Issue: Multiple OpenCV versions
```bash
# List all OpenCV packages
dpkg -l | grep opencv

# Remove specific version
sudo apt remove libopencv-*4.5*
```

### Issue: pkg-config not finding OpenCV
```bash
# Add to ~/.bashrc
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
source ~/.bashrc
```

### Issue: Python can't find cv2
```bash
# Find cv2 location
find /usr -name "cv2*.so" 2>/dev/null

# Add to PYTHONPATH in ~/.bashrc
export PYTHONPATH=/usr/local/lib/python3.10/site-packages:$PYTHONPATH
```

## For JetPack 6.2 Specific Issues
- JetPack 6.2 uses Ubuntu 22.04 (Jammy)
- CUDA 12.2+ is included
- TensorRT 8.6+ is included
- Check NVIDIA forums for latest updates: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/