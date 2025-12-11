# VisionConnect-Plus Quick Start Guide

## For JetPack 6.2 Users - Get Running in 5 Minutes

### 1. Install Dependencies (One Command)
```bash
cd VisionConnect-Plus
./install_all_deps.sh
```

### 2. Build the Project
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select visionconnect
```

### 3. Configure Your Camera
Edit `config/config.yaml`:
```yaml
camera:
    ros__parameters:
        # For USB camera:
        resource: "v4l2:///dev/video0"
        
        # For video file (default):
        # resource: "file:///home/cw-orin/Videos/videoplayback.mp4"
```

### 4. Run the System
```bash
./run_nodes.sh
```

### 5. View the Dashboard
Open your browser to: http://localhost:8080

## Troubleshooting Quick Fixes

### Missing Nodes Error
If you see "No such file or directory" for detect/classify/etc:
```bash
# Check what was built
ls build/visionconnect/

# Clean rebuild
rm -rf build install log
colcon build --packages-select visionconnect
```

### Camera Not Working
```bash
# List available cameras
v4l2-ctl --list-devices

# Test USB camera
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

### Library Errors
```bash
# Update library paths
export LD_LIBRARY_PATH=/usr/local/lib:/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
sudo ldconfig
```

## What's Next?

1. Download TensorRT models and place in project root
2. Customize detection thresholds in config.yaml
3. Check README.md for detailed documentation
4. See INSTALL_JETPACK_6.2.md for troubleshooting