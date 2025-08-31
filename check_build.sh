#!/bin/bash
# Build diagnostic script for VisionConnect-Plus

echo "=== VisionConnect-Plus Build Diagnostics ==="
echo ""

# Check built executables
echo "1. Checking built executables:"
echo "------------------------------"
for node in camera detect classify lanedet preview dashboard gui; do
    if [ -f "build/visionconnect/$node" ]; then
        echo "✓ $node - built successfully"
    else
        echo "✗ $node - NOT BUILT"
    fi
done

# Check libraries
echo -e "\n2. Checking built libraries:"
echo "------------------------------"
if [ -f "build/visionconnect/libcommon.a" ]; then
    echo "✓ libcommon.a - built successfully"
else
    echo "✗ libcommon.a - NOT BUILT"
fi

# Check ROS2 message libraries
echo -e "\n3. Checking ROS2 message libraries:"
echo "------------------------------"
for lib in libvisionconnect__rosidl_generator_c.so libvisionconnect__rosidl_typesupport_cpp.so; do
    if [ -f "build/visionconnect/$lib" ]; then
        echo "✓ $lib - built"
    else
        echo "✗ $lib - NOT BUILT"
    fi
done

# Check model files
echo -e "\n4. Checking required model files:"
echo "------------------------------"
for model in yolov8s-detect.engine yolov8n-TSR.engine lane_detect.engine; do
    if [ -f "$model" ]; then
        echo "✓ $model - found"
    else
        echo "✗ $model - MISSING (download required)"
    fi
done

# Check label files
echo -e "\n5. Checking label files:"
echo "------------------------------"
for label in labels_detect.txt labels_classify.txt; do
    if [ -f "$label" ]; then
        echo "✓ $label - found"
    else
        echo "✗ $label - MISSING"
    fi
done

# Check dependencies
echo -e "\n6. Checking system dependencies:"
echo "------------------------------"
# ROS2
if [ -d "/opt/ros/humble" ]; then
    echo "✓ ROS2 Humble installed"
else
    echo "✗ ROS2 Humble NOT FOUND"
fi

# CUDA
if command -v nvcc &> /dev/null; then
    echo "✓ CUDA installed ($(nvcc --version | grep release | awk '{print $6}'))"
else
    echo "✗ CUDA NOT FOUND"
fi

# TensorRT
if dpkg -l | grep -q tensorrt; then
    echo "✓ TensorRT installed"
else
    echo "✗ TensorRT NOT FOUND"
fi

# OpenCV
if pkg-config --exists opencv4; then
    echo "✓ OpenCV installed ($(pkg-config --modversion opencv4))"
else
    echo "✗ OpenCV NOT FOUND"
fi

# Qt5
if pkg-config --exists Qt5Core; then
    echo "✓ Qt5 installed"
else
    echo "✗ Qt5 NOT FOUND"
fi

# jetson-utils
if [ -d "/usr/local/include/jetson-utils" ]; then
    echo "✓ jetson-utils installed"
else
    echo "✗ jetson-utils NOT FOUND"
fi

echo -e "\n7. Build recommendations:"
echo "------------------------------"
if [ ! -f "build/visionconnect/detect" ] || [ ! -f "build/visionconnect/classify" ] || [ ! -f "build/visionconnect/lanedet" ]; then
    echo "Some nodes failed to build. Try:"
    echo "1. Check the build log: cat log/latest_build/visionconnect/streams.log"
    echo "2. Clean rebuild: rm -rf build install log && colcon build --packages-select visionconnect"
    echo "3. Build with verbose output: colcon build --packages-select visionconnect --event-handlers console_direct+"
fi

echo -e "\nFor missing models, you need to:"
echo "1. Convert YOLO models to TensorRT format"
echo "2. Place .engine files in the project root"
echo "3. Create label files with class names"