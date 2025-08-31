#!/bin/bash
# Script to install jetson-inference and jetson-utils from source

echo "This script will install jetson-inference and jetson-utils from source."
echo "This process requires internet connection and may take 30-60 minutes."
echo ""
echo "Prerequisites: CUDA, cuDNN, and TensorRT should already be installed."
echo ""
read -p "Do you want to continue? (y/N) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled."
    exit 1
fi

# Install dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y git cmake libpython3-dev python3-numpy

# Clone and build jetson-inference (includes jetson-utils)
cd ~
if [ ! -d "jetson-inference" ]; then
    echo "Cloning jetson-inference repository..."
    git clone --recursive https://github.com/dusty-nv/jetson-inference
else
    echo "jetson-inference directory already exists, updating..."
    cd jetson-inference
    git pull
    git submodule update --init
    cd ..
fi

cd jetson-inference
mkdir -p build
cd build

echo "Configuring build..."
cmake ../

echo "Building jetson-inference and jetson-utils..."
make -j$(nproc)

echo "Installing..."
sudo make install
sudo ldconfig

echo ""
echo "Installation complete!"
echo "You can now build the VisionConnect-Plus project."
echo ""
echo "To build VisionConnect-Plus:"
echo "  cd /home/cw-orin/VisionConnect-Plus"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select visionconnect"
echo "  source install/setup.bash"
echo "  ros2 launch visionconnect test.launch.py"