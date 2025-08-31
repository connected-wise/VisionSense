#!/bin/bash

# Quick installation script for OpenCV with CUDA on JetPack 6.2
# This tries to find and install pre-built packages first

set -e

echo "=== Quick OpenCV CUDA Installation for JetPack 6.2 ==="
echo ""

# Function to check if OpenCV has CUDA support
check_opencv_cuda() {
    python3 -c "import cv2; print(cv2.cuda.getCudaEnabledDeviceCount() > 0)" 2>/dev/null || echo "False"
}

# Check current OpenCV status
echo "Checking current OpenCV installation..."
CURRENT_VERSION=$(opencv_version 2>/dev/null || echo "Not installed")
echo "Current OpenCV version: $CURRENT_VERSION"

if [ "$CURRENT_VERSION" != "Not installed" ]; then
    HAS_CUDA=$(check_opencv_cuda)
    echo "CUDA support: $HAS_CUDA"
    
    if [ "$HAS_CUDA" = "True" ]; then
        echo "OpenCV already has CUDA support!"
        exit 0
    fi
fi

# Method 1: Check for NVIDIA's OpenCV packages
echo ""
echo "Method 1: Checking for NVIDIA pre-built packages..."

# Remove existing OpenCV if it doesn't have CUDA
if [ "$CURRENT_VERSION" != "Not installed" ] && [ "$HAS_CUDA" != "True" ]; then
    echo "Removing existing OpenCV without CUDA support..."
    read -p "Remove existing OpenCV? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt remove -y '*opencv*' || true
        sudo apt autoremove -y
    fi
fi

# Check for NVIDIA's OpenCV in apt
echo "Searching for NVIDIA OpenCV packages..."
apt-cache search opencv | grep -i nvidia | grep -i cuda || echo "No NVIDIA OpenCV packages found in apt"

# Method 2: Download pre-built OpenCV from NVIDIA (if available)
echo ""
echo "Method 2: Checking NVIDIA NGC for pre-built OpenCV..."
echo "Visit: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-ml"
echo "This container includes OpenCV with CUDA pre-built for Jetson"

# Method 3: Use JetsonHacks automated script
echo ""
echo "Method 3: JetsonHacks automated build script"
echo "This is faster than manual build and handles Jetson-specific optimizations"

read -p "Do you want to use JetsonHacks build script? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd $HOME
    if [ ! -d "buildOpenCVJetson" ]; then
        git clone https://github.com/jetsonhacks/buildOpenCVJetson.git
    fi
    cd buildOpenCVJetson
    
    # Modify the script for OpenCV 4.8.0 and JetPack 6.2
    sed -i 's/OPENCV_VERSION=.*/OPENCV_VERSION=4.8.0/' buildOpenCV.sh
    
    echo "Starting automated build..."
    echo "Note: This will take 1-2 hours but handles all Jetson optimizations"
    ./buildOpenCV.sh
    
    cd $HOME/VisionConnect-Plus
fi

# Method 4: Install from JetPack SDK Manager
echo ""
echo "Method 4: JetPack SDK Manager"
echo "If you have access to a host machine with NVIDIA SDK Manager:"
echo "1. Connect your Jetson to the host"
echo "2. Run SDK Manager and select JetPack 6.2"
echo "3. Under 'Jetson SDK Components', ensure 'OpenCV' is selected"
echo "4. This will install OpenCV with CUDA support"

# Verify installation
echo ""
echo "Verifying OpenCV CUDA support..."
python3 << EOF
try:
    import cv2
    print(f"OpenCV Version: {cv2.__version__}")
    cuda_count = cv2.cuda.getCudaEnabledDeviceCount()
    print(f"CUDA Devices: {cuda_count}")
    if cuda_count > 0:
        print("SUCCESS: OpenCV has CUDA support!")
        # List CUDA modules
        cuda_modules = [attr for attr in dir(cv2.cuda) if not attr.startswith('_')]
        print(f"Available CUDA modules: {len(cuda_modules)}")
    else:
        print("WARNING: OpenCV installed but no CUDA support detected")
except Exception as e:
    print(f"Error: {e}")
EOF

echo ""
echo "=== Installation Options Summary ==="
echo "1. Build from source: Run ./install_opencv_cuda.sh (most reliable, takes 1-2 hours)"
echo "2. JetsonHacks script: Automated build optimized for Jetson"
echo "3. NGC Container: Pre-built ML container with OpenCV CUDA"
echo "4. SDK Manager: Official NVIDIA method (requires host PC)"
echo ""
echo "For VisionConnect-Plus, you need these OpenCV CUDA modules:"
echo "- opencv_cudaarithm"
echo "- opencv_cudawarping"
echo "- opencv_cudaimgproc"