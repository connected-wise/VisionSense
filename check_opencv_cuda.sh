#!/bin/bash

echo "=== OpenCV CUDA Support Check ==="
echo ""

# Check current OpenCV version
echo "Current OpenCV version:"
opencv_version

echo ""
echo "Checking for CUDA support in OpenCV..."
opencv_version -v | grep -i cuda || echo "No CUDA support found in current OpenCV installation"

echo ""
echo "=== Required for VisionConnect-Plus ==="
echo "This project requires OpenCV with CUDA support, specifically:"
echo "- opencv_cudaarithm"
echo "- opencv_cudawarping"
echo "- opencv_cudaimgproc"

echo ""
echo "=== Checking for CUDA libraries ==="
echo "CUDA installation found at:"
ls -la /usr/local/cuda* 2>/dev/null || echo "CUDA not found in /usr/local"

echo ""
echo "=== Solutions ==="
echo "For JetPack 6.2, you have two options:"
echo ""
echo "Option 1: Install pre-built OpenCV with CUDA (if available):"
echo "  Check NVIDIA forums for JetPack 6.2 compatible OpenCV packages"
echo ""
echo "Option 2: Build OpenCV from source with CUDA support:"
echo "  1. git clone https://github.com/opencv/opencv.git"
echo "  2. git clone https://github.com/opencv/opencv_contrib.git"
echo "  3. Build with CUDA flags enabled"
echo ""
echo "Option 3: Use JetsonHacks scripts:"
echo "  https://github.com/jetsonhacks/buildOpenCVJetson"
echo ""
echo "=== Temporary Workaround ==="
echo "The project has been modified to compile without CUDA support,"
echo "but functionality will be limited until OpenCV with CUDA is installed."