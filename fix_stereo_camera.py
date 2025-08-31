#!/usr/bin/env python3
"""
Fixed stereo camera test script for Arducam with proper Y16 handling
Addresses common GStreamer issues
"""

import cv2
import numpy as np
import sys
import os

def fix_gstreamer_issues():
    """Fix common GStreamer configuration issues"""
    print("\n=== Checking GStreamer Configuration ===")
    
    # Check if running as correct user/group
    import subprocess
    result = subprocess.run("groups", shell=True, capture_output=True, text=True)
    if "video" not in result.stdout:
        print("WARNING: User not in 'video' group. Run: sudo usermod -a -G video $USER")
    
    # Check GStreamer plugin path
    gst_plugin_path = os.environ.get('GST_PLUGIN_PATH', '')
    print(f"GST_PLUGIN_PATH: {gst_plugin_path if gst_plugin_path else 'Not set'}")
    
    # Check for v4l2 plugin
    result = subprocess.run("gst-inspect-1.0 v4l2src", shell=True, capture_output=True)
    if result.returncode != 0:
        print("ERROR: v4l2src plugin not found. Install: sudo apt-get install gstreamer1.0-plugins-good")
        return False
    
    return True

def test_fixed_v4l2():
    """Test V4L2 capture with proper Y16 handling"""
    print("\n=== Testing Fixed V4L2 Capture ===")
    
    # Open camera with V4L2 backend
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("Failed to open camera - check permissions")
        return False
    
    # Set Y16 format - use correct FOURCC
    fourcc = cv2.VideoWriter_fourcc('Y', '1', '6', ' ')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    
    # Set proper resolution from v4l2-ctl output
    # Using 3840x1200 as shown in formats list
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    
    # CRITICAL: Disable RGB conversion for Y16
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    
    # Set buffer size to 1 to reduce latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    print(f"Format set: {cap.get(cv2.CAP_PROP_FOURCC)}")
    print(f"Resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
    print(f"FPS: {cap.get(cv2.CAP_PROP_FPS)}")
    
    # Test capture
    ret, frame = cap.read()
    if ret:
        print(f"Frame captured successfully!")
        print(f"  Shape: {frame.shape}")
        print(f"  Dtype: {frame.dtype}")
        
        # Handle Y16 format properly
        if frame.dtype == np.uint16 or len(frame.shape) == 2:
            # For Y16, data might need reshaping
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            
            # Reshape if needed
            if frame.size == h * w * 2:  # Y16 is 2 bytes per pixel
                frame = frame.view(np.uint16).reshape(h, w)
            elif frame.size == h * w:
                frame = frame.reshape(h, w)
            
            print(f"  Reshaped to: {frame.shape}")
            print(f"  Min/Max values: {frame.min()}/{frame.max()}")
            
            # Convert to 8-bit for display
            if frame.dtype == np.uint16:
                frame_8bit = (frame >> 8).astype(np.uint8)  # Use high byte
            else:
                frame_8bit = frame
            
            # Split stereo
            left_img = frame_8bit[:, :w//2]
            right_img = frame_8bit[:, w//2:]
            
            print(f"  Left image: {left_img.shape}")
            print(f"  Right image: {right_img.shape}")
            
            # Save test images
            cv2.imwrite("stereo_left.png", left_img)
            cv2.imwrite("stereo_right.png", right_img)
            print("  Saved stereo_left.png and stereo_right.png")
            
            cap.release()
            return True
    
    print("Failed to capture frame")
    cap.release()
    return False

def test_fixed_gstreamer():
    """Test GStreamer with proper pipeline for Y16"""
    print("\n=== Testing Fixed GStreamer Pipeline ===")
    
    pipelines = [
        # Pipeline 1: Direct Y16 with proper caps
        "v4l2src device=/dev/video0 ! video/x-raw,format=GRAY16_LE,width=3840,height=1200 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink",
        
        # Pipeline 2: With queue for buffering
        "v4l2src device=/dev/video0 ! video/x-raw,format=GRAY16_LE,width=3840,height=1200 ! queue ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=true sync=false",
        
        # Pipeline 3: Alternative with different conversion
        "v4l2src device=/dev/video0 ! video/x-raw,format=GRAY16_LE,width=3840,height=1200 ! videoconvert ! appsink",
        
        # Pipeline 4: Using videoscale for compatibility
        "v4l2src device=/dev/video0 ! video/x-raw,format=GRAY16_LE,width=3840,height=1200 ! videoscale ! video/x-raw,width=1920,height=600 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink"
    ]
    
    for idx, pipeline in enumerate(pipelines):
        print(f"\nTrying pipeline {idx + 1}:")
        print(f"  {pipeline}")
        
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print(f"  Failed to open pipeline {idx + 1}")
                continue
            
            ret, frame = cap.read()
            if ret:
                print(f"  SUCCESS! Frame: {frame.shape}, dtype: {frame.dtype}")
                
                # Save result
                if len(frame.shape) == 2 or frame.shape[2] == 1:
                    cv2.imwrite(f"gst_test_{idx+1}.png", frame)
                else:
                    cv2.imwrite(f"gst_test_{idx+1}.png", cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                
                cap.release()
                return True
            else:
                print(f"  Failed to read frame")
            
            cap.release()
        except Exception as e:
            print(f"  Error: {e}")
    
    return False

def create_working_node():
    """Create a simplified working stereo camera node"""
    print("\n=== Creating Fixed Stereo Camera Node ===")
    
    code = '''#include "ros_compat.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class StereoCameraNode {
private:
    cv::VideoCapture cap;
    Publisher<sensor_msgs::msg::Image> left_pub;
    Publisher<sensor_msgs::msg::Image> right_pub;
    
public:
    bool initialize() {
        // Use V4L2 directly - most reliable
        cap.open(0, cv::CAP_V4L2);
        if (!cap.isOpened()) return false;
        
        // Set Y16 format
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','1','6',' '));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
        cap.set(cv::CAP_PROP_CONVERT_RGB, 0);  // Critical for Y16
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        
        return true;
    }
    
    void captureAndPublish() {
        cv::Mat frame;
        if (!cap.read(frame)) return;
        
        // Handle Y16 format
        cv::Mat frame_8bit;
        if (frame.type() == CV_16UC1) {
            // Convert 16-bit to 8-bit (use high byte)
            frame.convertTo(frame_8bit, CV_8UC1, 1.0/256.0);
        } else {
            frame_8bit = frame;
        }
        
        // Ensure proper dimensions
        if (frame_8bit.cols != 3840) return;
        
        // Split stereo
        cv::Mat left = frame_8bit(cv::Rect(0, 0, 1920, 1200));
        cv::Mat right = frame_8bit(cv::Rect(1920, 0, 1920, 1200));
        
        // Convert to ROS messages using cv_bridge
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", left).toImageMsg();
        auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", right).toImageMsg();
        
        left_pub->publish(*left_msg);
        right_pub->publish(*right_msg);
    }
};
'''
    
    print("Simplified node code ready. Key fixes:")
    print("1. Use V4L2 backend directly (most reliable)")
    print("2. Set CONVERT_RGB to 0 for Y16 format")
    print("3. Use 3840x1200 resolution (from v4l2-ctl)")
    print("4. Convert 16-bit to 8-bit using high byte")
    print("5. Use cv_bridge for proper ROS message conversion")
    
    return True

def main():
    print("Stereo Camera Troubleshooting Script")
    print("=====================================")
    
    # Check system configuration
    if not fix_gstreamer_issues():
        print("\nPlease fix GStreamer issues first")
    
    # Test different methods
    v4l2_works = test_fixed_v4l2()
    gst_works = test_fixed_gstreamer()
    
    print("\n=== DIAGNOSIS ===")
    if v4l2_works:
        print("✓ V4L2 capture works - use this method")
        print("  Resolution: 3840x1200 (not 3200x1300)")
        print("  Key: Set CAP_PROP_CONVERT_RGB to 0")
    else:
        print("✗ V4L2 capture failed")
    
    if gst_works:
        print("✓ GStreamer pipeline works")
    else:
        print("✗ GStreamer pipeline failed")
        print("  Likely issue: GRAY16_LE format support")
    
    print("\n=== RECOMMENDED FIX ===")
    if v4l2_works:
        print("1. Update node_stereo_camera.cpp to use 3840x1200 resolution")
        print("2. Ensure CAP_PROP_CONVERT_RGB is set to 0")
        print("3. Use V4L2 backend, not GStreamer")
        create_working_node()
    else:
        print("1. Check camera permissions: sudo usermod -a -G video $USER")
        print("2. Verify camera with: v4l2-ctl -d /dev/video0 --all")
        print("3. Test with lower resolution first")

if __name__ == "__main__":
    main()