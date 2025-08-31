#include "ros_compat.h"
#include "image_converter.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <thread>
#include <chrono>

// Stereo camera parameters
struct StereoCameraParams {
    std::string device_path = "/dev/video0";
    int width = 3840;        // Full stereo width (corrected from v4l2-ctl)
    int height = 1200;       // Height (corrected from v4l2-ctl)
    int single_width = 1920; // Width of single camera
    bool enable_left = true;
    bool enable_right = true;
    bool y16_format = true;  // Use Y16 format
};

// Global variables
cv::VideoCapture* capture = NULL;
Publisher<sensor_msgs::Image> left_image_pub = NULL;
Publisher<sensor_msgs::Image> right_image_pub = NULL;
Publisher<std_msgs::msg::Float32> framerate_pub = NULL;
Publisher<std_msgs::msg::Header> timestamp_pub = NULL;
imageConverter* left_image_cvt = NULL;
imageConverter* right_image_cvt = NULL;
StereoCameraParams params;

// Convert cv::Mat to ROS Image message
void matToImageMsg(const cv::Mat& mat, sensor_msgs::Image& msg) {
    msg.height = mat.rows;
    msg.width = mat.cols;
    msg.encoding = "mono8";  // For grayscale images
    msg.step = mat.cols;
    msg.data.resize(mat.rows * mat.cols);
    memcpy(msg.data.data(), mat.data, mat.rows * mat.cols);
}

// Acquire and publish stereo frame
bool acquireStereoFrame() {
    cv::Mat frame;
    
    // Capture frame
    if (!capture->read(frame)) {
        ROS_ERROR("Failed to capture frame from stereo camera");
        return false;
    }
    
    cv::Mat frame_8bit;
    
    // Handle Y16 format
    if (params.y16_format) {
        // Check if frame is 16-bit
        if (frame.type() == CV_16UC1) {
            // Reshape if needed (Y16 format comes as single channel)
            if (frame.rows != params.height || frame.cols != params.width) {
                int total_pixels = frame.rows * frame.cols;
                if (total_pixels == params.width * params.height * 2) {
                    // Data is packed as bytes, need to reinterpret as uint16
                    frame = cv::Mat(params.height, params.width, CV_16UC1, frame.data);
                } else if (total_pixels == params.width * params.height) {
                    frame = frame.reshape(1, params.height);
                }
            }
            
            // Convert 16-bit to 8-bit for processing
            // Use high byte (shift right by 8) for better contrast
            frame.convertTo(frame_8bit, CV_8UC1, 1.0 / 256.0);
        } else if (frame.channels() == 1) {
            // Already 8-bit grayscale
            frame_8bit = frame;
        } else {
            // Unexpected format, try to handle
            if (frame.channels() > 1) {
                cv::cvtColor(frame, frame_8bit, cv::COLOR_BGR2GRAY);
            } else {
                frame_8bit = frame;
            }
        }
    } else if (frame.channels() == 1) {
        // Already 8-bit grayscale
        frame_8bit = frame;
    } else {
        // Convert color to grayscale if needed
        cv::cvtColor(frame, frame_8bit, cv::COLOR_BGR2GRAY);
    }
    
    // Split stereo image into left and right
    cv::Mat left_img, right_img;
    
    if (frame_8bit.cols == params.width) {
        // Split horizontally
        left_img = frame_8bit(cv::Rect(0, 0, params.single_width, params.height));
        right_img = frame_8bit(cv::Rect(params.single_width, 0, params.single_width, params.height));
    } else {
        ROS_WARN("Unexpected frame width: %d (expected %d)", frame_8bit.cols, params.width);
        return false;
    }
    
    // Create ROS messages
    sensor_msgs::Image left_msg, right_msg;
    std_msgs::msg::Header time_msg;
    std_msgs::msg::Float32 fps_msg;
    
    // Get timestamp
    auto now = ROS_TIME_NOW();
    
    // Convert and publish left image
    if (params.enable_left) {
        matToImageMsg(left_img, left_msg);
        left_msg.header.stamp = now;
        left_msg.header.frame_id = "stereo_left";
        left_image_pub->publish(left_msg);
    }
    
    // Convert and publish right image
    if (params.enable_right) {
        matToImageMsg(right_img, right_msg);
        right_msg.header.stamp = now;
        right_msg.header.frame_id = "stereo_right";
        right_image_pub->publish(right_msg);
    }
    
    // Publish timestamp and framerate
    time_msg.stamp = now;
    fps_msg.data = capture->get(cv::CAP_PROP_FPS);
    
    timestamp_pub->publish(time_msg);
    framerate_pub->publish(fps_msg);
    
    return true;
}

// Initialize stereo camera with Y16 format
bool initializeStereoCamera() {
    // Open camera with V4L2 backend
    capture = new cv::VideoCapture(params.device_path, cv::CAP_V4L2);
    
    if (!capture->isOpened()) {
        ROS_ERROR("Failed to open stereo camera at %s", params.device_path.c_str());
        return false;
    }
    
    // Set Y16 format if requested
    if (params.y16_format) {
        // Y16 format FOURCC
        int fourcc = cv::VideoWriter::fourcc('Y', '1', '6', ' ');
        if (!capture->set(cv::CAP_PROP_FOURCC, fourcc)) {
            ROS_WARN("Failed to set Y16 format, trying default");
        }
    }
    
    // Set resolution
    capture->set(cv::CAP_PROP_FRAME_WIDTH, params.width);
    capture->set(cv::CAP_PROP_FRAME_HEIGHT, params.height);
    
    // Disable RGB conversion for Y16
    if (params.y16_format) {
        capture->set(cv::CAP_PROP_CONVERT_RGB, 0);
    }
    
    // Set buffer size to reduce latency
    capture->set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    // Get actual settings
    double actual_width = capture->get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = capture->get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = capture->get(cv::CAP_PROP_FPS);
    
    ROS_INFO("Stereo camera initialized:");
    ROS_INFO("  Device: %s", params.device_path.c_str());
    ROS_INFO("  Resolution: %.0fx%.0f", actual_width, actual_height);
    ROS_INFO("  FPS: %.1f", fps);
    ROS_INFO("  Format: %s", params.y16_format ? "Y16" : "Default");
    
    // Update params with actual values
    params.width = static_cast<int>(actual_width);
    params.height = static_cast<int>(actual_height);
    params.single_width = params.width / 2;
    
    return true;
}

// Alternative initialization using GStreamer pipeline
bool initializeWithGStreamer() {
    std::string pipeline;
    
    if (params.y16_format) {
        // GStreamer pipeline for Y16 format with proper caps
        // Note: Using queue and proper sync settings for stability
        pipeline = "v4l2src device=" + params.device_path + 
                   " ! video/x-raw,format=GRAY16_LE,width=" + std::to_string(params.width) + 
                   ",height=" + std::to_string(params.height) + 
                   ",framerate=30/1 ! queue ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=true sync=false max-buffers=1";
    } else {
        // Standard pipeline
        pipeline = "v4l2src device=" + params.device_path + 
                   " ! video/x-raw,width=" + std::to_string(params.width) + 
                   ",height=" + std::to_string(params.height) + 
                   " ! videoconvert ! appsink";
    }
    
    ROS_INFO("Trying GStreamer pipeline: %s", pipeline.c_str());
    
    capture = new cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    
    if (!capture->isOpened()) {
        ROS_ERROR("Failed to open camera with GStreamer pipeline");
        return false;
    }
    
    ROS_INFO("Successfully opened stereo camera with GStreamer");
    return true;
}

int main(int argc, char **argv) {
    // Create ROS node
    ROS_CREATE_NODE("stereo_camera");
    
    // Declare and get parameters
    ROS_DECLARE_PARAMETER("device", params.device_path);
    ROS_DECLARE_PARAMETER("width", params.width);
    ROS_DECLARE_PARAMETER("height", params.height);
    ROS_DECLARE_PARAMETER("enable_left", params.enable_left);
    ROS_DECLARE_PARAMETER("enable_right", params.enable_right);
    ROS_DECLARE_PARAMETER("y16_format", params.y16_format);
    ROS_DECLARE_PARAMETER("use_gstreamer", false);
    
    ROS_GET_PARAMETER("device", params.device_path);
    ROS_GET_PARAMETER("width", params.width);
    ROS_GET_PARAMETER("height", params.height);
    ROS_GET_PARAMETER("enable_left", params.enable_left);
    ROS_GET_PARAMETER("enable_right", params.enable_right);
    ROS_GET_PARAMETER("y16_format", params.y16_format);
    
    bool use_gstreamer = false;
    ROS_GET_PARAMETER("use_gstreamer", use_gstreamer);
    
    params.single_width = params.width / 2;
    
    ROS_INFO("Starting stereo camera node");
    ROS_INFO("  Device: %s", params.device_path.c_str());
    ROS_INFO("  Resolution: %dx%d (single camera: %dx%d)", 
             params.width, params.height, params.single_width, params.height);
    
    // Initialize camera
    bool initialized = false;
    
    if (use_gstreamer) {
        initialized = initializeWithGStreamer();
        if (!initialized) {
            ROS_WARN("GStreamer failed, trying V4L2 directly");
            initialized = initializeStereoCamera();
        }
    } else {
        initialized = initializeStereoCamera();
        if (!initialized && params.y16_format) {
            ROS_WARN("V4L2 with Y16 failed, trying GStreamer");
            initialized = initializeWithGStreamer();
        }
    }
    
    if (!initialized) {
        ROS_ERROR("Failed to initialize stereo camera");
        return 1;
    }
    
    // Create image converters
    left_image_cvt = new imageConverter();
    right_image_cvt = new imageConverter();
    
    // Create publishers
    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "left/image_raw", 10, left_image_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "right/image_raw", 10, right_image_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::Float32, "framerate", 5, framerate_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::Header, "timestamp", 5, timestamp_pub);
    
    ROS_INFO("Stereo camera node started, publishing to:");
    ROS_INFO("  /stereo_camera/left/image_raw");
    ROS_INFO("  /stereo_camera/right/image_raw");
    
    // Main loop
    while (ROS_OK()) {
        if (!acquireStereoFrame()) {
            ROS_WARN("Failed to acquire frame, retrying...");
            // Small delay before retry
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (ROS_OK()) {
            ROS_SPIN_ONCE();
        }
    }
    
    // Cleanup
    if (capture) {
        capture->release();
        delete capture;
    }
    delete left_image_cvt;
    delete right_image_cvt;
    
    return 0;
}