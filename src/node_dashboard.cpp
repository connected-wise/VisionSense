#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "trtutil.h"
#include <sstream>

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_converter.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <sensor_msgs/msg/compressed_image.hpp>
#include <chrono>
#include <iomanip> 


// Compressed Image Publisher
Publisher<sensor_msgs::msg::CompressedImage> camera_dashboard_pub = NULL;
Publisher<sensor_msgs::msg::CompressedImage> lanedet_dashboard_pub = NULL;
Publisher<sensor_msgs::msg::CompressedImage> detect_dashboard_pub = NULL;
Publisher<sensor_msgs::msg::CompressedImage> classify_dashboard_pub = NULL;
Publisher<sensor_msgs::msg::CompressedImage> gui_dashboard_pub = NULL;

// Publish some logs
Publisher<std_msgs::msg::String> camera_logs_pub = NULL;
Publisher<std_msgs::msg::String> detect_logs_pub = NULL;
Publisher<std_msgs::msg::String> lanedet_logs_pub = NULL;
Publisher<std_msgs::msg::String> classify_logs_pub = NULL;

// Frame rate limiting (20 FPS max)
static constexpr double MAX_FPS = 20.0;
static constexpr int JPEG_QUALITY = 60;

// Timing for rate limiting
static auto last_camera_time = std::chrono::steady_clock::now();
static auto last_detect_time = std::chrono::steady_clock::now();
static auto last_lanes_time = std::chrono::steady_clock::now();
static auto last_gui_time = std::chrono::steady_clock::now();
static auto last_classify_time = std::chrono::steady_clock::now();

// Helper function to load reference sign image
cv::Mat load_sign_image(const std::string& class_name) {
    std::string signs_path = "/home/cw-orin/VisionConnect-Plus/src/signs/";
    std::string filename = signs_path + class_name + ".png";
    
    cv::Mat sign_img = cv::imread(filename, cv::IMREAD_COLOR);
    
    // If still empty, create placeholder
    if (sign_img.empty()) {
        sign_img = cv::Mat::zeros(200, 200, CV_8UC3);
        cv::putText(sign_img, class_name, cv::Point(10, 90), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(sign_img, "NOT FOUND", cv::Point(10, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }
    
    return sign_img;
}

// input image subscriber callback
void gui_callback(const sensor_msgs::ImageConstPtr input)
{
    // Rate limiting - 20 FPS max
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_gui_time).count();
    if (elapsed < (1000.0 / MAX_FPS)) {
        return;  // Skip this frame
    }
    last_gui_time = now;
    
    cv::Mat img;
    sensor_msgs::msg::CompressedImage msg;
    convert_message_to_frame(input, img);
    cv::resize(img, img, cv::Size(1310, 730));
    
    std::vector<uint8_t> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
    cv::imencode(".jpg", img, buffer, compression_params);

    msg.header.stamp = ROS_TIME_NOW();
    msg.format = "jpeg";
    msg.data = buffer;

    gui_dashboard_pub->publish(msg);
}

// input image subscriber callback
void camera_callback(const sensor_msgs::ImageConstPtr input)
{
    // Rate limiting - 20 FPS max
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_camera_time).count();
    if (elapsed < (1000.0 / MAX_FPS)) {
        return;  // Skip this frame
    }
    last_camera_time = now;
    
    cv::Mat img;
    sensor_msgs::msg::CompressedImage msg;
    
    convert_message_to_frame(input, img);
    cv::resize(img, img, cv::Size(640, 360));

    std::vector<uint8_t> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
    cv::imencode(".jpg", img, buffer, compression_params);

    msg.header.stamp = ROS_TIME_NOW();
    msg.format = "jpeg";
    msg.data = buffer;

    camera_dashboard_pub->publish(msg);
}

// detection callback
void detection_callback(const visionconnect::msg::Detect::SharedPtr input)
{
    // Rate limiting - 20 FPS max
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_detect_time).count();
    if (elapsed < (1000.0 / MAX_FPS)) {
        return;  // Skip this frame
    }
    last_detect_time = now;

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->image);
     
    cv::Mat img;
    sensor_msgs::msg::CompressedImage msg;
    
    convert_message_to_frame(img_msg, img);

    std::vector<uint8_t> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
    cv::imencode(".jpg", img, buffer, compression_params);

    msg.header.stamp = ROS_TIME_NOW();
    msg.format = "jpeg";
    msg.data = buffer;

    detect_dashboard_pub->publish(msg);

    // Construct the detection log
    std::ostringstream logStream;
    logStream << "Number of Objects Detected: " << input->num_detections -1 << "<br>";

    for(int i = 0; i < input->num_detections -1; i++)
    {
        logStream << "Object " << (i + 1) << ": Class " << input->classes[i] << ", Score: " << input->scores[i] << "<br>";
    }

    // Publish the detection log
    std_msgs::msg::String logMsg;
    logMsg.data = logStream.str();
    detect_logs_pub->publish(logMsg);

}

// detection overlay callback
void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input)
{
    // Rate limiting - 20 FPS max
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_lanes_time).count();
    if (elapsed < (1000.0 / MAX_FPS)) {
        return;  // Skip this frame
    }
    last_lanes_time = now;

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->laneimg);
    auto raw_msg = std::make_shared<sensor_msgs::msg::Image>(input->rawimg);
     
    cv::Mat img1 , img2, img_result;
    sensor_msgs::msg::CompressedImage msg;
    
    convert_message_to_frame(img_msg, img1);
    convert_message_to_frame(raw_msg, img2);

    cv::resize(img1, img1, cv::Size(640, 360));
    cv::resize(img2, img2, cv::Size(640, 180));
    // Convert grayscale to BGR - workaround for version mismatch
    cv::Mat img2_bgr;
    cv::merge(std::vector<cv::Mat>{img2, img2, img2}, img2_bgr);
    img2 = img2_bgr;
    cv::vconcat(img2, img1(cv::Rect(0, 180, 640, 180)), img_result);

    std::vector<uint8_t> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
    cv::imencode(".jpg", img_result, buffer, compression_params);

    msg.header.stamp = ROS_TIME_NOW();
    msg.format = "jpeg";
    msg.data = buffer;

    lanedet_dashboard_pub->publish(msg);

    // Construct the detection log
    std::ostringstream logStream;
    logStream << "Number of Lanes Detected: " << input->num_lanes << "<br>";

    for(int i = 0; i < input->num_lanes; i++)
    {
        logStream << "Lane " << (i + 1) << ", Score: " << input->probs[i] << "<br>";
    }

    // Publish the detection log
    std_msgs::msg::String logMsg;
    logMsg.data = logStream.str();
    lanedet_logs_pub->publish(logMsg);
}

// traffic signs classification callback
void classify_callback(const visionconnect::msg::Signs::SharedPtr input)
{
    // Rate limiting - 20 FPS max
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_classify_time).count();
    if (elapsed < (1000.0 / MAX_FPS)) {
        return;  // Skip this frame
    }
    last_classify_time = now;

    if (input->images.empty()) {
        return;
    }

    cv::Mat final_image;
    sensor_msgs::msg::CompressedImage msg;
    
    // Create side-by-side composite for all detected signs
    std::vector<cv::Mat> sign_pairs;
    
    for (size_t i = 0; i < input->images.size() && i < input->labels.size(); i++) {
        // Convert detected sign image
        auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->images[i]);
        cv::Mat detected_img;
        convert_message_to_frame(img_msg, detected_img);
        
        // Resize detected image to standard size
        cv::resize(detected_img, detected_img, cv::Size(200, 200));
        
        // Load reference sign image
        cv::Mat reference_img = load_sign_image(input->labels[i]);
        cv::resize(reference_img, reference_img, cv::Size(200, 200));
        
        // Create side-by-side pair
        cv::Mat pair;
        cv::hconcat(detected_img, reference_img, pair);
        
        // Add labels and score
        cv::putText(pair, "Detected", cv::Point(10, 25), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(pair, "Reference", cv::Point(210, 25), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        
        // Add class name and score at bottom
        std::string info = input->labels[i];
        if (i < input->scores.size()) {
            info += " (" + std::to_string((int)(input->scores[i] * 100)) + "%)";
        }
        cv::putText(pair, info, cv::Point(10, pair.rows - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        sign_pairs.push_back(pair);
    }
    
    if (sign_pairs.empty()) {
        return;
    }
    
    // Stack all pairs vertically
    if (sign_pairs.size() == 1) {
        final_image = sign_pairs[0];
    } else {
        cv::vconcat(sign_pairs, final_image);
    }
    
    // Resize to reasonable dashboard size if too large
    if (final_image.rows > 600) {
        double scale = 600.0 / final_image.rows;
        cv::resize(final_image, final_image, cv::Size(0, 0), scale, scale);
    }

    std::vector<uint8_t> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
    cv::imencode(".jpg", final_image, buffer, compression_params);

    msg.header.stamp = ROS_TIME_NOW();
    msg.format = "jpeg";
    msg.data = buffer;

    classify_dashboard_pub->publish(msg);

    // Construct the classification log
    std::ostringstream logStream;
    logStream << "Number of Signs Classified: " << input->labels.size() << "<br>";

    for(size_t i = 0; i < input->labels.size() && i < input->scores.size(); i++)
    {
        logStream << "Sign " << (i + 1) << ": " << input->labels[i] 
                 << ", Score: " << std::fixed << std::setprecision(2) << input->scores[i] << "<br>";
    }

    // Publish the classification log
    std_msgs::msg::String logMsg;
    logMsg.data = logStream.str();
    classify_logs_pub->publish(logMsg);
}

// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("dashboard");

    auto cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, camera_callback);
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto gui_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "gui_in", 1, gui_callback);
    auto classify_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs", 1, classify_callback);

    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "camera", 5, camera_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "lanes", 5, lanedet_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "detections", 5, detect_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "signclasses", 5, classify_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "gui", 5, gui_dashboard_pub);

    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "camera_logs", 5, camera_logs_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "detect_logs", 5, detect_logs_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "lanedet_logs", 5, lanedet_logs_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "classify_logs", 5, classify_logs_pub);

	// start publishing video frames
    ROS_INFO("Dashboard Node initialized (FPS limited to %.1f, JPEG quality %d%%)", MAX_FPS, JPEG_QUALITY);
    ROS_SPIN();

    return 0;
}
