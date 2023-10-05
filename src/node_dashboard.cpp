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
#include <sensor_msgs/msg/compressed_image.hpp> 


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

// input image subscriber callback
void gui_callback(const sensor_msgs::ImageConstPtr input)
{
    
    cv::Mat img;
    sensor_msgs::msg::CompressedImage msg;
    convert_message_to_frame(input, img);
    cv::resize(img, img, cv::Size(1310, 730));
    //cv::imshow("Perception Fusion - dashboard", img);
    //cv::waitKey(1);
    std::vector<uint8_t> buffer;
    cv::imencode(".jpg", img, buffer);

    msg.header.stamp = ROS_TIME_NOW();
    msg.format = "jpeg";
    msg.data = buffer;

    gui_dashboard_pub->publish(msg);
}

// input image subscriber callback
void camera_callback(const sensor_msgs::ImageConstPtr input)
{
    cv::Mat img;
    sensor_msgs::msg::CompressedImage msg;
    
    convert_message_to_frame(input, img);
    cv::resize(img, img, cv::Size(640, 360));

    std::vector<uint8_t> buffer;
    cv::imencode(".jpg", img, buffer);

    msg.header.stamp = ROS_TIME_NOW();
    msg.format = "jpeg";
    msg.data = buffer;

    camera_dashboard_pub->publish(msg);

}

// detection callback
void detection_callback(const visionconnect::msg::Detect::SharedPtr input)
{

    // ROS_INFO("Received detection message");

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->image);
     
    cv::Mat img;
    sensor_msgs::msg::CompressedImage msg;
    
    convert_message_to_frame(img_msg, img);

    std::vector<uint8_t> buffer;
    cv::imencode(".jpg", img, buffer);

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

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->laneimg);
    auto raw_msg = std::make_shared<sensor_msgs::msg::Image>(input->rawimg);
     
    cv::Mat img1 , img2, img_result;
    sensor_msgs::msg::CompressedImage msg;
    
    convert_message_to_frame(img_msg, img1);
    convert_message_to_frame(raw_msg, img2);

    cv::resize(img1, img1, cv::Size(640, 360));
    cv::resize(img2, img2, cv::Size(640, 180));
    cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);
    cv::vconcat(img2, img1(cv::Rect(0, 180, 640, 180)), img_result);

    //cv::imshow("lanes", img_result);
    //cv::waitKey(1);

    std::vector<uint8_t> buffer;
    cv::imencode(".jpg", img_result, buffer);

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

// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("dashboard");

    auto cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, camera_callback);
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto gui_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "gui_in", 1, gui_callback);

    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "camera", 5, camera_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "lanes", 5, lanedet_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "detections", 5, detect_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "signclasses", 5, classify_dashboard_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::CompressedImage, "gui", 5, gui_dashboard_pub);

    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "camera_logs", 5, camera_logs_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "detect_logs", 5, detect_logs_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "lanedet_logs", 5, lanedet_logs_pub);

	// start publishing video frames
    ROS_INFO("Dashboard Node initialized, waiting for images");
    ROS_SPIN();

    return 0;
}
