#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include "ros_compat.h"
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_yaml_param_parser/parser.h>

#include "sensor_msgs/msg/image.hpp"
#include <fstream>

// CV to ROS image
std::string mat_type2encoding(int mat_type);
void convert_frame_to_message(const cv::Mat &frame, sensor_msgs::msg::Image &msg);

// ROS to CV image
void convert_message_to_frame( const sensor_msgs::ImageConstPtr msg, cv::Mat &frame);
const std::map<std::string,int> cv_map ={
    {"mono8",   CV_8UC1},   // QImage::Format_Grayscale8
    {"bgr8",    CV_8UC3},   // QImage::Format_BGR888
    {"mono16",  CV_16SC1},  // QImage::Format_Grayscale16
    {"rgba8",   CV_8UC4}    // QImage::Format_RGBA8888
};

// Parse labelmap
std::vector< std::string > getClassNames(const std::string& classes_txt);

// Create text with background on image
const cv::Scalar clr_map[] = {
    cv::Scalar(0, 0, 0),
    cv::Scalar(0, 153, 0),
    cv::Scalar(153, 0, 0),
    cv::Scalar(0, 0, 153),
    cv::Scalar(0, 153, 153),
    cv::Scalar(153, 153, 0),
    cv::Scalar(153, 0, 76),
    cv::Scalar(255, 51, 55),
    cv::Scalar(102, 204, 0)
};
//const cv::Scalar clr_map[] = {
//     cv::Scalar(0x00, 0xFF, 0x00),
//     cv::Scalar(0x3F, 0xF8, 0xFF),
//     cv::Scalar(0xF9, 0x7A, 0x1E),
//     cv::Scalar(0xF9, 0x1E, 0x9F),
//     cv::Scalar(0x1E, 0xA3, 0xFA),
//     cv::Scalar(0xFF, 0xFF, 0xFF),
//     cv::Scalar(0xE5, 0xE1, 0x44),
//     cv::Scalar(0x00, 0x00, 0xFF)
//};
void putText_bckgrnd(cv::Mat& img,
    std::string& text, 
    double font_size, 
    cv::Point c1, 
    cv::Scalar bgrnd_clr);
    
#endif
