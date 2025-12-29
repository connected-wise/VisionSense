#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "trtutil.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "image_converter.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>

videoOutput* stream_left = NULL;
videoOutput* stream_right = NULL;
videoOutput* stream_detect = NULL;
videoOutput* stream_lanes = NULL;
videoOutput* stream_disparity = NULL;
imageConverter* image_cvt_left = NULL;
imageConverter* image_cvt_right = NULL;
imageConverter* image_cvt = NULL;
int display_w, display_h;


// input image subscriber callback
void signs_callback(const visionconnect::msg::Signs::SharedPtr input)
{
    
    ROS_INFO("%lu signs and lights detected", input->classes.size());

}

// left camera subscriber callback
void left_camera_callback(const sensor_msgs::ImageConstPtr input)
{
	if( !image_cvt_left || !image_cvt_left->Convert(input) )
	{
		ROS_INFO("failed to convert left %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;
	}
	stream_left->Render(image_cvt_left->ImageGPU(), image_cvt_left->GetWidth(), image_cvt_left->GetHeight());
}

// right camera subscriber callback
void right_camera_callback(const sensor_msgs::ImageConstPtr input)
{
	if( !image_cvt_right || !image_cvt_right->Convert(input) )
	{
		ROS_INFO("failed to convert right %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;
	}
	stream_right->Render(image_cvt_right->ImageGPU(), image_cvt_right->GetWidth(), image_cvt_right->GetHeight());
}

// detection overlay callback
void detection_callback(const visionconnect::msg::Detect::SharedPtr input)
{

    ROS_INFO("Received detection message");

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->image);
     
        // convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(img_msg))
	{
		ROS_INFO("failed to convert %ux%u %s image", img_msg->width, img_msg->height, img_msg->encoding.c_str());
		return;
	}

	stream_detect->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
}

// detection overlay callback
void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input)
{

    ROS_INFO("Received detection message");

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->laneimg);

    // convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(img_msg))
	{
		ROS_INFO("failed to convert %ux%u %s image", img_msg->width, img_msg->height, img_msg->encoding.c_str());
		return;
	}

	stream_lanes->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
}

// disparity viewer callback
void disparity_callback(const sensor_msgs::ImageConstPtr input)
{
    ROS_INFO("Received disparity image: %ux%u %s", input->width, input->height, input->encoding.c_str());

    // convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;
	}

	stream_disparity->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
}

// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("preview");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
    std::string output;

    ROS_DECLARE_PARAMETER("output", output);
    ROS_DECLARE_PARAMETER("width", display_w);
    ROS_DECLARE_PARAMETER("height", display_h);

    ROS_GET_PARAMETER("output", output);
    ROS_GET_PARAMETER("width", display_w);
    ROS_GET_PARAMETER("height", display_h);

    // videoOptions video_options;
    // video_options.width = display_w;
	// video_options.height = display_h;

    stream_left = videoOutput::Create(output.c_str());
    stream_right = videoOutput::Create(output.c_str());
    stream_detect = videoOutput::Create(output.c_str());
    stream_lanes = videoOutput::Create(output.c_str());
    stream_disparity = videoOutput::Create(output.c_str());

    if (!stream_left || !stream_right || !stream_detect || !stream_lanes || !stream_disparity)
	{
		ROS_ERROR("failed to open video output");
		return 0;
	}

    image_cvt_left = new imageConverter();
    image_cvt_right = new imageConverter();
    image_cvt = new imageConverter();

    auto left_cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "left/image_in", 1, left_camera_callback);
    auto right_cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "right/image_in", 1, right_camera_callback);
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto signs_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs_in", 1, signs_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto disparity_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "disparity_in", 1, disparity_callback);

	// start publishing video frames
    ROS_INFO("Preview Node initialized, waiting for images");
    ROS_SPIN();
    
    //free resources
	delete stream_left;
	delete stream_right;
	delete stream_detect;
	delete stream_lanes;
	delete stream_disparity;
	delete image_cvt_left;
	delete image_cvt_right;
	delete image_cvt;

    return 0;
}
