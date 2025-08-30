#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "trtutil.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "image_converter.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

videoOutput* stream1 = NULL;
videoOutput* stream2 = NULL;
videoOutput* stream3 = NULL;
imageConverter* image_cvt = NULL;
int display_w, display_h;


// input image subscriber callback
void signs_callback(const visionconnect::msg::Signs::SharedPtr input)
{
    
    ROS_INFO("%lu signs and lights detected", input->classes.size());

}

// input image subscriber callback
void camera_callback(const sensor_msgs::ImageConstPtr input)
{

    // convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

    // render the image
	stream1->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
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

    stream2->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
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

    stream3->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
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

    stream1 = videoOutput::Create(output.c_str());
    stream2 = videoOutput::Create(output.c_str());
    stream3 = videoOutput::Create(output.c_str());  

    if (!stream1 || !stream2 || !stream3)
	{
		ROS_ERROR("failed to open video output");
		return 0;
	}

    image_cvt = new imageConverter();

    auto cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, camera_callback);
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto signs_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs_in", 1, signs_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);

	// start publishing video frames
    ROS_INFO("Preview Node initialized, waiting for images");
    ROS_SPIN();
    
    //free resources
	delete stream1;
    delete stream2;
    delete stream3;
	delete image_cvt;

    return 0;
}
