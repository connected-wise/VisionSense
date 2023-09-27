#include "ros_compat.h"
#include "image_converter.h"
#include "common.h"
#include <jetson-utils/videoSource.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
//#include <common.h>


// globals	
videoSource* stream = NULL;
Publisher<sensor_msgs::Image> image_pub = NULL;
Publisher<std_msgs::msg::Float32> framerate_pub = NULL;
Publisher<std_msgs::msg::Header> timestamp_pub = NULL;
imageConverter* image_cvt = NULL;

// aquire and publish camera frame
bool aquireFrame()
{
	//uchar3 *Frame;
	imageConverter::PixelType* Frame = NULL;

	// get the latest frame
	if( !stream->Capture(&Frame, 50000) )
	{
		ROS_ERROR("failed to capture next frame");
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;
	std_msgs::msg::Header time;
	std_msgs::msg::Float32 frate;
	//cv::Mat img(stream->GetHeight(), stream->GetWidth(), CV_8UC3, Frame);
	//convert_frame_to_message(img, msg);

	// assure correct image size
	if( !image_cvt->Resize(stream->GetWidth(), stream->GetHeight(), imageConverter::ROSOutputFormat) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	if( !image_cvt->Convert(msg, imageConverter::ROSOutputFormat, Frame) )
	{
		ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
		return false;
	}

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();
	time.stamp = ROS_TIME_NOW();
	frate.data = stream->GetFrameRate();
	// publish the message
	image_pub->publish(msg);
	framerate_pub->publish(frate);
	timestamp_pub->publish(time);
	//ROS_INFO("published %ux%u video frame", stream->GetWidth(), stream->GetHeight());
	
	return true;
}


// node main loop
int main(int argc, char **argv)
{
	// create node instance
	ROS_CREATE_NODE("camera");

	// declare parameters
	videoOptions video_options;

	std::string resource_str;
	std::string codec_str;
	std::string flip_str;
	
	int video_width = video_options.width;
	int video_height = video_options.height;
	int latency = video_options.latency;
	
	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("width", video_width);
	ROS_DECLARE_PARAMETER("height", video_height);
	ROS_DECLARE_PARAMETER("framerate", video_options.frameRate);
	ROS_DECLARE_PARAMETER("loop", video_options.loop);
	ROS_DECLARE_PARAMETER("flip", flip_str);
	ROS_DECLARE_PARAMETER("latency", latency);
	
	//retrieve parameters
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", video_width);
	ROS_GET_PARAMETER("height", video_height);
	ROS_GET_PARAMETER("framerate", video_options.frameRate);
	ROS_GET_PARAMETER("loop", video_options.loop);
	ROS_GET_PARAMETER("flip", flip_str);
	ROS_GET_PARAMETER("latency", latency);
	
	if( resource_str.size() == 0 )
	{
		ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
		return 0;
	}

	if( codec_str.size() != 0 )
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	if( flip_str.size() != 0 )
		video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());
	
	video_options.width = video_width;
	video_options.height = video_height;
	video_options.latency = latency;
	
	ROS_INFO("opening video source: %s", resource_str.c_str());

	// open video source
	stream = videoSource::Create(resource_str.c_str(), video_options);
	image_cvt = new imageConverter();

	if( !stream )
	{
		ROS_ERROR("failed to open video source");
		return 0;
	}


	// advertise publisher topics
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "raw", 10, image_pub);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Float32, "framerate", 5, framerate_pub);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Header, "time", 5, timestamp_pub);

	// start the camera streaming
	if( !stream->Open() )
	{
		ROS_ERROR("failed to start streaming video source");
		return 0;
	}

	// start publishing video frames
	while( ROS_OK() )
	{
		if( !aquireFrame() )
		{
			if( !stream->IsStreaming() )
			{
				ROS_INFO("stream is closed or reached EOS, exiting node...");
				break;
			}
		}
		if( ROS_OK() )
			ROS_SPIN_ONCE();
	}

	// free resources
	delete stream;
	delete image_cvt;

	return 0;
}
