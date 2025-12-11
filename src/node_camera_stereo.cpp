/*
 * Simplified Stereo Camera Node
 * Publishes left and right rectified images only (no depth processing)
 * Based on node_camera.cpp structure
 */

#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

// Globals
videoSource* stream = NULL;
Publisher<sensor_msgs::Image> left_image_pub = NULL;
Publisher<sensor_msgs::Image> right_image_pub = NULL;
Publisher<std_msgs::msg::Float32> framerate_pub = NULL;
Publisher<std_msgs::msg::Header> timestamp_pub = NULL;
imageConverter* left_image_cvt = NULL;
imageConverter* right_image_cvt = NULL;

// For stereo camera splitting
uchar3* left_image_gpu = NULL;
uchar3* right_image_gpu = NULL;
int stereo_width = 3840;
int stereo_height = 1200;
int single_width = 1920;
int single_height = 1200;

// Acquire and publish stereo frame
bool acquireStereoFrame()
{
	imageConverter::PixelType* nextFrame = NULL;

	// Get the latest frame
	if (!stream->Capture(&nextFrame, 10000))
	{
		ROS_ERROR("Failed to capture next frame");
		return false;
	}

	// Allocate GPU memory for left and right images on first frame
	static bool first_frame = true;
	if (first_frame)
	{
		if (!cudaAllocMapped((void**)&left_image_gpu, single_width * single_height * sizeof(uchar3)))
		{
			ROS_ERROR("Failed to allocate CUDA memory for left image");
			return false;
		}
		if (!cudaAllocMapped((void**)&right_image_gpu, single_width * single_height * sizeof(uchar3)))
		{
			ROS_ERROR("Failed to allocate CUDA memory for right image");
			return false;
		}
		ROS_INFO("Stereo camera initialized: %dx%d (splitting to %dx%d per eye)",
		         stereo_width, stereo_height, single_width, single_height);
		first_frame = false;
	}

	// Split stereo frame into left and right on GPU
	// Left image: second half (single_width to stereo_width) - swapped
	// Right image: first half (0 to single_width) - swapped
	for (int y = 0; y < single_height; y++)
	{
		cudaMemcpy(
			left_image_gpu + y * single_width,
			nextFrame + y * stereo_width + single_width,
			single_width * sizeof(uchar3),
			cudaMemcpyDeviceToDevice
		);

		cudaMemcpy(
			right_image_gpu + y * single_width,
			nextFrame + y * stereo_width,
			single_width * sizeof(uchar3),
			cudaMemcpyDeviceToDevice
		);
	}

	// Prepare messages
	sensor_msgs::Image left_msg, right_msg;
	std_msgs::msg::Header time_msg;
	std_msgs::msg::Float32 frate;

	auto now = ROS_TIME_NOW();

	// Convert and publish left image
	if (!left_image_cvt->Resize(single_width, single_height, imageConverter::ROSOutputFormat))
	{
		ROS_ERROR("Failed to resize left image converter");
		return false;
	}

	if (!left_image_cvt->Convert(left_msg, imageConverter::ROSOutputFormat, left_image_gpu))
	{
		ROS_ERROR("Failed to convert left image");
		return false;
	}

	left_msg.header.stamp = now;
	left_msg.header.frame_id = "stereo_left";
	left_image_pub->publish(left_msg);

	// Convert and publish right image
	if (!right_image_cvt->Resize(single_width, single_height, imageConverter::ROSOutputFormat))
	{
		ROS_ERROR("Failed to resize right image converter");
		return false;
	}

	if (!right_image_cvt->Convert(right_msg, imageConverter::ROSOutputFormat, right_image_gpu))
	{
		ROS_ERROR("Failed to convert right image");
		return false;
	}

	right_msg.header.stamp = now;
	right_msg.header.frame_id = "stereo_right";
	right_image_pub->publish(right_msg);

	// Publish timestamp and framerate
	time_msg.stamp = now;
	frate.data = stream->GetFrameRate();
	timestamp_pub->publish(time_msg);
	framerate_pub->publish(frate);

	return true;
}

// Main
int main(int argc, char **argv)
{
	// Create node instance
	ROS_CREATE_NODE("camera_stereo");

	// Declare parameters
	videoOptions video_options;
	std::string resource_str;
	std::string codec_str;
	std::string flip_str;
	int framerate_int = 30;

	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("width", stereo_width);
	ROS_DECLARE_PARAMETER("height", stereo_height);
	ROS_DECLARE_PARAMETER("framerate", framerate_int);
	ROS_DECLARE_PARAMETER("flip", flip_str);
	ROS_DECLARE_PARAMETER("latency", video_options.latency);

	// Retrieve parameters
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", stereo_width);
	ROS_GET_PARAMETER("height", stereo_height);
	ROS_GET_PARAMETER("framerate", framerate_int);
	ROS_GET_PARAMETER("flip", flip_str);
	ROS_GET_PARAMETER("latency", video_options.latency);

	video_options.frameRate = static_cast<float>(framerate_int);

	if (resource_str.size() == 0)
	{
		ROS_ERROR("Resource parameter not set - please specify stereo camera resource");
		return 0;
	}

	if (codec_str.size() != 0)
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	if (flip_str.size() != 0)
		video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());

	video_options.width = stereo_width;
	video_options.height = stereo_height;
	single_width = stereo_width / 2;
	single_height = stereo_height;

	ROS_INFO("Opening stereo camera: %s (%dx%d)", resource_str.c_str(), stereo_width, stereo_height);
	ROS_INFO("Will publish left and right images at %dx%d each", single_width, single_height);

	// Open video source
	stream = videoSource::Create(resource_str.c_str(), video_options);
	left_image_cvt = new imageConverter();
	right_image_cvt = new imageConverter();

	if (!stream)
	{
		ROS_ERROR("Failed to open stereo camera");
		return 0;
	}

	// Create publishers
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "left/image_raw", 10, left_image_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "right/image_raw", 10, right_image_pub);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Float32, "framerate", 5, framerate_pub);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Header, "time", 5, timestamp_pub);

	// Start streaming
	if (!stream->Open())
	{
		ROS_ERROR("Failed to start streaming stereo camera");
		return 0;
	}

	ROS_INFO("Stereo camera node started - publishing to:");
	ROS_INFO("  /camera_stereo/left/image_raw");
	ROS_INFO("  /camera_stereo/right/image_raw");
	ROS_INFO("  /camera_stereo/framerate");

	// Main loop
	while (ROS_OK())
	{
		if (!acquireStereoFrame())
		{
			if (!stream->IsStreaming())
			{
				ROS_INFO("Stream closed or reached EOS, exiting...");
				break;
			}
		}

		if (ROS_OK())
			ROS_SPIN_ONCE();
	}

	// Cleanup
	delete stream;
	delete left_image_cvt;
	delete right_image_cvt;
	if (left_image_gpu) cudaFree(left_image_gpu);
	if (right_image_gpu) cudaFree(right_image_gpu);

	ROS_INFO("Stereo camera node shutdown complete");
	return 0;
}
