/*
 * Stereo Camera Node
 * Crops center, splits, and optionally rotates stereo images
 * Input: 3840x1200 -> Crop center 1440px -> Two 1200x1200 outputs
 */

#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <cuda_runtime.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>

extern "C" cudaError_t cudaStereoCropSplitRotate(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, int outputSize, cudaStream_t stream
);

extern "C" cudaError_t cudaStereoCropSplit(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, int outputSize, cudaStream_t stream
);

videoSource* stream = NULL;
Publisher<sensor_msgs::Image> left_image_pub = NULL;
Publisher<sensor_msgs::Image> right_image_pub = NULL;
Publisher<std_msgs::msg::Float32> framerate_pub = NULL;
Publisher<std_msgs::msg::Header> timestamp_pub = NULL;

uchar3* left_gpu = NULL;
uchar3* right_gpu = NULL;

int stereo_width = 3840;
int stereo_height = 1200;
int output_size = 1200;
bool rotated_lenses = true;

bool acquireStereoFrame()
{
	imageConverter::PixelType* nextFrame = NULL;

	if (!stream->Capture(&nextFrame, 10000))
	{
		ROS_ERROR("Failed to capture next frame");
		return false;
	}

	static bool first_frame = true;
	if (first_frame)
	{
		size_t buffer_size = output_size * output_size * sizeof(uchar3);
		if (!cudaAllocMapped((void**)&left_gpu, buffer_size) ||
		    !cudaAllocMapped((void**)&right_gpu, buffer_size))
		{
			ROS_ERROR("Failed to allocate CUDA memory");
			return false;
		}
		ROS_INFO("Stereo initialized: %dx%d -> %dx%d (rotated_lenses=%s)",
		         stereo_width, stereo_height, output_size, output_size,
		         rotated_lenses ? "true" : "false");
		first_frame = false;
	}

	cudaError_t err;
	if (rotated_lenses)
		err = cudaStereoCropSplitRotate(nextFrame, left_gpu, right_gpu,
		                                 stereo_width, stereo_height, output_size, NULL);
	else
		err = cudaStereoCropSplit(nextFrame, left_gpu, right_gpu,
		                           stereo_width, stereo_height, output_size, NULL);

	if (err != cudaSuccess)
	{
		ROS_ERROR("CUDA stereo processing failed: %s", cudaGetErrorString(err));
		return false;
	}

	sensor_msgs::Image left_msg, right_msg;
	auto now = ROS_TIME_NOW();
	const size_t img_size = output_size * output_size * 3;

	left_msg.header.stamp = now;
	left_msg.header.frame_id = "stereo_left";
	left_msg.width = output_size;
	left_msg.height = output_size;
	left_msg.encoding = "rgb8";
	left_msg.step = output_size * 3;
	left_msg.is_bigendian = false;
	left_msg.data.assign(reinterpret_cast<uint8_t*>(left_gpu),
	                     reinterpret_cast<uint8_t*>(left_gpu) + img_size);
	left_image_pub->publish(left_msg);

	right_msg.header.stamp = now;
	right_msg.header.frame_id = "stereo_right";
	right_msg.width = output_size;
	right_msg.height = output_size;
	right_msg.encoding = "rgb8";
	right_msg.step = output_size * 3;
	right_msg.is_bigendian = false;
	right_msg.data.assign(reinterpret_cast<uint8_t*>(right_gpu),
	                      reinterpret_cast<uint8_t*>(right_gpu) + img_size);
	right_image_pub->publish(right_msg);

	std_msgs::msg::Header time_msg;
	std_msgs::msg::Float32 frate;
	time_msg.stamp = now;
	frate.data = stream->GetFrameRate();
	timestamp_pub->publish(time_msg);
	framerate_pub->publish(frate);

	return true;
}

int main(int argc, char **argv)
{
	ROS_CREATE_NODE("camera_stereo");

	videoOptions video_options;
	std::string resource_str, codec_str, flip_str;
	int framerate_int = 30;
	int num_buffers_int = 4;

	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("width", stereo_width);
	ROS_DECLARE_PARAMETER("height", stereo_height);
	ROS_DECLARE_PARAMETER("framerate", framerate_int);
	ROS_DECLARE_PARAMETER("flip", flip_str);
	ROS_DECLARE_PARAMETER("latency", video_options.latency);
	ROS_DECLARE_PARAMETER("num_buffers", num_buffers_int);
	ROS_DECLARE_PARAMETER("rotated_lenses", rotated_lenses);

	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", stereo_width);
	ROS_GET_PARAMETER("height", stereo_height);
	ROS_GET_PARAMETER("framerate", framerate_int);
	ROS_GET_PARAMETER("flip", flip_str);
	ROS_GET_PARAMETER("latency", video_options.latency);
	ROS_GET_PARAMETER("num_buffers", num_buffers_int);
	ROS_GET_PARAMETER("rotated_lenses", rotated_lenses);

	video_options.numBuffers = static_cast<uint32_t>(num_buffers_int);
	video_options.frameRate = static_cast<float>(framerate_int);

	if (resource_str.empty())
	{
		ROS_ERROR("Resource parameter not set");
		return 0;
	}

	if (!codec_str.empty())
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());
	if (!flip_str.empty())
		video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());

	video_options.width = stereo_width;
	video_options.height = stereo_height;
	output_size = stereo_height;

	ROS_INFO("Opening stereo camera: %s (%dx%d)", resource_str.c_str(), stereo_width, stereo_height);

	stream = videoSource::Create(resource_str.c_str(), video_options);
	if (!stream)
	{
		ROS_ERROR("Failed to open stereo camera");
		return 0;
	}

	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "left/image_raw", 10, left_image_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "right/image_raw", 10, right_image_pub);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Float32, "framerate", 5, framerate_pub);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Header, "time", 5, timestamp_pub);

	if (!stream->Open())
	{
		ROS_ERROR("Failed to start streaming");
		return 0;
	}

	ROS_INFO("Stereo camera node started");

	while (ROS_OK())
	{
		if (!acquireStereoFrame() && !stream->IsStreaming())
			break;
		if (ROS_OK())
			ROS_SPIN_ONCE();
	}

	delete stream;
	if (left_gpu) cudaFree(left_gpu);
	if (right_gpu) cudaFree(right_gpu);

	return 0;
}
