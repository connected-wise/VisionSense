#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstring>

// globals
videoSource* stream = NULL;
Publisher<sensor_msgs::Image> image_pub = NULL;
Publisher<std_msgs::msg::Float32> framerate_pub = NULL;
Publisher<std_msgs::msg::Header> timestamp_pub = NULL;
imageConverter* image_cvt = NULL;

// Pre-allocated message buffer for zero-allocation publishing
sensor_msgs::Image img_msg;
bool msg_initialized = false;
bool enable_timing = false;

// Timing statistics
struct TimingStats {
	double capture_ms = 0;
	double convert_ms = 0;
	double publish_ms = 0;
	int frame_count = 0;
} timing;

inline double get_time_ms() {
	return std::chrono::duration<double, std::milli>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

// aquire and publish camera frame
bool aquireFrame()
{
	double t0 = 0, t1 = 0, t2 = 0, t3 = 0;
	if (enable_timing) t0 = get_time_ms();

	imageConverter::PixelType* nextFrame = NULL;

	// get the latest frame
	if( !stream->Capture(&nextFrame, 10000) )
	{
		ROS_ERROR("failed to capture next frame");
		return false;
	}

	if (enable_timing) t1 = get_time_ms();

	int width = stream->GetWidth();
	int height = stream->GetHeight();
	size_t img_size = width * height * 3;

	// Initialize message buffer on first frame
	if (!msg_initialized) {
		img_msg.data.resize(img_size);
		img_msg.header.frame_id = "camera";
		img_msg.encoding = "rgb8";
		img_msg.is_bigendian = false;
		msg_initialized = true;
		ROS_INFO("Camera initialized: %dx%d", width, height);
	}

	// assure correct image size
	if( !image_cvt->Resize(width, height, imageConverter::ROSOutputFormat) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	// Convert directly to pre-allocated buffer
	img_msg.width = width;
	img_msg.height = height;
	img_msg.step = width * 3;

	// Use imageConverter but copy to our pre-allocated buffer
	sensor_msgs::Image temp_msg;
	if( !image_cvt->Convert(temp_msg, imageConverter::ROSOutputFormat, nextFrame) )
	{
		ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
		return false;
	}

	// Fast copy to pre-allocated buffer
	std::memcpy(img_msg.data.data(), temp_msg.data.data(), img_size);

	if (enable_timing) t2 = get_time_ms();

	// populate timestamp in header field
	auto now = ROS_TIME_NOW();
	img_msg.header.stamp = now;

	// publish the message
	image_pub->publish(img_msg);

	std_msgs::msg::Header time_msg;
	std_msgs::msg::Float32 frate;
	time_msg.stamp = now;
	frate.data = stream->GetFrameRate();
	framerate_pub->publish(frate);
	timestamp_pub->publish(time_msg);

	if (enable_timing) {
		t3 = get_time_ms();
		timing.capture_ms += (t1 - t0);
		timing.convert_ms += (t2 - t1);
		timing.publish_ms += (t3 - t2);
		timing.frame_count++;

		if (timing.frame_count % 30 == 0) {
			int n = timing.frame_count;
			ROS_INFO("Timing (avg ms): capture=%.1f convert=%.1f publish=%.1f total=%.1f",
			         timing.capture_ms / n, timing.convert_ms / n,
			         timing.publish_ms / n,
			         (timing.capture_ms + timing.convert_ms + timing.publish_ms) / n);
		}
	}

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
	ROS_DECLARE_PARAMETER("enable_timing", enable_timing);

	//retrieve parameters
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", video_width);
	ROS_GET_PARAMETER("height", video_height);
	ROS_GET_PARAMETER("framerate", video_options.frameRate);
	ROS_GET_PARAMETER("loop", video_options.loop);
	ROS_GET_PARAMETER("flip", flip_str);
	ROS_GET_PARAMETER("latency", latency);
	ROS_GET_PARAMETER("enable_timing", enable_timing);
	
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


	// advertise publisher topics with BEST_EFFORT QoS to prevent subscriber back-pressure
	rclcpp::QoS qos_best_effort(2);
	qos_best_effort.best_effort();
	qos_best_effort.durability_volatile();
	image_pub = node->create_publisher<sensor_msgs::Image>("raw", qos_best_effort);
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
