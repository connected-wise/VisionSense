/*
 * Stereo Camera Node
 * Crops center, splits, and optionally rotates stereo images
 * Input: 3840x1200 -> Crop center 1440px -> Two 1200x1200 outputs
 *
 * Optimized for low-latency publishing with:
 * - BEST_EFFORT QoS to prevent blocking on slow subscribers
 * - Pre-allocated message buffers
 * - CUDA stream synchronization
 */

#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <cuda_runtime.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <cstring>

extern "C" cudaError_t cudaStereoCropSplitRotate(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, int outputSize, cudaStream_t stream
);

extern "C" cudaError_t cudaStereoCropSplit(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, int outputSize, cudaStream_t stream
);

// flipMode: 0=none, 1=rotate-180, 2=vertical-flip, 3=horizontal-flip
extern "C" cudaError_t cudaStereoCropSplitFlip(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, int outputSize, int flipMode, cudaStream_t stream
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
bool enable_timing = false;

// CUDA flip mode (replaces slow GStreamer videoflip)
// Options: "", "rotate-180", "vertical-flip", "horizontal-flip"
std::string cuda_flip_str = "";
int cuda_flip_mode = 0;  // 0=none, 1=rotate-180, 2=vertical-flip, 3=horizontal-flip

// Pre-allocated message buffers to avoid repeated allocation
sensor_msgs::Image left_msg, right_msg;
bool buffers_initialized = false;

// Timing statistics
struct TimingStats {
	double capture_ms = 0;
	double cuda_ms = 0;
	double copy_ms = 0;
	double publish_ms = 0;
	int frame_count = 0;
};
TimingStats timing_stats;

inline double get_time_ms() {
	return std::chrono::duration<double, std::milli>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool acquireStereoFrame()
{
	double t0 = 0, t1 = 0, t2 = 0, t3 = 0, t4 = 0;

	if (enable_timing) t0 = get_time_ms();

	imageConverter::PixelType* nextFrame = NULL;

	if (!stream->Capture(&nextFrame, 10000))
	{
		ROS_ERROR("Failed to capture next frame");
		return false;
	}

	if (enable_timing) t1 = get_time_ms();

	const size_t img_size = output_size * output_size * 3;

	// Initialize buffers on first frame
	if (!buffers_initialized)
	{
		size_t buffer_size = output_size * output_size * sizeof(uchar3);
		if (!cudaAllocMapped((void**)&left_gpu, buffer_size) ||
		    !cudaAllocMapped((void**)&right_gpu, buffer_size))
		{
			ROS_ERROR("Failed to allocate CUDA memory");
			return false;
		}

		// Pre-allocate message data buffers
		left_msg.data.resize(img_size);
		right_msg.data.resize(img_size);

		// Set static message fields
		left_msg.header.frame_id = "stereo_left";
		left_msg.width = output_size;
		left_msg.height = output_size;
		left_msg.encoding = "rgb8";
		left_msg.step = output_size * 3;
		left_msg.is_bigendian = false;

		right_msg.header.frame_id = "stereo_right";
		right_msg.width = output_size;
		right_msg.height = output_size;
		right_msg.encoding = "rgb8";
		right_msg.step = output_size * 3;
		right_msg.is_bigendian = false;

		ROS_INFO("Stereo initialized: %dx%d -> %dx%d (rotated_lenses=%s)",
		         stereo_width, stereo_height, output_size, output_size,
		         rotated_lenses ? "true" : "false");
		buffers_initialized = true;
	}

	cudaError_t err;
	if (rotated_lenses)
		err = cudaStereoCropSplitRotate(nextFrame, left_gpu, right_gpu,
		                                 stereo_width, stereo_height, output_size, NULL);
	else
		err = cudaStereoCropSplitFlip(nextFrame, left_gpu, right_gpu,
		                               stereo_width, stereo_height, output_size, cuda_flip_mode, NULL);

	if (err != cudaSuccess)
	{
		ROS_ERROR("CUDA stereo processing failed: %s", cudaGetErrorString(err));
		return false;
	}

	// Synchronize CUDA to ensure kernel completion before CPU copy
	cudaDeviceSynchronize();

	if (enable_timing) t2 = get_time_ms();

	auto now = ROS_TIME_NOW();

	// Update timestamps
	left_msg.header.stamp = now;
	right_msg.header.stamp = now;

	// Fast memory copy using memcpy (data buffers already sized)
	std::memcpy(left_msg.data.data(), reinterpret_cast<uint8_t*>(left_gpu), img_size);
	std::memcpy(right_msg.data.data(), reinterpret_cast<uint8_t*>(right_gpu), img_size);

	if (enable_timing) t3 = get_time_ms();

	// Publish images
	left_image_pub->publish(left_msg);
	right_image_pub->publish(right_msg);

	// Publish timing info
	std_msgs::msg::Header time_msg;
	std_msgs::msg::Float32 frate;
	time_msg.stamp = now;
	frate.data = stream->GetFrameRate();
	timestamp_pub->publish(time_msg);
	framerate_pub->publish(frate);

	if (enable_timing) {
		t4 = get_time_ms();
		timing_stats.capture_ms += (t1 - t0);
		timing_stats.cuda_ms += (t2 - t1);
		timing_stats.copy_ms += (t3 - t2);
		timing_stats.publish_ms += (t4 - t3);
		timing_stats.frame_count++;

		// Print timing every 30 frames
		if (timing_stats.frame_count % 30 == 0) {
			int n = timing_stats.frame_count;
			ROS_INFO("Timing (avg ms): capture=%.1f cuda=%.1f copy=%.1f publish=%.1f total=%.1f",
			         timing_stats.capture_ms / n,
			         timing_stats.cuda_ms / n,
			         timing_stats.copy_ms / n,
			         timing_stats.publish_ms / n,
			         (timing_stats.capture_ms + timing_stats.cuda_ms +
			          timing_stats.copy_ms + timing_stats.publish_ms) / n);
		}
	}

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
	ROS_DECLARE_PARAMETER("enable_timing", enable_timing);
	ROS_DECLARE_PARAMETER("cuda_flip", cuda_flip_str);

	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", stereo_width);
	ROS_GET_PARAMETER("height", stereo_height);
	ROS_GET_PARAMETER("framerate", framerate_int);
	ROS_GET_PARAMETER("flip", flip_str);
	ROS_GET_PARAMETER("latency", video_options.latency);
	ROS_GET_PARAMETER("num_buffers", num_buffers_int);
	ROS_GET_PARAMETER("rotated_lenses", rotated_lenses);
	ROS_GET_PARAMETER("enable_timing", enable_timing);
	ROS_GET_PARAMETER("cuda_flip", cuda_flip_str);

	// Parse cuda_flip mode: "", "rotate-180", "vertical-flip", "horizontal-flip"
	if (cuda_flip_str == "rotate-180")
		cuda_flip_mode = 1;
	else if (cuda_flip_str == "vertical-flip")
		cuda_flip_mode = 2;
	else if (cuda_flip_str == "horizontal-flip")
		cuda_flip_mode = 3;
	else
		cuda_flip_mode = 0;  // no flip

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

	// Create publishers with BEST_EFFORT QoS to prevent blocking on slow subscribers
	// This is critical for maintaining high frame rates
	rclcpp::QoS qos_best_effort(2);
	qos_best_effort.best_effort();
	qos_best_effort.durability_volatile();

	left_image_pub = node->create_publisher<sensor_msgs::Image>("left/image_raw", qos_best_effort);
	right_image_pub = node->create_publisher<sensor_msgs::Image>("right/image_raw", qos_best_effort);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Float32, "framerate", 5, framerate_pub);
	ROS_CREATE_PUBLISHER(std_msgs::msg::Header, "time", 5, timestamp_pub);

	if (!stream->Open())
	{
		ROS_ERROR("Failed to start streaming");
		return 0;
	}

	ROS_INFO("Stereo camera node started (QoS: BEST_EFFORT, cuda_flip: %s, timing: %s)",
	         cuda_flip_str.empty() ? "none" : cuda_flip_str.c_str(),
	         enable_timing ? "enabled" : "disabled");

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
