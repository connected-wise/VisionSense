#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
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

// Software white balance (white_balance parameter).
//
// With the IMX477 tuning shipped in JetPack 7.2 (imx477.nito) the image comes
// out magenta: the ISP subtracts too little black level, its auto white
// balance has no effect (wbmode auto and off give the same picture), and red
// and blue clip before green. Neither an ISP override file nor another
// tuning file fixes it on this release, so it is corrected here.
//
// The error is not a plain gain, so for every green level this measures what
// red and blue a neutral surface currently gets, and builds one lookup table
// per channel that maps those back onto green. Green is left as it is.
struct WhiteBalance {
	bool enabled = false;
	bool have_lut = false;
	float curve[2][256];   // red, blue: observed value -> corrected value
	cv::Mat lut;           // 1x256, 3 channels (R, G, B)
	int frame_count = 0;

	static constexpr int kBins = 32;       // green levels, 8 wide
	static constexpr int kMinPixels = 20;  // per bin, of the 8x subsample

	void update(const cv::Mat& rgb) {
		// Histograms of red and blue for each green level
		static uint16_t hist[2][kBins][256];
		static int count[kBins];

		// With an estimate in hand, measure only pixels that it turns
		// near-gray, so a large colored area (sky, grass, a red truck) does
		// not bend the curves. Start, or fall back, with every pixel.
		for (int pass = have_lut ? 0 : 1; pass < 2; pass++) {
			const bool near_gray_only = (pass == 0);
			std::memset(hist, 0, sizeof(hist));
			std::memset(count, 0, sizeof(count));
			long used = 0, valid = 0;
			for (int y = 0; y < rgb.rows; y += 8) {
				const uint8_t* row = rgb.ptr<uint8_t>(y);
				for (int x = 0; x < rgb.cols; x += 8) {
					const uint8_t* px = row + 3 * x;
					// Clipped pixels carry no color
					if (px[0] > 245 || px[1] > 245 || px[2] > 245) continue;
					valid++;
					if (near_gray_only) {
						float r = curve[0][px[0]], g = px[1], b = curve[1][px[2]];
						float hi = std::max(r, std::max(g, b));
						float lo = std::min(r, std::min(g, b));
						if (hi - lo > 0.15f * hi + 8.0f) continue;
					}
					int bin = px[1] >> 3;
					if (hist[0][bin][px[0]] < 65535) hist[0][bin][px[0]]++;
					if (hist[1][bin][px[2]] < 65535) hist[1][bin][px[2]]++;
					count[bin]++;
					used++;
				}
			}
			if (valid < 200) return;                 // nothing to measure
			if (near_gray_only && used < valid / 8) continue;  // too few: use all

			for (int c = 0; c < 2; c++) {
				// Points (median of this channel, green level), made monotonic
				float px_val[kBins + 2], green[kBins + 2];
				int n = 0;
				px_val[n] = 0.0f; green[n++] = 0.0f;
				for (int bin = 0; bin < kBins; bin++) {
					if (count[bin] < kMinPixels) continue;
					int half = count[bin] / 2, acc = 0, median = 0;
					for (int v = 0; v < 256; v++) {
						acc += hist[c][bin][v];
						if (acc > half) { median = v; break; }
					}
					if ((float)median <= px_val[n - 1]) continue;
					px_val[n] = (float)median; green[n++] = bin * 8 + 4.0f;
				}
				if (n < 4) return;                   // too little of the range
				if (px_val[n - 1] < 255.0f) { px_val[n] = 255.0f; green[n++] = 255.0f; }

				// Interpolate to a full table and follow changes slowly
				const float alpha = have_lut ? 0.2f : 1.0f;
				int k = 0;
				for (int v = 0; v < 256; v++) {
					while (k < n - 2 && (float)v > px_val[k + 1]) k++;
					float t = (v - px_val[k]) / std::max(1.0f, px_val[k + 1] - px_val[k]);
					float target = green[k] + std::min(1.0f, std::max(0.0f, t)) * (green[k + 1] - green[k]);
					curve[c][v] = have_lut ? curve[c][v] + alpha * (target - curve[c][v]) : target;
				}
			}
			break;
		}
		have_lut = true;

		lut.create(1, 256, CV_8UC3);
		for (int v = 0; v < 256; v++)
			lut.at<cv::Vec3b>(0, v) = cv::Vec3b(cv::saturate_cast<uint8_t>(curve[0][v]),
			                                    (uint8_t)v,
			                                    cv::saturate_cast<uint8_t>(curve[1][v]));
	}

	// src and dst: 8-bit RGB buffers of the same size
	void apply(const uint8_t* src, uint8_t* dst, int width, int height) {
		cv::Mat in(height, width, CV_8UC3, const_cast<uint8_t*>(src));
		// Re-measure about three times a second
		if (frame_count++ % 10 == 0) update(in);
		if (!have_lut) {
			std::memcpy(dst, src, (size_t)width * height * 3);
			return;
		}
		cv::Mat out(height, width, CV_8UC3, dst);
		cv::LUT(in, lut, out);
	}
} white_balance;

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

	// Fast copy to pre-allocated buffer (color-corrected on the way if asked)
	if (white_balance.enabled)
		white_balance.apply(temp_msg.data.data(), img_msg.data.data(), width, height);
	else
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
	ROS_DECLARE_PARAMETER("white_balance", white_balance.enabled);

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
	ROS_GET_PARAMETER("white_balance", white_balance.enabled);
	
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
	
	if( white_balance.enabled )
		ROS_INFO("software white balance is on");

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
	// Argus sometimes aborts a capture session for good ("CaptureAborted",
	// seen with the IMX477 a few minutes into a run): every Capture() then
	// times out although the stream still counts as open. A new session works,
	// so give up after a few timeouts and let the launch file respawn the node.
	const int max_failed_captures = 3;
	int failed_captures = 0;

	while( ROS_OK() )
	{
		if( !aquireFrame() )
		{
			if( !stream->IsStreaming() )
			{
				ROS_INFO("stream is closed or reached EOS, exiting node...");
				break;
			}
			if( ++failed_captures >= max_failed_captures )
			{
				ROS_ERROR("%d captures in a row failed, exiting node so it can be restarted", failed_captures);
				break;
			}
		}
		else
			failed_captures = 0;
		if( ROS_OK() )
			ROS_SPIN_ONCE();
	}

	// free resources
	delete stream;
	delete image_cvt;

	return 0;
}
