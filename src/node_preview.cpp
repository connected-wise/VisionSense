#include "ros_compat.h"
#include "common.h"
#include "trtutil.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>

// Preview uses cv::imshow (one OpenCV window per topic) instead of
// jetson-utils videoOutput. Multiple videoOutputs all targeting display://0
// shared a single glDisplay surface and raced each other on Render() — only
// one stream was ever visible at a time. cv::imshow gives independent windows
// with proper auto-scaling and no contention.
int display_w = 960;
int display_h = 540;

static const char* kLeftWin      = "camera_stereo left";
static const char* kRightWin     = "camera_stereo right";
static const char* kDisparityWin = "stereo_depth disparity";
static const char* kDetectWin    = "detect overlay";
static const char* kLanesWin     = "lanes overlay";

static bool left_win_ready      = false;
static bool right_win_ready     = false;
static bool disparity_win_ready = false;
static bool detect_win_ready    = false;
static bool lanes_win_ready     = false;

static void ensureWindow(const char* name, bool& flag, int w, int h)
{
	if (flag) return;
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::resizeWindow(name, w, h);
	flag = true;
}

// Wrap a ROS image message as a BGR cv::Mat without copying when possible.
static cv::Mat msgToBgr(const sensor_msgs::ImageConstPtr& msg)
{
	const int w = static_cast<int>(msg->width);
	const int h = static_cast<int>(msg->height);
	const int step = static_cast<int>(msg->step);
	uint8_t* data = const_cast<uint8_t*>(msg->data.data());

	if (msg->encoding == "bgr8")
	{
		return cv::Mat(h, w, CV_8UC3, data, step);
	}
	if (msg->encoding == "rgb8")
	{
		cv::Mat src(h, w, CV_8UC3, data, step);
		cv::Mat dst;
		cv::cvtColor(src, dst, cv::COLOR_RGB2BGR);
		return dst;
	}
	if (msg->encoding == "mono8")
	{
		cv::Mat src(h, w, CV_8UC1, data, step);
		cv::Mat dst;
		cv::cvtColor(src, dst, cv::COLOR_GRAY2BGR);
		return dst;
	}
	ROS_INFO("preview: unsupported encoding '%s'", msg->encoding.c_str());
	return cv::Mat();
}

// Aspect-preserving downscale, never upscales.
static cv::Mat fitTo(const cv::Mat& src, int max_w, int max_h)
{
	if (max_w <= 0 || max_h <= 0) return src;
	const double scale_w = static_cast<double>(max_w) / src.cols;
	const double scale_h = static_cast<double>(max_h) / src.rows;
	const double scale   = std::min({scale_w, scale_h, 1.0});
	if (scale >= 0.999) return src;
	cv::Mat out;
	cv::resize(src, out, cv::Size(), scale, scale, cv::INTER_AREA);
	return out;
}

void signs_callback(const visionconnect::msg::Signs::SharedPtr input)
{
	ROS_INFO("%lu signs and lights detected", input->classes.size());
}

void left_camera_callback(const sensor_msgs::ImageConstPtr input)
{
	cv::Mat bgr = msgToBgr(input);
	if (bgr.empty()) return;
	cv::Mat shown = fitTo(bgr, display_w, display_h);
	ensureWindow(kLeftWin, left_win_ready, shown.cols, shown.rows);
	cv::imshow(kLeftWin, shown);
	cv::waitKey(1);
}

void right_camera_callback(const sensor_msgs::ImageConstPtr input)
{
	cv::Mat bgr = msgToBgr(input);
	if (bgr.empty()) return;
	cv::Mat shown = fitTo(bgr, display_w, display_h);
	ensureWindow(kRightWin, right_win_ready, shown.cols, shown.rows);
	cv::imshow(kRightWin, shown);
	cv::waitKey(1);
}

void disparity_callback(const sensor_msgs::ImageConstPtr input)
{
	cv::Mat bgr = msgToBgr(input);
	if (bgr.empty()) return;
	// Disparity color is typically 480×480 — show at native size.
	ensureWindow(kDisparityWin, disparity_win_ready, bgr.cols, bgr.rows);
	cv::imshow(kDisparityWin, bgr);
	cv::waitKey(1);
}

void detection_callback(const visionconnect::msg::Detect::SharedPtr input)
{
	auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->image);
	cv::Mat bgr = msgToBgr(img_msg);
	if (bgr.empty()) return;
	cv::Mat shown = fitTo(bgr, display_w, display_h);
	ensureWindow(kDetectWin, detect_win_ready, shown.cols, shown.rows);
	cv::imshow(kDetectWin, shown);
	cv::waitKey(1);
}

void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input)
{
	auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->laneimg);
	cv::Mat bgr = msgToBgr(img_msg);
	if (bgr.empty()) return;
	cv::Mat shown = fitTo(bgr, display_w, display_h);
	ensureWindow(kLanesWin, lanes_win_ready, shown.cols, shown.rows);
	cv::imshow(kLanesWin, shown);
	cv::waitKey(1);
}

int main(int argc, char **argv)
{
	ROS_CREATE_NODE("preview");
	std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");

	// `output` declared for back-compat with old configs that pass display://0
	// — we no longer use it (cv::imshow drives DISPLAY directly).
	std::string output;
	ROS_DECLARE_PARAMETER("output", output);
	ROS_DECLARE_PARAMETER("width", display_w);
	ROS_DECLARE_PARAMETER("height", display_h);
	ROS_GET_PARAMETER("output", output);
	ROS_GET_PARAMETER("width", display_w);
	ROS_GET_PARAMETER("height", display_h);

	rclcpp::QoS qos_best_effort(1);
	qos_best_effort.best_effort();
	qos_best_effort.durability_volatile();

	auto left_sub = node->create_subscription<sensor_msgs::Image>(
	    "left/image_in", qos_best_effort, left_camera_callback);
	auto right_sub = node->create_subscription<sensor_msgs::Image>(
	    "right/image_in", qos_best_effort, right_camera_callback);
	auto disparity_sub = node->create_subscription<sensor_msgs::Image>(
	    "disparity_in", qos_best_effort, disparity_callback);
	auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
	auto signs_sub  = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs,  "signs_in",  1, signs_callback);
	auto lanes_sub  = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes,  "lanes_in",  1, lanes_callback);

	ROS_INFO("Preview Node initialized (cv::imshow), display fit %dx%d, waiting for images",
	         display_w, display_h);
	ROS_SPIN();

	cv::destroyAllWindows();
	return 0;
}
