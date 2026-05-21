/*
 * Stereo Camera Node
 * Splits, crops, and optionally rotates the captured stereo frame.
 *
 * Input: 3840x1200 stereo (1920x1200 per eye)
 *
 * Output per eye depends on rotated_lenses:
 *   - rotated_lenses=false (default): 1440x900 — inner-edge horizontal crop
 *     (1920 → 1440, 480 px removed from the outer side of each eye) plus
 *     symmetric vertical crop (1200 → 900, 150 px top + 150 px bottom).
 *     See STEREO_OUT_W / STEREO_OUT_H in src/cuda/stereo_rotate.cu.
 *   - rotated_lenses=true (legacy): 1200x1200 square. The 90° per-eye
 *     rotation maps sensor rows (1200) to natural-width, so this path
 *     cannot produce non-square outputs larger than 1200.
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
#include <sensor_msgs/msg/camera_info.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstring>
#include <fstream>

// Square legacy path (rotated_lenses=true). Takes outputSize (= stereo_height).
extern "C" cudaError_t cudaStereoCropSplitRotate(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, int outputSize, cudaStream_t stream
);

// Non-rotated paths. Output dims are baked into the kernel (STEREO_OUT_W ×
// STEREO_OUT_H, currently 1440×900), so no outputSize argument is needed.
extern "C" cudaError_t cudaStereoCropSplit(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, cudaStream_t stream
);

// flipMode: 0=none, 1=rotate-180, 2=vertical-flip, 3=horizontal-flip
extern "C" cudaError_t cudaStereoCropSplitFlip(
    const uchar3* input, uchar3* leftOut, uchar3* rightOut,
    int stereoWidth, int stereoHeight, int flipMode, cudaStream_t stream
);

videoSource* stream = NULL;
Publisher<sensor_msgs::Image> left_image_pub = NULL;
Publisher<sensor_msgs::Image> right_image_pub = NULL;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub  = nullptr;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub = nullptr;
Publisher<std_msgs::msg::Float32> framerate_pub = NULL;
Publisher<std_msgs::msg::Header> timestamp_pub = NULL;

uchar3* left_gpu = NULL;
uchar3* right_gpu = NULL;

int stereo_width  = 3840;
int stereo_height = 1200;
// Output per eye. Set in main() from rotated_lenses:
//   - false → 1440 × 900  (must match STEREO_OUT_W / STEREO_OUT_H in stereo_rotate.cu)
//   - true  → 1200 × 1200 (legacy square, ties to stereo_height for the rotated kernel)
int output_w = 1200;
int output_h = 1200;
bool rotated_lenses = true;
bool enable_timing = false;

// CUDA flip mode (replaces slow GStreamer videoflip)
// Options: "", "rotate-180", "vertical-flip", "horizontal-flip"
std::string cuda_flip_str = "";
int cuda_flip_mode = 0;  // 0=none, 1=rotate-180, 2=vertical-flip, 3=horizontal-flip

// Pre-allocated message buffers to avoid repeated allocation
sensor_msgs::Image left_msg, right_msg;
sensor_msgs::msg::CameraInfo left_info_msg, right_info_msg;
bool buffers_initialized = false;

// Optical geometry — set in main() once params are read. We pre-fill the
// CameraInfo messages on first frame and only re-stamp them per cycle.
double baseline_m_      = 0.1013;
double focal_length_mm_ = 3.6;
double sensor_width_mm_ = 2.7;     // post-crop equivalent width (FFS-side); not used for K
double pixel_pitch_mm_  = 0.003;   // AR0234 = 3.0 µm; defines fx/fy in pixels for camera_info

// If non-empty, node loads calibrated K/D/R/P from this YAML (produced by
// scripts/compute_stereo_calib.py) and uses them in the published CameraInfo
// — required for NITROS RectifyNode to produce geometrically correct rectified
// images. Empty → falls back to the focal/pixel-pitch math below (identity R,
// zero D, principal point at image center).
std::string calibration_file_;

// Helper: copy a flat YAML data list into a fixed-size CameraInfo array field.
template <size_t N>
static bool loadMatrix(const YAML::Node& root, const std::string& key,
                       std::array<double, N>& dst, size_t expected_rows, size_t expected_cols)
{
	if (!root[key] || !root[key]["data"]) {
		ROS_ERROR("calibration YAML missing %s/data", key.c_str());
		return false;
	}
	auto rows = root[key]["rows"].as<size_t>();
	auto cols = root[key]["cols"].as<size_t>();
	auto data = root[key]["data"].as<std::vector<double>>();
	if (rows != expected_rows || cols != expected_cols || data.size() != N) {
		ROS_ERROR("calibration %s shape mismatch: got %zux%zu (%zu vals), expected %zux%zu (%zu vals)",
		          key.c_str(), rows, cols, data.size(), expected_rows, expected_cols, N);
		return false;
	}
	for (size_t i = 0; i < N; ++i) dst[i] = data[i];
	return true;
}

static bool loadCalibrationInto(const std::string& path,
                                sensor_msgs::msg::CameraInfo& left,
                                sensor_msgs::msg::CameraInfo& right)
{
	std::ifstream f(path);
	if (!f.good()) {
		ROS_ERROR("calibration_file not readable: %s", path.c_str());
		return false;
	}
	YAML::Node root;
	try {
		root = YAML::LoadFile(path);
	} catch (const YAML::Exception& e) {
		ROS_ERROR("YAML parse error in %s: %s", path.c_str(), e.what());
		return false;
	}

	if (!root["image_width"] || !root["image_height"]) {
		ROS_ERROR("calibration YAML missing image_width/image_height");
		return false;
	}
	const uint32_t w = root["image_width"].as<uint32_t>();
	const uint32_t h = root["image_height"].as<uint32_t>();
	if (w != left.width || h != left.height) {
		ROS_ERROR("calibration image size %ux%u doesn't match output %ux%u — recapture or rescale",
		          w, h, left.width, left.height);
		return false;
	}

	auto fillEye = [&](sensor_msgs::msg::CameraInfo& m, const std::string& Kkey,
	                   const std::string& Dkey, const std::string& Rkey,
	                   const std::string& Pkey) -> bool {
		// D: variable length; resize to whatever the YAML carries (typically 5 for plumb_bob).
		auto Dvec = root[Dkey]["data"].as<std::vector<double>>();
		m.d.assign(Dvec.begin(), Dvec.end());
		m.distortion_model = "plumb_bob";
		if (!loadMatrix<9> (root, Kkey, m.k, 3, 3)) return false;
		if (!loadMatrix<9> (root, Rkey, m.r, 3, 3)) return false;
		if (!loadMatrix<12>(root, Pkey, m.p, 3, 4)) return false;
		return true;
	};
	if (!fillEye(left,  "K_left",  "D_left",  "R1", "P1")) return false;
	if (!fillEye(right, "K_right", "D_right", "R2", "P2")) return false;

	ROS_INFO("Loaded stereo calibration from %s "
	         "(fx_rect=%.2f, baseline=%.3fm derived from P2[0,3])",
	         path.c_str(), left.p[0],
	         (right.p[3] != 0.0 ? -right.p[3] / right.p[0] : 0.0));
	return true;
}

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

	const size_t img_size = static_cast<size_t>(output_w) * output_h * 3;

	// Initialize buffers on first frame
	if (!buffers_initialized)
	{
		size_t buffer_size = static_cast<size_t>(output_w) * output_h * sizeof(uchar3);
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
		left_msg.width  = output_w;
		left_msg.height = output_h;
		left_msg.encoding = "rgb8";
		left_msg.step = output_w * 3;
		left_msg.is_bigendian = false;

		right_msg.header.frame_id = "stereo_right";
		right_msg.width  = output_w;
		right_msg.height = output_h;
		right_msg.encoding = "rgb8";
		right_msg.step = output_w * 3;
		right_msg.is_bigendian = false;

		// Populate static CameraInfo. Two paths:
		//   1. calibration_file_ set → load real K/D/R/P from the YAML produced
		//      by scripts/compute_stereo_calib.py. This is what NITROS
		//      RectifyNode needs to undistort+rectify.
		//   2. empty → fall back to focal/pixel-pitch math: identity R, zero D,
		//      fx=fy from optics, principal point at image center, P with
		//      -fx*baseline on the right. Geometrically correct only if the
		//      pair is already rectified (which on this rig it isn't).
		auto seedEye = [&](sensor_msgs::msg::CameraInfo& m, const std::string& frame) {
			m.header.frame_id = frame;
			m.width  = output_w;
			m.height = output_h;
			m.binning_x = m.binning_y = 0;
			m.roi.do_rectify = false;
		};
		seedEye(left_info_msg,  "stereo_left");
		seedEye(right_info_msg, "stereo_right");

		bool loaded_from_file = false;
		if (!calibration_file_.empty()) {
			loaded_from_file = loadCalibrationInto(calibration_file_,
			                                       left_info_msg, right_info_msg);
			if (!loaded_from_file) {
				ROS_ERROR("calibration_file set but load failed — using focal/pixel-pitch fallback");
			}
		}
		if (!loaded_from_file) {
			const double fx_px = focal_length_mm_ / pixel_pitch_mm_;
			const double fy_px = fx_px;
			const double cx = output_w / 2.0;
			const double cy = output_h / 2.0;
			auto fillUncalibrated = [&](sensor_msgs::msg::CameraInfo& m, double tx) {
				m.distortion_model = "plumb_bob";
				m.d.assign(5, 0.0);
				m.k = {fx_px, 0.0, cx,  0.0, fy_px, cy,  0.0, 0.0, 1.0};
				m.r = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
				m.p = {fx_px, 0.0, cx,  tx,  0.0, fy_px, cy, 0.0,  0.0, 0.0, 1.0, 0.0};
			};
			fillUncalibrated(left_info_msg,  0.0);
			fillUncalibrated(right_info_msg, -fx_px * baseline_m_);
			ROS_WARN("Using uncalibrated CameraInfo (focal/pixel-pitch math). "
			         "Set the `calibration_file` parameter to a YAML produced by "
			         "scripts/compute_stereo_calib.py for NITROS rectification.");
		}

		ROS_INFO("Stereo initialized: %dx%d -> %dx%d (rotated_lenses=%s)",
		         stereo_width, stereo_height, output_w, output_h,
		         rotated_lenses ? "true" : "false");
		buffers_initialized = true;
	}

	cudaError_t err;
	if (rotated_lenses)
		err = cudaStereoCropSplitRotate(nextFrame, left_gpu, right_gpu,
		                                 stereo_width, stereo_height, output_w, NULL);
	else
		// Sensor cols [0..stereo_width/2) → leftOut → /camera_stereo/left.
		// Sensor cols [stereo_width/2..stereo_width) → rightOut → /camera_stereo/right.
		// No L↔R buffer swap: topics carry the raw split-half assignment of the
		// Arducam sensor, matching the current physical wiring. NOTE: the
		// stereo_calib.yaml in this tree predates the physical sensor swap and
		// must be regenerated (capture_stereo_calib.py + compute_stereo_calib.py)
		// before stereo depth output is geometrically meaningful.
		err = cudaStereoCropSplitFlip(nextFrame, left_gpu, right_gpu,
		                               stereo_width, stereo_height, cuda_flip_mode, NULL);

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

	// Stamp + publish CameraInfo. Static fields populated once at init.
	if (left_info_pub && right_info_pub) {
		left_info_msg.header.stamp  = now;
		right_info_msg.header.stamp = now;
		left_info_pub->publish(left_info_msg);
		right_info_pub->publish(right_info_msg);
	}

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
	// Optical geometry — used to populate CameraInfo (K/P) for the ESS path and
	// any other consumer that needs intrinsics. sensor_width_mm should reflect
	// what the consumer downstream actually sees (e.g. 2.7 for the 900×900
	// center-crop the FFS stereo_depth uses).
	ROS_DECLARE_PARAMETER("focal_length_mm", focal_length_mm_);
	ROS_DECLARE_PARAMETER("sensor_width_mm", sensor_width_mm_);
	ROS_DECLARE_PARAMETER("baseline_mm",     baseline_m_ * 1000.0);
	// Pixel pitch in millimetres. Drives camera_info's fx/fy:
	//   fx_px = focal_mm / pixel_pitch_mm. AR0234 default = 3.0 µm = 0.003 mm.
	ROS_DECLARE_PARAMETER("pixel_pitch_mm", pixel_pitch_mm_);
	// Path to a stereo_calib.yaml from scripts/compute_stereo_calib.py. If set,
	// camera_info is populated with calibrated K/D/R/P per eye (required by
	// NITROS RectifyNode). If empty, falls back to focal/pixel-pitch math.
	ROS_DECLARE_PARAMETER("calibration_file", calibration_file_);

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
	double baseline_mm_param = 0.0;
	ROS_GET_PARAMETER("focal_length_mm", focal_length_mm_);
	ROS_GET_PARAMETER("sensor_width_mm", sensor_width_mm_);
	ROS_GET_PARAMETER("baseline_mm",     baseline_mm_param);
	ROS_GET_PARAMETER("pixel_pitch_mm",  pixel_pitch_mm_);
	ROS_GET_PARAMETER("calibration_file", calibration_file_);
	if (baseline_mm_param > 0) baseline_m_ = baseline_mm_param / 1000.0;

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

	// Pick output dims per rotation path. The non-rotated kernel hardcodes
	// STEREO_OUT_W × STEREO_OUT_H (1440×900), so we mirror those here for the
	// message dimensions / buffer sizing.
	if (rotated_lenses) {
		output_w = stereo_height;  // 1200
		output_h = stereo_height;  // 1200 (square, legacy)
	} else {
		output_w = 1440;
		output_h = 900;
	}

	ROS_INFO("Opening stereo camera: %s (%dx%d)", resource_str.c_str(), stereo_width, stereo_height);

	stream = videoSource::Create(resource_str.c_str(), video_options);
	if (!stream)
	{
		ROS_ERROR("Failed to open stereo camera");
		return 0;
	}

	// Publish BEST_EFFORT — what FFS used originally and what gave best perf.
	// Drops over RELIABLE handshakes / back-pressure when a downstream node is
	// slow. NITROS ESS RectifyNode WILL refuse incompatible-QoS messages with
	// a "no messages sent" warning if you switch to that backend; bump these
	// to .reliable() in that case.
	rclcpp::QoS qos_pub(2);
	qos_pub.best_effort();
	qos_pub.durability_volatile();

	left_image_pub  = node->create_publisher<sensor_msgs::Image>("left/image_raw",  qos_pub);
	right_image_pub = node->create_publisher<sensor_msgs::Image>("right/image_raw", qos_pub);
	left_info_pub   = node->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info",  qos_pub);
	right_info_pub  = node->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", qos_pub);
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
