#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "trtutil.h"
#include <X11/Xlib.h>

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "visionconnect/msg/adas.hpp"
#include "visionconnect/msg/scene_data.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <thread>
#ifdef Success
#undef Success  // X11 defines 'Success' which conflicts with Eigen
#endif
#include <Eigen/Dense>

// Background MP4 writer: callers push frames from the display loop; a worker
// thread does the actual encode. The queue is bounded and drops on overflow so
// the display loop is never blocked by a slow disk or codec.
class FrameRecorder {
public:
    FrameRecorder() = default;
    ~FrameRecorder() { stop(); }

    void configure(const std::string& path, int fps) {
        path_ = path;
        fps_  = fps > 0 ? fps : 30;
    }

    // Open the writer once we know the frame size and start the worker.
    bool open(const cv::Size& size) {
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        if (!writer_.open(path_, fourcc, fps_, size, /*isColor=*/true)) {
            return false;
        }
        running_ = true;
        worker_  = std::thread(&FrameRecorder::run, this);
        return true;
    }

    bool is_open() const { return writer_.isOpened(); }

    // Non-blocking enqueue. Drops the frame if the queue is already full so the
    // 30 Hz display callback never stalls on disk I/O.
    void push(const cv::Mat& frame) {
        if (!running_) return;
        {
            std::lock_guard<std::mutex> lk(m_);
            if (q_.size() >= kMaxQueue) {
                ++dropped_;
                return;
            }
            q_.emplace_back(frame.clone());
        }
        cv_.notify_one();
    }

    void stop() {
        if (!running_.exchange(false)) return;
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        if (writer_.isOpened()) writer_.release();
    }

    size_t dropped() const { return dropped_.load(); }
    const std::string& path() const { return path_; }

private:
    void run() {
        while (true) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [&] { return !q_.empty() || !running_; });
                if (!running_ && q_.empty()) break;
                frame = std::move(q_.front());
                q_.pop_front();
            }
            writer_.write(frame);
        }
    }

    static constexpr size_t kMaxQueue = 4;
    std::string                 path_;
    int                         fps_ = 30;
    cv::VideoWriter             writer_;
    std::thread                 worker_;
    std::mutex                  m_;
    std::condition_variable     cv_;
    std::deque<cv::Mat>         q_;
    std::atomic<bool>           running_{false};
    std::atomic<size_t>         dropped_{0};
};

static FrameRecorder g_recorder;
static bool          g_record_enabled = false;

visionconnect::msg::Detect::SharedPtr detect_msg = NULL;
visionconnect::msg::Signs::SharedPtr signs_msg = NULL;
visionconnect::msg::Lanes::SharedPtr lanes_msg = NULL;
visionconnect::msg::ADAS::SharedPtr adas_msg = NULL;

int screen_width = 1920;
int screen_height = 1080;

// When false, the gui node still publishes /gui/fusion (for the web
// dashboard) but skips creating the native cv::imshow window. Driven by the
// `display` ROS parameter, which defaults to true to preserve legacy behavior.
static bool g_display_enabled = true;

// Additional data streams for enhanced GUI
cv::Mat driver_monitor_image;
cv::Mat stereo_depth_colored;      // Pre-colorized disparity for display (updated in callback)
cv::Mat last_main_view;            // Last fused main view (for timer-based refresh)
cv::Mat bev_display;               // Bird's Eye View image from BEV node
std::mutex depth_mutex;
std::mutex main_view_mutex;
std::mutex bev_mutex;
std::string driver_state = "UNKNOWN";
double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
double latitude = 0.0, longitude = 0.0, altitude = 0.0;
int gps_satellites = 0;
std::string gps_fix_status = "NO FIX";

// Depth legend config (must match stereo_depth node's max_depth_m)
constexpr float DEPTH_LEGEND_MAX_M = 50.0f;

// 3D bounding box support
cv::Mat stereo_depth_raw;             // Raw 32FC1 depth map in meters
std::mutex raw_depth_mutex;
bool enable_3d_boxes = true;

// Camera intrinsics (1200x1200 stereo left image)
constexpr float CAM_FX = 750.0f;
constexpr float CAM_FY = 750.0f;
constexpr float CAM_CX = 600.0f;
constexpr float CAM_CY = 600.0f;

// Class-based size priors {length, width, height} in meters
// Index: 0=pedestrian, 1=cyclist, 2=car, 3=bus, 4=truck, 5=train,
//        6=traffic light (placeholder), 7=traffic sign (placeholder), 8=bike
struct SizePrior { float length, width, height; };
constexpr SizePrior SIZE_PRIORS[] = {
    {0.5f, 0.5f, 1.7f},   // pedestrian
    {1.8f, 0.8f, 1.7f},   // cyclist
    {4.5f, 1.8f, 1.5f},   // car
    {12.0f, 2.6f, 3.5f},  // bus
    {8.0f, 2.5f, 3.0f},   // truck
    {15.0f, 3.0f, 4.0f},  // train
    {0.3f, 0.3f, 0.6f},   // traffic light (placeholder, not drawn in 3D)
    {0.5f, 0.5f, 0.6f},   // traffic sign  (placeholder, not drawn in 3D)
    {1.8f, 0.7f, 1.1f},   // bike (rider-less bicycle footprint)
};
static constexpr int NUM_SIZE_PRIORS = sizeof(SIZE_PRIORS) / sizeof(SIZE_PRIORS[0]);

struct OrientedBox3D {
    float cx, cy, cz;           // 3D center in camera frame (meters)
    float length, width, height; // oriented dimensions
    float yaw;                   // rotation around Y-axis (radians)
    bool valid = false;
};

// Declare publishers
Publisher<sensor_msgs::Image> gui_pub = NULL;
Publisher<visionconnect::msg::SceneData> scene_pub = NULL;

// Scene data tracking variables
auto scene_start_time = std::chrono::steady_clock::now();
std::string previous_ego_lane = "V22";  // Assume starting in center lane
std::string current_ego_lane = "V22";

// Global quadrant assignments (populated by fuse_data, used by publishSceneData)
std::string quadrant_v11 = "None";
std::string quadrant_v12 = "None";
std::string quadrant_v13 = "None";
std::string quadrant_v21 = "None";
std::string quadrant_v22 = "None";
std::string quadrant_v23 = "None";

// Global lane region availability (populated by fuse_data, used by publishSceneData)
bool lane_region_1_available = false;  // Left lane region (between lines 0-1)
bool lane_region_2_available = false;  // Center lane region (between lines 1-2)
bool lane_region_3_available = false;  // Right lane region (between lines 2-3)

// Helper function to create data summary panel
cv::Mat createSummaryPanel(int width, int height)
{
    cv::Mat panel(height, width, CV_8UC3, cv::Scalar(30, 30, 30));

    int margin = 15;
    int col_width = width / 2;
    int line_height = 28;
    int y = margin;

    cv::Scalar title_color(255, 200, 0);
    cv::Scalar data_color(255, 255, 255);
    cv::Scalar alert_color(0, 255, 0);
    cv::Scalar warn_color(0, 165, 255);
    cv::Scalar danger_color(0, 0, 255);

    // Title
    cv::putText(panel, "SYSTEM STATUS", cv::Point(margin, y + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, title_color, 2);
    y += 35;

    // Column 1: Detections
    int col1_x = margin;
    int col1_y = y;

    cv::putText(panel, "DETECTIONS", cv::Point(col1_x, col1_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, title_color, 1);
    col1_y += line_height;

    int num_vehicles = 0, num_pedestrians = 0, num_cyclists = 0, num_bikes = 0;
    if (detect_msg != NULL) {
        for (uint16_t i = 0; i < detect_msg->num_detections; ++i) {
            int cls = static_cast<int>(detect_msg->classes[i]);
            if (cls >= 2 && cls <= 4) num_vehicles++;
            else if (cls == 0) num_pedestrians++;
            else if (cls == 1) num_cyclists++;
            else if (cls == 8) num_bikes++;
        }
    }

    char text[128];
    snprintf(text, sizeof(text), "Vehicles: %d", num_vehicles);
    cv::putText(panel, text, cv::Point(col1_x, col1_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col1_y += line_height;

    snprintf(text, sizeof(text), "Pedestrians: %d", num_pedestrians);
    cv::putText(panel, text, cv::Point(col1_x, col1_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col1_y += line_height;

    snprintf(text, sizeof(text), "Bikes: %d", num_bikes);
    cv::putText(panel, text, cv::Point(col1_x, col1_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col1_y += line_height;

    snprintf(text, sizeof(text), "Cyclists: %d", num_cyclists);
    cv::putText(panel, text, cv::Point(col1_x, col1_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col1_y += line_height;

    int num_lanes = (lanes_msg != NULL) ? lanes_msg->num_lanes : 0;
    snprintf(text, sizeof(text), "Lanes: %d", num_lanes);
    cv::putText(panel, text, cv::Point(col1_x, col1_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col1_y += line_height;

    // Driver state
    cv::Scalar state_color = data_color;
    if (driver_state == "DROWSY" || driver_state == "NO_DRIVER") {
        state_color = danger_color;
    } else if (driver_state == "DISTRACTED") {
        state_color = warn_color;
    } else if (driver_state == "ALERT") {
        state_color = alert_color;
    }
    snprintf(text, sizeof(text), "Driver: %s", driver_state.c_str());
    cv::putText(panel, text, cv::Point(col1_x, col1_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, state_color, 1);

    // Column 2: GPS & IMU
    int col2_x = col1_x + col_width;
    int col2_y = y;

    cv::putText(panel, "GPS / IMU", cv::Point(col2_x, col2_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, title_color, 1);
    col2_y += line_height;

    cv::Scalar fix_color = (gps_satellites > 0) ? alert_color : danger_color;
    snprintf(text, sizeof(text), "Fix: %s (%d sats)", gps_fix_status.c_str(), gps_satellites);
    cv::putText(panel, text, cv::Point(col2_x, col2_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, fix_color, 1);
    col2_y += line_height;

    snprintf(text, sizeof(text), "Lat: %.6f", latitude);
    cv::putText(panel, text, cv::Point(col2_x, col2_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col2_y += line_height;

    snprintf(text, sizeof(text), "Lon: %.6f", longitude);
    cv::putText(panel, text, cv::Point(col2_x, col2_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col2_y += line_height;

    snprintf(text, sizeof(text), "Alt: %.1f m", altitude);
    cv::putText(panel, text, cv::Point(col2_x, col2_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col2_y += line_height;

    snprintf(text, sizeof(text), "IMU: %.1f, %.1f, %.1f", accel_x, accel_y, accel_z);
    cv::putText(panel, text, cv::Point(col2_x, col2_y), cv::FONT_HERSHEY_SIMPLEX, 0.45, data_color, 1);

    return panel;
}

// Draw compact status overlay on the main detection image (bottom-left corner)
void drawStatusOverlay(cv::Mat& img)
{
    if (img.empty()) return;

    // --- Gather data ---
    int nv = 0, np = 0, nc = 0, nb = 0;
    if (detect_msg != NULL) {
        for (uint16_t i = 0; i < detect_msg->num_detections; ++i) {
            int cls = static_cast<int>(detect_msg->classes[i]);
            if (cls >= 2 && cls <= 4) nv++;
            else if (cls == 0) np++;
            else if (cls == 1) nc++;
            else if (cls == 8) nb++;
        }
    }
    int nl = (lanes_msg != NULL) ? lanes_msg->num_lanes : 0;

    // --- Layout ---
    const int row_h = 16;
    const int pad_x = 8;
    const int pad_y = 5;
    const int rows = 3;
    const int bar_h = pad_y * 2 + rows * row_h;
    const int bar_w = 330;
    const int ox = 6;
    const int oy = img.rows - bar_h - 6;
    const double fs = 0.34;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    // --- Background: rounded-corner look via filled rect + border ---
    cv::Rect bg(ox, oy, bar_w, bar_h);
    bg &= cv::Rect(0, 0, img.cols, img.rows);
    if (bg.width <= 0 || bg.height <= 0) return;

    // Darken background
    img(bg) *= 0.25;
    // Subtle border
    cv::rectangle(img, bg, cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    // Top accent line (amber)
    cv::line(img, cv::Point(bg.x + 1, bg.y), cv::Point(bg.x + bg.width - 1, bg.y),
             cv::Scalar(255, 200, 0), 1, cv::LINE_AA);

    // --- Colors ---
    cv::Scalar col_label(130, 130, 140);   // dim label
    cv::Scalar col_value(220, 220, 225);   // bright value
    cv::Scalar col_accent(255, 200, 0);    // amber accent
    cv::Scalar col_green(80, 230, 80);
    cv::Scalar col_red(80, 80, 240);
    cv::Scalar col_warn(0, 165, 255);

    int tx = ox + pad_x;
    int ty = oy + pad_y + 11;
    char text[64];

    // --- Row 1: Detection counts with colored badges + driver state ---
    // Vehicle count (green tint)
    snprintf(text, sizeof(text), "%d", nv);
    cv::putText(img, text, cv::Point(tx, ty), font, fs, col_green, 1, cv::LINE_AA);
    int tw = cv::getTextSize(text, font, fs, 1, nullptr).width;
    cv::putText(img, "V", cv::Point(tx + tw + 1, ty), font, fs * 0.85, col_label, 1, cv::LINE_AA);
    tx += tw + 14;

    // Pedestrian count (amber)
    snprintf(text, sizeof(text), "%d", np);
    cv::putText(img, text, cv::Point(tx, ty), font, fs, col_accent, 1, cv::LINE_AA);
    tw = cv::getTextSize(text, font, fs, 1, nullptr).width;
    cv::putText(img, "P", cv::Point(tx + tw + 1, ty), font, fs * 0.85, col_label, 1, cv::LINE_AA);
    tx += tw + 14;

    // Cyclist count (cyan)
    snprintf(text, sizeof(text), "%d", nc);
    cv::putText(img, text, cv::Point(tx, ty), font, fs, cv::Scalar(255, 200, 100), 1, cv::LINE_AA);
    tw = cv::getTextSize(text, font, fs, 1, nullptr).width;
    cv::putText(img, "C", cv::Point(tx + tw + 1, ty), font, fs * 0.85, col_label, 1, cv::LINE_AA);
    tx += tw + 14;

    // Bike count (light green)
    snprintf(text, sizeof(text), "%d", nb);
    cv::putText(img, text, cv::Point(tx, ty), font, fs, cv::Scalar(180, 240, 90), 1, cv::LINE_AA);
    tw = cv::getTextSize(text, font, fs, 1, nullptr).width;
    cv::putText(img, "B", cv::Point(tx + tw + 1, ty), font, fs * 0.85, col_label, 1, cv::LINE_AA);
    tx += tw + 18;

    // Separator dot
    cv::circle(img, cv::Point(tx, ty - 4), 2, col_label, -1, cv::LINE_AA);
    tx += 10;

    // Lanes
    snprintf(text, sizeof(text), "%d", nl);
    cv::putText(img, text, cv::Point(tx, ty), font, fs, col_value, 1, cv::LINE_AA);
    tw = cv::getTextSize(text, font, fs, 1, nullptr).width;
    cv::putText(img, "Lanes", cv::Point(tx + tw + 2, ty), font, fs * 0.85, col_label, 1, cv::LINE_AA);
    tx += tw + 40;

    // Separator dot
    cv::circle(img, cv::Point(tx, ty - 4), 2, col_label, -1, cv::LINE_AA);
    tx += 10;

    // Driver state with color coding
    cv::Scalar drv_color = col_value;
    if (driver_state == "DROWSY" || driver_state == "NO_DRIVER") drv_color = col_red;
    else if (driver_state == "DISTRACTED") drv_color = col_warn;
    else if (driver_state == "ALERT") drv_color = col_green;
    cv::putText(img, driver_state.c_str(), cv::Point(tx, ty), font, fs, drv_color, 1, cv::LINE_AA);

    // --- Row 2: GPS ---
    tx = ox + pad_x;
    ty += row_h;

    cv::Scalar gps_color = (gps_satellites > 0) ? col_green : col_red;
    // GPS status indicator dot
    cv::circle(img, cv::Point(tx + 3, ty - 4), 3, gps_color, -1, cv::LINE_AA);
    tx += 12;

    cv::putText(img, "GPS", cv::Point(tx, ty), font, fs * 0.85, col_label, 1, cv::LINE_AA);
    tx += 26;

    snprintf(text, sizeof(text), "%.5f, %.5f", latitude, longitude);
    cv::putText(img, text, cv::Point(tx, ty), font, fs, col_value, 1, cv::LINE_AA);
    tw = cv::getTextSize(text, font, fs, 1, nullptr).width;
    tx += tw + 10;

    snprintf(text, sizeof(text), "%.0fm", altitude);
    cv::putText(img, text, cv::Point(tx, ty), font, fs * 0.9, col_label, 1, cv::LINE_AA);

    // --- Row 3: IMU ---
    tx = ox + pad_x;
    ty += row_h;

    cv::putText(img, "IMU", cv::Point(tx, ty), font, fs * 0.85, col_label, 1, cv::LINE_AA);
    tx += 28;

    snprintf(text, sizeof(text), "%.1f  %.1f  %.1f", accel_x, accel_y, accel_z);
    cv::putText(img, text, cv::Point(tx, ty), font, fs, col_value, 1, cv::LINE_AA);
    tw = cv::getTextSize(text, font, fs, 1, nullptr).width;
    cv::putText(img, "m/s2", cv::Point(tx + tw + 4, ty), font, fs * 0.8, col_label, 1, cv::LINE_AA);
}

// Helper function to create composite display
cv::Mat createCompositeDisplay(const cv::Mat& main_fused, int screen_width, int screen_height)
{
    // Layout: Main 2/3 width full height, side panels 1/3 width × 1/3 height each
    int main_width = (screen_width * 2) / 3;
    int main_height = screen_height;
    int side_width = screen_width - main_width;
    int side_height = screen_height / 3;
    int side_height_bottom = screen_height - side_height * 2;  // absorb rounding remainder

    cv::Mat composite(screen_height, screen_width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Main view (left, full height)
    cv::Mat main_scaled;
    if (!main_fused.empty()) {
        cv::resize(main_fused, main_scaled, cv::Size(main_width, main_height));
        main_scaled.copyTo(composite(cv::Rect(0, 0, main_width, main_height)));
    }
    cv::putText(composite, "MAIN VIEW", cv::Point(10, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

    // Driver monitor (top-right)
    cv::Mat driver_display;
    if (!driver_monitor_image.empty()) {
        cv::resize(driver_monitor_image, driver_display, cv::Size(side_width, side_height));
    } else {
        driver_display = cv::Mat(side_height, side_width, CV_8UC3, cv::Scalar(40, 40, 40));
        cv::putText(driver_display, "Driver Monitor", cv::Point(side_width/2 - 80, side_height/2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 150, 150), 1);
        cv::putText(driver_display, "No Data", cv::Point(side_width/2 - 40, side_height/2 + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
    }
    driver_display.copyTo(composite(cv::Rect(main_width, 0, side_width, side_height)));
    cv::putText(composite, "DRIVER MONITOR", cv::Point(main_width + 10, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // Stereo depth (middle-right) - use pre-colorized image from callback
    cv::Mat depth_display;
    {
        cv::Mat depth_snap;
        {
            std::lock_guard<std::mutex> lock(depth_mutex);
            if (!stereo_depth_colored.empty()) {
                depth_snap = stereo_depth_colored;  // Shallow copy, ref-counted
            }
        }
        if (!depth_snap.empty()) {
            cv::resize(depth_snap, depth_display, cv::Size(side_width, side_height), 0, 0, cv::INTER_NEAREST);
        }
    }

    if (depth_display.empty()) {
        depth_display = cv::Mat(side_height, side_width, CV_8UC3, cv::Scalar(40, 40, 40));
        cv::putText(depth_display, "Stereo Depth", cv::Point(side_width/2 - 70, side_height/2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 150, 150), 1);
        cv::putText(depth_display, "No Data", cv::Point(side_width/2 - 40, side_height/2 + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
    } else {
        // Draw distance legend color bar (TURBO gradient with distance labels)
        int bar_w = 14;
        int bar_h = depth_display.rows - 50;
        int bar_y = 35;
        int pad = 4;
        int legend_w = pad + bar_w + 8 + 30 + pad;  // pad + bar + tick/gap + label + pad
        int bar_x = depth_display.cols - legend_w + pad;

        // Semi-transparent dark background behind legend area
        cv::Rect bg_rect(bar_x - pad, bar_y - pad, legend_w, bar_h + 2 * pad);
        bg_rect &= cv::Rect(0, 0, depth_display.cols, depth_display.rows);
        depth_display(bg_rect) *= 0.4;

        // Build 1-pixel wide TURBO gradient: top=255 (far/red), bottom=0 (near/blue)
        cv::Mat grad(bar_h, 1, CV_8UC1);
        for (int i = 0; i < bar_h; i++) {
            grad.at<uchar>(i, 0) = static_cast<uchar>(255 - 255 * i / (bar_h - 1));
        }
        cv::Mat grad_color;
        cv::applyColorMap(grad, grad_color, cv::COLORMAP_TURBO);
        cv::resize(grad_color, grad_color, cv::Size(bar_w, bar_h), 0, 0, cv::INTER_NEAREST);
        grad_color.copyTo(depth_display(cv::Rect(bar_x, bar_y, bar_w, bar_h)));

        // Border around color bar
        cv::rectangle(depth_display, cv::Point(bar_x - 1, bar_y - 1),
                      cv::Point(bar_x + bar_w, bar_y + bar_h), cv::Scalar(180, 180, 180), 1);

        // Distance tick labels
        const int num_ticks = 6;  // 0m, 10m, 20m, 30m, 40m, 50m
        char label[16];
        for (int i = 0; i < num_ticks; i++) {
            float depth_val = DEPTH_LEGEND_MAX_M * i / (num_ticks - 1);
            int y = bar_y + bar_h - 1 - (bar_h - 1) * i / (num_ticks - 1);
            // Tick mark
            cv::line(depth_display, cv::Point(bar_x + bar_w, y),
                     cv::Point(bar_x + bar_w + 3, y), cv::Scalar(180, 180, 180), 1);
            // Label
            snprintf(label, sizeof(label), "%dm", (int)depth_val);
            cv::putText(depth_display, label, cv::Point(bar_x + bar_w + 5, y + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(255, 255, 255), 1);
        }
    }
    depth_display.copyTo(composite(cv::Rect(main_width, side_height, side_width, side_height)));
    cv::putText(composite, "STEREO DEPTH", cv::Point(main_width + 10, side_height + 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // BEV panel (bottom-right) - uses side_height_bottom to fill remaining space
    cv::Mat bev_panel;
    {
        std::lock_guard<std::mutex> lock(bev_mutex);
        if (!bev_display.empty()) {
            cv::resize(bev_display, bev_panel, cv::Size(side_width, side_height_bottom));
        }
    }
    if (bev_panel.empty()) {
        bev_panel = cv::Mat(side_height_bottom, side_width, CV_8UC3, cv::Scalar(40, 42, 45));
        cv::putText(bev_panel, "Bird's Eye View", cv::Point(side_width/2 - 80, side_height_bottom/2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 150, 150), 1);
        cv::putText(bev_panel, "Waiting...", cv::Point(side_width/2 - 40, side_height_bottom/2 + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
    }
    bev_panel.copyTo(composite(cv::Rect(main_width, side_height * 2, side_width, side_height_bottom)));
    cv::putText(composite, "BIRD'S EYE VIEW", cv::Point(main_width + 10, side_height * 2 + 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    return composite;
}

void publishSceneData() {
    if (scene_pub == NULL || detect_msg == NULL || lanes_msg == NULL || adas_msg == NULL) {
        return;
    }
    
    visionconnect::msg::SceneData scene_data;
    
    // Calculate elapsed time in M:SS format
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - scene_start_time);
    int total_seconds = duration.count();
    int minutes = total_seconds / 60;
    int seconds = total_seconds % 60;
    
    std::ostringstream time_stream;
    time_stream << minutes << ":" << std::setfill('0') << std::setw(2) << seconds;
    scene_data.time = time_stream.str();
    
    // Determine ego transition based on ADAS lane change detection
    if (adas_msg->lane_change_right) {
        scene_data.ego_trans = "V33"; // Right lane change
    } else if (adas_msg->lane_change_left) {
        scene_data.ego_trans = "V31"; // Left lane change
    } else {
        scene_data.ego_trans = "None"; // No lane change detected
    }
    
    // Update current ego lane based on lane change direction for tracking
    if (adas_msg->lane_change_right) {
        current_ego_lane = "V23"; // Right lane
        previous_ego_lane = current_ego_lane;
    } else if (adas_msg->lane_change_left) {
        current_ego_lane = "V21"; // Left lane
        previous_ego_lane = current_ego_lane;
    } else {
        // If no lane change, determine current lane based on deviation
        float deviation_percent = std::abs(adas_msg->lane_center_offset * 100.0f);
        if (deviation_percent > 30.0f) {
            if (adas_msg->lane_center_offset > 0) {
                current_ego_lane = "V23"; // Right lane
            } else {
                current_ego_lane = "V21"; // Left lane
            }
        } else {
            current_ego_lane = "V22"; // Center lane
        }
        previous_ego_lane = current_ego_lane;
    }
    
    // Set lane region status based on actual lane region availability computed in fuse_data
    scene_data.lane_1 = lane_region_1_available;
    scene_data.lane_2 = lane_region_2_available;
    scene_data.lane_3 = lane_region_3_available;
    
    // Use quadrant assignments computed in fuse_data()
    scene_data.v11 = quadrant_v11;
    scene_data.v12 = quadrant_v12;
    scene_data.v13 = quadrant_v13;
    scene_data.v21 = quadrant_v21;
    scene_data.v22 = quadrant_v22;
    scene_data.v23 = quadrant_v23;
    
    // Pedestrian detection logic
    std::vector<std::string> pedestrian_positions;

    if (detect_msg != NULL) {
        int img_width = detect_msg->image.width * 3;  // Get actual image width from detect msg
        float camera_center_x = img_width / 2.0f;
        
        for (int i = 0; i < detect_msg->num_detections; i++) {
            if (detect_msg->classes[i] == 0) { // Pedestrian class
                int x = static_cast<int>(detect_msg->boxes[i].data[0]);
                int w = static_cast<int>(detect_msg->boxes[i].data[2]);
                int center_x = x + w/2;
                
                if (center_x < camera_center_x * 0.4f) {
                    pedestrian_positions.push_back("Left");
                } else if (center_x > camera_center_x * 1.6f) {
                    pedestrian_positions.push_back("Right");
                } else {
                    pedestrian_positions.push_back("Center");
                }
            }
        }
    }
    
    // Set pedestrian status
    if (pedestrian_positions.empty()) {
        scene_data.pedest = "None";
    } else {
        bool has_left = false, has_right = false;
        for (const auto& pos : pedestrian_positions) {
            if (pos == "Left") has_left = true;
            else if (pos == "Right") has_right = true;
        }
        
        if (has_left && has_right) {
            scene_data.pedest = "Both";
        } else if (has_left) {
            scene_data.pedest = "Left";
        } else if (has_right) {
            scene_data.pedest = "Right";
        } else {
            scene_data.pedest = "Center";
        }
    }
    
    // Set fixed values as requested
    scene_data.speed = 45;
    scene_data.warning = "None";
    
    // Publish the scene data
    scene_pub->publish(scene_data);
}

// Filled / outlined rounded rectangle. OpenCV doesn't ship one; we build it
// from a cross of axis-aligned rects + 4 corner pieces (circles for fill,
// 90° arcs for outline). Used by the lane-departure HUD banner.
static void drawRoundedRect(cv::Mat& img, cv::Rect r, int radius,
                            cv::Scalar color, int thickness)
{
    radius = std::max(0, std::min(radius, std::min(r.width, r.height) / 2));
    if (thickness < 0) {
        cv::rectangle(img, cv::Rect(r.x + radius, r.y, r.width - 2*radius, r.height),
                      color, -1, cv::LINE_AA);
        cv::rectangle(img, cv::Rect(r.x, r.y + radius, r.width, r.height - 2*radius),
                      color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(r.x + radius,             r.y + radius),             radius, color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(r.x + r.width - radius-1, r.y + radius),             radius, color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(r.x + radius,             r.y + r.height - radius-1), radius, color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(r.x + r.width - radius-1, r.y + r.height - radius-1), radius, color, -1, cv::LINE_AA);
    } else {
        cv::line(img, cv::Point(r.x + radius,             r.y),                 cv::Point(r.x + r.width - radius-1, r.y),                 color, thickness, cv::LINE_AA);
        cv::line(img, cv::Point(r.x + radius,             r.y + r.height-1),    cv::Point(r.x + r.width - radius-1, r.y + r.height-1),    color, thickness, cv::LINE_AA);
        cv::line(img, cv::Point(r.x,                      r.y + radius),        cv::Point(r.x,                      r.y + r.height - radius-1), color, thickness, cv::LINE_AA);
        cv::line(img, cv::Point(r.x + r.width-1,          r.y + radius),        cv::Point(r.x + r.width-1,          r.y + r.height - radius-1), color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(r.x + radius,             r.y + radius),             cv::Size(radius, radius), 180, 0, 90, color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(r.x + r.width - radius-1, r.y + radius),             cv::Size(radius, radius), 270, 0, 90, color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(r.x + radius,             r.y + r.height - radius-1), cv::Size(radius, radius),  90, 0, 90, color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(r.x + r.width - radius-1, r.y + r.height - radius-1), cv::Size(radius, radius),   0, 0, 90, color, thickness, cv::LINE_AA);
    }
}

// HUD-style lane-departure badge. direction: -1=left (rendered top-left),
// +1=right (rendered top-right). Compact rounded pill with a chevron on the
// departure side and "LANE DEPARTURE" next to it.
static void drawLaneDepartureBanner(cv::Mat& image, int direction)
{
    if (direction == 0) return;

    const int W = image.cols;
    const int H = image.rows;

    // Compact badge — scales with frame, clamped tight. All sizing constants
    // halved from the original to shrink the badge to ~50% of its previous
    // footprint (linear; area is ~¼). Auto-fit loop below still handles the
    // smaller text zone gracefully.
    const int bw = std::clamp(static_cast<int>(W * 0.085f), 90, 150);
    const int bh = std::clamp(static_cast<int>(H * 0.025f), 18, 27);
    const int margin = std::max(5, static_cast<int>(H * 0.018f));
    const int bx = (direction < 0) ? margin : (W - bw - margin);
    const int by = margin;
    const cv::Rect rect(bx, by, bw, bh);
    const int radius = bh / 3;

    // Vivid crimson — distinct from the amber used elsewhere and reads as
    // "warning" at a glance. BGR for ~#F03250.
    const cv::Scalar warn(80, 50, 240);
    const cv::Scalar warn_dim(60, 35, 175);
    const cv::Scalar white(245, 245, 245);
    const cv::Scalar bg_dark(20, 20, 25);

    // Translucent dark backdrop — blend only the badge ROI.
    const cv::Rect roi = rect & cv::Rect(0, 0, W, H);
    if (roi.width > 0 && roi.height > 0) {
        cv::Mat band = image(roi).clone();
        cv::Mat filled = band.clone();
        drawRoundedRect(filled, cv::Rect(rect.x - roi.x, rect.y - roi.y, rect.width, rect.height),
                        radius, bg_dark, -1);
        cv::addWeighted(filled, 0.78, band, 0.22, 0, image(roi));
    }

    // Crimson border + thin inner highlight for depth.
    drawRoundedRect(image, rect, radius, warn, 2);
    drawRoundedRect(image, cv::Rect(rect.x + 3, rect.y + 3, rect.width - 6, rect.height - 6),
                    std::max(1, radius - 2), warn_dim, 1);

    // Chevron on the departure side (matches the badge's screen-edge anchor).
    const int arrow_w = bh / 2;
    const int arrow_h = static_cast<int>(bh * 0.55f);
    const int arrow_cy = by + bh / 2;
    const int side_pad = bh / 3;
    const int arrow_cx = (direction < 0)
        ? bx + side_pad + arrow_w / 2
        : bx + bw - side_pad - arrow_w / 2;
    std::vector<cv::Point> chev;
    if (direction < 0) {
        chev = {
            cv::Point(arrow_cx - arrow_w / 2, arrow_cy),
            cv::Point(arrow_cx + arrow_w / 2, arrow_cy - arrow_h / 2),
            cv::Point(arrow_cx + arrow_w / 2, arrow_cy + arrow_h / 2),
        };
    } else {
        chev = {
            cv::Point(arrow_cx + arrow_w / 2, arrow_cy),
            cv::Point(arrow_cx - arrow_w / 2, arrow_cy - arrow_h / 2),
            cv::Point(arrow_cx - arrow_w / 2, arrow_cy + arrow_h / 2),
        };
    }
    cv::fillConvexPoly(image, chev, warn, cv::LINE_AA);

    // "LANE DEPARTURE" — centered in the space not covered by the arrow.
    const std::string text = "LANE DEPARTURE";
    int thickness = 1;
    double scale = bh * 0.012;
    int baseline = 0;
    cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, scale, thickness, &baseline);
    const int text_zone_x = (direction < 0) ? bx + side_pad * 2 + arrow_w : bx + side_pad;
    const int text_zone_w = bw - (side_pad * 3 + arrow_w);
    // Lower floor + finer step now that bh can drop to ~18 px after the 50%
    // shrink. At the previous 0.35 floor the loop never executed for small
    // badges (initial scale was already below 0.35) and "LANE DEPARTURE"
    // would overflow the pill horizontally.
    while (ts.width > text_zone_w && scale > 0.18) {
        scale -= 0.03;
        ts = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, scale, thickness, &baseline);
    }
    const int tx = text_zone_x + (text_zone_w - ts.width) / 2;
    const int ty = by + (bh + ts.height) / 2 - 1;
    cv::putText(image, text, cv::Point(tx, ty),
                cv::FONT_HERSHEY_DUPLEX, scale, white, thickness, cv::LINE_AA);
}

void drawLaneChangeIndicator(cv::Mat &image)
{
    if (adas_msg == NULL) {
        return;
    }

    int img_width = image.cols;
    int img_height = image.rows;

    // Top-center HUD banner — replaces the old fixed-pos large yellow text.
    if (adas_msg->lane_change_left) {
        drawLaneDepartureBanner(image, -1);
    } else if (adas_msg->lane_change_right) {
        drawLaneDepartureBanner(image, +1);
    }

    // Use calibrated points directly from ADAS message (no recalculation)
    int bottom_y = img_height - 10;  // Position closer to actual bottom

    // Convert normalized coordinates (0.0-1.0) to GUI display coordinates
    int calibrated_center_x = static_cast<int>(adas_msg->calibrated_camera_center_x * img_width);
    int lane_midpoint_x = static_cast<int>(adas_msg->current_lane_midpoint_x * img_width);

    // Draw calibrated camera center (green vertical line) - aligned to bottom
    cv::line(image, cv::Point(calibrated_center_x, bottom_y - 30),
             cv::Point(calibrated_center_x, bottom_y), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    // Draw current lane midpoint (blue vertical line) - aligned to bottom
    cv::line(image, cv::Point(lane_midpoint_x, bottom_y - 30),
             cv::Point(lane_midpoint_x, bottom_y), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

    // Draw deviation indicator line between the two points
    cv::line(image, cv::Point(calibrated_center_x, bottom_y - 15),
             cv::Point(lane_midpoint_x, bottom_y - 15), cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

    // Show deviation value positioned above the deviation indicator
    int deviation_percent = static_cast<int>(std::round(adas_msg->lane_center_offset * 100.0f));
    std::string deviation_text = std::to_string(deviation_percent) + "%";

    // Position label at the center of the deviation indicator
    int label_x = (calibrated_center_x + lane_midpoint_x) / 2;
    int label_y = bottom_y - 40;

    // Get text size to center it properly
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(deviation_text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
    label_x -= text_size.width / 2;  // Center the text horizontally

    cv::putText(image, deviation_text, cv::Point(label_x, label_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

// Extract median depth from the central region of a bounding box
// Box coords are in full resolution (1200x1200). Returns depth in meters, or -1.0f on failure.
float extract_box_depth(int bx, int by, int bw, int bh, const cv::Mat& depth_map)
{
    if (depth_map.empty() || depth_map.type() != CV_32FC1)
        return -1.0f;

    // Central 50% of the box (robust to edge noise)
    int cx = bx + bw / 4;
    int cy = by + bh / 4;
    int cw = bw / 2;
    int ch = bh / 2;

    // Clamp to depth map bounds
    cx = std::max(0, cx);
    cy = std::max(0, cy);
    if (cx + cw > depth_map.cols) cw = depth_map.cols - cx;
    if (cy + ch > depth_map.rows) ch = depth_map.rows - cy;
    if (cw <= 0 || ch <= 0)
        return -1.0f;

    // Collect valid depth samples
    std::vector<float> samples;
    samples.reserve(cw * ch);
    for (int row = cy; row < cy + ch; row++) {
        const float* ptr = depth_map.ptr<float>(row);
        for (int col = cx; col < cx + cw; col++) {
            float d = ptr[col];
            if (std::isfinite(d) && d > 0.1f && d < 100.0f)
                samples.push_back(d);
        }
    }

    if (samples.size() < 5)
        return -1.0f;

    // Median via nth_element (O(n))
    size_t mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    return samples[mid];
}

// Estimate oriented 3D bounding box via frustum back-projection + PCA
// Box coords (bx,by,bw,bh) are in full resolution (1200x1200), depth_map is 32FC1 meters
OrientedBox3D estimate_frustum_3d_box(int bx, int by, int bw, int bh,
                                       const cv::Mat& depth_map, int cls)
{
    OrientedBox3D result{};
    // Only build a 3D box for physical road objects (skip traffic light/sign placeholders).
    const bool physical = (cls >= 0 && cls <= 5) || cls == 8;
    if (depth_map.empty() || depth_map.type() != CV_32FC1 || !physical || cls >= NUM_SIZE_PRIORS)
        return result;

    const SizePrior& prior = SIZE_PRIORS[cls];

    // Clamp box to image bounds
    int x0 = std::max(0, bx);
    int y0 = std::max(0, by);
    int x1 = std::min(depth_map.cols, bx + bw);
    int y1 = std::min(depth_map.rows, by + bh);
    if (x1 <= x0 || y1 <= y0) return result;

    // Back-project pixels to 3D (subsample every 2nd pixel for speed)
    std::vector<Eigen::Vector3f> points;
    points.reserve(((x1 - x0) / 2 + 1) * ((y1 - y0) / 2 + 1));

    for (int v = y0; v < y1; v += 2) {
        const float* ptr = depth_map.ptr<float>(v);
        for (int u = x0; u < x1; u += 2) {
            float d = ptr[u];
            if (!std::isfinite(d) || d <= 0.1f || d > 100.0f) continue;
            float X = (u - CAM_CX) * d / CAM_FX;
            float Y = (v - CAM_CY) * d / CAM_FY;
            points.emplace_back(X, Y, d);
        }
    }

    if (points.size() < 20) return result;

    // Compute median Z for foreground segmentation
    std::vector<float> z_vals;
    z_vals.reserve(points.size());
    for (const auto& p : points) z_vals.push_back(p.z());
    size_t mid = z_vals.size() / 2;
    std::nth_element(z_vals.begin(), z_vals.begin() + mid, z_vals.end());
    float median_z = z_vals[mid];

    // Segment foreground: keep points within ±tolerance of median
    float tol = std::clamp(prior.length * 0.5f, 0.3f, 3.0f);
    std::vector<Eigen::Vector3f> fg_pts;
    fg_pts.reserve(points.size());
    for (const auto& p : points) {
        if (std::abs(p.z() - median_z) <= tol)
            fg_pts.push_back(p);
    }

    if (fg_pts.size() < 10) return result;

    // Compute centroid
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : fg_pts) centroid += p;
    centroid /= static_cast<float>(fg_pts.size());

    // PCA on XZ plane for yaw estimation
    float yaw = 0.0f;
    if (fg_pts.size() >= 50) {
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto& p : fg_pts) {
            float dx = p.x() - centroid.x();
            float dz = p.z() - centroid.z();
            cov(0, 0) += dx * dx;
            cov(0, 1) += dx * dz;
            cov(1, 0) += dx * dz;
            cov(1, 1) += dz * dz;
        }
        cov /= static_cast<float>(fg_pts.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
        Eigen::Vector2f evals = solver.eigenvalues();
        Eigen::Matrix2f evecs = solver.eigenvectors();

        // Principal eigenvector (largest eigenvalue is last for SelfAdjoint)
        Eigen::Vector2f principal = evecs.col(1); // (dx, dz)

        // Only use PCA yaw if there's clear elongation
        if (evals(1) > 1.5f * evals(0)) {
            yaw = std::atan2(principal.x(), principal.y()); // atan2(X, Z)
        }
    }

    // Rotate foreground points into oriented frame to measure extents
    float cos_y = std::cos(-yaw), sin_y = std::sin(-yaw);
    std::vector<float> lx_vals, ly_vals, lz_vals;
    lx_vals.reserve(fg_pts.size());
    ly_vals.reserve(fg_pts.size());
    lz_vals.reserve(fg_pts.size());

    for (const auto& p : fg_pts) {
        float dx = p.x() - centroid.x();
        float dy = p.y() - centroid.y();
        float dz = p.z() - centroid.z();
        // Rotate around Y axis
        lx_vals.push_back(cos_y * dx - sin_y * dz);
        ly_vals.push_back(dy);
        lz_vals.push_back(sin_y * dx + cos_y * dz);
    }

    // Use 5th/95th percentile extents to reject outliers
    auto percentile = [](std::vector<float>& v, float pct) -> float {
        size_t idx = static_cast<size_t>(pct * (v.size() - 1));
        idx = std::min(idx, v.size() - 1);
        std::nth_element(v.begin(), v.begin() + idx, v.end());
        return v[idx];
    };

    float lx_lo = percentile(lx_vals, 0.05f), lx_hi = percentile(lx_vals, 0.95f);
    float ly_lo = percentile(ly_vals, 0.05f), ly_hi = percentile(ly_vals, 0.95f);
    float lz_lo = percentile(lz_vals, 0.05f), lz_hi = percentile(lz_vals, 0.95f);

    float meas_length = lz_hi - lz_lo;
    float meas_width  = lx_hi - lx_lo;
    float meas_height = ly_hi - ly_lo;

    // Clamp dimensions to [0.5x, 1.2x] of class prior, or use pure priors if too few points
    if (fg_pts.size() >= 50) {
        result.length = std::clamp(meas_length, prior.length * 0.5f, prior.length * 1.2f);
        result.width  = std::clamp(meas_width,  prior.width  * 0.5f, prior.width  * 1.2f);
        result.height = std::clamp(meas_height, prior.height * 0.5f, prior.height * 1.2f);
    } else {
        result.length = prior.length;
        result.width  = prior.width;
        result.height = prior.height;
    }

    // Center position: use centroid XZ, fix Y so box top aligns with observed 5th-percentile Y
    result.cx = centroid.x();
    result.cz = centroid.z();
    result.cy = ly_lo + centroid.y() + result.height * 0.5f;
    result.yaw = yaw;
    result.valid = true;
    return result;
}

// Draw oriented 3D wireframe bounding box projected onto display image
void draw_oriented_3d_box(cv::Mat& img, const OrientedBox3D& box, cv::Scalar color, int thickness)
{
    float hw = box.width  * 0.5f;
    float hh = box.height * 0.5f;
    float hl = box.length * 0.5f;

    // 8 corners in local oriented frame (X=right, Y=down, Z=forward)
    float local[8][3] = {
        {-hw, -hh, -hl}, { hw, -hh, -hl}, { hw,  hh, -hl}, {-hw,  hh, -hl}, // back face
        {-hw, -hh,  hl}, { hw, -hh,  hl}, { hw,  hh,  hl}, {-hw,  hh,  hl}, // front face
    };

    // Rotate by yaw around Y-axis and translate to world
    float cos_y = std::cos(box.yaw), sin_y = std::sin(box.yaw);
    cv::Point2f proj[8];
    bool behind = false;

    for (int i = 0; i < 8; i++) {
        float rx = cos_y * local[i][0] + sin_y * local[i][2];
        float ry = local[i][1];
        float rz = -sin_y * local[i][0] + cos_y * local[i][2];

        float X = rx + box.cx;
        float Y = ry + box.cy;
        float Z = rz + box.cz;

        if (Z <= 0.0f) { behind = true; break; }

        // Project to full-resolution pixel coords, then divide by 3 for display
        float u = (CAM_FX * X / Z + CAM_CX) / 3.0f;
        float v = (CAM_FY * Y / Z + CAM_CY) / 3.0f;
        proj[i] = cv::Point2f(u, v);
    }

    if (behind) return;

    // Determine which face is closer to camera for depth-based line thickness
    float front_z = 0, back_z = 0;
    for (int i = 0; i < 4; i++) back_z += local[i][2];
    for (int i = 4; i < 8; i++) front_z += local[i][2];
    // After rotation, recompute
    float face_z[8];
    for (int i = 0; i < 8; i++) {
        face_z[i] = (-sin_y * local[i][0] + cos_y * local[i][2]) + box.cz;
    }

    int thin = std::max(1, thickness / 2);

    // 12 edges: 4 bottom, 4 top, 4 vertical pillars
    // Back face (indices 0-3)
    auto edge = [&](int a, int b, bool is_back) {
        int t = is_back ? thin : thickness;
        cv::line(img, proj[a], proj[b], color, t, cv::LINE_AA);
    };

    // Determine which quad is "back" (farther average Z)
    float avg_back = (face_z[0] + face_z[1] + face_z[2] + face_z[3]) / 4.0f;
    float avg_front = (face_z[4] + face_z[5] + face_z[6] + face_z[7]) / 4.0f;
    bool face0_is_back = (avg_back > avg_front);

    // Back face edges (draw first, behind)
    edge(0, 1, face0_is_back);  edge(1, 2, face0_is_back);
    edge(2, 3, face0_is_back);  edge(3, 0, face0_is_back);

    // Connecting pillars
    edge(0, 4, true); edge(1, 5, true);
    edge(2, 6, true); edge(3, 7, true);

    // Front face edges (draw last, on top)
    edge(4, 5, !face0_is_back); edge(5, 6, !face0_is_back);
    edge(6, 7, !face0_is_back); edge(7, 4, !face0_is_back);
}

void fuse_data()
{
    std::vector<cv::Scalar> colors = {
        cv::Scalar(10, 230, 240), //pedestrian
        cv::Scalar(230, 240, 10), //cyclist
        cv::Scalar(190, 150, 25), //car
        cv::Scalar(240, 120, 10), //bus
        cv::Scalar(100, 10, 240), //truck
        cv::Scalar(240, 130, 10), //train
        cv::Scalar(160, 90, 160), //traffic light
        cv::Scalar(120, 55, 70),  //traffic sign
        cv::Scalar(180, 240, 90)  //bike
    };
    cv::Mat img;

    // Only detect_msg is required (it contains the image)
    if (detect_msg == NULL)
        return;

    // Make local copies of shared pointers to prevent race conditions
    // These could be modified by callbacks while we're processing
    auto local_detect_msg = detect_msg;
    auto local_lanes_msg = lanes_msg;
    auto local_signs_msg = signs_msg;
    auto local_adas_msg = adas_msg;
    cv::Mat local_driver_image = driver_monitor_image.clone();

    // Snapshot raw depth for distance labels and 3D bounding boxes
    cv::Mat local_depth;
    try {
        std::lock_guard<std::mutex> lock(raw_depth_mutex);
        if (!stereo_depth_raw.empty())
            local_depth = stereo_depth_raw.clone();
    } catch (...) {
        // Depth snapshot failed — proceed without depth
    }

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(local_detect_msg->image);
    convert_message_to_frame(img_msg, img);

    if (img.empty()) {
        ROS_WARN("Empty image received in fuse_data");
        return;
    }

    // Convert RGB to BGR for OpenCV display if needed
    if (img_msg->encoding == "rgb8" && img.channels() == 3) {
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    }

    // Draw lanes if available (using local copy)
    if (local_lanes_msg != NULL && !local_lanes_msg->xs.empty()) {
        // Copy lane data locally to avoid race conditions
        auto local_xs = local_lanes_msg->xs;
        auto local_ys = local_lanes_msg->ys;

        std::vector<cv::Scalar> lane_colors = {
            cv::Scalar(255, 0, 0),
            cv::Scalar(0, 255, 0),
            cv::Scalar(0, 0, 255),
            cv::Scalar(255, 255, 0)
        };

        std::vector<cv::Point> current_lane;
        size_t lane_index = 0;
        for (size_t j = 0; j < local_xs.size() && j < local_ys.size(); ++j) {
            int x = local_xs[j];
            int y = local_ys[j];

            if (x == -1 && y == -1) {
                if (!current_lane.empty()) {
                    cv::polylines(img, current_lane, false, lane_colors[lane_index % lane_colors.size()], 3);
                    current_lane.clear();
                    lane_index++;
                }
            } else {
                current_lane.push_back(cv::Point(x/3, y/3));
            }
        }
        // Draw circles
        for (size_t j = 0; j < local_xs.size() && j < local_ys.size(); ++j) {
            int x = local_xs[j];
            int y = local_ys[j];
            if (x != -1 && y != -1) {
                cv::circle(img, cv::Point(x/3, y/3), 2, cv::Scalar(50,50,50), -1);
            }
        }
    }

    auto boxes = local_detect_msg->boxes;
    auto classes = local_detect_msg->classes;
    auto scores = local_detect_msg->scores;
    auto num_detections = local_detect_msg->num_detections;
    auto track_list = local_detect_msg->track_list;

    // Signs are optional
    std::vector<std::string> signLabels;
    std::vector<float> signScores;
    if (local_signs_msg != NULL) {
        signLabels = local_signs_msg->labels;
        signScores = local_signs_msg->scores;
    }

    // --- Drawing style constants ---
    const int FONT = cv::FONT_HERSHEY_DUPLEX;
    const int LINE_TYPE = cv::LINE_AA;
    const int PAD_X = 3;
    const int PAD_Y = 2;

    // Dynamic font scale: maps depth → font size (closer objects get larger labels)
    // Returns scale in [0.28 .. 0.50] range, default 0.38 when no depth
    auto depth_font_scale = [](float depth) -> double {
        if (depth <= 0.0f) return 0.38;
        if (depth < 3.0f) return 0.50;
        if (depth > 40.0f) return 0.28;
        // Linear interpolation: 3m→0.50, 40m→0.28
        return 0.50 - (depth - 3.0f) * (0.22 / 37.0f);
    };

    auto fmt_score = [](float s) -> std::string {
        int pct = static_cast<int>(s * 100.0f + 0.5f);
        return std::to_string(pct) + "%";
    };

    // Draw label above box: translucent dark pill with colored text, no border
    auto draw_label = [&](const std::string& text, int lx, int ly,
                          cv::Scalar color, double scale) {
        int thick = 1;
        int bl = 0;
        cv::Size ts = cv::getTextSize(text, FONT, scale, thick, &bl);
        cv::Point tl(lx, ly - ts.height - PAD_Y * 2);
        cv::Point br(lx + ts.width + PAD_X * 2, ly);

        // Semi-transparent dark background
        cv::Mat roi = img(cv::Rect(
            std::max(0, tl.x), std::max(0, tl.y),
            std::min(br.x, img.cols) - std::max(0, tl.x),
            std::min(br.y, img.rows) - std::max(0, tl.y)));
        roi *= 0.4;

        // Colored text on dark background
        cv::putText(img, text, cv::Point(lx + PAD_X, ly - PAD_Y),
                    FONT, scale, color, thick, LINE_TYPE);
    };

    // Draw distance text centered below the box
    auto draw_distance = [&](float depth, int bx, int by, int bw, int bh,
                             cv::Scalar color, double scale) {
        if (depth <= 0.0f) return;
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1fm", depth);
        int thick = 1;
        int bl = 0;
        cv::Size ts = cv::getTextSize(buf, FONT, scale * 0.85, thick, &bl);
        int tx = bx + (bw - ts.width) / 2;
        int ty = by + bh + ts.height + 4;
        if (ty > img.rows - 2) return;

        // Subtle shadow for readability
        cv::putText(img, buf, cv::Point(tx + 1, ty + 1),
                    FONT, scale * 0.85, cv::Scalar(0, 0, 0), thick + 1, LINE_TYPE);
        cv::putText(img, buf, cv::Point(tx, ty),
                    FONT, scale * 0.85, color, thick, LINE_TYPE);
    };

    // 2D box: thin rectangle with bold corner accents
    auto draw_box_2d = [&](int bx, int by, int bw, int bh, cv::Scalar color) {
        // Thin full rectangle
        cv::rectangle(img, cv::Rect(bx, by, bw, bh), color, 1, LINE_TYPE);
        // Bold corner accents
        int cl = std::max(4, std::min(12, std::min(bw, bh) / 4));
        int ct = 2;
        cv::line(img, {bx, by}, {bx + cl, by}, color, ct, LINE_TYPE);
        cv::line(img, {bx, by}, {bx, by + cl}, color, ct, LINE_TYPE);
        cv::line(img, {bx + bw, by}, {bx + bw - cl, by}, color, ct, LINE_TYPE);
        cv::line(img, {bx + bw, by}, {bx + bw, by + cl}, color, ct, LINE_TYPE);
        cv::line(img, {bx, by + bh}, {bx + cl, by + bh}, color, ct, LINE_TYPE);
        cv::line(img, {bx, by + bh}, {bx, by + bh - cl}, color, ct, LINE_TYPE);
        cv::line(img, {bx + bw, by + bh}, {bx + bw - cl, by + bh}, color, ct, LINE_TYPE);
        cv::line(img, {bx + bw, by + bh}, {bx + bw, by + bh - cl}, color, ct, LINE_TYPE);
    };

    size_t j = 0;
    cv::String label = "";
    int baseline = 0;

    for (int i = 0; i < num_detections; i++)
    {
        int x = static_cast<int>(boxes[i].data[0]/3);
        int y = static_cast<int>(boxes[i].data[1]/3);
        int w = static_cast<int>(boxes[i].data[2]/3);
        int h = static_cast<int>(boxes[i].data[3]/3);
        int cls = static_cast<int>(classes[i]);

        // --- Physical objects (classes 0-5, 8=bike): single pass 2D/3D ---
        if ((cls >= 0 && cls <= 5) || cls == 8)
        {
            // Index aligns with legacy class slots (0..5 + 8); slots 6/7 are
            // placeholders so direct indexing stays valid for bike at 8.
            static const char* const short_names[] = {
                "pedestrian", "cyclist", "car", "bus", "truck", "train",
                "tlight", "tsign", "bike"
            };
            if (!track_list[i].empty()) {
                label = track_list[i] + " " + fmt_score(scores[i]);
            } else {
                label = std::string(short_names[cls]) + " " + fmt_score(scores[i]);
            }

            cv::Scalar box_color = colors[cls];
            if ((cls == 0 || cls == 1) && scores[i] > 0.7)
                box_color = cv::Scalar(0, 0, 255);

            // Depth + optional 3D box estimation
            float depth = -1.0f;
            OrientedBox3D obox{};
            int full_x = static_cast<int>(boxes[i].data[0]);
            int full_y = static_cast<int>(boxes[i].data[1]);
            int full_w = static_cast<int>(boxes[i].data[2]);
            int full_h = static_cast<int>(boxes[i].data[3]);
            if (!local_depth.empty()) {
                if (enable_3d_boxes) {
                    obox = estimate_frustum_3d_box(full_x, full_y, full_w, full_h, local_depth, cls);
                    if (obox.valid) depth = obox.cz;
                }
                if (depth <= 0.0f)
                    depth = extract_box_depth(full_x, full_y, full_w, full_h, local_depth);
            }

            double fscale = depth_font_scale(depth);

            if (obox.valid) {
                draw_oriented_3d_box(img, obox, box_color, 2);
            } else {
                draw_box_2d(x, y, w, h, box_color);
            }
            draw_label(label, x, y, box_color, fscale);
            draw_distance(depth, x, y, w, h, box_color, fscale);
        }

        else if (cls == 6)  // traffic light
        {
            // Always draw the detector's box. If the classifier has a result
            // for this slot AND it identifies one of the known colors with
            // enough confidence, override the box color + label; otherwise
            // fall back to the generic "tlight" label and detector score.
            // Crucially `j` always advances when a classifier slot exists,
            // so the index stays in sync with downstream tlight/tsign entries.
            cv::Scalar box_color = colors[6];
            label = std::string("tlight ") + fmt_score(scores[i]);

            if (j < signLabels.size() && j < signScores.size()) {
                const std::string& clab = signLabels[j];
                const float cscore = signScores[j];
                if (cscore > 0.5f) {
                    if (clab == "red light") {
                        box_color = cv::Scalar(0, 0, 255);
                        label = clab + " " + fmt_score(cscore);
                    } else if (clab == "green light") {
                        box_color = cv::Scalar(55, 235, 100);
                        label = clab + " " + fmt_score(cscore);
                    } else if (clab == "yellow light") {
                        box_color = cv::Scalar(0, 200, 255);
                        label = clab + " " + fmt_score(cscore);
                    }
                }
                j++;
            }
            draw_box_2d(x, y, w, h, box_color);
            draw_label(label, x, y, box_color, 0.38);
        }

        else if (cls == 7)  // traffic sign
        {
            // Mirror of cls==6: always draw the detector's box, overlay the
            // classifier label when it's available, confident, and not the
            // uninformative "guide sign" string.
            cv::Scalar box_color = colors[7];
            label = std::string("tsign ") + fmt_score(scores[i]);

            if (j < signLabels.size() && j < signScores.size()) {
                const std::string& clab = signLabels[j];
                const float cscore = signScores[j];
                if (cscore > 0.65f && clab != "guide sign") {
                    label = clab + " " + fmt_score(cscore);
                }
                j++;
            }
            draw_box_2d(x, y, w, h, box_color);
            draw_label(label, x, y, box_color, 0.38);
        }
    }

    // Draw road trapezoid based on lane polylines with extrapolation
    if (local_lanes_msg != NULL && !local_lanes_msg->xs.empty()) {
        // Step 1: Extract individual lane polylines (use local copy already made)
        auto local_xs = local_lanes_msg->xs;
        auto local_ys = local_lanes_msg->ys;
        std::vector<std::vector<cv::Point>> lane_polylines;
        std::vector<cv::Point> current_lane;

        for (size_t j = 0; j < local_xs.size() && j < local_ys.size(); ++j) {
            int x = local_xs[j];
            int y = local_ys[j];

            if (x == -1 && y == -1) {
                // Lane delimiter - save current polyline
                if (!current_lane.empty()) {
                    lane_polylines.push_back(current_lane);
                    current_lane.clear();
                }
            } else {
                current_lane.push_back(cv::Point(x/3, y/3));
            }
        }
        // Add last polyline if exists
        if (!current_lane.empty()) {
            lane_polylines.push_back(current_lane);
        }
        
        if (local_adas_msg != NULL && lane_polylines.size() >= 1) {
            int img_height = img.rows;
            int img_width = img.cols;

            // Use extrapolated points from ADAS (convert from normalized coordinates)
            std::vector<cv::Point> lane_top_points_by_color(4);
            std::vector<cv::Point> lane_bottom_points_by_color(4);
            std::vector<bool> valid_lane_by_color(4, false);

            // Convert normalized coordinates from ADAS to GUI display coordinates
            for (int i = 0; i < 4; i++) {
                if (local_adas_msg->lane_valid[i]) {
                    lane_top_points_by_color[i] = cv::Point(
                        static_cast<int>(local_adas_msg->lane_top_x[i] * img_width),
                        static_cast<int>(local_adas_msg->lane_top_y[i] * img_height)
                    );
                    lane_bottom_points_by_color[i] = cv::Point(
                        static_cast<int>(local_adas_msg->lane_bottom_x[i] * img_width),
                        static_cast<int>(local_adas_msg->lane_bottom_y[i] * img_height)
                    );
                    valid_lane_by_color[i] = true;
                }
            }
            
            // Draw extrapolation lines with original colors using ADAS-provided points
            std::vector<cv::Scalar> debug_colors = {
                cv::Scalar(255, 0, 0),    // Blue
                cv::Scalar(0, 255, 0),    // Green  
                cv::Scalar(0, 0, 255),    // Red
                cv::Scalar(255, 255, 0)   // Cyan
            };
            
            for (int lane_idx = 0; lane_idx < 4; lane_idx++) {
                if (valid_lane_by_color[lane_idx]) {
                    cv::line(img, lane_top_points_by_color[lane_idx], lane_bottom_points_by_color[lane_idx], debug_colors[lane_idx], 2);
                }
            }
            
            // Draw filled lane areas with proper corner clipping and higher visibility
            
            // Simple polygon creation with corner handling
            auto createClippedLaneArea = [&](cv::Point top_left, cv::Point top_right, 
                                           cv::Point bottom_right, cv::Point bottom_left) -> std::vector<cv::Point> {
                std::vector<cv::Point> polygon;
                
                // Start with top points
                polygon.push_back(top_left);
                polygon.push_back(top_right);
                
                // Handle right side crossing
                if (bottom_right.x >= img.cols) {
                    // Right line crosses right edge - add crossing point and corner
                    polygon.push_back(cv::Point(img.cols-1, bottom_right.y));
                    polygon.push_back(cv::Point(img.cols-1, img_height-1)); // Bottom-right corner
                } else {
                    // Right line reaches bottom normally
                    polygon.push_back(bottom_right);
                }
                
                // Handle left side crossing  
                if (bottom_left.x <= 0) {
                    // Left line crosses left edge - add corner and crossing point
                    if (polygon.back().x > 0) {
                        polygon.push_back(cv::Point(0, img_height-1)); // Bottom-left corner
                    }
                    polygon.push_back(cv::Point(0, bottom_left.y));
                } else {
                    // Left line reaches bottom normally
                    polygon.push_back(bottom_left);
                }
                
                return polygon;
            };
            
            // Create transparent overlay for all lane areas
            cv::Mat overlay = img.clone();
            
            // Ego lane indices are now picked geometrically by ADAS (whichever
            // pair of boundaries straddles the camera center), not hardcoded
            // to channels [1,2]. Side lanes are "anything to the left of
            // ego_left" and "anything to the right of ego_right".
            int ego_left  = local_adas_msg->ego_left_lane_idx;
            int ego_right = local_adas_msg->ego_right_lane_idx;

            auto fillLaneAndLabel = [&](int left_idx, int right_idx,
                                        const cv::Scalar& tint, const std::string& label) {
                if (left_idx < 0 || right_idx < 0) return;
                if (!valid_lane_by_color[left_idx] || !valid_lane_by_color[right_idx]) return;
                std::vector<cv::Point> area = createClippedLaneArea(
                    lane_top_points_by_color[left_idx],
                    lane_top_points_by_color[right_idx],
                    lane_bottom_points_by_color[right_idx],
                    lane_bottom_points_by_color[left_idx]
                );
                cv::fillPoly(overlay, area, tint);
                cv::Point center(
                    std::accumulate(area.begin(), area.end(), 0, [](int s, const cv::Point& p){ return s + p.x; }) / (int)area.size(),
                    std::accumulate(area.begin(), area.end(), 0, [](int s, const cv::Point& p){ return s + p.y; }) / (int)area.size()
                );
                cv::putText(overlay, label, center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
            };

            // Find the nearest valid lane to the outside of each ego boundary
            // to paint the adjacent-left and adjacent-right side regions.
            int side_left = -1, side_right = -1;
            if (ego_left >= 0) {
                for (int i = ego_left - 1; i >= 0; --i) {
                    if (valid_lane_by_color[i]) { side_left = i; break; }
                }
            }
            if (ego_right >= 0) {
                for (int i = ego_right + 1; i < 4; ++i) {
                    if (valid_lane_by_color[i]) { side_right = i; break; }
                }
            }

            // Side lane on the left of ego (blue tint)
            fillLaneAndLabel(side_left, ego_left, cv::Scalar(255, 200, 150), "Lane 1");
            // Ego lane (green tint) — the actual current lane region
            fillLaneAndLabel(ego_left, ego_right, cv::Scalar(150, 255, 150), "Lane 2");
            // Side lane on the right of ego (red tint)
            fillLaneAndLabel(ego_right, side_right, cv::Scalar(150, 200, 255), "Lane 3");

            // Blend overlay with original image for transparency
            cv::addWeighted(img, 0.6, overlay, 0.4, 0, img);

            // Availability flags now reflect the geometry-resolved regions.
            lane_region_1_available = (side_left >= 0 && ego_left >= 0
                                       && valid_lane_by_color[side_left] && valid_lane_by_color[ego_left]);
            lane_region_2_available = (ego_left >= 0 && ego_right >= 0
                                       && valid_lane_by_color[ego_left] && valid_lane_by_color[ego_right]);
            lane_region_3_available = (ego_right >= 0 && side_right >= 0
                                       && valid_lane_by_color[ego_right] && valid_lane_by_color[side_right]);

            // Vehicle quadrant identification system
            if (local_detect_msg != NULL) {
                // Clear previous quadrant assignments
                quadrant_v11 = "None";
                quadrant_v12 = "None";
                quadrant_v13 = "None";
                quadrant_v21 = "None";
                quadrant_v22 = "None";
                quadrant_v23 = "None";
                
                // Fixed Y threshold for row division (adjust as needed)
                int row_threshold_y = img_height * 0.75; // Second row is bottom 25% of image
                
                // Data structures for vehicle positioning
                struct VehicleInfo {
                    int detection_idx;
                    int bottom_center_x;
                    int bottom_center_y;
                    std::string track_id;
                    int class_id;
                };
                
                // Collect vehicles by lane
                std::vector<VehicleInfo> lane1_vehicles, lane2_vehicles, lane3_vehicles;
                
                // Analyze each detected vehicle
                for (int i = 0; i < local_detect_msg->num_detections; i++) {
                    // Only analyze tracked road objects (pedestrians, cyclists, cars, buses, trucks, bikes)
                    int det_cls = static_cast<int>(local_detect_msg->classes[i]);
                    bool is_tracked_road_obj = (det_cls >= 0 && det_cls <= 4) || det_cls == 8;
                    if (is_tracked_road_obj && !local_detect_msg->track_list[i].empty()) {
                        // Get vehicle bounding box (scaled coordinates)
                        int x = static_cast<int>(local_detect_msg->boxes[i].data[0]/3);
                        int y = static_cast<int>(local_detect_msg->boxes[i].data[1]/3);
                        int w = static_cast<int>(local_detect_msg->boxes[i].data[2]/3);
                        int h = static_cast<int>(local_detect_msg->boxes[i].data[3]/3);

                        // Use bottom center of bounding box for position
                        int bottom_center_x = x + w/2;
                        int bottom_center_y = y + h;

                        VehicleInfo vehicle = {i, bottom_center_x, bottom_center_y,
                                             local_detect_msg->track_list[i], static_cast<int>(local_detect_msg->classes[i])};
                        
                        // Check which lane the vehicle is in using point-in-polygon test
                        // Lane 1 check
                        if (valid_lane_by_color[0] && valid_lane_by_color[1]) {
                            std::vector<cv::Point> lane1_area = createClippedLaneArea(
                                lane_top_points_by_color[0], lane_top_points_by_color[1],
                                lane_bottom_points_by_color[1], lane_bottom_points_by_color[0]
                            );
                            if (cv::pointPolygonTest(lane1_area, cv::Point2f(bottom_center_x, bottom_center_y), false) >= 0) {
                                lane1_vehicles.push_back(vehicle);
                                continue;
                            }
                        }
                        
                        // Lane 2 check
                        if (valid_lane_by_color[1] && valid_lane_by_color[2]) {
                            std::vector<cv::Point> lane2_area = createClippedLaneArea(
                                lane_top_points_by_color[1], lane_top_points_by_color[2],
                                lane_bottom_points_by_color[2], lane_bottom_points_by_color[1]
                            );
                            if (cv::pointPolygonTest(lane2_area, cv::Point2f(bottom_center_x, bottom_center_y), false) >= 0) {
                                lane2_vehicles.push_back(vehicle);
                                continue;
                            }
                        }
                        
                        // Lane 3 check
                        if (valid_lane_by_color[2] && valid_lane_by_color[3]) {
                            std::vector<cv::Point> lane3_area = createClippedLaneArea(
                                lane_top_points_by_color[2], lane_top_points_by_color[3],
                                lane_bottom_points_by_color[3], lane_bottom_points_by_color[2]
                            );
                            if (cv::pointPolygonTest(lane3_area, cv::Point2f(bottom_center_x, bottom_center_y), false) >= 0) {
                                lane3_vehicles.push_back(vehicle);
                            }
                        }
                    }
                }
                
                // Assign vehicles to quadrants with priority system
                auto assignVehiclesToQuadrants = [&](std::vector<VehicleInfo>& vehicles, int lane_id) {
                    if (vehicles.empty()) return;
                    
                    // Sort by Y coordinate (descending - closest to camera first)
                    std::sort(vehicles.begin(), vehicles.end(), 
                        [](const VehicleInfo& a, const VehicleInfo& b) { return a.bottom_center_y > b.bottom_center_y; });
                    
                    std::string row2_quadrant = "V2" + std::to_string(lane_id); // e.g., V21, V22, V23
                    std::string row1_quadrant = "V1" + std::to_string(lane_id); // e.g., V11, V12, V13
                    
                    // Assign closest vehicle to appropriate row
                    const auto& closest = vehicles[0];
                    std::string assigned_quadrant;
                    
                    if (closest.bottom_center_y >= row_threshold_y) {
                        assigned_quadrant = row2_quadrant; // Second row (close to camera)
                    } else {
                        assigned_quadrant = row1_quadrant; // First row (far from camera)
                    }
                    
                    // Store in global quadrant variables for SceneData publishing
                    if (assigned_quadrant == "V11") quadrant_v11 = closest.track_id;
                    else if (assigned_quadrant == "V12") quadrant_v12 = closest.track_id;
                    else if (assigned_quadrant == "V13") quadrant_v13 = closest.track_id;
                    else if (assigned_quadrant == "V21") quadrant_v21 = closest.track_id;
                    else if (assigned_quadrant == "V22") quadrant_v22 = closest.track_id;
                    else if (assigned_quadrant == "V23") quadrant_v23 = closest.track_id;
                    
                    // Draw vehicle position indicator with just quadrant ID
                    cv::circle(img, cv::Point(closest.bottom_center_x, closest.bottom_center_y), 4, cv::Scalar(255, 255, 255), -1);
                    cv::putText(img, assigned_quadrant, cv::Point(closest.bottom_center_x - 15, closest.bottom_center_y - 10),
                               cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);

                    // If there's a second vehicle, assign it to the other row
                    if (vehicles.size() > 1) {
                        const auto& second = vehicles[1];
                        std::string second_quadrant;

                        if (assigned_quadrant == row2_quadrant) {
                            second_quadrant = row1_quadrant; // Put second vehicle in first row
                        } else {
                            second_quadrant = row2_quadrant; // Put second vehicle in second row
                        }

                        // Store second vehicle in global quadrant variables
                        if (second_quadrant == "V11") quadrant_v11 = second.track_id;
                        else if (second_quadrant == "V12") quadrant_v12 = second.track_id;
                        else if (second_quadrant == "V13") quadrant_v13 = second.track_id;
                        else if (second_quadrant == "V21") quadrant_v21 = second.track_id;
                        else if (second_quadrant == "V22") quadrant_v22 = second.track_id;
                        else if (second_quadrant == "V23") quadrant_v23 = second.track_id;

                        // Draw second vehicle position indicator with just quadrant ID
                        cv::circle(img, cv::Point(second.bottom_center_x, second.bottom_center_y), 3, cv::Scalar(255, 255, 255), -1);
                        cv::putText(img, second_quadrant, cv::Point(second.bottom_center_x - 15, second.bottom_center_y - 10),
                                   cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);
                    }
                };
                
                // Assign vehicles in each lane
                assignVehiclesToQuadrants(lane1_vehicles, 1);
                assignVehiclesToQuadrants(lane2_vehicles, 2); 
                assignVehiclesToQuadrants(lane3_vehicles, 3);
            }
        } else {
            // If ADAS data not available, set lane region availability based on lane data existence
            lane_region_1_available = !local_lanes_msg->xs.empty();
            lane_region_2_available = !local_lanes_msg->xs.empty();
            lane_region_3_available = !local_lanes_msg->xs.empty();
        }
    }

    // Draw ADAS lane-change indicator (warning visual — keep on both
    // dashboard and local display since it's safety-critical and not
    // duplicated anywhere else).
    drawLaneChangeIndicator(img);

    // Publish ONLY the main fused view — BEFORE the bottom status overlay
    // (counts/GPS/IMU/driver). The web dashboard's header chips already
    // surface that telemetry, so duplicating it inside the image is visual
    // noise that also obscures pixels in the lower-left of the scene.
    sensor_msgs::msg::Image msg;
    msg.header.stamp = ROS_TIME_NOW();
    convert_frame_to_message(img, msg);
    gui_pub->publish(msg);

    // Now add the status overlay for the local cv::imshow composite path.
    // The dashboard never sees this version.
    drawStatusOverlay(img);

    // Publish scene data
    publishSceneData();

    // Store annotated main view for timer-based display refresh.
    {
        std::lock_guard<std::mutex> lock(main_view_mutex);
        last_main_view = img.clone();
    }
    // Note: cv::imshow is handled by display_refresh_callback timer at 30 fps
}

// input image subscriber callback
void signs_callback(const visionconnect::msg::Signs::SharedPtr input)
{
    signs_msg = input;
}

// input image subscriber callback
void camera_callback(const sensor_msgs::ImageConstPtr /* input */)
{
    // img_msg = input;
}

// detection overlay callback
void detection_callback(const visionconnect::msg::Detect::SharedPtr input)
{
    detect_msg = input;  
    fuse_data(); // Fuse data from all the topics
}

// detection overlay callback
void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input)
{
    lanes_msg = input;  
}

// ADAS alerts callback
void adas_callback(const visionconnect::msg::ADAS::SharedPtr input)
{
    adas_msg = input;
}

// Driver monitor image callback
void driver_monitor_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
        return;
    }

    // Resize target for driver monitor panel (small size for efficiency)
    const int target_w = 320;
    const int target_h = 180;

    cv::Mat full_image;
    if (msg->encoding == "rgb8") {
        cv::Mat rgb_image(msg->height, msg->width, CV_8UC3,
                         const_cast<uint8_t*>(msg->data.data()), msg->step);
        // Resize first (faster), then convert color on smaller image
        cv::Mat small_rgb;
        cv::resize(rgb_image, small_rgb, cv::Size(target_w, target_h), 0, 0, cv::INTER_NEAREST);
        cv::cvtColor(small_rgb, driver_monitor_image, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "bgr8") {
        cv::Mat bgr_image(msg->height, msg->width, CV_8UC3,
                         const_cast<uint8_t*>(msg->data.data()), msg->step);
        cv::resize(bgr_image, driver_monitor_image, cv::Size(target_w, target_h), 0, 0, cv::INTER_NEAREST);
    } else {
        return;
    }
}

// Driver state callback
void driver_state_callback(const std_msgs::msg::String::SharedPtr msg)
{
    static bool first_call = true;
    driver_state = msg->data;
    if (first_call) {
        ROS_INFO("Driver state callback triggered: %s", driver_state.c_str());
        first_call = false;
    }
}

// BEV image callback
void bev_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) return;

    if (msg->encoding == "bgr8") {
        cv::Mat src(msg->height, msg->width, CV_8UC3,
                    const_cast<uint8_t*>(msg->data.data()), msg->step);
        std::lock_guard<std::mutex> lock(bev_mutex);
        bev_display = src.clone();
    }
}

// Stereo depth callback - applies colormap immediately for fast display
void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static bool first_call = true;

    // Validate image data
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
        return;
    }

    cv::Mat colored;

    if (msg->encoding == "bgr8") {
        // Pre-colorized depth from stereo_depth node — just copy
        cv::Mat src(msg->height, msg->width, CV_8UC3,
                   const_cast<uint8_t*>(msg->data.data()), msg->step);
        colored = src.clone();
        if (first_call) {
            ROS_INFO("Depth callback: bgr8 %dx%d", msg->width, msg->height);
            first_call = false;
        }
    } else if (msg->encoding == "mono8") {
        // Most common case: 8-bit normalized disparity from stereo_depth node
        // Apply colormap directly - no conversion needed
        cv::Mat disp_8u(msg->height, msg->width, CV_8UC1,
                       const_cast<uint8_t*>(msg->data.data()), msg->step);
        cv::applyColorMap(disp_8u, colored, cv::COLORMAP_TURBO);
        if (first_call) {
            ROS_INFO("Depth callback: mono8 %dx%d", msg->width, msg->height);
            first_call = false;
        }
    } else if (msg->encoding == "32FC1") {
        // Float disparity - normalize then colormap
        cv::Mat depth_float(msg->height, msg->width, CV_32FC1,
                           const_cast<uint8_t*>(msg->data.data()), msg->step);
        cv::Mat normalized;
        double min_val, max_val;
        cv::minMaxLoc(depth_float, &min_val, &max_val);
        if (max_val > min_val) {
            depth_float.convertTo(normalized, CV_8U, 255.0 / (max_val - min_val),
                                 -min_val * 255.0 / (max_val - min_val));
        } else {
            normalized = cv::Mat::zeros(depth_float.size(), CV_8U);
        }
        cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);
        if (first_call) {
            ROS_INFO("Depth callback: 32FC1 %dx%d", msg->width, msg->height);
            first_call = false;
        }
    } else if (msg->encoding == "16UC1") {
        // 16-bit depth - normalize then colormap
        cv::Mat depth_16u(msg->height, msg->width, CV_16UC1,
                         const_cast<uint8_t*>(msg->data.data()), msg->step);
        cv::Mat normalized;
        double min_val, max_val;
        cv::minMaxLoc(depth_16u, &min_val, &max_val);
        if (max_val > min_val) {
            depth_16u.convertTo(normalized, CV_8U, 255.0 / (max_val - min_val),
                               -min_val * 255.0 / (max_val - min_val));
        } else {
            normalized = cv::Mat::zeros(depth_16u.size(), CV_8U);
        }
        cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);
        if (first_call) {
            ROS_INFO("Depth callback: 16UC1 %dx%d", msg->width, msg->height);
            first_call = false;
        }
    } else {
        if (first_call) {
            ROS_WARN("Unsupported depth encoding: %s", msg->encoding.c_str());
            first_call = false;
        }
        return;
    }

    // Update colored disparity for display (fast swap)
    {
        std::lock_guard<std::mutex> lock(depth_mutex);
        stereo_depth_colored = colored;
    }
}

// Raw depth callback for 3D bounding box support
void raw_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static bool first_call = true;
    if (msg->encoding != "32FC1") {
        if (first_call) {
            ROS_WARN("Raw depth: expected 32FC1, got %s", msg->encoding.c_str());
            first_call = false;
        }
        return;
    }

    cv::Mat depth(msg->height, msg->width, CV_32FC1,
                  const_cast<uint8_t*>(msg->data.data()), msg->step);

    {
        std::lock_guard<std::mutex> lock(raw_depth_mutex);
        stereo_depth_raw = depth.clone();
    }

    if (first_call) {
        ROS_INFO("Raw depth callback: 32FC1 %dx%d", msg->width, msg->height);
        first_call = false;
    }
}

// IMU callback
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    static bool first_call = true;
    static int callback_count = 0;

    accel_x = msg->linear_acceleration.x;
    accel_y = msg->linear_acceleration.y;
    accel_z = msg->linear_acceleration.z;
    gyro_x = msg->angular_velocity.x;
    gyro_y = msg->angular_velocity.y;
    gyro_z = msg->angular_velocity.z;

    if (first_call) {
        ROS_INFO("IMU callback triggered: accel=%.2f,%.2f,%.2f gyro=%.4f,%.4f,%.4f",
                 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        first_call = false;
    }

    // Log every 50 callbacks
    if (callback_count++ % 50 == 0) {
        ROS_INFO("IMU Update - Accel: [%.2f, %.2f, %.2f] m/s² | Gyro: [%.4f, %.4f, %.4f] rad/s",
                 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    }
}

// GPS callback
void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    static bool first_call = true;
    static int callback_count = 0;

    latitude = msg->latitude;
    longitude = msg->longitude;
    altitude = msg->altitude;

    // Determine fix status
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
        gps_fix_status = "NO FIX";
        gps_satellites = 0;
    } else if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        gps_fix_status = "3D FIX";
        gps_satellites = 4;  // Minimum for 3D fix
    } else if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
        gps_fix_status = "SBAS FIX";
        gps_satellites = 6;
    } else if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
        gps_fix_status = "GBAS FIX";
        gps_satellites = 8;
    }

    if (first_call) {
        ROS_INFO("GPS callback triggered: lat=%.6f lon=%.6f alt=%.1f status=%s",
                 latitude, longitude, altitude, gps_fix_status.c_str());
        first_call = false;
    }

    if (callback_count++ % 50 == 0) {
        ROS_INFO("GPS Update - Lat: %.6f | Lon: %.6f | Alt: %.1f | Status: %s",
                 latitude, longitude, altitude, gps_fix_status.c_str());
    }
}

// Timer callback for display refresh (decouples disparity display from detection rate)
void display_refresh_callback()
{
    cv::Mat main_view;
    {
        std::lock_guard<std::mutex> lock(main_view_mutex);
        if (last_main_view.empty()) return;
        main_view = last_main_view;  // Shallow copy, fast
    }

    // Skip the (expensive) composite build entirely when the on-screen
    // window is disabled — saves ~10ms/frame and frees a core for the rest
    // of the stack. Recording also depends on the composite, so it implies
    // display=true.
    if (!g_display_enabled && !g_record_enabled) return;

    cv::Mat composite = createCompositeDisplay(main_view, screen_width, screen_height);
    if (g_display_enabled) {
        cv::imshow("Perception Fusion", composite);
        cv::waitKey(1);
    }

    if (g_record_enabled) {
        if (!g_recorder.is_open()) {
            if (g_recorder.open(composite.size())) {
                ROS_INFO("Recording fused view to %s (%dx%d)",
                         g_recorder.path().c_str(), composite.cols, composite.rows);
            } else {
                ROS_ERROR("Failed to open recorder at %s — disabling recording",
                          g_recorder.path().c_str());
                g_record_enabled = false;
            }
        }
        g_recorder.push(composite);
    }
}

// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("gui");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");

    // Detect screen resolution using X11
    bool resolution_detected = false;
    Display* disp = XOpenDisplay(NULL);
    if (disp) {
        Screen* scrn = DefaultScreenOfDisplay(disp);
        screen_width = scrn->width;
        screen_height = scrn->height;
        XCloseDisplay(disp);
        resolution_detected = true;
        ROS_INFO("Detected screen resolution via X11: %dx%d", screen_width, screen_height);
    }
    if (!resolution_detected) {
        // Fallback: try xrandr to detect resolution
        FILE* pipe = popen("xrandr 2>/dev/null | grep '\\*' | head -1 | awk '{print $1}'", "r");
        if (pipe) {
            char buf[64];
            if (fgets(buf, sizeof(buf), pipe)) {
                int w, h;
                if (sscanf(buf, "%dx%d", &w, &h) == 2 && w > 0 && h > 0) {
                    screen_width = w;
                    screen_height = h;
                    resolution_detected = true;
                    ROS_INFO("Detected screen resolution via xrandr: %dx%d", screen_width, screen_height);
                }
            }
            pclose(pipe);
        }
    }
    if (!resolution_detected) {
        ROS_WARN("Could not detect screen resolution, using default %dx%d", screen_width, screen_height);
    }

    // Allow overriding screen resolution via ROS parameters
    node->declare_parameter("screen_width", 0);
    node->declare_parameter("screen_height", 0);
    int param_w = 0, param_h = 0;
    node->get_parameter("screen_width", param_w);
    node->get_parameter("screen_height", param_h);
    if (param_w > 0 && param_h > 0) {
        screen_width = param_w;
        screen_height = param_h;
        ROS_INFO("Screen resolution overridden by parameters: %dx%d", screen_width, screen_height);
    }

    // `display` controls whether the on-screen GUI window is shown. When
    // false, the gui node still publishes /gui/fusion for the web dashboard
    // but skips namedWindow/imshow entirely.
    node->declare_parameter("display", true);
    node->get_parameter("display", g_display_enabled);
    ROS_INFO("On-screen GUI window: %s", g_display_enabled ? "enabled" : "disabled");

    if (g_display_enabled) {
        cv::namedWindow("Perception Fusion", cv::WINDOW_NORMAL);
        cv::resizeWindow("Perception Fusion", screen_width, screen_height);
        cv::setWindowProperty("Perception Fusion", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    }

    // Read 3D box config parameter
    node->declare_parameter("enable_3d_boxes", true);
    node->get_parameter("enable_3d_boxes", enable_3d_boxes);
    ROS_INFO("3D bounding boxes: %s", enable_3d_boxes ? "enabled" : "disabled");

    // Recording config: writes the on-screen fused composite to MP4. The
    // worker thread + dropping queue keep this off the display hot path.
    node->declare_parameter("record", false);
    node->declare_parameter("record_path", std::string(""));
    node->declare_parameter("record_fps", 30);
    bool record_enabled = false;
    std::string record_path;
    int record_fps = 30;
    node->get_parameter("record", record_enabled);
    node->get_parameter("record_path", record_path);
    node->get_parameter("record_fps", record_fps);
    if (record_enabled) {
        if (record_path.empty()) {
            std::time_t t = std::time(nullptr);
            std::tm tm{}; localtime_r(&t, &tm);
            char buf[64];
            std::strftime(buf, sizeof(buf), "/tmp/visionsense_%Y%m%d-%H%M%S.mp4", &tm);
            record_path = buf;
        }
        g_recorder.configure(record_path, record_fps);
        g_record_enabled = true;
        ROS_INFO("Recording requested: path=%s fps=%d", record_path.c_str(), record_fps);
    }
    rclcpp::on_shutdown([]() {
        if (g_record_enabled) {
            ROS_INFO("Finalizing recording (%zu frames dropped)", g_recorder.dropped());
            g_recorder.stop();
        }
    });

    // Use BEST_EFFORT QoS for image subscribers to prevent back-pressure
    rclcpp::QoS qos_best_effort(1);
    qos_best_effort.best_effort();
    qos_best_effort.durability_volatile();

    auto cam_sub = node->create_subscription<sensor_msgs::Image>(
        "image_in", qos_best_effort, camera_callback);
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto signs_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs_in", 1, signs_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto adas_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::ADAS, "adas_in", 1, adas_callback);

    // Subscribe to additional data streams for enhanced GUI
    rclcpp::QoS qos_best_effort_2(2);
    qos_best_effort_2.best_effort();
    qos_best_effort_2.durability_volatile();

    // Create a separate callback group for depth to allow parallel execution
    auto depth_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions depth_options;
    depth_options.callback_group = depth_cb_group;

    auto driver_monitor_sub = node->create_subscription<sensor_msgs::Image>(
        "/driver_monitor/image", qos_best_effort_2, driver_monitor_callback);
    auto driver_state_sub = ROS_CREATE_SUBSCRIBER(std_msgs::msg::String, "/driver_monitor/state", 2, driver_state_callback);
    auto depth_sub = node->create_subscription<sensor_msgs::Image>(
        "/stereo_depth/depth_color", qos_best_effort_2, depth_callback, depth_options);

    // Raw depth subscription for 3D bounding boxes
    auto raw_depth_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions raw_depth_options;
    raw_depth_options.callback_group = raw_depth_cb_group;
    auto raw_depth_sub = node->create_subscription<sensor_msgs::Image>(
        "/stereo_depth/depth", qos_best_effort_2, raw_depth_callback, raw_depth_options);

    // BEV node publishes under its own namespace "/bev" (set by ROS_CREATE_NODE
    // in node_bev.cpp) so the full topic is /bev/image, not /bev/bev/image.
    auto bev_sub = node->create_subscription<sensor_msgs::Image>(
        "/bev/image", qos_best_effort_2, bev_callback);
    auto imu_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::Imu, "/imu_gps/imu/data", 2, imu_callback);
    auto gps_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::NavSatFix, "/imu_gps/gps/fix", 2, gps_callback);

    // Fusion publisher: BEST_EFFORT + VOLATILE + KEEP_LAST(1).
    // Depth=1 matches the dashboard subscriber's depth and ensures
    // latest-frame-wins semantics — a slow downstream cannot starve newer
    // frames out of the queue. BEST_EFFORT skips retransmission (correct for
    // video where stale frames are useless).
    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();
    qos_pub.durability_volatile();
    gui_pub = node->create_publisher<sensor_msgs::Image>("fusion", qos_pub);
    ROS_CREATE_PUBLISHER(visionconnect::msg::SceneData, "scene_data", 2, scene_pub);

    // Timer for display refresh (30 Hz) - decouples disparity display from detection rate
    auto display_timer = node->create_wall_timer(
        std::chrono::milliseconds(33),  // ~30 fps
        display_refresh_callback);

    // Use MultiThreadedExecutor to allow depth callback to run in parallel with fuse_data
    ROS_INFO("Preview Node initialized, waiting for images");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    return 0;
}
