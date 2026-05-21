// node_dashboard.cpp
//
// Self-contained dashboard server for VisionSense. Subscribes to image and
// telemetry topics, JPEG-encodes images, and pushes everything directly over
// HTTP + WebSocket to browser clients via the embedded DashboardServer. There
// is intentionally no rosbridge or external HTTP server in the launch graph —
// this single binary owns the data path end-to-end.

#include "ros_compat.h"
#include "common.h"
#include "trtutil.h"
#include "image_converter.h"
#include "dashboard_server.h"
#include "webrtc_server.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using visionsense::DashboardServer;
using visionsense::StreamId;
using visionsense::WebRTCManager;

namespace {

// Rate cap matches the fastest upstream publisher (camera at 30 Hz). Dropping
// below 30 here adds up to 33 ms of forced latency on the camera/detect/lanes
// streams. JPEG q55 is visually indistinguishable from q60 at these sizes and
// shaves ~15 % off the wire payload — and is only used for fallback / non-
// WebRTC-eligible streams (detect, signs).
constexpr double kMaxFps = 30.0;
constexpr int kJpegQuality = 55;
constexpr int kDashboardPort = 8080;

std::unique_ptr<DashboardServer> g_server;
std::unique_ptr<WebRTCManager>   g_webrtc;

// Per-stream frame counters for the 1 Hz FPS telemetry tick.
std::atomic<uint32_t> g_count_fused{0};
std::atomic<uint32_t> g_count_depth{0};
std::atomic<uint32_t> g_count_driver{0};
std::atomic<uint32_t> g_count_bev{0};
std::atomic<uint32_t> g_count_camera{0};

// Raw camera side selector. 0 = left eye, 1 = right eye. Toggled by the
// browser via `{"t":"camera-side","side":"left|right"}`. Single global is
// fine — only one viewer at a time on this Jetson dashboard.
std::atomic<int> g_camera_side{0};

bool rate_limit(std::chrono::steady_clock::time_point &last) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();
    if (elapsed_ms < (1000.0 / kMaxFps)) return false;
    last = now;
    return true;
}

// Encode a cv::Mat to JPEG bytes. Returns empty on failure.
std::vector<uint8_t> jpeg_encode(const cv::Mat &img) {
    std::vector<uint8_t> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
    if (img.empty()) return buffer;
    cv::imencode(".jpg", img, buffer, params);
    return buffer;
}

// Resize-if-needed helper. Skips the copy when dimensions already match.
void resize_to(cv::Mat &img, int width, int height) {
    if (img.cols == width && img.rows == height) return;
    cv::Mat tmp;
    cv::resize(img, tmp, cv::Size(width, height));
    img = std::move(tmp);
}

// Escape a string for embedding inside a JSON string literal (RFC 8259).
std::string json_escape(const std::string &s) {
    std::ostringstream o;
    for (char c : s) {
        switch (c) {
            case '"':  o << "\\\""; break;
            case '\\': o << "\\\\"; break;
            case '\b': o << "\\b";  break;
            case '\f': o << "\\f";  break;
            case '\n': o << "\\n";  break;
            case '\r': o << "\\r";  break;
            case '\t': o << "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    o << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                      << static_cast<int>(c) << std::dec;
                } else {
                    o << c;
                }
        }
    }
    return o.str();
}

}  // namespace

// ---------------------------------------------------------------------------
// Image callbacks — each one: rate-limit, decode → cv::Mat, resize, JPEG,
// push to the WS server.
// ---------------------------------------------------------------------------

// Common helper: fan a resized frame out to WebRTC (always, no-op if no
// subscribers) and to the JPEG fallback path (skipped when WebRTC has a
// subscriber for this stream so we don't burn CPU on encode that nobody
// will read).
static inline void fan_out(StreamId stream, const cv::Mat &img,
                            std::atomic<uint32_t> &counter) {
    g_webrtc->push_frame(stream, img);
    if (!g_webrtc->has_active_subscribers(stream)) {
        auto jpeg = jpeg_encode(img);
        if (!jpeg.empty()) g_server->push_jpeg(stream, std::move(jpeg));
    }
    counter.fetch_add(1, std::memory_order_relaxed);
}

void gui_callback(const sensor_msgs::ImageConstPtr input) {
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    cv::Mat img;
    convert_message_to_frame(input, img);
    resize_to(img, 1280, 720);
    fan_out(visionsense::STREAM_FUSED, img, g_count_fused);
}

// Camera callbacks — one per stereo eye. Whichever side is currently
// selected (g_camera_side) gets fanned out as STREAM_CAMERA; the other is
// silently dropped. Two separate rate limiters so neither starves the other
// when the active side switches.
void camera_left_callback(const sensor_msgs::ImageConstPtr input) {
    if (g_camera_side.load(std::memory_order_relaxed) != 0) return;
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    cv::Mat img;
    convert_message_to_frame(input, img);
    resize_to(img, 1280, 720);
    fan_out(visionsense::STREAM_CAMERA, img, g_count_camera);
}

void camera_right_callback(const sensor_msgs::ImageConstPtr input) {
    if (g_camera_side.load(std::memory_order_relaxed) != 1) return;
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    cv::Mat img;
    convert_message_to_frame(input, img);
    resize_to(img, 1280, 720);
    fan_out(visionsense::STREAM_CAMERA, img, g_count_camera);
}

void depth_callback(const sensor_msgs::ImageConstPtr input) {
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    cv::Mat img;
    convert_message_to_frame(input, img);
    // depth_color upstream is already turbo/inferno-colorized.
    resize_to(img, 720, 450);
    fan_out(visionsense::STREAM_DEPTH, img, g_count_depth);
}

void driver_callback(const sensor_msgs::ImageConstPtr input) {
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    cv::Mat img;
    convert_message_to_frame(input, img);
    resize_to(img, 640, 480);
    fan_out(visionsense::STREAM_DRIVER, img, g_count_driver);
}

void bev_callback(const sensor_msgs::ImageConstPtr input) {
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    cv::Mat img;
    convert_message_to_frame(input, img);
    // BEV is 480×360 native; no resize needed.
    fan_out(visionsense::STREAM_BEV, img, g_count_bev);
}

void detection_callback(const visionconnect::msg::Detect::SharedPtr input) {
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->image);
    cv::Mat img;
    convert_message_to_frame(img_msg, img);
    auto jpeg = jpeg_encode(img);
    if (!jpeg.empty()) {
        g_server->push_jpeg(visionsense::STREAM_DETECT, std::move(jpeg));
    }

    // Log: detection count + per-object class/score, sent as JSON.
    std::ostringstream html;
    int n = static_cast<int>(input->num_detections) - 1;
    if (n < 0) n = 0;
    html << "Detected " << n << " object" << (n == 1 ? "" : "s") << "<br>";
    for (int i = 0; i < n && i < static_cast<int>(input->classes.size()); ++i) {
        html << "&nbsp;&middot; class " << static_cast<int>(input->classes[i])
             << " &mdash; " << std::fixed << std::setprecision(2)
             << input->scores[i] << "<br>";
    }
    std::ostringstream payload;
    payload << "{\"t\":\"log\",\"src\":\"detect\",\"html\":\""
            << json_escape(html.str()) << "\"}";
    g_server->push_json(payload.str());
}

void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input) {
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    auto lane_msg = std::make_shared<sensor_msgs::msg::Image>(input->laneimg);
    cv::Mat lane_img;
    convert_message_to_frame(lane_msg, lane_img);
    resize_to(lane_img, 640, 360);
    auto jpeg = jpeg_encode(lane_img);
    if (!jpeg.empty()) {
        g_server->push_jpeg(visionsense::STREAM_LANES, std::move(jpeg));
    }

    std::ostringstream html;
    html << "Detected " << input->num_lanes << " lane"
         << (input->num_lanes == 1 ? "" : "s") << "<br>";
    for (size_t i = 0; i < input->probs.size() && i < 4; ++i) {
        html << "&nbsp;&middot; lane " << (i + 1) << " &mdash; "
             << std::fixed << std::setprecision(2) << input->probs[i] << "<br>";
    }
    std::ostringstream payload;
    payload << "{\"t\":\"log\",\"src\":\"lanedet\",\"html\":\""
            << json_escape(html.str()) << "\"}";
    g_server->push_json(payload.str());
}

void classify_callback(const visionconnect::msg::Signs::SharedPtr input) {
    static auto last = std::chrono::steady_clock::now();
    if (!rate_limit(last)) return;
    if (input->images.empty()) return;

    // Build a vertical stack of detected-sign crops with their labels overlaid.
    std::vector<cv::Mat> stacked;
    for (size_t i = 0; i < input->images.size() && i < input->labels.size(); ++i) {
        auto im = std::make_shared<sensor_msgs::msg::Image>(input->images[i]);
        cv::Mat crop;
        convert_message_to_frame(im, crop);
        if (crop.empty()) continue;
        cv::resize(crop, crop, cv::Size(200, 200));
        std::string info = input->labels[i];
        if (i < input->scores.size()) {
            info += " (" + std::to_string(static_cast<int>(input->scores[i] * 100)) + "%)";
        }
        cv::putText(crop, info, cv::Point(8, crop.rows - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        stacked.push_back(crop);
    }
    if (stacked.empty()) return;
    cv::Mat composite;
    if (stacked.size() == 1) composite = stacked[0];
    else cv::vconcat(stacked, composite);
    if (composite.rows > 600) {
        double scale = 600.0 / composite.rows;
        cv::resize(composite, composite, cv::Size(0, 0), scale, scale);
    }
    auto jpeg = jpeg_encode(composite);
    if (!jpeg.empty()) {
        g_server->push_jpeg(visionsense::STREAM_SIGNS, std::move(jpeg));
    }

    std::ostringstream html;
    html << "Classified " << input->labels.size() << " sign"
         << (input->labels.size() == 1 ? "" : "s") << "<br>";
    for (size_t i = 0; i < input->labels.size() && i < input->scores.size(); ++i) {
        html << "&nbsp;&middot; " << input->labels[i] << " &mdash; "
             << std::fixed << std::setprecision(2) << input->scores[i] << "<br>";
    }
    std::ostringstream payload;
    payload << "{\"t\":\"log\",\"src\":\"classify\",\"html\":\""
            << json_escape(html.str()) << "\"}";
    g_server->push_json(payload.str());
}

// ---------------------------------------------------------------------------
// Telemetry callbacks — small JSON, no rate limit needed.
// ---------------------------------------------------------------------------

void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::ostringstream o;
    o << "{\"t\":\"gps\",\"lat\":" << std::fixed << std::setprecision(6)
      << msg->latitude << ",\"lon\":" << msg->longitude
      << ",\"alt\":" << std::setprecision(1) << msg->altitude
      << ",\"fix\":" << static_cast<int>(msg->status.status) << "}";
    g_server->push_json(o.str());
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::ostringstream o;
    o << "{\"t\":\"imu\""
      << ",\"ax\":" << std::fixed << std::setprecision(3) << msg->linear_acceleration.x
      << ",\"ay\":" << msg->linear_acceleration.y
      << ",\"az\":" << msg->linear_acceleration.z
      << ",\"wx\":" << msg->angular_velocity.x
      << ",\"wy\":" << msg->angular_velocity.y
      << ",\"wz\":" << msg->angular_velocity.z << "}";
    g_server->push_json(o.str());
}

void driver_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::ostringstream o;
    o << "{\"t\":\"driver\",\"state\":\"" << json_escape(msg->data) << "\"}";
    g_server->push_json(o.str());
}

// ---------------------------------------------------------------------------
// 1 Hz FPS aggregator. Pulls the atomic counters into a single text frame so
// the dashboard can render a per-stream Hz readout without timing every blob
// arrival in JS.
// ---------------------------------------------------------------------------

void fps_tick() {
    auto take = [](std::atomic<uint32_t> &c) -> uint32_t {
        return c.exchange(0, std::memory_order_relaxed);
    };
    uint32_t f = take(g_count_fused);
    uint32_t d = take(g_count_depth);
    uint32_t dr = take(g_count_driver);
    uint32_t b = take(g_count_bev);
    uint32_t cam = take(g_count_camera);
    std::ostringstream o;
    o << "{\"t\":\"fps\""
      << ",\"fused\":" << f
      << ",\"depth\":" << d
      << ",\"driver\":" << dr
      << ",\"bev\":" << b
      << ",\"camera\":" << cam << "}";
    g_server->push_json(o.str());
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
    ROS_CREATE_NODE("dashboard");

    std::string mount_dir;
    try {
        mount_dir = ament_index_cpp::get_package_share_directory("visionconnect")
                    + "/launch/dashboard";
    } catch (const std::exception &e) {
        ROS_INFO("Failed to resolve dashboard share dir: %s", e.what());
        return 1;
    }

    g_server = std::make_unique<DashboardServer>(kDashboardPort, mount_dir);
    if (!g_server->start()) {
        ROS_INFO("Dashboard HTTP+WS server failed to bind port %d", kDashboardPort);
        return 1;
    }
    ROS_INFO("Dashboard HTTP+WS listening on 0.0.0.0:%d (mount=%s)",
             kDashboardPort, mount_dir.c_str());

    g_webrtc = std::make_unique<WebRTCManager>(
        [](visionsense::ClientHandle c, const std::string &json) {
            g_server->push_json_to_client(c, json);
        });
    if (!g_webrtc->start()) {
        ROS_INFO("WebRTC manager failed to start (missing GStreamer elements?)");
        return 1;
    }
    ROS_INFO("WebRTC (NVENC H.264 via webrtcbin) ready — browser will use "
             "video streams for fused/depth/driver/bev/camera/lanes.");

    // Route inbound text frames into (1) the WebRTC signaling handler and
    // then (2) any app-level message handler if WebRTC didn't claim it.
    g_server->set_text_handler(
        [](visionsense::ClientHandle c, const std::string &json) {
            if (g_webrtc->on_signaling_message(c, json)) return true;
            // App-level: {"t":"camera-side","side":"left|right"}
            if (json.find("\"camera-side\"") != std::string::npos) {
                if (json.find("\"right\"") != std::string::npos) {
                    g_camera_side.store(1, std::memory_order_relaxed);
                    ROS_INFO("Camera side switched to RIGHT");
                } else {
                    g_camera_side.store(0, std::memory_order_relaxed);
                    ROS_INFO("Camera side switched to LEFT");
                }
                return true;
            }
            // App-level: {"t":"shutdown"} — gracefully tear down the whole
            // VisionSense launch. We MUST use SIGINT (not SIGTERM/SIGKILL):
            // the stereo camera nodes wedge the V4L2 capture until reboot
            // if killed with anything stronger.
            if (json.find("\"shutdown\"") != std::string::npos) {
                ROS_INFO("Dashboard requested shutdown — signaling launch group");
                g_server->push_json_to_client(
                    c, std::string("{\"t\":\"shutdown-ack\"}"));
                // Detached worker: ack flushes first, then we cascade SIGINT.
                std::thread([]() {
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    // First choice: SIGINT the ros2 launch parent — it
                    // propagates to every child node in the LaunchDescription
                    // (cameras, detect, gui, dashboard, …) in the right order.
                    int rc = std::system(
                        "pkill -INT -f 'ros2 launch visionconnect' 2>/dev/null");
                    (void)rc;
                    // Fallback: if no ros2 launch parent (e.g. someone ran
                    // `ros2 run visionconnect dashboard` standalone), at
                    // least take ourselves down cleanly.
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    if (rclcpp::ok()) {
                        ROS_INFO("No ros2 launch parent found — self-SIGINT");
                        std::raise(SIGINT);
                    }
                }).detach();
                return true;
            }
            return false;
        });
    g_server->set_disconnect_handler([](visionsense::ClientHandle c) {
        g_webrtc->on_client_disconnected(c);
    });

    ROS_INFO("Open http://<host>:%d/ in any browser (PC, Mac, Android, iPhone).",
             kDashboardPort);

    // Image subscribers use BEST_EFFORT QoS so a slow consumer (us, when the
    // browser is gone) cannot back-pressure the upstream camera/detect/gui
    // publishers.
    rclcpp::QoS img_qos(1);
    img_qos.best_effort();
    img_qos.durability_volatile();

    auto cam_left_sub = node->create_subscription<sensor_msgs::Image>(
        "image_in", img_qos, camera_left_callback);
    auto cam_right_sub = node->create_subscription<sensor_msgs::Image>(
        "image_in_right", img_qos, camera_right_callback);
    auto gui_sub = node->create_subscription<sensor_msgs::Image>(
        "gui_in", img_qos, gui_callback);
    auto depth_sub = node->create_subscription<sensor_msgs::Image>(
        "depth_in", img_qos, depth_callback);
    auto driver_sub = node->create_subscription<sensor_msgs::Image>(
        "driver_in", img_qos, driver_callback);
    auto bev_sub = node->create_subscription<sensor_msgs::Image>(
        "bev_in", img_qos, bev_callback);

    auto detect_sub = ROS_CREATE_SUBSCRIBER(
        visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(
        visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto signs_sub = ROS_CREATE_SUBSCRIBER(
        visionconnect::msg::Signs, "signs_in", 1, classify_callback);

    // Telemetry subscribers — small messages, RELIABLE QoS so phones don't
    // miss the occasional GPS fix on a flaky link.
    auto gps_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_in", 10, gps_callback);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu_in", 10, imu_callback);
    auto dms_sub = node->create_subscription<std_msgs::msg::String>(
        "dms_in", 10, driver_state_callback);

    auto fps_timer = node->create_wall_timer(
        std::chrono::seconds(1), []() { fps_tick(); });

    ROS_INFO("Dashboard Node initialized (FPS cap %.1f, JPEG q%d)",
             kMaxFps, kJpegQuality);
    ROS_SPIN();

    g_webrtc->stop();
    g_webrtc.reset();
    g_server->stop();
    g_server.reset();
    return 0;
}
