// Bird's Eye View (BEV) mapping node
// Projects detections, lanes, and depth to a top-down view using stereo depth,
// accumulates a local map using GPS+IMU ego-motion estimation.

#include "ros_compat.h"
#include "common.h"
#include <opencv2/opencv.hpp>

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "visionconnect/msg/adas.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <deque>
#include <mutex>
#include <cmath>

// ---- Camera intrinsics (1200x1200 stereo left image) ----
static constexpr float CAM_FX = 750.0f;
static constexpr float CAM_FY = 750.0f;
static constexpr float CAM_CX = 600.0f;
static constexpr float CAM_CY = 600.0f;

// Class-based size priors {length, width} in meters (top-down footprint)
// Index: 0=pedestrian, 1=cyclist, 2=car, 3=bus, 4=truck, 5=train, 6=traffic_light, 7=traffic_sign
struct SizePrior { float length, width; };
static constexpr SizePrior SIZE_PRIORS[] = {
    {0.5f, 0.5f},    // pedestrian
    {1.8f, 0.8f},    // cyclist
    {4.5f, 1.8f},    // car
    {12.0f, 2.6f},   // bus
    {8.0f, 2.5f},    // truck
    {15.0f, 3.0f},   // train
    {0.3f, 0.3f},    // traffic light (not drawn, but avoids out-of-bounds)
    {0.3f, 0.3f},    // traffic sign
};
static constexpr int NUM_SIZE_PRIORS = sizeof(SIZE_PRIORS) / sizeof(SIZE_PRIORS[0]);

// Class colors (BGR) matching node_gui.cpp detection colors
static const cv::Scalar CLASS_COLORS[] = {
    cv::Scalar(0, 200, 255),   // pedestrian - yellow
    cv::Scalar(255, 150, 0),   // cyclist - blue
    cv::Scalar(0, 255, 0),     // car - green
    cv::Scalar(0, 165, 255),   // bus - orange
    cv::Scalar(0, 100, 255),   // truck - dark orange
    cv::Scalar(255, 0, 255),   // train - magenta
    cv::Scalar(0, 255, 255),   // traffic light - cyan
    cv::Scalar(255, 255, 0),   // traffic sign - cyan
};
static constexpr int NUM_CLASS_COLORS = sizeof(CLASS_COLORS) / sizeof(CLASS_COLORS[0]);

// ---- Data Structures ----

struct EgoState {
    double x_m = 0.0;           // Local ENU east (meters)
    double y_m = 0.0;           // Local ENU north (meters)
    double yaw_rad = 0.0;       // Heading (radians, 0=north, CW positive)
    double speed_mps = 0.0;     // Forward speed from GPS
    double lat0 = 0.0, lon0 = 0.0;  // GPS origin for local ENU
    bool origin_set = false;
    rclcpp::Time last_imu_time;
    bool imu_initialized = false;
    double prev_lat = 0.0, prev_lon = 0.0;
    bool prev_gps_set = false;
};

struct BEVObject {
    float bev_x_m, bev_z_m;    // Ego-local: x=right, z=forward (camera frame)
    float length, width;
    int class_id;
    std::string track_id;
    float depth_m;
};

struct MapPoint {
    float world_x, world_y;    // Local ENU frame
    uint8_t type;               // 0=ground, 1=lane, 2=obstacle
    double timestamp_sec;
};

// ---- Global state ----
// SharedPtr assignment is NOT thread-safe on ARM; DDS callbacks fire on network
// threads while the timer reads these from its own thread. A dedicated mutex
// guards the detect/lanes/adas pointers; the depth payload keeps its own mutex
// because it is a cv::Mat (not a SharedPtr).
static visionconnect::msg::Detect::SharedPtr g_detect_msg = nullptr;
static visionconnect::msg::Lanes::SharedPtr  g_lanes_msg  = nullptr;
static visionconnect::msg::ADAS::SharedPtr   g_adas_msg   = nullptr;
static std::mutex g_msg_mutex;

static sensor_msgs::msg::Image::SharedPtr g_depth_msg;  // latest depth message,
                                                        // kept by SharedPtr so
                                                        // we avoid a 5.76 MB
                                                        // clone per frame.
static std::mutex g_depth_mutex;

static EgoState g_ego;
static std::mutex g_ego_mutex;

static double g_gyro_z = 0.0;         // Latest gyro Z reading (rad/s)
static double g_gps_speed = 0.0;      // Latest GPS speed (m/s)

static std::deque<MapPoint> g_map_points;
static std::mutex g_map_mutex;

// Configuration
static int g_bev_width = 480;
static int g_bev_height = 360;
static float g_pixels_per_meter = 10.0f;
static float g_max_bev_range_m = 40.0f;
static float g_camera_height_m = 1.2f;
static float g_map_retention_sec = 30.0f;
static int g_max_map_points = 10000;
static bool g_enable_ground_plane = false;
static float g_gyro_alpha = 0.98f;

Publisher<sensor_msgs::Image> bev_pub = nullptr;

// ---- Helper Functions ----

// Get median depth from a small patch around a pixel
static float sample_depth(const cv::Mat& depth, int u, int v, int patch_size = 5)
{
    if (depth.empty()) return -1.0f;

    int half = patch_size / 2;
    int u0 = std::max(0, u - half);
    int u1 = std::min(depth.cols - 1, u + half);
    int v0 = std::max(0, v - half);
    int v1 = std::min(depth.rows - 1, v + half);

    std::vector<float> samples;
    for (int row = v0; row <= v1; row++) {
        const float* ptr = depth.ptr<float>(row);
        for (int col = u0; col <= u1; col++) {
            float d = ptr[col];
            if (d > 0.1f && d < 100.0f) {
                samples.push_back(d);
            }
        }
    }
    if (samples.empty()) return -1.0f;

    size_t mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    return samples[mid];
}

// Back-project pixel (u, v) with known depth to camera 3D coordinates
// Returns (X, Z) where X=right, Z=forward in camera frame
static std::pair<float, float> backproject_to_ground(float u, float v, float depth)
{
    float X = (u - CAM_CX) * depth / CAM_FX;
    float Z = depth;
    return {X, Z};
}

// Flat ground assumption: estimate depth from pixel row
static float flat_ground_depth(float v, float camera_height)
{
    float dy = v - CAM_CY;
    if (dy < 10.0f) return -1.0f;  // Above horizon
    return CAM_FY * camera_height / dy;
}

// Transform ego-local (x_cam, z_cam) to world (ENU) coordinates
static std::pair<float, float> ego_to_world(float x_cam, float z_cam, const EgoState& ego)
{
    // Camera frame: x=right, z=forward
    // World frame: x=east, y=north
    // yaw_rad: 0=north, CW positive
    double cos_yaw = std::cos(ego.yaw_rad);
    double sin_yaw = std::sin(ego.yaw_rad);

    // Forward (z_cam) maps to north component, right (x_cam) maps to east component
    float world_x = ego.x_m + x_cam * cos_yaw + z_cam * sin_yaw;
    float world_y = ego.y_m - x_cam * sin_yaw + z_cam * cos_yaw;
    return {world_x, world_y};
}

// Transform world coordinates back to ego-local for rendering
static std::pair<float, float> world_to_ego(float world_x, float world_y, const EgoState& ego)
{
    float dx = world_x - ego.x_m;
    float dy = world_y - ego.y_m;
    double cos_yaw = std::cos(-ego.yaw_rad);
    double sin_yaw = std::sin(-ego.yaw_rad);
    float local_x = dx * cos_yaw - dy * sin_yaw;  // right
    float local_z = dx * sin_yaw + dy * cos_yaw;   // forward
    return {local_x, local_z};
}

// Convert ego-local (x=right, z=forward) to BEV pixel coordinates
// BEV: ego at (width/2, height*0.75) facing up
static cv::Point local_to_bev_pixel(float x_local, float z_local)
{
    int px = g_bev_width / 2 + static_cast<int>(x_local * g_pixels_per_meter);
    int py = static_cast<int>(g_bev_height * 0.75f) - static_cast<int>(z_local * g_pixels_per_meter);
    return cv::Point(px, py);
}

// Check if BEV pixel is within canvas bounds
static bool bev_in_bounds(const cv::Point& p)
{
    return p.x >= 0 && p.x < g_bev_width && p.y >= 0 && p.y < g_bev_height;
}

// ---- Callbacks ----

void detect_callback(const visionconnect::msg::Detect::SharedPtr input)
{
    std::lock_guard<std::mutex> lock(g_msg_mutex);
    g_detect_msg = input;
}

void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input)
{
    std::lock_guard<std::mutex> lock(g_msg_mutex);
    g_lanes_msg = input;
}

void adas_callback(const visionconnect::msg::ADAS::SharedPtr input)
{
    std::lock_guard<std::mutex> lock(g_msg_mutex);
    g_adas_msg = input;
}

void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!msg || msg->encoding != "32FC1" || msg->data.empty()) return;

    // Keep the whole message by shared_ptr. The underlying 5.76 MB buffer is
    // ref-counted — the timer just wraps it in a cv::Mat (zero copy) when it
    // samples depth values. Previously this did g_depth_raw.clone() which
    // malloc'd + memcpy'd 5.76 MB on every stereo_depth frame.
    std::lock_guard<std::mutex> lock(g_depth_mutex);
    g_depth_msg = msg;
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double gz = msg->angular_velocity.z;

    std::lock_guard<std::mutex> lock(g_ego_mutex);
    rclcpp::Time now = msg->header.stamp;

    if (g_ego.imu_initialized) {
        double dt = (now - g_ego.last_imu_time).seconds();
        if (dt > 0.0 && dt < 0.5) {
            g_ego.yaw_rad += gz * dt;
            // Normalize yaw to [-pi, pi]
            while (g_ego.yaw_rad > M_PI) g_ego.yaw_rad -= 2.0 * M_PI;
            while (g_ego.yaw_rad < -M_PI) g_ego.yaw_rad += 2.0 * M_PI;
        }
    } else {
        g_ego.imu_initialized = true;
    }
    g_ego.last_imu_time = now;
    g_gyro_z = gz;
}

void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) return;

    double lat = msg->latitude;
    double lon = msg->longitude;

    std::lock_guard<std::mutex> lock(g_ego_mutex);

    if (!g_ego.origin_set) {
        g_ego.lat0 = lat;
        g_ego.lon0 = lon;
        g_ego.origin_set = true;
        g_ego.prev_lat = lat;
        g_ego.prev_lon = lon;
        g_ego.prev_gps_set = true;
        ROS_INFO("BEV: GPS origin set at lat=%.6f lon=%.6f", lat, lon);
        return;
    }

    // Convert to local ENU
    double east = (lon - g_ego.lon0) * std::cos(g_ego.lat0 * M_PI / 180.0) * 111320.0;
    double north = (lat - g_ego.lat0) * 111320.0;
    g_ego.x_m = east;
    g_ego.y_m = north;

    // GPS heading correction when moving
    if (g_ego.prev_gps_set && g_ego.speed_mps > 2.0) {
        double de = (lon - g_ego.prev_lon) * std::cos(g_ego.lat0 * M_PI / 180.0) * 111320.0;
        double dn = (lat - g_ego.prev_lat) * 111320.0;
        double dist = std::sqrt(de * de + dn * dn);
        if (dist > 0.5) {
            double gps_heading = std::atan2(de, dn);  // 0=north, CW positive
            // Complementary filter blend
            g_ego.yaw_rad = g_gyro_alpha * g_ego.yaw_rad + (1.0 - g_gyro_alpha) * gps_heading;
        }
    }
    g_ego.prev_lat = lat;
    g_ego.prev_lon = lon;
    g_ego.prev_gps_set = true;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(g_ego_mutex);
    g_ego.speed_mps = msg->twist.linear.x;
}

// ---- BEV Processing ----

static std::vector<BEVObject> project_detections(const cv::Mat& depth)
{
    std::vector<BEVObject> objects;
    visionconnect::msg::Detect::SharedPtr det;
    {
        std::lock_guard<std::mutex> lock(g_msg_mutex);
        det = g_detect_msg;   // ref bump, no copy
    }
    if (!det) return objects;

    for (uint16_t i = 0; i < det->num_detections; i++) {
        int cls = static_cast<int>(det->classes[i]);
        int bx = det->boxes[i].data[0];
        int by = det->boxes[i].data[1];
        int bw = det->boxes[i].data[2];
        int bh = det->boxes[i].data[3];

        // Skip traffic lights and signs for BEV (not ground objects)
        if (cls >= 6) continue;

        // Bottom-center of bounding box
        float u = bx + bw / 2.0f;
        float v = by + bh;  // Bottom of box

        // Get depth
        float d = sample_depth(depth, static_cast<int>(u), static_cast<int>(v), 7);
        if (d < 0.0f) {
            // Try flat ground assumption
            d = flat_ground_depth(v, g_camera_height_m);
        }
        if (d < 0.5f || d > g_max_bev_range_m) continue;

        auto [X, Z] = backproject_to_ground(u, v, d);

        BEVObject obj;
        obj.bev_x_m = X;
        obj.bev_z_m = Z;
        obj.depth_m = d;
        obj.class_id = cls;
        if (cls < NUM_SIZE_PRIORS) {
            obj.length = SIZE_PRIORS[cls].length;
            obj.width = SIZE_PRIORS[cls].width;
        } else {
            obj.length = 1.0f;
            obj.width = 1.0f;
        }
        if (i < 100 && !det->track_list[i].empty()) {
            obj.track_id = det->track_list[i];
        }
        objects.push_back(obj);
    }
    return objects;
}

static std::vector<std::vector<cv::Point2f>> project_lanes_to_bev(const cv::Mat& depth)
{
    std::vector<std::vector<cv::Point2f>> bev_lanes;
    visionconnect::msg::ADAS::SharedPtr adas;
    {
        std::lock_guard<std::mutex> lock(g_msg_mutex);
        adas = g_adas_msg;
    }
    if (!adas) return bev_lanes;

    // Use ADAS extrapolated lane boundaries (normalized coordinates)
    for (int lane_idx = 0; lane_idx < 4; lane_idx++) {
        if (!adas->lane_valid[lane_idx]) continue;

        float top_x = adas->lane_top_x[lane_idx];
        float top_y = adas->lane_top_y[lane_idx];
        float bot_x = adas->lane_bottom_x[lane_idx];
        float bot_y = adas->lane_bottom_y[lane_idx];

        // Sample 15 points along the lane line
        const int num_samples = 15;
        std::vector<cv::Point2f> bev_lane;

        for (int s = 0; s < num_samples; s++) {
            float t = static_cast<float>(s) / (num_samples - 1);
            float norm_x = top_x + t * (bot_x - top_x);
            float norm_y = top_y + t * (bot_y - top_y);

            // Convert normalized [0,1] to depth-image pixel coordinates. This
            // tracks whatever resolution stereo_depth publishes at, not the
            // previous hardcoded 1200x1200.
            float px = norm_x * static_cast<float>(depth.cols);
            float py = norm_y * static_cast<float>(depth.rows);

            // Get depth at this point
            float d = sample_depth(depth, static_cast<int>(px), static_cast<int>(py), 3);
            if (d < 0.0f) {
                d = flat_ground_depth(py, g_camera_height_m);
            }
            if (d < 0.5f || d > g_max_bev_range_m) continue;

            auto [X, Z] = backproject_to_ground(px, py, d);
            bev_lane.push_back(cv::Point2f(X, Z));
        }

        if (bev_lane.size() >= 2) {
            bev_lanes.push_back(bev_lane);
        }
    }
    return bev_lanes;
}

static void accumulate_map(const std::vector<BEVObject>& objects,
                           const std::vector<std::vector<cv::Point2f>>& lanes,
                           const EgoState& ego, double now_sec)
{
    std::lock_guard<std::mutex> lock(g_map_mutex);

    // Evict old points
    while (!g_map_points.empty() &&
           (now_sec - g_map_points.front().timestamp_sec) > g_map_retention_sec) {
        g_map_points.pop_front();
    }

    // Add lane points to world map
    for (const auto& lane : lanes) {
        for (const auto& pt : lane) {
            auto [wx, wy] = ego_to_world(pt.x, pt.y, ego);
            MapPoint mp;
            mp.world_x = wx;
            mp.world_y = wy;
            mp.type = 1;  // lane
            mp.timestamp_sec = now_sec;
            g_map_points.push_back(mp);
            if (static_cast<int>(g_map_points.size()) > g_max_map_points) {
                g_map_points.pop_front();
            }
        }
    }

    // Add object positions to world map
    for (const auto& obj : objects) {
        auto [wx, wy] = ego_to_world(obj.bev_x_m, obj.bev_z_m, ego);
        MapPoint mp;
        mp.world_x = wx;
        mp.world_y = wy;
        mp.type = 2;  // obstacle
        mp.timestamp_sec = now_sec;
        g_map_points.push_back(mp);
        if (static_cast<int>(g_map_points.size()) > g_max_map_points) {
            g_map_points.pop_front();
        }
    }
}

static cv::Mat render_bev(const std::vector<BEVObject>& objects,
                          const std::vector<std::vector<cv::Point2f>>& lanes,
                          const EgoState& ego)
{
    cv::Mat canvas(g_bev_height, g_bev_width, CV_8UC3, cv::Scalar(40, 42, 45));

    // Draw grid lines every 5 meters
    cv::Scalar grid_color(60, 62, 65);
    float range_x = g_bev_width / (2.0f * g_pixels_per_meter);
    float range_z_front = g_bev_height * 0.75f / g_pixels_per_meter;
    float range_z_back = g_bev_height * 0.25f / g_pixels_per_meter;

    for (float x = -range_x; x <= range_x; x += 5.0f) {
        cv::Point p1 = local_to_bev_pixel(x, -range_z_back);
        cv::Point p2 = local_to_bev_pixel(x, range_z_front);
        cv::line(canvas, p1, p2, grid_color, 1);
    }
    for (float z = -range_z_back; z <= range_z_front; z += 5.0f) {
        cv::Point p1 = local_to_bev_pixel(-range_x, z);
        cv::Point p2 = local_to_bev_pixel(range_x, z);
        cv::line(canvas, p1, p2, grid_color, 1);
    }

    // Draw range rings at 10m, 20m, 30m
    cv::Point ego_pixel = local_to_bev_pixel(0.0f, 0.0f);
    for (float r : {10.0f, 20.0f, 30.0f}) {
        int radius_px = static_cast<int>(r * g_pixels_per_meter);
        cv::circle(canvas, ego_pixel, radius_px, cv::Scalar(55, 57, 60), 1, cv::LINE_AA);
        // Range label
        char range_label[16];
        snprintf(range_label, sizeof(range_label), "%dm", static_cast<int>(r));
        cv::putText(canvas, range_label,
                    cv::Point(ego_pixel.x + 3, ego_pixel.y - radius_px + 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(100, 100, 100), 1);
    }

    // Draw accumulated map points
    {
        std::lock_guard<std::mutex> lock(g_map_mutex);
        for (const auto& mp : g_map_points) {
            auto [lx, lz] = world_to_ego(mp.world_x, mp.world_y, ego);
            cv::Point px = local_to_bev_pixel(lx, lz);
            if (!bev_in_bounds(px)) continue;

            if (mp.type == 1) {
                // Lane point - faded white
                cv::circle(canvas, px, 1, cv::Scalar(120, 120, 120), -1);
            } else if (mp.type == 2) {
                // Obstacle trail - faded red
                cv::circle(canvas, px, 1, cv::Scalar(60, 60, 100), -1);
            }
        }
    }

    // Draw current-frame lane lines (solid white/yellow)
    cv::Scalar lane_colors[] = {
        cv::Scalar(255, 100, 100),  // blue lane
        cv::Scalar(100, 255, 100),  // green lane
        cv::Scalar(100, 100, 255),  // red lane
        cv::Scalar(255, 255, 100),  // cyan lane
    };

    for (size_t i = 0; i < lanes.size() && i < 4; i++) {
        const auto& lane = lanes[i];
        for (size_t j = 1; j < lane.size(); j++) {
            cv::Point p1 = local_to_bev_pixel(lane[j - 1].x, lane[j - 1].y);
            cv::Point p2 = local_to_bev_pixel(lane[j].x, lane[j].y);
            if (bev_in_bounds(p1) || bev_in_bounds(p2)) {
                cv::line(canvas, p1, p2, lane_colors[i], 2, cv::LINE_AA);
            }
        }
    }

    // Draw detected objects as oriented rectangles
    for (const auto& obj : objects) {
        cv::Point center = local_to_bev_pixel(obj.bev_x_m, obj.bev_z_m);
        if (!bev_in_bounds(center)) continue;

        int cls = std::min(obj.class_id, NUM_CLASS_COLORS - 1);
        cv::Scalar color = CLASS_COLORS[std::max(0, cls)];

        // Draw rotated rectangle (oriented forward in BEV)
        float half_l = obj.length / 2.0f * g_pixels_per_meter;
        float half_w = obj.width / 2.0f * g_pixels_per_meter;

        // Object rectangle corners (no rotation -- objects face same as ego for now)
        cv::Point tl(center.x - static_cast<int>(half_w), center.y - static_cast<int>(half_l));
        cv::Point br(center.x + static_cast<int>(half_w), center.y + static_cast<int>(half_l));
        cv::rectangle(canvas, tl, br, color, 2);

        // Fill with semi-transparent color
        cv::Mat roi = canvas(cv::Rect(
            std::max(0, tl.x), std::max(0, tl.y),
            std::min(br.x, g_bev_width - 1) - std::max(0, tl.x),
            std::min(br.y, g_bev_height - 1) - std::max(0, tl.y)));
        if (!roi.empty()) {
            cv::Mat overlay;
            roi.copyTo(overlay);
            cv::rectangle(overlay, cv::Point(0, 0),
                          cv::Point(roi.cols, roi.rows), color, -1);
            cv::addWeighted(overlay, 0.25, roi, 0.75, 0, roi);
        }

        // Draw track ID label
        if (!obj.track_id.empty()) {
            cv::putText(canvas, obj.track_id,
                        cv::Point(center.x - 12, tl.y - 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.32, color, 1);
        }

        // Distance label
        char dist_label[16];
        snprintf(dist_label, sizeof(dist_label), "%.1fm", obj.depth_m);
        cv::putText(canvas, dist_label,
                    cv::Point(center.x - 14, br.y + 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(180, 180, 180), 1);
    }

    // Draw ego vehicle icon (triangle pointing up)
    {
        int ego_size = 12;
        cv::Point pts[3] = {
            cv::Point(ego_pixel.x, ego_pixel.y - ego_size),          // top
            cv::Point(ego_pixel.x - ego_size / 2, ego_pixel.y + ego_size / 2),  // bottom-left
            cv::Point(ego_pixel.x + ego_size / 2, ego_pixel.y + ego_size / 2),  // bottom-right
        };
        cv::fillConvexPoly(canvas, pts, 3, cv::Scalar(0, 220, 255));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{{pts[0], pts[1], pts[2]}},
                      true, cv::Scalar(0, 180, 220), 1, cv::LINE_AA);
    }

    // Draw heading indicator line from ego
    {
        cv::Point heading_end(ego_pixel.x, ego_pixel.y - 25);
        cv::arrowedLine(canvas, ego_pixel, heading_end,
                        cv::Scalar(0, 220, 255), 1, cv::LINE_AA, 0, 0.3);
    }

    // Status overlays
    char text[128];

    // Speed
    snprintf(text, sizeof(text), "%.0f km/h", ego.speed_mps * 3.6);
    cv::putText(canvas, text, cv::Point(8, g_bev_height - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);

    // Heading
    float heading_deg = static_cast<float>(ego.yaw_rad * 180.0 / M_PI);
    if (heading_deg < 0) heading_deg += 360.0f;
    snprintf(text, sizeof(text), "HDG: %.0f", heading_deg);
    cv::putText(canvas, text, cv::Point(8, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(180, 180, 180), 1);

    // GPS status
    if (ego.origin_set) {
        snprintf(text, sizeof(text), "GPS: OK");
        cv::putText(canvas, text, cv::Point(g_bev_width - 65, 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 200, 0), 1);
    } else {
        snprintf(text, sizeof(text), "GPS: --");
        cv::putText(canvas, text, cv::Point(g_bev_width - 65, 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 200), 1);
    }

    // Title
    cv::putText(canvas, "BIRD'S EYE VIEW", cv::Point(g_bev_width / 2 - 65, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(255, 200, 0), 1);

    return canvas;
}

// ---- Timer callback (main processing loop) ----

void bev_timer_callback()
{
    // Skip the whole pipeline when nothing is listening. This is the single
    // biggest perf win for the BEV node: previously projection, map
    // accumulation, and full-canvas rendering ran every tick even when no
    // subscriber was viewing /bev/image.
    if (!bev_pub || bev_pub->get_subscription_count() == 0) {
        return;
    }

    // Snapshot depth — zero-copy via shared_ptr + cv::Mat header wrapper.
    cv::Mat depth;
    sensor_msgs::msg::Image::SharedPtr depth_msg;
    {
        std::lock_guard<std::mutex> lock(g_depth_mutex);
        depth_msg = g_depth_msg;
    }
    if (depth_msg) {
        depth = cv::Mat(depth_msg->height, depth_msg->width, CV_32FC1,
                        const_cast<uint8_t*>(depth_msg->data.data()),
                        depth_msg->step);
    }

    // Snapshot ego state
    EgoState ego;
    {
        std::lock_guard<std::mutex> lock(g_ego_mutex);
        ego = g_ego;
    }

    // Project detections and lanes
    auto objects = project_detections(depth);
    auto lanes = project_lanes_to_bev(depth);

    // Accumulate to world map
    double now_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    accumulate_map(objects, lanes, ego, now_sec);

    // Render BEV
    cv::Mat bev_image = render_bev(objects, lanes, ego);

    // Publish
    sensor_msgs::Image msg;
    msg.header.stamp = ROS_TIME_NOW();
    msg.header.frame_id = "bev";
    convert_frame_to_message(bev_image, msg);
    bev_pub->publish(msg);
}

// ---- Main ----

int main(int argc, char** argv)
{
    ROS_CREATE_NODE("bev");

    // Declare and read parameters
    node->declare_parameter("bev_width", 480);
    node->declare_parameter("bev_height", 360);
    node->declare_parameter("pixels_per_meter", 10.0);
    node->declare_parameter("max_bev_range_m", 40.0);
    node->declare_parameter("camera_height_m", 1.2);
    node->declare_parameter("map_retention_sec", 30.0);
    node->declare_parameter("max_map_points", 10000);
    node->declare_parameter("enable_ground_plane", false);
    node->declare_parameter("render_rate_hz", 10);
    node->declare_parameter("gyro_alpha", 0.98);

    node->get_parameter("bev_width", g_bev_width);
    node->get_parameter("bev_height", g_bev_height);
    double ppm;
    node->get_parameter("pixels_per_meter", ppm);
    g_pixels_per_meter = static_cast<float>(ppm);
    double mbr;
    node->get_parameter("max_bev_range_m", mbr);
    g_max_bev_range_m = static_cast<float>(mbr);
    double ch;
    node->get_parameter("camera_height_m", ch);
    g_camera_height_m = static_cast<float>(ch);
    double mrs;
    node->get_parameter("map_retention_sec", mrs);
    g_map_retention_sec = static_cast<float>(mrs);
    node->get_parameter("max_map_points", g_max_map_points);
    node->get_parameter("enable_ground_plane", g_enable_ground_plane);
    int render_rate_hz = 10;
    node->get_parameter("render_rate_hz", render_rate_hz);
    double ga;
    node->get_parameter("gyro_alpha", ga);
    g_gyro_alpha = static_cast<float>(ga);

    ROS_INFO("BEV Node: %dx%d @ %d Hz, %.1f px/m, range %.1fm",
             g_bev_width, g_bev_height, render_rate_hz, g_pixels_per_meter, g_max_bev_range_m);

    // Create subscribers
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detect_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto adas_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::ADAS, "adas_in", 1, adas_callback);

    // Depth with BEST_EFFORT QoS and separate callback group
    rclcpp::QoS qos_be(1);
    qos_be.best_effort();
    qos_be.durability_volatile();

    auto depth_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions depth_opts;
    depth_opts.callback_group = depth_cb_group;
    auto depth_sub = node->create_subscription<sensor_msgs::Image>(
        "/stereo_depth/depth", qos_be, depth_callback, depth_opts);

    // IMU and GPS
    auto imu_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::Imu, "/imu_gps/imu/data", 2, imu_callback);
    auto gps_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::NavSatFix, "/imu_gps/gps/fix", 2, gps_callback);
    auto vel_sub = ROS_CREATE_SUBSCRIBER(geometry_msgs::msg::TwistStamped, "/imu_gps/gps/velocity", 2, velocity_callback);

    // Publisher
    rclcpp::QoS qos_pub(1);
    qos_pub.best_effort();
    qos_pub.durability_volatile();
    bev_pub = node->create_publisher<sensor_msgs::Image>("image", qos_pub);

    // Timer for BEV rendering
    int period_ms = 1000 / std::max(1, render_rate_hz);
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(period_ms), bev_timer_callback);

    ROS_INFO("BEV Node initialized, waiting for data...");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}
