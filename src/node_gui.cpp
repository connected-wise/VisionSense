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
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <numeric>

visionconnect::msg::Detect::SharedPtr detect_msg = NULL;
visionconnect::msg::Signs::SharedPtr signs_msg = NULL;
visionconnect::msg::Lanes::SharedPtr lanes_msg = NULL;
visionconnect::msg::ADAS::SharedPtr adas_msg = NULL;

int screen_width = 1920;
int screen_height = 1080;

// Additional data streams for enhanced GUI
cv::Mat driver_monitor_image;
cv::Mat stereo_depth_colored;      // Pre-colorized disparity for display (updated in callback)
cv::Mat last_main_view;            // Last fused main view (for timer-based refresh)
std::mutex depth_mutex;
std::mutex main_view_mutex;
std::string driver_state = "UNKNOWN";
double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
double latitude = 0.0, longitude = 0.0, altitude = 0.0;
int gps_satellites = 0;
std::string gps_fix_status = "NO FIX";

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

    int num_vehicles = 0, num_pedestrians = 0, num_cyclists = 0;
    if (detect_msg != NULL) {
        for (uint16_t i = 0; i < detect_msg->num_detections; ++i) {
            int cls = static_cast<int>(detect_msg->classes[i]);
            if (cls >= 2 && cls <= 4) num_vehicles++;
            else if (cls == 0) num_pedestrians++;
            else if (cls == 1) num_cyclists++;
        }
    }

    char text[128];
    snprintf(text, sizeof(text), "Vehicles: %d", num_vehicles);
    cv::putText(panel, text, cv::Point(col1_x, col1_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, data_color, 1);
    col1_y += line_height;

    snprintf(text, sizeof(text), "Pedestrians: %d", num_pedestrians);
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

// Helper function to create composite display
cv::Mat createCompositeDisplay(const cv::Mat& main_fused, int screen_width, int screen_height)
{
    // Layout: Main 2/3 width full height, side panels 1/3 width × 1/3 height each
    int main_width = (screen_width * 2) / 3;
    int main_height = screen_height;
    int side_width = screen_width / 3;
    int side_height = screen_height / 3;

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
        std::lock_guard<std::mutex> lock(depth_mutex);
        if (!stereo_depth_colored.empty()) {
            cv::resize(stereo_depth_colored, depth_display, cv::Size(side_width, side_height), 0, 0, cv::INTER_NEAREST);
        }
    }

    if (depth_display.empty()) {
        depth_display = cv::Mat(side_height, side_width, CV_8UC3, cv::Scalar(40, 40, 40));
        cv::putText(depth_display, "Stereo Depth", cv::Point(side_width/2 - 70, side_height/2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 150, 150), 1);
        cv::putText(depth_display, "No Data", cv::Point(side_width/2 - 40, side_height/2 + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
    }
    depth_display.copyTo(composite(cv::Rect(main_width, side_height, side_width, side_height)));
    cv::putText(composite, "STEREO DEPTH", cv::Point(main_width + 10, side_height + 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // Summary panel (bottom-right)
    cv::Mat summary = createSummaryPanel(side_width, side_height);
    summary.copyTo(composite(cv::Rect(main_width, side_height * 2, side_width, side_height)));

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

void drawLaneChangeIndicator(cv::Mat &image)
{
    if (adas_msg == NULL) {
        return;
    }
    
    int img_width = image.cols;
    int img_height = image.rows;
    
    // Lane departure warnings at top
    if (adas_msg->lane_change_left) {
        cv::putText(image, "LANE DEPARTURE LEFT", cv::Point(50, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
    }
    if (adas_msg->lane_change_right) {
        cv::putText(image, "LANE DEPARTURE RIGHT", cv::Point(50, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
    }
    
    // Use calibrated points directly from ADAS message (no recalculation)
    int bottom_y = img_height - 10;  // Position closer to actual bottom
    
    // Convert normalized coordinates (0.0-1.0) to GUI display coordinates
    int calibrated_center_x = static_cast<int>(adas_msg->calibrated_camera_center_x * img_width);
    int lane_midpoint_x = static_cast<int>(adas_msg->current_lane_midpoint_x * img_width);
    
    // Draw calibrated camera center (green vertical line) - aligned to bottom
    cv::line(image, cv::Point(calibrated_center_x, bottom_y - 30), 
             cv::Point(calibrated_center_x, bottom_y), cv::Scalar(0, 255, 0), 2);
    
    // Draw current lane midpoint (blue vertical line) - aligned to bottom
    cv::line(image, cv::Point(lane_midpoint_x, bottom_y - 30), 
             cv::Point(lane_midpoint_x, bottom_y), cv::Scalar(255, 0, 0), 2);
    
    // Draw deviation indicator line between the two points
    cv::line(image, cv::Point(calibrated_center_x, bottom_y - 15), 
             cv::Point(lane_midpoint_x, bottom_y - 15), cv::Scalar(255, 255, 0), 2);
    
    // Show deviation value positioned above the deviation indicator
    int deviation_percent = static_cast<int>(std::round(adas_msg->lane_center_offset * 100.0f));
    std::string deviation_text = std::to_string(deviation_percent) + "%";
    
    // Position label at the center of the deviation indicator
    int label_x = (calibrated_center_x + lane_midpoint_x) / 2;
    int label_y = bottom_y - 40;
    
    // Get text size to center it properly
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(deviation_text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
    label_x -= text_size.width / 2;  // Center the text horizontally
    
    cv::putText(image, deviation_text, cv::Point(label_x, label_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
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
        cv::Scalar(120, 55, 70)   //traffic sign
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

    size_t j = 0; // index for sign and light labels
    cv::String label = "";
    int baseline = 0;

    for (int i = 0; i < num_detections; i++)
    {
        int x = static_cast<int>(boxes[i].data[0]/3);
        int y = static_cast<int>(boxes[i].data[1]/3);
        int w = static_cast<int>(boxes[i].data[2]/3);
        int h = static_cast<int>(boxes[i].data[3]/3);

        if (classes[i] == 0) // if pedestrian
        {
            if (!track_list[i].empty()) {
                label = track_list[i] + " " + std::to_string(scores[i]).substr(0, 4);
            } else {
                label = "pedestrian " + std::to_string(scores[i]).substr(0, 4);
            }
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            if (scores[i] > 0.7)
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
        }

        else if (classes[i] == 1) // if cyclist
        {
            if (!track_list[i].empty()) {
                label = track_list[i] + " " + std::to_string(scores[i]).substr(0, 4);
            } else {
                label = "cyclist " + std::to_string(scores[i]).substr(0, 4);
            }
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            if (scores[i] > 0.7)
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
        }

        else if (classes[i] == 2) // if vehicle-car
        {
            if (!track_list[i].empty()) {
                label = track_list[i] + " " + std::to_string(scores[i]).substr(0, 4);
            } else {
                label = "vehicle-car " + std::to_string(scores[i]).substr(0, 4);
            }
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            cv::rectangle(img, cv::Rect(x, y, w, h), colors[classes[i]], 4);

        }
        
        else if (classes[i] == 3) // if vehicle-bus
        {
            if (!track_list[i].empty()) {
                label = track_list[i] + " " + std::to_string(scores[i]).substr(0, 4);
            } else {
                label = "vehicle-bus " + std::to_string(scores[i]).substr(0, 4);
            }
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            cv::rectangle(img, cv::Rect(x, y, w, h), colors[classes[i]], 4);

        }

        else if (classes[i] == 4) // if vehicle-truck
        {
            if (!track_list[i].empty()) {
                label = track_list[i] + " " + std::to_string(scores[i]).substr(0, 4);
            } else {
                label = "vehicle-truck " + std::to_string(scores[i]).substr(0, 4);
            }
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            cv::rectangle(img, cv::Rect(x, y, w, h), colors[classes[i]], 4);

        }

        else if (classes[i] == 5) // if vehicle-train
        {
            label = "vehicle-train " + std::to_string(scores[i]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            cv::rectangle(img, cv::Rect(x, y, w, h), colors[classes[i]], 4);
        }
        
        else if (classes[i] == 6 && j<signLabels.size() && j<signScores.size() && signScores[j]>0.5)  // if traffic light
        {
            label = signLabels[j];
            if (label == "red light") 
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height);           
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), cv::Scalar(0, 0, 255), cv::FILLED); // Red background
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
                cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
                j++;
            }              

            else if (label == "green light")
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height);  
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), cv::Scalar(55, 235, 100), cv::FILLED); // Green background
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(55, 235, 100), 4);
                cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
                j++;
            }
                
            else if (label == "yellow light")
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height);  
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), cv::Scalar(0, 255, 255), cv::FILLED); // Yellow background
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 255), 4); 
                cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
                j++;
            }
            else
            {
                j++;
                continue;
            }
            
        }

        else if (classes[i] == 7 && j<signLabels.size() && j<signScores.size() && signScores[j]>0.65)  // if traffic sign
        {
            label = signLabels[j];
            if (label != "guide sign")
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height); 
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED); 
                cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);   
            }
            j++;
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
            
            // Lane 1: Filled area between blue and green extrapolated lines
            if (valid_lane_by_color[0] && valid_lane_by_color[1]) {
                std::vector<cv::Point> lane1_area = createClippedLaneArea(
                    lane_top_points_by_color[0],      // Blue top
                    lane_top_points_by_color[1],      // Green top  
                    lane_bottom_points_by_color[1],   // Green bottom
                    lane_bottom_points_by_color[0]    // Blue bottom
                );
                
                // Fill area on overlay with blue tint
                cv::fillPoly(overlay, lane1_area, cv::Scalar(255, 200, 150)); // Blue tint
                
                // Add lane 1 label
                cv::Point lane1_center = cv::Point(
                    std::accumulate(lane1_area.begin(), lane1_area.end(), 0, [](int sum, const cv::Point& p) { return sum + p.x; }) / lane1_area.size(),
                    std::accumulate(lane1_area.begin(), lane1_area.end(), 0, [](int sum, const cv::Point& p) { return sum + p.y; }) / lane1_area.size()
                );
                cv::putText(overlay, "Lane 1", lane1_center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
            }
            
            // Lane 2: Filled area between green and red extrapolated lines  
            if (valid_lane_by_color[1] && valid_lane_by_color[2]) {
                
                std::vector<cv::Point> lane2_area = createClippedLaneArea(
                    lane_top_points_by_color[1],      // Green top
                    lane_top_points_by_color[2],      // Red top
                    lane_bottom_points_by_color[2],   // Red bottom
                    lane_bottom_points_by_color[1]    // Green bottom
                );
                
                // Fill area on overlay with green tint
                cv::fillPoly(overlay, lane2_area, cv::Scalar(150, 255, 150)); // Green tint
                
                // Add lane 2 label
                cv::Point lane2_center = cv::Point(
                    std::accumulate(lane2_area.begin(), lane2_area.end(), 0, [](int sum, const cv::Point& p) { return sum + p.x; }) / lane2_area.size(),
                    std::accumulate(lane2_area.begin(), lane2_area.end(), 0, [](int sum, const cv::Point& p) { return sum + p.y; }) / lane2_area.size()
                );
                cv::putText(overlay, "Lane 2", lane2_center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
            }
            
            // Lane 3: Filled area between red and cyan extrapolated lines
            if (valid_lane_by_color[2] && valid_lane_by_color[3]) {
                std::vector<cv::Point> lane3_area = createClippedLaneArea(
                    lane_top_points_by_color[2],      // Red top
                    lane_top_points_by_color[3],      // Cyan top
                    lane_bottom_points_by_color[3],   // Cyan bottom
                    lane_bottom_points_by_color[2]    // Red bottom
                );
                
                // Fill area on overlay with red tint
                cv::fillPoly(overlay, lane3_area, cv::Scalar(150, 200, 255)); // Red tint
                
                // Add lane 3 label
                cv::Point lane3_center = cv::Point(
                    std::accumulate(lane3_area.begin(), lane3_area.end(), 0, [](int sum, const cv::Point& p) { return sum + p.x; }) / lane3_area.size(),
                    std::accumulate(lane3_area.begin(), lane3_area.end(), 0, [](int sum, const cv::Point& p) { return sum + p.y; }) / lane3_area.size()
                );
                cv::putText(overlay, "Lane 3", lane3_center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
            }
            
            // Blend overlay with original image for transparency
            cv::addWeighted(img, 0.6, overlay, 0.4, 0, img);
            
            // Update lane region availability based on which lane areas can be drawn
            lane_region_1_available = valid_lane_by_color[0] && valid_lane_by_color[1]; // Lane 1 (between lines 0-1)
            lane_region_2_available = valid_lane_by_color[1] && valid_lane_by_color[2]; // Lane 2 (between lines 1-2)  
            lane_region_3_available = valid_lane_by_color[2] && valid_lane_by_color[3]; // Lane 3 (between lines 2-3)

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
                    // Only analyze tracked vehicle classes (cars, trucks, buses, pedestrians, cyclists)
                    if (local_detect_msg->classes[i] >= 0 && local_detect_msg->classes[i] <= 4 && !local_detect_msg->track_list[i].empty()) {
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

    // Draw ADAS indicators on top of everything
    drawLaneChangeIndicator(img);

    // Publish scene data
    publishSceneData();

    // Store main view for timer-based display refresh
    {
        std::lock_guard<std::mutex> lock(main_view_mutex);
        last_main_view = img.clone();
    }

    // Create composite display for ROS publishing
    cv::Mat composite = createCompositeDisplay(img, screen_width, screen_height);

    sensor_msgs::msg::Image msg;
    msg.header.stamp = ROS_TIME_NOW();
    convert_frame_to_message(composite, msg);
    gui_pub->publish(msg);
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

// Stereo depth callback - applies colormap immediately for fast display
void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static bool first_call = true;

    // Validate image data
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
        return;
    }

    cv::Mat colored;

    if (msg->encoding == "mono8") {
        // Most common case: 8-bit normalized disparity from stereo_depth node
        // Apply colormap directly - no conversion needed
        cv::Mat disp_8u(msg->height, msg->width, CV_8UC1,
                       const_cast<uint8_t*>(msg->data.data()), msg->step);
        cv::applyColorMap(disp_8u, colored, cv::COLORMAP_JET);
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
        cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
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
        cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
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

    // Create composite display with latest main view and depth
    cv::Mat composite = createCompositeDisplay(main_view, screen_width, screen_height);
    cv::imshow("Perception Fusion", composite);
    cv::waitKey(1);
}

// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("gui");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");

    // Detect screen resolution using X11
    Display* disp = XOpenDisplay(NULL);
    if (disp) {
        Screen* scrn = DefaultScreenOfDisplay(disp);
        screen_width = scrn->width;
        screen_height = scrn->height;
        XCloseDisplay(disp);
        ROS_INFO("Detected screen resolution: %dx%d", screen_width, screen_height);
    }

    cv::namedWindow("Perception Fusion", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Perception Fusion", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

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
        "/stereo_depth/disparity", qos_best_effort_2, depth_callback, depth_options);
    auto imu_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::Imu, "/imu_gps/imu/data", 2, imu_callback);
    auto gps_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::NavSatFix, "/imu_gps/gps/fix", 2, gps_callback);

    // Publish fusion image with BEST_EFFORT QoS
    rclcpp::QoS qos_pub(2);
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
