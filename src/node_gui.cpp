#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "trtutil.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "visionconnect/msg/adas.hpp"
#include "visionconnect/msg/scene_data.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

visionconnect::msg::Detect::SharedPtr detect_msg = NULL;
visionconnect::msg::Signs::SharedPtr signs_msg = NULL;
visionconnect::msg::Lanes::SharedPtr lanes_msg = NULL;
visionconnect::msg::ADAS::SharedPtr adas_msg = NULL;

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

void drawlane(cv::Mat &image)
{
    // The lane points should be calculated in the postprocess function

    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0)
    };

    std::vector<cv::Point> current_lane;
    size_t lane_index = 0;
    for (size_t j = 0; j < lanes_msg->xs.size(); ++j) {
        int x = lanes_msg->xs[j];
        int y = lanes_msg->ys[j];

        if (x == -1 && y == -1) {
            // This means we've reached the delimiter for a lane
            // Draw the current lane if it has points
            if (!current_lane.empty()) {
                cv::polylines(image, current_lane, false, colors[lane_index], 3);
                current_lane.clear();  // Clear current_lane for the next set of points
                lane_index++;
            }
        } else {
            current_lane.push_back(cv::Point(x/3, y/3));
        }
    }
    // Now, draw the circles
    for (size_t j = 0; j < lanes_msg->xs.size(); ++j) {
        int x = lanes_msg->xs[j];
        int y = lanes_msg->ys[j];

        if (x != -1 && y != -1) {
            cv::circle(image, cv::Point(x/3, y/3), 2, cv::Scalar(50,50,50), -1);
        }
    }
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
        int img_width = 1920;  // Use default image width
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
    
    ROS_INFO("Published scene data: Time=%s, EgoTrans=%s, Pedest=%s", 
             scene_data.time.c_str(), scene_data.ego_trans.c_str(), scene_data.pedest.c_str());
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
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(detect_msg->image);

    convert_message_to_frame(img_msg, img);

    if (detect_msg==NULL || signs_msg==NULL || lanes_msg==NULL)
        return;

    drawlane(img);

    auto boxes = detect_msg->boxes;
    auto classes = detect_msg->classes;
    auto scores = detect_msg->scores;
    auto num_detections = detect_msg->num_detections;
    auto track_list = detect_msg->track_list;
    auto signLabels = signs_msg->labels;
    auto signScores = signs_msg->scores;
    
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
        
        else if (classes[i] == 6 && signScores[j]>0.5 && j<signLabels.size())  // if traffic light
        {
            label = signLabels[j];
            std::cout << "label: " << label << std::endl;
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

        else if (classes[i] == 7 && signScores[j]>0.65 && j<signLabels.size())  // if traffic sign
        {
            label = signLabels[j];
            std::cout << "label: " << label << std::endl;

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
    if (lanes_msg != NULL && !lanes_msg->xs.empty()) {
        // Step 1: Extract individual lane polylines
        std::vector<std::vector<cv::Point>> lane_polylines;
        std::vector<cv::Point> current_lane;
        
        for (size_t j = 0; j < lanes_msg->xs.size(); ++j) {
            int x = lanes_msg->xs[j];
            int y = lanes_msg->ys[j];

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
        
        if (adas_msg != NULL && lane_polylines.size() >= 1) {
            int img_height = img.rows;
            int img_width = img.cols;
            
            // Use extrapolated points from ADAS (convert from normalized coordinates)
            std::vector<cv::Point> lane_top_points_by_color(4);
            std::vector<cv::Point> lane_bottom_points_by_color(4);
            std::vector<bool> valid_lane_by_color(4, false);
            
            // Convert normalized coordinates from ADAS to GUI display coordinates
            for (int i = 0; i < 4; i++) {
                if (adas_msg->lane_valid[i]) {
                    lane_top_points_by_color[i] = cv::Point(
                        static_cast<int>(adas_msg->lane_top_x[i] * img_width),
                        static_cast<int>(adas_msg->lane_top_y[i] * img_height)
                    );
                    lane_bottom_points_by_color[i] = cv::Point(
                        static_cast<int>(adas_msg->lane_bottom_x[i] * img_width),
                        static_cast<int>(adas_msg->lane_bottom_y[i] * img_height)
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
            if (detect_msg != NULL) {
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
                for (int i = 0; i < detect_msg->num_detections; i++) {
                    // Only analyze tracked vehicle classes (cars, trucks, buses, pedestrians, cyclists)
                    if (detect_msg->classes[i] >= 0 && detect_msg->classes[i] <= 4 && !detect_msg->track_list[i].empty()) {
                        // Get vehicle bounding box (scaled coordinates)
                        int x = static_cast<int>(detect_msg->boxes[i].data[0]/3);
                        int y = static_cast<int>(detect_msg->boxes[i].data[1]/3);
                        int w = static_cast<int>(detect_msg->boxes[i].data[2]/3);
                        int h = static_cast<int>(detect_msg->boxes[i].data[3]/3);
                        
                        // Use bottom center of bounding box for position
                        int bottom_center_x = x + w/2;
                        int bottom_center_y = y + h;
                        
                        VehicleInfo vehicle = {i, bottom_center_x, bottom_center_y, 
                                             detect_msg->track_list[i], static_cast<int>(detect_msg->classes[i])};
                        
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
                    
                    // Debug output
                    std::cout << "Lane " << lane_id << " - Closest: " << closest.track_id << " -> " << assigned_quadrant;
                    if (vehicles.size() > 1) {
                        std::string second_quadrant = (assigned_quadrant[1] == '2') ? row1_quadrant : row2_quadrant;
                        std::cout << ", Second: " << vehicles[1].track_id << " -> " << second_quadrant;
                    }
                    std::cout << std::endl;
                };
                
                // Assign vehicles in each lane
                assignVehiclesToQuadrants(lane1_vehicles, 1);
                assignVehiclesToQuadrants(lane2_vehicles, 2); 
                assignVehiclesToQuadrants(lane3_vehicles, 3);
            }
        } else {
            // If ADAS data not available, set lane region availability based on lane data existence
            lane_region_1_available = !lanes_msg->xs.empty();
            lane_region_2_available = !lanes_msg->xs.empty();
            lane_region_3_available = !lanes_msg->xs.empty();
        }
    }

    // Draw ADAS indicators on top of everything
    drawLaneChangeIndicator(img);

    // Publish scene data
    publishSceneData();

    sensor_msgs::msg::Image msg;
    msg.header.stamp = ROS_TIME_NOW();
    // Convert output image to ROS message
    convert_frame_to_message(img, msg);
    gui_pub->publish(msg);

    cv::resize(img, img, cv::Size(1920, 1080));
    cv::imshow("Perception Fusion", img);
    cv::waitKey(1);
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
    ROS_INFO("ADAS message received: left=%d, right=%d, offset=%.2f", 
             adas_msg->lane_change_left, adas_msg->lane_change_right, adas_msg->lane_center_offset);
}

// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("gui");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
    cv::namedWindow("Perception Fusion", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Perception Fusion", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    auto cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, camera_callback);
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto signs_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs_in", 1, signs_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto adas_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::ADAS, "adas_in", 1, adas_callback);

    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "fusion", 5, gui_pub);
    ROS_CREATE_PUBLISHER(visionconnect::msg::SceneData, "scene_data", 10, scene_pub);

    // start publishing video frames
    ROS_INFO("Preview Node initialized, waiting for images");
    ROS_SPIN();
    
    return 0;
}