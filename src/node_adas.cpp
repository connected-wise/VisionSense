#include "ros_compat.h"
#include "common.h"
#include "visionconnect/msg/lanes.hpp"
#include "visionconnect/msg/adas.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>

visionconnect::msg::Lanes::SharedPtr lanes_msg = NULL;
Publisher<visionconnect::msg::ADAS> adas_pub = NULL;

class LaneDepartureWarning {
private:
    int image_width_;
    int image_height_;
    float warning_threshold_;
    float severe_threshold_;
    
    // Camera center calibration
    float calibrated_camera_center_x_;
    float convergence_rate_;
    float min_lane_quality_threshold_;
    bool is_camera_center_initialized_;
    
    struct LaneExtrapolation {
        cv::Point2f top_point;
        cv::Point2f bottom_point;
        float slope;
        float intercept;
        bool valid;
        float quality_score;  // Quality score for this lane
    };
    
public:
    LaneDepartureWarning() 
        : image_width_(1920), image_height_(1080), 
          warning_threshold_(50.0f), severe_threshold_(100.0f),
          calibrated_camera_center_x_(0.0f), convergence_rate_(0.01f),
          min_lane_quality_threshold_(0.85f), is_camera_center_initialized_(false) {
        // Initialize calibrated camera center to image center
        calibrated_camera_center_x_ = image_width_ / 2.0f;
    }
    
    void setImageDimensions(int width, int height) {
        image_width_ = width;
        image_height_ = height;
        // Reinitialize calibrated camera center if dimensions change
        if (!is_camera_center_initialized_) {
            calibrated_camera_center_x_ = image_width_ / 2.0f;
        }
    }
    
    void updateCameraCenter(float current_lane_midpoint, float lane_quality_score) {
        if (lane_quality_score >= min_lane_quality_threshold_) {
            if (!is_camera_center_initialized_) {
                // First high-quality detection: set calibrated center close to lane center
                calibrated_camera_center_x_ = current_lane_midpoint;
                is_camera_center_initialized_ = true;
                ROS_INFO("Camera center initialized to lane midpoint: %.2f", calibrated_camera_center_x_);
            } else {
                // Gradual convergence using exponential moving average
                float old_center = calibrated_camera_center_x_;
                calibrated_camera_center_x_ = (1.0f - convergence_rate_) * calibrated_camera_center_x_ + 
                                             convergence_rate_ * current_lane_midpoint;
                ROS_DEBUG("Camera center calibrated: %.2f -> %.2f (lane: %.2f, quality: %.2f)", 
                         old_center, calibrated_camera_center_x_, current_lane_midpoint, lane_quality_score);
            }
        }
    }
    
    float getCalibratedCameraCenter() const {
        return calibrated_camera_center_x_;
    }
    
    LaneExtrapolation extrapolateLane(const std::vector<cv::Point2f>& polyline, float max_height_y) {
        LaneExtrapolation result;
        result.valid = false;
        
        if (polyline.size() < 5) return result;
        
        // Quality checks using normalized coordinates (0.0-1.0)
        const float MIN_LANE_LENGTH = 0.05f; // Minimum 5% of image height
        const float MIN_Y_RANGE = 0.05f;
        
        cv::Point2f top_point = polyline[0];
        cv::Point2f bottom_point = polyline[0];
        
        for (const auto& point : polyline) {
            if (point.y < top_point.y) top_point = point;
            if (point.y > bottom_point.y) bottom_point = point;
        }
        
        float lane_length = std::abs(bottom_point.y - top_point.y);
        float y_range = lane_length;
        
        if (lane_length < MIN_LANE_LENGTH || y_range < MIN_Y_RANGE) {
            return result;
        }
        
        // Robust line fitting using median slope calculation
        std::vector<float> slopes;
        std::vector<cv::Point2f> core_points;
        
        // Filter out outliers by using middle 70% of points
        std::vector<cv::Point2f> sorted_points = polyline;
        std::sort(sorted_points.begin(), sorted_points.end(), 
            [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        
        int start_idx = sorted_points.size() * 0.15;
        int end_idx = sorted_points.size() * 0.85;
        
        for (int i = start_idx; i < end_idx && i < (int)sorted_points.size(); i++) {
            core_points.push_back(sorted_points[i]);
        }
        
        // Calculate slope between consecutive point pairs
        if (core_points.size() >= 2) {
            for (size_t i = 0; i < core_points.size() - 1; i++) {
                for (size_t j = i + 1; j < core_points.size(); j++) {
                    cv::Point2f p1 = core_points[i];
                    cv::Point2f p2 = core_points[j];
                    
                    if (std::abs(p2.y - p1.y) > 0.01f) { // 1% of image height minimum
                        float point_slope = (p2.x - p1.x) / (p2.y - p1.y);
                        slopes.push_back(point_slope);
                    }
                }
            }
        }
        
        // Calculate median slope
        float slope = 0;
        if (!slopes.empty()) {
            std::sort(slopes.begin(), slopes.end());
            size_t mid = slopes.size() / 2;
            if (slopes.size() % 2 == 0) {
                slope = (slopes[mid-1] + slopes[mid]) / 2.0f;
            } else {
                slope = slopes[mid];
            }
        }
        
        // Calculate intercept using median of core points
        float intercept = 0;
        if (!core_points.empty()) {
            std::vector<float> intercepts;
            for (const auto& pt : core_points) {
                intercepts.push_back(pt.x - slope * pt.y);
            }
            std::sort(intercepts.begin(), intercepts.end());
            size_t mid = intercepts.size() / 2;
            if (intercepts.size() % 2 == 0) {
                intercept = (intercepts[mid-1] + intercepts[mid]) / 2.0f;
            } else {
                intercept = intercepts[mid];
            }
        }
        
        // Calculate extrapolated points in normalized coordinates
        float top_x_at_max_height = slope * max_height_y + intercept;
        result.top_point = cv::Point2f(top_x_at_max_height, max_height_y);
        
        float target_y = 1.0f; // Bottom of image in normalized coordinates
        float bottom_x = slope * 1.0f + intercept;
        
        // Handle edge cases using normalized coordinates
        if (bottom_x < 0.0f) {
            if (std::abs(slope) > 1e-6) {
                target_y = (0.0f - intercept) / slope;
                target_y = std::max(max_height_y + 0.05f, std::min(1.0f, target_y));
                result.bottom_point = cv::Point2f(0.0f, target_y);
            } else {
                result.bottom_point = cv::Point2f(0.0f, max_height_y + 0.1f);
            }
        } else if (bottom_x >= 1.0f) {
            if (std::abs(slope) > 1e-6) {
                target_y = (1.0f - intercept) / slope;
                target_y = std::max(max_height_y + 0.05f, std::min(1.0f, target_y));
                result.bottom_point = cv::Point2f(1.0f, target_y);
            } else {
                result.bottom_point = cv::Point2f(1.0f, max_height_y + 0.1f);
            }
        } else {
            result.bottom_point = cv::Point2f(bottom_x, 1.0f);
        }
        
        result.slope = slope;
        result.intercept = intercept;
        result.valid = true;
        
        // Calculate quality score based on lane length and consistency
        float normalized_length = lane_length; // Already normalized
        float slope_consistency = 0.0f;
        if (!slopes.empty()) {
            size_t mid = slopes.size() / 2;
            slope_consistency = 1.0f / (1.0f + std::abs(slope - slopes[mid]));
        }
        result.quality_score = std::min(1.0f, normalized_length * 2.0f + slope_consistency * 0.5f);
        
        return result;
    }
    
    visionconnect::msg::ADAS processLanes(const visionconnect::msg::Lanes::SharedPtr& lanes) {
        visionconnect::msg::ADAS adas_msg;
        adas_msg.timestamp = ROS_TIME_NOW();
        adas_msg.alert_source = "adas_node";
        adas_msg.lane_change_left = false;
        adas_msg.lane_change_right = false;
        adas_msg.lane_center_offset = 0.0f;
        
        if (!lanes || lanes->xs.empty()) {
            return adas_msg;
        }
        
        // Extract individual lane polylines with normalized coordinates
        std::vector<std::vector<cv::Point2f>> lane_polylines;
        std::vector<cv::Point2f> current_lane;
        
        for (size_t j = 0; j < lanes->xs.size(); ++j) {
            int x = lanes->xs[j];
            int y = lanes->ys[j];

            if (x == -1 && y == -1) {
                // Lane delimiter
                if (!current_lane.empty()) {
                    lane_polylines.push_back(current_lane);
                    current_lane.clear();
                }
            } else {
                // Convert to normalized coordinates (0.0-1.0) directly
                float normalized_x = float(x) / float(image_width_);
                float normalized_y = float(y) / float(image_height_);
                current_lane.push_back(cv::Point2f(normalized_x, normalized_y));
            }
        }
        
        // Add last polyline if exists
        if (!current_lane.empty()) {
            lane_polylines.push_back(current_lane);
        }
        
        if (lane_polylines.size() < 2) {
            ROS_WARN("Insufficient lane data for ADAS processing");
            return adas_msg;
        }
        
        // Find maximum height (minimum Y) across all lanes in normalized coordinates
        float max_height_y = 1.0f; // Start at bottom of image
        std::vector<bool> valid_lane_by_color(4, false);
        
        for (size_t lane_idx = 0; lane_idx < lane_polylines.size() && lane_idx < 4; lane_idx++) {
            const auto& polyline = lane_polylines[lane_idx];
            
            if (polyline.size() >= 5) {
                cv::Point2f top_point = polyline[0];
                for (const auto& point : polyline) {
                    if (point.y < top_point.y) top_point = point;
                }
                max_height_y = std::min(max_height_y, top_point.y);
                valid_lane_by_color[lane_idx] = true;
            }
        }
        
        // Extrapolate green (index 1) and red (index 2) lanes - center lane boundaries
        LaneExtrapolation green_lane = {cv::Point2f(0, 0), cv::Point2f(0, 0), 0.0f, 0.0f, false, 0.0f};
        LaneExtrapolation red_lane = {cv::Point2f(0, 0), cv::Point2f(0, 0), 0.0f, 0.0f, false, 0.0f};
        
        if (valid_lane_by_color[1] && lane_polylines.size() > 1) {
            green_lane = extrapolateLane(lane_polylines[1], max_height_y);
            ROS_INFO("Green lane extrapolated: valid=%d, top=(%.3f,%.3f), bottom=(%.3f,%.3f)", 
                     green_lane.valid, green_lane.top_point.x, green_lane.top_point.y,
                     green_lane.bottom_point.x, green_lane.bottom_point.y);
        }
        
        if (valid_lane_by_color[2] && lane_polylines.size() > 2) {
            red_lane = extrapolateLane(lane_polylines[2], max_height_y);
            ROS_INFO("Red lane extrapolated: valid=%d, top=(%.3f,%.3f), bottom=(%.3f,%.3f)", 
                     red_lane.valid, red_lane.top_point.x, red_lane.top_point.y,
                     red_lane.bottom_point.x, red_lane.bottom_point.y);
        }
        
        // Calculate lane departure if both center lane boundaries are valid
        if (green_lane.valid && red_lane.valid) {
            // Use bottom points directly - already in normalized coordinates
            float green_x_normalized = green_lane.bottom_point.x;
            float red_x_normalized = red_lane.bottom_point.x;
            
            // Debug: Print bottom points 
            ROS_INFO("Bottom Points Normalized - Green: (%.3f,%.3f), Red: (%.3f,%.3f)", 
                     green_lane.bottom_point.x, green_lane.bottom_point.y,
                     red_lane.bottom_point.x, red_lane.bottom_point.y);
            
            // Current lane center midpoint at vehicle position (normalized)
            float current_lane_midpoint_normalized = (green_x_normalized + red_x_normalized) / 2.0f;
            float camera_center_normalized = getCalibratedCameraCenter() / float(image_width_);
            
            ROS_INFO("Normalized coordinates - Green: %.3f, Red: %.3f, Midpoint: %.3f, Camera: %.3f", 
                     green_x_normalized, red_x_normalized, current_lane_midpoint_normalized, camera_center_normalized);
            
            // Calculate combined lane quality score
            float combined_lane_quality = (green_lane.quality_score + red_lane.quality_score) / 2.0f;
            
            // Update calibrated camera center based on current lane quality (convert to pixel coords for internal calculation)
            updateCameraCenter(current_lane_midpoint_normalized * image_width_, combined_lane_quality);
            
            // Calculate new deviation in normalized coordinates
            float deviation_normalized = camera_center_normalized - current_lane_midpoint_normalized;
            adas_msg.lane_center_offset = deviation_normalized;
            
            // Set visualization points in message (normalized coordinates)
            adas_msg.calibrated_camera_center_x = camera_center_normalized;
            adas_msg.current_lane_midpoint_x = current_lane_midpoint_normalized;
            adas_msg.lane_quality_score = combined_lane_quality;
            
            // Publish extrapolated lane points for all lanes (normalized coordinates)
            // Initialize arrays
            for (int i = 0; i < 4; i++) {
                adas_msg.lane_top_x[i] = 0.0f;
                adas_msg.lane_bottom_x[i] = 0.0f;
                adas_msg.lane_top_y[i] = 0.0f;
                adas_msg.lane_bottom_y[i] = 0.0f;
                adas_msg.lane_valid[i] = false;
            }
            
            // Set extrapolated points for all valid lanes
            for (size_t lane_idx = 0; lane_idx < std::min(size_t(4), lane_polylines.size()); lane_idx++) {
                if (valid_lane_by_color[lane_idx]) {
                    LaneExtrapolation lane = extrapolateLane(lane_polylines[lane_idx], max_height_y);
                    if (lane.valid) {
                        // Points are already in normalized coordinates
                        adas_msg.lane_top_x[lane_idx] = lane.top_point.x;
                        adas_msg.lane_bottom_x[lane_idx] = lane.bottom_point.x;
                        adas_msg.lane_top_y[lane_idx] = lane.top_point.y;
                        adas_msg.lane_bottom_y[lane_idx] = lane.bottom_point.y;
                        adas_msg.lane_valid[lane_idx] = true;
                    }
                }
            }
            
            // Determine lane change direction based on deviation (normalized coordinates)
            float abs_deviation_normalized = std::abs(deviation_normalized);
            
            if (abs_deviation_normalized > 0.05f) { // Threshold for lane departure warning (5% of image width)
                if (deviation_normalized < 0) {
                    adas_msg.lane_change_left = true;
                    ROS_INFO("Lane departure left detected, deviation: %.3f (%.1f%%) - Camera: %.3f, Lane: %.3f", 
                             deviation_normalized, deviation_normalized * 100.0f, camera_center_normalized, current_lane_midpoint_normalized);
                } else {
                    adas_msg.lane_change_right = true;
                    ROS_INFO("Lane departure right detected, deviation: %.3f (%.1f%%) - Camera: %.3f, Lane: %.3f", 
                             deviation_normalized, deviation_normalized * 100.0f, camera_center_normalized, current_lane_midpoint_normalized);
                }
            }
        } else {
            ROS_DEBUG("Center lane boundaries not available for ADAS processing");
        }
        
        return adas_msg;
    }
};

LaneDepartureWarning ldw_processor;

void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input) {
    lanes_msg = input;
    
    if (lanes_msg != NULL) {
        // Process lane departure detection
        auto adas_msg = ldw_processor.processLanes(lanes_msg);
        adas_pub->publish(adas_msg);
        
        // Debug output
        if (adas_msg.lane_change_left || adas_msg.lane_change_right) {
            std::string direction = adas_msg.lane_change_left ? "left" : "right";
            ROS_INFO("ADAS Alert: Lane change %s detected, offset: %.2f px", 
                     direction.c_str(), adas_msg.lane_center_offset);
        }
    }
}

int main(int argc, char **argv) {
    ROS_CREATE_NODE("adas");
    
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
    
    // Set image dimensions based on camera resolution (from config)
    ldw_processor.setImageDimensions(1920, 1080);
    
    // Create subscribers and publishers
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    ROS_CREATE_PUBLISHER(visionconnect::msg::ADAS, "adas_alerts", 5, adas_pub);
    
    ROS_INFO("ADAS Node initialized, monitoring for lane departure");
    ROS_SPIN();
    
    return 0;
}