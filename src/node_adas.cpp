#include "ros_compat.h"
#include "common.h"
#include "visionconnect/msg/lanes.hpp"
#include "visionconnect/msg/adas.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <array>
#include <limits>

visionconnect::msg::Lanes::SharedPtr lanes_msg = NULL;
Publisher<visionconnect::msg::ADAS> adas_pub = NULL;

class LaneDepartureWarning {
private:
    int image_width_;
    int image_height_;
    float warning_threshold_;
    float severe_threshold_;

    // Normalized X (0..1) of the camera optical center within the lanedet
    // image. Defaults to 0.5 but the physical mount on this rig is biased
    // right-of-center, so the real value is closer to 0.75. Used both as the
    // initial EMA seed and as the ego-pair split point in processLanes().
    float ref_center_x_normalized_;

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
          ref_center_x_normalized_(0.65f),
          calibrated_camera_center_x_(0.0f), convergence_rate_(0.01f),
          min_lane_quality_threshold_(0.85f), is_camera_center_initialized_(false) {
        calibrated_camera_center_x_ = image_width_ * ref_center_x_normalized_;
    }

    void setImageDimensions(int width, int height) {
        image_width_ = width;
        image_height_ = height;
        if (!is_camera_center_initialized_) {
            calibrated_camera_center_x_ = image_width_ * ref_center_x_normalized_;
        }
    }

    void setRefCenterXNormalized(float v) {
        ref_center_x_normalized_ = std::min(0.95f, std::max(0.05f, v));
        if (!is_camera_center_initialized_) {
            calibrated_camera_center_x_ = image_width_ * ref_center_x_normalized_;
        }
    }

    float getRefCenterXNormalized() const { return ref_center_x_normalized_; }
    
    void updateCameraCenter(float current_lane_midpoint, float lane_quality_score) {
        if (lane_quality_score >= min_lane_quality_threshold_) {
            if (!is_camera_center_initialized_) {
                // First high-quality detection: set calibrated center close to lane center
                calibrated_camera_center_x_ = current_lane_midpoint;
                is_camera_center_initialized_ = true;
            } else {
                // Gradual convergence using exponential moving average
                float old_center = calibrated_camera_center_x_;
                calibrated_camera_center_x_ = (1.0f - convergence_rate_) * calibrated_camera_center_x_ + 
                                             convergence_rate_ * current_lane_midpoint;
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
        
        // Extrapolate every valid lane up front. Indices match the model's
        // channel order (blue=0, green=1, red=2, cyan=3) so downstream viz keeps
        // the same coloring; we just don't trust them as ego boundaries anymore.
        std::array<LaneExtrapolation, 4> all_lanes;
        for (auto& l : all_lanes) {
            l = {cv::Point2f(0, 0), cv::Point2f(0, 0), 0.0f, 0.0f, false, 0.0f};
        }
        for (size_t lane_idx = 0; lane_idx < std::min(size_t(4), lane_polylines.size()); lane_idx++) {
            if (valid_lane_by_color[lane_idx]) {
                all_lanes[lane_idx] = extrapolateLane(lane_polylines[lane_idx], max_height_y);
            }
        }

        // Publish per-channel extrapolated points (initialize first, then fill).
        // These are color-coded viz only — ego pair selection happens below.
        for (int i = 0; i < 4; i++) {
            adas_msg.lane_top_x[i]    = 0.0f;
            adas_msg.lane_bottom_x[i] = 0.0f;
            adas_msg.lane_top_y[i]    = 0.0f;
            adas_msg.lane_bottom_y[i] = 0.0f;
            adas_msg.lane_valid[i]    = false;
        }
        for (int i = 0; i < 4; i++) {
            if (all_lanes[i].valid) {
                adas_msg.lane_top_x[i]    = all_lanes[i].top_point.x;
                adas_msg.lane_bottom_x[i] = all_lanes[i].bottom_point.x;
                adas_msg.lane_top_y[i]    = all_lanes[i].top_point.y;
                adas_msg.lane_bottom_y[i] = all_lanes[i].bottom_point.y;
                adas_msg.lane_valid[i]    = true;
            }
        }

        // Ego-lane selection by geometry, not by channel index. Pick the
        // closest valid boundary on each side of the camera optical center at
        // the bottom of the image. Use the *uncalibrated* reference center
        // here (configurable via param, default 0.75 because the physical
        // mount on this rig is biased right-of-image-center) to avoid a
        // feedback loop where a bad selection biases the EMA that we'd then
        // use to keep picking the same bad pair.
        const float ref_center_normalized = ref_center_x_normalized_;
        int ego_left_idx = -1, ego_right_idx = -1;
        float best_left_x  = -std::numeric_limits<float>::infinity();
        float best_right_x =  std::numeric_limits<float>::infinity();
        for (int i = 0; i < 4; i++) {
            if (!all_lanes[i].valid) continue;
            float bx = all_lanes[i].bottom_point.x;
            if (bx < ref_center_normalized && bx > best_left_x) {
                best_left_x = bx;
                ego_left_idx = i;
            } else if (bx > ref_center_normalized && bx < best_right_x) {
                best_right_x = bx;
                ego_right_idx = i;
            }
        }
        adas_msg.ego_left_lane_idx  = static_cast<int8_t>(ego_left_idx);
        adas_msg.ego_right_lane_idx = static_cast<int8_t>(ego_right_idx);

        // Calculate lane departure if both ego boundaries were resolved
        if (ego_left_idx >= 0 && ego_right_idx >= 0) {
            const LaneExtrapolation& left_lane  = all_lanes[ego_left_idx];
            const LaneExtrapolation& right_lane = all_lanes[ego_right_idx];

            float current_lane_midpoint_normalized =
                (left_lane.bottom_point.x + right_lane.bottom_point.x) / 2.0f;
            float combined_lane_quality =
                (left_lane.quality_score + right_lane.quality_score) / 2.0f;

            // Update calibrated camera center (EMA tracks mount offset only —
            // it converges to the lane midpoint observed under good conditions).
            updateCameraCenter(current_lane_midpoint_normalized * image_width_, combined_lane_quality);

            // Deviation is camera-mount-relative-to-lane (uses calibrated center).
            float camera_center_normalized = getCalibratedCameraCenter() / float(image_width_);
            float deviation_normalized = camera_center_normalized - current_lane_midpoint_normalized;

            adas_msg.lane_center_offset        = deviation_normalized;
            adas_msg.calibrated_camera_center_x = camera_center_normalized;
            adas_msg.current_lane_midpoint_x    = current_lane_midpoint_normalized;
            adas_msg.lane_quality_score         = combined_lane_quality;

            if (std::abs(deviation_normalized) > 0.05f) {
                if (deviation_normalized < 0) {
                    adas_msg.lane_change_left = true;
                } else {
                    adas_msg.lane_change_right = true;
                }
            }
        } else {
            ROS_DEBUG("Ego-lane boundaries not available for ADAS processing");
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
        }
    }
}

int main(int argc, char **argv) {
    ROS_CREATE_NODE("adas");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");

    // Image dimensions of the frame lanedet was given (= main_eye stereo crop:
    // 1440x900 when rotated_lenses=false, 1200x1200 when true). Lane points in
    // /lanedet/lanes are in that pixel space; normalizing with the wrong
    // resolution distorts the slope (aspect-ratio mismatch) and extrapolation
    // endpoints, producing visibly tilted lane regions downstream.
    int image_width  = 1440;
    int image_height = 900;
    ROS_DECLARE_PARAMETER("image_width",  image_width);
    ROS_DECLARE_PARAMETER("image_height", image_height);
    ROS_GET_PARAMETER("image_width",  image_width);
    ROS_GET_PARAMETER("image_height", image_height);
    ldw_processor.setImageDimensions(image_width, image_height);
    ROS_INFO("ADAS image dimensions: %dx%d", image_width, image_height);

    // Normalized X position (0..1) of the camera optical center within the
    // lanedet frame. Default 0.65 = camera is mounted right-of-center on this
    // rig; lower to 0.5 if a future re-mount centers the lens. Set in
    // config.yaml under `adas.ros__parameters.ref_center_x_normalized`.
    double ref_center_x_normalized = 0.65;
    ROS_DECLARE_PARAMETER("ref_center_x_normalized", ref_center_x_normalized);
    ROS_GET_PARAMETER("ref_center_x_normalized", ref_center_x_normalized);
    ldw_processor.setRefCenterXNormalized(static_cast<float>(ref_center_x_normalized));
    ROS_INFO("ADAS ref_center_x_normalized: %.3f", ldw_processor.getRefCenterXNormalized());

    // Create subscribers and publishers
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    ROS_CREATE_PUBLISHER(visionconnect::msg::ADAS, "adas_alerts", 5, adas_pub);

    ROS_INFO("ADAS Node initialized, monitoring for lane departure");
    ROS_SPIN();

    return 0;
}
