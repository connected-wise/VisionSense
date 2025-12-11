# ADAS (Advanced Driver Assistance System) alerts message

# Lane change detection
bool lane_change_left
bool lane_change_right
float32 lane_center_offset       # Distance from center in pixels (negative = left, positive = right)

# Calibrated camera center and lane midpoint for visualization
float32 calibrated_camera_center_x    # Adaptive camera center X position
float32 current_lane_midpoint_x       # Current lane center midpoint X position
float32 lane_quality_score            # Quality score of current lane detection

# Extrapolated lane boundary points (normalized coordinates 0.0-1.0)
float32[4] lane_top_x                 # Top X coordinates for 4 lanes (blue, green, red, cyan)
float32[4] lane_bottom_x              # Bottom X coordinates for 4 lanes
float32[4] lane_top_y                 # Top Y coordinates for 4 lanes
float32[4] lane_bottom_y              # Bottom Y coordinates for 4 lanes
bool[4] lane_valid                    # Which lanes are valid

# Timestamp and metadata
builtin_interfaces/Time timestamp
string alert_source              # Source node generating the alert