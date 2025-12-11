# GUI Node Enhancement Plan

## Current State Analysis

The `node_gui.cpp` already handles data fusion from:
- **Detection** (vehicles, pedestrians, cyclists, traffic lights, signs)
- **Lane detection** (lane lines and areas)
- **ADAS** (lane change alerts, lane positioning)
- **Traffic signs** (classification from classifier node)

**Current output:**
- Main fused image scaled from 640x360 to 1920x1080 (line 788)
- Displayed in fullscreen OpenCV window (line 832)

## Proposed Enhancement

### New Layout Structure

```
┌─────────────────────┬──────────────┐
│                     │  Driver Mon  │
│   Main Fused View   │   640x360    │
│      1280x720       │              │
│   (scale 2x from    ├──────────────┤
│    640x360)         │ Stereo Depth │
│                     │   640x360    │
├─────────────────────┴──────────────┤
│     Data Summary Panel             │
│          1920x360                  │
│  (Detection stats, GPS, IMU, etc)  │
└────────────────────────────────────┘
```

**Total Output:** 1920x1080 (scales adaptively to screen resolution)

### Components to Add

#### 1. Driver Monitor Integration
- Subscribe to `/driver_monitor/image` topic
- Display in top-right panel (640x360)
- Shows driver state overlay

#### 2. Stereo Depth Integration
- Subscribe to `/stereo_sgbm/depth` topic
- Convert float depth to colorized visualization
- Display in middle-right panel (640x360)

#### 3. Data Summary Panel (Bottom, 1920x360)

**Column 1: Detections**
- Vehicles count
- Pedestrians count
- Cyclists count
- Lane lines count

**Column 2: Traffic Signs**
- Total signs count
- Sign classifications breakdown

**Column 3: Driver & IMU**
- Driver state (color-coded)
- IMU acceleration (X, Y, Z)
- IMU gyro data

**Column 4: GPS Data**
- Fix status
- Satellite count
- Latitude/Longitude
- Altitude

### Implementation Changes

#### New Subscribers Needed
```cpp
auto driver_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image,
    "/driver_monitor/image", 2, driver_callback);

auto depth_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image,
    "/stereo_sgbm/depth", 2, depth_callback);

auto imu_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::Imu,
    "/imu/data", 2, imu_callback);

auto gps_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::NavSatFix,
    "/gps/fix", 2, gps_callback);

auto driver_state_sub = ROS_CREATE_SUBSCRIBER(std_msgs::msg::String,
    "/driver_monitor/state", 2, driver_state_callback);
```

#### New Global Variables
```cpp
cv::Mat driver_monitor_image;
cv::Mat stereo_depth_image;

// Driver data
std::string driver_state = "UNKNOWN";

// IMU data
double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;

// GPS data
double latitude = 0.0, longitude = 0.0, altitude = 0.0;
int gps_satellites = 0;
std::string gps_fix_status = "NO FIX";
```

#### Modified fuse_data() Function

**Current line 788-790:**
```cpp
sensor_msgs::msg::Image msg;
msg.header.stamp = ROS_TIME_NOW();
convert_frame_to_message(img, msg);
gui_pub->publish(msg);

cv::resize(img, img, cv::Size(1920, 1080));
cv::imshow("Perception Fusion", img);
cv::waitKey(1);
```

**Replace with:**
```cpp
// Create composite display
cv::Mat composite = createCompositeDisplay(img);

sensor_msgs::msg::Image msg;
msg.header.stamp = ROS_TIME_NOW();
convert_frame_to_message(composite, msg);
gui_pub->publish(msg);

cv::imshow("Perception Fusion", composite);
cv::waitKey(1);
```

#### New Helper Functions

**1. createCompositeDisplay()**
```cpp
cv::Mat createCompositeDisplay(const cv::Mat& main_fused)
{
    // Get screen resolution
    cv::Rect screen = cv::getWindowImageRect("Perception Fusion");
    int screen_width = (screen.width > 0) ? screen.width : 1920;
    int screen_height = (screen.height > 0) ? screen.height : 1080;

    // Calculate adaptive layout
    int main_width = (screen_width * 2) / 3;      // 1280 for 1920px
    int main_height = (screen_height * 2) / 3;    // 720 for 1080px
    int side_width = screen_width / 3;             // 640 for 1920px
    int side_height = main_height / 2;             // 360 for 720px
    int summary_height = screen_height - main_height; // 360 for 1080px

    // Create output canvas
    cv::Mat composite(screen_height, screen_width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Scale and place main view
    cv::Mat main_scaled;
    cv::resize(main_fused, main_scaled, cv::Size(main_width, main_height));
    main_scaled.copyTo(composite(cv::Rect(0, 0, main_width, main_height)));

    // Scale and place driver monitor
    if (!driver_monitor_image.empty()) {
        cv::Mat driver_scaled;
        cv::resize(driver_monitor_image, driver_scaled, cv::Size(side_width, side_height));
        driver_scaled.copyTo(composite(cv::Rect(main_width, 0, side_width, side_height)));
    }

    // Scale and place depth view
    if (!stereo_depth_image.empty()) {
        cv::Mat depth_color = depthToColormap(stereo_depth_image);
        cv::Mat depth_scaled;
        cv::resize(depth_color, depth_scaled, cv::Size(side_width, side_height));
        depth_scaled.copyTo(composite(cv::Rect(main_width, side_height, side_width, side_height)));
    }

    // Create and place summary panel
    cv::Mat summary = createSummaryPanel(screen_width, summary_height);
    summary.copyTo(composite(cv::Rect(0, main_height, screen_width, summary_height)));

    // Add panel labels
    addPanelLabels(composite, main_width, side_width, side_height);

    return composite;
}
```

**2. depthToColormap()**
```cpp
cv::Mat depthToColormap(const cv::Mat& depth_float)
{
    if (depth_float.empty()) return cv::Mat();

    // Normalize and colorize
    cv::Mat depth_norm;
    double min_val, max_val;
    cv::minMaxLoc(depth_float, &min_val, &max_val);

    if (max_val > min_val) {
        depth_float.convertTo(depth_norm, CV_8U,
            255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
    } else {
        depth_norm = cv::Mat::zeros(depth_float.size(), CV_8U);
    }

    cv::Mat depth_color;
    cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);

    return depth_color;
}
```

**3. createSummaryPanel()**
```cpp
cv::Mat createSummaryPanel(int width, int height)
{
    cv::Mat panel(height, width, CV_8UC3, cv::Scalar(30, 30, 30));

    int margin = 20;
    int col_width = width / 4;
    int line_height = 30;
    int y = margin + 20;

    cv::Scalar title_color(255, 200, 0);
    cv::Scalar data_color(255, 255, 255);

    // Title
    cv::putText(panel, "SYSTEM STATUS", cv::Point(margin, y),
                cv::FONT_HERSHEY_BOLD, 0.8, title_color, 2);
    y += line_height;

    // Column 1: Detections (use existing detection counts from fuse_data)
    int col1_x = margin;
    drawDetectionStats(panel, col1_x, y, line_height);

    // Column 2: Traffic Signs (use existing sign data)
    int col2_x = col1_x + col_width;
    drawSignStats(panel, col2_x, y, line_height);

    // Column 3: Driver & IMU
    int col3_x = col2_x + col_width;
    drawDriverIMUStats(panel, col3_x, y, line_height);

    // Column 4: GPS
    int col4_x = col3_x + col_width;
    drawGPSStats(panel, col4_x, y, line_height);

    return panel;
}
```

### Screen Adaptive Scaling

**Current Issue:**
- Image is hardcoded to 1920x1080 (line 788)
- Fullscreen mode creates white space on larger screens

**Solution:**
1. Query actual screen resolution using `cv::getWindowImageRect()`
2. Calculate layout proportionally:
   - Main view: 66.7% width, 66.7% height
   - Side panels: 33.3% width, 33.3% height each (stacked)
   - Summary: 100% width, 33.3% height
3. Scale fonts proportionally to screen size

### Benefits

1. **Comprehensive View:** All system data visible at once
2. **Screen Adaptive:** Works on any resolution (1080p, 1440p, 4K)
3. **Better Utilization:** No empty white space
4. **Real-time Monitoring:** Driver safety + navigation + perception
5. **Maintains Existing Code:** Builds on current fusion logic

## Next Steps

1. Add new message subscriptions
2. Implement helper functions
3. Modify fuse_data() to use new composite display
4. Add CMakeLists.txt dependencies if needed (OpenCV freetype for better fonts)
5. Test with all nodes running

## Verification Steps

1. Run all nodes:
   - camera
   - detect
   - classify
   - lanedet
   - adas
   - driver_monitor
   - stereo_sgbm
   - imu_gps
   - gui

2. Verify layout scales correctly on your screen
3. Check all panels update in real-time
4. Confirm no performance degradation
