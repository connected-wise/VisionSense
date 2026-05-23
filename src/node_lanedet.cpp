#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "ros_compat.h"
#include "common.h"
#include "trtutil.h"

#include <opencv2/opencv.hpp>
#include "visionconnect/msg/lanes.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#define INPUT_H 208
#define INPUT_W 976
#define BATCH_SIZE 1
#define POINTS_COUNT 50

// Declare publishers
Publisher<visionconnect::msg::Lanes> lanedet_pub = NULL;

// Globals
Engine *engine;
std::vector<int> YS;
int IMAGE_W, IMAGE_H, CROP_START_Y, CROP_HEIGHT;
float crop_ratio_param = 0.5f;   // bottom half by default; overridden from ROS param

// Preprocessing constants (RGB-channel order). Defaults match the legacy
// uniform-125 centering; production config switches to BGR ImageNet means.
std::array<float, 3> pre_sub_rgb = {125.f, 125.f, 125.f};
std::array<float, 3> pre_div_rgb = {1.f,   1.f,   1.f};
bool pre_normalize = false;

// CLAHE on the cropped band (luma only). Built lazily on first use.
bool clahe_enable = false;
double clahe_clip_limit = 2.0;
int clahe_tile_grid = 8;
cv::Ptr<cv::CLAHE> clahe_obj;

std::vector<int> linspace(int start, int end, int num) {
    std::vector<int> result;
    if (num == 0) return result;
    if (num == 1) {
        result.push_back(start);
        return result;
    }
    double delta = (end - start) / double(num - 1);
    for (int i = 0; i < num; ++i) {
        result.push_back(static_cast<int>(start + delta * i));
    }
    return result;
}


std::pair<std::vector<int>, int> GetLane(const cv::Mat& scoreMap, float thr = 0.3) {

    std::vector<int> coordinate(POINTS_COUNT, 0);
    int coordSum = 0;
    //cv::imshow("ScoreMap", scoreMap);
    //cv::waitKey(0);

    for (int i = 0; i < POINTS_COUNT; i++) {
        int lineId = YS[i];  
        cv::Mat line = scoreMap.row(lineId);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(line, &minVal, &maxVal, &minLoc, &maxLoc);

            if (maxVal > thr) {  // Scale the value by 255 before thresholding.
            coordinate[i] = maxLoc.x;
            coordSum++;
        }
    }

    if (coordSum < 2) {
        std::fill(coordinate.begin(), coordinate.end(), 0);
    }

    return {coordinate, coordSum};
}

std::vector<std::vector<std::pair<int, int>>> GetLines(const std::vector<cv::Mat>& scoreMaps, const std::vector<float>& laneExistenceScores, float thr = 0.2) {
    std::vector<std::vector<std::pair<int, int>>> coordinates;
    std::vector<bool> existArray(scoreMaps.size());

    YS = linspace(4, INPUT_H - 1, POINTS_COUNT);

    for (size_t i = 0; i < laneExistenceScores.size(); i++) {
        existArray[i] = (laneExistenceScores[i] > thr);
    }

    for (size_t l = 0; l <= scoreMaps.size(); l++) {
        if (existArray[l] || l == 0) {
            
            auto [coordinate, coordSum] = GetLane(scoreMaps[l], thr);

            if (coordSum > 1) {
                for (size_t i = 0; i < coordinate.size(); i++) {
                    coordinate[i] = static_cast<int>(coordinate[i]);
                }

                std::vector<std::pair<int, int>> curCoords;
                for (size_t i = 0; i < coordinate.size(); i++) {
                    if (coordinate[i] > 0) {
                        curCoords.push_back({coordinate[i] + 1, YS[i]});
                    }
                }
                coordinates.push_back(curCoords);
            } else {
                coordinates.push_back({});
            }
        } else {
            coordinates.push_back({});
        }
    }

    return coordinates;
}

std::vector<std::vector<float>> run_engine(cv::Mat img, const std::string& encoding)
{
    // ROI starting at `CROP_START_Y` (= IMAGE_H * crop_ratio) with height
    // `CROP_HEIGHT` (the rest of the frame). cudaMemcpy2D in the engine uses
    // img.step so the non-contiguous stride is handled correctly.
    cv::Mat cropped = img(cv::Rect(0, CROP_START_Y, img.cols, CROP_HEIGHT));

    // Optional CLAHE on the luma channel of the cropped band. Stabilizes the
    // input distribution against camera AE / lighting swings without color
    // shifts. We always clone (CLAHE needs contiguous memory anyway), and the
    // engine's cudaMemcpy2D uses the new Mat's step so all good.
    cv::Mat fed = cropped;
    if (clahe_enable) {
        cv::Mat ycc;
        const int code_to_ycc   = (encoding == "bgr8") ? cv::COLOR_BGR2YCrCb : cv::COLOR_RGB2YCrCb;
        const int code_from_ycc = (encoding == "bgr8") ? cv::COLOR_YCrCb2BGR : cv::COLOR_YCrCb2RGB;
        cv::cvtColor(cropped, ycc, code_to_ycc);
        std::vector<cv::Mat> ch;
        cv::split(ycc, ch);
        if (!clahe_obj) {
            clahe_obj = cv::createCLAHE(clahe_clip_limit, cv::Size(clahe_tile_grid, clahe_tile_grid));
        }
        clahe_obj->apply(ch[0], ch[0]);
        cv::merge(ch, ycc);
        cv::cvtColor(ycc, fed, code_from_ycc);
    }

    const auto &inputDims = engine->getInputDims();
    std::vector<std::vector<cv::Mat>> inputs;
    inputs.reserve(inputDims.size());
    for (size_t k = 0; k < inputDims.size(); ++k) {
        std::vector<cv::Mat> input(BATCH_SIZE, fed);
        inputs.emplace_back(std::move(input));
    }

    std::vector<std::vector<std::vector<float>>> featureVectors;
    // sub/div are in RGB channel order. The CUDA kernel applies swap_rb FIRST,
    // so after that step the data is RGB regardless of input encoding and the
    // RGB-ordered constants are correct in both cases.
    bool success = engine->runInference(inputs, featureVectors,
                                        pre_sub_rgb, pre_div_rgb,
                                        /*normalize=*/pre_normalize,
                                        /*swap_rb=*/(encoding == "bgr8"),
                                        /*aspect_ratio_pad=*/false);
    if (!success)
    {
        throw std::runtime_error("Unable to run inference.");
    }

    return featureVectors[0];
}

void drawlane(visionconnect::msg::Lanes &lanes_msg, cv::Mat &image)
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
    for (size_t j = 0; j < lanes_msg.xs.size(); ++j) {
        int x = lanes_msg.xs[j];
        int y = lanes_msg.ys[j];

        if (x == -1 && y == -1) {
            // This means we've reached the delimiter for a lane
            // Draw the current lane if it has points
            if (!current_lane.empty()) {
                cv::polylines(image, current_lane, false, colors[lane_index], 7);
                current_lane.clear();  // Clear current_lane for the next set of points
                lane_index++;
            }
        } else {
            current_lane.push_back(cv::Point(x, y));
        }
    }
    // Now, draw the circles
    for (size_t j = 0; j < lanes_msg.xs.size(); ++j) {
        int x = lanes_msg.xs[j];
        int y = lanes_msg.ys[j];

        if (x != -1 && y != -1) {
            cv::circle(image, cv::Point(x, y), 5, cv::Scalar(50,50,50), -1);
        }
    }
}

void postprocess(std::vector<std::vector<float>> &output, cv::Mat &image)
{
    visionconnect::msg::Lanes lanes_msg;
    std::vector<cv::Mat> lanes;
    cv::Mat laneresult, rawimg; 

    // Check if we have enough outputs
    if (output.size() < 2) {
        ROS_ERROR("Not enough outputs from model. Expected at least 2, got %zu", output.size());
        return;
    }
    
    // The model outputs are:
    // output[0]: lane maps with shape [1, 5, 208, 976] - flattened to 5*208*976 values
    // output[1]: lane existence scores with shape [1, 4] - 4 values
    auto laneMapVector = output[0];  // This contains all 5 channels of lane maps
    auto laneExistenceVector = output[1];  // This contains 4 lane existence scores
    
    // Check sizes
    size_t expected_map_size = 5 * INPUT_H * INPUT_W;  // 5 channels
    if (laneMapVector.size() < expected_map_size) {
        ROS_ERROR("Output[0] size mismatch. Expected %zu, got %zu", expected_map_size, laneMapVector.size());
        return;
    }
    
    if (laneExistenceVector.size() < 4) {
        ROS_ERROR("Output[1] size mismatch. Expected 4, got %zu", laneExistenceVector.size());
        return;
    }
    
    // Create the raw image from the first channel (background)
    rawimg = cv::Mat(INPUT_H, INPUT_W, CV_32F, laneMapVector.data());
    rawimg.convertTo(rawimg, CV_8UC1, 255.0);
    
    sensor_msgs::msg::Image raw;
    raw.header.stamp = ROS_TIME_NOW();
    convert_frame_to_message(rawimg, raw);

    lanes_msg.rawimg = raw;

    // Process the 4 lane channels (skip channel 0 which is background)
    for (int c = 1; c < 5; ++c) {
        float* startPtr = &laneMapVector[c * INPUT_H * INPUT_W];
        cv::Mat lane(INPUT_H, INPUT_W, CV_32F, startPtr);
        cv::blur(lane, lane, cv::Size(11, 11));
        lanes.push_back(lane);
    }
    
    // lines data is std::vector<std::vector<std::pair<int, int>>>
    // Pass the lane existence scores from output[1]
    auto lines = GetLines(lanes, laneExistenceVector, 0.25);
    
    // Map (x,y) from network input space (INPUT_W × INPUT_H) back into the
    // full image. Y must use CROP_HEIGHT (the size of the actual crop) and
    // the offset is CROP_START_Y (where the crop began). The old code used
    // (IMAGE_H - CROP) for the scale, which only matched CROP_HEIGHT when
    // the crop was a perfect 50/50 split — any other ratio mislocated lanes.
    float scaleY = CROP_HEIGHT / static_cast<float>(INPUT_H);
    float scaleX = IMAGE_W / static_cast<float>(INPUT_W);
    size_t lane_index = 0;
    for (const auto& line : lines) {
        for (const auto& point : line) {
            int x = point.first;
            int y = point.second;
            x = static_cast<int>(x * scaleX);
            y = static_cast<int>(y * scaleY + CROP_START_Y);
            lanes_msg.xs.push_back(x);
            lanes_msg.ys.push_back(y);
        }
        // Add delimiters after each lane
        lanes_msg.xs.push_back(-1);
        lanes_msg.ys.push_back(-1);
        if (lane_index < laneExistenceVector.size()) {
            lanes_msg.probs[lane_index] = laneExistenceVector[lane_index];
        }
        lane_index++;
    }
    lanes_msg.num_lanes = lane_index;
    drawlane(lanes_msg, image);
    cv::resize(image, laneresult, cv::Size(IMAGE_W/3, IMAGE_H/3));
    
    // Convert lane results image to ROS message
    sensor_msgs::msg::Image msg;
    msg.header.stamp = ROS_TIME_NOW();
    convert_frame_to_message(laneresult, msg);
    lanes_msg.laneimg = msg;

    lanedet_pub->publish(lanes_msg);

    //cv::imshow("Lane Detection", laneresult);
    //cv::waitKey(1);
}

// input image subscriber callback
void img_callback(const sensor_msgs::msg::Image::SharedPtr input)
{
    cv::Mat img;
    convert_message_to_frame(input, img);

    IMAGE_W = img.cols;
    IMAGE_H = img.rows;
    // Clamp to a safe range so a bad config can't produce a zero-height crop.
    float r = std::min(0.95f, std::max(0.0f, crop_ratio_param));
    CROP_START_Y = static_cast<int>(IMAGE_H * r);
    CROP_HEIGHT  = IMAGE_H - CROP_START_Y;

    if(img.empty())
    {
        ROS_INFO("Failed to convert the input to Opencv image");
        return;
    }

    auto output = run_engine(img, input->encoding);
    postprocess(output, img);
}

void load_engine(std::string path)
{
    if (!Engine::doesFileExist(path))
    {
        throw std::runtime_error("Error: Unable to find engine at:" + path);
    } 

    // Specify our GPU inference configuration options
    Options options;
    options.precision = Precision::FP16;
    options.optBatchSize = 1;
    options.maxBatchSize = 1;

    engine = new Engine(options);

    // Load the TensorRT engine file from disk
    bool success = engine->loadNetwork(path);
    if (!success)
    {
        throw std::runtime_error("Unable to load TRT engine.");
    }
}


int main(int argc, char *argv[])
{

    ROS_CREATE_NODE("lanedet");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
    std::string model_str, engine_path;

    ROS_DECLARE_PARAMETER("model", model_str);
    ROS_GET_PARAMETER("model", model_str);

    // Horizon crop ratio (fraction of image height skipped from top before
    // feeding the network). Tune with scripts/lanedet_crop_calibrator.py.
    double crop_ratio = 0.5;
    ROS_DECLARE_PARAMETER("crop_ratio", crop_ratio);
    ROS_GET_PARAMETER("crop_ratio", crop_ratio);
    crop_ratio_param = static_cast<float>(crop_ratio);
    ROS_INFO("lanedet crop_ratio = %.3f (= skip top %.0f%% of image)",
             crop_ratio_param, crop_ratio_param * 100.0f);

    // Preprocessing constants. Vectors in RGB order; defaults preserve the
    // legacy uniform-125 centering, so absent config = unchanged behavior.
    std::vector<double> sub_rgb_v = {125.0, 125.0, 125.0};
    std::vector<double> div_rgb_v = {1.0,   1.0,   1.0};
    bool normalize_v = false;
    ROS_DECLARE_PARAMETER("pre_sub_rgb",   sub_rgb_v);
    ROS_DECLARE_PARAMETER("pre_div_rgb",   div_rgb_v);
    ROS_DECLARE_PARAMETER("pre_normalize", normalize_v);
    ROS_GET_PARAMETER("pre_sub_rgb",   sub_rgb_v);
    ROS_GET_PARAMETER("pre_div_rgb",   div_rgb_v);
    ROS_GET_PARAMETER("pre_normalize", normalize_v);
    if (sub_rgb_v.size() == 3) {
        for (int i = 0; i < 3; ++i) pre_sub_rgb[i] = static_cast<float>(sub_rgb_v[i]);
    }
    if (div_rgb_v.size() == 3) {
        for (int i = 0; i < 3; ++i) pre_div_rgb[i] = static_cast<float>(div_rgb_v[i]);
    }
    pre_normalize = normalize_v;
    ROS_INFO("lanedet preprocess: sub=[%.3f, %.3f, %.3f] div=[%.3f, %.3f, %.3f] /255=%s",
             pre_sub_rgb[0], pre_sub_rgb[1], pre_sub_rgb[2],
             pre_div_rgb[0], pre_div_rgb[1], pre_div_rgb[2],
             pre_normalize ? "yes" : "no");

    // CLAHE on the cropped band (luma channel only, optional).
    bool clahe_v = false;
    double clahe_clip_v = 2.0;
    int    clahe_grid_v = 8;
    ROS_DECLARE_PARAMETER("clahe_enable",     clahe_v);
    ROS_DECLARE_PARAMETER("clahe_clip_limit", clahe_clip_v);
    ROS_DECLARE_PARAMETER("clahe_tile_grid",  clahe_grid_v);
    ROS_GET_PARAMETER("clahe_enable",     clahe_v);
    ROS_GET_PARAMETER("clahe_clip_limit", clahe_clip_v);
    ROS_GET_PARAMETER("clahe_tile_grid",  clahe_grid_v);
    clahe_enable     = clahe_v;
    clahe_clip_limit = clahe_clip_v;
    clahe_tile_grid  = std::max(2, clahe_grid_v);
    ROS_INFO("lanedet CLAHE: %s (clip=%.2f, tile=%dx%d)",
             clahe_enable ? "ON" : "off",
             clahe_clip_limit, clahe_tile_grid, clahe_tile_grid);

    engine_path = package_share_directory + "/graphs/lane-detection/" + model_str;

    load_engine(engine_path);

    // Use BEST_EFFORT QoS to match camera publisher and prevent back-pressure
    rclcpp::QoS qos_best_effort(1);
    qos_best_effort.best_effort();
    qos_best_effort.durability_volatile();
    auto img_sub = node->create_subscription<sensor_msgs::Image>(
        "image_in", qos_best_effort, img_callback);

    ROS_CREATE_PUBLISHER(visionconnect::msg::Lanes, "lanes", 2, lanedet_pub);

	// start publishing video frames
    ROS_INFO("Lane Detection Node initialized, publishing lane lines...");
    ROS_SPIN();

    delete engine;

    return 0;
}
