#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "ros_compat.h"
#include "common.h"
#include "trtutil.h"

#include <opencv2/cudaimgproc.hpp>
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
int IMAGE_W, IMAGE_H, CROP;

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

std::vector<std::vector<float>> run_engine(cv::Mat img)
{
    const auto &inputDims = engine->getInputDims();
    std::vector<std::vector<cv::cuda::GpuMat>> inputs;

    for (const auto &inputDim : inputDims)
    { // For each of the model inputs...
        std::vector<cv::cuda::GpuMat> input;
        for (size_t j = 0; j < BATCH_SIZE; ++j)
        { // For each element we want to add to the batch...
            cv::cuda::GpuMat gpu_img, resized;
            gpu_img.upload(img);
            // cv::Rect(x, y, width, height) where x, y is the top left corner
            cv::cuda::GpuMat cropped(gpu_img, cv::Rect(0, img.rows/2, img.cols, CROP));
            cv::cuda::cvtColor(cropped, cropped, cv::COLOR_BGR2RGB);
            cv::cuda::resize(cropped, resized, cv::Size(inputDim.d[2], inputDim.d[1]));

            input.emplace_back(std::move(resized));
        }
        inputs.emplace_back(std::move(input));
    }

    std::vector<std::vector<std::vector<float>>> featureVectors; // Considers a batch output
    std::array<float, 3> subVals{125.f, 125.f, 125.f};
    std::array<float, 3> divVals{1.0f, 1.0f, 1.0f};
    bool normalize = false;

    bool success = engine->runInference(inputs, featureVectors, subVals, divVals, normalize);
    if (!success)
    {
        throw std::runtime_error("Unable to run inference.");
    }

    std::cout << "running lane detection engine.." << std::endl;
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

    auto laneVector = output[1];
    rawimg = cv::Mat(INPUT_H, INPUT_W, CV_32F, laneVector.data());
    rawimg.convertTo(rawimg, CV_8UC1, 255.0);
    
    sensor_msgs::msg::Image raw;
    raw.header.stamp = ROS_TIME_NOW();
    convert_frame_to_message(rawimg, raw);

    lanes_msg.rawimg = raw;

    for (int c = 1; c < 5; ++c) {
        float* startPtr = &laneVector[c * INPUT_H * INPUT_W];
        cv::Mat lane(INPUT_H, INPUT_W, CV_32F, startPtr);
        cv::blur(lane, lane, cv::Size(11, 11));
        lanes.push_back(lane);
        //cv::imshow("Lane" + std::to_string(c), lane);
    }
    // lines data is std::vector<std::vector<std::pair<int, int>>>
    auto lines = GetLines(lanes, output[0], 0.25);
    
    float scaleY = (IMAGE_H - CROP) / static_cast<float>(INPUT_H);
    float scaleX = IMAGE_W / static_cast<float>(INPUT_W);
    size_t lane_index = 0;
    for (const auto& line : lines) {
        for (const auto& point : line) {
            int x = point.first;
            int y = point.second;
            x = static_cast<int>(x * scaleX);
            y = static_cast<int>(y * scaleY + CROP);
            lanes_msg.xs.push_back(x);
            lanes_msg.ys.push_back(y);
        }
        // Add delimiters after each lane
        lanes_msg.xs.push_back(-1);
        lanes_msg.ys.push_back(-1);
        lanes_msg.probs[lane_index] = output[0][lane_index];
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
    CROP = IMAGE_H/2; 
	
    if(img.empty())
    {
        ROS_INFO("Failed to convert the input to Opencv image");
        return;	
    }

    auto output = run_engine(img);
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

    engine_path = package_share_directory + "/graphs/lane-detection/" + model_str;

    load_engine(engine_path);

    auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, img_callback);

    ROS_CREATE_PUBLISHER(visionconnect::msg::Lanes, "lanes", 10, lanedet_pub);

	// start publishing video frames
    ROS_INFO("Lane Detection Node initialized, publishing lane lines...");
    ROS_SPIN();

    delete engine;

    return 0;
}
