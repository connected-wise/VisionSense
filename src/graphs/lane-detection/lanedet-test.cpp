#include <iostream>
#include "trtutil.h"

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

#define INPUT_H 208
#define INPUT_W 976
#define BATCH_SIZE 1
#define POINTS_COUNT 50


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
    cv::imshow("ScoreMap", scoreMap);
    cv::waitKey(0);


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
            std::cout << "Lane Point Count: " << coordSum << std::endl;

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
            // Apply if input is BGR image:
            std::cout << "crop = " << CROP << std::endl;
            std::cout << img.rows << std::endl;
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

    std::cout << "running lane engine.." << std::endl;
    return featureVectors[0];
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
    cv::Mat image = cv::imread("lanetest2.png");
    //cv::resize(image, image, cv::Size(1920, 1080));

    IMAGE_W = image.cols;
    IMAGE_H = image.rows;
    CROP = IMAGE_H/2; //IMAGE_W * (static_cast<float>(INPUT_H) / INPUT_W);


    std::string engine_path;

    engine_path = "lane_detect.engine";

    load_engine(engine_path);
    
    auto output = run_engine(image);
    
    std::cout << "size of output: " << output[1].size() << std::endl;
    for (float score : output[0]) {
        std::cout << "Lane Existence Score: " << score << std::endl;
    }

    //postprocess(output, image);

    std::vector<cv::Mat> lanes;
    auto laneVector = output[1];
    for (int c = 1; c < 5; ++c) {
        float* startPtr = &laneVector[c * INPUT_H * INPUT_W];
        cv::Mat lane(INPUT_H, INPUT_W, CV_32F, startPtr);
        cv::blur(lane, lane, cv::Size(11, 11));
        lanes.push_back(lane);
        cv::imshow("Lane" + std::to_string(c), lane);
    }

    std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 255, 0),
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 255, 255)
    };

    auto lines = GetLines(lanes, output[0], 0.25);

    for (const auto& line : lines) {
        std::cout << "Line coordinates: ";
        for (const auto& coord : line) {
            std::cout << "(" << coord.first << "," << coord.second << ") ";
        }
        std::cout << std::endl;
    }

    float scaleY = (IMAGE_H - CROP) / static_cast<float>(INPUT_H);
    float scaleX = IMAGE_W / static_cast<float>(INPUT_W);

    std::cout << "Scale Y: " << scaleY << " Scale X: " << scaleX << std::endl;

    for (int num = 0; num < 4; ++num) {
        if (!lines[num].empty()) {
            for (size_t i = 0; i < lines[num].size() - 1; ++i) {
                int x1 = lines[num][i].first;
                int y1 = lines[num][i].second;
                x1 = static_cast<int>(x1 * scaleX);
                y1 = static_cast<int>(y1 * scaleY + CROP);

                int x2 = lines[num][i + 1].first;
                int y2 = lines[num][i + 1].second;
                x2 = static_cast<int>(x2 * scaleX);
                y2 = static_cast<int>(y2 * scaleY + CROP);

                cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), colors[num], 7);
                cv::circle(image, cv::Point(x1, y1), 5, cv::Scalar(50,50,50), -1);
            }
        }
    }

    cv::imshow("LaneResult", image);
    cv::waitKey(0);

    delete engine;

    return 0;
}