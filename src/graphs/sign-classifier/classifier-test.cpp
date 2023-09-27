
#include <iostream>
#include <fstream>
#include "trtutil.h"
#include <filesystem>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

// Globals
std::vector<std::string> classes;
Engine *engine;


// Get label map
std::vector< std::string > getClassNames(const std::string& classes_txt)
{
    std::ifstream classes_file(classes_txt);
    std::vector< std::string > classes;
    if (!classes_file.good())
    {
        std::cerr << "ERROR: can't read file with classes names.n";
        return classes;
    }
    std::string class_name;
    while (std::getline(classes_file, class_name))
    {
        classes.push_back(class_name);
    }
    return classes;
}

void postprocess(std::vector<std::vector<std::vector<float>>> &featureVectors)
{
    std::vector<std::string> labels;
    int batch_size = featureVectors.size();
    // Find max confidence class
    for (int i = 0; i < batch_size; i++)
    {
        std::vector<float> output = featureVectors[i][0];
        int max_class = std::distance(output.begin(), std::max_element(output.begin(), output.end()));

         // Testing the output
        std::cout << classes[max_class] <<" detected. Score: " << output[max_class] << std::endl;

    }
    return;
}


std::vector<std::vector<std::vector<float>>> run_engine(std::vector<cv::Mat> images)
{

    int batch_size = images.size();

    const auto &inputDims = engine->getInputDims();
    std::vector<std::vector<cv::cuda::GpuMat>> inputs;

    for (const auto &inputDim : inputDims)
    { // For each of the model inputs...
        std::vector<cv::cuda::GpuMat> input;
        for (size_t j = 0; j < batch_size; ++j)
        { // For each element we want to add to the batch...
            cv::cuda::GpuMat gpu_img, resized;
            gpu_img.upload(images[j]);
            // Apply if input is BGR image:
            cv::cuda::cvtColor(gpu_img, gpu_img, cv::COLOR_BGR2RGB);
            //cv::cuda::resize(gpu_img, resized, cv::Size(inputDim.d[2], inputDim.d[1]));
            resized = engine->resizeKeepAspectRatioPadRightBottom(gpu_img, inputDim.d[2], inputDim.d[1], cv::COLOR_BGR2RGB); // TRT dims are (height, width) whereas OpenCV is (width, height)
            input.emplace_back(std::move(resized));
        }
        inputs.emplace_back(std::move(input));
    }

    std::vector<std::vector<std::vector<float>>> featureVectors; // Considers a batch output
    std::array<float, 3> subVals{0.f, 0.f, 0.f};
    std::array<float, 3> divVals{1.f, 1.f, 1.f};
    bool normalize = true;

    bool success = engine->runInference(inputs, featureVectors, subVals, divVals, normalize);
    if (!success)
    {
        throw std::runtime_error("Unable to run inference.");
    }

    std::cout << "running classifier engine.." << std::endl;
    return featureVectors;
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
    options.optBatchSize = 5;
    options.maxBatchSize = 20;

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

    std::string engine_path, label_path;
    std::vector<cv::Mat> imagestest;

    for (const auto & file : std::filesystem::directory_iterator("test")) {
        imagestest.push_back(cv::imread(file.path()));
        std::cout << file.path() << std::endl;
    }

    // imagestest.push_back(cv::imread("test1.jpg"));
    // imagestest.push_back(cv::imread("test2.jpg"));
    // imagestest.push_back(cv::imread("test3.jpg"));
    // imagestest.push_back(cv::imread("test4.jpg"));
    // imagestest.push_back(cv::imread("test5.jpg"));
    // imagestest.push_back(cv::imread("test6.jpg"));
    // imagestest.push_back(cv::imread("test7.png"));
    // imagestest.push_back(cv::imread("test8.png"));
    // imagestest.push_back(cv::imread("test9.jpg"));
    // imagestest.push_back(cv::imread("test10.jpg"));
    // imagestest.push_back(cv::imread("test11.jpg"));
    // imagestest.push_back(cv::imread("test12.jpg"));
    // imagestest.push_back(cv::imread("test13.jpg"));
    // imagestest.push_back(cv::imread("test14.jpg"));
    // imagestest.push_back(cv::imread("test15.jpg"));
    // imagestest.push_back(cv::imread("test16.jpg"));  

    engine_path = "classify.engine";
    label_path = "labels_classify.txt";

    load_engine(engine_path);

    classes = getClassNames(label_path);

    auto output = run_engine(imagestest);
    postprocess(output);

    delete engine;

    return 0;
}
