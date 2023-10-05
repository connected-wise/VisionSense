#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "ros_compat.h"
#include "common.h"
#include "trtutil.h"

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#define INPUT_H 320
#define INPUT_W 320

// Declare publishers
Publisher<visionconnect::msg::Signs> classifier_pub = NULL;

// Globals
std::vector<std::string> classes;
Engine *engine;
std::vector<double> THRS;


void postprocess(std::vector<std::vector<std::vector<float>>> &featureVectors, std::vector<float> detclasses)
{
    visionconnect::msg::Signs classify_msg;

    int batch_size = featureVectors.size();
    classify_msg.labels.resize(batch_size);
    classify_msg.scores.resize(batch_size);

    // Find max confidence class
    for (int i = 0; i < batch_size; i++)
    {
        std::vector<float> output = featureVectors[i][0];
        int max_class = std::distance(output.begin(), std::max_element(output.begin(), output.end()));

         // Testing the output
        // std::cout << classes[max_class] <<" detected. Conf Score: " << output[max_class] << std::endl;
        
        classify_msg.labels[i] = classes[max_class]; //(detclasses[i] == 6) ? "traffic light" : "traffic sign";
        classify_msg.scores[i] = output[max_class];

        
        // if (detclasses[i] == 6 && output[max_class] > THRS[0] && (classes[max_class] == "red" || classes[max_class] == "yellow" || classes[max_class] == "green"))
        // {
        //     //std::cout << classes[max_class] <<" Light detected. Score: " << output[max_class] << std::endl;
        //     classify_msg.labels[i] = classes[max_class];

        // }
        // if (detclasses[i] == 7 && output[max_class] > THRS[1] && (classes[max_class] != "red" && classes[max_class] != "yellow" && classes[max_class] != "green"))
        // {
        //     //std::cout << classes[max_class] <<" detected. Score: " << output[max_class] << std::endl;
        //     classify_msg.labels[i] = classes[max_class];
        // }  

    }
    classifier_pub->publish(classify_msg);
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

    return featureVectors;
}

void signs_callback(visionconnect::msg::Signs::SharedPtr input)
{

    std::vector<cv::Mat> images; 

    for (const auto img_msg : input->images)
    {
        cv::Mat cpu_img;     
        auto cpu_img_msg = std::make_shared<sensor_msgs::msg::Image>(img_msg);
        convert_message_to_frame(cpu_img_msg, cpu_img);
        cv::imshow("Classifier Test",cpu_img);
        cv::waitKey(1);
        images.push_back(cpu_img);
    }
    // std::cout << "running classifier callback: " << images.size() << std::endl;
    
    if (images.size() == 0)
    {
        return;
    }
    auto output = run_engine(images);
    postprocess(output, input->classes);
    
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

    ROS_CREATE_NODE("classify");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
    std::string model_str, label_str, engine_path, label_path;

    ROS_DECLARE_PARAMETER("model", model_str);
    ROS_DECLARE_PARAMETER("labels", label_str);
    ROS_DECLARE_PARAMETER("thresholds", THRS);
    
    ROS_GET_PARAMETER("model", model_str);
    ROS_GET_PARAMETER("labels", label_str);
    ROS_GET_PARAMETER("thresholds", THRS);

    engine_path = package_share_directory + "/graphs/sign-classifier/" + model_str;
    label_path = package_share_directory + "/graphs/sign-classifier/" + label_str;

    load_engine(engine_path);
    classes = getClassNames(label_path);
    auto signs_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs_in", 1, signs_callback);

    ROS_CREATE_PUBLISHER(visionconnect::msg::Signs, "signs", 1, classifier_pub);

	// start publishing video frames
    ROS_INFO("Classifier Node initialized, publishing classifications...");
    ROS_SPIN();

    delete engine;

    return 0;
}
