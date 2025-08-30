#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "image_converter.h"
#include "trtutil.h"
#include "BYTETracker.h"
#include <map>
#include <set>
#include <string>
#include <algorithm>

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/track.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

Engine *engine;
#define NMS_THRESH 0.5
#define BATCH_SIZE 1
float m_ratio = 1;
Publisher<visionconnect::msg::Detect> detect_pub = NULL;
Publisher<visionconnect::msg::Signs> signs_pub = NULL;
Publisher<visionconnect::msg::Track> track_pub = NULL;

BYTETracker* tracker = nullptr;

std::vector<std::string> classes;
std::vector<double> THRS;
int track_frame_rate = 30;
int track_buffer = 30;

// Custom tracking variables
int frame_counter = 0;
const int FRAME_RESET_INTERVAL = 300; // Reset every 10 seconds instead of 1 second
std::map<int, int> class_counters; // class_id -> next_id_number
std::map<int, std::string> class_to_name = {
    {0, "pedest"},    // pedestrian
    {1, "cyclist"},   // cyclist
    {2, "car"},       // vehicle-car
    {3, "bus"},       // vehicle-bus
    {4, "truck"}      // vehicle-truck
};
std::set<int> tracked_classes = {0, 1, 2, 3, 4}; // Only track these classes
std::map<int, std::string> track_id_to_custom_name;

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

// Helper function to generate custom track name
std::string generate_custom_track_name(int class_id, int track_id) {
    if (class_to_name.find(class_id) == class_to_name.end()) {
        return "unknown" + std::to_string(track_id);
    }
    
    // Check if we already have a custom name for this track_id
    if (track_id_to_custom_name.find(track_id) != track_id_to_custom_name.end()) {
        return track_id_to_custom_name[track_id];
    }
    
    // Generate new custom name
    if (class_counters.find(class_id) == class_counters.end()) {
        class_counters[class_id] = 1;
    }
    
    std::string custom_name = class_to_name[class_id] + std::to_string(class_counters[class_id]);
    class_counters[class_id]++;
    
    // Store the mapping
    track_id_to_custom_name[track_id] = custom_name;
    
    return custom_name;
}

// Helper function to reset tracking counters every 30 frames
void reset_tracking_if_needed() {
    frame_counter++;
    if (frame_counter >= FRAME_RESET_INTERVAL) {
        frame_counter = 0;
        class_counters.clear();
        track_id_to_custom_name.clear();
        ROS_INFO("Reset tracking counters after %d frames", FRAME_RESET_INTERVAL);
    }
}


void postprocess(std::vector<float> &featureVector, const std::vector<nvinfer1::Dims> &outputDims, cv::Mat &img)
{
    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;
    visionconnect::msg::Box detect_box;
    visionconnect::msg::Detect detect_msg;
    visionconnect::msg::Signs classifier_msg;
    
    // Shrink and send input feed to output stream
    cv::Mat output;
    cv::resize(img, output, img.size() / 3);

    std::string labelTxt;
    auto numChannels = outputDims[0].d[1];
    auto numAnchors = outputDims[0].d[2];
    auto numClasses = classes.size();

    float m_imgHeight = img.rows;
    float m_imgWidth = img.cols;

    cv::Mat out = cv::Mat(numChannels, numAnchors, CV_32F, featureVector.data());
    out = out.t();
    int counter = 0;

    for (int i = 0; i < numAnchors; i++)
    {

        auto rowPtr = out.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto maxSPtr = std::max_element(scoresPtr, scoresPtr + numClasses);
        float score = *maxSPtr;
        if (score > THRS[0])
        {
            counter++;

            if (counter > 99)
                continue;
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y0 = std::clamp((y - 0.5f * h) * m_ratio, 0.f, m_imgHeight);
            float x1 = std::clamp((x + 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y1 = std::clamp((y + 0.5f * h) * m_ratio, 0.f, m_imgHeight);

            int label = maxSPtr - scoresPtr;

            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = (x1 - x0);
            bbox.height = (y1 - y0);

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
        }
    }
    // Custom NMS implementation to avoid OpenCV DNN dependency
    // Group boxes by class and apply NMS per class
    std::map<int, std::vector<int>> class_indices;
    for (size_t i = 0; i < labels.size(); i++) {
        class_indices[labels[i]].push_back(i);
    }
    
    // Custom NMS function
    auto computeIOU = [](const cv::Rect& a, const cv::Rect& b) -> float {
        float xA = std::max(a.x, b.x);
        float yA = std::max(a.y, b.y);
        float xB = std::min(a.x + a.width, b.x + b.width);
        float yB = std::min(a.y + a.height, b.y + b.height);
        
        float interArea = std::max(0.0f, xB - xA) * std::max(0.0f, yB - yA);
        float boxAArea = a.width * a.height;
        float boxBArea = b.width * b.height;
        
        return interArea / (boxAArea + boxBArea - interArea);
    };
    
    for (auto& [class_id, class_idx] : class_indices) {
        // Sort by score
        std::sort(class_idx.begin(), class_idx.end(), 
                  [&scores](int a, int b) { return scores[a] > scores[b]; });
        
        std::vector<bool> suppressed(class_idx.size(), false);
        
        for (size_t i = 0; i < class_idx.size(); i++) {
            if (suppressed[i]) continue;
            
            indices.push_back(class_idx[i]);
            
            for (size_t j = i + 1; j < class_idx.size(); j++) {
                if (suppressed[j]) continue;
                
                float iou = computeIOU(bboxes[class_idx[i]], bboxes[class_idx[j]]);
                if (iou > NMS_THRESH) {
                    suppressed[j] = true;
                }
            }
        }
    }

    // Reset tracking counters if needed
    reset_tracking_if_needed();
    
    // Convert detections to Object format for tracking - only tracked classes
    std::vector<Object> objects;
    for (auto &index : indices)
    {
        // Only track specified object classes
        if (tracked_classes.find(labels[index]) != tracked_classes.end()) {
            Object obj;
            obj.rect = bboxes[index];
            obj.label = labels[index];
            obj.prob = scores[index];
            obj.tracker_id = -1;
            objects.push_back(obj);
        }
    }
    
    // Run tracking
    std::vector<STrack> tracked_objects;
    if (tracker) {
        tracked_objects = tracker->update(objects);
    }
    
    // Prepare track message
    visionconnect::msg::Track track_msg;
    track_msg.num_tracked = 0;
    
    // Initialize track_ids array with empty strings
    for (int k = 0; k < 100; k++) {
        track_msg.track_ids[k] = "";
        detect_msg.track_list[k] = "";
    }
    
    int i = 0;
    for (auto &index : indices)
    {
        // Copy raw detect to ROS message
        detect_msg.classes[i] = labels[index];
        detect_msg.scores[i] = scores[index];
        
        // Find corresponding tracked object for this detection (only for tracked classes)
        std::string custom_track_id = "";
        if (tracked_classes.find(labels[index]) != tracked_classes.end()) {
            for (const auto& track : tracked_objects) {
                if (track.label == labels[index] && 
                    std::abs(track.tlbr[0] - bboxes[index].x) < 100 &&
                    std::abs(track.tlbr[1] - bboxes[index].y) < 100) {
                    custom_track_id = generate_custom_track_name(labels[index], track.track_id);
                    break;
                }
            }
        }
        
        detect_msg.track_list[i] = custom_track_id;

        detect_box.data = {
            static_cast<signed short>(bboxes[index].x),
            static_cast<signed short>(bboxes[index].y),
            static_cast<signed short>(bboxes[index].width),
            static_cast<signed short>(bboxes[index].height)
        };
        detect_msg.boxes[i] = detect_box;

        // Sign or Light detected
        if (labels[index] == 6 || labels[index] == 7)
        {
            // Crop and convert through ROS
            sensor_msgs::msg::Image msgClassifyImg;

            convert_frame_to_message(img(bboxes[index]), msgClassifyImg);
            classifier_msg.images.push_back(msgClassifyImg);
            classifier_msg.classes.push_back(labels[index]);
            classifier_msg.scores.push_back(scores[index]);
            classifier_msg.boxes.push_back(detect_box);
        }

        // Redo box and draw for scaled down image
        cv::Rect_<float> bbox = cv::Rect(bboxes[index].x / 3, bboxes[index].y / 3, bboxes[index].width / 3, bboxes[index].height / 3);
        cv::rectangle(output, bbox, colors[(int)labels[index]], 4);
        
        // Track ID drawing disabled - only GUI node draws labels
        // if (!custom_track_id.empty()) {
        //     cv::putText(output, custom_track_id, 
        //                cv::Point(bbox.x, bbox.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[(int)labels[index]], 1);
        // }

        i++;
    }
    
    // Fill track message with current tracked objects (only tracked classes)
    int track_idx = 0;
    for (const auto& track : tracked_objects) {
        if (track.is_activated && track_idx < 100 && 
            tracked_classes.find(track.label) != tracked_classes.end()) {
            track_msg.classes[track_idx] = track.label;
            track_msg.track_ids[track_idx] = generate_custom_track_name(track.label, track.track_id);
            
            visionconnect::msg::Box track_box;
            track_box.data = {
                static_cast<signed short>(track.tlbr[0]),
                static_cast<signed short>(track.tlbr[1]),
                static_cast<signed short>(track.tlbr[2] - track.tlbr[0]),
                static_cast<signed short>(track.tlbr[3] - track.tlbr[1])
            };
            track_msg.boxes[track_idx] = track_box;
            track_idx++;
        }
    }
    track_msg.num_tracked = track_idx;

    detect_msg.num_detections = i;
    sensor_msgs::msg::Image msg;
    msg.header.stamp = ROS_TIME_NOW();

    // Convert output image to ROS message
    convert_frame_to_message(output, msg);
    detect_msg.image = msg;

    // Publish the messages
    detect_pub->publish(detect_msg);
    signs_pub->publish(classifier_msg);
    track_pub->publish(track_msg);
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

void run_engine(cv::Mat img)
{
    //cv::imshow("detection", img);
    //cv::waitKey(1);

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
            cv::cuda::cvtColor(gpu_img, gpu_img, cv::COLOR_BGR2RGB);
            resized = engine->resizeKeepAspectRatioPadRightBottom(gpu_img, inputDim.d[2], inputDim.d[1], cv::COLOR_BGR2RGB); // TRT dims are (height, width) whereas OpenCV is (width, height)
            input.emplace_back(std::move(resized));
        }
        inputs.emplace_back(std::move(input));
    }

    m_ratio = 1.f / std::min(inputDims[0].d[2] / static_cast<float>(img.cols), inputDims[0].d[1] / static_cast<float>(img.rows));

    std::vector<std::vector<std::vector<float>>> featureVectors; // Considers a batch output
    std::array<float, 3> subVals{0.f, 0.f, 0.f};
    std::array<float, 3> divVals{1.f, 1.f, 1.f};
    bool normalize = true;

    bool success = engine->runInference(inputs, featureVectors, subVals, divVals, normalize);
    if (!success)
    {
        throw std::runtime_error("Unable to run inference.");
    }

    // std::cout << "size of output: " << featureVectors[0][0].size() << std::endl;
    const auto outputDims = engine->getOutputDims();

    std::cout << "running detection engine.." << std::endl;

    postprocess(featureVectors[0][0], outputDims, img);
}

// input image subscriber callback
void img_callback(const sensor_msgs::msg::Image::SharedPtr input)
{
    
    cv::Mat img;
    convert_message_to_frame(input, img);
	
    if(img.empty())
    {
	ROS_INFO("Failed to convert the input to Opencv image");
	return;	
    }

    run_engine(img);
}


// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("detect");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
    std::string model_str, label_str, engine_path, label_path;

    ROS_DECLARE_PARAMETER("model", model_str);
    ROS_DECLARE_PARAMETER("labels", label_str);
    ROS_DECLARE_PARAMETER("thresholds", THRS);
    ROS_DECLARE_PARAMETER("track_frame_rate", track_frame_rate);
    ROS_DECLARE_PARAMETER("track_buffer", track_buffer);
    
    ROS_GET_PARAMETER("model", model_str);
    ROS_GET_PARAMETER("labels", label_str);
    ROS_GET_PARAMETER("thresholds", THRS);
    ROS_GET_PARAMETER("track_frame_rate", track_frame_rate);
    ROS_GET_PARAMETER("track_buffer", track_buffer);

    engine_path = package_share_directory + "/graphs/object-detection/" + model_str;
    label_path = package_share_directory + "/graphs/object-detection/" + label_str;

    load_engine(engine_path);
    classes = getClassNames(label_path);
    
    // Initialize tracker
    tracker = new BYTETracker(track_frame_rate, track_buffer);
    
    auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, img_callback);

    ROS_CREATE_PUBLISHER(visionconnect::msg::Detect, "detections", 10, detect_pub);
    ROS_CREATE_PUBLISHER(visionconnect::msg::Signs, "signs", 10, signs_pub);
    ROS_CREATE_PUBLISHER(visionconnect::msg::Track, "tracks", 10, track_pub);

	// start publishing video frames
    ROS_INFO("Detect Node initialized, publishing detections...");
    ROS_SPIN();

    delete engine;
    delete tracker;
    detect_pub.reset();
    signs_pub.reset();
    track_pub.reset();

    return 0;
}
