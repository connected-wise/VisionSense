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
#define BATCH_SIZE 1
#define MAX_PUBLISH_DETS 100
float m_ratio = 1;

// YOLO26 engine class IDs -> legacy class IDs used by downstream nodes
// (gui/bev/classify). bike is a new legacy slot (8).
//   0 car          -> 2 vehicle-car
//   1 truck        -> 4 vehicle-truck
//   2 bus          -> 3 vehicle-bus
//   3 train        -> 5 train
//   4 bike         -> 8 bike (NEW class slot)
//   5 cyclist      -> 1 cyclist
//   6 person       -> 0 pedestrian
//   7 traffic_light -> 6 traffic light
//   8 traffic_sign -> 7 traffic sign
static const int kYolo26ToLegacy[9] = {2, 4, 3, 5, 8, 1, 0, 6, 7};
Publisher<visionconnect::msg::Detect> detect_pub = NULL;
Publisher<visionconnect::msg::Signs> signs_pub = NULL;
Publisher<visionconnect::msg::Track> track_pub = NULL;

BYTETracker* tracker = nullptr;

std::vector<std::string> classes;
std::vector<double> THRS;
int track_frame_rate = 30;
int track_buffer = 30;
std::string current_image_encoding = "bgr8";  // Track input encoding for output

// Custom tracking variables
int frame_counter = 0;
const int FRAME_RESET_INTERVAL = 300; // Reset every 10 seconds instead of 1 second
std::map<int, int> class_counters; // class_id -> next_id_number
std::map<int, std::string> class_to_name = {
    {0, "pedest"},    // pedestrian
    {1, "cyclist"},   // cyclist
    {2, "car"},       // vehicle-car
    {3, "bus"},       // vehicle-bus
    {4, "truck"},     // vehicle-truck
    {8, "bike"}       // bike (rider-less)
};
std::set<int> tracked_classes = {0, 1, 2, 3, 4, 8}; // Only track these classes
std::map<int, std::string> track_id_to_custom_name;

std::vector<cv::Scalar> colors = {
    cv::Scalar(10, 230, 240), //pedestrian
    cv::Scalar(230, 240, 10), //cyclist
    cv::Scalar(190, 150, 25), //car
    cv::Scalar(240, 120, 10), //bus
    cv::Scalar(100, 10, 240), //truck
    cv::Scalar(240, 130, 10), //train
    cv::Scalar(160, 90, 160), //traffic light
    cv::Scalar(120, 55, 70),  //traffic sign
    cv::Scalar(180, 240, 90)  //bike
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
    visionconnect::msg::Box detect_box;
    visionconnect::msg::Detect detect_msg;
    visionconnect::msg::Signs classifier_msg;

    // Shrink and send input feed to output stream
    cv::Mat output;
    cv::resize(img, output, img.size() / 3);

    // YOLO26 end2end (NMS-free) output: [batch, max_dets, 6] where each row is
    // [x1, y1, x2, y2, score, class_id] in the letterboxed input image space.
    // The model already deduplicates via the one-to-one head, so no NMS here.
    const int numDetections = outputDims[0].d[1];   // 300
    const int numFeatures   = outputDims[0].d[2];   // 6

    const float m_imgHeight = static_cast<float>(img.rows);
    const float m_imgWidth  = static_cast<float>(img.cols);

    std::vector<cv::Rect> bboxes;
    std::vector<float>    scores;
    std::vector<int>      labels;
    bboxes.reserve(numDetections);
    scores.reserve(numDetections);
    labels.reserve(numDetections);

    for (int i = 0; i < numDetections; i++)
    {
        const float* row = featureVector.data() + i * numFeatures;
        const float score = row[4];
        if (score <= static_cast<float>(THRS[0])) continue;

        const int engine_cls = static_cast<int>(row[5]);
        if (engine_cls < 0 || engine_cls >= 9) continue;
        const int label = kYolo26ToLegacy[engine_cls];
        if (label < 0) continue;

        const float x0 = std::clamp(row[0] * m_ratio, 0.f, m_imgWidth);
        const float y0 = std::clamp(row[1] * m_ratio, 0.f, m_imgHeight);
        const float x1 = std::clamp(row[2] * m_ratio, 0.f, m_imgWidth);
        const float y1 = std::clamp(row[3] * m_ratio, 0.f, m_imgHeight);

        cv::Rect_<float> bbox;
        bbox.x = x0;
        bbox.y = y0;
        bbox.width  = (x1 - x0);
        bbox.height = (y1 - y0);
        if (bbox.width <= 0.f || bbox.height <= 0.f) continue;

        bboxes.push_back(bbox);
        labels.push_back(label);
        scores.push_back(score);

        // Detect/Track ROS msgs use fixed 100-slot arrays — cap here to avoid OOB.
        if (static_cast<int>(bboxes.size()) >= MAX_PUBLISH_DETS) break;
    }

    // Reset tracking counters if needed
    reset_tracking_if_needed();

    // Convert detections to Object format for tracking - only tracked classes
    std::vector<Object> objects;
    for (size_t k = 0; k < bboxes.size(); k++)
    {
        if (tracked_classes.find(labels[k]) != tracked_classes.end()) {
            Object obj;
            obj.rect = bboxes[k];
            obj.label = labels[k];
            obj.prob = scores[k];
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
    for (size_t k = 0; k < bboxes.size(); k++)
    {
        detect_msg.classes[i] = labels[k];
        detect_msg.scores[i]  = scores[k];

        // Find corresponding tracked object for this detection (only for tracked classes)
        std::string custom_track_id = "";
        if (tracked_classes.find(labels[k]) != tracked_classes.end()) {
            for (const auto& track : tracked_objects) {
                if (track.label == labels[k] &&
                    std::abs(track.tlbr[0] - bboxes[k].x) < 100 &&
                    std::abs(track.tlbr[1] - bboxes[k].y) < 100) {
                    custom_track_id = generate_custom_track_name(labels[k], track.track_id);
                    break;
                }
            }
        }
        detect_msg.track_list[i] = custom_track_id;

        detect_box.data = {
            static_cast<signed short>(bboxes[k].x),
            static_cast<signed short>(bboxes[k].y),
            static_cast<signed short>(bboxes[k].width),
            static_cast<signed short>(bboxes[k].height)
        };
        detect_msg.boxes[i] = detect_box;

        // Sign or Light detected -> forward crop to classifier
        if (labels[k] == 6 || labels[k] == 7)
        {
            sensor_msgs::msg::Image msgClassifyImg;
            convert_frame_to_message(img(bboxes[k]), msgClassifyImg);
            msgClassifyImg.encoding = current_image_encoding;
            classifier_msg.images.push_back(msgClassifyImg);
            classifier_msg.classes.push_back(labels[k]);
            classifier_msg.scores.push_back(scores[k]);
            classifier_msg.boxes.push_back(detect_box);
        }

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

    convert_frame_to_message(output, msg);
    msg.encoding = current_image_encoding;
    detect_msg.image = msg;

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

void run_engine(cv::Mat img, const std::string& encoding)
{
    const auto &inputDims = engine->getInputDims();
    std::vector<std::vector<cv::Mat>> inputs;
    inputs.reserve(inputDims.size());
    for (size_t k = 0; k < inputDims.size(); ++k) {
        std::vector<cv::Mat> input(BATCH_SIZE, img);
        inputs.emplace_back(std::move(input));
    }

    m_ratio = 1.f / std::min(inputDims[0].d[2] / static_cast<float>(img.cols), inputDims[0].d[1] / static_cast<float>(img.rows));

    std::vector<std::vector<std::vector<float>>> featureVectors;
    std::array<float, 3> subVals{0.f, 0.f, 0.f};
    std::array<float, 3> divVals{1.f, 1.f, 1.f};

    bool success = engine->runInference(inputs, featureVectors,
                                        subVals, divVals,
                                        /*normalize=*/true,
                                        /*swap_rb=*/(encoding == "bgr8"),
                                        /*aspect_ratio_pad=*/true);
    if (!success)
    {
        throw std::runtime_error("Unable to run inference.");
    }

    const auto outputDims = engine->getOutputDims();
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

    // Store input encoding so output uses same encoding
    current_image_encoding = input->encoding;
    run_engine(img, input->encoding);
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

    // Use BEST_EFFORT QoS to match camera publisher and prevent back-pressure
    rclcpp::QoS qos_best_effort(1);
    qos_best_effort.best_effort();
    qos_best_effort.durability_volatile();
    auto img_sub = node->create_subscription<sensor_msgs::Image>(
        "image_in", qos_best_effort, img_callback);

    ROS_CREATE_PUBLISHER(visionconnect::msg::Detect, "detections", 2, detect_pub);
    ROS_CREATE_PUBLISHER(visionconnect::msg::Signs, "signs", 2, signs_pub);
    ROS_CREATE_PUBLISHER(visionconnect::msg::Track, "tracks", 2, track_pub);

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
