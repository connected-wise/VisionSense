/*
 * Driver Monitoring Node - TensorRT Implementation
 *
 * Uses TensorRT-accelerated face detection and gaze estimation
 * to monitor driver attention and detect distraction/drowsiness.
 */

#include "ros_compat.h"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <memory>
#include <cmath>

// Driver states
enum DriverState { ALERT = 0, DISTRACTED = 1, DROWSY = 2, NO_DRIVER = 3 };

// TensorRT Logger
class TRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            ROS_WARN("[TensorRT] %s", msg);
    }
} gLogger;

// Face detection result
struct FaceBox {
    int x1, y1, x2, y2;
    float confidence;
};

// ============================================================================
// Face Detector (YOLOv11)
// ============================================================================
class FaceDetector {
public:
    static constexpr int INPUT_H = 640;
    static constexpr int INPUT_W = 640;
    static constexpr int NUM_DETECTIONS = 8400;

    FaceDetector(const std::string& engine_path, float conf_thresh = 0.5f, float nms_thresh = 0.45f)
        : conf_thresh_(conf_thresh), nms_thresh_(nms_thresh) {

        std::ifstream file(engine_path, std::ios::binary);
        if (!file.good()) throw std::runtime_error("Failed to open: " + engine_path);

        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> data(size);
        file.read(data.data(), size);

        runtime_.reset(nvinfer1::createInferRuntime(gLogger));
        engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
        context_.reset(engine_->createExecutionContext());

        input_host_.resize(3 * INPUT_H * INPUT_W);
        output_host_.resize(5 * NUM_DETECTIONS);

        cudaMalloc(&buffers_[0], input_host_.size() * sizeof(float));
        cudaMalloc(&buffers_[1], output_host_.size() * sizeof(float));
        cudaStreamCreate(&stream_);

        context_->setTensorAddress("images", buffers_[0]);
        context_->setTensorAddress("output0", buffers_[1]);
    }

    ~FaceDetector() {
        if (stream_) cudaStreamDestroy(stream_);
        for (auto& buf : buffers_) if (buf) cudaFree(buf);
    }

    std::vector<FaceBox> detect(const cv::Mat& image) {
        if (image.empty()) return {};

        int orig_w = image.cols, orig_h = image.rows;
        preprocess(image);

        cudaMemcpyAsync(buffers_[0], input_host_.data(),
                        input_host_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        context_->enqueueV3(stream_);
        cudaMemcpyAsync(output_host_.data(), buffers_[1],
                        output_host_.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        return postprocess(orig_w, orig_h);
    }

private:
    void preprocess(const cv::Mat& image) {
        float scale = std::min((float)INPUT_W / image.cols, (float)INPUT_H / image.rows);
        int new_w = image.cols * scale, new_h = image.rows * scale;
        scale_ = scale;
        pad_x_ = (INPUT_W - new_w) / 2;
        pad_y_ = (INPUT_H - new_h) / 2;

        cv::Mat resized, padded(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
        cv::resize(image, resized, cv::Size(new_w, new_h));
        resized.copyTo(padded(cv::Rect(pad_x_, pad_y_, new_w, new_h)));

        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);

        for (int c = 0; c < 3; c++)
            for (int h = 0; h < INPUT_H; h++)
                for (int w = 0; w < INPUT_W; w++)
                    input_host_[c * INPUT_H * INPUT_W + h * INPUT_W + w] =
                        rgb.at<cv::Vec3b>(h, w)[c] / 255.0f;
    }

    std::vector<FaceBox> postprocess(int orig_w, int orig_h) {
        std::vector<FaceBox> boxes;
        for (int i = 0; i < NUM_DETECTIONS; i++) {
            float conf = output_host_[4 * NUM_DETECTIONS + i];
            if (conf < conf_thresh_) continue;

            float cx = output_host_[0 * NUM_DETECTIONS + i];
            float cy = output_host_[1 * NUM_DETECTIONS + i];
            float w = output_host_[2 * NUM_DETECTIONS + i];
            float h = output_host_[3 * NUM_DETECTIONS + i];

            float x1 = (cx - w / 2 - pad_x_) / scale_;
            float y1 = (cy - h / 2 - pad_y_) / scale_;
            float x2 = (cx + w / 2 - pad_x_) / scale_;
            float y2 = (cy + h / 2 - pad_y_) / scale_;

            FaceBox box;
            box.x1 = std::max(0, std::min(orig_w - 1, (int)x1));
            box.y1 = std::max(0, std::min(orig_h - 1, (int)y1));
            box.x2 = std::max(0, std::min(orig_w - 1, (int)x2));
            box.y2 = std::max(0, std::min(orig_h - 1, (int)y2));
            box.confidence = conf;

            if (box.x2 - box.x1 > 10 && box.y2 - box.y1 > 10)
                boxes.push_back(box);
        }
        nms(boxes);
        return boxes;
    }

    void nms(std::vector<FaceBox>& boxes) {
        std::sort(boxes.begin(), boxes.end(), [](const FaceBox& a, const FaceBox& b) {
            return a.confidence > b.confidence;
        });
        std::vector<bool> suppressed(boxes.size(), false);
        std::vector<FaceBox> result;
        for (size_t i = 0; i < boxes.size(); i++) {
            if (suppressed[i]) continue;
            result.push_back(boxes[i]);
            for (size_t j = i + 1; j < boxes.size(); j++) {
                if (!suppressed[j] && iou(boxes[i], boxes[j]) > nms_thresh_)
                    suppressed[j] = true;
            }
        }
        boxes = std::move(result);
    }

    float iou(const FaceBox& a, const FaceBox& b) {
        int x1 = std::max(a.x1, b.x1), y1 = std::max(a.y1, b.y1);
        int x2 = std::min(a.x2, b.x2), y2 = std::min(a.y2, b.y2);
        float inter = std::max(0, x2 - x1) * std::max(0, y2 - y1);
        float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
        float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
        return inter / (area_a + area_b - inter + 1e-6f);
    }

    float conf_thresh_, nms_thresh_, scale_;
    int pad_x_, pad_y_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    void* buffers_[2] = {nullptr, nullptr};
    cudaStream_t stream_ = nullptr;
    std::vector<float> input_host_, output_host_;
};

// ============================================================================
// Gaze Estimator (ResNet18)
// ============================================================================
class GazeEstimator {
public:
    static constexpr int INPUT_H = 448;
    static constexpr int INPUT_W = 448;
    static constexpr int NUM_BINS = 90;
    static constexpr float BIN_WIDTH = 4.0f;
    static constexpr float ANGLE_OFFSET = 180.0f;
    static constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float STD[3] = {0.229f, 0.224f, 0.225f};

    GazeEstimator(const std::string& engine_path) {
        std::ifstream file(engine_path, std::ios::binary);
        if (!file.good()) throw std::runtime_error("Failed to open: " + engine_path);

        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> data(size);
        file.read(data.data(), size);

        runtime_.reset(nvinfer1::createInferRuntime(gLogger));
        engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
        context_.reset(engine_->createExecutionContext());

        input_host_.resize(3 * INPUT_H * INPUT_W);
        pitch_host_.resize(NUM_BINS);
        yaw_host_.resize(NUM_BINS);

        cudaMalloc(&buffers_[0], input_host_.size() * sizeof(float));
        cudaMalloc(&buffers_[1], NUM_BINS * sizeof(float));
        cudaMalloc(&buffers_[2], NUM_BINS * sizeof(float));
        cudaStreamCreate(&stream_);

        context_->setTensorAddress("input", buffers_[0]);
        context_->setTensorAddress("pitch", buffers_[1]);
        context_->setTensorAddress("yaw", buffers_[2]);
    }

    ~GazeEstimator() {
        if (stream_) cudaStreamDestroy(stream_);
        for (auto& buf : buffers_) if (buf) cudaFree(buf);
    }

    bool infer(const cv::Mat& face_crop, float& pitch, float& yaw) {
        if (face_crop.empty()) return false;

        preprocess(face_crop);

        cudaMemcpyAsync(buffers_[0], input_host_.data(),
                        input_host_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        if (!context_->enqueueV3(stream_)) return false;
        cudaMemcpyAsync(pitch_host_.data(), buffers_[1], NUM_BINS * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaMemcpyAsync(yaw_host_.data(), buffers_[2], NUM_BINS * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        pitch = decodeAngle(pitch_host_.data());
        yaw = decodeAngle(yaw_host_.data());
        return true;
    }

private:
    void preprocess(const cv::Mat& image) {
        cv::Mat resized, rgb;
        cv::resize(image, resized, cv::Size(INPUT_W, INPUT_H));
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

        for (int c = 0; c < 3; c++)
            for (int h = 0; h < INPUT_H; h++)
                for (int w = 0; w < INPUT_W; w++)
                    input_host_[c * INPUT_H * INPUT_W + h * INPUT_W + w] =
                        (rgb.at<cv::Vec3b>(h, w)[c] / 255.0f - MEAN[c]) / STD[c];
    }

    float decodeAngle(const float* logits) {
        float max_val = *std::max_element(logits, logits + NUM_BINS);
        float sum = 0.0f;
        std::vector<float> probs(NUM_BINS);
        for (int i = 0; i < NUM_BINS; i++) {
            probs[i] = std::exp(logits[i] - max_val);
            sum += probs[i];
        }
        float angle = 0.0f;
        for (int i = 0; i < NUM_BINS; i++)
            angle += (probs[i] / sum) * i;
        return (angle * BIN_WIDTH - ANGLE_OFFSET) * M_PI / 180.0f;
    }

    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    void* buffers_[3] = {nullptr, nullptr, nullptr};
    cudaStream_t stream_ = nullptr;
    std::vector<float> input_host_, pitch_host_, yaw_host_;
};

constexpr float GazeEstimator::MEAN[3];
constexpr float GazeEstimator::STD[3];

// ============================================================================
// ROS2 Node
// ============================================================================
Publisher<sensor_msgs::Image> image_pub = NULL;
Publisher<std_msgs::msg::String> state_pub = NULL;
Publisher<std_msgs::msg::Bool> alert_pub = NULL;

std::unique_ptr<FaceDetector> face_detector;
std::unique_ptr<GazeEstimator> gaze_estimator;

// Driver state tracking
int frames_no_face = 0;
int frames_looking_away = 0;
const int NO_FACE_THRESHOLD = 30;      // ~1 second at 30fps
const int LOOKING_AWAY_THRESHOLD = 60; // ~2 seconds at 30fps
const float YAW_THRESHOLD = 0.52f;     // ~30 degrees in radians
const float PITCH_THRESHOLD = 0.52f;   // ~30 degrees

const char* getStateName(DriverState state) {
    switch (state) {
        case ALERT: return "ALERT";
        case DISTRACTED: return "DISTRACTED";
        case DROWSY: return "DROWSY";
        case NO_DRIVER: return "NO_DRIVER";
        default: return "UNKNOWN";
    }
}

void drawGaze(cv::Mat& image, const cv::Point& center, float pitch, float yaw, int length = 100) {
    float dx = -length * std::sin(pitch) * std::cos(yaw);
    float dy = -length * std::sin(yaw);
    cv::Point end(center.x + (int)dx, center.y + (int)dy);
    cv::arrowedLine(image, center, end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);
    cv::circle(image, center, 4, cv::Scalar(0, 0, 255), -1);
}

void image_callback(const sensor_msgs::Image::SharedPtr msg) {
    // Convert ROS image to OpenCV
    cv::Mat frame;
    if (msg->encoding == "rgb8") {
        frame = cv::Mat(msg->height, msg->width, CV_8UC3,
                        const_cast<uint8_t*>(msg->data.data()), msg->step).clone();
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "bgr8") {
        frame = cv::Mat(msg->height, msg->width, CV_8UC3,
                        const_cast<uint8_t*>(msg->data.data()), msg->step).clone();
    } else {
        ROS_WARN("Unsupported encoding: %s", msg->encoding.c_str());
        return;
    }

    DriverState state = ALERT;
    cv::Mat output = frame.clone();

    // Detect faces
    auto faces = face_detector->detect(frame);

    if (faces.empty()) {
        frames_no_face++;
        frames_looking_away = 0;
        if (frames_no_face > NO_FACE_THRESHOLD)
            state = NO_DRIVER;
        cv::putText(output, "NO FACE", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    } else {
        frames_no_face = 0;

        // Get largest face (driver)
        FaceBox& main_face = faces[0];
        for (auto& f : faces)
            if ((f.x2 - f.x1) * (f.y2 - f.y1) > (main_face.x2 - main_face.x1) * (main_face.y2 - main_face.y1))
                main_face = f;

        // Extract face ROI with padding
        int pad = 10;
        int x1 = std::max(0, main_face.x1 - pad);
        int y1 = std::max(0, main_face.y1 - pad);
        int x2 = std::min(frame.cols - 1, main_face.x2 + pad);
        int y2 = std::min(frame.rows - 1, main_face.y2 + pad);
        cv::Rect roi(x1, y1, x2 - x1, y2 - y1);

        // Run gaze estimation
        float pitch, yaw;
        if (gaze_estimator->infer(frame(roi), pitch, yaw)) {
            // Check if looking away
            if (std::abs(yaw) > YAW_THRESHOLD || std::abs(pitch) > PITCH_THRESHOLD) {
                frames_looking_away++;
                if (frames_looking_away > LOOKING_AWAY_THRESHOLD)
                    state = DISTRACTED;
            } else {
                frames_looking_away = 0;
            }

            // Draw visualization
            cv::rectangle(output, roi, cv::Scalar(255, 0, 0), 2);
            cv::Point center((x1 + x2) / 2, (y1 + y2) / 2);
            int arrow_len = (x2 - x1) / 2 + 30;
            drawGaze(output, center, pitch, yaw, arrow_len);

            // Display angles
            char text[64];
            snprintf(text, sizeof(text), "P:%.0f Y:%.0f", pitch * 180 / M_PI, yaw * 180 / M_PI);
            cv::putText(output, text, cv::Point(x1, y2 + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
    }

    // Draw state
    cv::Scalar color = (state == ALERT) ? cv::Scalar(0, 255, 0) :
                       (state == DISTRACTED) ? cv::Scalar(0, 165, 255) :
                       (state == DROWSY) ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 128, 128);
    cv::putText(output, getStateName(state), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);

    // Publish state
    std_msgs::msg::String state_msg;
    state_msg.data = getStateName(state);
    state_pub->publish(state_msg);

    // Publish alert
    std_msgs::msg::Bool alert_msg;
    alert_msg.data = (state == DISTRACTED || state == DROWSY || state == NO_DRIVER);
    alert_pub->publish(alert_msg);

    // Publish annotated image
    cv::cvtColor(output, output, cv::COLOR_BGR2RGB);
    sensor_msgs::Image img_msg;
    img_msg.header.stamp = msg->header.stamp;
    img_msg.header.frame_id = msg->header.frame_id;
    img_msg.height = output.rows;
    img_msg.width = output.cols;
    img_msg.encoding = "rgb8";
    img_msg.step = output.cols * 3;
    img_msg.data.assign(output.data, output.data + img_msg.step * img_msg.height);
    image_pub->publish(img_msg);
}

int main(int argc, char** argv) {
    ROS_CREATE_NODE("driver_monitor");
    ROS_INFO("Driver Monitoring Node - TensorRT");

    // Get parameters
    std::string face_engine, gaze_engine, camera_topic;
    float conf_thresh = 0.5f;

    ROS_DECLARE_PARAMETER("face_engine", face_engine);
    ROS_DECLARE_PARAMETER("gaze_engine", gaze_engine);
    ROS_DECLARE_PARAMETER("camera_topic", camera_topic);
    ROS_DECLARE_PARAMETER("confidence", conf_thresh);

    ROS_GET_PARAMETER("face_engine", face_engine);
    ROS_GET_PARAMETER("gaze_engine", gaze_engine);
    ROS_GET_PARAMETER("camera_topic", camera_topic);
    ROS_GET_PARAMETER("confidence", conf_thresh);

    // Defaults
    if (face_engine.empty())
        face_engine = "/home/jetson/gaze-estimation/weights/yolov11n_face_fp16.engine";
    if (gaze_engine.empty())
        gaze_engine = "/home/jetson/gaze-estimation/weights/resnet18_gaze_fp16.engine";
    if (camera_topic.empty())
        camera_topic = "/camera/raw";

    // Load models
    ROS_INFO("Loading face detector: %s", face_engine.c_str());
    face_detector = std::make_unique<FaceDetector>(face_engine, conf_thresh);

    ROS_INFO("Loading gaze estimator: %s", gaze_engine.c_str());
    gaze_estimator = std::make_unique<GazeEstimator>(gaze_engine);

    // Publishers
    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "image", 2, image_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "state", 10, state_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::Bool, "alert", 10, alert_pub);

    // Subscriber
    ROS_INFO("Subscribing to: %s", camera_topic.c_str());
    auto image_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, camera_topic.c_str(), 2, image_callback);

    ROS_INFO("Driver monitoring ready");

    ROS_SPIN();

    face_detector.reset();
    gaze_estimator.reset();

    return 0;
}
