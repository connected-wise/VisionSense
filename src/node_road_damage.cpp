/*
 * Road Damage Node - TensorRT Implementation
 *
 * Segments potholes in the forward mono camera image with a YOLO
 * instance-segmentation model, for fleet data collection (3F0R layout, where
 * it takes the driver monitor's place). Publishes the annotated image, a
 * state string for the dashboard, and logs geotagged events to disk.
 */

#include "ros_compat.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

#include <sys/stat.h>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <memory>
#include <mutex>
#include <cmath>
#include <chrono>
#include <cstring>
#include <ctime>

// TensorRT Logger
class TRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            ROS_WARN("[TensorRT] %s", msg);
    }
} gLogger;

// One segmented instance, in original image coordinates
struct Damage {
    int x1, y1, x2, y2;
    float confidence;
    int class_id;
    std::vector<float> coeffs;  // mask coefficients, one per prototype
    float area_pct = 0.0f;      // mask area as % of the image
};

// ============================================================================
// Damage Segmenter (YOLOv8-seg / YOLO11-seg export)
//
//   input   [1, 3, H, W]             RGB, 0..1, letterboxed
//   output  [1, 4 + nc + nm, N]      cx, cy, w, h, nc class scores, nm coeffs
//   proto   [1, nm, H/4, W/4]        mask prototypes
//
// Tensor names and sizes are read from the engine, so any class count and
// input size works.
// ============================================================================
class DamageSegmenter {
public:
    DamageSegmenter(const std::string& engine_path, float conf_thresh, float nms_thresh)
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
        if (!engine_) throw std::runtime_error("Failed to deserialize: " + engine_path);
        context_.reset(engine_->createExecutionContext());

        // Find the input, the detection output (3 dims) and the prototypes (4 dims)
        std::string in_name, det_name, proto_name;
        nvinfer1::Dims in_dims{}, det_dims{}, proto_dims{};
        for (int i = 0; i < engine_->getNbIOTensors(); i++) {
            const char* name = engine_->getIOTensorName(i);
            nvinfer1::Dims dims = engine_->getTensorShape(name);
            if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
                in_name = name; in_dims = dims;
            } else if (dims.nbDims == 3) {
                det_name = name; det_dims = dims;
            } else if (dims.nbDims == 4) {
                proto_name = name; proto_dims = dims;
            }
        }
        if (in_name.empty() || det_name.empty() || proto_name.empty() || in_dims.nbDims != 4)
            throw std::runtime_error("Not a YOLO segmentation engine: " + engine_path);

        input_h_ = in_dims.d[2];
        input_w_ = in_dims.d[3];
        num_masks_ = proto_dims.d[1];
        proto_h_ = proto_dims.d[2];
        proto_w_ = proto_dims.d[3];
        num_channels_ = det_dims.d[1];
        num_anchors_ = det_dims.d[2];
        num_classes_ = num_channels_ - 4 - num_masks_;
        if (num_classes_ < 1)
            throw std::runtime_error("Unexpected output layout in: " + engine_path);

        ROS_INFO("Segmenter: input %dx%d, %d class(es), %d anchors, %d prototypes at %dx%d",
                 input_w_, input_h_, num_classes_, num_anchors_, num_masks_, proto_w_, proto_h_);

        input_host_.resize(3 * input_h_ * input_w_);
        det_host_.resize(num_channels_ * num_anchors_);
        proto_host_.resize(num_masks_ * proto_h_ * proto_w_);

        cudaMalloc(&buffers_[0], input_host_.size() * sizeof(float));
        cudaMalloc(&buffers_[1], det_host_.size() * sizeof(float));
        cudaMalloc(&buffers_[2], proto_host_.size() * sizeof(float));
        cudaStreamCreate(&stream_);

        context_->setTensorAddress(in_name.c_str(), buffers_[0]);
        context_->setTensorAddress(det_name.c_str(), buffers_[1]);
        context_->setTensorAddress(proto_name.c_str(), buffers_[2]);
    }

    ~DamageSegmenter() {
        if (stream_) cudaStreamDestroy(stream_);
        for (auto& buf : buffers_) if (buf) cudaFree(buf);
    }

    int numClasses() const { return num_classes_; }

    // rgb: 8-bit RGB frame. mask: 8-bit, frame size, 255 where damage was
    // segmented (all instances merged).
    std::vector<Damage> segment(const cv::Mat& rgb, cv::Mat& mask) {
        mask = cv::Mat();
        if (rgb.empty()) return {};

        preprocess(rgb);

        cudaMemcpyAsync(buffers_[0], input_host_.data(),
                        input_host_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        context_->enqueueV3(stream_);
        cudaMemcpyAsync(det_host_.data(), buffers_[1],
                        det_host_.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaMemcpyAsync(proto_host_.data(), buffers_[2],
                        proto_host_.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        std::vector<Damage> found = postprocess(rgb.cols, rgb.rows);
        if (!found.empty())
            buildMask(found, rgb.cols, rgb.rows, mask);
        return found;
    }

private:
    void preprocess(const cv::Mat& rgb) {
        float scale = std::min((float)input_w_ / rgb.cols, (float)input_h_ / rgb.rows);
        int new_w = rgb.cols * scale, new_h = rgb.rows * scale;
        scale_ = scale;
        pad_x_ = (input_w_ - new_w) / 2;
        pad_y_ = (input_h_ - new_h) / 2;

        cv::Mat resized, padded(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
        cv::resize(rgb, resized, cv::Size(new_w, new_h));
        resized.copyTo(padded(cv::Rect(pad_x_, pad_y_, new_w, new_h)));

        // HWC uint8 -> CHW float 0..1, written straight into the host buffer
        cv::Mat scaled;
        padded.convertTo(scaled, CV_32FC3, 1.0 / 255.0);
        const size_t plane = (size_t)input_h_ * input_w_;
        std::vector<cv::Mat> planes = {
            cv::Mat(input_h_, input_w_, CV_32FC1, input_host_.data()),
            cv::Mat(input_h_, input_w_, CV_32FC1, input_host_.data() + plane),
            cv::Mat(input_h_, input_w_, CV_32FC1, input_host_.data() + 2 * plane),
        };
        cv::split(scaled, planes);
    }

    std::vector<Damage> postprocess(int orig_w, int orig_h) {
        const int N = num_anchors_;
        std::vector<Damage> boxes;
        for (int i = 0; i < N; i++) {
            int best_class = 0;
            float conf = det_host_[4 * N + i];
            for (int c = 1; c < num_classes_; c++) {
                float v = det_host_[(4 + c) * N + i];
                if (v > conf) { conf = v; best_class = c; }
            }
            if (conf < conf_thresh_) continue;

            float cx = det_host_[0 * N + i];
            float cy = det_host_[1 * N + i];
            float w = det_host_[2 * N + i];
            float h = det_host_[3 * N + i];

            Damage d;
            d.x1 = std::max(0, std::min(orig_w - 1, (int)((cx - w / 2 - pad_x_) / scale_)));
            d.y1 = std::max(0, std::min(orig_h - 1, (int)((cy - h / 2 - pad_y_) / scale_)));
            d.x2 = std::max(0, std::min(orig_w - 1, (int)((cx + w / 2 - pad_x_) / scale_)));
            d.y2 = std::max(0, std::min(orig_h - 1, (int)((cy + h / 2 - pad_y_) / scale_)));
            d.confidence = conf;
            d.class_id = best_class;
            if (d.x2 - d.x1 < 4 || d.y2 - d.y1 < 4) continue;

            d.coeffs.resize(num_masks_);
            for (int m = 0; m < num_masks_; m++)
                d.coeffs[m] = det_host_[(4 + num_classes_ + m) * N + i];
            boxes.push_back(std::move(d));
        }
        nms(boxes);
        return boxes;
    }

    // Decode each instance mask at prototype resolution, merge them, and
    // upscale the merged mask once to the frame size. The merge keeps the
    // logits (not the thresholded masks) so the upscaled outline is smooth.
    void buildMask(std::vector<Damage>& found, int orig_w, int orig_h, cv::Mat& mask) {
        const float ratio = (float)proto_w_ / input_w_;  // prototype px per input px
        const float kOutside = -8.0f;                    // logit for "not damage"
        cv::Mat protos(num_masks_, proto_h_ * proto_w_, CV_32FC1, proto_host_.data());
        cv::Mat merged(proto_h_, proto_w_, CV_32FC1, cv::Scalar(kOutside));

        // The part of the prototype grid that holds the image (no letterbox bars)
        cv::Rect content((int)std::round(pad_x_ * ratio), (int)std::round(pad_y_ * ratio),
                         (int)std::round((input_w_ - 2 * pad_x_) * ratio),
                         (int)std::round((input_h_ - 2 * pad_y_) * ratio));
        content &= cv::Rect(0, 0, proto_w_, proto_h_);

        for (auto& d : found) {
            cv::Mat coeffs(1, num_masks_, CV_32FC1, d.coeffs.data());
            cv::Mat product = coeffs * protos;
            cv::Mat logits = product.reshape(1, proto_h_);

            // The instance's box in prototype coordinates; the mask is only
            // valid inside it.
            int bx1 = (int)std::floor((d.x1 * scale_ + pad_x_) * ratio);
            int by1 = (int)std::floor((d.y1 * scale_ + pad_y_) * ratio);
            int bx2 = (int)std::ceil((d.x2 * scale_ + pad_x_) * ratio);
            int by2 = (int)std::ceil((d.y2 * scale_ + pad_y_) * ratio);
            cv::Rect box(bx1, by1, bx2 - bx1, by2 - by1);
            box &= content;
            if (box.area() <= 0) continue;

            // sigmoid(x) > 0.5  <=>  x > 0
            // Cap the logits so a confident pixel cannot push the outline out
            // further than "outside" pulls it in when upscaling.
            cv::Mat instance;
            cv::threshold(logits(box), instance, -kOutside, 0, cv::THRESH_TRUNC);
            d.area_pct = 100.0f * cv::countNonZero(instance > 0.0f) / std::max(1, content.area());
            cv::Mat region = merged(box);
            cv::max(region, instance, region);
        }

        cv::Mat upscaled;
        cv::resize(merged(content), upscaled, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);
        mask = upscaled > 0.0f;
    }

    void nms(std::vector<Damage>& boxes) {
        std::sort(boxes.begin(), boxes.end(), [](const Damage& a, const Damage& b) {
            return a.confidence > b.confidence;
        });
        std::vector<bool> suppressed(boxes.size(), false);
        std::vector<Damage> result;
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

    float iou(const Damage& a, const Damage& b) {
        int x1 = std::max(a.x1, b.x1), y1 = std::max(a.y1, b.y1);
        int x2 = std::min(a.x2, b.x2), y2 = std::min(a.y2, b.y2);
        float inter = std::max(0, x2 - x1) * std::max(0, y2 - y1);
        float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
        float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
        return inter / (area_a + area_b - inter + 1e-6f);
    }

    float conf_thresh_, nms_thresh_, scale_ = 1.0f;
    int pad_x_ = 0, pad_y_ = 0;
    int input_h_ = 0, input_w_ = 0;
    int num_channels_ = 0, num_anchors_ = 0, num_classes_ = 0;
    int num_masks_ = 0, proto_h_ = 0, proto_w_ = 0;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    void* buffers_[3] = {nullptr, nullptr, nullptr};
    cudaStream_t stream_ = nullptr;
    std::vector<float> input_host_, det_host_, proto_host_;
};

// ============================================================================
// ROS2 Node
// ============================================================================
Publisher<sensor_msgs::Image> image_pub = NULL;
Publisher<std_msgs::msg::String> state_pub = NULL;
Publisher<std_msgs::msg::Bool> alert_pub = NULL;

std::unique_ptr<DamageSegmenter> segmenter;

// Class names, in model order ("pothole" for the default model)
std::vector<std::string> labels;

// The state stays raised for a moment after the last detection so a
// pothole passing through the image does not flicker on the dashboard.
double last_damage_ms = -1e12;
int last_count = 0;
const double HOLD_MS = 1000.0;

// Frames are skipped to keep to max_rate_hz. Segmenting every camera frame
// is not needed to find potholes, and the GPU is shared: at the camera's full
// rate this node alone made stereo depth about 50% slower on an Orin NX.
double max_rate_hz = 5.0;
double last_frame_ms = -1e12;

// Event log: one CSV row and one annotated JPEG per event
std::string log_dir;
double log_interval_s = 2.0;
double last_log_time = 0.0;
std::ofstream log_csv;

// Latest GPS fix, to geotag events
std::mutex gps_mutex;
sensor_msgs::msg::NavSatFix last_fix;
bool have_fix = false;

// Pre-allocated output message buffer
sensor_msgs::Image out_msg;
bool out_msg_initialized = false;
bool enable_timing = false;

// Timing statistics
struct TimingStats {
    double total_ms = 0;
    double infer_ms = 0;
    double draw_ms = 0;
    int frame_count = 0;
} rd_timing;

inline double rd_get_time_ms() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string labelOf(int class_id) {
    if (class_id >= 0 && class_id < (int)labels.size()) return labels[class_id];
    return "damage";
}

std::string upper(std::string s) {
    for (auto& c : s) c = std::toupper((unsigned char)c);
    return s;
}

// "CLEAR", "1 POTHOLE", "3 POTHOLES"; "2 DAMAGE" when the classes are mixed
std::string stateName(const std::vector<Damage>& found, int count) {
    if (count == 0) return "CLEAR";
    std::string name = "DAMAGE";
    if (!found.empty()) {
        bool same = true;
        for (auto& d : found) same = same && d.class_id == found[0].class_id;
        if (same) name = upper(labelOf(found[0].class_id)) + (count > 1 ? "S" : "");
    } else if (labels.size() == 1) {
        name = upper(labels[0]) + (count > 1 ? "S" : "");
    }
    return std::to_string(count) + " " + name;
}

void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(gps_mutex);
    last_fix = *msg;
    have_fix = true;
}

// Colors are RGB: the node works on the camera's rgb8 frames directly
void drawDamage(cv::Mat& image, const std::vector<Damage>& found, const cv::Mat& mask) {
    const cv::Scalar fill(255, 40, 40), edge(255, 220, 0);

    if (!mask.empty()) {
        cv::Mat tinted;
        cv::addWeighted(image, 0.55, cv::Mat(image.size(), image.type(), fill), 0.45, 0, tinted);
        tinted.copyTo(image, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(image, contours, -1, edge, 2, cv::LINE_AA);
    }

    for (auto& d : found) {
        cv::rectangle(image, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2), edge, 1);
        char text[64];
        snprintf(text, sizeof(text), "%s %.0f%%", labelOf(d.class_id).c_str(), d.confidence * 100);
        int text_y = d.y1 - 6;
        if (text_y < 15) text_y = d.y2 + 18;
        cv::putText(image, text, cv::Point(d.x1, text_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, edge, 2, cv::LINE_AA);
    }
}

// Append the event to events.csv and save the annotated frame next to it
void logEvent(const cv::Mat& annotated_rgb, const std::vector<Damage>& found) {
    if (log_dir.empty() || found.empty()) return;

    double now = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if (now - last_log_time < log_interval_s) return;
    last_log_time = now;

    std::time_t t = (std::time_t)now;
    std::tm tm_utc;
    gmtime_r(&t, &tm_utc);
    char stamp[32], iso[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_utc);
    std::strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);

    float max_conf = 0, area_pct = 0;
    for (auto& d : found) {
        max_conf = std::max(max_conf, d.confidence);
        area_pct += d.area_pct;
    }

    std::string image_name = std::string(stamp) + ".jpg";
    cv::Mat bgr;
    cv::cvtColor(annotated_rgb, bgr, cv::COLOR_RGB2BGR);
    cv::imwrite(log_dir + "/" + image_name, bgr, {cv::IMWRITE_JPEG_QUALITY, 85});

    char row[256];
    {
        std::lock_guard<std::mutex> lock(gps_mutex);
        // NavSatStatus: -1 = no fix. Leave the position empty without one.
        if (have_fix && last_fix.status.status >= 0)
            snprintf(row, sizeof(row), "%s,%.6f,%.6f,%.1f,%zu,%s,%.2f,%.2f,%s",
                     iso, last_fix.latitude, last_fix.longitude, last_fix.altitude,
                     found.size(), labelOf(found[0].class_id).c_str(),
                     max_conf, area_pct, image_name.c_str());
        else
            snprintf(row, sizeof(row), "%s,,,,%zu,%s,%.2f,%.2f,%s",
                     iso, found.size(), labelOf(found[0].class_id).c_str(),
                     max_conf, area_pct, image_name.c_str());
    }
    log_csv << row << std::endl;
}

void image_callback(const sensor_msgs::Image::SharedPtr msg) {
    double t0 = 0, t1 = 0, t2 = 0;
    const double now_ms = rd_get_time_ms();
    // 10% slack so a frame that arrives a little early is not skipped
    if (max_rate_hz > 0 && now_ms - last_frame_ms < 900.0 / max_rate_hz) return;
    last_frame_ms = now_ms;
    if (enable_timing) t0 = now_ms;

    // The model wants RGB, which is what the camera publishes; the annotated
    // copy stays RGB too, so there is no color conversion on the hot path.
    cv::Mat output;
    if (msg->encoding == "rgb8") {
        cv::Mat frame(msg->height, msg->width, CV_8UC3,
                      const_cast<uint8_t*>(msg->data.data()), msg->step);
        output = frame.clone();
    } else if (msg->encoding == "bgr8") {
        cv::Mat frame(msg->height, msg->width, CV_8UC3,
                      const_cast<uint8_t*>(msg->data.data()), msg->step);
        cv::cvtColor(frame, output, cv::COLOR_BGR2RGB);
    } else {
        ROS_WARN("Unsupported encoding: %s", msg->encoding.c_str());
        return;
    }

    // Initialize output message buffer
    size_t img_size = msg->height * msg->width * 3;
    if (!out_msg_initialized) {
        out_msg.data.resize(img_size);
        out_msg.header.frame_id = msg->header.frame_id;
        out_msg.encoding = "rgb8";
        out_msg.is_bigendian = false;
        out_msg_initialized = true;
    }

    // Segment on the clean frame (output is still an unannotated copy here)
    cv::Mat mask;
    auto found = segmenter->segment(output, mask);

    if (enable_timing) t1 = rd_get_time_ms();

    if (!found.empty()) {
        last_damage_ms = now_ms;
        last_count = (int)found.size();
        drawDamage(output, found, mask);
    }
    int count = (now_ms - last_damage_ms <= HOLD_MS) ? last_count : 0;
    std::string state = stateName(found, count);

    // Draw state
    cv::Scalar color = (count == 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 40, 40);
    cv::putText(output, state, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);

    logEvent(output, found);

    // Publish state
    std_msgs::msg::String state_msg;
    state_msg.data = state;
    state_pub->publish(state_msg);

    // Publish alert
    std_msgs::msg::Bool alert_msg;
    alert_msg.data = (count > 0);
    alert_pub->publish(alert_msg);

    // Publish annotated image using pre-allocated buffer
    out_msg.header.stamp = msg->header.stamp;
    out_msg.height = output.rows;
    out_msg.width = output.cols;
    out_msg.step = output.cols * 3;
    std::memcpy(out_msg.data.data(), output.data, img_size);
    image_pub->publish(out_msg);

    if (enable_timing) {
        t2 = rd_get_time_ms();
        rd_timing.total_ms += (t2 - t0);
        rd_timing.infer_ms += (t1 - t0);
        rd_timing.draw_ms += (t2 - t1);
        rd_timing.frame_count++;

        if (rd_timing.frame_count % 30 == 0) {
            int n = rd_timing.frame_count;
            ROS_INFO("Road Damage Timing (avg ms): segment=%.1f draw+publish=%.1f total=%.1f",
                     rd_timing.infer_ms / n, rd_timing.draw_ms / n, rd_timing.total_ms / n);
        }
    }
}

// mkdir -p
bool makeDirs(const std::string& path) {
    for (size_t i = 1; i <= path.size(); i++) {
        if (i == path.size() || path[i] == '/') {
            std::string part = path.substr(0, i);
            if (mkdir(part.c_str(), 0755) != 0 && errno != EEXIST) return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    ROS_CREATE_NODE("road_damage");
    ROS_INFO("Road Damage Node - TensorRT");

    // Get parameters
    std::string engine, camera_topic, labels_str;
    float conf_thresh = 0.4f;

    ROS_DECLARE_PARAMETER("engine", engine);
    ROS_DECLARE_PARAMETER("camera_topic", camera_topic);
    ROS_DECLARE_PARAMETER("labels", labels_str);
    ROS_DECLARE_PARAMETER("confidence", conf_thresh);
    ROS_DECLARE_PARAMETER("max_rate_hz", max_rate_hz);
    ROS_DECLARE_PARAMETER("log_dir", log_dir);
    ROS_DECLARE_PARAMETER("log_interval_s", log_interval_s);
    ROS_DECLARE_PARAMETER("enable_timing", enable_timing);

    ROS_GET_PARAMETER("engine", engine);
    ROS_GET_PARAMETER("camera_topic", camera_topic);
    ROS_GET_PARAMETER("labels", labels_str);
    ROS_GET_PARAMETER("confidence", conf_thresh);
    ROS_GET_PARAMETER("max_rate_hz", max_rate_hz);
    ROS_GET_PARAMETER("log_dir", log_dir);
    ROS_GET_PARAMETER("log_interval_s", log_interval_s);
    ROS_GET_PARAMETER("enable_timing", enable_timing);

    // Defaults. A relative engine path resolves against the installed graphs
    // directory (share/visionconnect/graphs), like detect/lanedet/classify.
    if (engine.empty())
        engine = "road-damage/yolov8s-pothole-seg_fp16.engine";
    if (camera_topic.empty())
        camera_topic = "/camera/raw";
    if (labels_str.empty())
        labels_str = "pothole";

    const std::string graphs_dir =
        ament_index_cpp::get_package_share_directory("visionconnect") + "/graphs/";
    if (engine[0] != '/')
        engine = graphs_dir + engine;

    // Comma-separated class names, in model order
    std::stringstream ss(labels_str);
    for (std::string name; std::getline(ss, name, ',');)
        if (!name.empty()) labels.push_back(name);

    // Load model
    ROS_INFO("Loading segmenter: %s", engine.c_str());
    segmenter = std::make_unique<DamageSegmenter>(engine, conf_thresh, 0.45f);
    if ((int)labels.size() != segmenter->numClasses())
        ROS_WARN("%zu label(s) configured but the model has %d class(es)",
                 labels.size(), segmenter->numClasses());

    // Event log. "~/" is the home directory; an empty log_dir turns it off.
    if (log_dir.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        log_dir = std::string(home ? home : "") + log_dir.substr(1);
    }
    if (!log_dir.empty()) {
        const std::string csv_path = log_dir + "/events.csv";
        struct stat st;
        bool is_new = (stat(csv_path.c_str(), &st) != 0);
        if (makeDirs(log_dir))
            log_csv.open(csv_path, std::ios::app);
        if (log_csv.is_open()) {
            if (is_new)
                log_csv << "time_utc,latitude,longitude,altitude_m,count,label,"
                           "max_confidence,area_pct,image" << std::endl;
            ROS_INFO("Logging events to %s (at most one every %.1f s)",
                     log_dir.c_str(), log_interval_s);
        } else {
            ROS_WARN("Cannot write to %s - event logging is off", log_dir.c_str());
            log_dir.clear();
        }
    }

    // Publishers - use BEST_EFFORT QoS for image to match subscribers
    rclcpp::QoS qos_pub(2);
    qos_pub.best_effort();
    qos_pub.durability_volatile();
    image_pub = node->create_publisher<sensor_msgs::Image>("image", qos_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "state", 10, state_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::Bool, "alert", 10, alert_pub);

    // Subscribers - use BEST_EFFORT QoS on the image to prevent back-pressure
    ROS_INFO("Subscribing to: %s", camera_topic.c_str());
    rclcpp::QoS qos_best_effort(2);
    qos_best_effort.best_effort();
    qos_best_effort.durability_volatile();
    auto image_sub = node->create_subscription<sensor_msgs::Image>(
        camera_topic.c_str(), qos_best_effort, image_callback);
    auto gps_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_in", 10, gps_callback);

    ROS_INFO("Road damage segmentation ready (at most %.1f Hz, timing: %s)",
             max_rate_hz, enable_timing ? "enabled" : "disabled");

    ROS_SPIN();

    segmenter.reset();

    return 0;
}
