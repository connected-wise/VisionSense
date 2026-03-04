/**
 * TensorRT-Accelerated Stereo Depth Node
 *
 * Combined node that:
 * 1. Subscribes to left/right raw images from camera_stereo
 * 2. Preprocesses images (resize, pad, normalize) for LightStereo model
 * 3. Runs TensorRT inference for disparity estimation
 * 4. Converts disparity to depth (meters) and publishes depth + colorized depth
 *
 * Uses LightStereo deep learning model for accurate stereo matching
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <fstream>

//==============================================================================
// TensorRT Logger
//==============================================================================
class TRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TRT] " << msg << std::endl;
        }
    }
};

static TRTLogger g_trt_logger;

//==============================================================================
// Preprocessor - Matches LightStereo model requirements (Optimized)
//==============================================================================
class StereoPreprocessor {
public:
    static constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float STD[3] = {0.229f, 0.224f, 0.225f};

    StereoPreprocessor(int target_h, int target_w)
        : target_h_(target_h), target_w_(target_w) {
        output_.resize(3 * target_h * target_w);
        // Pre-allocate intermediate buffers
        resized_.create(target_h, target_w, CV_8UC3);
        padded_.create(target_h, target_w, CV_8UC3);
    }

    // Process RGB uint8 image -> normalized CHW tensor
    // Optimized: works directly with uint8, pre-allocated buffers
    const std::vector<float>& process(const cv::Mat& input_rgb) {
        int in_h = input_rgb.rows;
        int in_w = input_rgb.cols;

        // Resize to fit within target while maintaining aspect ratio
        float scale = std::min((float)target_h_ / in_h, (float)target_w_ / in_w);
        int new_h = static_cast<int>(in_h * scale);
        int new_w = static_cast<int>(in_w * scale);

        cv::resize(input_rgb, resized_, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

        // RightTopPad to target size
        pad_top_ = target_h_ - new_h;
        pad_right_ = target_w_ - new_w;
        valid_h_ = new_h;
        valid_w_ = new_w;

        cv::copyMakeBorder(resized_, padded_, pad_top_, 0, 0, pad_right_, cv::BORDER_REPLICATE);

        // Optimized HWC->CHW transpose + normalize
        const int hw = target_h_ * target_w_;
        float* out_r = output_.data();
        float* out_g = output_.data() + hw;
        float* out_b = output_.data() + 2 * hw;

        const float inv_255 = 1.0f / 255.0f;
        const float inv_std_r = 1.0f / STD[0];
        const float inv_std_g = 1.0f / STD[1];
        const float inv_std_b = 1.0f / STD[2];

        for (int y = 0; y < target_h_; y++) {
            const uchar* row = padded_.ptr<uchar>(y);
            const int row_offset = y * target_w_;
            for (int x = 0; x < target_w_; x++) {
                const int px = x * 3;
                out_r[row_offset + x] = (row[px + 0] * inv_255 - MEAN[0]) * inv_std_r;
                out_g[row_offset + x] = (row[px + 1] * inv_255 - MEAN[1]) * inv_std_g;
                out_b[row_offset + x] = (row[px + 2] * inv_255 - MEAN[2]) * inv_std_b;
            }
        }

        return output_;
    }

    int getPadTop() const { return pad_top_; }
    int getPadRight() const { return pad_right_; }
    int getValidHeight() const { return valid_h_; }
    int getValidWidth() const { return valid_w_; }

private:
    int target_h_, target_w_;
    int pad_top_ = 0, pad_right_ = 0;
    int valid_h_ = 0, valid_w_ = 0;
    std::vector<float> output_;
    cv::Mat resized_, padded_;
};

constexpr float StereoPreprocessor::MEAN[3];
constexpr float StereoPreprocessor::STD[3];

//==============================================================================
// TensorRT Stereo Engine
//==============================================================================
class StereoTRTEngine {
public:
    StereoTRTEngine(const std::string& engine_path) {
        std::ifstream file(engine_path, std::ios::binary);
        if (!file.good()) {
            throw std::runtime_error("Cannot open engine file: " + engine_path);
        }

        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> engine_data(size);
        file.read(engine_data.data(), size);
        file.close();

        runtime_ = nvinfer1::createInferRuntime(g_trt_logger);
        engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
        context_ = engine_->createExecutionContext();

        if (!engine_ || !context_) {
            throw std::runtime_error("Failed to create TensorRT engine");
        }

        left_dims_ = engine_->getTensorShape("left_img");
        right_dims_ = engine_->getTensorShape("right_img");
        output_dims_ = engine_->getTensorShape("disp_pred");

        input_h_ = left_dims_.d[2];
        input_w_ = left_dims_.d[3];
        output_h_ = output_dims_.d[output_dims_.nbDims - 2];
        output_w_ = output_dims_.d[output_dims_.nbDims - 1];

        size_t input_bytes = tensorVolume(left_dims_) * sizeof(float);
        size_t output_bytes = tensorVolume(output_dims_) * sizeof(float);

        cudaMalloc(&d_left_, input_bytes);
        cudaMalloc(&d_right_, input_bytes);
        cudaMalloc(&d_output_, output_bytes);

        context_->setTensorAddress("left_img", d_left_);
        context_->setTensorAddress("right_img", d_right_);
        context_->setTensorAddress("disp_pred", d_output_);

        cudaStreamCreate(&stream_);
        output_.resize(tensorVolume(output_dims_));

        std::cout << "Stereo engine loaded: input " << input_w_ << "x" << input_h_
                  << ", output " << output_w_ << "x" << output_h_ << std::endl;
    }

    ~StereoTRTEngine() {
        if (stream_) cudaStreamDestroy(stream_);
        if (d_left_) cudaFree(d_left_);
        if (d_right_) cudaFree(d_right_);
        if (d_output_) cudaFree(d_output_);
        if (context_) delete context_;
        if (engine_) delete engine_;
        if (runtime_) delete runtime_;
    }

    void infer(const std::vector<float>& left, const std::vector<float>& right) {
        size_t input_bytes = tensorVolume(left_dims_) * sizeof(float);

        cudaMemcpyAsync(d_left_, left.data(), input_bytes, cudaMemcpyHostToDevice, stream_);
        cudaMemcpyAsync(d_right_, right.data(), input_bytes, cudaMemcpyHostToDevice, stream_);

        context_->enqueueV3(stream_);

        cudaMemcpyAsync(output_.data(), d_output_, output_.size() * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);
    }

    void warmup(const std::vector<float>& left, const std::vector<float>& right, int iterations = 5) {
        size_t input_bytes = tensorVolume(left_dims_) * sizeof(float);
        cudaMemcpy(d_left_, left.data(), input_bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(d_right_, right.data(), input_bytes, cudaMemcpyHostToDevice);

        for (int i = 0; i < iterations; i++) {
            context_->enqueueV3(stream_);
            cudaStreamSynchronize(stream_);
        }
    }

    const std::vector<float>& getOutput() const { return output_; }
    int inputHeight() const { return input_h_; }
    int inputWidth() const { return input_w_; }
    int outputHeight() const { return output_h_; }
    int outputWidth() const { return output_w_; }

private:
    size_t tensorVolume(const nvinfer1::Dims& dims) {
        size_t vol = 1;
        for (int i = 0; i < dims.nbDims; ++i) vol *= dims.d[i];
        return vol;
    }

    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_ = nullptr;

    nvinfer1::Dims left_dims_, right_dims_, output_dims_;
    int input_h_, input_w_, output_h_, output_w_;

    void* d_left_ = nullptr;
    void* d_right_ = nullptr;
    void* d_output_ = nullptr;

    std::vector<float> output_;
};

//==============================================================================
// ROS2 Stereo Depth Node (Optimized)
//==============================================================================
class StereoDepthNode : public rclcpp::Node
{
public:
    StereoDepthNode() : Node("stereo_depth")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing TensorRT Stereo Depth Node...");

        this->declare_parameter("model", "stereo-depth/LightStereo-S-KITTI.engine");
        this->declare_parameter("max_disparity", 192.0);
        this->declare_parameter("warmup_iterations", 5);
        this->declare_parameter("baseline_mm", 100.0);
        this->declare_parameter("focal_length_mm", 3.6);
        this->declare_parameter("sensor_width_mm", 5.76);
        this->declare_parameter("max_depth_m", 50.0);
        this->declare_parameter("color_publish_size", 480);

        std::string model_name = this->get_parameter("model").as_string();
        max_disparity_ = this->get_parameter("max_disparity").as_double();
        int warmup_iters = this->get_parameter("warmup_iterations").as_int();
        baseline_m_ = this->get_parameter("baseline_mm").as_double() / 1000.0;
        focal_length_mm_ = this->get_parameter("focal_length_mm").as_double();
        sensor_width_mm_ = this->get_parameter("sensor_width_mm").as_double();
        max_depth_m_ = this->get_parameter("max_depth_m").as_double();
        color_publish_size_ = this->get_parameter("color_publish_size").as_int();

        std::string graphs_path = ament_index_cpp::get_package_share_directory("visionconnect") + "/graphs/";
        std::string engine_path = graphs_path + model_name;

        RCLCPP_INFO(this->get_logger(), "Loading TensorRT engine: %s", engine_path.c_str());

        try {
            engine_ = std::make_unique<StereoTRTEngine>(engine_path);
            RCLCPP_INFO(this->get_logger(), "TensorRT engine loaded: %dx%d",
                        engine_->inputWidth(), engine_->inputHeight());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load TensorRT engine: %s", e.what());
            throw;
        }

        left_preproc_ = std::make_unique<StereoPreprocessor>(engine_->inputHeight(), engine_->inputWidth());
        right_preproc_ = std::make_unique<StereoPreprocessor>(engine_->inputHeight(), engine_->inputWidth());

        // Pre-allocate postprocessing buffers
        disp_full_.create(engine_->outputHeight(), engine_->outputWidth(), CV_32FC1);

        // Publishers with BEST_EFFORT QoS, depth=1 to always serve latest frame
        rclcpp::QoS qos_pub(1);
        qos_pub.best_effort();
        qos_pub.durability_volatile();
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth", qos_pub);
        depth_color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_color", qos_pub);

        // Synchronized subscribers with BEST_EFFORT QoS, depth=2 for reliable sync matching
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.depth = 2;
        left_sub_.subscribe(this, "left/image_raw", qos_profile);
        right_sub_.subscribe(this, "right/image_raw", qos_profile);

        sync_ = std::make_shared<message_filters::TimeSynchronizer<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
            left_sub_, right_sub_, 2);

        sync_->registerCallback(
            std::bind(&StereoDepthNode::stereoCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Stereo depth node ready");

        warmup_done_ = false;
        warmup_iterations_ = warmup_iters;
    }

private:
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        // Zero-copy message to Mat conversion
        cv::Mat left = msgToMatNoCopy(left_msg);
        cv::Mat right = msgToMatNoCopy(right_msg);

        // Handle color conversion - avoid unnecessary copies
        cv::Mat left_rgb, right_rgb;
        if (left_msg->encoding == "rgb8") {
            left_rgb = left;
            right_rgb = right;
        } else if (left_msg->encoding == "bgr8") {
            cv::cvtColor(left, left_rgb, cv::COLOR_BGR2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_BGR2RGB);
        } else if (left.channels() == 1) {
            cv::cvtColor(left, left_rgb, cv::COLOR_GRAY2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_GRAY2RGB);
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Unknown encoding '%s', assuming BGR",
                            left_msg->encoding.c_str());
            cv::cvtColor(left, left_rgb, cv::COLOR_BGR2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_BGR2RGB);
        }

        // Preprocess (optimized - works with uint8 directly)
        const auto& left_data = left_preproc_->process(left_rgb);
        const auto& right_data = right_preproc_->process(right_rgb);

        // Warmup on first frame
        if (!warmup_done_) {
            RCLCPP_INFO(this->get_logger(), "Warming up TensorRT engine...");
            engine_->warmup(left_data, right_data, warmup_iterations_);
            warmup_done_ = true;
            RCLCPP_INFO(this->get_logger(), "Warmup complete");
        }

        // Run inference
        engine_->infer(left_data, right_data);

        // Get output and padding info
        const auto& disp_output = engine_->getOutput();
        int out_h = engine_->outputHeight();
        int out_w = engine_->outputWidth();
        int pad_top = left_preproc_->getPadTop();
        int valid_h = left_preproc_->getValidHeight();
        int valid_w = left_preproc_->getValidWidth();

        // Copy disparity to pre-allocated buffer
        memcpy(disp_full_.data, disp_output.data(), out_h * out_w * sizeof(float));

        // Extract valid region and resize to original dimensions
        cv::Rect valid_roi(0, pad_top, valid_w, valid_h);
        cv::Mat disp_valid = disp_full_(valid_roi);
        cv::resize(disp_valid, disp_resized_, cv::Size(left.cols, left.rows), 0, 0, cv::INTER_LINEAR);

        // Initialize depth buffers and messages once (pre-allocated, zero alloc in hot path)
        if (!depth_initialized_) {
            focal_length_px_ = (focal_length_mm_ / sensor_width_mm_) * (double)left.cols;
            bf_ = baseline_m_ * focal_length_px_;
            RCLCPP_INFO(this->get_logger(), "Depth params: focal_px=%.1f, bf=%.3f, max_depth=%.1fm",
                        focal_length_px_, bf_, max_depth_m_);

            int rows = left.rows, cols = left.cols;

            depth_norm_.create(rows, cols, CV_8UC1);

            depth_msg_.header.frame_id = "stereo_depth";
            depth_msg_.encoding = "32FC1";
            depth_msg_.is_bigendian = false;
            depth_msg_.height = rows;
            depth_msg_.width = cols;
            depth_msg_.step = cols * sizeof(float);
            depth_msg_.data.resize(rows * cols * sizeof(float));

            int pub_size = color_publish_size_;
            depth_norm_small_.create(pub_size, pub_size, CV_8UC1);
            zero_mask_.create(pub_size, pub_size, CV_8UC1);

            depth_color_msg_.header.frame_id = "stereo_depth";
            depth_color_msg_.encoding = "bgr8";
            depth_color_msg_.is_bigendian = false;
            depth_color_msg_.height = pub_size;
            depth_color_msg_.width = pub_size;
            depth_color_msg_.step = pub_size * 3;
            depth_color_msg_.data.resize(pub_size * pub_size * 3);

            depth_initialized_ = true;
        }

        int rows = disp_resized_.rows;
        int cols = disp_resized_.cols;

        // Write depth directly into pre-allocated message buffer (no intermediate Mat + memcpy)
        cv::Mat depth_map(rows, cols, CV_32FC1, depth_msg_.data.data());
        for (int r = 0; r < rows; r++) {
            const float* disp_row = disp_resized_.ptr<float>(r);
            float* depth_row = depth_map.ptr<float>(r);
            for (int c = 0; c < cols; c++) {
                depth_row[c] = (disp_row[c] > 1.0f) ? (float)(bf_ / disp_row[c]) : 0.0f;
            }
        }

        // Normalize full-res depth to uint8
        depth_map.convertTo(depth_norm_, CV_8UC1, 255.0 / max_depth_m_);

        // Resize mono8 BEFORE colormap — much cheaper than resizing bgr8
        int pub_size = color_publish_size_;
        cv::resize(depth_norm_, depth_norm_small_, cv::Size(pub_size, pub_size), 0, 0, cv::INTER_AREA);

        // Colormap on small image, directly into pre-allocated message buffer
        cv::Mat depth_color(pub_size, pub_size, CV_8UC3, depth_color_msg_.data.data());
        cv::applyColorMap(depth_norm_small_, depth_color, cv::COLORMAP_TURBO);

        // Zero-mask using pre-allocated buffer
        cv::compare(depth_norm_small_, cv::Scalar(0), zero_mask_, cv::CMP_EQ);
        depth_color.setTo(cv::Scalar(0, 0, 0), zero_mask_);

        // Publish (ROS copies internally, but no per-frame allocation)
        depth_msg_.header.stamp = left_msg->header.stamp;
        depth_color_msg_.header.stamp = left_msg->header.stamp;
        if (depth_pub_->get_subscription_count() > 0) {
            depth_pub_->publish(depth_msg_);
        }
        depth_color_pub_->publish(depth_color_msg_);
    }

    cv::Mat msgToMatNoCopy(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        int cv_type;
        if (msg->encoding == "mono8")
            cv_type = CV_8UC1;
        else if (msg->encoding == "bgr8" || msg->encoding == "rgb8")
            cv_type = CV_8UC3;
        else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Unknown encoding: %s", msg->encoding.c_str());
            cv_type = CV_8UC3;
        }
        return cv::Mat(msg->height, msg->width, cv_type,
                       const_cast<uint8_t*>(msg->data.data()), msg->step);
    }

    std::unique_ptr<StereoTRTEngine> engine_;
    std::unique_ptr<StereoPreprocessor> left_preproc_;
    std::unique_ptr<StereoPreprocessor> right_preproc_;

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_color_pub_;

    double max_disparity_;
    int warmup_iterations_;
    bool warmup_done_;

    // Depth conversion parameters
    double baseline_m_, focal_length_mm_, sensor_width_mm_, max_depth_m_;
    double focal_length_px_ = 0.0, bf_ = 0.0;
    bool depth_initialized_ = false;

    // Pre-allocated buffers (zero-allocation hot path)
    cv::Mat disp_full_;
    cv::Mat disp_resized_;
    cv::Mat depth_norm_;        // CV_8UC1 - normalized for colormap
    cv::Mat depth_norm_small_;  // CV_8UC1 - resized for colormap
    cv::Mat zero_mask_;         // CV_8UC1 - pre-allocated mask
    int color_publish_size_ = 480;
    sensor_msgs::msg::Image depth_msg_;
    sensor_msgs::msg::Image depth_color_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<StereoDepthNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_depth"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
