/**
 * TensorRT-Accelerated Stereo Depth Node
 *
 * Combined node that:
 * 1. Subscribes to left/right raw images from camera_stereo
 * 2. Preprocesses images (resize, pad, normalize) for LightStereo model
 * 3. Runs TensorRT inference for disparity estimation
 * 4. Publishes disparity image to /stereo_depth/disparity
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
#include <chrono>

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
// Preprocessor - Matches LightStereo model requirements
//==============================================================================
class StereoPreprocessor {
public:
    // ImageNet normalization parameters
    static constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float STD[3] = {0.229f, 0.224f, 0.225f};

    StereoPreprocessor(int target_h, int target_w)
        : target_h_(target_h), target_w_(target_w) {
        output_.resize(3 * target_h * target_w);
    }

    // Process RGB image -> normalized CHW tensor
    // Uses RightTopPad: pad top and right with edge replication
    const std::vector<float>& process(const cv::Mat& input_rgb) {
        int in_h = input_rgb.rows;
        int in_w = input_rgb.cols;

        // Step 1: Resize to fit within target while maintaining aspect ratio
        float scale = std::min((float)target_h_ / in_h, (float)target_w_ / in_w);
        int new_h = static_cast<int>(in_h * scale);
        int new_w = static_cast<int>(in_w * scale);

        cv::Mat resized;
        cv::resize(input_rgb, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

        // Step 2: RightTopPad to target size (pad top and right with edge replication)
        int pad_top = target_h_ - new_h;
        int pad_right = target_w_ - new_w;

        cv::Mat padded;
        cv::copyMakeBorder(resized, padded, pad_top, 0, 0, pad_right, cv::BORDER_REPLICATE);

        // Step 3: Transpose (HWC -> CHW) and Normalize
        for (int y = 0; y < target_h_; y++) {
            for (int x = 0; x < target_w_; x++) {
                const cv::Vec3f& pixel = padded.at<cv::Vec3f>(y, x);
                for (int c = 0; c < 3; c++) {
                    int idx = c * target_h_ * target_w_ + y * target_w_ + x;
                    output_[idx] = ((pixel[c] / 255.0f) - MEAN[c]) / STD[c];
                }
            }
        }

        return output_;
    }

    const std::vector<float>& getOutput() const { return output_; }

private:
    int target_h_;
    int target_w_;
    std::vector<float> output_;
};

constexpr float StereoPreprocessor::MEAN[3];
constexpr float StereoPreprocessor::STD[3];

//==============================================================================
// TensorRT Stereo Engine
//==============================================================================
class StereoTRTEngine {
public:
    StereoTRTEngine(const std::string& engine_path) {
        // Load engine file
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

        // Create runtime and engine
        runtime_ = nvinfer1::createInferRuntime(g_trt_logger);
        engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
        context_ = engine_->createExecutionContext();

        if (!engine_ || !context_) {
            throw std::runtime_error("Failed to create TensorRT engine");
        }

        // Get tensor info
        left_dims_ = engine_->getTensorShape("left_img");
        right_dims_ = engine_->getTensorShape("right_img");
        output_dims_ = engine_->getTensorShape("disp_pred");

        input_h_ = left_dims_.d[2];
        input_w_ = left_dims_.d[3];
        output_h_ = output_dims_.d[output_dims_.nbDims - 2];
        output_w_ = output_dims_.d[output_dims_.nbDims - 1];

        size_t input_bytes = tensorVolume(left_dims_) * sizeof(float);
        size_t output_bytes = tensorVolume(output_dims_) * sizeof(float);

        // Allocate GPU buffers
        cudaMalloc(&d_left_, input_bytes);
        cudaMalloc(&d_right_, input_bytes);
        cudaMalloc(&d_output_, output_bytes);

        // Set tensor addresses
        context_->setTensorAddress("left_img", d_left_);
        context_->setTensorAddress("right_img", d_right_);
        context_->setTensorAddress("disp_pred", d_output_);

        cudaStreamCreate(&stream_);

        // Allocate output buffer
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
        for (int i = 0; i < dims.nbDims; ++i) {
            vol *= dims.d[i];
        }
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
// ROS2 Stereo Depth Node
//==============================================================================
class StereoDepthNode : public rclcpp::Node
{
public:
    StereoDepthNode() : Node("stereo_depth")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing TensorRT Stereo Depth Node...");

        // Declare parameters
        this->declare_parameter("model", "stereo-depth/LightStereo-S-KITTI.engine");
        this->declare_parameter("max_disparity", 192.0);
        this->declare_parameter("warmup_iterations", 5);

        // Get parameters
        std::string model_name = this->get_parameter("model").as_string();
        max_disparity_ = this->get_parameter("max_disparity").as_double();
        int warmup_iters = this->get_parameter("warmup_iterations").as_int();

        // Resolve model path
        std::string graphs_path = ament_index_cpp::get_package_share_directory("visionconnect") + "/graphs/";
        std::string engine_path = graphs_path + model_name;

        RCLCPP_INFO(this->get_logger(), "Loading TensorRT engine: %s", engine_path.c_str());

        // Initialize TensorRT engine
        try {
            engine_ = std::make_unique<StereoTRTEngine>(engine_path);
            RCLCPP_INFO(this->get_logger(), "✓ TensorRT engine loaded successfully");
            RCLCPP_INFO(this->get_logger(), "  Input size: %dx%d", engine_->inputWidth(), engine_->inputHeight());
            RCLCPP_INFO(this->get_logger(), "  Output size: %dx%d", engine_->outputWidth(), engine_->outputHeight());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load TensorRT engine: %s", e.what());
            throw;
        }

        // Initialize preprocessors
        left_preproc_ = std::make_unique<StereoPreprocessor>(engine_->inputHeight(), engine_->inputWidth());
        right_preproc_ = std::make_unique<StereoPreprocessor>(engine_->inputHeight(), engine_->inputWidth());

        // Create publisher
        disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("disparity", 10);

        // Create synchronized subscribers
        left_sub_.subscribe(this, "left/image_raw");
        right_sub_.subscribe(this, "right/image_raw");

        sync_ = std::make_shared<message_filters::TimeSynchronizer<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
            left_sub_, right_sub_, 10);

        sync_->registerCallback(
            std::bind(&StereoDepthNode::stereoCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "TensorRT stereo depth node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: left/image_raw, right/image_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing disparity to: /stereo_depth/disparity");

        // Warmup will happen on first frame
        warmup_done_ = false;
        warmup_iterations_ = warmup_iters;
    }

private:
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        auto t_start = std::chrono::high_resolution_clock::now();

        // Convert ROS messages to OpenCV Mat
        cv::Mat left = msgToMat(left_msg);
        cv::Mat right = msgToMat(right_msg);

        // Convert BGR to RGB and to float32
        cv::Mat left_rgb, right_rgb;
        if (left.channels() == 3) {
            cv::cvtColor(left, left_rgb, cv::COLOR_BGR2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_BGR2RGB);
        } else {
            cv::cvtColor(left, left_rgb, cv::COLOR_GRAY2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_GRAY2RGB);
        }

        // Convert to float32 for preprocessing
        left_rgb.convertTo(left_rgb, CV_32FC3);
        right_rgb.convertTo(right_rgb, CV_32FC3);

        // Preprocess images
        const auto& left_data = left_preproc_->process(left_rgb);
        const auto& right_data = right_preproc_->process(right_rgb);

        // Warmup on first frame
        if (!warmup_done_) {
            RCLCPP_INFO(this->get_logger(), "Warming up TensorRT engine (%d iterations)...", warmup_iterations_);
            engine_->warmup(left_data, right_data, warmup_iterations_);
            warmup_done_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ Warmup complete");
        }

        // Run inference
        engine_->infer(left_data, right_data);

        // Get output disparity
        const auto& disp_output = engine_->getOutput();
        int out_h = engine_->outputHeight();
        int out_w = engine_->outputWidth();

        // Convert disparity to 8-bit normalized image
        cv::Mat disp_8u(out_h, out_w, CV_8UC1);
        for (int y = 0; y < out_h; y++) {
            for (int x = 0; x < out_w; x++) {
                float d = std::max(0.0f, std::min(disp_output[y * out_w + x] / (float)max_disparity_, 1.0f));
                disp_8u.at<uchar>(y, x) = static_cast<uchar>(d * 255);
            }
        }

        // Publish disparity
        auto disp_msg = matToMsg(disp_8u, left_msg->header, "mono8");
        disparity_pub_->publish(disp_msg);

        auto t_end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        // Log performance periodically
        frame_count_++;
        total_time_ms_ += ms;
        if (frame_count_ % 30 == 0) {
            double avg_ms = total_time_ms_ / frame_count_;
            double fps = 1000.0 / avg_ms;
            RCLCPP_INFO(this->get_logger(), "Frame %d | %.1fms | %.1f FPS", frame_count_, ms, fps);
        }
    }

    cv::Mat msgToMat(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        int cv_type;
        if (msg->encoding == "mono8")
            cv_type = CV_8UC1;
        else if (msg->encoding == "bgr8")
            cv_type = CV_8UC3;
        else if (msg->encoding == "rgb8")
            cv_type = CV_8UC3;
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown encoding: %s, assuming bgr8", msg->encoding.c_str());
            cv_type = CV_8UC3;
        }

        cv::Mat img(msg->height, msg->width, cv_type,
                    const_cast<uint8_t*>(&msg->data[0]), msg->step);
        return img.clone();
    }

    sensor_msgs::msg::Image matToMsg(const cv::Mat& img,
                                     const std_msgs::msg::Header& header,
                                     const std::string& encoding)
    {
        sensor_msgs::msg::Image msg;
        msg.header = header;
        msg.height = img.rows;
        msg.width = img.cols;
        msg.encoding = encoding;
        msg.is_bigendian = false;
        msg.step = img.cols * img.elemSize();

        size_t size = msg.step * msg.height;
        msg.data.resize(size);
        memcpy(&msg.data[0], img.data, size);

        return msg;
    }

    // TensorRT engine
    std::unique_ptr<StereoTRTEngine> engine_;

    // Preprocessors
    std::unique_ptr<StereoPreprocessor> left_preproc_;
    std::unique_ptr<StereoPreprocessor> right_preproc_;

    // Subscribers and synchronizer
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;

    // Parameters
    double max_disparity_;
    int warmup_iterations_;
    bool warmup_done_;

    // Performance tracking
    int frame_count_ = 0;
    double total_time_ms_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<StereoDepthNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_depth"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
