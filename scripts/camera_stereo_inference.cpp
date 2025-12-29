/*
 * Stereo Camera Inference - Standalone Implementation
 *
 * Captures stereo images from camera and runs TensorRT stereo matching inference.
 * No project dependencies - only requires system libraries:
 *   - OpenCV
 *   - TensorRT
 *   - CUDA
 *   - jetson-utils
 *
 * Build:
 *   g++ -O3 camera_stereo_inference.cpp -o camera_stereo_inference \
 *       $(pkg-config --cflags --libs opencv4) \
 *       -I/usr/include/aarch64-linux-gnu \
 *       -lnvinfer -lcudart -ljetson-utils
 *
 * Usage:
 *   ./camera_stereo_inference <engine_path> <camera_resource> [options]
 *
 * Example:
 *   ./camera_stereo_inference model.engine v4l2:///dev/video1 --width 3840 --height 1200
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <jetson-utils/videoSource.h>

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

//==============================================================================
// Global State
//==============================================================================
static bool g_signal_received = false;
static TRTLogger g_logger;

void signalHandler(int sig) {
    g_signal_received = true;
    std::cout << "\nShutdown signal received..." << std::endl;
}

//==============================================================================
// Utility Functions
//==============================================================================
inline bool fileExists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

inline size_t tensorVolume(const nvinfer1::Dims& dims) {
    size_t vol = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        vol *= dims.d[i];
    }
    return vol;
}

//==============================================================================
// Preprocessing - CPU implementation matching Python exactly
//==============================================================================
class Preprocessor {
public:
    // ImageNet normalization parameters
    static constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float STD[3] = {0.229f, 0.224f, 0.225f};

    Preprocessor(int target_h, int target_w)
        : target_h_(target_h), target_w_(target_w) {
        output_.resize(3 * target_h * target_w);
    }

    // Process RGB float32 image (HWC, 0-255) -> normalized CHW
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

constexpr float Preprocessor::MEAN[3];
constexpr float Preprocessor::STD[3];

//==============================================================================
// TensorRT Inference Engine
//==============================================================================
class StereoEngine {
public:
    StereoEngine(const std::string& engine_path) {
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
        runtime_ = nvinfer1::createInferRuntime(g_logger);
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

        std::cout << "Engine loaded: input " << input_w_ << "x" << input_h_
                  << ", output " << output_w_ << "x" << output_h_ << std::endl;
    }

    ~StereoEngine() {
        cudaStreamDestroy(stream_);
        cudaFree(d_left_);
        cudaFree(d_right_);
        cudaFree(d_output_);
        delete context_;
        delete engine_;
        delete runtime_;
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

    void warmup(const std::vector<float>& left, const std::vector<float>& right, int iterations) {
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
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_;

    nvinfer1::Dims left_dims_, right_dims_, output_dims_;
    int input_h_, input_w_, output_h_, output_w_;

    void* d_left_ = nullptr;
    void* d_right_ = nullptr;
    void* d_output_ = nullptr;

    std::vector<float> output_;
};

//==============================================================================
// Disparity Visualization
//==============================================================================
cv::Mat colorizeDisparity(const std::vector<float>& disp, int h, int w, float max_disp = 192.0f) {
    cv::Mat disp_norm(h, w, CV_8UC1);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float d = std::max(0.0f, std::min(disp[y * w + x] / max_disp, 1.0f));
            disp_norm.at<uchar>(y, x) = static_cast<uchar>(d * 255);
        }
    }

    cv::Mat colored;
    cv::applyColorMap(disp_norm, colored, cv::COLORMAP_JET);
    return colored;
}

//==============================================================================
// Main Application
//==============================================================================
void printUsage(const char* prog) {
    std::cerr << "Stereo Camera Inference\n\n"
              << "Usage: " << prog << " <engine_path> <camera> [options]\n\n"
              << "Arguments:\n"
              << "  engine_path    TensorRT engine file (.engine)\n"
              << "  camera         Camera resource (e.g., v4l2:///dev/video1)\n\n"
              << "Options:\n"
              << "  --width W      Stereo frame width (default: 3840)\n"
              << "  --height H     Stereo frame height (default: 1200)\n"
              << "  --fps F        Framerate (default: 30)\n"
              << "  --warmup N     Warmup iterations (default: 10)\n"
              << "  --no-show      Disable display\n"
              << "  --save PATH    Save disparity frames to directory\n"
              << "  -v, --verbose  Verbose output\n";
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    // Parse arguments
    std::string engine_path = argv[1];
    std::string camera_resource = argv[2];

    int stereo_width = 3840;
    int stereo_height = 1200;
    int framerate = 30;
    int warmup_iters = 10;
    bool show_display = true;
    bool verbose = false;
    std::string save_path;

    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--width" && i + 1 < argc) stereo_width = std::stoi(argv[++i]);
        else if (arg == "--height" && i + 1 < argc) stereo_height = std::stoi(argv[++i]);
        else if (arg == "--fps" && i + 1 < argc) framerate = std::stoi(argv[++i]);
        else if (arg == "--warmup" && i + 1 < argc) warmup_iters = std::stoi(argv[++i]);
        else if (arg == "--no-show") show_display = false;
        else if (arg == "--save" && i + 1 < argc) save_path = argv[++i];
        else if (arg == "-v" || arg == "--verbose") verbose = true;
        else {
            std::cerr << "Unknown option: " << arg << std::endl;
            return 1;
        }
    }

    int single_width = stereo_width / 2;
    int single_height = stereo_height;

    // Validate engine file
    if (!fileExists(engine_path)) {
        std::cerr << "Error: Engine file not found: " << engine_path << std::endl;
        return 1;
    }

    // Setup signal handler
    signal(SIGINT, signalHandler);

    std::cout << "========== Stereo Camera Inference ==========" << std::endl;
    std::cout << "Engine: " << engine_path << std::endl;
    std::cout << "Camera: " << camera_resource << std::endl;
    std::cout << "Resolution: " << stereo_width << "x" << stereo_height
              << " (single: " << single_width << "x" << single_height << ")" << std::endl;
    std::cout << "=============================================" << std::endl;

    try {
        // Initialize TensorRT engine
        StereoEngine engine(engine_path);

        // Initialize preprocessors
        Preprocessor left_preproc(engine.inputHeight(), engine.inputWidth());
        Preprocessor right_preproc(engine.inputHeight(), engine.inputWidth());

        // Setup camera
        videoOptions opts;
        opts.width = stereo_width;
        opts.height = stereo_height;
        opts.frameRate = static_cast<float>(framerate);
        opts.zeroCopy = true;

        videoSource* camera = videoSource::Create(camera_resource.c_str(), opts);
        if (!camera || !camera->Open()) {
            std::cerr << "Error: Cannot open camera: " << camera_resource << std::endl;
            return 1;
        }

        std::cout << "Camera opened successfully" << std::endl;

        // Warmup
        std::cout << "Warming up (" << warmup_iters << " iterations)..." << std::endl;
        void* frame = nullptr;
        if (camera->Capture(&frame, IMAGE_RGB8, 5000)) {
            uchar3* rgb = (uchar3*)frame;

            cv::Mat left_rgb(single_height, single_width, CV_8UC3);
            cv::Mat right_rgb(single_height, single_width, CV_8UC3);

            for (int y = 0; y < single_height; y++) {
                memcpy(left_rgb.ptr(y), rgb + y * stereo_width + single_width, single_width * 3);
                memcpy(right_rgb.ptr(y), rgb + y * stereo_width, single_width * 3);
            }

            left_rgb.convertTo(left_rgb, CV_32FC3);
            right_rgb.convertTo(right_rgb, CV_32FC3);

            const auto& left_data = left_preproc.process(left_rgb);
            const auto& right_data = right_preproc.process(right_rgb);

            engine.warmup(left_data, right_data, warmup_iters);
            std::cout << "Warmup complete" << std::endl;
        }

        // Create display windows
        if (show_display) {
            cv::namedWindow("Left", cv::WINDOW_NORMAL);
            cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
            cv::resizeWindow("Left", 640, 400);
            cv::resizeWindow("Disparity", 640, 400);
        }

        // Create save directory
        if (!save_path.empty()) {
            std::string cmd = "mkdir -p " + save_path;
            (void)system(cmd.c_str());
        }

        // Main loop
        int frame_count = 0;
        double total_time = 0;

        std::cout << "Running... (Ctrl+C or 'q' to exit)" << std::endl;

        while (!g_signal_received) {
            void* next_frame = nullptr;
            if (!camera->Capture(&next_frame, IMAGE_RGB8, 1000)) {
                if (!camera->IsStreaming()) break;
                continue;
            }

            auto t_start = std::chrono::high_resolution_clock::now();

            uchar3* rgb = (uchar3*)next_frame;

            // Split stereo frame (left=right half, right=left half for typical stereo cameras)
            cv::Mat left_rgb(single_height, single_width, CV_8UC3);
            cv::Mat right_rgb(single_height, single_width, CV_8UC3);

            for (int y = 0; y < single_height; y++) {
                memcpy(left_rgb.ptr(y), rgb + y * stereo_width + single_width, single_width * 3);
                memcpy(right_rgb.ptr(y), rgb + y * stereo_width, single_width * 3);
            }

            left_rgb.convertTo(left_rgb, CV_32FC3);
            right_rgb.convertTo(right_rgb, CV_32FC3);

            // Preprocess
            const auto& left_data = left_preproc.process(left_rgb);
            const auto& right_data = right_preproc.process(right_rgb);

            // Inference
            engine.infer(left_data, right_data);

            auto t_end = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            total_time += ms;
            frame_count++;

            double fps = 1000.0 / ms;

            // Print stats periodically
            if (frame_count % 30 == 0) {
                std::cout << std::fixed << std::setprecision(1)
                          << "Frame " << frame_count
                          << " | " << ms << "ms"
                          << " | " << fps << " FPS" << std::endl;
            }

            // Display
            if (show_display) {
                cv::Mat disp_color = colorizeDisparity(engine.getOutput(),
                                                        engine.outputHeight(),
                                                        engine.outputWidth());

                // Add FPS overlay
                cv::putText(disp_color, "FPS: " + std::to_string((int)fps),
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                           1.0, cv::Scalar(255, 255, 255), 2);

                // Convert for display
                cv::Mat left_display;
                left_rgb.convertTo(left_display, CV_8UC3);
                cv::cvtColor(left_display, left_display, cv::COLOR_RGB2BGR);

                cv::imshow("Left", left_display);
                cv::imshow("Disparity", disp_color);

                int key = cv::waitKey(1);
                if (key == 'q' || key == 27) break;
                if (key == 's') {
                    std::string fname = "disparity_" + std::to_string(frame_count) + ".png";
                    cv::imwrite(fname, disp_color);
                    std::cout << "Saved: " << fname << std::endl;
                }
            }

            // Save frames
            if (!save_path.empty()) {
                cv::Mat disp_color = colorizeDisparity(engine.getOutput(),
                                                        engine.outputHeight(),
                                                        engine.outputWidth());
                cv::imwrite(save_path + "/disp_" + std::to_string(frame_count) + ".png", disp_color);
            }
        }

        // Summary
        std::cout << "\n========== Summary ==========" << std::endl;
        std::cout << "Frames: " << frame_count << std::endl;
        if (frame_count > 0) {
            std::cout << std::fixed << std::setprecision(2)
                      << "Avg time: " << (total_time / frame_count) << " ms" << std::endl
                      << "Avg FPS: " << (1000.0 * frame_count / total_time) << std::endl;
        }
        std::cout << "=============================" << std::endl;

        // Cleanup
        if (show_display) cv::destroyAllWindows();
        delete camera;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
