/**
 * Fast-FoundationStereo Stereo Depth Node
 *
 * Runs the Fast-FoundationStereo model (EdgeNeXt feature backbone + GWC volume +
 * 3D hourglass + GRU refinement) from a single self-contained TensorRT engine
 * produced by scripts/make_single_onnx.py (no custom plugin required — the
 * GWC volume is built from pure ONNX ops).
 *
 * Pipeline per frame — mirrors Fast-FoundationStereo/scripts/run_demo_single_trt.py:
 *   1. Receive rectified left/right rgb8 images from camera_stereo
 *   2. Resize each side to the model input size (e.g. 320x320) with INTER_AREA
 *   3. Async H2D raw uint8 HWC, then launch a CUDA kernel that converts
 *      HWC u8 -> CHW f32 RGB *and* applies ImageNet normalisation directly
 *      into the engine's 'left_image'/'right_image' buffers. The single-ONNX
 *      export strips the in-graph normalize_image step, so we MUST normalise
 *      externally — see src/cuda/stereo_preproc.cu.
 *   4. enqueueV3, async D2H disparity [1,1,H,W] float32, one stream sync
 *   5. Clamp negative disparities to 0 (matches reference disp.clip(0, None))
 *   6. Disparity -> depth using fx scaled to the resized image
 *
 * useSpinWait equivalent: main() sets cudaDeviceScheduleSpin before any
 * CUDA context is created, so cudaStreamSynchronize spins instead of yielding.
 * Matches trtexec --useSpinWait at the cost of one busy CPU core.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <chrono>
#include <dlfcn.h>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// GPU preprocessing kernel (src/cuda/stereo_preproc.cu)
extern "C" cudaError_t cudaHwcU8ToChwF32Rgb(
    const uint8_t* d_in, float* d_out, int H, int W, cudaStream_t stream);

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
// TensorRT engine wrapper — loads a single Fast-FoundationStereo engine with
// named IO tensors ("left_image", "right_image", "disparity").
//==============================================================================
class FFStereoEngine {
public:
    FFStereoEngine(const std::string& engine_path, const std::string& plugin_path) {
        // Plugin loading is optional now. The single-ONNX export produces a
        // fully self-contained engine with no custom ops, so we only dlopen
        // if the caller explicitly passes a plugin path (kept for backwards
        // compatibility with the legacy two-stage GWC-plugin engine).
        if (!plugin_path.empty()) {
            void* handle = dlopen(plugin_path.c_str(), RTLD_NOW);
            if (!handle) {
                throw std::runtime_error("dlopen plugin failed: " + std::string(dlerror()));
            }
        }

        std::ifstream f(engine_path, std::ios::binary | std::ios::ate);
        if (!f) {
            throw std::runtime_error("Cannot open engine file: " + engine_path);
        }
        std::streamsize size = f.tellg();
        f.seekg(0, std::ios::beg);
        std::vector<char> blob(size);
        if (!f.read(blob.data(), size)) {
            throw std::runtime_error("Failed reading engine: " + engine_path);
        }

        runtime_ = nvinfer1::createInferRuntime(g_trt_logger);
        engine_  = runtime_->deserializeCudaEngine(blob.data(), blob.size());
        if (!engine_) {
            throw std::runtime_error("deserializeCudaEngine returned null — "
                                     "engine/TRT version mismatch, or a legacy "
                                     "engine that still needs libgwc_plugin.so");
        }
        context_ = engine_->createExecutionContext();
        if (!context_) {
            throw std::runtime_error("Failed to create execution context");
        }

        // Allocate device buffers from each IO tensor's static shape.
        int n = engine_->getNbIOTensors();
        for (int i = 0; i < n; ++i) {
            const char* name = engine_->getIOTensorName(i);
            auto dims  = engine_->getTensorShape(name);
            auto dtype = engine_->getTensorDataType(name);
            size_t bytes = dtypeSize(dtype);
            for (int j = 0; j < dims.nbDims; ++j) bytes *= static_cast<size_t>(dims.d[j]);

            void* ptr = nullptr;
            cudaMalloc(&ptr, bytes);
            buffers_[name]     = ptr;
            bytes_[name]       = bytes;
            context_->setTensorAddress(name, ptr);
        }

        auto left_dims = engine_->getTensorShape("left_image");
        if (left_dims.nbDims != 4) {
            throw std::runtime_error("Expected 'left_image' tensor to be 4D [B,C,H,W]");
        }
        input_h_ = left_dims.d[2];
        input_w_ = left_dims.d[3];

        auto disp_dims = engine_->getTensorShape("disparity");
        output_h_ = disp_dims.d[disp_dims.nbDims - 2];
        output_w_ = disp_dims.d[disp_dims.nbDims - 1];
        disp_elems_ = static_cast<size_t>(output_h_) * static_cast<size_t>(output_w_);

        cudaStreamCreate(&stream_);
        host_disp_.resize(disp_elems_);

        // Staging buffers for raw uint8 HWC RGB uploads.
        //   h_*_u8_ — pinned (page-locked) host memory, so cudaMemcpyAsync can
        //             run truly asynchronously without staging through the
        //             driver's own bounce buffer.
        //   d_*_u8_ — regular cudaMalloc'd device memory. The preproc kernel
        //             reads from these; keeping them in GPU memory rather than
        //             zero-copy-mapped system memory gives cached reads and
        //             matches the reference stereo_trt.cu implementation.
        const size_t u8_bytes = static_cast<size_t>(input_h_) * input_w_ * 3;
        cudaMallocHost(&h_left_u8_,  u8_bytes);
        cudaMallocHost(&h_right_u8_, u8_bytes);
        cudaMalloc(&d_left_u8_,  u8_bytes);
        cudaMalloc(&d_right_u8_, u8_bytes);

        // cudaEvents for per-stage GPU timing (no extra stream syncs).
        cudaEventCreate(&ev_start_);
        cudaEventCreate(&ev_after_upload_);
        cudaEventCreate(&ev_after_infer_);
        cudaEventCreate(&ev_end_);

        std::cout << "Fast-FoundationStereo engine loaded: input " << input_w_ << "x"
                  << input_h_ << ", disp " << output_w_ << "x" << output_h_ << std::endl;
    }

    ~FFStereoEngine() {
        if (ev_start_)        cudaEventDestroy(ev_start_);
        if (ev_after_upload_) cudaEventDestroy(ev_after_upload_);
        if (ev_after_infer_)  cudaEventDestroy(ev_after_infer_);
        if (ev_end_)          cudaEventDestroy(ev_end_);
        if (stream_) cudaStreamDestroy(stream_);
        if (d_left_u8_)  cudaFree(d_left_u8_);
        if (d_right_u8_) cudaFree(d_right_u8_);
        if (h_left_u8_)  cudaFreeHost(h_left_u8_);
        if (h_right_u8_) cudaFreeHost(h_right_u8_);
        for (auto& kv : buffers_) cudaFree(kv.second);
        if (context_) delete context_;
        if (engine_)  delete engine_;
        if (runtime_) delete runtime_;
    }

    // Fill the pinned host staging buffers from HWC RGB cv::Mat data.
    // The actual H2D copies happen inside infer() on the stream.
    void stage(const uint8_t* left_u8, const uint8_t* right_u8) {
        const size_t u8_bytes = static_cast<size_t>(input_h_) * input_w_ * 3;
        std::memcpy(h_left_u8_,  left_u8,  u8_bytes);
        std::memcpy(h_right_u8_, right_u8, u8_bytes);
    }

    // Run H2D -> preprocess kernel -> engine -> D2H, all on one stream, with
    // per-stage cudaEvents recorded around each phase. A single stream sync
    // at the end blocks only once.
    void infer() {
        const size_t u8_bytes = static_cast<size_t>(input_h_) * input_w_ * 3;
        cudaEventRecord(ev_start_, stream_);

        cudaMemcpyAsync(d_left_u8_,  h_left_u8_,  u8_bytes, cudaMemcpyHostToDevice, stream_);
        cudaMemcpyAsync(d_right_u8_, h_right_u8_, u8_bytes, cudaMemcpyHostToDevice, stream_);

        // Preprocessing kernel reads cached device memory and writes CHW
        // float32 directly into the engine's input buffers.
        cudaHwcU8ToChwF32Rgb(d_left_u8_,
                             reinterpret_cast<float*>(buffers_["left_image"]),
                             input_h_, input_w_, stream_);
        cudaHwcU8ToChwF32Rgb(d_right_u8_,
                             reinterpret_cast<float*>(buffers_["right_image"]),
                             input_h_, input_w_, stream_);
        cudaEventRecord(ev_after_upload_, stream_);

        context_->enqueueV3(stream_);
        cudaEventRecord(ev_after_infer_, stream_);

        cudaMemcpyAsync(host_disp_.data(), buffers_["disparity"],
                        disp_elems_ * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
        cudaEventRecord(ev_end_, stream_);

        cudaStreamSynchronize(stream_);

        // Resolve timings (these calls are free once the events have fired).
        cudaEventElapsedTime(&last_upload_ms_, ev_start_,        ev_after_upload_);
        cudaEventElapsedTime(&last_infer_ms_,  ev_after_upload_, ev_after_infer_);
        cudaEventElapsedTime(&last_d2h_ms_,    ev_after_infer_,  ev_end_);
    }

    // Convenience entry point used by the node callback: stage + infer.
    void stage_and_infer(const uint8_t* left_u8, const uint8_t* right_u8) {
        stage(left_u8, right_u8);
        infer();
    }

    // Run a few iterations with zero-filled inputs to eat the first-inference
    // setup cost. Called once at node startup, outside the ROS hot path.
    void warmup(int iterations) {
        const size_t u8_bytes = static_cast<size_t>(input_h_) * input_w_ * 3;
        std::memset(h_left_u8_,  0, u8_bytes);
        std::memset(h_right_u8_, 0, u8_bytes);
        for (int i = 0; i < iterations; ++i) {
            infer();
        }
    }

    float lastUploadMs() const { return last_upload_ms_; }
    float lastInferMs()  const { return last_infer_ms_;  }
    float lastD2hMs()    const { return last_d2h_ms_;    }

    const std::vector<float>& disparity() const { return host_disp_; }
    int inputHeight()  const { return input_h_; }
    int inputWidth()   const { return input_w_; }
    int outputHeight() const { return output_h_; }
    int outputWidth()  const { return output_w_; }

private:
    static size_t dtypeSize(nvinfer1::DataType t) {
        switch (t) {
            case nvinfer1::DataType::kFLOAT: return 4;
            case nvinfer1::DataType::kHALF:  return 2;
            case nvinfer1::DataType::kINT32: return 4;
            case nvinfer1::DataType::kINT8:  return 1;
            case nvinfer1::DataType::kBOOL:  return 1;
            default: return 4;
        }
    }

    nvinfer1::IRuntime*          runtime_ = nullptr;
    nvinfer1::ICudaEngine*       engine_  = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t                 stream_  = nullptr;

    std::map<std::string, void*>  buffers_;
    std::map<std::string, size_t> bytes_;

    // Pinned host + device u8 staging buffers (one pair per eye).
    uint8_t* h_left_u8_  = nullptr;
    uint8_t* h_right_u8_ = nullptr;
    uint8_t* d_left_u8_  = nullptr;
    uint8_t* d_right_u8_ = nullptr;

    // Per-stage GPU timing (filled in by infer()).
    cudaEvent_t ev_start_        = nullptr;
    cudaEvent_t ev_after_upload_ = nullptr;
    cudaEvent_t ev_after_infer_  = nullptr;
    cudaEvent_t ev_end_          = nullptr;
    float last_upload_ms_ = 0.f;
    float last_infer_ms_  = 0.f;
    float last_d2h_ms_    = 0.f;


    int input_h_ = 0, input_w_ = 0;
    int output_h_ = 0, output_w_ = 0;
    size_t disp_elems_ = 0;
    std::vector<float> host_disp_;
};

//==============================================================================
// ROS2 Stereo Depth Node
//==============================================================================
class StereoDepthNode : public rclcpp::Node
{
public:
    StereoDepthNode() : Node("stereo_depth")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Fast-FoundationStereo depth node...");

        this->declare_parameter("engine",  "stereo-depth/fast_foundationstereo.engine");
        // Plugin path empty by default — the single-ONNX engine is self-contained.
        // Set to "stereo-depth/libgwc_plugin.so" if loading a legacy 2-stage engine.
        this->declare_parameter("plugin",  "");
        this->declare_parameter("max_disparity", 64.0);
        this->declare_parameter("warmup_iterations", 5);
        this->declare_parameter("baseline_mm", 100.0);
        this->declare_parameter("focal_length_mm", 3.6);
        this->declare_parameter("sensor_width_mm", 5.76);
        this->declare_parameter("color_publish_size", 480);
        // Empirical multiplier applied to the computed depth (depth = scale × bf/d).
        // Use this to compensate for systematic depth bias when the model
        // produces inflated disparity at far range — e.g. small-baseline rigs
        // where 10 m objects fall to ~4 px true disparity and the GRU pulls
        // them toward a nearer prior. Default 1.0 (no scaling). If depth reads
        // 10× too small uniformly, set 10.0. Validate against ground truth at
        // both near AND far before committing — if the scale fixes far but
        // breaks near, the model is non-linearly biased and you need a richer
        // fix (different engine, higher input resolution, etc.).
        this->declare_parameter("depth_scale", 1.0);
        // Temporal EMA smoothing on the per-frame disparity:
        //   disp_t = α·disp_{t-1} + (1-α)·disp_t
        // α=0   → no smoothing (raw frames, visibly jittery on static scenes)
        // α=0.7 → moderate smoothing (sweet spot for most scenes)
        // α=0.9 → heavy smoothing (very smooth, noticeable motion blur)
        this->declare_parameter("ema_alpha", 0.7);
        // FIXED disparity range (in MODEL-resolution pixels) for the colormap.
        // Auto-range per frame is visually jittery — even on a static scene the
        // model's min/max wobble each frame and remap the whole TURBO palette.
        // A fixed range keeps colours stable. Tune disp_vmax to spread the
        // distance band of interest across the colour spectrum:
        //   disp_vmax = 28  → outdoor scenes (mid-far objects in mid-range colours)
        //   disp_vmax = 64  → near scenes (close objects saturate red later)
        // Pixels with disparity >= disp_vmax saturate at the red end of TURBO.
        this->declare_parameter("disp_vmin", 0.0);
        this->declare_parameter("disp_vmax", 28.0);
        // Resolution at which /stereo_depth/depth is published.
        //   0  = native left-image resolution (backwards compatible; GUI and
        //        other subscribers expect this).
        //   N  = publish at an N×N float map (e.g. 480 to match the model's
        //        native output and skip the ~10-15 ms upsize + ~5.76 MB
        //        serialise on the hot path).
        // Kept at 0 by default — opt in explicitly per topology.
        this->declare_parameter("depth_publish_size", 0);

        const std::string engine_rel = this->get_parameter("engine").as_string();
        const std::string plugin_rel = this->get_parameter("plugin").as_string();
        max_disparity_     = this->get_parameter("max_disparity").as_double();
        warmup_iterations_ = this->get_parameter("warmup_iterations").as_int();
        baseline_m_        = this->get_parameter("baseline_mm").as_double() / 1000.0;
        focal_length_mm_   = this->get_parameter("focal_length_mm").as_double();
        sensor_width_mm_   = this->get_parameter("sensor_width_mm").as_double();
        color_publish_size_ = this->get_parameter("color_publish_size").as_int();
        depth_scale_       = this->get_parameter("depth_scale").as_double();
        ema_alpha_         = this->get_parameter("ema_alpha").as_double();
        disp_vmin_         = this->get_parameter("disp_vmin").as_double();
        disp_vmax_         = this->get_parameter("disp_vmax").as_double();
        depth_publish_size_ = this->get_parameter("depth_publish_size").as_int();

        const std::string graphs_path =
            ament_index_cpp::get_package_share_directory("visionconnect") + "/graphs/";
        const std::string engine_path = graphs_path + engine_rel;
        // Resolve plugin only if the parameter is non-empty; the single-ONNX
        // engine doesn't need one. Passing an empty string here makes
        // FFStereoEngine skip the dlopen entirely.
        const std::string plugin_path = plugin_rel.empty() ? "" : (graphs_path + plugin_rel);

        if (!plugin_path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Loading plugin: %s", plugin_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "No plugin configured (single-ONNX engine)");
        }
        RCLCPP_INFO(this->get_logger(), "Loading engine: %s", engine_path.c_str());

        try {
            engine_ = std::make_unique<FFStereoEngine>(engine_path, plugin_path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load Fast-FoundationStereo engine: %s", e.what());
            throw;
        }

        model_h_ = engine_->inputHeight();
        model_w_ = engine_->inputWidth();
        disp_h_  = engine_->outputHeight();
        disp_w_  = engine_->outputWidth();

        // Pre-allocate preprocessing buffers (model-resolution uint8 RGB HWC).
        // These are fed raw into the engine's GPU preprocessing kernel.
        left_resized_.create(model_h_, model_w_, CV_8UC3);
        right_resized_.create(model_h_, model_w_, CV_8UC3);

        // Pre-allocate disparity -> depth working buffer at model resolution.
        depth_model_.create(disp_h_, disp_w_, CV_32FC1);

        // Run warmup NOW, before spinning — this bakes out the first-inference
        // setup cost (cuDNN/cuBLAS/TRT plan finalisation, kernel JIT, etc.) so
        // the first ROS callback sees steady-state timings instead of a 1+ sec
        // stall that backs up the subscriber queue.
        RCLCPP_INFO(this->get_logger(), "Warming up TensorRT engine (%d iterations)...",
                    warmup_iterations_);
        engine_->warmup(warmup_iterations_);
        RCLCPP_INFO(this->get_logger(),
                    "Warmup complete — last infer %.1f ms",
                    engine_->lastInferMs());

        // Publishers
        rclcpp::QoS qos_pub(1);
        qos_pub.best_effort();
        qos_pub.durability_volatile();
        depth_pub_       = this->create_publisher<sensor_msgs::msg::Image>("depth", qos_pub);
        depth_color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_color", qos_pub);

        // Synchronized subscribers
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

        RCLCPP_INFO(this->get_logger(), "Fast-FoundationStereo depth node ready");
    }

private:
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        const auto t0 = std::chrono::steady_clock::now();

        // Wrap incoming messages zero-copy, then center-crop a min(w,h) square
        // per eye before resizing to the engine's square input. FFS was trained
        // on natural-aspect pairs; an anisotropic resize (1440×900 → 320×320 =
        // 4.5× horizontal vs 2.8× vertical) makes the model output inflated
        // disparity and collapsed depth ~10× (10 m car read as 1 m). The square
        // crop keeps the resize isotropic and restores correct depth. ROI is a
        // zero-copy view; net CPU is LOWER than the un-cropped path because
        // cv::resize operates on 0.81 MP instead of 1.30 MP per eye.
        // sensor_width_mm in config must reflect the cropped span:
        //   5.76 × (eye_h / 1920)  → 2.7 mm for the 1440×900 → 900×900 path.
        cv::Mat left_full  = msgToMatNoCopy(left_msg);
        cv::Mat right_full = msgToMatNoCopy(right_msg);

        const int crop  = std::min(left_full.cols, left_full.rows);
        const int x_off = (left_full.cols - crop) / 2;
        const int y_off = (left_full.rows - crop) / 2;
        const cv::Rect roi(x_off, y_off, crop, crop);
        cv::Mat left  = left_full(roi);
        cv::Mat right = right_full(roi);

        cv::Mat left_rgb, right_rgb;
        if (left_msg->encoding == "rgb8") {
            left_rgb  = left;
            right_rgb = right;
        } else if (left_msg->encoding == "bgr8") {
            cv::cvtColor(left,  left_rgb,  cv::COLOR_BGR2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_BGR2RGB);
        } else if (left.channels() == 1) {
            cv::cvtColor(left,  left_rgb,  cv::COLOR_GRAY2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_GRAY2RGB);
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(),
                             "Unknown encoding '%s', assuming BGR",
                             left_msg->encoding.c_str());
            cv::cvtColor(left,  left_rgb,  cv::COLOR_BGR2RGB);
            cv::cvtColor(right, right_rgb, cv::COLOR_BGR2RGB);
        }

        // Resize the full eye directly to model input (anisotropic for the
        // 1440×900 case). INTER_AREA is the right choice for this heavy
        // downsample (~4.5× horizontal, ~2.8× vertical); INTER_LINEAR aliases
        // noticeably at these ratios. The reference uses INTER_LINEAR but at
        // a much smaller resize ratio where the difference is negligible.
        cv::resize(left_rgb,  left_resized_,  cv::Size(model_w_, model_h_), 0, 0, cv::INTER_AREA);
        cv::resize(right_rgb, right_resized_, cv::Size(model_w_, model_h_), 0, 0, cv::INTER_AREA);

        const auto t_preproc = std::chrono::steady_clock::now();

        // stage writes into cudaHostAllocMapped buffers (zero-copy on Jetson),
        // then infer runs preproc kernel -> engine -> D2H -> single sync,
        // populating cudaEvent timings we can read back below.
        engine_->stage_and_infer(left_resized_.data, right_resized_.data);

        const auto t_infer = std::chrono::steady_clock::now();

        // Wrap disparity in a Mat (no copy — points at the host buffer).
        const auto& disp_out = engine_->disparity();
        cv::Mat disp_model(disp_h_, disp_w_,
                           CV_32FC1, const_cast<float*>(disp_out.data()));

        // Clamp negatives to 0 — matches reference disp.clip(0, None)
        // (Fast-FoundationStereo/scripts/run_demo_single_trt.py line 292).
        cv::max(disp_model, 0.0f, disp_model);

        // Temporal EMA to suppress per-frame model jitter on static scenes.
        // First frame seeds the buffer; subsequent frames blend in place.
        //   disp_model = α·disp_prev_ + (1-α)·disp_model
        // disp_model wraps the engine's host output buffer, so writing back to
        // it is fine — the engine overwrites it on every infer().
        if (ema_alpha_ > 0.0) {
            if (disp_prev_.empty() ||
                disp_prev_.rows != disp_h_ || disp_prev_.cols != disp_w_) {
                disp_prev_.create(disp_h_, disp_w_, CV_32FC1);
                disp_model.copyTo(disp_prev_);
            } else {
                cv::addWeighted(disp_prev_, ema_alpha_,
                                disp_model, 1.0 - ema_alpha_, 0.0,
                                disp_model);
                disp_model.copyTo(disp_prev_);
            }
        }

        // One-time initialization of depth conversion constants + output messages.
        if (!depth_initialized_) {
            // fx is computed at the MODEL input resolution: depth is computed in
            // that space and then resized up to the source resolution. This is
            // the numerically correct approach per Fast-FoundationStereo README §7.5.
            focal_length_px_model_ = (focal_length_mm_ / sensor_width_mm_) * (double)model_w_;
            bf_model_ = baseline_m_ * focal_length_px_model_;
            RCLCPP_INFO(this->get_logger(),
                        "Depth params: fx(model)=%.2f px, baseline=%.3f m, bf=%.4f, depth_scale=%.3f",
                        focal_length_px_model_, baseline_m_, bf_model_, depth_scale_);

            // depth_publish_size=0 means "native source resolution", otherwise
            // publish at an exact square size (default 480, matching the model).
            const int depth_rows = (depth_publish_size_ > 0) ? depth_publish_size_ : left.rows;
            const int depth_cols = (depth_publish_size_ > 0) ? depth_publish_size_ : left.cols;

            depth_msg_.header.frame_id = "stereo_depth";
            depth_msg_.encoding        = "32FC1";
            depth_msg_.is_bigendian    = false;
            depth_msg_.height          = depth_rows;
            depth_msg_.width           = depth_cols;
            depth_msg_.step            = depth_cols * sizeof(float);
            depth_msg_.data.resize(depth_rows * depth_cols * sizeof(float));

            const int pub_size = color_publish_size_;
            disp_small_.create(pub_size, pub_size, CV_32FC1);
            disp_norm_small_.create(pub_size, pub_size, CV_8UC1);
            zero_mask_.create(pub_size, pub_size, CV_8UC1);

            depth_color_msg_.header.frame_id = "stereo_depth";
            depth_color_msg_.encoding        = "bgr8";
            depth_color_msg_.is_bigendian    = false;
            depth_color_msg_.height          = pub_size;
            depth_color_msg_.width           = pub_size;
            depth_color_msg_.step            = pub_size * 3;
            depth_color_msg_.data.resize(pub_size * pub_size * 3);

            depth_initialized_ = true;
        }

        // Depth topic: compute depth from disparity only when someone cares.
        const bool publish_depth = (depth_pub_->get_subscription_count() > 0);
        if (publish_depth) {
            // Disparity → depth at model resolution. The clip(0)+shadow filter
            // already zeroed unphysical pixels; here we only guard against
            // sub-pixel disparity values that would blow up bf/d toward
            // infinity (treated as no-data → 0 m).
            for (int r = 0; r < disp_h_; ++r) {
                const float* disp_row  = disp_model.ptr<float>(r);
                float*       depth_row = depth_model_.ptr<float>(r);
                for (int c = 0; c < disp_w_; ++c) {
                    float d = disp_row[c];
                    depth_row[c] = (d > 1.0f) ? static_cast<float>(depth_scale_ * bf_model_ / d) : 0.0f;
                }
            }
            // Write into the pre-allocated depth_msg_ buffer. When publishing
            // at model resolution (the default), the resize is a no-op copy;
            // when opting into source-res publishing, it's a 480 -> source
            // bilinear upsize of a float image (~10-15 ms).
            cv::Mat depth_out(depth_msg_.height, depth_msg_.width, CV_32FC1,
                              depth_msg_.data.data());
            if (depth_out.rows == depth_model_.rows && depth_out.cols == depth_model_.cols) {
                std::memcpy(depth_out.data, depth_model_.data,
                            static_cast<size_t>(depth_out.rows) * depth_out.cols * sizeof(float));
            } else {
                cv::resize(depth_model_, depth_out, depth_out.size(), 0, 0, cv::INTER_LINEAR);
            }
            depth_msg_.header.stamp = left_msg->header.stamp;
            depth_pub_->publish(depth_msg_);
        }

        // Colorize disparity with a FIXED [disp_vmin_, disp_vmax_] range so
        // the colormap is stable frame-to-frame. Per-frame min/max auto-range
        // (what the reference Utils.vis_disparity does) is visually jittery —
        // tiny model fluctuations remap the whole TURBO palette on a static
        // scene. With the fixed range:
        //     u8 = (255 / (vmax-vmin)) · disp  +  (-vmin · 255 / (vmax-vmin))
        //     pixels < vmin → blue end; pixels > vmax → red end
        const int pub_size = color_publish_size_;
        cv::resize(disp_model, disp_small_, cv::Size(pub_size, pub_size), 0, 0, cv::INTER_AREA);

        const double rng = std::max(1e-6, disp_vmax_ - disp_vmin_);
        const double disp_scale = 255.0 / rng;
        const double disp_bias  = -disp_vmin_ * disp_scale;
        disp_small_.convertTo(disp_norm_small_, CV_8UC1, disp_scale, disp_bias);

        cv::Mat depth_color(pub_size, pub_size, CV_8UC3, depth_color_msg_.data.data());
        cv::applyColorMap(disp_norm_small_, depth_color, cv::COLORMAP_TURBO);

        // Diagnostic min/max of the displayed disparity (helps tune disp_vmax_).
        double dmin = 0.0, dmax = 0.0;
        cv::minMaxLoc(disp_small_, &dmin, &dmax);

        depth_color_msg_.header.stamp = left_msg->header.stamp;
        depth_color_pub_->publish(depth_color_msg_);

        const auto t_end = std::chrono::steady_clock::now();
        const double ms_preproc = std::chrono::duration<double, std::milli>(t_preproc - t0).count();
        const double ms_infer_wall = std::chrono::duration<double, std::milli>(t_infer - t_preproc).count();
        const double ms_post    = std::chrono::duration<double, std::milli>(t_end - t_infer).count();
        const double ms_total   = std::chrono::duration<double, std::milli>(t_end - t0).count();
        if (frame_id_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[profile] cpu_pre=%.1f  gpu_wall=%.1f [prep=%.1f trt=%.1f d2h=%.1f]  post=%.1f  total=%.1f ms  disp[%.1f,%.1f]  pub_depth=%d",
                ms_preproc, ms_infer_wall,
                engine_->lastUploadMs(), engine_->lastInferMs(), engine_->lastD2hMs(),
                ms_post, ms_total, dmin, dmax, publish_depth ? 1 : 0);
        }
        ++frame_id_;
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

    std::unique_ptr<FFStereoEngine> engine_;

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_color_pub_;

    // Model geometry (filled after engine load)
    int model_h_ = 0, model_w_ = 0;
    int disp_h_ = 0,  disp_w_ = 0;

    double max_disparity_     = 192.0;
    int    warmup_iterations_ = 5;
    int    depth_publish_size_ = 480;

    // Depth conversion
    double baseline_m_, focal_length_mm_, sensor_width_mm_;
    double focal_length_px_model_ = 0.0, bf_model_ = 0.0;
    double depth_scale_           = 1.0;   // empirical multiplier on computed depth
    bool   depth_initialized_     = false;

    // Smoothing + colormap tuning.
    double ema_alpha_  = 0.7;     // 0 disables; 0.7 is the live_trt.py default
    double disp_vmin_  = 0.0;     // colormap blue end (model-disp pixels)
    double disp_vmax_  = 28.0;    // colormap red end  (model-disp pixels)
    cv::Mat disp_prev_;           // previous-frame disparity for EMA

    // Pre-allocated preprocessing buffers (model-resolution uint8 RGB HWC).
    cv::Mat left_resized_;
    cv::Mat right_resized_;

    // Rolling frame counter for timing log cadence.
    size_t frame_id_ = 0;

    // Pre-allocated postprocessing buffers
    cv::Mat depth_model_;     // CV_32FC1 — model-resolution depth
    cv::Mat disp_small_;      // CV_32FC1 — resized disparity for colorization
    cv::Mat disp_norm_small_; // CV_8UC1  — normalized for colormap
    cv::Mat zero_mask_;       // CV_8UC1  — pre-allocated mask
    int     color_publish_size_ = 480;

    sensor_msgs::msg::Image depth_msg_;
    sensor_msgs::msg::Image depth_color_msg_;
};

int main(int argc, char** argv)
{
    // useSpinWait equivalent: ask the CUDA driver to spin (busy-wait) when
    // blocking on the device. This is the C++ runtime analogue of trtexec's
    // --useSpinWait flag — cudaStreamSynchronize() will spin instead of
    // yielding via syscalls, removing scheduler jitter from the per-frame
    // latency. Costs one busy CPU core while the GPU is running, but the
    // stereo depth node has its own thread on a dedicated executor so that's
    // an acceptable trade for low/stable latency on Jetson.
    //
    // MUST be called before any CUDA API call that creates a context (i.e.
    // before the FFStereoEngine constructor runs).
    cudaError_t spin_err = cudaSetDeviceFlags(cudaDeviceScheduleSpin);
    if (spin_err != cudaSuccess && spin_err != cudaErrorSetOnActiveProcess) {
        std::cerr << "[stereo_depth] warning: cudaSetDeviceFlags(Spin) failed: "
                  << cudaGetErrorString(spin_err) << std::endl;
    }

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
