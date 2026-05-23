/*
 * Stereo Depth Node — Fast-FoundationStereo (FFS) variant.
 *
 * Functionally identical to node_stereo_depth.cpp (LightStereo) — the C++
 * pipeline is engine-agnostic and auto-discovers IO binding shape from
 * the loaded engine. Built as a separate binary so both engines can
 * coexist and so the launch graph picks the model by executable name.
 *
 * FFS specifics (from fast_foundationstereo.yaml):
 *   - input shape: 320×320 (square, vs LightStereo's 320×512)
 *   - max_disp:    64        (vs 192 for LightStereo) — limits the
 *                            closest measurable depth to ~0.46 m at
 *                            100 mm baseline
 *   - normalize:   false     (cross-validated empirically; engine
 *                            handles any required input scaling)
 *
 * Pipeline (per frame, after L/R image sync):
 *   1. Two raw RGB uchar3 images from /camera_stereo/{left,right}/image_raw
 *      get cudaMemcpyAsync'd to GPU (one allocation per eye, reused frame to
 *      frame).
 *   2. A single fused CUDA kernel per eye (cudaStereoRectifyResizeNormCHW)
 *      consumes the raw image + the pre-baked float2 remap (model-resolution)
 *      and writes directly into the TRT engine's input binding: rectified,
 *      resized to model dims, /255, HWC→CHW.
 *   3. TRT enqueueV3 runs FFS on a high-priority CUDA stream; disparity at
 *      (1, 1, model_h, model_w) lands in another device buffer.
 *   4. cudaMemcpyAsync brings disparity back to host. cv::resize lifts it to
 *      eye resolution (matching the published /camera/raw geometry) with a
 *      proportional disparity scale, then computes Z = bf / disp.
 *   5. Two messages published: 32FC1 depth at eye resolution + an INFERNO
 *      colormap (BGR8) at depth_color_publish_w wide for cheap viz.
 *
 * The rectification calibration is loaded from stereo_calib.yaml via the
 * `calibration_file` ROS parameter. Bake-once init builds the two model-
 * resolution remaps with cv::initUndistortRectifyMap using a Krect scaled
 * from full eye res down to the engine input size — so the kernel does the
 * combined undistort + rectify + downscale in one bilinear lookup.
 */

#include "ros_compat.h"

#include <sensor_msgs/msg/image.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <chrono>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Fused remap + resize + ImageNet-norm + HWC→CHW kernel (src/cuda/stereo_rectify.cu).
extern "C" cudaError_t cudaStereoRectifyResizeNormCHW(
    const uchar3* raw_rgb_or_bgr,
    int raw_w, int raw_h,
    const float2* remap_map,
    int model_w, int model_h,
    float* out_chw_f32,
    float mean_r, float mean_g, float mean_b,
    float inv_std_r, float inv_std_g, float inv_std_b,
    int channel_order_bgr,
    cudaStream_t stream);

namespace {

// TensorRT logger — same pattern as node_driver_monitor.
class TRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) ROS_WARN("[TensorRT] %s", msg);
    }
} g_logger;

// This specific engine appears to have its input normalization baked into
// the ONNX graph (the OpenStereo deploy/export.py exports BARE — without
// normalization — so whoever built this engine added preprocessing layers
// in a custom export step). Applying ImageNet (mean/std) externally on
// top double-normalizes and pushes disparities significantly too high.
//
// Cross-validated WITHOUT vs WITH normalization on the new calibration:
//
//   preprocess         LS/SGBM disp ratio (median)
//   /255 only          1.017     ← agrees with SGBM ground truth
//   /255 + ImageNet    1.696     ← 70 % too high; saturates close
//
// And on KITTI samples:
//   KITTI 000007 (close-range scene):
//      /255 only        → median 6.25 m, proper sky/foreground ordering
//      /255 + ImageNet  → median 2.36 m, sky band = foreground band (collapsed)
//
// Keep mean=0, std=1 so the fused CUDA kernel just does x/255 — the
// engine handles any further normalization internally.
constexpr float kImageNetMean[3] = {0.0f, 0.0f, 0.0f};
constexpr float kImageNetStd[3]  = {1.0f, 1.0f, 1.0f};

inline cv::Mat matFromNode(const YAML::Node& node) {
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    std::vector<double> data = node["data"].as<std::vector<double>>();
    cv::Mat m(rows, cols, CV_64F);
    std::memcpy(m.data, data.data(), data.size() * sizeof(double));
    return m;
}

// Load engine file from disk into a deserialized ICudaEngine.
nvinfer1::ICudaEngine* loadEngine(nvinfer1::IRuntime* runtime, const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) {
        ROS_ERROR("stereo_depth: engine_file_path not readable: %s", path.c_str());
        return nullptr;
    }
    f.seekg(0, std::ios::end);
    size_t size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> bytes(size);
    f.read(bytes.data(), size);
    return runtime->deserializeCudaEngine(bytes.data(), size);
}

} // namespace

class StereoDepthNode : public rclcpp::Node {
public:
    StereoDepthNode() : rclcpp::Node("stereo_depth_ffs") {
        // ────────────────────────────── parameters
        declare_parameter("engine_file_path",      std::string(""));
        declare_parameter("calibration_file",      std::string(""));
        declare_parameter("depth_vmax",            20.0);
        declare_parameter("warmup_iters",          3);
        declare_parameter("sync_slop_s",           0.05);
        declare_parameter("depth_color_publish_w", 480);

        const std::string engine_path = get_parameter("engine_file_path").as_string();
        const std::string calib_path  = get_parameter("calibration_file").as_string();
        depth_vmax_  = get_parameter("depth_vmax").as_double();
        int warmup   = get_parameter("warmup_iters").as_int();
        slop_ns_     = static_cast<int64_t>(get_parameter("sync_slop_s").as_double() * 1e9);
        color_w_req_ = get_parameter("depth_color_publish_w").as_int();

        if (engine_path.empty() || !std::ifstream(engine_path).good())
            throw std::runtime_error("engine_file_path missing or unreadable: " + engine_path);
        if (calib_path.empty()  || !std::ifstream(calib_path).good())
            throw std::runtime_error("calibration_file missing or unreadable: " + calib_path);

        loadCalibration(calib_path);
        initEngine(engine_path);
        bakeRectifyMaps();
        allocateDeviceBuffers();
        if (warmup > 0) runWarmup(warmup);
        setupRos();

        ROS_INFO("stereo_depth: ready (eye=%dx%d, model=%dx%d, baseline=%.2fmm, bf=%.3f)",
                 eye_w_, eye_h_, model_w_, model_h_, baseline_m_ * 1000.0, bf_);
    }

    ~StereoDepthNode() override {
        if (stream_)         cudaStreamDestroy(stream_);
        if (d_raw_left_)     cudaFree(d_raw_left_);
        if (d_raw_right_)    cudaFree(d_raw_right_);
        if (d_left_in_)      cudaFree(d_left_in_);
        if (d_right_in_)     cudaFree(d_right_in_);
        if (d_disp_out_)     cudaFree(d_disp_out_);
        if (d_remap_left_)   cudaFree(d_remap_left_);
        if (d_remap_right_)  cudaFree(d_remap_right_);
    }

private:
    // ───────────────────────────────────────────────────────── calibration
    void loadCalibration(const std::string& path) {
        YAML::Node y = YAML::LoadFile(path);
        eye_w_ = y["image_width"].as<int>();
        eye_h_ = y["image_height"].as<int>();

        K1_ = matFromNode(y["K_left"]);
        K2_ = matFromNode(y["K_right"]);
        D1_ = matFromNode(y["D_left"]);
        D2_ = matFromNode(y["D_right"]);
        R1_ = matFromNode(y["R1"]);
        R2_ = matFromNode(y["R2"]);
        P1_ = matFromNode(y["P1"]);
        P2_ = matFromNode(y["P2"]);

        // fx_rect and baseline are encoded in the rectified projection matrices.
        // baseline_m = -P2[0,3] / P1[0,0]. We compute bf = fx_rect * baseline_m
        // and apply Z = bf / disparity in postprocess.
        fx_rect_    = P1_.at<double>(0, 0);
        baseline_m_ = -P2_.at<double>(0, 3) / fx_rect_;
        bf_         = fx_rect_ * baseline_m_;
    }

    // ───────────────────────────────────────────────────────── TRT engine
    void initEngine(const std::string& path) {
        runtime_.reset(nvinfer1::createInferRuntime(g_logger));
        engine_.reset(loadEngine(runtime_.get(), path));
        if (!engine_) throw std::runtime_error("failed to deserialize engine: " + path);
        context_.reset(engine_->createExecutionContext());

        // High-priority CUDA stream so this engine preempts camera_stereo's
        // crop+flip kernel and other TRT models when they share the GPU.
        int prio_low = 0, prio_high = 0;
        cudaDeviceGetStreamPriorityRange(&prio_low, &prio_high);
        cudaStreamCreateWithPriority(&stream_, cudaStreamNonBlocking, prio_high);

        // Discover IO tensor names — accept either ordering of (left, right).
        const int n_io = engine_->getNbIOTensors();
        for (int i = 0; i < n_io; ++i) {
            const char* name = engine_->getIOTensorName(i);
            std::string lname(name);
            std::transform(lname.begin(), lname.end(), lname.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            const bool is_input = (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT);
            if (is_input) {
                if      (lname.find("left")  != std::string::npos) left_in_name_  = name;
                else if (lname.find("right") != std::string::npos) right_in_name_ = name;
            } else {
                disp_out_name_ = name;
            }
        }
        if (left_in_name_.empty() || right_in_name_.empty() || disp_out_name_.empty())
            throw std::runtime_error("engine IO tensor names missing left/right/disparity");

        auto in_shape = engine_->getTensorShape(left_in_name_.c_str());
        // Expect (1, 3, model_h, model_w)
        if (in_shape.nbDims != 4)
            throw std::runtime_error("expected 4D input tensor");
        model_h_ = in_shape.d[2];
        model_w_ = in_shape.d[3];

        ROS_INFO("stereo_depth: engine io  left=%s  right=%s  disp=%s  shape=%dx%d",
                 left_in_name_.c_str(), right_in_name_.c_str(), disp_out_name_.c_str(),
                 model_w_, model_h_);
    }

    // Bake the rectify maps at MODEL resolution. We scale the rectified
    // intrinsics (Krect = P[:3,:3]) from eye-res down to model-res so a single
    // cv::initUndistortRectifyMap call yields a (model_h, model_w) float2 map
    // pointing directly into the raw image — fusing undistort + rectify +
    // downscale into the bilinear-lookup kernel.
    void bakeRectifyMaps() {
        const double sx = static_cast<double>(model_w_) / eye_w_;
        const double sy = static_cast<double>(model_h_) / eye_h_;

        auto scaled_K = [&](const cv::Mat& P) {
            cv::Mat K = P(cv::Rect(0, 0, 3, 3)).clone();
            K.at<double>(0, 0) *= sx; K.at<double>(0, 2) *= sx;
            K.at<double>(1, 1) *= sy; K.at<double>(1, 2) *= sy;
            return K;
        };
        const cv::Mat Krect1 = scaled_K(P1_);
        const cv::Mat Krect2 = scaled_K(P2_);

        cv::Mat map1_l, map2_l, map1_r, map2_r;
        cv::initUndistortRectifyMap(K1_, D1_, R1_, Krect1,
                                    cv::Size(model_w_, model_h_),
                                    CV_32FC2, map1_l, map2_l);
        cv::initUndistortRectifyMap(K2_, D2_, R2_, Krect2,
                                    cv::Size(model_w_, model_h_),
                                    CV_32FC2, map1_r, map2_r);

        // initUndistortRectifyMap with CV_32FC2 returns map1 = (model_h, model_w, 2)
        // already packed as float2 with (x_src, y_src). map2 is empty in this mode.
        cudaMalloc(&d_remap_left_,  sizeof(float2) * model_w_ * model_h_);
        cudaMalloc(&d_remap_right_, sizeof(float2) * model_w_ * model_h_);
        cudaMemcpy(d_remap_left_,  map1_l.data, sizeof(float2) * model_w_ * model_h_, cudaMemcpyHostToDevice);
        cudaMemcpy(d_remap_right_, map1_r.data, sizeof(float2) * model_w_ * model_h_, cudaMemcpyHostToDevice);
    }

    void allocateDeviceBuffers() {
        // Raw eye images (uchar3 HWC).
        cudaMalloc(&d_raw_left_,  sizeof(uchar3) * eye_w_ * eye_h_);
        cudaMalloc(&d_raw_right_, sizeof(uchar3) * eye_w_ * eye_h_);

        // TRT input bindings (float CHW).
        cudaMalloc(&d_left_in_,  sizeof(float) * 3 * model_w_ * model_h_);
        cudaMalloc(&d_right_in_, sizeof(float) * 3 * model_w_ * model_h_);

        // TRT output (1, 1, model_h, model_w).
        cudaMalloc(&d_disp_out_, sizeof(float) * model_w_ * model_h_);

        context_->setTensorAddress(left_in_name_.c_str(),  d_left_in_);
        context_->setTensorAddress(right_in_name_.c_str(), d_right_in_);
        context_->setTensorAddress(disp_out_name_.c_str(), d_disp_out_);

        host_disp_.resize(model_w_ * model_h_);
    }

    void runWarmup(int iters) {
        // Zero-fill inputs and run a few invocations to load TRT lazy state.
        cudaMemsetAsync(d_left_in_,  0, sizeof(float) * 3 * model_w_ * model_h_, stream_);
        cudaMemsetAsync(d_right_in_, 0, sizeof(float) * 3 * model_w_ * model_h_, stream_);
        for (int i = 0; i < iters; ++i) {
            context_->enqueueV3(stream_);
        }
        cudaStreamSynchronize(stream_);
    }

    void setupRos() {
        // Output resolution for the colormap viz topic — match Python defaults.
        if (color_w_req_ <= 0 || color_w_req_ >= eye_w_) {
            color_pub_w_ = eye_w_;
            color_pub_h_ = eye_h_;
        } else {
            color_pub_w_ = color_w_req_;
            color_pub_h_ = static_cast<int>(std::round(
                static_cast<double>(eye_h_) * color_pub_w_ / eye_w_));
        }

        // QoS — match Python: BEST_EFFORT VOLATILE depth=2 for both in & out.
        rclcpp::QoS qos(rclcpp::KeepLast(2));
        qos.best_effort();
        qos.durability_volatile();

        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth",       qos);
        color_pub_ = create_publisher<sensor_msgs::msg::Image>("depth_color", qos);

        // Pre-shape output messages.
        depth_msg_.encoding = "32FC1";
        depth_msg_.width    = eye_w_;
        depth_msg_.height   = eye_h_;
        depth_msg_.step     = eye_w_ * sizeof(float);
        depth_msg_.header.frame_id = "stereo_left";
        depth_msg_.data.resize(static_cast<size_t>(eye_w_) * eye_h_ * sizeof(float));

        color_msg_.encoding = "bgr8";
        color_msg_.width    = color_pub_w_;
        color_msg_.height   = color_pub_h_;
        color_msg_.step     = color_pub_w_ * 3;
        color_msg_.header.frame_id = "stereo_left";
        color_msg_.data.resize(static_cast<size_t>(color_pub_w_) * color_pub_h_ * 3);

        right_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "right/image_raw", qos,
            std::bind(&StereoDepthNode::onRight, this, std::placeholders::_1));
        left_sub_  = create_subscription<sensor_msgs::msg::Image>(
            "left/image_raw",  qos,
            std::bind(&StereoDepthNode::onLeft,  this, std::placeholders::_1));
    }

    // ───────────────────────────────────────────────────────── callbacks
    void onRight(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(right_mtx_);
        latest_right_ = msg;
    }

    void onLeft(sensor_msgs::msg::Image::ConstSharedPtr left_msg) {
        sensor_msgs::msg::Image::ConstSharedPtr right_msg;
        {
            std::lock_guard<std::mutex> lk(right_mtx_);
            right_msg = latest_right_;
        }
        if (!right_msg) return;

        const int64_t l_ns =
            static_cast<int64_t>(left_msg->header.stamp.sec) * 1'000'000'000LL +
            left_msg->header.stamp.nanosec;
        const int64_t r_ns =
            static_cast<int64_t>(right_msg->header.stamp.sec) * 1'000'000'000LL +
            right_msg->header.stamp.nanosec;
        if (std::abs(l_ns - r_ns) > slop_ns_) return;

        if (static_cast<int>(left_msg->width)  != eye_w_ ||
            static_cast<int>(left_msg->height) != eye_h_ ||
            static_cast<int>(right_msg->width) != eye_w_ ||
            static_cast<int>(right_msg->height)!= eye_h_) {
            if ((frame_count_ % 60) == 0)
                ROS_WARN("stereo_depth: image size %ux%u doesn't match calibration %dx%d",
                         left_msg->width, left_msg->height, eye_w_, eye_h_);
            return;
        }

        const int order_l = encodingToOrder(left_msg->encoding);
        const int order_r = encodingToOrder(right_msg->encoding);
        if (order_l < 0 || order_r < 0) {
            if ((frame_count_ % 60) == 0)
                ROS_WARN("stereo_depth: unsupported encoding %s / %s",
                         left_msg->encoding.c_str(), right_msg->encoding.c_str());
            return;
        }

        runFrame(left_msg, right_msg, order_l, order_r);
    }

    static int encodingToOrder(const std::string& enc) {
        if (enc == "rgb8") return 0;
        if (enc == "bgr8") return 1;
        return -1;
    }

    // ───────────────────────────────────────────────────────── hot loop
    void runFrame(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& right_msg,
                  int order_l, int order_r)
    {
        const size_t raw_bytes = static_cast<size_t>(eye_w_) * eye_h_ * sizeof(uchar3);

        // Host → device upload for both eyes (no intermediate cv::Mat needed —
        // the message data is already tightly packed uchar3 for rgb8/bgr8 with
        // step == width*3 after node_camera_stereo's CUDA crop+flip).
        cudaMemcpyAsync(d_raw_left_,  left_msg->data.data(),  raw_bytes, cudaMemcpyHostToDevice, stream_);
        cudaMemcpyAsync(d_raw_right_, right_msg->data.data(), raw_bytes, cudaMemcpyHostToDevice, stream_);

        // Fused per-eye preprocess into TRT input bindings.
        cudaStereoRectifyResizeNormCHW(
            static_cast<const uchar3*>(d_raw_left_), eye_w_, eye_h_,
            static_cast<const float2*>(d_remap_left_),  model_w_, model_h_,
            static_cast<float*>(d_left_in_),
            kImageNetMean[0], kImageNetMean[1], kImageNetMean[2],
            1.0f / kImageNetStd[0], 1.0f / kImageNetStd[1], 1.0f / kImageNetStd[2],
            order_l, stream_);

        cudaStereoRectifyResizeNormCHW(
            static_cast<const uchar3*>(d_raw_right_), eye_w_, eye_h_,
            static_cast<const float2*>(d_remap_right_), model_w_, model_h_,
            static_cast<float*>(d_right_in_),
            kImageNetMean[0], kImageNetMean[1], kImageNetMean[2],
            1.0f / kImageNetStd[0], 1.0f / kImageNetStd[1], 1.0f / kImageNetStd[2],
            order_r, stream_);

        // Inference.
        if (!context_->enqueueV3(stream_)) {
            ROS_ERROR("stereo_depth: enqueueV3 failed");
            return;
        }

        // D2H disparity.
        cudaMemcpyAsync(host_disp_.data(), d_disp_out_,
                        sizeof(float) * model_w_ * model_h_,
                        cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // Build a model-resolution Mat over the host buffer.
        cv::Mat disp_model(model_h_, model_w_, CV_32F, host_disp_.data());

        // Resize to eye resolution + scale disparity to eye-pixel units.
        // (Engine outputs disparity in MODEL pixel units; bf is in EYE pixel
        // units because P1[0,0] is eye-res fx.)
        cv::Mat disp_src;
        cv::resize(disp_model, disp_src, cv::Size(eye_w_, eye_h_), 0, 0, cv::INTER_LINEAR);
        disp_src *= static_cast<float>(eye_w_) / static_cast<float>(model_w_);

        // depth = bf / disparity (with NaN for invalid).
        cv::Mat depth(eye_h_, eye_w_, CV_32F,
                      const_cast<uint8_t*>(depth_msg_.data.data()));
        const float bf = static_cast<float>(bf_);
        for (int y = 0; y < eye_h_; ++y) {
            const float* d = disp_src.ptr<float>(y);
            float* z = depth.ptr<float>(y);
            for (int x = 0; x < eye_w_; ++x) {
                z[x] = (d[x] > 0.5f) ? (bf / d[x])
                                     : std::numeric_limits<float>::quiet_NaN();
            }
        }
        depth_msg_.header.stamp = left_msg->header.stamp;
        depth_pub_->publish(depth_msg_);

        // Color viz at a downsized resolution.
        cv::Mat disp_small;
        if (color_pub_w_ != eye_w_ || color_pub_h_ != eye_h_) {
            cv::resize(disp_src, disp_small, cv::Size(color_pub_w_, color_pub_h_), 0, 0, cv::INTER_NEAREST);
        } else {
            disp_small = disp_src;
        }
        cv::Mat z_small(disp_small.size(), CV_32F);
        cv::Mat mask = disp_small > 0.5f;
        for (int y = 0; y < color_pub_h_; ++y) {
            const float* d = disp_small.ptr<float>(y);
            float* z = z_small.ptr<float>(y);
            const uint8_t* m = mask.ptr<uint8_t>(y);
            for (int x = 0; x < color_pub_w_; ++x) {
                if (m[x]) {
                    float v = bf / d[x];
                    if (v > static_cast<float>(depth_vmax_)) v = static_cast<float>(depth_vmax_);
                    if (v < 0.0f) v = 0.0f;
                    z[x] = v;
                } else {
                    z[x] = static_cast<float>(depth_vmax_);
                }
            }
        }
        cv::Mat norm8u;
        z_small.convertTo(norm8u, CV_8U,
                          -255.0 / depth_vmax_,
                          255.0);
        cv::Mat cm;
        cv::applyColorMap(norm8u, cm, cv::COLORMAP_INFERNO);
        cm.setTo(cv::Scalar(0, 0, 0), ~mask);
        std::memcpy(color_msg_.data.data(), cm.data, color_msg_.data.size());
        color_msg_.header.stamp = left_msg->header.stamp;
        color_pub_->publish(color_msg_);

        // Lightweight per-30-frame heartbeat.
        if ((++frame_count_ % 30) == 0) {
            ROS_INFO("stereo_depth: %d frames published", frame_count_);
        }
    }

private:
    // calibration
    int eye_w_ = 0, eye_h_ = 0;
    cv::Mat K1_, K2_, D1_, D2_, R1_, R2_, P1_, P2_;
    double fx_rect_ = 0.0, baseline_m_ = 0.0, bf_ = 0.0;

    // TRT
    std::unique_ptr<nvinfer1::IRuntime>          runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    std::string left_in_name_, right_in_name_, disp_out_name_;
    int model_w_ = 0, model_h_ = 0;

    // CUDA
    cudaStream_t stream_ = nullptr;
    void* d_raw_left_    = nullptr;
    void* d_raw_right_   = nullptr;
    void* d_remap_left_  = nullptr;
    void* d_remap_right_ = nullptr;
    void* d_left_in_     = nullptr;
    void* d_right_in_    = nullptr;
    void* d_disp_out_    = nullptr;

    // host scratch
    std::vector<float> host_disp_;

    // ROS
    int    color_w_req_ = 480;
    int    color_pub_w_ = 0, color_pub_h_ = 0;
    int    frame_count_ = 0;
    double depth_vmax_  = 20.0;
    int64_t slop_ns_    = 50'000'000;

    sensor_msgs::msg::Image depth_msg_;
    sensor_msgs::msg::Image color_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;

    std::mutex right_mtx_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_right_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<StereoDepthNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        ROS_ERROR("stereo_depth: fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
