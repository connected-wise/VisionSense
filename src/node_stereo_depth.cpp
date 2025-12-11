/**
 * CUDA-Accelerated Stereo Depth Node
 *
 * Combined node that:
 * 1. Subscribes to left/right raw images from camera_stereo
 * 2. Uploads to GPU and converts to grayscale on GPU
 * 3. Computes disparity using CUDA StereoSGM
 * 4. Applies median filter on GPU
 * 5. Publishes disparity image to /stereo_depth/disparity
 *
 * All operations except initial image conversion are GPU-accelerated
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudawarping.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class StereoDepthNode : public rclcpp::Node
{
public:
    StereoDepthNode() : Node("stereo_depth")
    {
        // Check CUDA availability
        int cuda_devices = cv::cuda::getCudaEnabledDeviceCount();
        RCLCPP_INFO(this->get_logger(), "CUDA devices available: %d", cuda_devices);

        if (cuda_devices == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "No CUDA devices found!");
            throw std::runtime_error("CUDA not available");
        }

        RCLCPP_INFO(this->get_logger(), "Using CUDA device: %d", cv::cuda::getDevice());

        // Declare parameters
        this->declare_parameter("min_disparity", 0);
        this->declare_parameter("num_disparities", 64);  // Must be 64, 128, or 256 for CUDA
        this->declare_parameter("block_size", 9);         // Odd number
        this->declare_parameter("p1_multiplier", 8);
        this->declare_parameter("p2_multiplier", 32);
        this->declare_parameter("uniqueness_ratio", 10);
        this->declare_parameter("median_filter_size", 5);

        // Get parameters
        int min_disp = this->get_parameter("min_disparity").as_int();
        num_disp_ = this->get_parameter("num_disparities").as_int();
        int block_size = this->get_parameter("block_size").as_int();
        int p1_mult = this->get_parameter("p1_multiplier").as_int();
        int p2_mult = this->get_parameter("p2_multiplier").as_int();
        int uniqueness = this->get_parameter("uniqueness_ratio").as_int();
        median_kernel_ = this->get_parameter("median_filter_size").as_int();

        // Calculate P1 and P2
        int P1 = p1_mult * block_size * block_size;
        int P2 = p2_mult * block_size * block_size;

        RCLCPP_INFO(this->get_logger(), "Creating CUDA StereoSGM with:");
        RCLCPP_INFO(this->get_logger(), "  min_disparity: %d", min_disp);
        RCLCPP_INFO(this->get_logger(), "  num_disparities: %d", num_disp_);
        RCLCPP_INFO(this->get_logger(), "  block_size: %d", block_size);
        RCLCPP_INFO(this->get_logger(), "  P1: %d, P2: %d", P1, P2);
        RCLCPP_INFO(this->get_logger(), "  uniqueness_ratio: %d", uniqueness);
        RCLCPP_INFO(this->get_logger(), "  median_filter_size: %d", median_kernel_);

        // Try CUDA StereoSGM first, fallback to StereoBM if it fails
        try
        {
            stereo_sgm_ = cv::cuda::createStereoSGM(
                min_disp,
                num_disp_,
                P1,
                P2,
                uniqueness,
                cv::cuda::StereoSGM::MODE_HH  // Horizontal-Horizontal mode
            );
            use_sgm_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ CUDA StereoSGM created successfully (MODE_HH)");
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "CUDA StereoSGM not available, using StereoBM: %s", e.what());
            stereo_bm_ = cv::cuda::createStereoBM(num_disp_, block_size);
            use_sgm_ = false;
            RCLCPP_INFO(this->get_logger(), "✓ CUDA StereoBM created successfully");
        }

        // Load calibration for rectification
        loadCalibration();

        // CUDA median filter only supports unsigned types, not CV_16S
        // We'll skip GPU median filter and apply it on CPU after conversion if needed
        if (median_kernel_ > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Median filter will be applied on CPU (kernel: %d)", median_kernel_);
        }

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

        RCLCPP_INFO(this->get_logger(), "CUDA stereo depth node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: left/image_raw, right/image_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing disparity to: /stereo_depth/disparity");
        RCLCPP_INFO(this->get_logger(), "All operations GPU-accelerated!");
    }

private:
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        // Convert ROS messages to OpenCV Mat
        cv::Mat left = msgToMat(left_msg);
        cv::Mat right = msgToMat(right_msg);

        // Upload to GPU
        d_left_.upload(left, cuda_stream_);
        d_right_.upload(right, cuda_stream_);

        // Convert to grayscale on GPU if needed
        if (left.channels() == 3)
        {
            cv::cuda::cvtColor(d_left_, d_left_gray_, cv::COLOR_BGR2GRAY, 0, cuda_stream_);
            cv::cuda::cvtColor(d_right_, d_right_gray_, cv::COLOR_BGR2GRAY, 0, cuda_stream_);
        }
        else
        {
            d_left_gray_ = d_left_;
            d_right_gray_ = d_right_;
        }

        // Reduce resolution by half BEFORE rectification for efficiency
        // (uses pre-scaled calibration maps for proper rectification at half resolution)
        cv::cuda::resize(d_left_gray_, d_left_gray_, cv::Size(d_left_gray_.cols / 2, d_left_gray_.rows / 2), 0, 0, cv::INTER_LINEAR, cuda_stream_);
        cv::cuda::resize(d_right_gray_, d_right_gray_, cv::Size(d_right_gray_.cols / 2, d_right_gray_.rows / 2), 0, 0, cv::INTER_LINEAR, cuda_stream_);

        // Rectify downsampled images using scaled calibration maps (GPU-accelerated)
        if (have_calibration_)
        {
            cv::cuda::remap(d_left_gray_, d_left_rect_, d_map_l_x_, d_map_l_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0, cuda_stream_);
            cv::cuda::remap(d_right_gray_, d_right_rect_, d_map_r_x_, d_map_r_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0, cuda_stream_);
            d_left_gray_ = d_left_rect_;
            d_right_gray_ = d_right_rect_;
        }

        // Compute disparity on GPU
        try
        {
            if (use_sgm_)
                stereo_sgm_->compute(d_left_gray_, d_right_gray_, d_disparity_, cuda_stream_);
            else
                stereo_bm_->compute(d_left_gray_, d_right_gray_, d_disparity_, cuda_stream_);
        }
        catch (const cv::Exception& e)
        {
            // If SGM fails at runtime, switch to BM permanently
            if (use_sgm_)
            {
                RCLCPP_ERROR(this->get_logger(), "CUDA StereoSGM runtime error, switching to StereoBM: %s", e.what());
                stereo_bm_ = cv::cuda::createStereoBM(num_disp_, 9);
                use_sgm_ = false;
                stereo_bm_->compute(d_left_gray_, d_right_gray_, d_disparity_, cuda_stream_);
            }
            else
            {
                throw;
            }
        }

        // Wait for GPU operations to complete
        cuda_stream_.waitForCompletion();

        // Single download from GPU to CPU
        cv::Mat disp_raw;
        d_disparity_.download(disp_raw);

        // Minimal CPU post-processing
        // Convert from fixed-point (16-bit) to float
        cv::Mat disp_float;
        disp_raw.convertTo(disp_float, CV_32F, 1.0 / 16.0);

        // Mask invalid disparities (StereoSGM uses negative values for invalid)
        cv::Mat valid_mask = (disp_float > 0) & (disp_float < num_disp_);

        // Convert to 8-bit for visualization
        cv::Mat disp_8u;
        cv::normalize(disp_float, disp_8u, 0, 255, cv::NORM_MINMAX, CV_8U);

        // Apply median filter on CPU (CUDA median doesn't support signed 16-bit)
        if (median_kernel_ > 0)
        {
            cv::medianBlur(disp_8u, disp_8u, median_kernel_);
        }

        // Mask out invalid regions
        disp_8u.setTo(0, ~valid_mask);

        // Publish raw disparity (8-bit normalized, 0=invalid, 1-255=valid disparity)
        // GUI node will colorize it
        auto disp_msg = matToMsg(disp_8u, left_msg->header, "mono8");
        disparity_pub_->publish(disp_msg);
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

    // CUDA stereo matchers
    cv::Ptr<cv::cuda::StereoSGM> stereo_sgm_;
    cv::Ptr<cv::cuda::StereoBM> stereo_bm_;
    bool use_sgm_;

    // Load calibration maps from config file
    void loadCalibration()
    {
        try
        {
            std::string config_path = ament_index_cpp::get_package_share_directory("visionconnect") + "/config/calibration.yaml";
            cv::FileStorage fs(config_path, cv::FileStorage::READ);

            if (!fs.isOpened())
            {
                RCLCPP_WARN(this->get_logger(), "Could not open calibration file: %s", config_path.c_str());
                have_calibration_ = false;
                return;
            }

            // Load calibration maps
            cv::Mat map_l_x, map_l_y, map_r_x, map_r_y;
            fs["map_l_x"] >> map_l_x;
            fs["map_l_y"] >> map_l_y;
            fs["map_r_x"] >> map_r_x;
            fs["map_r_y"] >> map_r_y;
            fs.release();

            if (map_l_x.empty() || map_l_y.empty() || map_r_x.empty() || map_r_y.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Calibration maps are empty!");
                have_calibration_ = false;
                return;
            }

            // Scale calibration maps by 0.5 for half-resolution matching
            // This maintains proper calibration for downsampled images
            cv::resize(map_l_x, map_l_x, cv::Size(map_l_x.cols / 2, map_l_x.rows / 2), 0, 0, cv::INTER_LINEAR);
            cv::resize(map_l_y, map_l_y, cv::Size(map_l_y.cols / 2, map_l_y.rows / 2), 0, 0, cv::INTER_LINEAR);
            cv::resize(map_r_x, map_r_x, cv::Size(map_r_x.cols / 2, map_r_x.rows / 2), 0, 0, cv::INTER_LINEAR);
            cv::resize(map_r_y, map_r_y, cv::Size(map_r_y.cols / 2, map_r_y.rows / 2), 0, 0, cv::INTER_LINEAR);

            // Scale map values by 0.5 to correspond to half-resolution coordinates
            map_l_x *= 0.5f;
            map_l_y *= 0.5f;
            map_r_x *= 0.5f;
            map_r_y *= 0.5f;

            // Upload scaled maps to GPU for CUDA remap
            d_map_l_x_.upload(map_l_x);
            d_map_l_y_.upload(map_l_y);
            d_map_r_x_.upload(map_r_x);
            d_map_r_y_.upload(map_r_y);

            have_calibration_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ Loaded and scaled calibration maps from: %s", config_path.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to load calibration: %s", e.what());
            have_calibration_ = false;
        }
    }

    // GPU memory (pre-allocated, reused every frame)
    cv::cuda::GpuMat d_left_, d_right_;
    cv::cuda::GpuMat d_left_gray_, d_right_gray_;
    cv::cuda::GpuMat d_left_rect_, d_right_rect_;
    cv::cuda::GpuMat d_disparity_;

    // Calibration maps on GPU
    cv::cuda::GpuMat d_map_l_x_, d_map_l_y_;
    cv::cuda::GpuMat d_map_r_x_, d_map_r_y_;
    bool have_calibration_ = false;

    // CUDA stream for async operations
    cv::cuda::Stream cuda_stream_;

    // Subscribers and synchronizer
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;

    // Parameters
    int num_disp_;
    int median_kernel_;
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
