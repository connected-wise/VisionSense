#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <memory>
#include <vector>

// Define which backend to use
#define USE_OPENCV_CUDA 0  // Set to 1 when OpenCV with CUDA is available
#define USE_JETSON_UTILS 1 // Fallback to jetson-utils

#if USE_JETSON_UTILS
#include <jetson-utils/cudaMappedMemory.h>
#include <jetson-utils/cudaResize.h>
#include <jetson-utils/cudaColorspace.h>
#endif

namespace cuda_backend {

// Abstract interface for CUDA operations
class ICudaBackend {
public:
    virtual ~ICudaBackend() = default;
    
    // Image operations
    virtual bool resize(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, 
                       cv::Size dsize, double fx = 0, double fy = 0, 
                       int interpolation = cv::INTER_LINEAR) = 0;
    
    virtual bool split(const cv::cuda::GpuMat& src, 
                      std::vector<cv::cuda::GpuMat>& channels) = 0;
    
    virtual bool subtract(const cv::cuda::GpuMat& src1, const cv::Scalar& value,
                         cv::cuda::GpuMat& dst) = 0;
    
    virtual bool divide(const cv::cuda::GpuMat& src1, const cv::Scalar& value,
                       cv::cuda::GpuMat& dst) = 0;
    
    virtual std::string getBackendName() const = 0;
};

// Jetson-utils based implementation
class JetsonUtilsBackend : public ICudaBackend {
public:
    JetsonUtilsBackend();
    ~JetsonUtilsBackend();
    
    bool resize(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, 
               cv::Size dsize, double fx = 0, double fy = 0, 
               int interpolation = cv::INTER_LINEAR) override;
    
    bool split(const cv::cuda::GpuMat& src, 
              std::vector<cv::cuda::GpuMat>& channels) override;
    
    bool subtract(const cv::cuda::GpuMat& src1, const cv::Scalar& value,
                 cv::cuda::GpuMat& dst) override;
    
    bool divide(const cv::cuda::GpuMat& src1, const cv::Scalar& value,
               cv::cuda::GpuMat& dst) override;
    
    std::string getBackendName() const override { return "JetsonUtils"; }
    
private:
    // Helper to convert between cv::cuda::GpuMat and jetson-utils format
    void* gpuMatToJetsonPtr(const cv::cuda::GpuMat& mat);
};

// Factory to get the appropriate backend
std::unique_ptr<ICudaBackend> createCudaBackend();

// Global instance (initialized on first use)
ICudaBackend* getCudaBackend();

} // namespace cuda_backend