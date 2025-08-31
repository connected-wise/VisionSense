#include "cuda_backend.h"
#include <iostream>
#include <cuda_runtime.h>

namespace cuda_backend {

// Helper functions for element-wise operations
void subtractScalarCPU(float* data, int size, float val0, float val1, float val2) {
    for (int i = 0; i < size; i += 3) {
        data[i] -= val0;
        if (i + 1 < size) data[i + 1] -= val1;
        if (i + 2 < size) data[i + 2] -= val2;
    }
}

void divideScalarCPU(float* data, int size, float val0, float val1, float val2) {
    for (int i = 0; i < size; i += 3) {
        data[i] /= val0;
        if (i + 1 < size) data[i + 1] /= val1;
        if (i + 2 < size) data[i + 2] /= val2;
    }
}

// JetsonUtils Backend Implementation
JetsonUtilsBackend::JetsonUtilsBackend() {
    std::cout << "Using Jetson-Utils CUDA backend (OpenCV CUDA not available)" << std::endl;
}

JetsonUtilsBackend::~JetsonUtilsBackend() {}

void* JetsonUtilsBackend::gpuMatToJetsonPtr(const cv::cuda::GpuMat& mat) {
    return reinterpret_cast<void*>(mat.data);
}

bool JetsonUtilsBackend::resize(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, 
                                cv::Size dsize, double fx, double fy, 
                                int interpolation) {
    // Create destination if needed
    if (dst.empty()) {
        dst.create(dsize, src.type());
    }
    
    #if USE_JETSON_UTILS
    // Use jetson-utils cudaResize
    // Note: jetson-utils expects different format, so we need to handle format conversion
    if (src.type() == CV_8UC3) {
        // For RGB/BGR images
        if (cudaResize((uchar3*)src.data, src.cols, src.rows,
                      (uchar3*)dst.data, dst.cols, dst.rows)) {
            return false; // cudaResize returns non-zero on error
        }
        return true;
    } else if (src.type() == CV_8UC1) {
        // For grayscale
        if (cudaResize((unsigned char*)src.data, src.cols, src.rows,
                      (unsigned char*)dst.data, dst.cols, dst.rows)) {
            return false;
        }
        return true;
    }
    #endif
    
    // Fallback to CPU resize
    cv::Mat cpu_src, cpu_dst;
    src.download(cpu_src);
    cv::resize(cpu_src, cpu_dst, dsize, fx, fy, interpolation);
    dst.upload(cpu_dst);
    return true;
}

bool JetsonUtilsBackend::split(const cv::cuda::GpuMat& src, 
                               std::vector<cv::cuda::GpuMat>& channels) {
    if (src.channels() != 3) {
        std::cerr << "Split only supports 3-channel images" << std::endl;
        return false;
    }
    
    channels.resize(3);
    for (int i = 0; i < 3; i++) {
        channels[i].create(src.rows, src.cols, CV_8UC1);
    }
    
    // CPU fallback for channel splitting
    cv::Mat cpu_src;
    src.download(cpu_src);
    
    std::vector<cv::Mat> cpu_channels;
    cv::split(cpu_src, cpu_channels);
    
    for (int i = 0; i < 3; i++) {
        channels[i].upload(cpu_channels[i]);
    }
    return true;
}

bool JetsonUtilsBackend::subtract(const cv::cuda::GpuMat& src1, const cv::Scalar& value,
                                  cv::cuda::GpuMat& dst) {
    // Ensure dst is allocated
    if (dst.empty()) {
        dst.create(src1.size(), src1.type());
    }
    
    // Copy src to dst if they're different
    if (src1.data != dst.data) {
        src1.copyTo(dst);
    }
    
    // For float data
    if (dst.type() == CV_32FC3) {
        int total_elements = dst.rows * dst.cols * dst.channels();
        // Download to CPU, perform operation, upload back
        cv::Mat cpu_mat;
        dst.download(cpu_mat);
        
        // Use the CPU helper function
        subtractScalarCPU((float*)cpu_mat.data, total_elements, 
                         value[0], value[1], value[2]);
        
        dst.upload(cpu_mat);
        return true;
    }
    
    // Fallback to CPU
    cv::Mat cpu_mat;
    src1.download(cpu_mat);
    cv::subtract(cpu_mat, value, cpu_mat);
    dst.upload(cpu_mat);
    return true;
}

bool JetsonUtilsBackend::divide(const cv::cuda::GpuMat& src1, const cv::Scalar& value,
                               cv::cuda::GpuMat& dst) {
    // Ensure dst is allocated
    if (dst.empty()) {
        dst.create(src1.size(), src1.type());
    }
    
    // Copy src to dst if they're different
    if (src1.data != dst.data) {
        src1.copyTo(dst);
    }
    
    // For float data
    if (dst.type() == CV_32FC3) {
        int total_elements = dst.rows * dst.cols * dst.channels();
        // Download to CPU, perform operation, upload back
        cv::Mat cpu_mat;
        dst.download(cpu_mat);
        
        // Use the CPU helper function
        divideScalarCPU((float*)cpu_mat.data, total_elements,
                       value[0], value[1], value[2]);
        
        dst.upload(cpu_mat);
        return true;
    }
    
    // Fallback to CPU
    cv::Mat cpu_mat;
    src1.download(cpu_mat);
    cv::divide(cpu_mat, value, cpu_mat);
    dst.upload(cpu_mat);
    return true;
}

// Factory function
std::unique_ptr<ICudaBackend> createCudaBackend() {
    #if USE_OPENCV_CUDA
        // When OpenCV CUDA is available, return that backend
        // return std::make_unique<OpenCVCudaBackend>();
    #endif
    
    // Default to JetsonUtils backend
    return std::make_unique<JetsonUtilsBackend>();
}

// Global instance
ICudaBackend* getCudaBackend() {
    static std::unique_ptr<ICudaBackend> backend = createCudaBackend();
    return backend.get();
}

} // namespace cuda_backend