#pragma once

// This file provides OpenCV CUDA-compatible functions that use our backend
// Include this instead of opencv2/cudaarithm.hpp and opencv2/cudawarping.hpp

#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>
#include "cuda_backend.h"

namespace cv {
namespace cuda {

// Image resize function
inline void resize(InputArray _src, OutputArray _dst, Size dsize, 
                  double fx = 0, double fy = 0, int interpolation = INTER_LINEAR, 
                  Stream& stream = Stream::Null()) {
    cv::cuda::GpuMat src = _src.getGpuMat();
    cv::cuda::GpuMat dst;
    
    cuda_backend::getCudaBackend()->resize(src, dst, dsize, fx, fy, interpolation);
    
    dst.copyTo(_dst);
}

// Channel split function
inline void split(InputArray _src, OutputArrayOfArrays _dst, Stream& stream = Stream::Null()) {
    cv::cuda::GpuMat src = _src.getGpuMat();
    std::vector<cv::cuda::GpuMat> channels;
    
    cuda_backend::getCudaBackend()->split(src, channels);
    
    // Convert GpuMat vector to output array
    std::vector<cv::Mat> cpu_channels;
    for (size_t i = 0; i < channels.size(); i++) {
        cv::Mat temp;
        channels[i].download(temp);
        cpu_channels.push_back(temp);
    }
    
    // Use vector assignment which is supported by OutputArrayOfArrays
    _dst.assign(cpu_channels);
}

// Subtract scalar from image
inline void subtract(InputArray _src1, InputArray _src2, OutputArray _dst, 
                    InputArray mask = noArray(), int dtype = -1, 
                    Stream& stream = Stream::Null()) {
    cv::cuda::GpuMat src = _src1.getGpuMat();
    cv::cuda::GpuMat dst;
    
    // Handle scalar case
    if (_src2.kind() == _InputArray::MATX) {
        cv::Mat m = _src2.getMat();
        cv::Scalar scalar;
        if (m.total() == 1) {
            scalar = cv::Scalar(m.at<double>(0));
        } else if (m.total() <= 4) {
            double vals[4] = {0, 0, 0, 0};
            for (int i = 0; i < m.total(); i++) {
                vals[i] = m.at<double>(i);
            }
            scalar = cv::Scalar(vals[0], vals[1], vals[2], vals[3]);
        }
        cuda_backend::getCudaBackend()->subtract(src, scalar, dst);
    } else {
        // For mat-mat subtraction, fall back to CPU for now
        cv::Mat cpu_src1, cpu_src2, cpu_dst;
        src.download(cpu_src1);
        _src2.getGpuMat().download(cpu_src2);
        cv::subtract(cpu_src1, cpu_src2, cpu_dst);
        dst.upload(cpu_dst);
    }
    
    dst.copyTo(_dst);
}

// Color conversion function
inline void cvtColor(InputArray _src, OutputArray _dst, int code, int dstCn = 0,
                    Stream& stream = Stream::Null()) {
    // Fall back to CPU implementation for color conversion
    cv::Mat cpu_src, cpu_dst;
    _src.getGpuMat().download(cpu_src);
    cv::cvtColor(cpu_src, cpu_dst, code, dstCn);
    _dst.getGpuMatRef().upload(cpu_dst);
}

// Divide image by scalar
inline void divide(InputArray _src1, InputArray _src2, OutputArray _dst, 
                  double scale = 1, int dtype = -1, 
                  Stream& stream = Stream::Null()) {
    cv::cuda::GpuMat src = _src1.getGpuMat();
    cv::cuda::GpuMat dst;
    
    // Handle scalar case
    if (_src2.kind() == _InputArray::MATX) {
        cv::Mat m = _src2.getMat();
        cv::Scalar scalar;
        if (m.total() == 1) {
            scalar = cv::Scalar(m.at<double>(0));
        } else if (m.total() <= 4) {
            double vals[4] = {0, 0, 0, 0};
            for (int i = 0; i < m.total(); i++) {
                vals[i] = m.at<double>(i);
            }
            scalar = cv::Scalar(vals[0], vals[1], vals[2], vals[3]);
        }
        cuda_backend::getCudaBackend()->divide(src, scalar, dst);
    } else {
        // For mat-mat division, fall back to CPU for now
        cv::Mat cpu_src1, cpu_src2, cpu_dst;
        src.download(cpu_src1);
        _src2.getGpuMat().download(cpu_src2);
        cv::divide(cpu_src1, cpu_src2, cpu_dst, scale);
        dst.upload(cpu_dst);
    }
    
    dst.copyTo(_dst);
}

} // namespace cuda
} // namespace cv