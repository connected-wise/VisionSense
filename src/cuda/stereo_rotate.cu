/*
 * CUDA kernels for stereo image processing
 * Input: 3840x1200 -> Two 1200x1200 outputs
 *
 * Rotated lenses (rotated_lenses=true):
 *   Crop center 1440px: [right_cam:0-1199 | skip:1440 | left_cam:2640-3839]
 *
 * Non-rotated lenses (rotated_lenses=false):
 *   Inner-half crop, matching Fast-FoundationStereo/cpp_demo/stereo_trt.cu:
 *   [skip:720 | left:720-1919 | right:1920-3119 | skip:720]
 *   Each eye camera occupies one half of the sensor (0-1919 = left cam,
 *   1920-3839 = right cam); we take the inner 1200 columns of each so the
 *   two eyes' fields of view are adjacent and the stereo baseline matches
 *   the reference demo exactly.
 */

#include <cuda_runtime.h>
#include <stdint.h>

__global__ void stereoCropSplitRotateKernel(
    const uchar3* __restrict__ input,
    uchar3* __restrict__ leftOut,
    uchar3* __restrict__ rightOut,
    int stereoWidth,
    int outputSize,
    int leftCamStart
)
{
    int outX = blockIdx.x * blockDim.x + threadIdx.x;
    int outY = blockIdx.y * blockDim.y + threadIdx.y;

    if (outX >= outputSize || outY >= outputSize)
        return;

    int outIdx = outY * outputSize + outX;

    // Left output from left camera (cols 2640-3839) with 90° CCW
    // 90° CCW: out(x,y) = in(y, width-1-x)
    {
        int inRow = outX;
        int inCol = leftCamStart + (outputSize - 1 - outY);
        leftOut[outIdx] = input[inRow * stereoWidth + inCol];
    }

    // Right output from right camera (cols 0-1199) with 90° CW
    {
        int inRow = outputSize - 1 - outX;
        int inCol = outY;
        rightOut[outIdx] = input[inRow * stereoWidth + inCol];
    }
}

__global__ void stereoCropSplitKernel(
    const uchar3* __restrict__ input,
    uchar3* __restrict__ leftOut,
    uchar3* __restrict__ rightOut,
    int stereoWidth,
    int outputSize,
    int cropMargin
)
{
    int outX = blockIdx.x * blockDim.x + threadIdx.x;
    int outY = blockIdx.y * blockDim.y + threadIdx.y;

    if (outX >= outputSize || outY >= outputSize)
        return;

    int outIdx = outY * outputSize + outX;

    // Inner-half crop of each eye (matches ./stereo_trt reference).
    // Left output: cols 720 to 1919 (inner half of left camera)
    int leftCamStart = cropMargin;
    leftOut[outIdx] = input[outY * stereoWidth + leftCamStart + outX];

    // Right output: cols 1920 to 3119 (inner half of right camera)
    int rightCamStart = stereoWidth - cropMargin - outputSize;
    rightOut[outIdx] = input[outY * stereoWidth + rightCamStart + outX];
}

// Flip modes for CUDA processing (replaces slow GStreamer videoflip)
// Mode 0: no flip
// Mode 1: rotate-180 (flip both X and Y)
// Mode 2: vertical-flip (flip Y only)
// Mode 3: horizontal-flip (flip X only)
__global__ void stereoCropSplitFlipKernel(
    const uchar3* __restrict__ input,
    uchar3* __restrict__ leftOut,
    uchar3* __restrict__ rightOut,
    int stereoWidth,
    int stereoHeight,
    int outputSize,
    int cropMargin,
    int flipMode
)
{
    int outX = blockIdx.x * blockDim.x + threadIdx.x;
    int outY = blockIdx.y * blockDim.y + threadIdx.y;

    if (outX >= outputSize || outY >= outputSize)
        return;

    int outIdx = outY * outputSize + outX;

    // Calculate input coordinates based on flip mode
    int inY, inX_offset;
    switch (flipMode) {
        case 1:  // rotate-180: flip both X and Y
            inY = stereoHeight - 1 - outY;
            inX_offset = outputSize - 1 - outX;
            break;
        case 2:  // vertical-flip: flip Y only
            inY = stereoHeight - 1 - outY;
            inX_offset = outX;
            break;
        case 3:  // horizontal-flip: flip X only
            inY = outY;
            inX_offset = outputSize - 1 - outX;
            break;
        default: // no flip
            inY = outY;
            inX_offset = outX;
            break;
    }

    // Inner-half crop of each eye (matches ./stereo_trt reference).
    // Left output: cols 720 to 1919 (inner half of left camera)
    int leftCamStart = cropMargin;
    leftOut[outIdx] = input[inY * stereoWidth + leftCamStart + inX_offset];

    // Right output: cols 1920 to 3119 (inner half of right camera)
    int rightCamStart = stereoWidth - cropMargin - outputSize;
    rightOut[outIdx] = input[inY * stereoWidth + rightCamStart + inX_offset];
}

extern "C" cudaError_t cudaStereoCropSplitRotate(
    const uchar3* input,
    uchar3* leftOut,
    uchar3* rightOut,
    int stereoWidth,
    int stereoHeight,
    int outputSize,
    cudaStream_t stream
)
{
    int leftCamStart = stereoWidth - outputSize;  // 3840 - 1200 = 2640

    dim3 blockDim(16, 16);
    dim3 gridDim(
        (outputSize + blockDim.x - 1) / blockDim.x,
        (outputSize + blockDim.y - 1) / blockDim.y
    );

    stereoCropSplitRotateKernel<<<gridDim, blockDim, 0, stream>>>(
        input, leftOut, rightOut, stereoWidth, outputSize, leftCamStart
    );

    return cudaGetLastError();
}

extern "C" cudaError_t cudaStereoCropSplit(
    const uchar3* input,
    uchar3* leftOut,
    uchar3* rightOut,
    int stereoWidth,
    int stereoHeight,
    int outputSize,
    cudaStream_t stream
)
{
    // Inner-half crop: left region 720-1919, right region 1920-3119
    // Matches Fast-FoundationStereo/cpp_demo/stereo_trt.cu crop_split_rotate.
    int cropMargin = 720;

    dim3 blockDim(16, 16);
    dim3 gridDim(
        (outputSize + blockDim.x - 1) / blockDim.x,
        (outputSize + blockDim.y - 1) / blockDim.y
    );

    stereoCropSplitKernel<<<gridDim, blockDim, 0, stream>>>(
        input, leftOut, rightOut, stereoWidth, outputSize, cropMargin
    );

    return cudaGetLastError();
}

// Stereo crop/split with configurable flip mode (replaces slow GStreamer videoflip)
// flipMode: 0=none, 1=rotate-180, 2=vertical-flip, 3=horizontal-flip
extern "C" cudaError_t cudaStereoCropSplitFlip(
    const uchar3* input,
    uchar3* leftOut,
    uchar3* rightOut,
    int stereoWidth,
    int stereoHeight,
    int outputSize,
    int flipMode,
    cudaStream_t stream
)
{
    // Inner-half crop: left region 720-1919, right region 1920-3119
    // Matches Fast-FoundationStereo/cpp_demo/stereo_trt.cu crop_split_rotate.
    int cropMargin = 720;

    dim3 blockDim(16, 16);
    dim3 gridDim(
        (outputSize + blockDim.x - 1) / blockDim.x,
        (outputSize + blockDim.y - 1) / blockDim.y
    );

    stereoCropSplitFlipKernel<<<gridDim, blockDim, 0, stream>>>(
        input, leftOut, rightOut, stereoWidth, stereoHeight, outputSize, cropMargin, flipMode
    );

    return cudaGetLastError();
}
