/*
 * CUDA kernels for stereo image processing
 * Input: 3840x1200 -> Two 1200x1200 outputs
 *
 * Rotated lenses (rotated_lenses=true):
 *   Crop center 1440px: [right_cam:0-1199 | skip:1440 | left_cam:2640-3839]
 *
 * Non-rotated lenses (rotated_lenses=false):
 *   Crop 360px from sides: [skip:360 | left:360-1559 | gap:720 | right:2280-3479 | skip:360]
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

    // Crop 360px from left and right sides, then split remaining region
    // Left output: cols 360 to 1559 (after left margin)
    int leftCamStart = cropMargin;
    leftOut[outIdx] = input[outY * stereoWidth + leftCamStart + outX];

    // Right output: cols 2280 to 3479 (before right margin)
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

    // Left output: cols 360 to 1559 (after left margin)
    int leftCamStart = cropMargin;
    leftOut[outIdx] = input[inY * stereoWidth + leftCamStart + inX_offset];

    // Right output: cols 2280 to 3479 (before right margin)
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
    // Crop 360px from each side: left region 360-1559, right region 2280-3479
    int cropMargin = 360;

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
    int cropMargin = 360;

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
