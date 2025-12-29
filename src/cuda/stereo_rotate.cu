/*
 * CUDA kernels for stereo image processing
 * Input: 3840x1200 -> Crop center 1440px -> Two 1200x1200 outputs
 * Layout: [right_cam:1200 | skip:1440 | left_cam:1200]
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
    int leftCamStart
)
{
    int outX = blockIdx.x * blockDim.x + threadIdx.x;
    int outY = blockIdx.y * blockDim.y + threadIdx.y;

    if (outX >= outputSize || outY >= outputSize)
        return;

    int outIdx = outY * outputSize + outX;

    // Left output from left camera (cols 2640-3839)
    leftOut[outIdx] = input[outY * stereoWidth + leftCamStart + outX];

    // Right output from right camera (cols 0-1199)
    rightOut[outIdx] = input[outY * stereoWidth + outX];
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
    int leftCamStart = stereoWidth - outputSize;  // 3840 - 1200 = 2640

    dim3 blockDim(16, 16);
    dim3 gridDim(
        (outputSize + blockDim.x - 1) / blockDim.x,
        (outputSize + blockDim.y - 1) / blockDim.y
    );

    stereoCropSplitKernel<<<gridDim, blockDim, 0, stream>>>(
        input, leftOut, rightOut, stereoWidth, outputSize, leftCamStart
    );

    return cudaGetLastError();
}
