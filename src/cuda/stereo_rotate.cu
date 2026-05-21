/*
 * CUDA kernels for stereo image processing.
 *
 * Rotated lenses (rotated_lenses=true) — LEGACY square 1200x1200 per eye:
 *   90° per-eye rotation. Crop center 1440px:
 *   [right_cam:0-1199 | skip:1440 | left_cam:2640-3839]
 *   Output is forced square because the 90° rotation maps sensor-rows (1200)
 *   into natural-width, capping natural-width at 1200. Cannot produce
 *   non-square outputs larger than 1200 in natural-width.
 *
 * Non-rotated lenses (rotated_lenses=false) — 1440x900 per eye:
 *   Inner-edge crop: each eye keeps its inner 1440 columns (closer to the
 *   other eye) and the central 900 rows (150 trimmed top + 150 trimmed bottom).
 *     left  output  = input cols [480 .. 1919], rows [150 .. 1049]
 *     right output  = input cols [1920 .. 3359], rows [150 .. 1049]
 *   The inner-edge crop preserves the short stereo baseline used by the
 *   Fast-FoundationStereo reference geometry. Output dims are hardcoded —
 *   change CONSTS_OUT_W / CONSTS_OUT_H below if you ever need to retune.
 */

// Per-eye output dimensions for the non-rotated path.
// 1920 (per-eye sensor width) → 1440 keeps inner edge; 1200 (sensor height) →
// 900 trims 150 from top + 150 from bottom.
#define STEREO_OUT_W       1440
#define STEREO_OUT_H        900
#define STEREO_CROP_MARGIN_X 480   // (1920 - 1440), all taken from outer side
#define STEREO_CROP_MARGIN_Y 150   // (1200 -  900) / 2, symmetric

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
    int stereoWidth
)
{
    int outX = blockIdx.x * blockDim.x + threadIdx.x;
    int outY = blockIdx.y * blockDim.y + threadIdx.y;

    if (outX >= STEREO_OUT_W || outY >= STEREO_OUT_H)
        return;

    int outIdx = outY * STEREO_OUT_W + outX;

    // Inner-edge crop, vertically centered.
    // Left  output: cols [480 .. 1919] (inner 1440 of left camera).
    // Right output: cols [1920 .. 3359] (inner 1440 of right camera).
    int inY = STEREO_CROP_MARGIN_Y + outY;

    int leftCamStart  = STEREO_CROP_MARGIN_X;
    int rightCamStart = stereoWidth - STEREO_CROP_MARGIN_X - STEREO_OUT_W;

    leftOut [outIdx] = input[inY * stereoWidth + leftCamStart  + outX];
    rightOut[outIdx] = input[inY * stereoWidth + rightCamStart + outX];
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
    int flipMode
)
{
    int outX = blockIdx.x * blockDim.x + threadIdx.x;
    int outY = blockIdx.y * blockDim.y + threadIdx.y;

    if (outX >= STEREO_OUT_W || outY >= STEREO_OUT_H)
        return;

    int outIdx = outY * STEREO_OUT_W + outX;

    // Flip happens WITHIN the cropped output window. For an upside-down camera
    // with cuda_flip=rotate-180, the produced 1440×900 image is rotated 180°,
    // not the full sensor — which is what consumers expect.
    int srcX, srcY;
    switch (flipMode) {
        case 1:  // rotate-180: flip both X and Y within the crop window
            srcX = STEREO_OUT_W - 1 - outX;
            srcY = STEREO_OUT_H - 1 - outY;
            break;
        case 2:  // vertical-flip
            srcX = outX;
            srcY = STEREO_OUT_H - 1 - outY;
            break;
        case 3:  // horizontal-flip
            srcX = STEREO_OUT_W - 1 - outX;
            srcY = outY;
            break;
        default: // no flip
            srcX = outX;
            srcY = outY;
            break;
    }

    int inY = STEREO_CROP_MARGIN_Y + srcY;

    int leftCamStart  = STEREO_CROP_MARGIN_X;
    int rightCamStart = stereoWidth - STEREO_CROP_MARGIN_X - STEREO_OUT_W;

    leftOut [outIdx] = input[inY * stereoWidth + leftCamStart  + srcX];
    rightOut[outIdx] = input[inY * stereoWidth + rightCamStart + srcX];
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

// Non-rotated path → produces STEREO_OUT_W × STEREO_OUT_H per eye.
extern "C" cudaError_t cudaStereoCropSplit(
    const uchar3* input,
    uchar3* leftOut,
    uchar3* rightOut,
    int stereoWidth,
    int stereoHeight,
    cudaStream_t stream
)
{
    (void)stereoHeight;  // crop is fixed via STEREO_CROP_MARGIN_Y

    dim3 blockDim(16, 16);
    dim3 gridDim(
        (STEREO_OUT_W + blockDim.x - 1) / blockDim.x,
        (STEREO_OUT_H + blockDim.y - 1) / blockDim.y
    );

    stereoCropSplitKernel<<<gridDim, blockDim, 0, stream>>>(
        input, leftOut, rightOut, stereoWidth
    );

    return cudaGetLastError();
}

// Non-rotated path with optional flip applied inside the crop window.
// flipMode: 0=none, 1=rotate-180, 2=vertical-flip, 3=horizontal-flip
extern "C" cudaError_t cudaStereoCropSplitFlip(
    const uchar3* input,
    uchar3* leftOut,
    uchar3* rightOut,
    int stereoWidth,
    int stereoHeight,
    int flipMode,
    cudaStream_t stream
)
{
    (void)stereoHeight;

    dim3 blockDim(16, 16);
    dim3 gridDim(
        (STEREO_OUT_W + blockDim.x - 1) / blockDim.x,
        (STEREO_OUT_H + blockDim.y - 1) / blockDim.y
    );

    stereoCropSplitFlipKernel<<<gridDim, blockDim, 0, stream>>>(
        input, leftOut, rightOut, stereoWidth, flipMode
    );

    return cudaGetLastError();
}
