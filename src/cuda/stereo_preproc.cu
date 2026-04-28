/*
 * GPU preprocessing kernel for Fast-FoundationStereo.
 *
 * Converts an HWC uint8 RGB image (as delivered by node_camera_stereo.cpp,
 * encoded as "rgb8") into a CHW float32 RGB tensor suitable for direct
 * ingestion by the Fast-FoundationStereo TensorRT engine. The model expects
 * raw [0, 255] values — mean/std normalisation is baked into the engine.
 *
 * This mirrors the `bgr_hwc_u8_to_rgb_chw_f32` kernel in
 * /home/jetson/Fast-FoundationStereo/cpp_demo/stereo_trt.cu, but does NOT
 * swap channels because camera_stereo already publishes RGB bytes.
 */

#include <cuda_runtime.h>
#include <stdint.h>

__global__ void hwc_u8_to_chw_f32_rgb_kernel(const uint8_t* __restrict__ in,
                                              float* __restrict__ out,
                                              int H, int W)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= W || y >= H) return;

    int hwc = (y * W + x) * 3;
    uint8_t r = in[hwc + 0];
    uint8_t g = in[hwc + 1];
    uint8_t b = in[hwc + 2];

    out[(0 * H + y) * W + x] = static_cast<float>(r);
    out[(1 * H + y) * W + x] = static_cast<float>(g);
    out[(2 * H + y) * W + x] = static_cast<float>(b);
}

extern "C" cudaError_t cudaHwcU8ToChwF32Rgb(
    const uint8_t* d_in,
    float*         d_out,
    int            H,
    int            W,
    cudaStream_t   stream)
{
    dim3 block(16, 16);
    dim3 grid((W + 15) / 16, (H + 15) / 16);
    hwc_u8_to_chw_f32_rgb_kernel<<<grid, block, 0, stream>>>(d_in, d_out, H, W);
    return cudaGetLastError();
}
