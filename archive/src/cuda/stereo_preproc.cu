/*
 * GPU preprocessing kernel for Fast-FoundationStereo.
 *
 * Converts an HWC uint8 RGB image (as delivered by node_camera_stereo.cpp,
 * encoded as "rgb8") into a CHW float32 RGB tensor suitable for direct
 * ingestion by the Fast-FoundationStereo TensorRT engine.
 *
 * The single-ONNX export (scripts/make_single_onnx.py) STRIPS the in-graph
 * normalize_image step (it monkey-patches it to identity before tracing), so
 * the engine expects ImageNet-normalised float32:
 *     out = (pixel_uint8 - mean) / std
 *     mean = [123.675, 116.28, 103.53]    (ImageNet, 0-255 scale)
 *     std  = [ 58.395,  57.12,  57.375]
 *
 * This must match Fast-FoundationStereo/scripts/run_demo_single_trt.py's
 * normalize_imagenet(): ((img/255) - [0.485,0.456,0.406]) / [0.229,0.224,0.225]
 * which is algebraically identical to the 0-255-scale form above.
 */

#include <cuda_runtime.h>
#include <stdint.h>

__constant__ float c_mean[3] = {123.675f, 116.28f, 103.53f};
__constant__ float c_std [3] = { 58.395f,  57.12f,  57.375f};

__global__ void hwc_u8_to_chw_f32_rgb_kernel(const uint8_t* __restrict__ in,
                                              float* __restrict__ out,
                                              int H, int W)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= W || y >= H) return;

    int hwc = (y * W + x) * 3;
    float r = static_cast<float>(in[hwc + 0]);
    float g = static_cast<float>(in[hwc + 1]);
    float b = static_cast<float>(in[hwc + 2]);

    out[(0 * H + y) * W + x] = (r - c_mean[0]) / c_std[0];
    out[(1 * H + y) * W + x] = (g - c_mean[1]) / c_std[1];
    out[(2 * H + y) * W + x] = (b - c_mean[2]) / c_std[2];
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
