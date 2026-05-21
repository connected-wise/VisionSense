/*
 * Fused stereo-rectify preprocessing for LightStereo / FFS TRT engine input.
 *
 * One kernel per eye consumes:
 *   - raw RGB image at full eye resolution (e.g. 1440x900 uchar3)
 *   - a pre-baked remap of size (model_h x model_w) carrying float2 source
 *     coordinates in the raw image (filled at init via cv::initUndistortRectifyMap
 *     with the destination intrinsics scaled to model resolution)
 *
 * And produces:
 *   - rectified, resized, BGR/RGB-swappable, ImageNet-normalized, HWC->CHW
 *     float tensor at (3, model_h, model_w) — written directly into the TRT
 *     input binding for the engine.
 *
 * Fusing remap+resize+norm+transpose into one pass avoids two intermediate
 * eye-sized float buffers, two memory passes, and the Python interpreter
 * round-trip the previous stereo_depth_lightstereo.py paid per frame.
 */

#include <cuda_runtime.h>
#include <stdint.h>

__device__ __forceinline__ float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

__global__ void rectifyResizeNormCHWKernel(
    const uchar3* __restrict__ raw,     // (raw_h, raw_w)
    int raw_w, int raw_h,
    const float2* __restrict__ remap,   // (model_h, model_w) — source pixel coords
    int model_w, int model_h,
    float* __restrict__ out_chw,        // (3, model_h, model_w)
    float mean_r, float mean_g, float mean_b,
    float inv_std_r, float inv_std_g, float inv_std_b,
    int channel_order)                  // 0 = source already RGB; 1 = source BGR -> swap
{
    const int u = blockIdx.x * blockDim.x + threadIdx.x;
    const int v = blockIdx.y * blockDim.y + threadIdx.y;
    if (u >= model_w || v >= model_h) return;

    const int map_idx = v * model_w + u;
    const float2 src  = remap[map_idx];

    // Bilinear sample at (src.x, src.y) into raw image, with edge clamp.
    const float fu = src.x;
    const float fv = src.y;
    const int u0 = (int)floorf(fu);
    const int v0 = (int)floorf(fv);
    const int u1 = u0 + 1;
    const int v1 = v0 + 1;

    const int cu0 = max(0, min(raw_w - 1, u0));
    const int cu1 = max(0, min(raw_w - 1, u1));
    const int cv0 = max(0, min(raw_h - 1, v0));
    const int cv1 = max(0, min(raw_h - 1, v1));

    const float du = fu - (float)u0;
    const float dv = fv - (float)v0;
    const float w00 = (1.0f - du) * (1.0f - dv);
    const float w01 =         du  * (1.0f - dv);
    const float w10 = (1.0f - du) *         dv;
    const float w11 =         du  *         dv;

    const uchar3 p00 = raw[cv0 * raw_w + cu0];
    const uchar3 p01 = raw[cv0 * raw_w + cu1];
    const uchar3 p10 = raw[cv1 * raw_w + cu0];
    const uchar3 p11 = raw[cv1 * raw_w + cu1];

    float c0 = (float)p00.x * w00 + (float)p01.x * w01 + (float)p10.x * w10 + (float)p11.x * w11;
    float c1 = (float)p00.y * w00 + (float)p01.y * w01 + (float)p10.y * w10 + (float)p11.y * w11;
    float c2 = (float)p00.z * w00 + (float)p01.z * w01 + (float)p10.z * w10 + (float)p11.z * w11;

    // If raw is BGR, swap so the network sees R,G,B.
    if (channel_order == 1) {
        const float tmp = c0; c0 = c2; c2 = tmp;
    }

    // /255 + ImageNet norm.
    const float r = (c0 * (1.0f / 255.0f) - mean_r) * inv_std_r;
    const float g = (c1 * (1.0f / 255.0f) - mean_g) * inv_std_g;
    const float b = (c2 * (1.0f / 255.0f) - mean_b) * inv_std_b;

    // HWC -> CHW write to the TRT input binding.
    const int hw   = model_h * model_w;
    const int pidx = v * model_w + u;
    out_chw[0 * hw + pidx] = r;
    out_chw[1 * hw + pidx] = g;
    out_chw[2 * hw + pidx] = b;
}

extern "C" cudaError_t cudaStereoRectifyResizeNormCHW(
    const uchar3* raw_rgb_or_bgr,
    int raw_w, int raw_h,
    const float2* remap_map,
    int model_w, int model_h,
    float* out_chw_f32,
    float mean_r, float mean_g, float mean_b,
    float inv_std_r, float inv_std_g, float inv_std_b,
    int channel_order_bgr,   // 0 = RGB input, 1 = BGR input
    cudaStream_t stream)
{
    dim3 blockDim(16, 16);
    dim3 gridDim((model_w + blockDim.x - 1) / blockDim.x,
                 (model_h + blockDim.y - 1) / blockDim.y);

    rectifyResizeNormCHWKernel<<<gridDim, blockDim, 0, stream>>>(
        raw_rgb_or_bgr, raw_w, raw_h,
        remap_map, model_w, model_h,
        out_chw_f32,
        mean_r, mean_g, mean_b,
        inv_std_r, inv_std_g, inv_std_b,
        channel_order_bgr);

    return cudaGetLastError();
}
