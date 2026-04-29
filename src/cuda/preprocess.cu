#include "preprocess.h"

#include <cuda_runtime.h>

namespace {

__global__ void preprocessKernel(
    const uint8_t* __restrict__ src,
    int src_w, int src_h, int src_pitch,
    float* __restrict__ dst,
    int dst_w, int dst_h,
    int unpad_w, int unpad_h,
    bool swap_rb, bool normalize,
    float sub0, float sub1, float sub2,
    float div0, float div1, float div2)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) return;

    const int plane = dst_w * dst_h;
    const int didx = y * dst_w + x;

    float c0, c1, c2;

    if (x >= unpad_w || y >= unpad_h) {
        c0 = c1 = c2 = 0.0f;
    } else {
        const float scale_x = static_cast<float>(src_w) / unpad_w;
        const float scale_y = static_cast<float>(src_h) / unpad_h;
        float gx = (x + 0.5f) * scale_x - 0.5f;
        float gy = (y + 0.5f) * scale_y - 0.5f;
        if (gx < 0.0f) gx = 0.0f;
        if (gy < 0.0f) gy = 0.0f;
        int x0 = static_cast<int>(gx);
        int y0 = static_cast<int>(gy);
        int x1 = min(src_w - 1, x0 + 1);
        int y1 = min(src_h - 1, y0 + 1);
        const float ax = gx - x0;
        const float ay = gy - y0;

        const uint8_t* row0 = src + y0 * src_pitch;
        const uint8_t* row1 = src + y1 * src_pitch;
        const uint8_t* p00 = row0 + x0 * 3;
        const uint8_t* p01 = row0 + x1 * 3;
        const uint8_t* p10 = row1 + x0 * 3;
        const uint8_t* p11 = row1 + x1 * 3;

        const float w00 = (1.0f - ax) * (1.0f - ay);
        const float w01 = ax * (1.0f - ay);
        const float w10 = (1.0f - ax) * ay;
        const float w11 = ax * ay;

        c0 = p00[0] * w00 + p01[0] * w01 + p10[0] * w10 + p11[0] * w11;
        c1 = p00[1] * w00 + p01[1] * w01 + p10[1] * w10 + p11[1] * w11;
        c2 = p00[2] * w00 + p01[2] * w01 + p10[2] * w10 + p11[2] * w11;
    }

    if (swap_rb) {
        float t = c0;
        c0 = c2;
        c2 = t;
    }

    if (normalize) {
        const float inv255 = 1.0f / 255.0f;
        c0 *= inv255;
        c1 *= inv255;
        c2 *= inv255;
    }

    c0 = (c0 - sub0) / div0;
    c1 = (c1 - sub1) / div1;
    c2 = (c2 - sub2) / div2;

    dst[0 * plane + didx] = c0;
    dst[1 * plane + didx] = c1;
    dst[2 * plane + didx] = c2;
}

} // namespace

void launchPreprocess(
    const uint8_t* src, int src_w, int src_h, size_t src_pitch_bytes,
    float* dst, int dst_w, int dst_h,
    bool swap_rb, bool aspect_ratio_pad, bool normalize,
    float sub0, float sub1, float sub2,
    float div0, float div1, float div2,
    cudaStream_t stream)
{
    int unpad_w = dst_w;
    int unpad_h = dst_h;
    if (aspect_ratio_pad) {
        const float r = fminf(static_cast<float>(dst_w) / src_w,
                              static_cast<float>(dst_h) / src_h);
        unpad_w = static_cast<int>(r * src_w);
        unpad_h = static_cast<int>(r * src_h);
        if (unpad_w < 1) unpad_w = 1;
        if (unpad_h < 1) unpad_h = 1;
    }

    const dim3 block(16, 16);
    const dim3 grid((dst_w + block.x - 1) / block.x,
                    (dst_h + block.y - 1) / block.y);

    preprocessKernel<<<grid, block, 0, stream>>>(
        src, src_w, src_h, static_cast<int>(src_pitch_bytes),
        dst, dst_w, dst_h,
        unpad_w, unpad_h,
        swap_rb, normalize,
        sub0, sub1, sub2,
        div0, div1, div2);
}
