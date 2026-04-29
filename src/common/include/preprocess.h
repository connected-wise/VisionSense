#pragma once

#include <cstdint>
#include <cuda_runtime.h>

// Fused preprocessing kernel.
//
// Reads an interleaved 3-channel uint8 image from device memory, performs
// bilinear resize (optionally aspect-ratio-preserving with right/bottom zero
// padding), optionally swaps the outer two channels (BGR<->RGB), normalizes,
// and writes the result as planar (CHW) float32 directly into the destination
// tensor buffer (typically a TensorRT input binding).
//
// dst layout: 3 planes of dst_w*dst_h floats, in the order C0, C1, C2 where
// C0 corresponds to the first channel of the source (or the last, if swap_rb).
void launchPreprocess(
    const uint8_t* src, int src_w, int src_h, size_t src_pitch_bytes,
    float* dst, int dst_w, int dst_h,
    bool swap_rb, bool aspect_ratio_pad, bool normalize,
    float sub0, float sub1, float sub2,
    float div0, float div1, float div2,
    cudaStream_t stream);
