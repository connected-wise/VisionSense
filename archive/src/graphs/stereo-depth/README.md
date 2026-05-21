# Fast-FoundationStereo — Engine Build Guide

This folder contains everything needed to build the Fast-FoundationStereo
stereo-depth TensorRT engine on an NVIDIA Jetson Orin device. The
`stereo_depth` ROS node (`src/node_stereo_depth.cpp`) loads
`stereo_plugin.engine` together with the custom `libgwc_plugin.so` plugin.

## Folder contents

| File | Size | Purpose |
|---|---|---|
| `stereo_combined.onnx` | ~60 MB | Single-engine ONNX export (EdgeNeXt backbone + GWC volume + 3D hourglass + GRU refinement). Re-buildable from PyTorch via `export_combined.py`. |
| `stereo_combined.yaml` | <1 KB | Export args (image_size 480×480, valid_iters=8, max_disp=192). |
| `gwc_plugin.cu` | ~7 KB | TensorRT `IPluginV2DynamicExt` for the Group-wise Correlation cost volume. |
| `Makefile` | <1 KB | Builds `libgwc_plugin.so` (and the optional standalone `stereo_trt` demo). |
| `export_combined.py` | ~9 KB | PyTorch `.pth` → ONNX exporter. Only needed to regenerate the ONNX from a different checkpoint or input resolution. |
| `libgwc_plugin.so` | ~53 KB | Pre-built plugin for `sm_87` (Orin Nano / NX / AGX). Rebuild for other architectures. |
| `stereo_plugin.engine` | ~37 MB | Pre-built TensorRT engine for `sm_87` at 480×480 / `valid_iters=8`. **Engine files are not portable** across Jetson revisions — rebuild on the target device. |

## Build dependencies (Jetson Orin)

| Component | Tested version |
|---|---|
| JetPack | 6.1+ (L4T R36.4.x) |
| CUDA | 12.6 |
| TensorRT | 10.3 |
| OpenCV | 4.x with GStreamer (only for the optional standalone demo) |

The runtime ROS node has zero Python dependencies. Python is only needed if
you want to re-export the ONNX from a PyTorch checkpoint.

## Generating the TensorRT engine on a new device

Run these from this folder (`src/graphs/stereo-depth/`):

```bash
# 1. Build the GWC plugin for the target SM. Edit ARCH in the Makefile if not Orin (sm_87).
make libgwc_plugin.so

# 2. Build the TensorRT engine from the shipped ONNX, with the plugin linked in.
/usr/src/tensorrt/bin/trtexec \
    --onnx=stereo_combined.onnx \
    --saveEngine=stereo_plugin.engine \
    --staticPlugins=./libgwc_plugin.so \
    --fp16
```

The build takes ~30 minutes on Orin Nano the first time. The resulting
`stereo_plugin.engine` is reusable on the same device.

## Re-exporting the ONNX from PyTorch (optional)

Only needed if you want a different input resolution, a different
`valid_iters`, or a different checkpoint. Requires PyTorch 2.8+ (Jetson
wheel) and the Fast-FoundationStereo PyTorch source tree at
`/home/jetson/Fast-FoundationStereo/`:

```bash
python3 export_combined.py \
    --model_dir  /home/jetson/Fast-FoundationStereo/weights/23-36-37/model_best_bp2_serialize.pth \
    --save_path  . \
    --height 480 --width 480 \
    --valid_iters 8 --max_disp 192
```

Then rebuild the engine with the `trtexec` command above.

### Performance knobs

| Knob | Effect |
|---|---|
| Smaller `--height/--width` | Quadratic speed-up. 320×320 ≈ 2× faster than 480×480. |
| Lower `--valid_iters` (min 1) | Each iteration is ~8 ms. 8→4 saves ~32 ms. |
| Lighter checkpoint (`20-30-48` instead of `23-36-37`) | ~15–20 % faster on the hourglass. |
| Higher `--max_disp` | Linear in cost-volume size — keep at 192 unless more range is needed. |

## Runtime integration

`src/node_stereo_depth.cpp` does the following at startup:

1. `dlopen("libgwc_plugin.so", RTLD_NOW)` — **must** precede engine
   deserialisation, otherwise `deserializeCudaEngine` returns null.
2. `IRuntime::deserializeCudaEngine(stereo_plugin.engine)`.
3. Allocates device buffers for `left`, `right`, `disp` tensors.

Per frame:

1. CUDA preprocessing kernel (`src/cuda/stereo_preproc.cu`) does HWC `uint8`
   BGR → CHW `float32` RGB directly into the engine input buffers (raw
   `[0, 255]` — mean/std normalisation is baked into the engine).
2. `enqueueV3` runs the engine.
3. The disparity output (`[1, 1, 480, 480]` float32) is copied back and
   converted to depth via `depth = fx · baseline / disparity`.

Both files (`stereo_plugin.engine` and `libgwc_plugin.so`) are referenced
from `config/config.yaml` under `stereo_depth.ros__parameters`.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `deserializeCudaEngine` returns null | Plugin not loaded before deserialise | Verify `dlopen("libgwc_plugin.so", RTLD_NOW)` runs first |
| `cudaErrorNoKernelImageForDevice` on first inference | Plugin or engine built for wrong SM | Rebuild both on the target device |
| GPU compute ~250 ms instead of ~190 ms | GPU clock throttled | `sudo jetson_clocks` |
| Disparity inverted or near-zero | Left/right swapped | Toggle `rotated_lenses` in the stereo camera node |

## References

- Upstream PyTorch source: `/home/jetson/Fast-FoundationStereo/`
- Standalone C++ demo: `/home/jetson/Fast-FoundationStereo/cpp_demo/`
- Model card: `/home/jetson/Fast-FoundationStereo/model_card.md`
