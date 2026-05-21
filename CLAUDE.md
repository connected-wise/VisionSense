# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

VisionSense (package name `visionconnect`) is a ROS2 Humble computer vision system for autonomous vehicles on NVIDIA Jetson Orin Nano/NX. Object detection, traffic-sign classification, lane detection, stereo depth, driver monitoring, BEV mapping, GPS/IMU fusion, web dashboard, native GUI — all wired into a single launch graph.

Hardware target: **Jetson Orin NX 16 GB** with JetPack 6.2 (CUDA 12.6, TensorRT 10, OpenCV 4.8). Cameras: Arducam dual-AR0234 stereo (3840×1200 over `/dev/video1`) + IMX219 mono CSI (driver-monitor).

## Build, Run, Test

```bash
# From workspace root
colcon build --packages-select visionconnect
source install/setup.bash

# Full pipeline (always launch via the helper, NOT plain `ros2 launch` —
# the script sets FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA, sweeps stale
# /dev/shm/fastrtps_* segments, and kicks the IMX219 service post-discovery)
./launch_visionsense.sh
```

Individual nodes for debugging:
```bash
ros2 run visionconnect detect
ros2 run visionconnect lanedet
ros2 run visionconnect stereo_depth          # C++ stereo depth (default in launch)
ros2 run visionconnect stereo_depth_lightstereo.py   # Python fallback (installed alongside)
```

## Architecture

### Inference graph (all C++ except where noted)

| Node | Source | Purpose |
|---|---|---|
| `camera` | `src/node_camera.cpp` | Mono camera capture (used outside the full launch — see IMX219 note below) |
| `camera_stereo` | `src/node_camera_stereo.cpp` | Arducam stereo: capture 3840×1200, CUDA crop+split+flip into two 1440×900 rgb8 streams, publish + `CameraInfo` from `stereo_calib.yaml` |
| `detect` | `src/node_detect.cpp` | YOLOv8 (TensorRT) + BYTE tracker, publishes detections + cropped traffic-sign sub-images |
| `classify` | `src/node_classify.cpp` | YOLOv8-cls (TensorRT) refining sign sub-images |
| `lanedet` | `src/node_lanedet.cpp` | Lane segmentation (TensorRT) |
| `stereo_depth` | `src/node_stereo_depth.cpp` | LightStereo-S (TensorRT) — fused CUDA rectify+resize+norm+CHW preprocess in `src/cuda/stereo_rectify.cu`, two-input TRT inference, depth + colormap publish |
| `adas` | `src/node_adas.cpp` | Lane-departure HUD overlay |
| `bev` | `src/node_bev.cpp` | Bird's-eye view rasterization with IMU/GPS heading |
| `driver_monitor` | `src/node_driver_monitor.cpp` | YOLOv11-face + ResNet18-gaze, alert states |
| `dashboard` | `src/node_dashboard.cpp` | Embedded HTTP + WebSocket server on `:8080` (libwebsockets). Sole web entry point — no rosbridge, no `python3 -m http.server` sidecar. |
| `gui` | `src/node_gui.cpp` | Composite cv::imshow fusion view |
| `imu_gps` | `src/node_imu_gps.cpp` | IMU/GPS publisher (gpsd + serial IMU) |

### Common libraries

`src/common/` (shared static lib):
- `trtutil.{h,cpp}` — generic `Engine` class (used by detect/lanedet/classify). Note: `stereo_depth` does NOT use it (writes raw nvinfer1 calls directly because it needs two inputs and a custom CUDA preprocess).
- `image_converter.{h,cpp}` — jetson-utils bridging
- `BYTETracker`, `kalmanFilter`, `STrack`, `lapjv` — tracking
- `ros_compat.h` — `ROS_INFO/WARN/ERROR` macros (no `_THROTTLE` variants; gate with a frame counter if needed)

### Critical data-flow facts

1. **`/camera/raw` is published by a systemd unit**, not by `visionsense.launch.py`. The IMX219 driver-monitor camera lives in `visionsense-imx219.service` — see [IMX219 reopen bug](#imx219-reopen-bug-and-systemd-service) below.

2. **Stereo eye topic labels match the physical sensor wiring after the user's sensor swap.** The old in-code `cudaStereoCropSplitFlip(... right_gpu, left_gpu, ...)` swap was removed; the kernel now goes straight `(leftOut → /camera_stereo/left, rightOut → /camera_stereo/right)`. `stereo.main_eye` in `config/config.yaml` was flipped from `right` to `left` to match.

3. **Stereo depth requires rectification**, done inside `stereo_depth` via `cv::initUndistortRectifyMap` (at MODEL resolution, not eye resolution — the maps are baked into a fused CUDA kernel). `camera_stereo` does NOT rectify; it publishes raw cropped+rotated images. Other consumers (detect/lanedet/gui) get raw images and don't need rectification.

4. **DDS configuration is in the launch script, not the launch file.** `launch_visionsense.sh` sets `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`. Bypassing the script breaks one stereo eye to ~2 Hz.

5. **The dashboard is `node_dashboard` itself** — it embeds libwebsockets and serves `launch/dashboard/dashboard.html` directly. Don't add a sidecar HTTP server.

## Key Files / Config

- `config/config.yaml` — all node parameters, plus `stereo.main_eye`, `ui.display`/`ui.dashboard` toggles
- `config/stereo_calib.yaml` — stereo calibration produced by `scripts/compute_stereo_calib.py`
- `config/stereo_calib.yaml.tz41` — backup with the optimizer's "raw" (Tz=-41 mm) stereo extrinsics. Production `stereo_calib.yaml` has Ty/Tz forced to 0 to avoid degenerate rectification.
- `fastdds_profile.shm.xml` — pure-SHM transport for the 3-node `test_stereo_depth.launch.py` debug graph. NOT used by the full pipeline.
- `launch/visionsense.launch.py` — full pipeline launch
- `launch/dashboard/dashboard.html` — UI frontend served by `node_dashboard`

## Stereo Calibration Workflow

When the cameras are bumped / re-seated / swapped, regenerate `config/stereo_calib.yaml`:

```bash
# 1. Free /dev/video1 (only one process can hold it). Stop VisionSense.
cd scripts
python3 capture_stereo_calib.py          # SPACE to save; saves only when both eyes detect
                                          # Defaults: 7x5 inner corners, 30 mm squares, rotate-180

# 2. Compute
python3 compute_stereo_calib.py \
    --dir ./stereo_calib_images \
    --out ../config/stereo_calib.yaml \
    --fix-aspect
# Iterative outlier rejection is on by default (--max-reject-rounds 4,
# --reject-threshold 1.5 px)
```

Expected calibration quality:
- per-eye RMS < 0.5 px
- stereo RMS < 1.0 px
- ||T|| ≈ 100 mm (matches Arducam baseline)
- |Tx| >> |Ty|, |Tz|

If `|Tz|` ends up large (the optimizer attributing residual error to phantom Z baseline), the `stereoRectify` zoom factor explodes and the visible FOV shrinks. The fallback is to manually set `Ty=Tz=0` in the YAML and re-run `stereoRectify` — see the workflow at the bottom of the comment block at the top of `config/stereo_calib.yaml`.

## IMX219 Reopen Bug and Systemd Service

The Jetson Argus stack (libargus + nvargus-daemon + tegra-camrtc) does not survive a Producer close on this rig — every second `nvarguscamerasrc` session fails with `INVALID_SETTINGS` / `Argus Correctable Error Status` until reboot. Bare `gst-launch-1.0 nvarguscamerasrc ! ...` reproduces it, so the bug is in NVIDIA's stack, not VisionSense.

**Workaround:** the IMX219 is owned by `visionsense-imx219.service` — a systemd unit that opens the camera once at boot, never closes it, and publishes `/camera/raw` for the entire uptime. VisionSense's `visionsense.launch.py` does NOT instantiate a `camera` node; it just subscribes to `/camera/raw`.

The unit:
- File: `scripts/visionsense-imx219.service` (source-controlled) → installed to `/etc/systemd/system/`
- Pre-flight: `scripts/imx219-csi-warmup.sh` does a brief Arducam V4L2 stream-on/off cycle on `/dev/video1` before opening Argus. Without this, Argus's probe of sensor 1 (the Arducam) at boot poisons IMX219 capture with cascading SCF/PD_CRC errors ~5 s in.
- Passwordless sudo for `systemctl restart visionsense-imx219.service` is set via `/etc/sudoers.d/visionsense-imx219`. `launch_visionsense.sh` uses this to re-kick the service ~3 s after subscribers come up — a fix for a FastDDS SHM late-joiner discovery race.

`install_all_deps.sh` step 17 installs all of the above.

**Do NOT `systemctl stop visionsense-imx219.service` during normal operation** — that closes Argus and triggers the bug. Only stop it immediately before a reboot.

## DDS / FastDDS Quirks

- Two `image_raw` streams = ~9 MB / 33 ms. Default 512 KB SHM segment can't hold them — `right/image_raw` drops to ~2 Hz.
- Production fix: `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` (env var set in `launch_visionsense.sh`). Kernel UDP buffers must be 16 MB (`install_all_deps.sh` writes `/etc/sysctl.d/99-ros2-fastdds.conf`).
- Multi-line `bash -c 'export FASTDDS=... && ros2 launch ...'` silently breaks the export. Verify with `cat /proc/$(pgrep -f camera_stereo)/environ | grep FASTDDS`.
- Crashed runs leave `/dev/shm/fastrtps_*` segments that choke discovery on the next launch. `launch_visionsense.sh` sweeps them automatically when nothing visionconnect-related is running.
- Test-only `test_stereo_depth.launch.py` uses `fastdds_profile.shm.xml` (64 MB SHM, no UDP) because the only meaningful traffic is the two image streams. **Don't use this profile for the full pipeline** — small detection/lane messages get starved by the same 64 MB queue.

## Custom Messages (`msg/`)

- `Box.msg`, `Detect.msg`, `Lanes.msg`, `Signs.msg`, `Track.msg`, `ADAS.msg`, `SceneData.msg`

All are referenced from `CMakeLists.txt` `rosidl_generate_interfaces` block.

## Coding Conventions Worth Knowing

- C++17, `-Ofast`, CUDA arch 87 (Orin).
- No `cv::cuda::*` — write raw CUDA kernels in `src/cuda/`. The existing `stereo_rotate.cu`, `preprocess.cu`, and `stereo_rectify.cu` are the templates.
- TRT engines are **not portable** across Jetson units / TRT versions. Regenerate from ONNX on the target device.
- C++ TRT pattern: deserialize → `setTensorAddress` → `cudaMallocAsync` GPU buffers → `cudaMemcpyAsync` H→D → `enqueueV3(stream)` → `cudaMemcpyAsync` D→H → `cudaStreamSynchronize`. See `src/node_driver_monitor.cpp` for the canonical example and `src/node_stereo_depth.cpp` for the two-input variant.
- Don't add throttled logging — `ROS_WARN_THROTTLE` etc. are not defined in `ros_compat.h`. Gate with `(frame_count_ % N) == 0` instead.

## Web Dashboard

- Single binary `node_dashboard` serves `launch/dashboard/dashboard.html` over HTTP + WebSocket on `:8080`.
- Wire protocol: binary JPEG frames (stream-id byte + data) for images, JSON for telemetry.
- `ui.dashboard: true` in `config/config.yaml` auto-opens Chromium in kiosk mode to `http://localhost:8080/` when VisionSense launches.
- **Do not add rosbridge or a separate Python HTTP server** — that architecture was rejected.

## Common Failure Modes

| Symptom | First thing to check |
|---|---|
| `/camera/raw` silent, `driver_monitor` idle | `systemctl status visionsense-imx219.service` — must be active |
| `/camera_stereo/right/image_raw` at 2 Hz, left at 30 Hz | `cat /proc/$(pgrep -f camera_stereo)/environ \| grep FASTDDS` — env var must be set |
| Topics look silent despite "ready" log | `ls /dev/shm/fastrtps_* \| wc -l` — wipe if >5–10 (`rm -f /dev/shm/fastrtps_*`) |
| Stereo depth all yellow / no structure | `depth_vmax` in `config/config.yaml` — drop to 5.0 for indoor scenes (was 20.0 by default) |
| Bogus `fx_rect`, all-black rectified sample | `config/stereo_calib.yaml` has degenerate `(R, T)`. Recapture with varied poses, or force `Ty=Tz=0`. See [Stereo Calibration Workflow](#stereo-calibration-workflow). |
| Stereo L/R labels look swapped | Check `src/node_camera_stereo.cpp:298` (the `cudaStereoCropSplitFlip` arg order) AND `stereo.main_eye` in `config.yaml`. The fix has to be in BOTH places or the calibration is also remapped. |

## When in Doubt

- Don't add sidecar processes (rosbridge, http.server, etc.) — extend an existing C++ node instead.
- Don't bypass `launch_visionsense.sh` — it does several things `ros2 launch` alone doesn't.
- Don't `systemctl stop visionsense-imx219.service` while VisionSense is running.
- Don't run multiple `compute_stereo_calib.py` in parallel — it parallelizes across all cores already, and you'll just thrash.
