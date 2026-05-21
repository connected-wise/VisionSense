# VisionSense

<p align="center">
  <img src="assets/Logo_Symbol_Dark.png" alt="VisionSense Logo" width="150"/>
</p>

<p align="center">
  <b>Advanced Autonomous Vehicle Perception System</b><br>
  Real-time perception powered by TensorRT on NVIDIA Jetson
</p>

<p align="center">
  <a href="#features">Features</a> •
  <a href="#system-architecture">Architecture</a> •
  <a href="#installation">Installation</a> •
  <a href="#usage">Usage</a> •
  <a href="#nodes">Nodes</a>
</p>

---

https://github.com/user-attachments/assets/8b5bfc2b-9bf6-4562-895b-04ba0c5b41e3

## Overview

VisionSense is a comprehensive ROS2-based computer vision system designed for autonomous vehicles running on NVIDIA Jetson platforms with JetPack 6.2. It provides a complete perception pipeline with real-time object detection, lane detection, traffic sign recognition, stereo depth estimation, and driver monitoring capabilities.

## Features

| Feature | Description | Model/Method |
|---------|-------------|--------------|
| **Object Detection** | Detect vehicles, pedestrians, cyclists, traffic signs/lights | YOLOv8 + TensorRT |
| **Multi-Object Tracking** | Track objects across frames with unique IDs | BYTE Tracker + Kalman Filter |
| **Lane Detection** | Segment and detect lane lines | Neural Network + TensorRT |
| **Traffic Sign Recognition** | Classify 50+ traffic sign types | YOLOv8 Classifier + TensorRT |
| **Stereo Depth Estimation** | Dense depth maps from stereo camera | LightStereo-S + TensorRT |
| **Driver Monitoring** | Face detection and gaze estimation | YOLOv11 + ResNet18 + TensorRT |
| **Data Fusion GUI** | Real-time visualization of all perception data | OpenCV + X11 |
| **Web Dashboard** | Remote monitoring interface | HTTP Server |

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            VisionSense Architecture                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌──────────────┐     ┌──────────────┐     ┌──────────────┐                │
│   │ Mono Camera  │     │Stereo Camera │     │   IMU/GPS    │                │
│   │  (CSI/USB)   │     │  (Arducam)   │     │   Module     │                │
│   └──────┬───────┘     └──────┬───────┘     └──────┬───────┘                │
│          │                    │                    │                         │
│          ▼                    ▼                    ▼                         │
│   ┌──────────────┐     ┌──────────────┐     ┌──────────────┐                │
│   │    camera    │     │ camera_stereo│     │   imu_gps    │                │
│   │     node     │     │     node     │     │     node     │                │
│   └──────┬───────┘     └──────┬───────┘     └──────┬───────┘                │
│          │                    │                    │                         │
│          ▼                    ├────────┬───────────┘                         │
│   ┌──────────────┐            │        │                                     │
│   │   driver     │            ▼        ▼                                     │
│   │   monitor    │     ┌─────────┐ ┌─────────┐                               │
│   └──────┬───────┘     │ detect  │ │ stereo  │                               │
│          │             │  node   │ │  depth  │                               │
│          │             └────┬────┘ └────┬────┘                               │
│          │                  │           │                                    │
│          │             ┌────┴────┐      │                                    │
│          │             ▼         ▼      │                                    │
│          │      ┌─────────┐ ┌─────────┐ │                                    │
│          │      │classify │ │ lanedet │ │                                    │
│          │      │  node   │ │  node   │ │                                    │
│          │      └────┬────┘ └────┬────┘ │                                    │
│          │           │           │      │                                    │
│          │           └─────┬─────┘      │                                    │
│          │                 │            │                                    │
│          │                 ▼            │                                    │
│          │          ┌──────────┐        │                                    │
│          │          │   adas   │        │                                    │
│          │          │   node   │        │                                    │
│          │          └────┬─────┘        │                                    │
│          │               │              │                                    │
│          └───────────────┼──────────────┘                                    │
│                          ▼                                                   │
│                   ┌──────────────┐     ┌──────────────┐                      │
│                   │     GUI      │     │  Dashboard   │                      │
│                   │  (Display)   │     │    (Web)     │                      │
│                   └──────────────┘     └──────────────┘                      │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## System Requirements

| Component | Requirement |
|-----------|-------------|
| **Hardware** | NVIDIA Jetson Orin Nano/NX/AGX |
| **OS** | Ubuntu 22.04 (JetPack 6.2) |
| **ROS2** | Humble Hawksbill |
| **CUDA** | 12.6+ |
| **TensorRT** | 10.x |
| **OpenCV** | 4.x with CUDA support |

---

## Nodes

### 1. Camera Node (`camera`)

Captures video from mono cameras (CSI or USB) for driver monitoring.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resource` | string | `csi://0` | Camera source URI |
| `width` | int | 1280 | Frame width |
| `height` | int | 720 | Frame height |

**Topics Published:**
- `/camera/raw` (`sensor_msgs/Image`) - Raw camera frames

**Supported Sources:**
- CSI Camera: `csi://0`
- USB Camera: `v4l2:///dev/video0`
- Video File: `file:///path/to/video.mp4`

---

### 2. Stereo Camera Node (`camera_stereo`)

Handles Arducam stereo camera with synchronized left/right image capture and CUDA-accelerated rotation.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resource` | string | `/dev/video1` | V4L2 device path |
| `width` | int | 3840 | Full stereo width (1920×2) |
| `height` | int | 1200 | Stereo height |
| `framerate` | int | 30 | Capture framerate |
| `rotated_lenses` | bool | false | Apply 90° rotation to each eye |
| `cuda_flip` | string | `rotate-180` | CUDA flip mode: `rotate-180`, `vertical-flip`, `horizontal-flip`, or empty for none |

**Topics Published:**
- `/camera_stereo/left/image_raw` (`sensor_msgs/Image`, `rgb8`) - Left camera. `rotated_lenses=false` → 1440×900 (default), `rotated_lenses=true` → 1200×1200.
- `/camera_stereo/right/image_raw` (`sensor_msgs/Image`, `rgb8`) - Right camera, same dims.
- `/camera_stereo/left/camera_info`, `/camera_stereo/right/camera_info` (`sensor_msgs/CameraInfo`) - Intrinsics + baseline, populated from `config/stereo_calib.yaml`.

**CUDA Kernels** (`src/cuda/stereo_rotate.cu`):
- `rotated_lenses=false`: inner-edge crop (1920→1440 per eye, 480 px removed from each eye's outer side) + symmetric vertical crop (1200→900) + optional flip (`cuda_flip=rotate-180` for upside-down mounts).
- `rotated_lenses=true` (legacy): 90° per-eye rotation, 1200×1200 square output.

---

### 3. Stereo Depth Node (`stereo_depth`)

Computes dense depth maps from the rectified stereo pair using **LightStereo-S** ([Guo et al., ICRA 2025](https://arxiv.org/abs/2406.19833)), OpenStereo's efficient 2D-cost-aggregation network. **C++ rclcpp node** driving a TensorRT engine via direct `enqueueV3` on a high-priority CUDA stream. A reference Python implementation is kept at `scripts/stereo_depth_lightstereo.py` as a fallback / debug tool — point the launch file's `executable` at it if the C++ port misbehaves.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `engine_file_path` | string | `lightstereo_s_320x512.engine` | TRT engine filename (resolved relative to installed `graphs/stereo-depth/` dir) |
| `calibration_file` | string | (resolved from `camera_stereo`) | Path to `stereo_calib.yaml` for rectification + metric depth conversion |
| `depth_vmax` | float | 5.0 | Colormap upper bound in metres (close=hot/bright, far=black). Tune for the scene — 5 m for indoor, 20+ m for outdoor. Only affects `/stereo_depth/depth_color`; `/stereo_depth/depth` is unaffected. |
| `warmup_iters` | int | 3 | TRT warmup runs (eats first-inference setup cost out of the hot path) |
| `sync_slop_s` | float | 0.05 | Max L/R stamp gap (s) before the pair is dropped |
| `depth_color_publish_w` | int | 480 | Width (px) of `/stereo_depth/depth_color`; 0 = native eye width |

**Topics Subscribed:**
- `left/image_raw` (`sensor_msgs/Image`, `rgb8` or `bgr8`) — Left stereo image
- `right/image_raw` (`sensor_msgs/Image`, `rgb8` or `bgr8`) — Right stereo image
- (Manual latest-right cache; `onLeft` processes when L/R stamps are within `sync_slop_s` — `message_filters` is unreliable under BEST_EFFORT QoS for large images.)

**Topics Published:**
- `/stereo_depth/depth` (`sensor_msgs/Image`, `32FC1`) — Metric depth in metres at native eye resolution (1440×900)
- `/stereo_depth/depth_color` (`sensor_msgs/Image`, `bgr8`) — INFERNO colormap of depth, downsized to `depth_color_publish_w` for cheap visualization

**Pipeline (per frame):**
1. Two raw RGB uchar3 images get `cudaMemcpyAsync`'d to GPU device buffers (allocated once at init).
2. A single fused CUDA kernel per eye (`cudaStereoRectifyResizeNormCHW` in `src/cuda/stereo_rectify.cu`) consumes the raw image + a pre-baked `float2` remap (already at MODEL resolution) and writes directly into the TRT engine's input binding: rectified, resized to 320×512, ImageNet-normalized, HWC→CHW — in one bilinear lookup.
3. `enqueueV3` runs LightStereo on a high-priority CUDA stream.
4. Disparity `[1, 1, 320, 512]` lands in another device buffer, gets `cudaMemcpyAsync`'d back to host. CPU then upscales to eye resolution, scales by `EYE_W / model_w` to convert to source-pixel units, and computes `Z = fx_rect · baseline / disp`.

**Why C++ and not the Python script:** all other inference nodes in the graph are C++ (detect, classify, lanedet, driver_monitor), so the stereo backend matches the rest. The fused preprocessing kernel saves the two intermediate eye-sized float buffers (~15 MB) that the Python `cv2.remap → resize → norm` pipeline kept resident, and `rclcpp` doesn't pay the per-byte serialization cost that bit the Python depth message (rclpy validates every `uint8` in a depth frame via Python, ~750 ms for a 5 MB frame unless worked around with `array.array`).

**Model Specifications:**
- Input: 320×512 RGB stereo pair after rectification + resize; preprocessing is `x/255` then ImageNet `(x-mean)/std` (mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225]) — matches the OpenStereo training config exactly.
- Output: Dense disparity `[1, 1, 320, 512]` float32. Upscaled to source res then ×(EYE_W/model_w) to convert to source-pixel units; depth via `Z = fx_rect · baseline / disp`.
- Architecture: MobileNetV2 backbone + GWC correlation volume at /4 stride + 2D cost aggregation (no expensive 3D convolutions) + context upsample to source res. ~1 M params.
- Engine: built with `--fp16` from `LightStereo-S-SceneFlow.onnx` via OpenStereo's `deploy/trt_profile.sh`.
- Measured on Orin NX 16GB + `jetson_clocks`: **~16 ms / ~63 fps** trtexec GPU compute at 320×512.
- Engine generation: clone [OpenStereo](https://github.com/XiandaGuo/OpenStereo) to `~/OpenStereo`, drop the pretrained `.ckpt` under `output/SceneFlow/LightStereo_S/lightstereo_s_sceneflow/default/ckpt/`, then `python3 deploy/export.py --config cfgs/lightstereo/lightstereo_s_sceneflow.yaml --weights <ckpt> --imgsz 320 512 --device 0 --simplify --half --include onnx` followed by `bash deploy/trt_profile.sh --onnx <onnx> --saveEngine lightstereo_s_320x512.engine --fp16`, and copy the result to `src/graphs/stereo-depth/`.

> **Calibration dependency:** depth metric correctness depends entirely on `config/stereo_calib.yaml`. The node loads `K_left/D_left/R1/P1` (and right counterparts) and bakes the rectification maps at MODEL resolution once at init. See [Stereo Calibration Workflow](#stereo-calibration-workflow) below for how to regenerate the file when the camera is moved.

> **DDS / network requirement:** the two image_raw streams together push ~9 MB / 33 ms. Without the kernel UDP buffers (auto-installed by `install_all_deps.sh`) and the FastDDS LARGE_DATA mode (auto-set by `launch_visionsense.sh`), one eye drops to ~2 Hz. See [Network / DDS Configuration](#network--dds-configuration) below.

---

### 4. Object Detection Node (`detect`)

Real-time object detection using YOLOv8 with TensorRT and multi-object tracking.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | `detect.engine` | TensorRT engine path |
| `labels` | string | `labels_detect.txt` | Class labels file |
| `thresholds` | float[] | [0.40, 0.45, ...] | Per-class confidence thresholds |
| `track_frame_rate` | int | 30 | Tracking frame rate |
| `track_buffer` | int | 30 | Lost track buffer size |

**Detected Classes:**
| ID | Class | Threshold |
|----|-------|-----------|
| 0 | Pedestrian | 0.45 |
| 1 | Cyclist | 0.45 |
| 2 | Vehicle-Car | 0.60 |
| 3 | Vehicle-Bus | 0.45 |
| 4 | Vehicle-Truck | 0.45 |
| 5 | Train | 0.50 |
| 6 | Traffic Light | 0.40 |
| 7 | Traffic Sign | 0.55 |

**Topics Subscribed:**
- `/detect/image_in` (`sensor_msgs/Image`) - Input image

**Topics Published:**
- `/detect/detections` (`visionconnect/Detect`) - Detection results with tracking
- `/detect/signs` (`visionconnect/Signs`) - Cropped traffic signs for classification

**Tracking Features:**
- BYTE tracker with Kalman filter prediction
- Unique ID assignment per tracked object
- ID format: `{ClassName}_{ID}` (e.g., `Car_001`, `Pedestrian_003`)

---

### 5. Traffic Sign Classification Node (`classify`)

Classifies detected traffic signs and lights into 50+ categories.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | `classify.engine` | TensorRT engine path |
| `labels` | string | `labels_classify.txt` | Class labels file |
| `thresholds` | float[] | [0.30, 0.75] | Traffic light/sign thresholds |

**Supported Sign Categories:**
- **Traffic Lights:** Red, Yellow, Green
- **Regulatory Signs:** Stop, Yield, Speed Limits (15-70 mph), No Entry, No U-Turn, etc.
- **Warning Signs:** Curve Ahead, Intersection, School Zone, Road Work, etc.
- **Guide Signs:** Lane Markers, Merge, Highway Signs

**Topics Subscribed:**
- `/classify/signs_in` (`visionconnect/Signs`) - Cropped sign images

**Topics Published:**
- `/classify/signs` (`visionconnect/Signs`) - Classified signs with labels

---

### 6. Lane Detection Node (`lanedet`)

Detects and segments lane lines using neural network inference.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | `lane_detect.engine` | TensorRT engine path |

**Topics Subscribed:**
- `/lanedet/image_in` (`sensor_msgs/Image`) - Input image

**Topics Published:**
- `/lanedet/lanes` (`visionconnect/Lanes`) - Detected lane data
  - `xs`, `ys`: Lane point coordinates
  - `probs`: Lane confidence (4 lanes max)
  - `num_lanes`: Number of detected lanes
  - `laneimg`: Visualization overlay

**Output:**
- Up to 4 lane lines detected
- Polyline representation with confidence scores
- Segmentation mask overlay

---

### 7. Driver Monitoring Node (`driver_monitor`)

TensorRT-accelerated driver attention monitoring using face detection and gaze estimation.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `face_engine` | string | `yolov11n_face_fp16.engine` | Face detection model |
| `gaze_engine` | string | `resnet18_gaze_fp16.engine` | Gaze estimation model |
| `camera_topic` | string | `/camera/raw` | Input camera topic |
| `confidence` | float | 0.5 | Face detection threshold |

**Driver States:**
| State | Condition | Alert |
|-------|-----------|-------|
| `ALERT` | Face detected, gaze forward | No |
| `DISTRACTED` | Gaze >30° off-center for 2s | Yes |
| `DROWSY` | Eyes closed (future) | Yes |
| `NO_DRIVER` | No face detected for 1s | Yes |

**Topics Subscribed:**
- `/camera/raw` (`sensor_msgs/Image`) - Driver-facing camera

**Topics Published:**
- `/driver_monitor/image` (`sensor_msgs/Image`) - Annotated output with gaze arrow
- `/driver_monitor/state` (`std_msgs/String`) - Current driver state
- `/driver_monitor/alert` (`std_msgs/Bool`) - Alert flag

**Models:**
- **Face Detection:** YOLOv11-nano (640×640 input, 8400 detections)
- **Gaze Estimation:** ResNet18 (448×448 input, pitch/yaw angles)

---

### 8. ADAS Node (`adas`)

Advanced Driver Assistance System alerts based on lane and detection data.

**Topics Subscribed:**
- `/adas/lanes_in` (`visionconnect/Lanes`) - Lane detection data

**Topics Published:**
- `/adas/adas_alerts` (`visionconnect/ADAS`) - ADAS warnings

**Alerts:**
- Lane departure warning
- Forward collision warning (with depth data)

---

### 9. IMU/GPS Node (`imu_gps`)

Sensor fusion for IMU and GPS data (BNO055 + GPS module).

**Topics Published:**
- `/imu_gps/imu/data` (`sensor_msgs/Imu`) - IMU orientation and acceleration
- `/imu_gps/gps/fix` (`sensor_msgs/NavSatFix`) - GPS coordinates

---

### 10. GUI Node (`gui`)

Real-time data fusion display with multi-panel layout.

**Layout:**
```
┌────────────────────────────┬─────────────────┐
│                            │ Driver Monitor  │
│                            │   (1/3 × 1/3)   │
│       Main View            ├─────────────────┤
│    (2/3 × Full Height)     │  Stereo Depth   │
│                            │   (1/3 × 1/3)   │
│    Object Detection +      ├─────────────────┤
│    Lane Overlay +          │    Summary      │
│    Traffic Signs           │   (1/3 × 1/3)   │
│                            │  Speed/GPS/IMU  │
└────────────────────────────┴─────────────────┘
```

**Topics Subscribed:**
- `/gui/image_in` - Main camera feed
- `/gui/detect_in` - Detection results
- `/gui/signs_in` - Classified signs
- `/gui/lanes_in` - Lane detection
- `/gui/adas_in` - ADAS alerts
- `/driver_monitor/image` - Driver monitor feed
- `/stereo_depth/depth_color` - Colorized depth visualization
- `/imu_gps/imu/data` - IMU data
- `/imu_gps/gps/fix` - GPS coordinates

---

### 11. Dashboard Node (`dashboard`)

Web-based monitoring interface accessible via browser.

**Access:** `http://<jetson-ip>:8080`

**Features:**
- Live video stream
- Detection statistics
- System status

---

## Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/connected-wise/VisionSense.git
cd VisionSense
```

### Step 2: Install ROS2 and Project Dependencies

Install ROS2 Humble, jetson-inference, and all required libraries:

```bash
sudo bash install_all_deps.sh
```

This script installs:
- ROS2 Humble desktop and vision packages
- Build tools (cmake, colcon, etc.)
- jetson-inference library
- Python dependencies (numpy, pyserial)
- System libraries (Eigen3, V4L utilities, **yaml-cpp**)
- The system OpenCV (`libopencv-dev`, ~4.8 from JetPack) — no source build required
- **Arducam camera driver** + device-tree overlay (combined AR0234 stereo + IMX219 mono)
- **`nvargus-daemon` override** (`enableCamInfiniteTimeout=1`) to avoid CSI buffer wedge on disconnect
- **Kernel UDP buffers** (`/etc/sysctl.d/99-ros2-fastdds.conf`, 16 MB) — required for stereo @ 30 Hz
- **`visionsense-imx219.service`** systemd unit + passwordless sudoers rule. This unit owns the IMX219 driver-monitor camera for the entire system uptime (workaround for an Argus reopen bug on this rig — every second `nvarguscamerasrc` session fails until reboot otherwise). VisionSense subscribes to `/camera/raw` as a normal ROS topic.

> The CUDA preprocessing kernels in `src/cuda/preprocess.cu`, `src/cuda/stereo_rotate.cu`, and `src/cuda/stereo_rectify.cu` replace the OpenCV `cv::cuda::*` ops earlier branches relied on, so an OpenCV-with-CUDA source build is no longer needed.

### Step 3: TensorRT Engines

The `.engine` files checked into the repo were built on this device's
Jetson+TensorRT version. **They will not load on a different device until
rebuilt from their ONNX source.**

The active stereo backend (LightStereo-S) uses
`src/graphs/stereo-depth/lightstereo_s_320x512.engine`. To rebuild it on a
fresh Jetson, see "Engine generation" in the [Stereo Depth Node](#3-stereo-depth-node-stereo_depth_lightstereopy)
section above.

Detection/classification/face/gaze engines under `src/graphs/object-detection`,
`src/graphs/classifier`, `src/graphs/driver-monitor`, etc. each have their own
ONNX source. The legacy FFS regeneration recipe is preserved in
`archive/scripts/regenerate_engines.sh` if you ever want it back.

### Step 4: Build VisionSense

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select visionconnect
```

## Network / DDS Configuration

The two stereo `image_raw` streams together push ~9 MB every 33 ms. Without proper DDS transport configuration, FastDDS' default 512 KB SHM segment can't hold both eyes' frames simultaneously and one of `/camera_stereo/{left,right}/image_raw` drops to ~2 Hz while the other holds 30 Hz — producing badly-skewed `TimeSynchronizer` callbacks downstream.

VisionSense uses **two different FastDDS configurations** depending on which launch is run:

| Launch | DDS config | Why |
|---|---|---|
| `visionsense.launch.py` (full pipeline, 12+ nodes — what the desktop icon runs) | `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` env var | UDP for discovery + many small messages (detections, lanes, signs), auto-allocated SHM only for big payloads. Set in `launch_visionsense.sh`. |
| `test_stereo_depth.launch.py` (3-node stereo isolation test) | `fastdds_profile.shm.xml` (64 MB SHM segment, no UDP) | Pure-SHM zero-copy is optimal when the only meaningful traffic is two big image streams. Set in `scripts/launch_visionsense.sh` for that workflow. |

**Don't mix them:** loading the SHM-only XML profile for the full pipeline funnels every small detection/lane message through the same 64 MB segment + 32-deep port queue and starves the GUI. Loading LARGE_DATA for the 3-node test is fine but unnecessary.

### Kernel UDP buffers (auto-installed)

`install_all_deps.sh` step 15 writes `/etc/sysctl.d/99-ros2-fastdds.conf`:

```
net.core.rmem_max = 16777216
net.core.rmem_default = 16777216
net.core.wmem_max = 16777216
net.core.wmem_default = 16777216
```

The kernel default of ~208 KB overflows immediately on a 4.32 MB stereo image (~67 UDP fragments) — required for LARGE_DATA's UDP path and any UDP fallback.

### Diagnosing asymmetric stereo Hz

If one eye lags behind the other, in this order:

1. **Confirm the env var actually reached `camera_stereo`** (multi-line `bash -c 'export ...'` invocations silently break the export):
   ```bash
   cat /proc/$(pgrep -f camera_stereo)/environ | tr '\0' '\n' | grep -E 'FASTDDS|FASTRTPS'
   ```
2. **Check the kernel buffers**: `sysctl net.core.rmem_max` should be `16777216`.
3. **Sweep zombie SHM segments** that pile up in `/dev/shm` after crashed/SIGINT'd runs:
   ```bash
   fastdds shm clean && rm -f /dev/shm/fastrtps_*
   ```

## Stereo Calibration Workflow

`config/stereo_calib.yaml` is consumed by both `camera_stereo` (populates `CameraInfo`) and `stereo_depth` (bakes the rectification maps). Regenerate it any time the stereo bar is bumped, the cameras are re-seated, or the per-eye crop dimensions change.

### Step 1 — Capture pairs

The capture script reads `/dev/video1` directly (bypassing ROS) and saves byte-identical L/R PNGs at the same per-eye geometry that `camera_stereo` publishes (1440×900 default, inner-edge crop, rotate-180 for the upside-down mount).

```bash
# Stop VisionSense first — only one process can hold /dev/video1.
sudo systemctl stop visionsense-imx219.service   # IMX219 mono still runs from here
# Then either fully stop VisionSense or just kill camera_stereo before capture.

cd scripts
python3 capture_stereo_calib.py
```

Hold the chessboard in varied poses — **the diversity of the captures is what determines whether OpenCV can fit the geometry**:
- 30–50 pairs is plenty when they're well-distributed; 100+ near-identical pairs is worse than 30 varied ones.
- Vary depth (some at ~50 cm, some at ~100 cm, some at ~150 cm).
- Vary pose (pitch, yaw, roll the board between captures).
- Centre the board across the image — close to each corner, not just the middle.
- Don't hold the board so close that disparity pushes it to opposite image edges in L vs R.

Default board: **7×5 inner corners, 30 mm squares** (= 8×6 squares). Override with `--cols/--rows/--square-mm` if you switch boards.

SPACE saves a pair only when both eyes detect the full board. Q/ESC exits. Re-runs continue from the last saved index.

### Step 2 — Compute calibration

```bash
python3 compute_stereo_calib.py \
    --dir ./stereo_calib_images \
    --out ../config/stereo_calib.yaml \
    --fix-aspect
```

The script does per-eye intrinsic calibration, iteratively drops pairs whose per-eye reprojection error exceeds `--reject-threshold` (default 1.5 px), then runs `cv2.stereoCalibrate` + `cv2.stereoRectify`. It writes the YAML plus a `stereo_calib_rectified_sample.png` for sanity-checking that epipolar lines line up.

Good output:
- `left RMS < 0.5 px`, `right RMS < 0.5 px`
- `stereo RMS < 1.0 px`
- `||T|| ≈ 100 mm` (matches Arducam baseline)
- `|Tx| >> |Ty|, |Tz|` (lenses on the same horizontal line, coplanar in Z)

### Step 3 — Sanity-check rectification

Render a few rectified pairs from the captures so you can visually confirm corresponding scene points sit on the same image row in both eyes:

```bash
# From repo root
python3 -c "
import cv2, yaml, numpy as np, os
c=yaml.safe_load(open('config/stereo_calib.yaml'))
m=lambda k: np.asarray(c[k]['data']).reshape(int(c[k]['rows']), int(c[k]['cols']))
K1,K2=m('K_left'),m('K_right'); D1,D2=np.asarray(c['D_left']['data']),np.asarray(c['D_right']['data'])
R1,R2,P1,P2=m('R1'),m('R2'),m('P1'),m('P2'); W,H=int(c['image_width']),int(c['image_height'])
m1l,m2l=cv2.initUndistortRectifyMap(K1,D1,R1,P1,(W,H),cv2.CV_16SC2)
m1r,m2r=cv2.initUndistortRectifyMap(K2,D2,R2,P2,(W,H),cv2.CV_16SC2)
for i in (0,40,80): 
    l,r=cv2.imread(f'scripts/stereo_calib_images/left_{i:03d}.png'),cv2.imread(f'scripts/stereo_calib_images/right_{i:03d}.png')
    if l is None: continue
    lr,rr=cv2.remap(l,m1l,m2l,1),cv2.remap(r,m1r,m2r,1); pair=np.hstack([lr,rr])
    for y in range(0,H,40): cv2.line(pair,(0,y),(2*W,y),(0,255,0),1)
    cv2.imwrite(f'testing/rectified_samples/pair_rect_{i:03d}.png', pair)
"
```

### Common failure modes

| Symptom | Cause | Fix |
|---|---|---|
| Rectified sample is all black/white, `fx_rect > 100k` | Optimizer found bogus (R, T) — usually from low pose diversity | Recapture with varied depth and angle |
| `||T||` doesn't match physical baseline (~100 mm) | Wrong `--square-mm` value, or board held at one constant depth | Measure squares with a ruler; recapture at varied depths |
| `|Tz|` or `|Ty|` > 5 mm | Optimizer attributes residual error to phantom Z/Y baseline | Improve capture diversity, or post-process the YAML to force `Tz=Ty=0` (loses some metric accuracy but keeps a useful rectification) |
| `findChessboardCorners` fails on all pairs | Wrong pattern size | Count inner corners; pass `--cols/--rows` accordingly |

## Usage

### Desktop Launcher
Double-click the **VisionSense** icon on the desktop.

### Command Line
```bash
source /opt/ros/humble/setup.bash
cd ~/VisionSense && source install/setup.bash
ros2 launch visionconnect visionsense.launch.py
```

### Individual Nodes
```bash
ros2 run visionconnect camera
ros2 run visionconnect detect
ros2 run visionconnect gui
```

## Configuration

Edit `config/config.yaml`:

```yaml
sensors:
    uv_camera:     true    # Mono camera for driver monitoring
    zed_camera:    true    # Stereo camera
    gps_module:    true    # GPS/IMU module

camera:
    ros__parameters:
        resource:   "csi://0"
        width:      1280
        height:     720

camera_stereo:
    ros__parameters:
        resource:       "/dev/video1"
        width:          3840
        height:         1200
        rotated_lenses: false      # false → 1440×900 per eye; true → 1200×1200 (legacy)
        cuda_flip:      "rotate-180"  # for upside-down mount
        baseline_mm:    101.3
        calibration_file: "stereo_calib.yaml"

stereo:
    main_eye: "left"   # which eye downstream detect/lanedet/gui/dashboard consume

stereo_depth_lightstereo:
    ros__parameters:
        engine_file_path: "lightstereo_s_320x512.engine"
        depth_vmax:       5.0    # colormap upper bound (m); 5 for indoor, 20+ for outdoor

detect:
    ros__parameters:
        model:      "detect.engine"
        thresholds: [0.40, 0.45, 0.45, 0.6, 0.45, 0.45, 0.5, 0.40, 0.55]

driver_monitor:
    ros__parameters:
        face_engine: "/path/to/yolov11n_face_fp16.engine"
        gaze_engine: "/path/to/resnet18_gaze_fp16.engine"
        confidence:  0.5
```

## Neural Network Models

| Model | Purpose | Input Size | Format |
|-------|---------|------------|--------|
| `detect.engine` | Object Detection | 640×640 | TensorRT FP16 |
| `classify.engine` | Sign Classification | 224×224 | TensorRT FP16 |
| `lane_detect.engine` | Lane Detection | 800×288 | TensorRT FP16 |
| `lightstereo_s_320x512.engine` | Stereo Depth | 320×512 | TensorRT FP16, LightStereo-S (ICRA 2025) — 2D cost aggregation, ~16 ms compute |
| `yolov11n_face_fp16.engine` | Face Detection | 640×640 | TensorRT FP16 |
| `resnet18_gaze_fp16.engine` | Gaze Estimation | 448×448 | TensorRT FP16 |

## ROS2 Topics Overview

```
/camera/raw                    - Mono camera output
/camera_stereo/left/image_raw  - Left stereo image
/camera_stereo/right/image_raw - Right stereo image
/camera_stereo/{left,right}/camera_info - Stereo CameraInfo (intrinsics + baseline)
/stereo_depth/depth            - Metric depth (sensor_msgs/Image, 32FC1, meters)
/stereo_depth/depth_color      - Colorized depth visualization (bgr8, TURBO)
/detect/detections             - Object detections with tracking
/detect/signs                  - Detected traffic signs
/classify/signs                - Classified traffic signs
/lanedet/lanes                 - Lane detection results
/driver_monitor/image          - Driver monitoring visualization
/driver_monitor/state          - Driver state (ALERT/DISTRACTED/etc)
/adas/adas_alerts              - ADAS warnings
/imu_gps/imu/data              - IMU sensor data
/imu_gps/gps/fix               - GPS coordinates
/gui/fusion                    - Fused visualization output
```

## Troubleshooting

### Camera Issues
```bash
# List available cameras
v4l2-ctl --list-devices

# Test stereo camera
gst-launch-1.0 v4l2src device=/dev/video1 ! videoconvert ! autovideosink
```

### Build Errors
```bash
# Clean rebuild
rm -rf build install log
colcon build --packages-select visionconnect
```

### TensorRT Issues
Ensure models are built for your specific Jetson platform (engine files are not portable across Jetson + TensorRT versions).

For the stereo depth engine, rebuild `lightstereo_s_320x512.engine` from the LightStereo-S ONNX — see the [Stereo Depth Node](#3-stereo-depth-node-stereo_depth_lightstereopy) section for the OpenStereo export + trtexec recipe.

## License

VisionSense is licensed for **non-commercial research and educational use only**.

✅ **Allowed:** Research, education, testing, developing your own technologies
❌ **Not Allowed:** Commercial use, integration into products, offering as a service
💼 **Commercial License:** Contact licensing@connectedwise.com

See [LICENSE](LICENSE) for full terms.

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit changes (`git commit -m 'feat: add feature'`)
4. Push to branch (`git push origin feature/my-feature`)
5. Open a Pull Request

---

<p align="center">
  <b>VisionSense</b> - Autonomous Vehicle Vision System<br>
  © 2025 ConnectedWise
</p>
