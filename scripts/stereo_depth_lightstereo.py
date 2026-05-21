#!/usr/bin/env python3
"""
stereo_depth_lightstereo.py
===========================

ROS2 node for the LightStereo-S stereo-depth backend
(https://arxiv.org/abs/2406.19833, OpenStereo authors, ICRA 2025).

Same publish contract as the previous FFS / MonSter backends:
  in:   /camera_stereo/{left,right}/image_raw      (sensor_msgs/Image, bgr8 or rgb8)
  out:  /stereo_depth/depth        (32FC1 metric meters, native eye resolution)
        /stereo_depth/depth_color  (bgr8 INFERNO colormap, configurable smaller width)

Per-frame pipeline (mirrors test_visionsense_msnet2d.py — the standalone
OpenStereo benchmark — exactly, so depth numbers match the live tool):
  1. left + right images (bgr8 or rgb8, 1440×900) ← /camera_stereo/{l,r}/image_raw
  2. cv2.remap with calibration (K/D/R/P from stereo_calib.yaml)
  3. resize to engine input (e.g. 320×512); RGB; /255 then ImageNet normalize
     (MEAN=[0.485,0.456,0.406], STD=[0.229,0.224,0.225]). The training cfg
     (cfgs/lightstereo/lightstereo_s_sceneflow.yaml) does this exact transform.
  4. TRT engine inference (torch CUDA tensors as IO buffers — no pycuda)
  5. upscale disparity to source res, ×(EYE_W/model_w) to get source-pixel units
  6. depth = fx_rect × baseline / disparity   (NaN where disp invalid)
  7. publish 32FC1 depth + downsized BGR8 INFERNO colormap

Pair sync: manual latest-right cache + process on left. message_filters'
ApproximateTimeSynchronizer silently drops pairs under BEST_EFFORT QoS in
rclpy, so we cache by hand.
"""

import array
import os
import sys
import time
import threading

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

import tensorrt as trt
import torch


# camera_stereo publishes BEST_EFFORT; match that so the SHM transport doesn't
# back-pressure the publisher when this node falls behind.
INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)
OUTPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

# LightStereo-S was trained on ImageNet-normalised inputs. Skipping these
# collapses depth to a single near-uniform value.
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)


def load_calibration(path: str) -> dict:
    with open(path) as f:
        c = yaml.safe_load(f)
    def mat(key):
        n = c[key]
        return np.asarray(n["data"], dtype=np.float64).reshape(int(n["rows"]), int(n["cols"]))
    P1 = mat("P1"); P2 = mat("P2")
    return {
        "K1": mat("K_left"),  "D1": np.asarray(c["D_left"]["data"],  dtype=np.float64),
        "K2": mat("K_right"), "D2": np.asarray(c["D_right"]["data"], dtype=np.float64),
        "R1": mat("R1"), "R2": mat("R2"),
        "P1": P1, "P2": P2,
        "image_size": (int(c["image_width"]), int(c["image_height"])),
        "fx_rect": float(P1[0, 0]),
        "baseline_m": -float(P2[0, 3]) / float(P1[0, 0]),
    }


class TrtStereo:
    """
    Loader for OpenStereo-exported TRT engines. Auto-discovers IO names and
    handles both (1,H,W) and (1,1,H,W) disparity output shapes.
    """
    def __init__(self, engine_path: str):
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(trt.Logger(trt.Logger.WARNING)).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"failed to load TRT engine: {engine_path}")
        self.context = self.engine.create_execution_context()
        # High-priority CUDA stream so this engine preempts camera_stereo's
        # crop+flip kernel and the other TRT models (detect/lanedet/classify/
        # driver_monitor) when they share the GPU in the full stack.
        self.stream = torch.cuda.Stream(priority=-1)

        self.io = {}
        self.input_names = []
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype_np = trt.nptype(self.engine.get_tensor_dtype(name))
            torch_dtype = {np.float32: torch.float32, np.float16: torch.float16}[dtype_np]
            tensor = torch.empty(shape, dtype=torch_dtype, device="cuda")
            is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            self.io[name] = {"tensor": tensor, "shape": shape}
            (self.input_names if is_input else self.output_names).append(name)
            self.context.set_tensor_address(name, tensor.data_ptr())

        if len(self.input_names) != 2 or len(self.output_names) != 1:
            raise RuntimeError(f"expected 2 inputs + 1 output, got {self.input_names} / {self.output_names}")
        self.left_in  = next(n for n in self.input_names if "left"  in n.lower())
        self.right_in = next(n for n in self.input_names if "right" in n.lower())
        self.disp_out = self.output_names[0]

        shp = self.io[self.left_in]["shape"]
        self.model_h, self.model_w = shp[2], shp[3]

    def infer(self, left_chw_f32_cuda: torch.Tensor, right_chw_f32_cuda: torch.Tensor) -> torch.Tensor:
        self.io[self.left_in ]["tensor"].copy_(left_chw_f32_cuda,  non_blocking=True)
        self.io[self.right_in]["tensor"].copy_(right_chw_f32_cuda, non_blocking=True)
        with torch.cuda.stream(self.stream):
            self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        disp = self.io[self.disp_out]["tensor"]
        while disp.ndim > 2:                   # (1,H,W) or (1,1,H,W) → (H,W)
            disp = disp[0]
        return disp


def msg_to_rgb(msg: Image) -> np.ndarray:
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "rgb8":
        return buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
    if msg.encoding == "bgr8":
        bgr = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    raise RuntimeError(f"unsupported encoding: {msg.encoding}")


def rgb_to_model_input(rgb: np.ndarray, target_h: int, target_w: int) -> torch.Tensor:
    """RGB HxWx3 uint8 → torch cuda (1,3,target_h,target_w) float32 with ImageNet norm."""
    if rgb.shape[:2] != (target_h, target_w):
        rgb = cv2.resize(rgb, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    arr = rgb.astype(np.float32) / 255.0
    arr = (arr - IMAGENET_MEAN) / IMAGENET_STD
    arr = arr.transpose(2, 0, 1)[None, ...]
    return torch.from_numpy(np.ascontiguousarray(arr)).cuda()


class StereoDepthLightStereo(Node):
    def __init__(self):
        super().__init__("stereo_depth_lightstereo")
        self.declare_parameter("engine_file_path",       "")
        self.declare_parameter("calibration_file",       "")
        self.declare_parameter("depth_vmax",             20.0)
        self.declare_parameter("warmup_iters",           3)
        self.declare_parameter("sync_slop_s",            0.05)
        # depth_color_publish_w: width (px) of /stereo_depth/depth_color.
        # 0 = native eye width. 480 matches what FFS used. Smaller = less
        # CPU in applyColorMap, less DDS traffic, less work in gui's
        # depth_callback.
        self.declare_parameter("depth_color_publish_w",  480)

        engine_path = self.get_parameter("engine_file_path").value
        calib_path  = self.get_parameter("calibration_file").value
        self.depth_vmax = float(self.get_parameter("depth_vmax").value)
        warmup = int(self.get_parameter("warmup_iters").value)
        self._slop_ns = int(float(self.get_parameter("sync_slop_s").value) * 1e9)
        self._color_w = int(self.get_parameter("depth_color_publish_w").value)

        if not engine_path or not os.path.exists(engine_path):
            raise RuntimeError(f"engine_file_path missing: {engine_path!r}")
        if not calib_path or not os.path.exists(calib_path):
            raise RuntimeError(f"calibration_file missing: {calib_path!r}")

        self.calib = load_calibration(calib_path)
        self.eye_w, self.eye_h = self.calib["image_size"]
        self.bf = self.calib["fx_rect"] * self.calib["baseline_m"]
        self.get_logger().info(
            f"calibration: image_size={self.calib['image_size']} "
            f"fx_rect={self.calib['fx_rect']:.2f} baseline={self.calib['baseline_m']*1000:.2f}mm"
        )

        self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(
            self.calib["K1"], self.calib["D1"], self.calib["R1"], self.calib["P1"],
            (self.eye_w, self.eye_h), cv2.CV_16SC2)
        self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(
            self.calib["K2"], self.calib["D2"], self.calib["R2"], self.calib["P2"],
            (self.eye_w, self.eye_h), cv2.CV_16SC2)

        self.trt_model = TrtStereo(engine_path)
        self.get_logger().info(f"engine ok: model input {self.trt_model.model_h}×{self.trt_model.model_w}")

        if warmup > 0:
            dummy = torch.zeros((1, 3, self.trt_model.model_h, self.trt_model.model_w),
                                dtype=torch.float32, device="cuda")
            for _ in range(warmup):
                _ = self.trt_model.infer(dummy, dummy)
            torch.cuda.synchronize()

        self.depth_pub = self.create_publisher(Image, "depth",       OUTPUT_QOS)
        self.color_pub = self.create_publisher(Image, "depth_color", OUTPUT_QOS)

        # Pre-allocate output message templates.
        self._depth_msg = Image()
        self._depth_msg.encoding = "32FC1"
        self._depth_msg.height = self.eye_h
        self._depth_msg.width  = self.eye_w
        self._depth_msg.step   = self.eye_w * 4
        self._depth_msg.header.frame_id = "stereo_left"

        if self._color_w <= 0 or self._color_w >= self.eye_w:
            self._color_pub_w, self._color_pub_h = self.eye_w, self.eye_h
        else:
            self._color_pub_w = self._color_w
            self._color_pub_h = int(round(self.eye_h * self._color_pub_w / self.eye_w))
        self.get_logger().info(
            f"color publish size: {self._color_pub_w}x{self._color_pub_h} (eye={self.eye_w}x{self.eye_h})"
        )

        self._color_msg = Image()
        self._color_msg.encoding = "bgr8"
        self._color_msg.height = self._color_pub_h
        self._color_msg.width  = self._color_pub_w
        self._color_msg.step   = self._color_pub_w * 3
        self._color_msg.header.frame_id = "stereo_left"

        self._latest_right: Image | None = None
        self._latest_right_ns: int = 0
        self._right_lock = threading.Lock()

        self.right_sub = self.create_subscription(
            Image, "right/image_raw", self._on_right, INPUT_QOS)
        self.left_sub = self.create_subscription(
            Image, "left/image_raw",  self._on_left,  INPUT_QOS)

        self._n = 0
        self._t_total = 0.0
        self.get_logger().info("ready")

    def _on_right(self, msg: Image):
        with self._right_lock:
            self._latest_right = msg
            self._latest_right_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _on_left(self, left_msg: Image):
        with self._right_lock:
            right_msg = self._latest_right
            right_ns = self._latest_right_ns
        if right_msg is None:
            return
        left_ns = left_msg.header.stamp.sec * 1_000_000_000 + left_msg.header.stamp.nanosec
        if abs(left_ns - right_ns) > self._slop_ns:
            return

        t0 = time.monotonic()

        left_rgb  = msg_to_rgb(left_msg)
        right_rgb = msg_to_rgb(right_msg)

        left_rect  = cv2.remap(left_rgb,  self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_rgb, self.map1_r, self.map2_r, cv2.INTER_LINEAR)

        l_in = rgb_to_model_input(left_rect,  self.trt_model.model_h, self.trt_model.model_w)
        r_in = rgb_to_model_input(right_rect, self.trt_model.model_h, self.trt_model.model_w)

        disp_cuda = self.trt_model.infer(l_in, r_in)
        disp_model = disp_cuda.detach().cpu().numpy()

        disp_src = cv2.resize(disp_model, (self.eye_w, self.eye_h), interpolation=cv2.INTER_LINEAR)
        disp_src *= (self.eye_w / self.trt_model.model_w)
        depth_m = np.where(disp_src > 0.5, self.bf / disp_src, np.float32(np.nan)).astype(np.float32, copy=False)

        # CRITICAL: Image.data as array.array('B', ...) — assigning bytes
        # triggers rclpy's per-uint8 validation in Python, which costs ~750 ms
        # for a 5 MB depth frame (microbenched). array.array does the bulk
        # conversion at C level in ~1 ms.
        self._depth_msg.header.stamp = left_msg.header.stamp
        self._depth_msg.data = array.array('B', depth_m.tobytes())
        self.depth_pub.publish(self._depth_msg)

        # Downsample BEFORE colormap so applyColorMap operates on the small
        # buffer (9× less work for 480-wide on a 1440-wide eye).
        if (self._color_pub_w, self._color_pub_h) != (self.eye_w, self.eye_h):
            disp_small = cv2.resize(disp_src, (self._color_pub_w, self._color_pub_h),
                                    interpolation=cv2.INTER_NEAREST)
        else:
            disp_small = disp_src
        z = np.where(disp_small > 0.5, self.bf / disp_small, np.float32(self.depth_vmax))
        z = np.clip(z, 0.0, self.depth_vmax)
        norm = ((1.0 - z / self.depth_vmax) * 255).astype(np.uint8)
        cm = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        cm[disp_small <= 0.5] = 0
        self._color_msg.header.stamp = left_msg.header.stamp
        self._color_msg.data = array.array('B', cm.tobytes())
        self.color_pub.publish(self._color_msg)

        dt = time.monotonic() - t0
        self._t_total += dt
        self._n += 1
        if self._n % 30 == 0:
            avg_ms = (self._t_total / self._n) * 1000.0
            self.get_logger().info(
                f"[profile] avg={avg_ms:.1f}ms ({1000.0/avg_ms:.2f}fps)  "
                f"d=[{float(np.nanmin(disp_src)):.1f},{float(np.nanmax(disp_src)):.1f}]px  "
                f"Z=[{float(np.nanmin(depth_m)):.2f},{float(np.nanmax(depth_m)):.2f}]m"
            )


def main():
    rclpy.init()
    try:
        node = StereoDepthLightStereo()
    except Exception as e:
        print(f"[stereo_depth_lightstereo] FATAL: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
