#!/usr/bin/env python3
"""
Standalone live stereo-depth test using Fast-FoundationStereo (FFS).

Sibling of test_stereo_depth.py (LightStereo) and test_stereo_depth_sgbm.py
(classical SGBM). Same camera pipeline, same rectification, same viewer —
only the inference engine differs. Use to evaluate FFS depth quality
against the cleaned-up calibration and compare to LightStereo/SGBM.

Engine details (from archive/src/graphs/stereo-depth/fast_foundationstereo.yaml):
    - Input shape: (1, 3, 320, 320)            ← SQUARE, not 320×512
    - max_disp:    64                            (was 192 for LightStereo)
    - valid_iters: 4 GRU iterations
    - normalize:   false  (yaml claim; empirically also true)

Pipeline (matches node_camera_stereo.cpp exactly):
    1. V4L2 capture: 3840×1200 UYVY @ 30 fps from /dev/video1
    2. Per-eye inner-edge crop → 1440×900 + rotate-180
    3. cv2.remap rectification using config/stereo_calib.yaml
    4. Aspect-distorting resize to 320×320, /255, HWC→CHW
    5. TensorRT enqueue on FFS engine
    6. Resize disparity back to 1440×900, scale by eye_w/model_w,
       Z = fx_rect × baseline / disparity_eye

REQUIRED: stop the ROS camera_stereo node before running.

Usage:
    python3 scripts/test_stereo_depth_ffs.py
    python3 scripts/test_stereo_depth_ffs.py --depth-vmax 30
    python3 scripts/test_stereo_depth_ffs.py --engine /path/to/other_ffs.engine
    python3 scripts/test_stereo_depth_ffs.py --normalize        # try with ImageNet

Viewer keys (same as test_stereo_depth.py):
    hover     → distance overlay
    click     → log to stdout
    c         → cycle hero (rectified-L / disparity / depth-heatmap)
    [ / ]     → halve / grow depth_vmax
    a         → auto vmax (p90 of scene)
    s         → save snapshot to testing/
    q / ESC   → quit
"""

import argparse
import os
import sys
import time
from datetime import datetime

# DISPLAY/XAUTHORITY auto-set (mirror test_stereo_depth.py)
os.environ.setdefault("DISPLAY", ":0")
if not os.environ.get("XAUTHORITY"):
    for _xa in (f"{os.path.expanduser('~')}/.Xauthority",
                f"/run/user/{os.getuid()}/gdm/Xauthority",
                f"/run/user/{os.getuid()}/mutter/Xauthority"):
        if os.path.isfile(_xa):
            os.environ["XAUTHORITY"] = _xa
            break

import cv2
import numpy as np
import yaml

try:
    import tensorrt as trt
    import torch
except ImportError as e:
    print(f"missing dependency: {e}")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────── geometry
# Mirrors node_camera_stereo.cpp / src/cuda/stereo_rotate.cu.
RAW_W, RAW_H        = 3840, 1200
EYE_W, EYE_H        = 1440, 900
CROP_X_OUTER        = 480
CROP_Y              = 150
SENSOR_LEFT_START   = CROP_X_OUTER
SENSOR_RIGHT_START  = RAW_W - CROP_X_OUTER - EYE_W

# ImageNet defaults (only applied if --normalize is passed; the FFS yaml
# says normalize: false, but kept here so we can A/B-test empirically).
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)


# ─────────────────────────────────────────────────────────────────── TRT
class TRTLogger(trt.ILogger):
    def __init__(self): trt.ILogger.__init__(self)
    def log(self, severity, msg):
        if severity <= trt.ILogger.WARNING:
            print(f"[TRT] {msg}")


class FFSEngine:
    """Wrapper for the FFS TRT engine. Two inputs (left/right) → one
    disparity output. IO names from the OpenStereo-FFS export:
        left_image, right_image → disparity
    """
    def __init__(self, engine_path):
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(TRTLogger()).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"deserialize failed: {engine_path}")
        self.context = self.engine.create_execution_context()

        self.left_in = self.right_in = self.disp_out = None
        for i in range(self.engine.num_io_tensors):
            n = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(n)
            ln = n.lower()
            if mode == trt.TensorIOMode.INPUT:
                if   "left"  in ln: self.left_in  = n
                elif "right" in ln: self.right_in = n
            else:
                self.disp_out = n
        if not (self.left_in and self.right_in and self.disp_out):
            raise RuntimeError("engine must have left/right inputs + disparity output")

        in_shape  = tuple(self.engine.get_tensor_shape(self.left_in))
        out_shape = tuple(self.engine.get_tensor_shape(self.disp_out))
        self.model_h, self.model_w = in_shape[2], in_shape[3]

        self.io = {
            self.left_in:  torch.empty(in_shape,  dtype=torch.float32, device="cuda"),
            self.right_in: torch.empty(in_shape,  dtype=torch.float32, device="cuda"),
            self.disp_out: torch.empty(out_shape, dtype=torch.float32, device="cuda"),
        }
        for n, t in self.io.items():
            self.context.set_tensor_address(n, t.data_ptr())
        self.stream = torch.cuda.Stream()

    def infer(self, l_chw_np, r_chw_np):
        l = torch.from_numpy(l_chw_np).cuda(non_blocking=True)
        r = torch.from_numpy(r_chw_np).cuda(non_blocking=True)
        self.io[self.left_in ].copy_(l, non_blocking=True)
        self.io[self.right_in].copy_(r, non_blocking=True)
        with torch.cuda.stream(self.stream):
            self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        d = self.io[self.disp_out].cpu().numpy()
        while d.ndim > 2: d = d[0]
        return d


# ─────────────────────────────────────────────────────────────────── calib
def load_calibration(path):
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
        "fx_rect":    float(P1[0, 0]),
        "baseline_m": -float(P2[0, 3]) / float(P1[0, 0]),
        "bf":         -float(P2[0, 3]),
    }


def build_rectify_maps(calib, eye_size):
    W, H = eye_size
    m1l, m2l = cv2.initUndistortRectifyMap(calib["K1"], calib["D1"], calib["R1"], calib["P1"], (W, H), cv2.CV_16SC2)
    m1r, m2r = cv2.initUndistortRectifyMap(calib["K2"], calib["D2"], calib["R2"], calib["P2"], (W, H), cv2.CV_16SC2)
    return (m1l, m2l), (m1r, m2r)


# ─────────────────────────────────────────────────────────────────── capture
def open_capture(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(
            f"could not open {device}. STOP VisionSense first — only one process "
            "can hold the V4L2 device. Run `pkill -INT -f camera_stereo`.")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"UYVY"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  RAW_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RAW_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FPS, 30)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (aw, ah) != (RAW_W, RAW_H):
        raise RuntimeError(f"requested {RAW_W}x{RAW_H} but got {aw}x{ah}")
    return cap


def split_and_process(frame, rotate_180):
    sensor_left  = frame[CROP_Y:CROP_Y + EYE_H,
                         SENSOR_LEFT_START :SENSOR_LEFT_START  + EYE_W]
    sensor_right = frame[CROP_Y:CROP_Y + EYE_H,
                         SENSOR_RIGHT_START:SENSOR_RIGHT_START + EYE_W]
    if rotate_180:
        sensor_left  = cv2.rotate(sensor_left,  cv2.ROTATE_180)
        sensor_right = cv2.rotate(sensor_right, cv2.ROTATE_180)
    return sensor_left, sensor_right


# ─────────────────────────────────────────────────────────────────── preprocess
def rgb_to_model_chw(rgb, h, w, normalize=False):
    """Resize → /255 → optional ImageNet norm → HWC→CHW.

    FFS yaml says normalize=false; default is /255 only. Pass normalize=True
    to A/B-test whether ImageNet (mean/std) actually helps. For this engine
    we expect it doesn't (mirrors the LightStereo finding).
    """
    if rgb.shape[:2] != (h, w):
        rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_LINEAR)
    arr = rgb.astype(np.float32) * (1.0 / 255.0)
    if normalize:
        arr = (arr - IMAGENET_MEAN) / IMAGENET_STD
    arr = arr.transpose(2, 0, 1)[None, ...]
    return np.ascontiguousarray(arr, dtype=np.float32)


# ─────────────────────────────────────────────────────────────────── viewer
# Matplotlib viewer — same design as test_stereo_depth.py.
import matplotlib
matplotlib.use("TkAgg" if "DISPLAY" in os.environ else "Agg")
import matplotlib.pyplot as plt


class LiveViewer:
    MODES = ["rectified-left", "disparity-color", "depth-heatmap"]

    def __init__(self, depth_vmax, win_title):
        self.depth_vmax = depth_vmax
        self.win = win_title
        self.mode = 2
        self.hover_xy = None
        self.depth_m = None
        self.disp_eye = None
        self.left_rect = None
        self._quit = False

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(13, 7))
        try:
            self.fig.canvas.manager.set_window_title(win_title)
        except Exception:
            pass
        self.im = self.ax.imshow(
            np.zeros((EYE_H, EYE_W, 3), dtype=np.uint8),
            interpolation="nearest")
        self.ax.set_axis_off()
        self.title = self.ax.set_title("waiting for frames…", fontsize=11)
        self.readout = self.ax.text(
            12, 36, "",
            color="yellow", fontsize=11, fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.4", fc="black", ec="none", alpha=0.7))
        self.vline = self.ax.axvline(0, color="yellow", lw=0.8, visible=False)
        self.hline = self.ax.axhline(0, color="yellow", lw=0.8, visible=False)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_move)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event",    self._on_key)
        self.fig.canvas.mpl_connect("close_event",        lambda _ev: setattr(self, "_quit", True))
        plt.tight_layout()
        plt.show(block=False)

    def _on_move(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None: return
        x, y = int(ev.xdata), int(ev.ydata)
        self.hover_xy = (x, y)
        self._refresh_readout()

    def _on_click(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None or self.depth_m is None: return
        x, y = int(ev.xdata), int(ev.ydata)
        z = self._depth_at(x, y)
        d = self.disp_eye[y, x] if (0 <= y < self.disp_eye.shape[0] and 0 <= x < self.disp_eye.shape[1]) else float("nan")
        if z is None:
            print(f"  [{x:4d},{y:4d}]  disp={d:.2f}px  no valid depth")
        else:
            print(f"  [{x:4d},{y:4d}]  disp={d:6.2f}px  depth={z*100:7.2f}cm ({z:.3f}m)")

    def _on_key(self, ev):
        if ev.key in ("q", "escape"):
            self._quit = True
        elif ev.key == "c":
            self.mode = (self.mode + 1) % len(self.MODES)
        elif ev.key in ("[", "down"):
            self.depth_vmax = max(0.5, self.depth_vmax * 0.75)
            print(f"  depth_vmax = {self.depth_vmax:.2f} m")
        elif ev.key in ("]", "up"):
            self.depth_vmax = min(200.0, self.depth_vmax * 1.33)
            print(f"  depth_vmax = {self.depth_vmax:.2f} m")
        elif ev.key == "a":
            if self.depth_m is not None:
                v = self.depth_m[np.isfinite(self.depth_m)]
                if v.size:
                    self.depth_vmax = float(np.quantile(v, 0.9))
                    print(f"  depth_vmax = {self.depth_vmax:.2f} m (auto p90)")
        elif ev.key == "s" and self.left_rect is not None:
            ts = datetime.now().strftime("%Y%m%d-%H%M%S")
            out_dir = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "testing"))
            os.makedirs(out_dir, exist_ok=True)
            out = os.path.join(out_dir, f"ffs_live_{ts}.png")
            mode = self.MODES[self.mode]
            if   mode == "rectified-left":  img = self.left_rect
            elif mode == "disparity-color": img = self._disp_color(self.disp_eye)
            else:                            img = self._depth_heat(self.depth_m, self.depth_vmax)
            cv2.imwrite(out, img)
            print(f"  saved {out}")

    def _depth_at(self, x, y):
        if self.depth_m is None: return None
        if not (0 <= y < self.depth_m.shape[0] and 0 <= x < self.depth_m.shape[1]):
            return None
        z = float(self.depth_m[y, x])
        return z if np.isfinite(z) else None

    def _refresh_readout(self):
        if self.hover_xy is None or self.disp_eye is None: return
        x, y = self.hover_xy
        z = self._depth_at(x, y)
        d = self.disp_eye[y, x] if (0 <= y < self.disp_eye.shape[0] and 0 <= x < self.disp_eye.shape[1]) else float("nan")
        label = (f"[{x},{y}]  disp={d:.1f}px   {z*100:.1f} cm ({z:.2f} m)"
                 if z is not None else f"[{x},{y}]  no depth")
        self.readout.set_text(label)
        self.vline.set_xdata([x, x]); self.vline.set_visible(True)
        self.hline.set_ydata([y, y]); self.hline.set_visible(True)

    @staticmethod
    def _disp_color(disp):
        valid = disp > 0.5
        norm = np.zeros_like(disp, dtype=np.uint8)
        if valid.any():
            dmax = max(1.0, float(disp[valid].max()))
            norm[valid] = np.clip(disp[valid] / dmax * 255, 0, 255).astype(np.uint8)
        out = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        out[~valid] = 0
        return out

    @staticmethod
    def _depth_heat(depth, vmax):
        valid = np.isfinite(depth)
        z = depth.copy()
        z[~valid] = vmax
        z = np.clip(z, 0.0, vmax)
        norm = ((1.0 - z / vmax) * 255).astype(np.uint8)
        out = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        out[~valid] = 0
        return out

    def update(self, left_rect_bgr, disp_eye, depth_m, fps_str):
        self.left_rect = left_rect_bgr
        self.disp_eye = disp_eye
        self.depth_m = depth_m
        mode = self.MODES[self.mode]
        if   mode == "rectified-left":  hero = cv2.cvtColor(left_rect_bgr, cv2.COLOR_BGR2RGB)
        elif mode == "disparity-color": hero = cv2.cvtColor(self._disp_color(disp_eye), cv2.COLOR_BGR2RGB)
        else:                            hero = cv2.cvtColor(self._depth_heat(depth_m, self.depth_vmax), cv2.COLOR_BGR2RGB)
        self.im.set_data(hero)
        self.title.set_text(f"FFS   |   mode: {mode}   |   vmax: {self.depth_vmax:.1f} m   |   {fps_str}   "
                            f"|   keys: c=cycle [/]=vmax  a=auto  s=save  q=quit")
        self._refresh_readout()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def should_quit(self):
        return self._quit


# ─────────────────────────────────────────────────────────────────── main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video1")
    ap.add_argument("--engine", default="../archive/src/graphs/stereo-depth/fast_foundationstereo.engine",
                    help="path to FFS .engine file (default: the one in archive/)")
    ap.add_argument("--calib",  default="../config/stereo_calib.yaml")
    ap.add_argument("--depth-vmax", type=float, default=50.0,
                    help="colormap upper bound (m). Adjustable live via []/up/down.")
    ap.add_argument("--no-rotate", action="store_true",
                    help="skip per-eye 180° rotation (only if camera mounted upright)")
    ap.add_argument("--normalize", action="store_true",
                    help="apply ImageNet mean/std after /255 (FFS yaml says normalize=false; "
                         "off by default — pass this only for A/B testing)")
    ap.add_argument("--warmup", type=int, default=3)
    args = ap.parse_args()

    sd = os.path.dirname(os.path.abspath(__file__))
    eng_path   = args.engine if os.path.isabs(args.engine) else os.path.normpath(os.path.join(sd, args.engine))
    calib_path = args.calib  if os.path.isabs(args.calib)  else os.path.normpath(os.path.join(sd, args.calib))
    for p, name in [(eng_path, "engine"), (calib_path, "calib")]:
        if not os.path.exists(p):
            print(f"ERROR: {name} not found: {p}"); sys.exit(1)

    print(f"device     : {args.device}")
    print(f"engine     : {eng_path}")
    print(f"calib      : {calib_path}")
    print(f"normalize  : {'ImageNet' if args.normalize else '/255 only'}")
    print(f"rotate-180 : {'no' if args.no_rotate else 'yes'}")

    calib = load_calibration(calib_path)
    if calib["image_size"] != (EYE_W, EYE_H):
        print(f"WARN: calib was made at {calib['image_size']}, but capture emits {EYE_W}×{EYE_H}")
    (m1l, m2l), (m1r, m2r) = build_rectify_maps(calib, (EYE_W, EYE_H))
    print(f"calib      : fx_rect={calib['fx_rect']:.2f}  baseline={calib['baseline_m']*1000:.2f} mm  bf={calib['bf']:.3f}")

    engine = FFSEngine(eng_path)
    print(f"engine     : input {engine.model_h}×{engine.model_w}  (SQUARE — aspect-distorting resize from 1440×900)")

    # Disparity-to-depth notes for FFS:
    #   The model outputs disparity in MODEL pixel units (0..max_disp).
    #   Even though the input is square (320×320) while the source is 1440×900,
    #   the disparity is a horizontal shift in input-image coords, so the
    #   scaling factor to convert to eye-pixel units is eye_w / model_w.
    print(f"            scale model→eye disp: {EYE_W/engine.model_w:.3f}")
    print(f"            min measurable depth ≈ bf/(max_disp×scale) "
          f"= {calib['bf'] / (64.0 * (EYE_W/engine.model_w)):.2f} m  (FFS max_disp=64)")
    print(f"            sky / very-far depth → disparity ≈ 0")

    cap = open_capture(args.device)
    print(f"cap        : opened {args.device} @ {RAW_W}×{RAW_H} UYVY 30 fps")

    # Warmup with zeros so first measured inference isn't slow.
    dummy = np.zeros((engine.model_h, engine.model_w, 3), dtype=np.uint8)
    z = rgb_to_model_chw(dummy, engine.model_h, engine.model_w, normalize=args.normalize)
    for _ in range(args.warmup):
        engine.infer(z, z)

    viewer = LiveViewer(args.depth_vmax, "FFS Stereo Depth (live) — hover for distance | c [ ] a s q")
    print("\n  hover  → distance overlay     |   click → log pixel/disp/depth")
    print("  c      → cycle hero view       |   [ / ] → vmax half / x1.33")
    print("  a      → auto vmax (p90)       |   s     → save snapshot")
    print("  q/ESC  → quit\n")

    fail = 0
    t_last_log = time.perf_counter()
    n_since_log = 0
    fps_str = "—"

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            fail += 1
            if fail > 30:
                print("capture failed >30 times, giving up"); break
            time.sleep(0.03); continue
        fail = 0

        left_bgr, right_bgr = split_and_process(frame, rotate_180=not args.no_rotate)
        left_rect  = cv2.remap(left_bgr,  m1l, m2l, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_bgr, m1r, m2r, cv2.INTER_LINEAR)

        l_in = rgb_to_model_chw(cv2.cvtColor(left_rect,  cv2.COLOR_BGR2RGB),
                                engine.model_h, engine.model_w, normalize=args.normalize)
        r_in = rgb_to_model_chw(cv2.cvtColor(right_rect, cv2.COLOR_BGR2RGB),
                                engine.model_h, engine.model_w, normalize=args.normalize)

        t_inf = time.perf_counter()
        disp_model = engine.infer(l_in, r_in)
        infer_ms = (time.perf_counter() - t_inf) * 1000

        disp_eye = cv2.resize(disp_model, (EYE_W, EYE_H), interpolation=cv2.INTER_LINEAR)
        disp_eye *= (EYE_W / engine.model_w)
        depth_m = np.where(disp_eye > 0.5, calib["bf"] / disp_eye, np.float32(np.nan)).astype(np.float32)

        n_since_log += 1
        if time.perf_counter() - t_last_log >= 1.0:
            fps = n_since_log / (time.perf_counter() - t_last_log)
            fps_str = f"{fps:.1f} fps (infer {infer_ms:.1f} ms)"
            t_last_log = time.perf_counter(); n_since_log = 0

        viewer.update(left_rect, disp_eye, depth_m, fps_str)
        if viewer.should_quit(): break

    cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
