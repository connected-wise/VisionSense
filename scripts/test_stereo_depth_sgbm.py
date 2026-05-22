#!/usr/bin/env python3
"""
Standalone live stereo-depth test using OpenCV SGBM (classical, no neural net).

Drop-in sibling of test_stereo_depth.py with the SAME camera pipeline, SAME
rectification, SAME viewer — but replaces the LightStereo TRT engine with
OpenCV's SemiGlobalBlockMatching algorithm. Use it to:

  - Cross-check LightStereo's output against a classical algorithm.
  - Troubleshoot whether depth issues are model artifacts or pipeline issues.
  - Get ground-truth-ish depth (SGBM is slower but very predictable).

Pipeline (matches node_camera_stereo.cpp + node_stereo_depth.cpp exactly):
    1. V4L2 capture: 3840×1200 UYVY @ 30 fps from /dev/video1
    2. Per-eye inner-edge crop → 1440×900 + rotate-180
    3. cv2.remap rectification using config/stereo_calib.yaml
    4. cv2.StereoSGBM_create → compute → /16 to get float disparity
    5. Z = fx_rect × baseline / disparity_eye

Performance: SGBM at 1440×900 with num_disparities=192 + block=7 runs
~250-300 ms per pair on Orin NX → ~3-4 fps. Use --downsample 2 or 4 to
trade resolution for speed (4× faster at /2, 16× faster at /4).

REQUIRED: stop the ROS camera_stereo node before running.

Usage:
    python3 scripts/test_stereo_depth_sgbm.py
    python3 scripts/test_stereo_depth_sgbm.py --num-disp 256 --block 9
    python3 scripts/test_stereo_depth_sgbm.py --downsample 2     # 4× faster
    python3 scripts/test_stereo_depth_sgbm.py --no-rotate

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


# ─────────────────────────────────────────────────────────────────── geometry
# Mirrors node_camera_stereo.cpp / src/cuda/stereo_rotate.cu.
RAW_W, RAW_H        = 3840, 1200
EYE_W, EYE_H        = 1440, 900
CROP_X_OUTER        = 480
CROP_Y              = 150
SENSOR_LEFT_START   = CROP_X_OUTER
SENSOR_RIGHT_START  = RAW_W - CROP_X_OUTER - EYE_W


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


# ─────────────────────────────────────────────────────────────────── SGBM
def make_sgbm(num_disp, block, mode_str, strict, use_color=True):
    """Build a StereoSGBM matcher.

    Two presets:
      - relaxed (default): aggressive matching, ~50-70% coverage, some noise.
        Closer to what you'd want for a "look at the depth map" experience.
      - strict (--strict):  conservative matching, ~15-25% coverage, very
        few false matches. Closer to research-grade output.

    num_disp MUST be a multiple of 16. P1/P2 use the standard
    8*ch*block² / 32*ch*block² formula (ch=3 for color, 1 for gray).
    """
    mode_map = {
        "sgbm":   cv2.STEREO_SGBM_MODE_SGBM,
        "3way":   cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        "hh":     cv2.STEREO_SGBM_MODE_HH,
        "hh4":    getattr(cv2, "STEREO_SGBM_MODE_HH4", cv2.STEREO_SGBM_MODE_HH),
    }
    mode = mode_map.get(mode_str, cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    ch = 3 if use_color else 1
    if strict:
        kw = dict(disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=100, speckleRange=2)
    else:
        # Relaxed: fewer filters reject pixels, so coverage jumps.
        kw = dict(disp12MaxDiff=-1, uniquenessRatio=2, speckleWindowSize=0,  speckleRange=2)
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block,
        P1=8  * ch * block ** 2,
        P2=32 * ch * block ** 2,
        preFilterCap=63,
        mode=mode,
        **kw,
    )


def fill_invalid(disp_eye, left_rgb):
    """Inpaint the invalid (disp <= 0) pixels using OpenCV's TELEA inpainting,
    guided by the left rectified image. Just for visualization — replaces
    holes with smoothed neighbour disparity so the heatmap doesn't look
    speckled with black."""
    valid = disp_eye > 0.5
    if valid.all() or not valid.any():
        return disp_eye.copy()
    # Normalize disparity to uint8 for inpainting
    dmax = max(1.0, float(disp_eye[valid].max()))
    d8 = np.clip(disp_eye / dmax * 255, 0, 255).astype(np.uint8)
    mask = (~valid).astype(np.uint8) * 255
    filled = cv2.inpaint(d8, mask, 3, cv2.INPAINT_TELEA)
    return (filled.astype(np.float32) / 255.0 * dmax)


# ─────────────────────────────────────────────────────────────────── viewer
# matplotlib viewer (same design as test_stereo_depth.py — Tk backend so we
# don't depend on OpenCV 4.13's broken Qt setMouseCallback path).
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
        self.title = self.ax.set_title("waiting for frames…", fontsize=11, color="black")
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
            out = os.path.join(out_dir, f"sgbm_live_{ts}.png")
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
        self.title.set_text(f"SGBM   |   mode: {mode}   |   vmax: {self.depth_vmax:.1f} m   |   {fps_str}   "
                            f"|   keys: c=cycle [=]/up/down=vmax  a=auto  s=save  q=quit")
        self._refresh_readout()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def should_quit(self):
        return self._quit


# ─────────────────────────────────────────────────────────────────── main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video1")
    ap.add_argument("--calib",  default="../config/stereo_calib.yaml")
    ap.add_argument("--no-rotate", action="store_true",
                    help="skip per-eye 180° rotation (camera mounted upright)")
    ap.add_argument("--downsample", type=int, default=1, choices=[1, 2, 4],
                    help="downsample rectified pair before SGBM "
                         "(1=no downsample/slow, 2=4× faster, 4=16× faster). "
                         "Disparity is upscaled back to eye resolution; bf is "
                         "scaled accordingly so depths stay correct.")
    ap.add_argument("--num-disp", type=int, default=192,
                    help="SGBM number of disparities (multiple of 16). Larger "
                         "= can measure closer objects but slower. Default 192.")
    ap.add_argument("--block", type=int, default=7,
                    help="SGBM block size (odd, 3-15). Default 7.")
    ap.add_argument("--sgbm-mode", default="3way", choices=["sgbm", "3way", "hh", "hh4"],
                    help="SGBM internal mode. 3way is fastest at modest quality "
                         "loss, hh is most accurate but ~3× slower.")
    ap.add_argument("--depth-vmax", type=float, default=20.0,
                    help="colormap upper bound (m). Adjustable live via []/up/down.")
    ap.add_argument("--strict", action="store_true",
                    help="strict SGBM filters (~15-25%% coverage, near-zero "
                         "false matches). Default is relaxed (~50-70%% coverage).")
    ap.add_argument("--inpaint", action="store_true",
                    help="inpaint invalid (no-match) pixels for cleaner viz. "
                         "Inpainted values are GUESSES based on nearby valid "
                         "pixels — don't trust them for metric depth.")
    ap.add_argument("--gray", action="store_true",
                    help="run SGBM on grayscale (faster) instead of color")
    args = ap.parse_args()

    if args.num_disp % 16 != 0:
        print(f"--num-disp must be a multiple of 16 (got {args.num_disp})")
        sys.exit(1)
    if args.block % 2 == 0:
        print(f"--block must be odd (got {args.block})")
        sys.exit(1)

    sd = os.path.dirname(os.path.abspath(__file__))
    calib_path = args.calib if os.path.isabs(args.calib) else os.path.normpath(os.path.join(sd, args.calib))
    if not os.path.exists(calib_path):
        print(f"ERROR: calib not found: {calib_path}"); sys.exit(1)

    print(f"device     : {args.device}")
    print(f"calib      : {calib_path}")
    print(f"rotate     : {'no' if args.no_rotate else 'yes (cuda_flip=rotate-180 equivalent)'}")
    print(f"downsample : {args.downsample}x")
    print(f"SGBM       : num_disp={args.num_disp}  block={args.block}  mode={args.sgbm_mode}")

    calib = load_calibration(calib_path)
    if calib["image_size"] != (EYE_W, EYE_H):
        print(f"WARN: calib was made at {calib['image_size']}, but capture emits {EYE_W}×{EYE_H}")
    (m1l, m2l), (m1r, m2r) = build_rectify_maps(calib, (EYE_W, EYE_H))
    print(f"calib      : fx_rect={calib['fx_rect']:.2f}  baseline={calib['baseline_m']*1000:.2f} mm  bf={calib['bf']:.3f}")

    # SGBM. If --downsample, we run at lower res and scale the result back.
    sgbm = make_sgbm(args.num_disp, args.block, args.sgbm_mode,
                     strict=args.strict, use_color=not args.gray)
    ds = args.downsample
    print(f"SGBM       : preset={'strict' if args.strict else 'relaxed'}  "
          f"channels={'gray' if args.gray else 'color'}  "
          f"inpaint={'yes' if args.inpaint else 'no'}")
    sgbm_w, sgbm_h = EYE_W // ds, EYE_H // ds
    # bf is "fx_rect_eye × baseline_eye_pixels". After downsampling, the
    # effective fx (in the downsampled image) is fx/ds, so disparity in
    # downsampled pixels relates to depth by Z = (fx/ds) × b / disp_ds.
    # After we resize the disparity back to eye resolution and multiply by
    # ds, we recover eye-pixel disparity and the original bf still applies.

    cap = open_capture(args.device)
    print(f"cap        : opened {args.device} @ {RAW_W}×{RAW_H} UYVY 30 fps")

    viewer = LiveViewer(args.depth_vmax, "SGBM Stereo Depth (live) — hover for distance | c [ ] a s q")
    print("\n  hover      → distance overlay")
    print("  click      → log pixel/disp/depth to stdout")
    print("  c          → cycle hero view (rectified-L / disparity / depth-heatmap)")
    print("  [ / ]      → halve / grow depth_vmax")
    print("  a          → auto-tune vmax to scene's p90")
    print("  s          → save snapshot to testing/")
    print("  q / ESC    → quit\n")

    fail = 0
    t_log = time.perf_counter()
    n_since = 0
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

        if ds > 1:
            L_in = cv2.resize(left_rect,  (sgbm_w, sgbm_h), interpolation=cv2.INTER_AREA)
            R_in = cv2.resize(right_rect, (sgbm_w, sgbm_h), interpolation=cv2.INTER_AREA)
        else:
            L_in, R_in = left_rect, right_rect
        if args.gray:
            L_in = cv2.cvtColor(L_in, cv2.COLOR_BGR2GRAY)
            R_in = cv2.cvtColor(R_in, cv2.COLOR_BGR2GRAY)

        t0 = time.perf_counter()
        disp_int = sgbm.compute(L_in, R_in)
        sgbm_ms = (time.perf_counter() - t0) * 1000
        disp_sub = disp_int.astype(np.float32) / 16.0

        if ds > 1:
            disp_eye = cv2.resize(disp_sub, (EYE_W, EYE_H), interpolation=cv2.INTER_LINEAR) * ds
        else:
            disp_eye = disp_sub

        # Optional inpaint to fill no-match regions for cleaner viz.
        if args.inpaint:
            disp_eye = fill_invalid(disp_eye, left_rect)

        depth_m = np.where(disp_eye > 0.5, calib["bf"] / disp_eye, np.float32(np.nan)).astype(np.float32)

        n_since += 1
        if time.perf_counter() - t_log >= 1.0:
            fps = n_since / (time.perf_counter() - t_log)
            fps_str = f"{fps:.1f} fps (sgbm {sgbm_ms:.0f} ms)"
            t_log = time.perf_counter(); n_since = 0

        viewer.update(left_rect, disp_eye, depth_m, fps_str)
        if viewer.should_quit(): break

    cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
