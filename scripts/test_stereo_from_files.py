#!/usr/bin/env python3
"""
Standalone stereo-depth test using pre-rectified L/R image files.

This script is the static-input twin of test_stereo_depth.py — same engine,
same preprocessing, same depth math — but instead of capturing live from
/dev/video1 it reads any pre-rectified pair (e.g. the KITTI samples
checked into testing/) and runs them through the production LightStereo
TRT engine.

Use it to:

  - Sanity-check the model on known-good pre-rectified pairs (KITTI
    is what the model was tuned for). If the model produces sensible
    depth here but garbage on the live rig, the problem is calibration
    or rectification, NOT the model.
  - Compare two scenes side-by-side at a glance.
  - Verify preprocessing matches the production C++ stereo_depth node.

Defaults to testing/000003-{left,right}.png. The KITTI calib (fx≈721,
baseline≈0.54 m, bf≈388) is used by default for depth math; pass
--calib /path/to/stereo_calib.yaml to use a different (e.g. your rig's)
calibration, but note that depths will only be meaningful if the
images were rectified with that same calib.

Usage:
    python3 scripts/test_stereo_from_files.py
    python3 scripts/test_stereo_from_files.py \\
        --left testing/000007-left.png --right testing/000007-right.png
    python3 scripts/test_stereo_from_files.py --batch --no-display
    python3 scripts/test_stereo_from_files.py --calib config/stereo_calib.yaml \\
        --left /some/rectified_left.png --right /some/rectified_right.png

Viewer keys (same as test_stereo_depth.py):
    hover → distance overlay     | click → log to stdout
    c     → cycle hero view      | s     → save snapshot
    q/ESC → quit
"""

import argparse
import os
import sys
import time
from datetime import datetime

# DISPLAY/XAUTHORITY auto-set: see test_stereo_depth.py for the rationale.
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

try:
    import tensorrt as trt
    import torch
except ImportError as e:
    print(f"missing dependency: {e}")
    sys.exit(1)


# KITTI gray camera defaults (matches the sample pairs in testing/).
# Override via --calib for a different rig.
KITTI_FX_RECT    = 721.5377     # px
KITTI_BASELINE_M = 0.5371657    # m
KITTI_BF         = KITTI_FX_RECT * KITTI_BASELINE_M  # ≈ 387.5

# NOTE: this engine has its normalization baked into the ONNX/TRT graph.
# Applying ImageNet (mean/std) externally on top double-normalizes and
# pushes disparities ~12× too high. Verified empirically. Use /255 only.


class TRTLogger(trt.ILogger):
    def __init__(self): trt.ILogger.__init__(self)
    def log(self, severity, msg):
        if severity <= trt.ILogger.WARNING:
            print(f"[TRT] {msg}")


class StereoTRT:
    """Same loader as test_stereo_depth.py / node_stereo_depth.cpp."""
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

    def infer(self, l_chw, r_chw):
        l = torch.from_numpy(l_chw).cuda(non_blocking=True)
        r = torch.from_numpy(r_chw).cuda(non_blocking=True)
        self.io[self.left_in ].copy_(l, non_blocking=True)
        self.io[self.right_in].copy_(r, non_blocking=True)
        with torch.cuda.stream(self.stream):
            self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        d = self.io[self.disp_out].cpu().numpy()
        while d.ndim > 2: d = d[0]
        return d


def rgb_to_model_chw(rgb, h, w):
    """Resize → /255 → HWC -> CHW. Engine handles normalization internally."""
    if rgb.shape[:2] != (h, w):
        rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_LINEAR)
    arr = rgb.astype(np.float32) * (1.0 / 255.0)
    arr = arr.transpose(2, 0, 1)[None, ...]
    return np.ascontiguousarray(arr, dtype=np.float32)


def load_calib(path):
    """Read fx_rect + baseline from a stereo_calib.yaml. Falls back to KITTI."""
    if not path:
        return {"fx_rect": KITTI_FX_RECT, "baseline_m": KITTI_BASELINE_M,
                "bf": KITTI_BF, "source": "KITTI defaults"}
    import yaml
    with open(path) as f:
        c = yaml.safe_load(f)
    P1 = np.asarray(c["P1"]["data"], dtype=np.float64).reshape(int(c["P1"]["rows"]), int(c["P1"]["cols"]))
    P2 = np.asarray(c["P2"]["data"], dtype=np.float64).reshape(int(c["P2"]["rows"]), int(c["P2"]["cols"]))
    fx = float(P1[0, 0]); bf = -float(P2[0, 3])
    return {"fx_rect": fx, "baseline_m": bf / fx, "bf": bf, "source": path}


# matplotlib viewer (same design as test_stereo_depth.py — see that script
# for why we don't use cv2's window/setMouseCallback on this system).
import matplotlib
matplotlib.use("TkAgg" if "DISPLAY" in os.environ else "Agg")
import matplotlib.pyplot as plt


class FileViewer:
    MODES = ["rectified-left", "rectified-right", "disparity-color", "depth-heatmap"]

    def __init__(self, left_bgr, right_bgr, disp_eye, depth_m, depth_vmax, win_title):
        self.left = left_bgr
        self.right = right_bgr
        self.disp = disp_eye
        self.depth_m = depth_m
        self.depth_vmax = depth_vmax
        self.mode = 3
        self.hover_xy = None
        self._quit = False
        self.win_title = win_title

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(13, 6))
        try:
            self.fig.canvas.manager.set_window_title(win_title)
        except Exception:
            pass
        self.im = self.ax.imshow(self._render(), interpolation="nearest")
        self.ax.set_axis_off()
        self.title = self.ax.set_title(self._title_text(), fontsize=10)
        self.readout = self.ax.text(
            12, 36, "hover anywhere",
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

    def _title_text(self):
        return (f"{self.win_title}   |   mode: {self.MODES[self.mode]}   "
                f"|   vmax: {self.depth_vmax:.1f} m   |   c=cycle  s=save  q=quit")

    def _depth_at(self, x, y):
        if not (0 <= y < self.depth_m.shape[0] and 0 <= x < self.depth_m.shape[1]):
            return None
        z = float(self.depth_m[y, x])
        return z if np.isfinite(z) else None

    def _on_move(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None: return
        self.hover_xy = (int(ev.xdata), int(ev.ydata))
        self._refresh_readout()

    def _on_click(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None: return
        x, y = int(ev.xdata), int(ev.ydata)
        z = self._depth_at(x, y)
        d = self.disp[y, x] if (0 <= y < self.disp.shape[0] and 0 <= x < self.disp.shape[1]) else float("nan")
        if z is None:
            print(f"  [{x:4d},{y:4d}]  disp={d:.2f}px  no valid depth")
        else:
            print(f"  [{x:4d},{y:4d}]  disp={d:6.2f}px  depth={z*100:7.2f}cm ({z:.3f}m)")

    def _on_key(self, ev):
        if ev.key in ("q", "escape"): self._quit = True
        elif ev.key == "c":
            self.mode = (self.mode + 1) % len(self.MODES)
            self.im.set_data(self._render())
            self.title.set_text(self._title_text())
            self.fig.canvas.draw_idle()
        elif ev.key == "s":
            ts = datetime.now().strftime("%Y%m%d-%H%M%S")
            out_dir = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "testing"))
            os.makedirs(out_dir, exist_ok=True)
            out = os.path.join(out_dir, f"stereo_test_{ts}.png")
            cv2.imwrite(out, cv2.cvtColor(self._render(), cv2.COLOR_RGB2BGR))
            print(f"  saved {out}")

    def _refresh_readout(self):
        if self.hover_xy is None: return
        x, y = self.hover_xy
        z = self._depth_at(x, y)
        d = self.disp[y, x] if (0 <= y < self.disp.shape[0] and 0 <= x < self.disp.shape[1]) else float("nan")
        label = (f"[{x},{y}]  disp={d:.1f}px   {z*100:.1f} cm ({z:.2f} m)"
                 if z is not None else f"[{x},{y}]  no depth")
        self.readout.set_text(label)
        self.vline.set_xdata([x, x]); self.vline.set_visible(True)
        self.hline.set_ydata([y, y]); self.hline.set_visible(True)
        self.fig.canvas.draw_idle()

    def _disp_color(self):
        valid = self.disp > 0.5
        norm = np.zeros_like(self.disp, dtype=np.uint8)
        if valid.any():
            dmax = max(1.0, float(self.disp[valid].max()))
            norm[valid] = np.clip(self.disp[valid] / dmax * 255, 0, 255).astype(np.uint8)
        out = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        out[~valid] = 0
        return cv2.cvtColor(out, cv2.COLOR_BGR2RGB)

    def _depth_heat(self):
        valid = np.isfinite(self.depth_m)
        z = self.depth_m.copy(); z[~valid] = self.depth_vmax
        z = np.clip(z, 0.0, self.depth_vmax)
        norm = ((1.0 - z / self.depth_vmax) * 255).astype(np.uint8)
        out = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        out[~valid] = 0
        return cv2.cvtColor(out, cv2.COLOR_BGR2RGB)

    def _render(self):
        m = self.MODES[self.mode]
        if   m == "rectified-left":   return cv2.cvtColor(self.left,  cv2.COLOR_BGR2RGB)
        elif m == "rectified-right":  return cv2.cvtColor(self.right, cv2.COLOR_BGR2RGB)
        elif m == "disparity-color":  return self._disp_color()
        else:                          return self._depth_heat()

    def loop(self):
        while not self._quit:
            try:
                self.fig.canvas.flush_events()
                time.sleep(0.02)
                if not plt.fignum_exists(self.fig.number):
                    break
            except KeyboardInterrupt:
                break
        plt.close(self.fig)


# ─────────────────────────────────────────────────────────────────── main
def run_pair(engine, calib, left_path, right_path, depth_vmax, show=True, label=""):
    left_bgr  = cv2.imread(left_path)
    right_bgr = cv2.imread(right_path)
    if left_bgr is None or right_bgr is None:
        print(f"ERROR: failed to read {left_path} or {right_path}")
        return None
    H, W = left_bgr.shape[:2]
    print(f"\n=== {label or os.path.basename(left_path)} ===")
    print(f"  size       : {W}x{H}")

    l_in = rgb_to_model_chw(cv2.cvtColor(left_bgr,  cv2.COLOR_BGR2RGB), engine.model_h, engine.model_w)
    r_in = rgb_to_model_chw(cv2.cvtColor(right_bgr, cv2.COLOR_BGR2RGB), engine.model_h, engine.model_w)

    t0 = time.perf_counter()
    disp_model = engine.infer(l_in, r_in)
    dt = (time.perf_counter() - t0) * 1000

    # Upscale to source res, scale model-pixel units → source-pixel units.
    disp_src = cv2.resize(disp_model, (W, H), interpolation=cv2.INTER_LINEAR)
    disp_src *= (W / engine.model_w)
    depth_m = np.where(disp_src > 0.5, calib["bf"] / disp_src, np.float32(np.nan)).astype(np.float32)

    valid = np.isfinite(depth_m)
    z = depth_m[valid]
    print(f"  inference  : {dt:.2f} ms")
    print(f"  disparity  : model-px [{disp_model.min():.2f}, {disp_model.max():.2f}]   "
          f"src-px [{disp_src.min():.2f}, {disp_src.max():.2f}]")
    print(f"  depth (m)  : valid {100*valid.sum()/depth_m.size:.1f}%   "
          f"min={z.min():.3f}  median={np.median(z):.3f}  "
          f"p95={np.quantile(z,0.95):.3f}  max={z.max():.3f}")
    print(f"  using bf   : {calib['bf']:.3f}   (fx_rect={calib['fx_rect']:.2f}  baseline={calib['baseline_m']*1000:.2f} mm)")
    print(f"  calib src  : {calib['source']}")

    # Sanity check: are far/close getting the right disparity sign? In a
    # properly-rectified pair, near objects → high disparity, far → low.
    # For KITTI street scenes the top quarter is mostly sky/distant (low
    # disp); the bottom quarter is mostly road foreground (higher disp).
    upper_disp = float(np.median(disp_src[:H // 4]))
    lower_disp = float(np.median(disp_src[3 * H // 4:]))
    print(f"  sanity     : top-quarter median disp = {upper_disp:.2f} px   "
          f"bottom-quarter median disp = {lower_disp:.2f} px   "
          f"(expect bottom > top if scene has near foreground / far background)")

    if not show:
        return depth_m
    viewer = FileViewer(left_bgr, right_bgr, disp_src, depth_m, depth_vmax,
                        win_title=label or os.path.basename(left_path))
    print("\n  hover → distance | click → log | c cycle | s save | q quit")
    viewer.loop()
    return depth_m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left",   default="../testing/000003-left.png")
    ap.add_argument("--right",  default="../testing/000003-right.png")
    ap.add_argument("--engine", default="../src/graphs/stereo-depth/lightstereo_s_320x512.engine")
    ap.add_argument("--calib",  default="",
                    help="optional stereo_calib.yaml; if omitted uses KITTI defaults "
                         "(fx≈721, baseline≈0.54 m) — correct for testing/0000*-*.png")
    ap.add_argument("--depth-vmax", type=float, default=50.0,
                    help="upper bound (m) for the heatmap viz (KITTI samples have "
                         "outdoor depths up to ~80 m; default 50)")
    ap.add_argument("--warmup", type=int, default=2)
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--batch", action="store_true",
                    help="run all pairs found by globbing testing/*-left.png — "
                         "useful to compare side-by-side on stdout")
    args = ap.parse_args()

    sd = os.path.dirname(os.path.abspath(__file__))
    def rel(p):
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(sd, p))

    eng_path = rel(args.engine)
    if not os.path.exists(eng_path):
        print(f"ERROR: engine not found: {eng_path}"); sys.exit(1)
    calib = load_calib(rel(args.calib) if args.calib else "")
    print(f"engine : {eng_path}")
    print(f"calib  : {calib['source']}   bf={calib['bf']:.3f}   "
          f"fx_rect={calib['fx_rect']:.2f}   baseline={calib['baseline_m']*1000:.2f} mm")

    engine = StereoTRT(eng_path)
    print(f"engine : input {engine.model_h}×{engine.model_w}")

    # Warm up so the first measured inference isn't slow.
    z = np.zeros((1, 3, engine.model_h, engine.model_w), dtype=np.float32)
    for _ in range(args.warmup):
        engine.infer(z, z)

    if args.batch:
        from glob import glob
        lefts = sorted(glob(os.path.join(sd, "..", "testing", "*-left.png")))
        if not lefts:
            print("no testing/*-left.png found"); sys.exit(1)
        for lp in lefts:
            rp = lp.replace("-left.png", "-right.png")
            if not os.path.exists(rp):
                print(f"  skip {os.path.basename(lp)}: missing matching right"); continue
            run_pair(engine, calib, lp, rp, args.depth_vmax, show=not args.no_display,
                     label=os.path.basename(lp).replace("-left.png", ""))
        return 0

    run_pair(engine, calib, rel(args.left), rel(args.right), args.depth_vmax,
             show=not args.no_display)
    return 0


if __name__ == "__main__":
    sys.exit(main())
