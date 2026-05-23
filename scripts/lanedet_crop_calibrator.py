#!/usr/bin/env python3
"""
Standalone live lane-detection horizon-crop calibrator.

The lane network is highly sensitive to *where* the horizon crop starts —
too much sky and the model loses lanes; too little and it can't see them
far enough out. Production node_lanedet hardcoded a 50/50 crop forever,
which only happens to be optimal on rigs whose mount + FOV centers the
horizon at exactly mid-image. This tool lets you find the right ratio
for your rig and feed it back into config/config.yaml.

Pipeline (matches node_lanedet.cpp exactly):
    1. V4L2 capture: 3840×1200 UYVY @ 30 fps from /dev/video1
    2. Per-eye inner-edge crop → 1440×900 (drop 480 px outer side,
       150 px symmetric top/bottom). LEFT eye only.
    3. Per-eye rotate-180 (camera mounted upside-down on this rig).
    4. Crop from y = H * crop_ratio to bottom → resize to 976×208.
    5. Preprocess: BGR→RGB, subtract 125, HWC→CHW (no /255, no ImageNet).
    6. TensorRT enqueue on lane_detect.engine.
    7. Postprocess: 4 lane channels × 50 sample rows; threshold and
       remap points back into the 1440×900 eye frame using
       scaleY = CROP_HEIGHT / 208 (NOT (IMAGE_H − CROP) / 208 like the
       old C++ — that was only correct at 50/50).

REQUIRED: stop the ROS camera_stereo node before running — V4L2 lets
only one process hold /dev/video1.

Usage:
    python3 scripts/lanedet_crop_calibrator.py            # defaults
    python3 scripts/lanedet_crop_calibrator.py --no-rotate
    python3 scripts/lanedet_crop_calibrator.py --eye right

Viewer keys:
    up / down   → adjust crop_ratio by ±0.01 (Shift = ±0.05)
    r           → reset to 0.50
    h           → toggle horizon line + crop-band overlay
    s           → save snapshot to testing/lanedet_crop_<ratio>_<ts>.png
    q / ESC     → quit and print the chosen crop_ratio

After calibration, put the chosen value into config/config.yaml under
`lanedet.ros__parameters.crop_ratio` and `colcon build --packages-select
visionconnect`.
"""

import argparse
import os
import sys
import time
from datetime import datetime

# Same X11 setup as test_stereo_depth.py — matplotlib backend needs a reachable
# DISPLAY+XAUTHORITY when launched from bare terminals or systemd.
os.environ.setdefault("DISPLAY", ":0")
if not os.environ.get("XAUTHORITY"):
    for _xauth in (f"{os.path.expanduser('~')}/.Xauthority",
                   f"/run/user/{os.getuid()}/gdm/Xauthority",
                   f"/run/user/{os.getuid()}/mutter/Xauthority"):
        if os.path.isfile(_xauth):
            os.environ["XAUTHORITY"] = _xauth
            break

import cv2
import numpy as np

try:
    import tensorrt as trt
    import torch
except ImportError as e:
    print(f"missing dependency: {e}")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────── geometry
# Mirrors node_camera_stereo.cpp / test_stereo_depth.py.
RAW_W, RAW_H        = 3840, 1200
EYE_W, EYE_H        = 1440, 900
CROP_X_OUTER        = 480
CROP_Y              = 150
SENSOR_LEFT_START   = CROP_X_OUTER                       # 480
SENSOR_RIGHT_START  = RAW_W - CROP_X_OUTER - EYE_W       # 1920

# Lane network input geometry (matches node_lanedet.cpp #defines).
NET_W, NET_H        = 976, 208
NUM_LANE_CHANNELS   = 4       # background channel 0 + 4 lane channels = 5 output channels total
POINTS_COUNT        = 50      # = same linspace step as node_lanedet GetLine
THR                 = 0.25    # existence + per-row score threshold

LANE_COLORS_BGR = [
    (255,   0,   0),  # 0 blue
    (  0, 255,   0),  # 1 green
    (  0,   0, 255),  # 2 red
    (255, 255,   0),  # 3 cyan
]

# Preprocessing presets. The constants live in RGB channel order — the
# network sees RGB after BGR→RGB swap. ERFNet-CULane (cardwing repo) was
# trained with VGG-style BGR mean centering: subtract [103.939, 116.779,
# 123.68] in BGR, no /255, no std. Expressed in RGB-input order that's
# sub=[123.68, 116.779, 103.939], which is the "vgg" preset below.
PREPROC_PRESETS = {
    # name        sub (RGB)                     div (RGB)               /255
    "legacy":   ([125.0,  125.0,  125.0],   [1.0,    1.0,    1.0],    False),
    "vgg":      ([123.68, 116.779, 103.939], [1.0,    1.0,    1.0],    False),
    "imagenet": ([0.485,  0.456,  0.406],   [0.229,  0.224,  0.225],  True),
}
PRESET_ORDER = ["vgg", "legacy", "imagenet"]


# ─────────────────────────────────────────────────────────────────── TRT
class TRTLogger(trt.ILogger):
    def __init__(self): trt.ILogger.__init__(self)
    def log(self, severity, msg):
        if severity <= trt.ILogger.WARNING:
            print(f"[TRT] {msg}")


class LaneTRT:
    """Loads the lane_detect engine and exposes infer(chw_np) -> (maps, exist)."""
    def __init__(self, engine_path):
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(TRTLogger()).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"deserialize failed: {engine_path}")
        self.context = self.engine.create_execution_context()

        # Identify IO by shape: input is 4D, output 583 is 4D, output 580 is 2D.
        self.in_name = None
        self.maps_name = None
        self.exist_name = None
        for i in range(self.engine.num_io_tensors):
            n = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(n)
            shape = tuple(self.engine.get_tensor_shape(n))
            if mode == trt.TensorIOMode.INPUT:
                self.in_name = n
                self.in_shape = shape
            else:
                if len(shape) == 4:
                    self.maps_name = n
                    self.maps_shape = shape
                else:
                    self.exist_name = n
                    self.exist_shape = shape
        if not (self.in_name and self.maps_name and self.exist_name):
            raise RuntimeError("engine missing expected lane input / maps / existence tensors")

        self.io = {
            self.in_name:    torch.empty(self.in_shape,    dtype=torch.float32, device="cuda"),
            self.maps_name:  torch.empty(self.maps_shape,  dtype=torch.float32, device="cuda"),
            self.exist_name: torch.empty(self.exist_shape, dtype=torch.float32, device="cuda"),
        }
        for n, t in self.io.items():
            self.context.set_tensor_address(n, t.data_ptr())
        self.stream = torch.cuda.Stream()

    def infer(self, chw_np):
        t = torch.from_numpy(chw_np).cuda(non_blocking=True)
        self.io[self.in_name].copy_(t, non_blocking=True)
        with torch.cuda.stream(self.stream):
            self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        maps  = self.io[self.maps_name ].cpu().numpy()[0]   # (5, 208, 976)
        exist = self.io[self.exist_name].cpu().numpy()[0]   # (4,)
        return maps, exist


# ─────────────────────────────────────────────────────────────────── capture
def open_capture(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(
            f"could not open {device}. STOP VisionSense first — only one "
            "process can hold the V4L2 device. Run `sudo systemctl stop "
            "visionsense` (or pkill the camera_stereo node) and retry.")
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


def grab_eye(frame, eye, rotate_180):
    """Return one eye's 1440×900 BGR view, matching node_camera_stereo."""
    x0 = SENSOR_LEFT_START if eye == "left" else SENSOR_RIGHT_START
    sub = frame[CROP_Y:CROP_Y + EYE_H, x0:x0 + EYE_W]
    if rotate_180:
        sub = cv2.rotate(sub, cv2.ROTATE_180)
    return sub


# ─────────────────────────────────────────────────────────────────── pipeline
def apply_clahe_band(bgr_band, clip, grid):
    """CLAHE on luma channel only; preserves color. Mirrors node_lanedet."""
    ycc = cv2.cvtColor(bgr_band, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycc)
    clahe = cv2.createCLAHE(clipLimit=float(clip), tileGridSize=(int(grid), int(grid)))
    y = clahe.apply(y)
    return cv2.cvtColor(cv2.merge([y, cr, cb]), cv2.COLOR_YCrCb2BGR)


def preprocess(eye_bgr, crop_ratio, preset_name, clahe_enable, clahe_clip, clahe_grid):
    """
    Crop top → optional CLAHE → resize to NET → BGR→RGB → (sub/div, optional /255)
    → HWC→CHW. Matches node_lanedet.cpp + src/cuda/preprocess.cu exactly.
    """
    h, w = eye_bgr.shape[:2]
    y0 = int(h * crop_ratio)
    y0 = max(0, min(h - 4, y0))           # never empty
    cropped = eye_bgr[y0:h, :]
    if clahe_enable:
        cropped = apply_clahe_band(cropped, clahe_clip, clahe_grid)

    resized = cv2.resize(cropped, (NET_W, NET_H), interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32)

    sub, div, normalize = PREPROC_PRESETS[preset_name]
    if normalize:
        rgb *= (1.0 / 255.0)
    rgb[..., 0] = (rgb[..., 0] - sub[0]) / div[0]
    rgb[..., 1] = (rgb[..., 1] - sub[1]) / div[1]
    rgb[..., 2] = (rgb[..., 2] - sub[2]) / div[2]

    chw = rgb.transpose(2, 0, 1)[None, ...]
    return np.ascontiguousarray(chw, dtype=np.float32), y0, (h - y0), cropped


def linspace_rows():
    """Same as node_lanedet: linspace(4, INPUT_H - 1, POINTS_COUNT)."""
    return np.linspace(4, NET_H - 1, POINTS_COUNT).astype(np.int32)


def extract_lanes(maps, existence, crop_start_y, crop_height, eye_w, eye_h):
    """
    Mirror of GetLines/GetLane from node_lanedet.cpp.

    `maps` is (5, NET_H, NET_W) — channel 0 = background, 1..4 = lanes.
    `existence` is (4,). Lanes are picked by argmax across each row of
    YS = linspace(4, NET_H-1, 50). Points with maxVal < THR are dropped.
    Output points are remapped from NET-space to full-eye-image space
    using the same scaleY = CROP_HEIGHT / NET_H formula as the C++ fix.
    """
    ys_in = linspace_rows()
    scaleX = eye_w / float(NET_W)
    scaleY = crop_height / float(NET_H)

    lanes_out = []
    exist_out = []
    for ch in range(NUM_LANE_CHANNELS):
        chan = ch + 1                     # +1 to skip background channel 0
        score = maps[chan]
        # Cheap 11×11 box blur to match the cv::blur call in node_lanedet.
        score = cv2.blur(score, (11, 11))

        exists_score = float(existence[ch])
        if exists_score <= THR and ch != 0:
            lanes_out.append([])
            exist_out.append(exists_score)
            continue

        pts = []
        for y_in in ys_in:
            row = score[y_in, :]
            x_in = int(np.argmax(row))
            if row[x_in] > THR:
                x_full = int((x_in + 1) * scaleX)
                y_full = int(y_in * scaleY + crop_start_y)
                pts.append((x_full, y_full))

        if len(pts) >= 2:
            lanes_out.append(pts)
        else:
            lanes_out.append([])
        exist_out.append(exists_score)
    return lanes_out, exist_out


def draw_overlay(eye_bgr, lanes, exist_scores, crop_start_y, crop_height,
                 crop_ratio, show_horizon, fps_str,
                 preset_name="vgg", clahe_enable=False):
    """Paint lanes + crop band + status text on a copy of eye_bgr."""
    out = eye_bgr.copy()
    h, w = out.shape[:2]

    if show_horizon:
        # Shade the area ABOVE the crop (= region the network never sees).
        shade = out.copy()
        cv2.rectangle(shade, (0, 0), (w, crop_start_y), (60, 60, 60), -1)
        out = cv2.addWeighted(shade, 0.45, out, 0.55, 0)
        cv2.line(out, (0, crop_start_y), (w, crop_start_y), (0, 255, 255), 2)
        cv2.putText(out, "horizon (top of crop)", (10, max(20, crop_start_y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    for i, pts in enumerate(lanes):
        if len(pts) < 2:
            continue
        col = LANE_COLORS_BGR[i % len(LANE_COLORS_BGR)]
        pts_np = np.asarray(pts, dtype=np.int32)
        cv2.polylines(out, [pts_np], False, col, 5)
        for (x, y) in pts:
            cv2.circle(out, (x, y), 4, col, -1)

    # Status panel
    n_valid = sum(1 for p in lanes if len(p) >= 2)
    lines = [
        f"crop_ratio = {crop_ratio:.3f}    crop_start_y = {crop_start_y}px    "
        f"crop_height = {crop_height}px",
        f"preset = {preset_name}    CLAHE = {'ON' if clahe_enable else 'off'}    "
        f"valid lanes: {n_valid}/4",
        f"existence: " + "  ".join(f"ch{i}={e:.2f}" for i, e in enumerate(exist_scores)),
        f"{fps_str}    keys: ↑/↓ crop  p=preset  k=CLAHE  r=reset  h=horizon  s=save  q=quit",
    ]
    y_text = 30
    for ln in lines:
        (tw, th), _ = cv2.getTextSize(ln, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(out, (8, y_text - th - 6), (12 + tw, y_text + 6), (0, 0, 0), -1)
        cv2.putText(out, ln, (10, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y_text += th + 14
    return out


# ─────────────────────────────────────────────────────────────────── viewer
# matplotlib (same reasoning as test_stereo_depth.py — OpenCV 4.13 Qt
# back-end on this Jetson doesn't register windows correctly).
import matplotlib
matplotlib.use("TkAgg" if "DISPLAY" in os.environ else "Agg")
import matplotlib.pyplot as plt


class LiveCalibrator:
    def __init__(self, init_ratio, win_title,
                 init_preset="vgg", init_clahe=False,
                 clahe_clip=2.0, clahe_grid=8):
        self.crop_ratio = float(init_ratio)
        self.preset_name = init_preset
        self.clahe_enable = bool(init_clahe)
        self.clahe_clip = float(clahe_clip)
        self.clahe_grid = int(clahe_grid)
        self.show_horizon = True
        self._quit = False
        self._save_pending = False

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(13, 7))
        self.fig.canvas.manager.set_window_title(win_title)
        self.im = self.ax.imshow(np.zeros((EYE_H, EYE_W, 3), dtype=np.uint8),
                                 interpolation="nearest")
        self.ax.set_axis_off()
        self.title = self.ax.set_title("waiting for first frame…",
                                       fontsize=11, color="black")
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("close_event",
                                    lambda _ev: setattr(self, "_quit", True))
        plt.tight_layout()
        plt.show(block=False)

    def _on_key(self, ev):
        step = 0.05 if (ev.key and "shift" in (ev.key or "")) else 0.01
        if   ev.key in ("q", "escape"):     self._quit = True
        elif ev.key in ("up",   "shift+up"):
            self.crop_ratio = min(0.95, self.crop_ratio + step)
            print(f"  crop_ratio = {self.crop_ratio:.3f}")
        elif ev.key in ("down", "shift+down"):
            self.crop_ratio = max(0.00, self.crop_ratio - step)
            print(f"  crop_ratio = {self.crop_ratio:.3f}")
        elif ev.key == "r":
            self.crop_ratio = 0.5
            print(f"  crop_ratio reset to 0.500")
        elif ev.key == "h":
            self.show_horizon = not self.show_horizon
        elif ev.key == "p":
            idx = (PRESET_ORDER.index(self.preset_name) + 1) % len(PRESET_ORDER)
            self.preset_name = PRESET_ORDER[idx]
            sub, div, norm = PREPROC_PRESETS[self.preset_name]
            print(f"  preset = {self.preset_name}    sub(RGB)={sub}  div(RGB)={div}  /255={norm}")
        elif ev.key == "k":
            self.clahe_enable = not self.clahe_enable
            print(f"  CLAHE = {'ON' if self.clahe_enable else 'off'} "
                  f"(clip={self.clahe_clip:.2f}, tile={self.clahe_grid}x{self.clahe_grid})")
        elif ev.key == "s":
            self._save_pending = True

    def consume_save(self):
        v = self._save_pending; self._save_pending = False; return v

    def update(self, bgr_overlay, fps_str):
        rgb = cv2.cvtColor(bgr_overlay, cv2.COLOR_BGR2RGB)
        self.im.set_data(rgb)
        self.title.set_text(
            f"lanedet calibrator   |   crop={self.crop_ratio:.3f}   "
            f"|   preset={self.preset_name}   |   CLAHE={'on' if self.clahe_enable else 'off'}   "
            f"|   {fps_str}")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def should_quit(self):
        return self._quit


# ─────────────────────────────────────────────────────────────────── main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video1")
    ap.add_argument("--eye", choices=("left", "right"), default="left",
                    help="which stereo eye to feed the model (default: left)")
    ap.add_argument("--engine",
                    default="../install/visionconnect/share/visionconnect/"
                            "graphs/lane-detection/lane_detect.engine")
    ap.add_argument("--crop-ratio", type=float, default=0.60,
                    help="initial horizon crop ratio (0..0.95). Adjust live "
                         "with arrow keys. Default 0.60 (current rig optimum).")
    ap.add_argument("--preset", choices=tuple(PREPROC_PRESETS.keys()), default="vgg",
                    help="preprocessing constants preset. vgg = ERFNet-CULane "
                         "training default (BGR-VGG means, no /255). "
                         "legacy = uniform 125 (pre-fix default). "
                         "imagenet = true ImageNet (RGB mean/std with /255). "
                         "Cycle live with 'p'.")
    ap.add_argument("--clahe", action="store_true",
                    help="apply CLAHE (luma only) to the cropped band before "
                         "feeding the network. Toggle live with 'k'.")
    ap.add_argument("--clahe-clip", type=float, default=2.0)
    ap.add_argument("--clahe-grid", type=int,   default=8)
    ap.add_argument("--no-rotate", action="store_true",
                    help="skip per-eye 180° rotation (only set if camera is "
                         "now mounted upright)")
    ap.add_argument("--warmup", type=int, default=3)
    args = ap.parse_args()

    sd = os.path.dirname(os.path.abspath(__file__))
    eng_path = args.engine if os.path.isabs(args.engine) \
        else os.path.normpath(os.path.join(sd, args.engine))
    if not os.path.exists(eng_path):
        print(f"ERROR: engine not found: {eng_path}")
        print("  (build the package first: `colcon build --packages-select visionconnect`)")
        sys.exit(1)

    print(f"device : {args.device}")
    print(f"engine : {eng_path}")
    print(f"eye    : {args.eye}")
    print(f"rotate : {'no' if args.no_rotate else 'yes'}")
    print(f"crop   : {args.crop_ratio:.3f} (initial)")
    print(f"preset : {args.preset}  (sub={PREPROC_PRESETS[args.preset][0]}  "
          f"div={PREPROC_PRESETS[args.preset][1]}  /255={PREPROC_PRESETS[args.preset][2]})")
    print(f"clahe  : {'on' if args.clahe else 'off'} (clip={args.clahe_clip}, tile={args.clahe_grid})")

    engine = LaneTRT(eng_path)
    print(f"engine : input {engine.in_shape}  maps {engine.maps_shape}  exist {engine.exist_shape}")

    cap = open_capture(args.device)
    print(f"cap    : opened {args.device} @ {RAW_W}×{RAW_H} UYVY 30 fps")

    # Warmup so first frame timing isn't bloated by lazy CUDA allocation.
    dummy_chw = np.zeros((1, 3, NET_H, NET_W), dtype=np.float32)
    for _ in range(args.warmup):
        engine.infer(dummy_chw)

    viewer = LiveCalibrator(args.crop_ratio,
                            "Lanedet Calibrator — ↑/↓ crop | p preset | k CLAHE | h horizon | s save | q quit",
                            init_preset=args.preset, init_clahe=args.clahe,
                            clahe_clip=args.clahe_clip, clahe_grid=args.clahe_grid)
    print("\n  ↑ / ↓   → crop_ratio ±0.01 (Shift = ±0.05)")
    print("  p       → cycle preset (vgg → legacy → imagenet)")
    print("  k       → toggle CLAHE on/off")
    print("  r       → reset crop to 0.50")
    print("  h       → toggle horizon overlay")
    print("  s       → save snapshot")
    print("  q / ESC → quit (prints final crop_ratio + preset + CLAHE state)\n")

    fail = 0
    t_last_log = time.perf_counter()
    n_since_log = 0
    fps_str = "—"

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            fail += 1
            if fail > 30:
                print("capture failed >30 times, giving up")
                break
            time.sleep(0.03)
            continue
        fail = 0

        eye = grab_eye(frame, args.eye, rotate_180=not args.no_rotate)
        chw, y0, ch, _band = preprocess(
            eye, viewer.crop_ratio, viewer.preset_name,
            viewer.clahe_enable, viewer.clahe_clip, viewer.clahe_grid)

        t_inf = time.perf_counter()
        maps, existence = engine.infer(chw)
        infer_ms = (time.perf_counter() - t_inf) * 1000

        lanes, exist_scores = extract_lanes(maps, existence, y0, ch, EYE_W, EYE_H)

        overlay = draw_overlay(eye, lanes, exist_scores, y0, ch,
                               viewer.crop_ratio, viewer.show_horizon, fps_str,
                               preset_name=viewer.preset_name,
                               clahe_enable=viewer.clahe_enable)

        if viewer.consume_save():
            out_dir = os.path.normpath(os.path.join(sd, "..", "testing"))
            os.makedirs(out_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d-%H%M%S")
            tag = f"crop{viewer.crop_ratio:.3f}_{viewer.preset_name}" \
                  f"{'_clahe' if viewer.clahe_enable else ''}"
            out_path = os.path.join(out_dir, f"lanedet_{tag}_{ts}.png")
            cv2.imwrite(out_path, overlay)
            print(f"  saved {out_path}")

        n_since_log += 1
        if time.perf_counter() - t_last_log >= 1.0:
            fps = n_since_log / (time.perf_counter() - t_last_log)
            fps_str = f"{fps:.1f} fps (infer {infer_ms:.1f} ms)"
            t_last_log = time.perf_counter()
            n_since_log = 0

        viewer.update(overlay, fps_str)
        if viewer.should_quit():
            break

    cap.release()
    cv2.destroyAllWindows()
    sub, div, norm = PREPROC_PRESETS[viewer.preset_name]
    print(f"\nfinal crop_ratio  = {viewer.crop_ratio:.3f}")
    print(f"final preset      = {viewer.preset_name}")
    print(f"  pre_sub_rgb : {sub}")
    print(f"  pre_div_rgb : {div}")
    print(f"  pre_normalize: {norm}")
    print(f"final CLAHE       = {'on' if viewer.clahe_enable else 'off'} "
          f"(clip={viewer.clahe_clip}, tile={viewer.clahe_grid})")
    print(f"\n  → Update config/config.yaml under lanedet.ros__parameters,")
    print(f"    then `colcon build --packages-select visionconnect` to take effect.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
