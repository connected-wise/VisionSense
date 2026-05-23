#!/usr/bin/env python3
"""Standalone ONNX runner for the YOLO26 detect model.

Goal: troubleshoot the "traffic lights never detected" symptom in node_detect.
Runs the engine in its native ONNX form (no TensorRT in the path) so we can
isolate whether the model itself emits traffic-light boxes at all, separately
from any preprocessing / class-mapping bug in the C++ node.

Model contract (verified from the .onnx graph):
    input  images : float32 [1, 3, 640, 640], RGB, /255, letterboxed
    output output0: float32 [1, 300, 6]  -> [x1, y1, x2, y2, score, class_id]
                    in the 640x640 letterboxed space, NMS already applied.

Class mapping (per the user, NOT what labels_detect.txt currently has — that
file holds the legacy slot order used by downstream nodes):
    0 car, 1 truck, 2 bus, 3 train, 4 bike,
    5 cyclist, 6 person, 7 traffic_light, 8 traffic_sign

Usage:
    python3 scripts/test_detect_onnx.py --image testing/full_left.png
    python3 scripts/test_detect_onnx.py --image <path> --conf 0.05 --out /tmp/det.png

A low --conf is the point: pass 0.05 to see whether traffic-light boxes are
emitted at all, even if the model isn't confident enough to clear the
production threshold (0.40 in config.yaml).
"""
import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

CLASS_NAMES = [
    "car",           # 0
    "truck",         # 1
    "bus",           # 2
    "train",         # 3
    "bike",          # 4
    "cyclist",       # 5
    "person",        # 6
    "traffic_light", # 7
    "traffic_sign",  # 8
]

# Distinct BGR colors for each class.
CLASS_COLORS = [
    (200, 180,  40),  # car         - cyan-ish
    (40,   80, 220),  # truck       - red-ish
    (40,  180, 220),  # bus         - orange
    (180,  40, 220),  # train       - magenta
    (90,  220,  40),  # bike        - green
    (220, 220,  40),  # cyclist     - yellow-cyan
    (40,  220, 220),  # person      - yellow
    (0,   140, 255),  # traffic_light - bright orange (HIGHLIGHT)
    (200,  60, 200),  # traffic_sign  - purple
]


def letterbox(img_bgr, new_size=640, color=(114, 114, 114), top_left=False):
    """Resize with unchanged aspect ratio, pad to (new_size, new_size).

    top_left=False (default): centered padding, YOLO-standard.
    top_left=True: bottom/right padding only — mimics src/cuda/preprocess.cu,
    which places the resized image at (0,0) and fills the bottom/right strip.

    Returns (canvas, ratio, (pad_w, pad_h)) where pad_w/pad_h is the offset of
    the resized image's top-left corner inside the canvas (0 in top_left mode).
    """
    h, w = img_bgr.shape[:2]
    ratio = min(new_size / w, new_size / h)
    new_w, new_h = int(round(w * ratio)), int(round(h * ratio))
    resized = cv2.resize(img_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    if top_left:
        pad_w, pad_h = 0, 0
    else:
        pad_w = (new_size - new_w) // 2
        pad_h = (new_size - new_h) // 2
    canvas = np.full((new_size, new_size, 3), color, dtype=np.uint8)
    canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized
    return canvas, ratio, (pad_w, pad_h)


def preprocess(img_bgr, input_size=640, mimic_cpp=False):
    """Standard YOLO preprocess, or C++-pipeline mimic.

    mimic_cpp=True reproduces what src/cuda/preprocess.cu does on rgb8 frames
    from camera_stereo: top-left letterbox with BLACK padding, no R/B swap
    (assumes input is already RGB-ordered, like the C++ kernel sees from the
    rgb8 message bytes). Use this to A/B whether the C++ preprocess is what's
    killing traffic-light detections.
    """
    if mimic_cpp:
        # Assume img_bgr is actually BGR from cv2.imread; convert to RGB FIRST
        # so the byte layout matches what the C++ kernel reads from an rgb8
        # ROS message (R,G,B order in memory). Then letterbox-top-left with
        # BLACK pad and skip any further swap.
        rgb_in = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        canvas, ratio, pad = letterbox(rgb_in, input_size, color=(0, 0, 0), top_left=True)
        x = canvas.astype(np.float32) * (1.0 / 255.0)
    else:
        canvas, ratio, pad = letterbox(img_bgr, input_size,
                                       color=(114, 114, 114), top_left=False)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        x = rgb.astype(np.float32) * (1.0 / 255.0)
    x = np.transpose(x, (2, 0, 1))[None]  # NCHW
    return np.ascontiguousarray(x), ratio, pad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="src/graphs/object-detection/yolo26_cw.onnx",
                    help="Path to YOLO26 ONNX (relative to repo root or absolute).")
    ap.add_argument("--image", required=True,
                    help="Path to a test image (PNG/JPG) containing traffic lights.")
    ap.add_argument("--conf", type=float, default=0.05,
                    help="Confidence cutoff for printing/drawing (default 0.05 — "
                         "low on purpose to see if low-confidence traffic-light "
                         "boxes are emitted at all).")
    ap.add_argument("--out", default="../testing/test_detect_out.png",
                    help="Annotated output image path.")
    ap.add_argument("--providers", default="auto",
                    help='ORT execution providers, comma-separated, or "auto" '
                         '(tries CUDA then CPU).')
    ap.add_argument("--mimic-cpp", action="store_true",
                    help="Use the C++ pipeline's preprocess (top-left letterbox, "
                         "BLACK pad, no R/B swap) instead of standard YOLO "
                         "preprocess. Use this to isolate whether the C++ "
                         "preprocess kernel is what's suppressing traffic-light "
                         "detections.")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    model_path = args.model
    if not os.path.isabs(model_path):
        model_path = str(repo_root / model_path)
    if not os.path.isfile(model_path):
        print(f"ERROR: model not found: {model_path}", file=sys.stderr)
        sys.exit(1)

    img_path = args.image
    if not os.path.isabs(img_path):
        img_path = str(repo_root / img_path)
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    if img is None:
        print(f"ERROR: failed to read image: {img_path}", file=sys.stderr)
        sys.exit(1)
    print(f"image: {img_path}  shape={img.shape}")

    if args.providers == "auto":
        providers = []
        avail = ort.get_available_providers()
        if "CUDAExecutionProvider" in avail:
            providers.append("CUDAExecutionProvider")
        providers.append("CPUExecutionProvider")
    else:
        providers = [p.strip() for p in args.providers.split(",") if p.strip()]
    print(f"ort providers: {providers}")

    sess = ort.InferenceSession(model_path, providers=providers)
    in_name = sess.get_inputs()[0].name
    in_shape = sess.get_inputs()[0].shape
    out_name = sess.get_outputs()[0].name
    print(f"model in: {in_name} {in_shape}    out: {out_name}")

    input_size = int(in_shape[2]) if isinstance(in_shape[2], int) else 640
    print(f"preprocess mode: {'C++ mimic (top-left, black pad)' if args.mimic_cpp else 'standard YOLO (centered, gray-114 pad)'}")
    x, ratio, (pad_w, pad_h) = preprocess(img, input_size, mimic_cpp=args.mimic_cpp)

    t0 = time.perf_counter()
    (out,) = sess.run([out_name], {in_name: x})
    t1 = time.perf_counter()
    print(f"inference: {(t1 - t0) * 1000:.1f} ms   output shape: {out.shape}")

    # out: [1, 300, 6] -> [N, 6]
    dets = out[0]

    # Per-class summary across ALL rows (no conf filter), to catch the case
    # where traffic-light dets exist but sit below the production threshold.
    print("\nper-class top score (any row, regardless of threshold):")
    for cls_id in range(len(CLASS_NAMES)):
        mask = dets[:, 5].astype(int) == cls_id
        if mask.any():
            top = dets[mask, 4].max()
            count = int((dets[mask, 4] > args.conf).sum())
            print(f"  [{cls_id}] {CLASS_NAMES[cls_id]:14s}  top_score={top:.3f}  "
                  f"count(>{args.conf})={count}")
        else:
            print(f"  [{cls_id}] {CLASS_NAMES[cls_id]:14s}  (no rows)")

    # Apply confidence threshold for drawing.
    keep = dets[:, 4] > args.conf
    dets = dets[keep]
    # Sort by score desc for legibility.
    order = np.argsort(-dets[:, 4])
    dets = dets[order]

    print(f"\ndetections above conf={args.conf}: {len(dets)}")
    print(f"{'idx':>3}  {'cls':>3}  {'name':<14}  {'score':>6}   bbox(x1,y1,x2,y2 orig)")

    img_h, img_w = img.shape[:2]
    annotated = img.copy()
    for i, d in enumerate(dets):
        x1, y1, x2, y2, score, cls = d
        cls_id = int(cls)
        # Map letterboxed coords back to original image coords.
        ox1 = (x1 - pad_w) / ratio
        oy1 = (y1 - pad_h) / ratio
        ox2 = (x2 - pad_w) / ratio
        oy2 = (y2 - pad_h) / ratio
        ox1 = int(np.clip(ox1, 0, img_w - 1))
        oy1 = int(np.clip(oy1, 0, img_h - 1))
        ox2 = int(np.clip(ox2, 0, img_w - 1))
        oy2 = int(np.clip(oy2, 0, img_h - 1))

        name = CLASS_NAMES[cls_id] if 0 <= cls_id < len(CLASS_NAMES) else f"cls{cls_id}"
        color = CLASS_COLORS[cls_id] if 0 <= cls_id < len(CLASS_COLORS) else (200, 200, 200)
        thickness = 3 if cls_id == 7 else 2  # bold traffic-light boxes
        cv2.rectangle(annotated, (ox1, oy1), (ox2, oy2), color, thickness)
        label = f"{name} {score:.2f}"
        (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(annotated,
                      (ox1, max(0, oy1 - th - 6)),
                      (ox1 + tw + 4, oy1),
                      color, -1)
        cv2.putText(annotated, label, (ox1 + 2, oy1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)

        print(f"{i:>3}  {cls_id:>3}  {name:<14}  {score:6.3f}   "
              f"({ox1},{oy1},{ox2},{oy2})")

    cv2.imwrite(args.out, annotated)
    print(f"\nannotated image written to: {args.out}")


if __name__ == "__main__":
    main()
