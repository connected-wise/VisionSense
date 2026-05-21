#!/usr/bin/env python3
"""
Translate a stereo calibration YAML produced at one image size to a new image
size, when the new size comes from a known per-eye crop of the old image.

Use case: you calibrated at the camera's old output dims (e.g. 1440×900 per
eye), then shrunk camera_stereo's output (e.g. to 1280×640) for performance.
Recalibrating from scratch costs ~20 minutes; this script shifts K's principal
point by the crop offset (D, R, T unchanged — intrinsic distortion is in
normalised coords and extrinsics are size-independent), then re-runs
cv2.stereoRectify with the new image_size to get fresh R1/R2/P1/P2/Q.

Crop convention matches src/cuda/stereo_rotate.cu's inner-edge logic AFTER the
per-eye 180° rotation. node_camera_stereo no longer applies the L/R buffer swap
(removed after the physical sensor rearrangement), so each topic carries the
RAW sensor half of the same index:

  Old image (1440×900):
    topic-LEFT  is sensor cols [480..1919]  rotated  →  image col i ↔ sensor col 1919-i
    topic-RIGHT is sensor cols [1920..3359] rotated  →  image col i ↔ sensor col 3359-i
  New image (1280×640) — inner-edge crop tightens (drop outer cols only):
    topic-LEFT  is sensor cols [640..1919]  rotated  →  image col i ↔ sensor col 1919-i
    topic-RIGHT is sensor cols [1920..3199] rotated  →  image col i ↔ sensor col 3199-i

  Mapping NEW col → OLD col:
    topic-LEFT  : new col j ↔ old col (j + 0)     (drop rightmost 160 cols of old)
    topic-RIGHT : new col j ↔ old col (j + 160)   (drop leftmost 160 cols of old)
    both eyes   : new row k ↔ old row (k + 130)   (drop top 130 rows of old)

  → K shift per eye:
       LEFT  : cx -=   0, cy -= 130
       RIGHT : cx -= 160, cy -= 130
"""

import argparse
import os
import sys

import numpy as np
import cv2
import yaml


# Crop offsets in OLD image pixel coordinates. Asymmetric in X by the inner-edge
# crop convention (no buffer swap). Override on CLI if you change crop dims again.
DEFAULT_OFFSETS = {
    "left":  (0,   130),   # (dx, dy) — left topic image's NEW (0,0) is OLD (  0, 130)
    "right": (160, 130),   # right topic image's NEW (0,0) is OLD (160, 130)
}


def load_matrix(node):
    rows = int(node["rows"])
    cols = int(node["cols"])
    data = np.asarray(node["data"], dtype=np.float64)
    if data.size != rows * cols:
        raise ValueError(f"size mismatch: rows={rows} cols={cols} data={data.size}")
    return data.reshape(rows, cols)


def matrix_to_node(m):
    m = np.asarray(m, dtype=np.float64)
    return {"rows": int(m.shape[0]), "cols": int(m.shape[1] if m.ndim > 1 else 1),
            "data": m.flatten().tolist()}


def write_yaml(path, payload):
    with open(path, "w") as f:
        for k, v in payload.items():
            if isinstance(v, dict):
                f.write(f"{k}:\n")
                f.write(f"  rows: {v['rows']}\n")
                f.write(f"  cols: {v['cols']}\n")
                f.write(f"  data: {v['data']}\n\n")
            elif isinstance(v, list):
                f.write(f"{k}: {v}\n")
            else:
                f.write(f"{k}: {v}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in",  dest="src", required=True,
                    help="source calibration YAML (the old, larger image-size one)")
    ap.add_argument("--out", dest="dst", required=True,
                    help="output translated calibration YAML")
    ap.add_argument("--new-width",  type=int, required=True,
                    help="new image width per eye after crop")
    ap.add_argument("--new-height", type=int, required=True,
                    help="new image height per eye after crop")
    ap.add_argument("--left-dx",  type=int, default=DEFAULT_OFFSETS["left"][0])
    ap.add_argument("--left-dy",  type=int, default=DEFAULT_OFFSETS["left"][1])
    ap.add_argument("--right-dx", type=int, default=DEFAULT_OFFSETS["right"][0])
    ap.add_argument("--right-dy", type=int, default=DEFAULT_OFFSETS["right"][1])
    args = ap.parse_args()

    with open(args.src) as f:
        src = yaml.safe_load(f)

    old_w = int(src["image_width"])
    old_h = int(src["image_height"])
    new_w = args.new_width
    new_h = args.new_height
    print(f"translating {os.path.basename(args.src)}: {old_w}x{old_h} → {new_w}x{new_h}")
    print(f"  left  offset: dx={args.left_dx}, dy={args.left_dy}")
    print(f"  right offset: dx={args.right_dx}, dy={args.right_dy}")

    # Sanity: new + offset must fit inside old image bounds.
    for side, (dx, dy) in (("left", (args.left_dx, args.left_dy)),
                            ("right", (args.right_dx, args.right_dy))):
        if dx + new_w > old_w or dy + new_h > old_h:
            sys.exit(f"ERROR: {side} crop ({dx}+{new_w}, {dy}+{new_h}) exceeds old image ({old_w}x{old_h})")

    K1 = load_matrix(src["K_left"])
    D1 = np.asarray(src["D_left"]["data"], dtype=np.float64)
    K2 = load_matrix(src["K_right"])
    D2 = np.asarray(src["D_right"]["data"], dtype=np.float64)
    R  = load_matrix(src["R"])
    T  = load_matrix(src["T"]).reshape(3, 1) if load_matrix(src["T"]).ndim == 2 else \
         np.asarray(src["T"]["data"], dtype=np.float64).reshape(3, 1)

    # Shift principal point by the crop offset. Distortion stays put: D is in
    # normalised image coordinates relative to the optical centre, so the
    # numbers don't change when you crop pixels around that centre.
    K1n = K1.copy()
    K1n[0, 2] -= args.left_dx
    K1n[1, 2] -= args.left_dy

    K2n = K2.copy()
    K2n[0, 2] -= args.right_dx
    K2n[1, 2] -= args.right_dy

    image_size = (new_w, new_h)

    print(f"  original  K_left cx,cy = {K1[0,2]:.2f}, {K1[1,2]:.2f}")
    print(f"  translated K_left cx,cy = {K1n[0,2]:.2f}, {K1n[1,2]:.2f}")
    print(f"  original  K_right cx,cy = {K2[0,2]:.2f}, {K2[1,2]:.2f}")
    print(f"  translated K_right cx,cy = {K2n[0,2]:.2f}, {K2n[1,2]:.2f}")

    # Re-run stereoRectify for the new image_size + adjusted K. R/T unchanged.
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1n, D1, K2n, D2, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0,
    )

    print(f"  new fx_rect (P1[0,0]) = {P1[0,0]:.2f}")
    print(f"  new P2[0,3] (-fx·b)   = {P2[0,3]:.2f}   "
          f"baseline = {-P2[0,3]/P1[0,0]*1000:.2f} mm")

    payload = {}
    payload["# Translated from"] = f"'{os.path.basename(args.src)}' ({old_w}x{old_h}) → ({new_w}x{new_h})"
    payload["image_width"]  = new_w
    payload["image_height"] = new_h
    for k in ("num_pairs_used", "chessboard_inner_corners",
              "chessboard_square_size_m",
              "reprojection_rms_left", "reprojection_rms_right",
              "reprojection_rms_stereo"):
        if k in src:
            payload[k] = src[k]
    payload["baseline_m"] = float(np.linalg.norm(T))

    payload["K_left"]  = matrix_to_node(K1n)
    payload["D_left"]  = matrix_to_node(D1.reshape(1, -1))
    payload["K_right"] = matrix_to_node(K2n)
    payload["D_right"] = matrix_to_node(D2.reshape(1, -1))
    payload["R"]       = matrix_to_node(R)
    payload["T"]       = matrix_to_node(T)
    # E,F are size-independent and downstream typically doesn't use them after
    # translation; copy through if present.
    for k in ("E", "F"):
        if k in src:
            payload[k] = src[k]
    payload["R1"] = matrix_to_node(R1)
    payload["R2"] = matrix_to_node(R2)
    payload["P1"] = matrix_to_node(P1)
    payload["P2"] = matrix_to_node(P2)
    payload["Q"]  = matrix_to_node(Q)
    payload["valid_roi_left"]  = [int(x) for x in roi1]
    payload["valid_roi_right"] = [int(x) for x in roi2]

    write_yaml(args.dst, payload)
    print(f"\nwrote {args.dst}")


if __name__ == "__main__":
    main()
