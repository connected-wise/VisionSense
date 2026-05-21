#!/usr/bin/env python3
"""
Stereo camera calibration from captured chessboard pairs.

Reads left_NNN.png / right_NNN.png pairs produced by capture_stereo_calib.py,
detects the chessboard in each, runs per-eye intrinsic calibration, then
stereo calibration + rectification. Writes a YAML calibration file consumable
by node_camera_stereo (for accurate CameraInfo) and any downstream rectifier
(NITROS RectifyNode, OpenCV initUndistortRectifyMap+remap, etc.).

Workflow:
    python3 scripts/capture_stereo_calib.py        # collect ~25 pairs
    python3 scripts/compute_stereo_calib.py        # this script
    → produces stereo_calib.yaml + a sample rectified pair for sanity-check

Defaults assume the same 9x6 inner-corner board / 25 mm squares as the
capture script. The board's physical square size matters for absolute scale
(baseline ends up in meters); intrinsics/distortion are scale-invariant.

A good calibration on this rig should produce:
    per-eye RMS reprojection error  < 0.5 px
    stereo RMS                      < 1.0 px
    baseline (||T||)                ~ 0.10 m (matches the measured 101.3 mm)
"""

import argparse
import os
import sys
from glob import glob

import cv2
import numpy as np


CHESSBOARD_FLAGS = (
    cv2.CALIB_CB_ADAPTIVE_THRESH
    | cv2.CALIB_CB_NORMALIZE_IMAGE
    | cv2.CALIB_CB_FILTER_QUADS
)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)


def build_object_points(pattern: tuple[int, int], square_size_m: float) -> np.ndarray:
    cols, rows = pattern
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return pts * square_size_m  # absolute scale (meters)


def detect_corners(gray: np.ndarray, pattern: tuple[int, int]):
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=CHESSBOARD_FLAGS)
    if not found:
        return None
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
    return corners


def load_pairs(image_dir: str, pattern: tuple[int, int]):
    """Return (image_size, [obj_pts], [left_corners], [right_corners], [pair_names])."""
    left_paths = sorted(glob(os.path.join(image_dir, "left_*.png")))
    if not left_paths:
        raise RuntimeError(f"no left_*.png found in {image_dir}")

    img_size = None
    left_pts, right_pts, names = [], [], []
    for lp in left_paths:
        rp = lp.replace("/left_", "/right_")
        if not os.path.exists(rp):
            print(f"  skip {os.path.basename(lp)}: no matching right image")
            continue

        l = cv2.imread(lp, cv2.IMREAD_GRAYSCALE)
        r = cv2.imread(rp, cv2.IMREAD_GRAYSCALE)
        if l is None or r is None or l.shape != r.shape:
            print(f"  skip {os.path.basename(lp)}: image load/size mismatch")
            continue
        if img_size is None:
            img_size = (l.shape[1], l.shape[0])  # OpenCV uses (w, h)
        elif (l.shape[1], l.shape[0]) != img_size:
            print(f"  skip {os.path.basename(lp)}: dims differ from {img_size}")
            continue

        lc = detect_corners(l, pattern)
        rc = detect_corners(r, pattern)
        if lc is None or rc is None:
            print(f"  skip {os.path.basename(lp)}: chessboard not found in both eyes")
            continue

        left_pts.append(lc)
        right_pts.append(rc)
        names.append(os.path.basename(lp).replace("left_", ""))

    if not left_pts:
        raise RuntimeError("no usable pairs (no chessboard detected in any)")

    print(f"\n{len(left_pts)} of {len(left_paths)} pairs usable")
    return img_size, left_pts, right_pts, names


def write_yaml(path: str, *, image_size, K1, D1, K2, D2,
               R, T, E, F, R1, R2, P1, P2, Q, roi1, roi2,
               rms_per_eye, rms_stereo, n_pairs, pattern, square_size_m):
    def mat_to_list(m):
        return np.asarray(m, dtype=np.float64).flatten().tolist()

    with open(path, "w") as f:
        f.write("# Stereo calibration produced by scripts/compute_stereo_calib.py\n")
        f.write(f"image_width: {image_size[0]}\n")
        f.write(f"image_height: {image_size[1]}\n")
        f.write(f"num_pairs_used: {n_pairs}\n")
        f.write(f"chessboard_inner_corners: [{pattern[0]}, {pattern[1]}]\n")
        f.write(f"chessboard_square_size_m: {square_size_m}\n")
        f.write(f"reprojection_rms_left: {rms_per_eye[0]:.4f}\n")
        f.write(f"reprojection_rms_right: {rms_per_eye[1]:.4f}\n")
        f.write(f"reprojection_rms_stereo: {rms_stereo:.4f}\n")
        f.write(f"baseline_m: {float(np.linalg.norm(T)):.6f}\n\n")

        def block(name, mat):
            rows, cols = np.asarray(mat).shape
            f.write(f"{name}:\n")
            f.write(f"  rows: {rows}\n  cols: {cols}\n")
            f.write(f"  data: {mat_to_list(mat)}\n\n")

        block("K_left", K1);   block("D_left", D1)
        block("K_right", K2);  block("D_right", D2)
        block("R", R);         block("T", T)
        block("E", E);         block("F", F)
        block("R1", R1);       block("R2", R2)
        block("P1", P1);       block("P2", P2)
        block("Q",  Q)
        f.write(f"valid_roi_left:  [{roi1[0]}, {roi1[1]}, {roi1[2]}, {roi1[3]}]\n")
        f.write(f"valid_roi_right: [{roi2[0]}, {roi2[1]}, {roi2[2]}, {roi2[3]}]\n")


def save_rectified_sample(image_dir, output_path,
                          K1, D1, K2, D2, R1, R2, P1, P2, image_size):
    """Rectify the first usable pair and save side-by-side with horizontal lines."""
    lefts = sorted(glob(os.path.join(image_dir, "left_*.png")))
    if not lefts:
        return
    l = cv2.imread(lefts[0])
    r = cv2.imread(lefts[0].replace("/left_", "/right_"))
    if l is None or r is None:
        return

    map1l, map2l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_16SC2)
    map1r, map2r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_16SC2)
    lr = cv2.remap(l, map1l, map2l, cv2.INTER_LINEAR)
    rr = cv2.remap(r, map1r, map2r, cv2.INTER_LINEAR)
    pair = np.hstack([lr, rr])
    # Horizontal epipolar lines — a good calibration puts the same scene point
    # on the SAME row in both rectified images.
    for y in range(0, pair.shape[0], 40):
        cv2.line(pair, (0, y), (pair.shape[1], y), (0, 255, 0), 1)
    cv2.imwrite(output_path, pair)
    print(f"rectified sample → {output_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="./stereo_calib_images",
                    help="directory containing left_NNN.png / right_NNN.png")
    ap.add_argument("--out", default="./stereo_calib.yaml",
                    help="output YAML path")
    ap.add_argument("--cols", type=int, default=7,
                    help="chessboard inner corners per row (default 7, "
                         "matches the 8x6-square board used on this rig)")
    ap.add_argument("--rows", type=int, default=5,
                    help="chessboard inner corners per column (default 5)")
    ap.add_argument("--square-mm", type=float, default=30.0,
                    help="physical chessboard square size in millimetres (default 30 mm)")
    ap.add_argument("--fix-aspect", action="store_true",
                    help="constrain fx == fy (set if you trust pixels are square)")
    ap.add_argument("--sample", default="./stereo_calib_rectified_sample.png",
                    help="path for the rectified-sample preview image")
    ap.add_argument("--max-reject-rounds", type=int, default=4,
                    help="iteratively drop worst-fit pairs and re-converge "
                         "(default 4 rounds; set 0 to disable)")
    ap.add_argument("--reject-threshold", type=float, default=1.5,
                    help="drop pairs whose per-eye reprojection error exceeds "
                         "this many pixels in either eye (default 1.5)")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    square_size_m = args.square_mm / 1000.0

    print(f"Loading pairs from {args.dir} ...")
    img_size, left_pts, right_pts, names = load_pairs(args.dir, pattern)

    obj_pts_template = build_object_points(pattern, square_size_m)
    obj_pts = [obj_pts_template] * len(left_pts)

    # ── Per-eye intrinsic calibration ────────────────────────────────────────
    print("\nPer-eye intrinsic calibration...")
    flags_intrinsic = 0
    if args.fix_aspect:
        flags_intrinsic |= cv2.CALIB_FIX_ASPECT_RATIO

    def fit_once(obj_pts, left_pts, right_pts, names):
        rms_l, K1, D1, rvecs_l, tvecs_l = cv2.calibrateCamera(
            obj_pts, left_pts, img_size, None, None, flags=flags_intrinsic)
        rms_r, K2, D2, rvecs_r, tvecs_r = cv2.calibrateCamera(
            obj_pts, right_pts, img_size, None, None, flags=flags_intrinsic)
        per_pair = []
        for i in range(len(obj_pts)):
            proj_l, _ = cv2.projectPoints(obj_pts[i], rvecs_l[i], tvecs_l[i], K1, D1)
            proj_r, _ = cv2.projectPoints(obj_pts[i], rvecs_r[i], tvecs_r[i], K2, D2)
            el = float(np.linalg.norm(left_pts[i]  - proj_l, axis=2).mean())
            er = float(np.linalg.norm(right_pts[i] - proj_r, axis=2).mean())
            per_pair.append((i, names[i], el, er, el + er))
        return rms_l, rms_r, K1, D1, K2, D2, per_pair

    rms_l, rms_r, K1, D1, K2, D2, per_pair = fit_once(obj_pts, left_pts, right_pts, names)
    print(f"  initial fit on {len(obj_pts)} pairs:  left RMS={rms_l:.4f}px  right RMS={rms_r:.4f}px")

    # ── Iterative outlier rejection ──────────────────────────────────────────
    # The pairs with the worst per-eye reprojection error are usually motion-
    # blurred frames, badly-mis-detected corners, or extreme-angle captures
    # where the distortion model doesn't fit. Dropping them and re-converging
    # typically tightens the fit dramatically without recapturing.
    for round_idx in range(1, args.max_reject_rounds + 1):
        # Drop any pair whose per-eye error exceeds the threshold in either eye.
        keep = [(i, n, el, er, tot) for (i, n, el, er, tot) in per_pair
                if el <= args.reject_threshold and er <= args.reject_threshold]
        n_drop = len(per_pair) - len(keep)
        if n_drop == 0:
            print(f"  round {round_idx}: all pairs within {args.reject_threshold:.2f} px — done")
            break
        kept_indices = sorted(idx for idx, *_ in keep)
        obj_pts    = [obj_pts[i]    for i in kept_indices]
        left_pts   = [left_pts[i]   for i in kept_indices]
        right_pts  = [right_pts[i]  for i in kept_indices]
        names      = [names[i]      for i in kept_indices]
        rms_l, rms_r, K1, D1, K2, D2, per_pair = fit_once(obj_pts, left_pts, right_pts, names)
        print(f"  round {round_idx}: dropped {n_drop}, {len(obj_pts)} remain  "
              f"left RMS={rms_l:.4f}px  right RMS={rms_r:.4f}px")
        if len(obj_pts) < 15:
            print(f"  ⚠ only {len(obj_pts)} pairs left — stopping rejection (need more data)")
            break

    print(f"\n  final per-eye RMS:  left={rms_l:.4f} px  right={rms_r:.4f} px")
    per_pair.sort(key=lambda x: -x[4])  # worst first
    print(f"  worst 5 surviving pairs:")
    for i, name, el, er, _ in per_pair[:5]:
        print(f"    [{i:3d}] {name:24s}  left={el:.3f} px  right={er:.3f} px")

    # ── Stereo calibration (extrinsics; keep intrinsics from above) ──────────
    print("\nStereo calibration...")
    flags_stereo = cv2.CALIB_FIX_INTRINSIC
    rms_stereo, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj_pts, left_pts, right_pts,
        K1, D1, K2, D2, img_size,
        flags=flags_stereo,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )
    baseline_m = float(np.linalg.norm(T))
    Tx, Ty, Tz = float(T[0]), float(T[1]), float(T[2])
    print(f"  stereo RMS: {rms_stereo:.4f} px")
    print(f"  T (mm):     Tx={Tx*1000:.2f}  Ty={Ty*1000:.2f}  Tz={Tz*1000:.2f}   "
          f"||T||={baseline_m*1000:.2f}")
    print(f"  baseline:   {baseline_m * 1000.0:.2f} mm  (compare to your physical measurement)")
    # If |Ty|+|Tz| is large relative to |Tx|, the two eyes aren't physically
    # coplanar/parallel and rectification will work harder. For a sane stereo
    # rig: |Tx| should dominate by 10×+.
    if abs(Tx) < 5 * max(abs(Ty), abs(Tz)):
        print(f"  ⚠ T has large non-Tx components — eyes may be misaligned in Y/Z, "
              f"or scale is off (check --square-mm)")

    # ── Rectification ────────────────────────────────────────────────────────
    print("\nStereo rectification...")
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, img_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0,
    )
    fx_rect = float(P1[0, 0])
    print(f"  rectified fx (P1[0,0]): {fx_rect:.2f} px")
    print(f"  P2[0,3] (= -fx*baseline): {float(P2[0,3]):.2f} px·m")

    # ── Write YAML ───────────────────────────────────────────────────────────
    write_yaml(
        args.out,
        image_size=img_size,
        K1=K1, D1=D1, K2=K2, D2=D2,
        R=R, T=T, E=E, F=F,
        R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
        roi1=roi1, roi2=roi2,
        rms_per_eye=(rms_l, rms_r), rms_stereo=rms_stereo,
        n_pairs=len(left_pts),
        pattern=pattern, square_size_m=square_size_m,
    )
    print(f"\nwrote {args.out}")

    # ── Sanity check: render one rectified pair with epipolar lines ─────────
    save_rectified_sample(args.dir, args.sample,
                          K1, D1, K2, D2, R1, R2, P1, P2, img_size)

    # Quality hints
    print()
    if rms_l > 0.5 or rms_r > 0.5:
        print("  ⚠ per-eye RMS > 0.5 px — capture more / better-spread pairs and re-run")
    if rms_stereo > 1.0:
        print("  ⚠ stereo RMS > 1.0 px — rectification will be loose; recapture")
    if abs(baseline_m * 1000.0 - 101.3) > 5.0:
        print(f"  ⚠ baseline ({baseline_m*1000:.1f} mm) differs from expected 101.3 mm "
              f"by more than 5 mm — check chessboard square size and pose diversity")


if __name__ == "__main__":
    main()
