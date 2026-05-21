#!/usr/bin/env python3
"""
Standalone stereo-camera calibration capture (no ROS).

Opens /dev/video1 directly via OpenCV+V4L2, applies the SAME pipeline as
node_camera_stereo so the saved frames are byte-equivalent to the published
/camera_stereo/{left,right}/image_raw topics:

  1. Capture 3840×1200 UYVY (Arducam stereo, two 1920×1200 eyes side-by-side).
  2. Per-eye crop:
       inner-edge horizontal — 480 px removed from each eye's outer side
       (left  eye keeps sensor cols [480 .. 1919], 1440 wide)
       (right eye keeps sensor cols [1920 .. 3359], 1440 wide)
       symmetric vertical — 150 px trimmed top + 150 px trimmed bottom
       (keep rows [150 .. 1049], 900 tall)
  3. Per-eye 180° rotation (matches cuda_flip=rotate-180; camera mounted
     upside-down on this rig).
  4. No L/R buffer swap — saved left_*.png comes from sensor cols [480..1919]
     (the kernel's leftOut), matching /camera_stereo/left/image_raw after the
     physical sensor swap. Pass --swap to re-enable the old upside-down
     compensation behavior.

Press SPACE to save a synchronized pair (one capture = one pair); only saves
when BOTH eyes detect the full chessboard. Press Q/ESC to quit. Re-runs
continue indexing from the last saved pair.

REQUIRED: the visionconnect camera_stereo node must NOT be running — only one
process can hold /dev/video1. SIGINT (Ctrl-C) the ROS launch first; never
SIGKILL the camera node (V4L2 wedges until reboot).

Defaults:
    pattern: 7x5 inner corners (8x6-square board — matches the board this rig
             was previously calibrated with; override with --cols/--rows for a
             different board).
    output : ./stereo_calib_images/
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np


# Mirror node_camera_stereo geometry. Change here AND in stereo_rotate.cu's
# STEREO_OUT_W / STEREO_OUT_H / STEREO_CROP_MARGIN_* if you ever retune.
RAW_W, RAW_H        = 3840, 1200    # capture from /dev/video1
EYE_W, EYE_H        = 1440, 900     # per-eye output size
CROP_X_OUTER        = 480           # = (1920 - EYE_W); removed from each eye's outer side
CROP_Y              = 150           # = (RAW_H - EYE_H) / 2; symmetric top/bottom

# Sensor regions in the raw frame (cols).
SENSOR_LEFT_START  = CROP_X_OUTER                   # 480
SENSOR_RIGHT_START = RAW_W - CROP_X_OUTER - EYE_W   # 1920


def open_capture(device: str, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"could not open {device} (is camera_stereo holding it?)")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'UYVY'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FPS, 30)

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    cc_str = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))
    print(f"[info] {device}: {aw}x{ah} {cc_str}", flush=True)
    if (aw, ah) != (width, height):
        raise RuntimeError(f"requested {width}x{height} but got {aw}x{ah}")
    return cap


def split_and_process(frame: np.ndarray, rotate_180: bool, swap_lr: bool):
    """Replicate node_camera_stereo's crop + rotate + L/R-swap pipeline.

    Returns (left_image, right_image) where each is EYE_H × EYE_W × 3 BGR
    and matches /camera_stereo/{left,right}/image_raw respectively.
    """
    sensor_left  = frame[CROP_Y:CROP_Y + EYE_H,
                         SENSOR_LEFT_START :SENSOR_LEFT_START  + EYE_W]
    sensor_right = frame[CROP_Y:CROP_Y + EYE_H,
                         SENSOR_RIGHT_START:SENSOR_RIGHT_START + EYE_W]

    if rotate_180:
        sensor_left  = cv2.rotate(sensor_left,  cv2.ROTATE_180)
        sensor_right = cv2.rotate(sensor_right, cv2.ROTATE_180)

    if swap_lr:
        # Legacy upside-down compensation: topic-left ← sensor-right
        # cropped+rotated. Off by default — the live node_camera_stereo no
        # longer does this swap after the physical sensor rearrangement.
        return sensor_right.copy(), sensor_left.copy()
    return sensor_left.copy(), sensor_right.copy()


def detect_corners_fast(bgr: np.ndarray, pattern: tuple[int, int]):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
             | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
    return corners if found else None


def annotate(img: np.ndarray, label: str, pattern: tuple[int, int]) -> tuple[np.ndarray, bool]:
    out = img.copy()
    corners = detect_corners_fast(img, pattern)
    ok = corners is not None
    if ok:
        cv2.drawChessboardCorners(out, pattern, corners, True)
    color = (0, 255, 0) if ok else (0, 0, 255)
    cv2.rectangle(out, (0, 0), (out.shape[1] - 1, out.shape[0] - 1), color, 4)
    cv2.putText(out, f"{label}  {'CORNERS OK' if ok else 'no corners'}",
                (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    return out, ok


def fit_width(img: np.ndarray, max_w: int) -> np.ndarray:
    if img.shape[1] <= max_w:
        return img
    s = max_w / img.shape[1]
    return cv2.resize(img, (max_w, int(img.shape[0] * s)), interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video1",
                    help="V4L2 device path (default /dev/video1)")
    ap.add_argument("--out", default="./stereo_calib_images",
                    help="directory to save captured pairs")
    ap.add_argument("--cols", type=int, default=7,
                    help="chessboard inner corners per row (default 7, "
                         "matches the 8x6-square board used previously)")
    ap.add_argument("--rows", type=int, default=5,
                    help="chessboard inner corners per column (default 5)")
    ap.add_argument("--display-width", type=int, default=800,
                    help="max width per eye in the preview window")
    ap.add_argument("--no-rotate", action="store_true",
                    help="skip the per-eye 180° rotation (camera mounted upright)")
    ap.add_argument("--swap", action="store_true",
                    help="enable legacy L/R buffer swap (only if node_camera_stereo "
                         "still applies the upside-down-mount swap; off by default "
                         "after the physical sensor rearrangement)")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    rotate_180 = not args.no_rotate
    swap_lr    = args.swap

    os.makedirs(args.out, exist_ok=True)
    existing = sorted([f for f in os.listdir(args.out) if f.startswith("left_")])
    index = len(existing)
    if index:
        print(f"[info] resuming from index {index} ({index} existing pair(s) in {args.out})")

    cap = open_capture(args.device, RAW_W, RAW_H)

    win = "calib capture — L | R   (SPACE = save, Q/ESC = quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, args.display_width * 2, args.display_width)

    print(f"[info] pattern: {pattern[0]}x{pattern[1]} inner corners, "
          f"rotate_180={rotate_180}, swap_lr={swap_lr}")
    print(f"[info] press SPACE to save (only when both eyes detect the board), "
          f"Q/ESC to quit")

    fail_count = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                fail_count += 1
                if fail_count > 30:
                    print("[error] too many capture failures, giving up")
                    break
                time.sleep(0.03)
                continue
            fail_count = 0

            left_img, right_img = split_and_process(frame, rotate_180, swap_lr)

            l_ann, l_ok = annotate(left_img,  "LEFT",  pattern)
            r_ann, r_ok = annotate(right_img, "RIGHT", pattern)

            pair = np.hstack([l_ann, r_ann])
            pair = fit_width(pair, args.display_width * 2)
            footer = np.zeros((44, pair.shape[1], 3), dtype=np.uint8)
            status = ("ready (SPACE to save)" if (l_ok and r_ok)
                      else "need both eyes to see the board")
            cv2.putText(footer, f"saved: {index}    {status}",
                        (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if (l_ok and r_ok) else (0, 200, 255), 2)
            cv2.imshow(win, np.vstack([pair, footer]))

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                if not (l_ok and r_ok):
                    print("[skip] chessboard not found in BOTH eyes — pair not saved")
                    continue
                lp = os.path.join(args.out, f"left_{index:03d}.png")
                rp = os.path.join(args.out, f"right_{index:03d}.png")
                cv2.imwrite(lp, left_img)
                cv2.imwrite(rp, right_img)
                print(f"[save] pair #{index} → {lp} + {rp}")
                index += 1
            elif key in (ord('q'), 27):  # q or ESC
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"[info] done; {index} total pair(s) in {args.out}")


if __name__ == "__main__":
    main()
