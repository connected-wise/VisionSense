#!/usr/bin/env python3
"""
Show raw stereo left/right pair side-by-side.

Grabs frames directly from /dev/video1 (Arducam stereo) and splits the
3840x1200 UYVY frame into left (0..1920) and right (1920..3840) halves.
Use this to confirm which eye is failing before any ROS / CUDA processing.

IMPORTANT: stop the ROS camera_stereo node first — V4L2 will not let two
processes open /dev/video1. Use SIGINT (Ctrl-C), never `pkill -9`, per
project memory (uvcvideo wedges on hard kill).

Usage:
    python3 scripts/show_stereo_pair.py            # /dev/video1, 3840x1200
    python3 scripts/show_stereo_pair.py /dev/video1 2560 720
"""

import sys
import time
import cv2
import numpy as np

# The Arducam stereo cam is mounted upside-down on this rig (matches the
# camera_stereo node's `cuda_flip: rotate-180`). Set False to view the raw,
# unrotated sensor data.
ROTATE_180 = True


def open_capture(device: str, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {device}")

    # UYVY is what the AR0234/Arducam advertises; jetson-utils uses it too.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'UYVY'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FPS, 30)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join([chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4)])
    print(f"[info] {device}: {actual_w}x{actual_h} {fourcc} @ {cap.get(cv2.CAP_PROP_FPS):.0f}fps")
    return cap


def stats(img: np.ndarray) -> str:
    mean = float(img.mean())
    std = float(img.std())
    return f"mean={mean:5.1f} std={std:5.1f}"


def main() -> int:
    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/video1"
    width = int(sys.argv[2]) if len(sys.argv) > 2 else 3840
    height = int(sys.argv[3]) if len(sys.argv) > 3 else 1200

    cap = open_capture(device, width, height)

    cv2.namedWindow("stereo pair (L | R)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("stereo pair (L | R)", 1600, 500)

    frame_count = 0
    fail_count = 0
    t0 = time.time()
    fps = 0.0

    print("[info] press 'q' to quit, 's' to save a snapshot")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            fail_count += 1
            print(f"[warn] capture failed (#{fail_count})")
            time.sleep(0.05)
            if fail_count > 30:
                print("[error] too many capture failures, giving up")
                break
            continue

        if frame.shape[1] < width:
            print(f"[warn] got {frame.shape[1]}x{frame.shape[0]}, expected {width}x{height}")

        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        half = frame.shape[1] // 2
        left = frame[:, :half]
        right = frame[:, half:]

        # Resize each eye to a sane preview size, side-by-side.
        preview_h = 600
        scale = preview_h / left.shape[0]
        preview_w = int(left.shape[1] * scale)
        left_small = cv2.resize(left, (preview_w, preview_h))
        right_small = cv2.resize(right, (preview_w, preview_h))

        # Annotate so it's obvious which is which.
        for img, label in ((left_small, "LEFT"), (right_small, "RIGHT")):
            cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1),
                          (0, 255, 0), 2)
            cv2.putText(img, label, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0), 2)

        pair = np.hstack([left_small, right_small])

        frame_count += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            t0 = time.time()

        hud = f"{fps:5.1f} fps | L {stats(left)} | R {stats(right)}"
        cv2.putText(pair, hud, (10, pair.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        cv2.imshow("stereo pair (L | R)", pair)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            ts = int(time.time())
            cv2.imwrite(f"/tmp/stereo_left_{ts}.png", left)
            cv2.imwrite(f"/tmp/stereo_right_{ts}.png", right)
            cv2.imwrite(f"/tmp/stereo_pair_{ts}.png", frame)
            print(f"[info] saved /tmp/stereo_*_{ts}.png")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
