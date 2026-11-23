#!/usr/bin/env python3
"""
Check that config/stereo_calib.yaml still fits the stereo camera.

Run it while VisionSense (or just camera_stereo) is running, with the cameras
looking at a textured scene. It takes one synchronized pair from
/camera_stereo/{left,right}/image_raw, rectifies it with the calibration, and
matches features between the two eyes. In a correctly rectified pair a point
sits on the same row in both images, so the vertical offset of the matches
tells how good the calibration is:

    median within about +-0.5 px, mean |offset| under 1 px   calibration fits
    several pixels or more                                    recalibrate

A wrong calibration does not only make distances wrong: the stereo network
searches along rows, so it loses the match altogether.

Usage:
    python3 scripts/check_stereo_calib.py [--calib config/stereo_calib.yaml]
                                          [--save rectified.jpg]
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def load_calib(path):
    with open(path) as fp:
        y = yaml.safe_load(fp)

    def mat(key):
        return np.array(y[key]["data"], dtype=np.float64).reshape(y[key]["rows"], y[key]["cols"])

    return y, {k: mat(k) for k in ("K_left", "D_left", "K_right", "D_right", "R1", "R2", "P1", "P2")}


def capture_pair(timeout_s=30.0):
    """One left/right pair with the same header stamp, as grayscale."""
    rclpy.init()
    node = Node("check_stereo_calib")
    pending = {"left": {}, "right": {}}
    pairs = []

    def callback(side):
        def on_image(msg):
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            pending[side][key] = cv2.cvtColor(img, code)
            other = "right" if side == "left" else "left"
            if key in pending[other]:
                pairs.append((pending["left"].pop(key), pending["right"].pop(key)))
            for frames in pending.values():
                while len(frames) > 6:
                    frames.pop(next(iter(frames)))
        return on_image

    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Image, "/camera_stereo/left/image_raw", callback("left"), qos)
    node.create_subscription(Image, "/camera_stereo/right/image_raw", callback("right"), qos)
    start = time.time()
    # Skip the first few pairs: auto-exposure is still settling
    while len(pairs) < 10 and time.time() - start < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    return pairs[-1] if pairs else (None, None)


def match(left, right):
    """Feature matches between the two eyes, outliers removed with RANSAC."""
    sift = cv2.SIFT_create(4000)
    kp_l, des_l = sift.detectAndCompute(left, None)
    kp_r, des_r = sift.detectAndCompute(right, None)
    if des_l is None or des_r is None:
        return np.empty((0, 2)), np.empty((0, 2))
    knn = cv2.BFMatcher().knnMatch(des_l, des_r, k=2)
    good = [m[0] for m in knn if len(m) == 2 and m[0].distance < 0.7 * m[1].distance]
    pts_l = np.float32([kp_l[m.queryIdx].pt for m in good]).reshape(-1, 2)
    pts_r = np.float32([kp_r[m.trainIdx].pt for m in good]).reshape(-1, 2)
    if len(pts_l) >= 30:
        _, mask = cv2.findFundamentalMat(pts_l, pts_r, cv2.FM_RANSAC, 1.5, 0.999)
        if mask is not None:
            keep = mask.ravel() == 1
            pts_l, pts_r = pts_l[keep], pts_r[keep]
    return pts_l, pts_r


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=os.path.join(here, "..", "config", "stereo_calib.yaml"),
                    help="calibration YAML (default: config/stereo_calib.yaml)")
    ap.add_argument("--save", default="",
                    help="write the rectified pair, side by side with row lines, to this file")
    args = ap.parse_args()

    y, c = load_calib(args.calib)
    width, height = y["image_width"], y["image_height"]
    baseline_mm = -c["P2"][0, 3] / c["P1"][0, 0] * 1000.0
    print(f"calibration: {width}x{height}, baseline {baseline_mm:.1f} mm, fx_rect {c['P1'][0, 0]:.1f} px")

    left, right = capture_pair()
    if left is None:
        print("no stereo pair received: is camera_stereo running?")
        return 2
    if (left.shape[1], left.shape[0]) != (width, height):
        print(f"camera frames are {left.shape[1]}x{left.shape[0]}, calibration is for {width}x{height}")
        return 2

    map_l = cv2.initUndistortRectifyMap(c["K_left"], c["D_left"], c["R1"], c["P1"], (width, height), cv2.CV_32FC1)
    map_r = cv2.initUndistortRectifyMap(c["K_right"], c["D_right"], c["R2"], c["P2"], (width, height), cv2.CV_32FC1)
    rect_l = cv2.remap(left, map_l[0], map_l[1], cv2.INTER_LINEAR)
    rect_r = cv2.remap(right, map_r[0], map_r[1], cv2.INTER_LINEAR)

    for name, (a, b) in (("as captured", (left, right)), ("rectified", (rect_l, rect_r))):
        pts_l, pts_r = match(a, b)
        if len(pts_l) < 20:
            print(f"{name:12s} only {len(pts_l)} matches: point the cameras at a textured scene")
            continue
        dy = pts_l[:, 1] - pts_r[:, 1]
        dx = pts_l[:, 0] - pts_r[:, 0]
        print(f"{name:12s} {len(dy):4d} matches | vertical offset: median {np.median(dy):+7.2f} px, "
              f"mean |.| {np.mean(np.abs(dy)):6.2f} px | disparity: median {np.median(dx):6.1f} px, "
              f"negative {100 * np.mean(dx < 0):.0f}%")
        if name == "rectified":
            result = (abs(np.median(dy)), np.mean(np.abs(dy)), np.mean(dx < 0))

    if args.save:
        both = cv2.cvtColor(np.hstack([rect_l, rect_r]), cv2.COLOR_GRAY2BGR)
        for row in range(40, height, 60):
            cv2.line(both, (0, row), (2 * width, row), (0, 255, 0), 1)
        cv2.imwrite(args.save, both)
        print(f"rectified pair saved to {args.save}")

    try:
        median_dy, mean_abs_dy, negative = result
    except NameError:
        return 2
    if median_dy <= 0.5 and mean_abs_dy <= 1.0 and negative < 0.05:
        print("OK: the calibration fits this camera")
        return 0
    print("RECALIBRATE: the rectified rows do not line up "
          "(scripts/capture_stereo_calib.py, then scripts/compute_stereo_calib.py)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
