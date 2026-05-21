#!/usr/bin/env python3
# depth_colorize.py
#
# Subscribes to a 32FC1 depth image and republishes a BGR8 TURBO-colormap
# visualization on `depth_color_out`. Used by stereo_depth_ess.launch.py to
# replace the BGR8 `depth_color` topic that the FFS C++ stereo_depth node
# previously produced in-process.
#
# Auto-ranged per frame (matches FFS behavior), so dark blue == near, red ==
# far. Pixels at exactly 0 m (ESS no-data) end up at the colormap floor.

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)


class DepthColorize(Node):
    def __init__(self):
        super().__init__('depth_colorize')
        self.sub = self.create_subscription(
            Image, 'depth_in', self._on_depth, SENSOR_QOS)
        self.pub = self.create_publisher(
            Image, 'depth_color_out', SENSOR_QOS)

    def _on_depth(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn(
                f"Expected 32FC1 depth, got {msg.encoding}; skipping",
                throttle_duration_sec=5.0,
            )
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)

        # Replace inf/NaN with 0 so they collapse to the colormap floor
        # rather than blowing up the auto-range.
        finite = np.isfinite(depth)
        valid = finite & (depth > 0.0)
        if not valid.any():
            return

        dmin = float(depth[valid].min())
        dmax = float(depth[valid].max())
        if dmax <= dmin:
            return

        norm = np.zeros_like(depth, dtype=np.uint8)
        scaled = (depth - dmin) * (255.0 / (dmax - dmin))
        np.clip(scaled, 0, 255, out=scaled)
        norm[valid] = scaled[valid].astype(np.uint8)

        bgr = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width  = msg.width
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = msg.width * 3
        out.data = bgr.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthColorize()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
