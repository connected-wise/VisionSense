#!/usr/bin/env python3
"""
Simple Display Node with Latency Overlay

Displays camera images with latency measurement overlaid on the image.
This is useful for visual verification of camera-to-display latency.

Usage:
    python3 scripts/simple_display.py [topic]

    topic: Camera topic to subscribe to (default: /camera_stereo/left/image_raw)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
import time


class SimpleDisplayNode(Node):
    """Simple image display with latency measurement"""

    def __init__(self, topic: str):
        super().__init__('simple_display')

        self.topic = topic
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0
        self.latency_ms = 0.0
        self.interval_ms = 0.0
        self.last_receive_time = None

        # QoS profile - match camera node
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos
        )

        self.get_logger().info(f'Subscribing to {topic}')
        self.get_logger().info('Press Q to quit')

        cv2.namedWindow('Latency Test', cv2.WINDOW_NORMAL)

    def image_callback(self, msg: Image):
        """Process and display image with latency overlay"""
        receive_time = time.time()

        # Calculate latency
        stamp_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latency_ms = (receive_time - stamp_time) * 1000

        # Calculate interval
        if self.last_receive_time is not None:
            self.interval_ms = (receive_time - self.last_receive_time) * 1000
        self.last_receive_time = receive_time

        # Calculate FPS
        self.frame_count += 1
        elapsed = receive_time - self.last_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = receive_time

        # Convert ROS image to OpenCV
        try:
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'mono8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # Resize for display if needed
        display_h = 720
        scale = display_h / img.shape[0]
        display_w = int(img.shape[1] * scale)
        img = cv2.resize(img, (display_w, display_h))

        # Draw latency overlay
        self._draw_overlay(img)

        # Display
        cv2.imshow('Latency Test', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit requested')
            rclpy.shutdown()

    def _draw_overlay(self, img):
        """Draw latency information on image"""
        # Semi-transparent background
        overlay = img.copy()
        cv2.rectangle(overlay, (10, 10), (400, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

        # Text
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 255, 0)  # Green
        if self.latency_ms > 50:
            color = (0, 0, 255)  # Red
        elif self.latency_ms > 20:
            color = (0, 255, 255)  # Yellow

        cv2.putText(img, f'Topic: {self.topic}', (20, 35),
                    font, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f'Transport Latency: {self.latency_ms:.1f} ms', (20, 60),
                    font, 0.6, color, 2)
        cv2.putText(img, f'Frame Interval: {self.interval_ms:.1f} ms', (20, 90),
                    font, 0.6, (255, 255, 255), 1)
        cv2.putText(img, f'Display FPS: {self.fps:.1f}', (20, 120),
                    font, 0.6, (255, 255, 255), 1)


def main(args=None):
    rclpy.init(args=args)

    # Get topic from command line or use default
    topic = '/camera_stereo/left/image_raw'
    if len(sys.argv) > 1:
        topic = sys.argv[1]

    node = SimpleDisplayNode(topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
