#!/usr/bin/env python3
"""
Display node for stereo disparity images.
Shows the disparity output with colormap visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np


class DisparityDisplay(Node):
    def __init__(self):
        super().__init__('disparity_display')

        # BEST_EFFORT QoS to match publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=2
        )

        self.sub = self.create_subscription(
            Image,
            '/stereo_depth/disparity',
            self.disparity_callback,
            qos
        )

        self.frame_count = 0
        self.get_logger().info('Disparity display ready, waiting for images...')

    def disparity_callback(self, msg):
        self.frame_count += 1

        # Convert to numpy array
        if msg.encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        elif msg.encoding == '32FC1':
            img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            img = (np.clip(img / 192.0, 0, 1) * 255).astype(np.uint8)
        else:
            self.get_logger().warn(f'Unknown encoding: {msg.encoding}')
            return

        # Apply colormap
        colored = cv2.applyColorMap(img, cv2.COLORMAP_JET)

        # Add frame info
        cv2.putText(colored, f'Frame: {self.frame_count}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(colored, f'Size: {msg.width}x{msg.height}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Display
        cv2.imshow('Disparity Output', colored)

        # Also save first frame for comparison
        if self.frame_count == 1:
            cv2.imwrite('/home/jetson/VisionSense/testing/disparity_ros_output.png', colored)
            cv2.imwrite('/home/jetson/VisionSense/testing/disparity_ros_gray.png', img)
            self.get_logger().info('Saved first frame to testing/disparity_ros_output.png')

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            self.get_logger().info('ESC pressed, shutting down')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DisparityDisplay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
