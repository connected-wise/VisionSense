#!/usr/bin/env python3
"""
Test node that publishes left/right stereo images from files.
Used to debug stereo_depth node preprocessing issues.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os


class StereoFilePublisher(Node):
    def __init__(self):
        super().__init__('stereo_file_publisher')

        # Declare parameters
        self.declare_parameter('left_image', '')
        self.declare_parameter('right_image', '')
        self.declare_parameter('rate', 1.0)  # Hz
        self.declare_parameter('encoding', 'rgb8')  # Match camera output

        # Get parameters
        left_path = self.get_parameter('left_image').value
        right_path = self.get_parameter('right_image').value
        rate = self.get_parameter('rate').value
        self.encoding = self.get_parameter('encoding').value

        # Load images
        self.get_logger().info(f'Loading left image: {left_path}')
        self.get_logger().info(f'Loading right image: {right_path}')

        self.left_img = cv2.imread(left_path)
        self.right_img = cv2.imread(right_path)

        if self.left_img is None:
            self.get_logger().error(f'Failed to load left image: {left_path}')
            raise RuntimeError(f'Cannot load {left_path}')
        if self.right_img is None:
            self.get_logger().error(f'Failed to load right image: {right_path}')
            raise RuntimeError(f'Cannot load {right_path}')

        self.get_logger().info(f'Left image shape: {self.left_img.shape}')
        self.get_logger().info(f'Right image shape: {self.right_img.shape}')
        self.get_logger().info(f'Publishing with encoding: {self.encoding}')

        # Convert BGR to RGB if needed (cv2.imread returns BGR)
        if self.encoding == 'rgb8':
            self.left_img = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2RGB)
            self.right_img = cv2.cvtColor(self.right_img, cv2.COLOR_BGR2RGB)
            self.get_logger().info('Converted images from BGR to RGB')

        # Create publishers with BEST_EFFORT QoS to match stereo_depth subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=2
        )

        self.left_pub = self.create_publisher(Image, 'left/image_raw', qos)
        self.right_pub = self.create_publisher(Image, 'right/image_raw', qos)

        # Create timer
        self.timer = self.create_timer(1.0 / rate, self.publish_images)
        self.frame_count = 0

        self.get_logger().info(f'Publishing at {rate} Hz')
        self.get_logger().info('Stereo file publisher ready')

    def publish_images(self):
        now = self.get_clock().now().to_msg()

        # Create left message
        left_msg = self.mat_to_msg(self.left_img, now, 'left_camera')

        # Create right message
        right_msg = self.mat_to_msg(self.right_img, now, 'right_camera')

        # Publish
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        self.frame_count += 1
        if self.frame_count % 5 == 0:
            self.get_logger().info(f'Published frame {self.frame_count}')

    def mat_to_msg(self, img, stamp, frame_id):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = self.encoding
        msg.is_bigendian = False
        msg.step = img.shape[1] * img.shape[2] if len(img.shape) > 2 else img.shape[1]
        msg.data = img.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)

    try:
        node = StereoFilePublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
