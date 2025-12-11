#!/usr/bin/env python3
"""
Display Driver Monitor Output

Subscribes to /driver_monitor/image and displays the annotated video
showing face detection, eye detection, and driver state.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class DriverMonitorDisplay(Node):
    def __init__(self):
        super().__init__('driver_monitor_display')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_state = "UNKNOWN"
        self.latest_alert = False
        self.frame_count = 0

        # Subscribe to driver monitor topics
        self.image_sub = self.create_subscription(
            Image,
            '/driver_monitor/image',
            self.image_callback,
            10)

        self.state_sub = self.create_subscription(
            String,
            '/driver_monitor/state',
            self.state_callback,
            10)

        self.alert_sub = self.create_subscription(
            Bool,
            '/driver_monitor/alert',
            self.alert_callback,
            10)

        self.get_logger().info('Driver Monitor Display Node Started')
        self.get_logger().info('Subscribing to /driver_monitor/image')
        self.get_logger().info('Press ESC or Q to quit')

    def image_callback(self, msg):
        """Callback for receiving annotated images"""
        try:
            self.frame_count += 1

            # Log first frame received
            if self.frame_count == 1:
                self.get_logger().info(f'First frame received! Size: {msg.width}x{msg.height}, Encoding: {msg.encoding}')

            # Log every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Received {self.frame_count} frames')

            # Convert ROS Image to OpenCV format
            # The driver monitor publishes rgb8, so convert to bgr8 for OpenCV display
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.latest_image = cv_image

            # Display the image
            if self.latest_image is not None:
                # Add additional text overlay with state and alert
                display_img = self.latest_image.copy()

                # Add alert indicator if active
                if self.latest_alert:
                    cv2.putText(display_img, "!!! ALERT !!!", (display_img.shape[1]//2 - 100, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

                cv2.imshow('Driver Monitor', display_img)
                key = cv2.waitKey(1)

                # ESC or 'q' to quit
                if key == 27 or key == ord('q'):
                    self.get_logger().info('Quit key pressed, shutting down...')
                    raise KeyboardInterrupt

        except KeyboardInterrupt:
            raise
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def state_callback(self, msg):
        """Callback for driver state updates"""
        self.latest_state = msg.data
        self.get_logger().info(f'Driver State: {self.latest_state}')

    def alert_callback(self, msg):
        """Callback for alert status"""
        if msg.data != self.latest_alert:
            self.latest_alert = msg.data
            if self.latest_alert:
                self.get_logger().warn('ALERT: Driver attention required!')
            else:
                self.get_logger().info('Alert cleared')

def main(args=None):
    rclpy.init(args=args)

    display_node = DriverMonitorDisplay()

    try:
        rclpy.spin(display_node)
    except KeyboardInterrupt:
        pass
    finally:
        display_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
