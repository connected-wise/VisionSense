#!/usr/bin/env python3
"""
VisionSense Latency Measurement Tool

Measures transport latency and inter-message timing across all ROS2 topics
in the VisionSense pipeline to identify bottlenecks.

Usage:
    ros2 run visionconnect measure_latency
    # or
    python3 scripts/measure_latency.py

Output:
    - Per-topic transport latency (receive time - publish time)
    - Inter-message interval (time between consecutive messages)
    - End-to-end pipeline latency (camera -> GUI)
    - Statistics updated every 2 seconds
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import threading

# Try to import custom messages
try:
    from visionconnect.msg import Detect, Lanes, Signs, ADAS
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: Custom messages not found. Only measuring Image topics.")


@dataclass
class TopicStats:
    """Statistics for a single topic"""
    name: str
    msg_count: int = 0
    last_receive_time: Optional[float] = None
    last_stamp_time: Optional[float] = None

    # Transport latency (receive - stamp)
    transport_latencies: List[float] = field(default_factory=list)

    # Inter-message interval
    intervals: List[float] = field(default_factory=list)

    # Camera timestamp for pipeline tracking
    camera_stamp: Optional[float] = None

    def add_measurement(self, receive_time: float, stamp_time: float, camera_stamp: Optional[float] = None):
        """Add a new measurement"""
        self.msg_count += 1

        # Transport latency
        transport_latency = (receive_time - stamp_time) * 1000  # ms
        self.transport_latencies.append(transport_latency)

        # Inter-message interval
        if self.last_receive_time is not None:
            interval = (receive_time - self.last_receive_time) * 1000  # ms
            self.intervals.append(interval)

        self.last_receive_time = receive_time
        self.last_stamp_time = stamp_time

        if camera_stamp is not None:
            self.camera_stamp = camera_stamp

        # Keep only last 100 measurements
        if len(self.transport_latencies) > 100:
            self.transport_latencies = self.transport_latencies[-100:]
        if len(self.intervals) > 100:
            self.intervals = self.intervals[-100:]

    def get_stats(self) -> Dict:
        """Get statistics summary"""
        stats = {
            'count': self.msg_count,
            'transport_ms': self._calc_stats(self.transport_latencies),
            'interval_ms': self._calc_stats(self.intervals),
        }
        return stats

    def _calc_stats(self, values: List[float]) -> Dict:
        if not values:
            return {'min': 0, 'max': 0, 'avg': 0, 'last': 0}
        return {
            'min': min(values),
            'max': max(values),
            'avg': sum(values) / len(values),
            'last': values[-1] if values else 0
        }

    def reset(self):
        """Reset statistics"""
        self.transport_latencies.clear()
        self.intervals.clear()


class LatencyMeasurementNode(Node):
    """ROS2 node for measuring latency across VisionSense topics"""

    def __init__(self):
        super().__init__('latency_measurement')

        self.stats: Dict[str, TopicStats] = {}
        self.stats_lock = threading.Lock()

        # Track camera timestamps for end-to-end latency
        self.camera_stamps: Dict[float, float] = {}  # stamp -> receive_time

        # QoS profile matching VisionSense nodes
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to all relevant topics
        self._setup_subscriptions(qos)

        # Timer for printing statistics
        self.stats_timer = self.create_timer(2.0, self._print_stats)

        self.get_logger().info('Latency measurement node started')
        self.get_logger().info('Waiting for messages...\n')

    def _setup_subscriptions(self, qos):
        """Set up subscriptions to all VisionSense topics"""

        # Camera topics (Image messages)
        image_topics = [
            '/camera_stereo/left/image_raw',
            '/camera_stereo/right/image_raw',
            '/camera/raw',
            '/gui/fusion',
            '/stereo_depth/disparity',
        ]

        for topic in image_topics:
            self.stats[topic] = TopicStats(name=topic)
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self._image_callback(msg, t),
                qos
            )

        # Camera timestamp topic
        self.create_subscription(
            Header, '/camera_stereo/time',
            self._camera_time_callback,
            qos
        )

        # Custom message topics
        if CUSTOM_MSGS_AVAILABLE:
            # Detect messages
            detect_topic = '/detect/detections'
            self.stats[detect_topic] = TopicStats(name=detect_topic)
            self.create_subscription(
                Detect, detect_topic,
                lambda msg, t=detect_topic: self._detect_callback(msg, t),
                qos
            )

            # Lanes messages
            lanes_topic = '/lanedet/lanes'
            self.stats[lanes_topic] = TopicStats(name=lanes_topic)
            self.create_subscription(
                Lanes, lanes_topic,
                lambda msg, t=lanes_topic: self._lanes_callback(msg, t),
                qos
            )

            # Signs messages
            signs_topic = '/classify/signs'
            self.stats[signs_topic] = TopicStats(name=signs_topic)
            self.create_subscription(
                Signs, signs_topic,
                lambda msg, t=signs_topic: self._signs_callback(msg, t),
                qos
            )

            # ADAS messages
            adas_topic = '/adas/adas_alerts'
            self.stats[adas_topic] = TopicStats(name=adas_topic)
            self.create_subscription(
                ADAS, adas_topic,
                lambda msg, t=adas_topic: self._adas_callback(msg, t),
                qos
            )

    def _get_stamp_seconds(self, stamp) -> float:
        """Convert ROS2 timestamp to seconds"""
        return stamp.sec + stamp.nanosec * 1e-9

    def _camera_time_callback(self, msg: Header):
        """Track camera timestamps for end-to-end measurement"""
        stamp = self._get_stamp_seconds(msg.stamp)
        receive_time = self.get_clock().now().nanoseconds * 1e-9
        self.camera_stamps[stamp] = receive_time

        # Keep only recent stamps
        if len(self.camera_stamps) > 100:
            oldest = sorted(self.camera_stamps.keys())[:50]
            for k in oldest:
                del self.camera_stamps[k]

    def _image_callback(self, msg: Image, topic: str):
        """Callback for Image messages"""
        receive_time = self.get_clock().now().nanoseconds * 1e-9
        stamp_time = self._get_stamp_seconds(msg.header.stamp)

        with self.stats_lock:
            if topic in self.stats:
                self.stats[topic].add_measurement(receive_time, stamp_time)

    def _detect_callback(self, msg, topic: str):
        """Callback for Detect messages"""
        receive_time = self.get_clock().now().nanoseconds * 1e-9
        stamp_time = self._get_stamp_seconds(msg.image.header.stamp)

        with self.stats_lock:
            if topic in self.stats:
                self.stats[topic].add_measurement(receive_time, stamp_time)

    def _lanes_callback(self, msg, topic: str):
        """Callback for Lanes messages"""
        receive_time = self.get_clock().now().nanoseconds * 1e-9
        stamp_time = self._get_stamp_seconds(msg.rawimg.header.stamp)

        with self.stats_lock:
            if topic in self.stats:
                self.stats[topic].add_measurement(receive_time, stamp_time)

    def _signs_callback(self, msg, topic: str):
        """Callback for Signs messages"""
        receive_time = self.get_clock().now().nanoseconds * 1e-9
        # Signs may not have a header, use current time
        stamp_time = receive_time

        with self.stats_lock:
            if topic in self.stats:
                self.stats[topic].add_measurement(receive_time, stamp_time)

    def _adas_callback(self, msg, topic: str):
        """Callback for ADAS messages"""
        receive_time = self.get_clock().now().nanoseconds * 1e-9
        stamp_time = receive_time  # ADAS may not have timestamp

        with self.stats_lock:
            if topic in self.stats:
                self.stats[topic].add_measurement(receive_time, stamp_time)

    def _print_stats(self):
        """Print latency statistics"""
        print("\033[2J\033[H")  # Clear screen
        print("=" * 90)
        print("VisionSense Latency Measurement")
        print("=" * 90)
        print(f"{'Topic':<40} {'Count':>6} {'Transport (ms)':>20} {'Interval (ms)':>20}")
        print(f"{'':40} {'':>6} {'avg / last':>20} {'avg / last':>20}")
        print("-" * 90)

        with self.stats_lock:
            # Sort topics by pipeline order
            order = [
                '/camera_stereo/left/image_raw',
                '/camera_stereo/right/image_raw',
                '/camera/raw',
                '/detect/detections',
                '/lanedet/lanes',
                '/classify/signs',
                '/stereo_depth/disparity',
                '/adas/adas_alerts',
                '/gui/fusion',
            ]

            active_topics = []
            for topic in order:
                if topic in self.stats and self.stats[topic].msg_count > 0:
                    active_topics.append(topic)

            # Add any other active topics not in order
            for topic, stat in self.stats.items():
                if topic not in order and stat.msg_count > 0:
                    active_topics.append(topic)

            if not active_topics:
                print("No messages received yet...")
                print("-" * 90)
                return

            for topic in active_topics:
                stat = self.stats[topic]
                s = stat.get_stats()

                transport = s['transport_ms']
                interval = s['interval_ms']

                # Color coding for latency
                transport_str = f"{transport['avg']:6.1f} / {transport['last']:6.1f}"
                interval_str = f"{interval['avg']:6.1f} / {interval['last']:6.1f}"

                # Highlight high latency
                if transport['avg'] > 50:
                    transport_str = f"\033[91m{transport_str}\033[0m"  # Red
                elif transport['avg'] > 20:
                    transport_str = f"\033[93m{transport_str}\033[0m"  # Yellow

                print(f"{topic:<40} {stat.msg_count:>6} {transport_str:>20} {interval_str:>20}")

        print("-" * 90)
        print("\nLegend: Transport = receive_time - msg_stamp (network/processing delay)")
        print("        Interval = time between consecutive messages (should be ~33ms for 30fps)")
        print("\nColor: \033[91mRed\033[0m = >50ms, \033[93mYellow\033[0m = >20ms")
        print("\nPress Ctrl+C to exit")


def main(args=None):
    rclpy.init(args=args)

    node = LatencyMeasurementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
