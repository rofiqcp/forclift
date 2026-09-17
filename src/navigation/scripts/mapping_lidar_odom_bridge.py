#!/usr/bin/python3
"""LiDAR-only scan gate for BAB 4.2 mapping.

This keeps the historical executable/topic names for GUI compatibility, but
uses NO odometry and NO IMU.  It only forwards fresh self-masked LiDAR scans
while a mapping session is enabled.  Robot pose is supplied separately by the
LiDAR-only SLAM engine on /mapping/pose.
"""
import copy
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool


class MappingLidarOnlyGate(Node):
    def __init__(self):
        super().__init__('mapping_lidar_odom_bridge')
        self.declare_parameter('source_scan_topic', '/mapping/scan_filtered')
        self.declare_parameter('output_scan_topic', '/mapping/scan_nav')
        self.declare_parameter('session_control_enabled', True)
        self.declare_parameter('initial_session_enabled', False)
        self.source_scan = str(self.get_parameter('source_scan_topic').value)
        self.output_scan = str(self.get_parameter('output_scan_topic').value)
        self.session_control_enabled = bool(self.get_parameter('session_control_enabled').value)
        self.session_enabled = bool(self.get_parameter('initial_session_enabled').value)
        self.scans = 0
        self.last_scan_wall = 0.0
        self.rate_hz = 0.0

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        session_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_pub = self.create_publisher(LaserScan, self.output_scan, scan_qos)
        self.status_pub = self.create_publisher(String, '/mapping/odom_status', 10)
        self.create_subscription(LaserScan, self.source_scan, self._scan_cb, scan_qos)
        if self.session_control_enabled:
            self.create_subscription(Bool, '/mapping/session_enabled', self._session_cb, session_qos)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f'Mapping gate READY LiDAR ONLY: {self.source_scan} -> {self.output_scan}; '
            f'odometry=false imu=false session_control={self.session_control_enabled} initial={self.session_enabled}')

    def _session_cb(self, msg):
        enabled = bool(msg.data)
        if enabled == self.session_enabled:
            return
        self.session_enabled = enabled
        self.scans = 0
        self.last_scan_wall = 0.0
        self.rate_hz = 0.0
        self.get_logger().info(
            f"Mapping scan gate {'ENABLED' if enabled else 'STANDBY'}; sensor=lidar_only")
        self._publish_status()

    def _scan_cb(self, scan):
        if not self.session_enabled:
            return
        now_wall = time.monotonic()
        if self.last_scan_wall > 0.0:
            dt = now_wall - self.last_scan_wall
            if dt > 1.0e-4:
                instant = 1.0 / dt
                self.rate_hz = instant if self.rate_hz <= 0.0 else 0.2 * instant + 0.8 * self.rate_hz
        self.last_scan_wall = now_wall
        out = copy.deepcopy(scan)
        self.scan_pub.publish(out)
        self.scans += 1

    def _publish_status(self):
        payload = {
            'ready': True,
            'session_enabled': self.session_enabled,
            'pairs': self.scans,
            'scans': self.scans,
            'scan_rate_hz': round(float(self.rate_hz), 3),
            'unmatched_odom': 0,
            'unmatched_imu': 0,
            'odometry_used': False,
            'imu_used': False,
            'sensor_source': 'lidar_only',
            'translation_source': 'lidar_scan_matching',
            'rotation_source': 'lidar_scan_matching',
            'queue_mode': 'latest_scan_only',
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingLidarOnlyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
