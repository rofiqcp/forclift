#!/usr/bin/python3
"""Localization sensor timestamp/jitter diagnostics.

This node is observational only. It never changes motion permission. It measures
header-stamp age, source-stamp period, receive period and jitter for the sensors
that feed localization so clock/transport skew can be quantified in field logs.
"""
from __future__ import annotations

from collections import deque
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String


class Series:
    def __init__(self, window=100):
        self.ages = deque(maxlen=window)
        self.stamp_dts = deque(maxlen=window)
        self.recv_dts = deque(maxlen=window)
        self.last_stamp = None
        self.last_recv = None
        self.count = 0

    def add(self, stamp_sec: float, now_ros: float, now_wall: float):
        self.count += 1
        age = now_ros - stamp_sec
        if math.isfinite(age):
            self.ages.append(age)
        if self.last_stamp is not None:
            dt = stamp_sec - self.last_stamp
            if 0.0 < dt < 10.0:
                self.stamp_dts.append(dt)
        if self.last_recv is not None:
            dt = now_wall - self.last_recv
            if 0.0 < dt < 10.0:
                self.recv_dts.append(dt)
        self.last_stamp = stamp_sec
        self.last_recv = now_wall

    @staticmethod
    def _stats(values):
        if not values:
            return None
        vals = list(values)
        mean = statistics.fmean(vals)
        return {
            'mean': mean,
            'min': min(vals),
            'max': max(vals),
            'stddev': statistics.pstdev(vals) if len(vals) > 1 else 0.0,
        }

    def snapshot(self):
        return {
            'count': self.count,
            'age_sec': self._stats(self.ages),
            'stamp_period_sec': self._stats(self.stamp_dts),
            'receive_period_sec': self._stats(self.recv_dts),
        }


class LocalizationTimingMonitor(Node):
    def __init__(self):
        super().__init__('localization_timing_monitor')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('window_samples', 100)
        window = max(10, min(1000, int(self.get_parameter('window_samples').value)))
        self.series = {name: Series(window) for name in (
            'scan_nav', 'imu', 'esc_odom', 'ekf_odom', 'amcl_pose')}
        self.pub = self.create_publisher(String, '/localization/timing_health', 10)
        self.create_subscription(LaserScan, '/scan_nav', self._cb('scan_nav'), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self._cb('imu'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/esc/odom', self._cb('esc_odom'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self._cb('ekf_odom'), 20)
        amcl_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._cb('amcl_pose'), amcl_qos)
        rate = max(0.2, min(10.0, float(self.get_parameter('publish_rate_hz').value)))
        self.timer = self.create_timer(1.0 / rate, self._publish)

    def _cb(self, name):
        def callback(msg):
            stamp = msg.header.stamp
            stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            now_ros = self.get_clock().now().nanoseconds * 1e-9
            self.series[name].add(stamp_sec, now_ros, time.monotonic())
        return callback

    def _publish(self):
        payload = {
            'wall_time': time.time(),
            'note': 'diagnostic_only; positive age means message stamp is older than ROS now',
            'sources': {name: series.snapshot() for name, series in self.series.items()},
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LocalizationTimingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
