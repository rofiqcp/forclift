#!/usr/bin/python3
"""Fail-closed pre-AMCL local-localization readiness gate.

Production localization contract (PART-5 Stage-1):
  /esc/odom + /imu/data -> EKF -> odom -> base_footprint
  /scan_nav             -> AMCL after this gate passes
  /lidar/odom            -> diagnostics only; never a startup dependency

AMCL may activate only after the independent local-motion foundation is real and
fresh: ESC odometry, IMU, navigation scan, LiDAR safety health, filtered EKF
odometry, and the required static/dynamic TF chain. No fake message or TF is
published by this node.
"""

import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def _norm(frame: str) -> str:
    return (frame or '').lstrip('/')


class PreAmclGate(Node):
    def __init__(self):
        super().__init__('autonomous_pre_amcl_gate')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('scan_topic', '/scan_nav')
        self.declare_parameter('esc_odom_topic', '/esc/odom')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('lidar_health_topic', '/lidar/safety_healthy')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('scan_frame', 'lidar_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_messages', 3)
        self.declare_parameter('freshness_sec', 2.0)
        self.declare_parameter('diagnostic_timeout_sec', 120.0)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.esc_odom_topic = str(self.get_parameter('esc_odom_topic').value)
        self.filtered_odom_topic = str(self.get_parameter('filtered_odom_topic').value)
        self.lidar_health_topic = str(self.get_parameter('lidar_health_topic').value)
        self.imu_frame = _norm(str(self.get_parameter('imu_frame').value))
        self.scan_frame = _norm(str(self.get_parameter('scan_frame').value))
        self.odom_frame = _norm(str(self.get_parameter('odom_frame').value))
        self.base_frame = _norm(str(self.get_parameter('base_frame').value))
        self.min_messages = max(1, int(self.get_parameter('min_messages').value))
        self.freshness_sec = max(0.5, float(self.get_parameter('freshness_sec').value))
        self.timeout_sec = max(5.0, float(self.get_parameter('diagnostic_timeout_sec').value))

        self.counts = {'imu': 0, 'scan': 0, 'esc_odom': 0, 'filtered_odom': 0}
        self.last_wall = {key: 0.0 for key in self.counts}
        self.frame_ok = {key: False for key in self.counts}
        self.lidar_health_seen = False
        self.lidar_healthy = False
        self.lidar_health_wall = 0.0
        self.ready = False
        self.start_wall = time.monotonic()
        self.last_status = ''
        self.last_timeout_report = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.esc_odom_topic, self._esc_odom_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.filtered_odom_topic, self._filtered_odom_cb, 20)
        state_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, self.lidar_health_topic, self._lidar_health_cb, state_qos)
        self.timer = self.create_timer(0.5, self._check)

        print(
            '[PRE-AMCL] required: /imu/data + /scan_nav + /esc/odom + '
            '/odometry/filtered + /lidar/safety_healthy + sensor/static TF + odom->base; '
            '/lidar/odom is DIAGNOSTIC ONLY', flush=True)

    def _touch(self, key: str):
        self.counts[key] += 1
        self.last_wall[key] = time.monotonic()

    def _imu_cb(self, msg: Imu):
        self._touch('imu')
        self.frame_ok['imu'] = _norm(msg.header.frame_id) == self.imu_frame

    def _scan_cb(self, msg: LaserScan):
        self._touch('scan')
        self.frame_ok['scan'] = _norm(msg.header.frame_id) == self.scan_frame

    def _esc_odom_cb(self, msg: Odometry):
        self._touch('esc_odom')
        self.frame_ok['esc_odom'] = (
            _norm(msg.header.frame_id) == self.odom_frame and
            _norm(msg.child_frame_id) == self.base_frame)

    def _filtered_odom_cb(self, msg: Odometry):
        self._touch('filtered_odom')
        self.frame_ok['filtered_odom'] = (
            _norm(msg.header.frame_id) == self.odom_frame and
            _norm(msg.child_frame_id) == self.base_frame)

    def _lidar_health_cb(self, msg: Bool):
        self.lidar_health_seen = True
        self.lidar_healthy = bool(msg.data)
        self.lidar_health_wall = time.monotonic()

    def _tf(self, target: str, source: str) -> bool:
        try:
            return self.tf_buffer.can_transform(
                target, source, Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return False

    def _check(self):
        now = time.monotonic()
        topic_ok = {}
        for key in self.counts:
            fresh = self.last_wall[key] > 0.0 and (now - self.last_wall[key]) <= self.freshness_sec
            topic_ok[key] = self.counts[key] >= self.min_messages and fresh and self.frame_ok[key]

        lidar_health_ok = (
            self.lidar_health_seen and self.lidar_healthy and self.lidar_health_wall > 0.0 and
            (now - self.lidar_health_wall) <= self.freshness_sec)
        tf_lidar = self._tf(self.base_frame, self.scan_frame)
        tf_imu = self._tf(self.base_frame, self.imu_frame)
        tf_local = self._tf(self.odom_frame, self.base_frame)

        status = (
            f"imu={int(topic_ok['imu'])} scan={int(topic_ok['scan'])} "
            f"esc_odom={int(topic_ok['esc_odom'])} ekf={int(topic_ok['filtered_odom'])} "
            f"lidar_health={int(lidar_health_ok)} tf_base_lidar={int(tf_lidar)} "
            f"tf_base_imu={int(tf_imu)} tf_odom_base={int(tf_local)}")
        if status != self.last_status:
            print('[PRE-AMCL] ' + status, flush=True)
            self.last_status = status

        if all(topic_ok.values()) and lidar_health_ok and tf_lidar and tf_imu and tf_local:
            self.ready = True
            print('[PRE-AMCL] READY — independent wheel+IMU local odometry verified; AMCL may activate', flush=True)
            self.get_logger().info('[PRE-AMCL] READY')
            return

        elapsed = now - self.start_wall
        if elapsed >= self.timeout_sec and now - self.last_timeout_report >= 10.0:
            self.last_timeout_report = now
            self.get_logger().error(
                '[PRE-AMCL] NOT READY after %.1fs. AMCL remains intentionally blocked. %s',
                elapsed, status)


def main():
    rclpy.init()
    node = PreAmclGate()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0 if node.ready else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
