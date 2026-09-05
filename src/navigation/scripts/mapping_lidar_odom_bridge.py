#!/usr/bin/python3
"""Isolated odometry/scan bridge for shared-sensor SLAM mapping.

Uses the existing diagnostic /lidar/odom as the mapping motion reference and
re-publishes /scan_nav only after a same-stamp odometry TF is available.
No autonomous TF, sensor driver, or actuator ownership is changed.
"""
import copy
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MappingLidarOdomBridge(Node):
    def __init__(self):
        super().__init__('mapping_lidar_odom_bridge')
        self.declare_parameter('source_scan_topic', '/scan_nav')
        self.declare_parameter('source_odom_topic', '/lidar/odom')
        self.declare_parameter('output_scan_topic', '/mapping/scan_nav')
        self.declare_parameter('mapping_odom_frame', 'mapping_odom')
        self.declare_parameter('mapping_base_frame', 'mapping_base_footprint')
        self.declare_parameter('mapping_lidar_frame', 'mapping_lidar_link')
        self.declare_parameter('source_base_frame', 'base_footprint')
        self.declare_parameter('source_lidar_frame', 'lidar_link')
        self.declare_parameter('pair_tolerance_sec', 0.060)

        self.source_scan = self.get_parameter('source_scan_topic').value
        self.source_odom = self.get_parameter('source_odom_topic').value
        self.output_scan = self.get_parameter('output_scan_topic').value
        self.odom_frame = self.get_parameter('mapping_odom_frame').value
        self.base_frame = self.get_parameter('mapping_base_frame').value
        self.lidar_frame = self.get_parameter('mapping_lidar_frame').value
        self.source_base = self.get_parameter('source_base_frame').value
        self.source_lidar = self.get_parameter('source_lidar_frame').value
        self.tolerance_ns = int(float(self.get_parameter('pair_tolerance_sec').value) * 1e9)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.scan_pub = self.create_publisher(LaserScan, self.output_scan, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/mapping/odom_status', 10)
        session_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/mapping/session_enabled', self._session_cb, session_qos)
        self.create_subscription(LaserScan, self.source_scan, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.source_odom, self._odom_cb, QoSProfile(depth=20))

        self.scans = deque(maxlen=40)
        self.last_odom = None
        self.published_stamps = deque(maxlen=80)
        self.static_ready = False
        self.session_enabled = False
        self.pairs = 0
        self.dropped_pairs = 0
        self.create_timer(0.25, self._ensure_static_tf)
        self.create_timer(0.50, self._publish_idle_status)
        self.get_logger().info(
            f'Mapping bridge READY: {self.source_odom} + {self.source_scan} -> '
            f'{self.odom_frame}->{self.base_frame}, {self.output_scan}[{self.lidar_frame}]')

    def _ensure_static_tf(self):
        if self.static_ready:
            return
        try:
            src = self.tf_buffer.lookup_transform(
                self.source_base, self.source_lidar, Time(), timeout=Duration(seconds=0.15))
        except Exception:
            return
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.child_frame_id = self.lidar_frame
        out.transform = src.transform
        self.static_broadcaster.sendTransform(out)
        self.static_ready = True
        self.get_logger().info('Mapping static TF copied from base_footprint -> lidar_link')

    def _session_cb(self, msg):
        enabled = bool(msg.data)
        if enabled == self.session_enabled:
            return
        self.session_enabled = enabled
        self.scans.clear()
        self.last_odom = None
        self.published_stamps.clear()
        self.pairs = 0
        self.dropped_pairs = 0
        self.get_logger().info(f"Mapping scan gate {'ENABLED' if enabled else 'STANDBY'}")

    def _mapping_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', self.base_frame, Time(), timeout=Duration(seconds=0.01))
            return {
                'map_x': round(float(tf.transform.translation.x), 4),
                'map_y': round(float(tf.transform.translation.y), 4),
                'map_yaw': round(float(yaw_from_quat(tf.transform.rotation)), 5),
                'pose_source': 'slam_tf',
            }
        except Exception:
            return {}

    def _publish_idle_status(self):
        payload = {
            'ready': self.static_ready,
            'session_enabled': self.session_enabled,
            'pairs': self.pairs,
            'unmatched_odom': self.dropped_pairs,
        }
        payload.update(self._mapping_pose())
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(msg)

    def _publish_tf(self, odom, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def _nearest_scan(self, target_ns):
        if not self.scans:
            return None
        best = min(self.scans, key=lambda item: abs(item[0] - target_ns))
        if abs(best[0] - target_ns) > self.tolerance_ns:
            return None
        return best[1]

    def _publish_pair(self, scan, odom):
        if not self.static_ready:
            return False
        sns = stamp_ns(scan.header.stamp)
        if sns in self.published_stamps:
            return True
        self._publish_tf(odom, scan.header.stamp)
        out = copy.deepcopy(scan)
        out.header.frame_id = self.lidar_frame
        self.scan_pub.publish(out)
        self.published_stamps.append(sns)
        self.pairs += 1
        self._publish_status(odom, scan)
        return True

    def _scan_cb(self, scan):
        if not self.session_enabled:
            return
        sns = stamp_ns(scan.header.stamp)
        self.scans.append((sns, scan))
        if self.last_odom is None:
            return
        ons = stamp_ns(self.last_odom.header.stamp)
        if abs(sns - ons) <= self.tolerance_ns:
            self._publish_pair(scan, self.last_odom)

    def _odom_cb(self, odom):
        if not self.session_enabled:
            return
        self.last_odom = odom
        ons = stamp_ns(odom.header.stamp)
        scan = self._nearest_scan(ons)
        if scan is None:
            self.dropped_pairs += 1
            return
        self._publish_pair(scan, odom)

    def _publish_status(self, odom, scan):
        msg = String()
        payload = {
            'ready': self.static_ready,
            'session_enabled': self.session_enabled,
            'pairs': self.pairs,
            'unmatched_odom': self.dropped_pairs,
            'x': round(float(odom.pose.pose.position.x), 4),
            'y': round(float(odom.pose.pose.position.y), 4),
            'yaw': round(float(yaw_from_quat(odom.pose.pose.orientation)), 5),
            'v': round(float(odom.twist.twist.linear.x), 4),
            'scan_odom_dt_ms': round(abs(stamp_ns(scan.header.stamp) - stamp_ns(odom.header.stamp)) / 1e6, 3),
            'stamp_ns': stamp_ns(scan.header.stamp),
        }
        payload.update(self._mapping_pose())
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingLidarOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
