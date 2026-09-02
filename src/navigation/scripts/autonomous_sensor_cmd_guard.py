#!/usr/bin/python3
"""Fail-safe velocity gate for autonomous navigation.

This node is the final autonomous command gate. Motion is permitted only when
the minimally filtered /scan_safety stream is fresh AND its continuous quality
monitor reports healthy, /imu/data is fresh, the ESC reports /esc/ready=true,
and the incoming collision-safe command is fresh. Any failed prerequisite immediately
forces ZERO velocity; recovery requires all prerequisites to remain stable.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool


class SensorCmdGuard(Node):
    def __init__(self):
        super().__init__('autonomous_sensor_cmd_guard')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_collision_safe')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan_safety')
        self.declare_parameter('lidar_health_topic', '/lidar/safety_healthy')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('esc_ready_topic', '/esc/ready')
        self.declare_parameter('require_esc_ready', True)
        self.declare_parameter('scan_timeout_sec', 0.45)
        self.declare_parameter('lidar_health_timeout_sec', 0.55)
        self.declare_parameter('imu_timeout_sec', 0.5)
        self.declare_parameter('esc_ready_timeout_sec', 0.75)
        self.declare_parameter('cmd_timeout_sec', 0.4)
        self.declare_parameter('stable_recovery_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.lidar_health_timeout = float(self.get_parameter('lidar_health_timeout_sec').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout_sec').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self.esc_ready_timeout = float(self.get_parameter('esc_ready_timeout_sec').value)
        self.require_esc_ready = bool(self.get_parameter('require_esc_ready').value)
        self.stable_recovery = float(self.get_parameter('stable_recovery_sec').value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        imu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_cmd_topic').value), 10)
        self.health_pub = self.create_publisher(Bool, '/sensor_guard/healthy', 10)
        self.create_subscription(
            Twist, str(self.get_parameter('input_cmd_topic').value), self._cmd_cb, 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value), self._scan_cb, sensor_qos)
        self.create_subscription(
            Bool, str(self.get_parameter('lidar_health_topic').value), self._lidar_health_cb, state_qos)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._imu_cb, imu_qos)
        self.create_subscription(
            Bool, str(self.get_parameter('esc_ready_topic').value), self._esc_ready_cb, state_qos)

        self.last_scan = None
        self.last_lidar_health = None
        self.lidar_healthy = False
        self.last_imu = None
        self.last_esc_ready = None
        self.esc_ready = False
        self.last_cmd = None
        self.latest_cmd = Twist()
        self.healthy_since = None
        self.is_healthy = False
        self.last_state_log = None
        self.has_been_healthy = False
        self.timer = self.create_timer(
            1.0 / max(5.0, float(self.get_parameter('publish_rate_hz').value)), self._tick)
        self.get_logger().info(
            '[SENSOR-GUARD] armed: safety-scan freshness + LiDAR quality + IMU + ESC-ready + fresh cmd required for motion')

    def _now(self):
        return time.monotonic()

    def _scan_cb(self, _msg):
        self.last_scan = self._now()

    def _lidar_health_cb(self, msg):
        self.last_lidar_health = self._now()
        self.lidar_healthy = bool(msg.data)

    def _imu_cb(self, _msg):
        self.last_imu = self._now()

    def _esc_ready_cb(self, msg):
        self.last_esc_ready = self._now()
        self.esc_ready = bool(msg.data)

    def _cmd_cb(self, msg):
        self.latest_cmd = msg
        self.last_cmd = self._now()

    @staticmethod
    def _zero():
        return Twist()

    def _tick(self):
        now = self._now()
        scan_ok = self.last_scan is not None and now - self.last_scan <= self.scan_timeout
        lidar_health_fresh = (
            self.last_lidar_health is not None and
            now - self.last_lidar_health <= self.lidar_health_timeout)
        lidar_ok = lidar_health_fresh and self.lidar_healthy
        imu_ok = self.last_imu is not None and now - self.last_imu <= self.imu_timeout
        esc_fresh = (
            self.last_esc_ready is not None and
            now - self.last_esc_ready <= self.esc_ready_timeout)
        esc_ok = (not self.require_esc_ready) or (esc_fresh and self.esc_ready)
        raw_healthy = scan_ok and lidar_ok and imu_ok and esc_ok

        if raw_healthy:
            if self.healthy_since is None:
                self.healthy_since = now
            stable = now - self.healthy_since >= self.stable_recovery
        else:
            self.healthy_since = None
            stable = False

        self.is_healthy = stable
        self.health_pub.publish(Bool(data=stable))

        cmd_fresh = self.last_cmd is not None and now - self.last_cmd <= self.cmd_timeout
        if stable and cmd_fresh:
            self.cmd_pub.publish(self.latest_cmd)
        else:
            self.cmd_pub.publish(self._zero())

        state = (scan_ok, lidar_ok, imu_ok, esc_ok, stable)
        if state != self.last_state_log:
            scan_age = float('inf') if self.last_scan is None else now - self.last_scan
            imu_age = float('inf') if self.last_imu is None else now - self.last_imu
            if stable:
                self.has_been_healthy = True
                self.get_logger().info(
                    '[SENSOR-GUARD] HEALTHY safety_scan=%.3fs lidar_quality=%s imu=%.3fs esc_ready=%s -> commands enabled' %
                    (scan_age, lidar_ok, imu_age, esc_ok))
            elif self.has_been_healthy:
                # A real runtime dropout after the system was healthy remains a
                # warning.  Initial startup settling is expected and is INFO.
                self.get_logger().warn(
                    '[SENSOR-GUARD] SAFETY HOLD -> ZERO scan_ok=%s lidar_quality=%s imu_ok=%s esc_ok=%s scan_age=%.3fs imu_age=%.3fs' %
                    (scan_ok, lidar_ok, imu_ok, esc_ok, scan_age, imu_age))
            else:
                self.get_logger().info(
                    '[SENSOR-GUARD] startup hold -> zero cmd; scan_ok=%s lidar_quality=%s imu_ok=%s esc_ok=%s' %
                    (scan_ok, lidar_ok, imu_ok, esc_ok))
            self.last_state_log = state


def main(args=None):
    rclpy.init(args=args)
    node = SensorCmdGuard()
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
