#!/usr/bin/python3
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String


def state_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ManualMotionHealth(Node):
    """Continuous fail-closed gate for mapping/commissioning manual motion.

    The node deliberately does not own E-stop or dead-man semantics. Those remain
    independent at the command source / ESC mux. This gate only proves that the
    minimum runtime prerequisites for manual movement are continuously healthy.
    """

    def __init__(self):
        super().__init__('manual_motion_health')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lidar_health_topic', '/lidar/safety_healthy')
        self.declare_parameter('esc_ready_topic', '/esc/ready')
        self.declare_parameter('output_topic', '/system/manual_motion_allowed')
        self.declare_parameter('status_topic', '/system/manual_motion_health')
        self.declare_parameter('lidar_timeout_sec', 0.60)
        self.declare_parameter('esc_timeout_sec', 0.80)
        self.declare_parameter('stable_recovery_sec', 0.75)
        self.declare_parameter('require_lidar', True)
        self.declare_parameter('require_esc', True)

        self.lidar_topic = str(self.get_parameter('lidar_health_topic').value)
        self.esc_topic = str(self.get_parameter('esc_ready_topic').value)
        self.lidar_timeout = max(0.05, float(self.get_parameter('lidar_timeout_sec').value))
        self.esc_timeout = max(0.05, float(self.get_parameter('esc_timeout_sec').value))
        self.stable_recovery = max(0.0, float(self.get_parameter('stable_recovery_sec').value))
        self.require_lidar = bool(self.get_parameter('require_lidar').value)
        self.require_esc = bool(self.get_parameter('require_esc').value)

        self.lidar_value = False
        self.esc_value = False
        self.lidar_stamp = None
        self.esc_stamp = None
        self.healthy_since = None
        self.allowed = False

        self.create_subscription(Bool, self.lidar_topic, self._on_lidar, state_qos())
        self.create_subscription(Bool, self.esc_topic, self._on_esc, state_qos())
        self.allowed_pub = self.create_publisher(
            Bool, str(self.get_parameter('output_topic').value), state_qos())
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), state_qos())

        hz = min(30.0, max(2.0, float(self.get_parameter('publish_rate_hz').value)))
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self._publish(False, ['startup'])

    def _on_lidar(self, msg):
        self.lidar_value = bool(msg.data)
        self.lidar_stamp = self.get_clock().now()

    def _on_esc(self, msg):
        self.esc_value = bool(msg.data)
        self.esc_stamp = self.get_clock().now()

    @staticmethod
    def _age(now, stamp):
        if stamp is None:
            return None
        return max(0.0, (now - stamp).nanoseconds / 1e9)

    def _tick(self):
        now = self.get_clock().now()
        lidar_age = self._age(now, self.lidar_stamp)
        esc_age = self._age(now, self.esc_stamp)
        reasons = []

        lidar_ok = True
        if self.require_lidar:
            lidar_ok = self.lidar_stamp is not None and self.lidar_value and lidar_age <= self.lidar_timeout
            if self.lidar_stamp is None:
                reasons.append('lidar:not_seen')
            elif lidar_age > self.lidar_timeout:
                reasons.append('lidar:stale')
            elif not self.lidar_value:
                reasons.append('lidar:unhealthy')

        esc_ok = True
        if self.require_esc:
            esc_ok = self.esc_stamp is not None and self.esc_value and esc_age <= self.esc_timeout
            if self.esc_stamp is None:
                reasons.append('esc:not_seen')
            elif esc_age > self.esc_timeout:
                reasons.append('esc:stale')
            elif not self.esc_value:
                reasons.append('esc:not_ready')

        raw_ok = lidar_ok and esc_ok
        if not raw_ok:
            self.healthy_since = None
            self.allowed = False
        else:
            if self.healthy_since is None:
                self.healthy_since = now
            stable_for = (now - self.healthy_since).nanoseconds / 1e9
            self.allowed = stable_for >= self.stable_recovery
            if not self.allowed:
                reasons.append('recovery:stabilizing')

        self._publish(self.allowed, reasons, lidar_age, esc_age)

    def _publish(self, allowed, reasons, lidar_age=None, esc_age=None):
        b = Bool(); b.data = bool(allowed); self.allowed_pub.publish(b)
        s = String()
        s.data = json.dumps({
            'allowed': bool(allowed),
            'reasons': list(reasons),
            'lidar_healthy': bool(self.lidar_value),
            'lidar_age_sec': lidar_age,
            'esc_ready': bool(self.esc_value),
            'esc_age_sec': esc_age,
        }, separators=(',', ':'))
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = ManualMotionHealth()
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
