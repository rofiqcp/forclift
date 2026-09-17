#!/usr/bin/python3
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


def zero_twist():
    return Twist()


class MissionCmdSelector(Node):
    """Select Nav2 or FSM velocity BEFORE smoother/collision/sensor safety."""
    def __init__(self):
        super().__init__('mission_cmd_selector')
        self.declare_parameter('nav_topic', '/cmd_vel_nav_raw')
        self.declare_parameter('mission_topic', '/mission/cmd_vel_raw')
        self.declare_parameter('override_topic', '/mission/velocity_override')
        self.declare_parameter('output_topic', '/cmd_vel_autonomy_raw')
        self.declare_parameter('nav_timeout_sec', 0.60)
        self.declare_parameter('mission_timeout_sec', 0.30)
        self.declare_parameter('override_timeout_sec', 0.50)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.nav = zero_twist(); self.nav_t = 0.0
        self.mission = zero_twist(); self.mission_t = 0.0
        self.override = False; self.override_t = 0.0; self.override_seen = False
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, self.get_parameter('nav_topic').value, self._nav_cb, qos)
        self.create_subscription(Twist, self.get_parameter('mission_topic').value, self._mission_cb, qos)
        self.create_subscription(Bool, self.get_parameter('override_topic').value, self._override_cb, state_qos)
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, qos)
        self.status_pub = self.create_publisher(String, '/mission/cmd_selector_status', state_qos)
        hz = max(5.0, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.last_status_t = 0.0
        self.get_logger().info('Mission command selector READY (fail-closed)')

    @staticmethod
    def _now(): return time.monotonic()
    def _nav_cb(self, msg): self.nav, self.nav_t = msg, self._now()
    def _mission_cb(self, msg): self.mission, self.mission_t = msg, self._now()
    def _override_cb(self, msg):
        self.override, self.override_t, self.override_seen = bool(msg.data), self._now(), True

    def _tick(self):
        now = self._now()
        nav_fresh = now - self.nav_t <= float(self.get_parameter('nav_timeout_sec').value)
        mission_fresh = now - self.mission_t <= float(self.get_parameter('mission_timeout_sec').value)
        override_fresh = self.override_seen and now - self.override_t <= float(self.get_parameter('override_timeout_sec').value)
        out = zero_twist(); source = 'IDLE'
        if self.override:
            if override_fresh and mission_fresh:
                out = self.mission; source = 'MISSION'
            else:
                source = 'MISSION_FAIL_CLOSED'
        elif nav_fresh:
            out = self.nav; source = 'NAV2'
        self.pub.publish(out)
        if now - self.last_status_t >= 0.5:
            m = String(); m.data = json.dumps({
                'source': source, 'override': self.override,
                'override_fresh': override_fresh, 'mission_fresh': mission_fresh,
                'nav_fresh': nav_fresh, 'linear_x': out.linear.x, 'angular_z': out.angular.z,
            }, separators=(',', ':'))
            self.status_pub.publish(m); self.last_status_t = now


def main(args=None):
    rclpy.init(args=args); node = MissionCmdSelector()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
