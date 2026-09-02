#!/usr/bin/env python3
"""GUI-mode compatibility publisher: /esc/odom -> /odom.

The production ESC topic remains /esc/odom. BAB-IV 4.1 uses the conventional
/odom name for raw odometry acquisition, so GUI mode exposes a lossless alias
without changing the ESC driver or autonomous launch.
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class GuiOdomAlias(Node):
    def __init__(self):
        super().__init__('gui_odom_alias')
        self.declare_parameter('source_topic', '/esc/odom')
        self.declare_parameter('output_topic', '/odom')
        source = str(self.get_parameter('source_topic').value)
        output = str(self.get_parameter('output_topic').value)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Odometry, output, qos)
        self.sub = self.create_subscription(Odometry, source, self._cb, qos)
        self.get_logger().info(f'[GUI-ODOM] alias active: {source} -> {output}')

    def _cb(self, msg: Odometry):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GuiOdomAlias()
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
