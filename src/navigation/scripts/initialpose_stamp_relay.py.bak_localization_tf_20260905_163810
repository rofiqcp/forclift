#!/usr/bin/python3
"""Deliver GUI/RViz initial poses only after AMCL is lifecycle ACTIVE.

The old one-shot relay could publish while AMCL was still unconfigured, so the
message was lost, or while AMCL was configured-but-inactive, which produces an
AMCL warning.  This relay caches the newest operator pose and releases it only
when /amcl/get_state reports ACTIVE.  The timestamp is refreshed/backdated at
delivery time so AMCL can transform it through the current odometry history.
"""

import copy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.duration import Duration
from rclpy.node import Node


class InitialPoseStampRelay(Node):
    def __init__(self):
        super().__init__('initialpose_stamp_relay')
        self.declare_parameter('input_topic', '/initialpose_safe')
        self.declare_parameter('output_topic', '/initialpose')
        self.declare_parameter('amcl_state_service', '/amcl/get_state')
        self.declare_parameter('backdate_sec', 0.15)
        self.declare_parameter('poll_period_sec', 0.20)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.state_service = str(self.get_parameter('amcl_state_service').value)
        self.backdate = max(0.0, float(self.get_parameter('backdate_sec').value))
        self.poll_period = max(0.05, float(self.get_parameter('poll_period_sec').value))

        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, self.input_topic, self._cb, 10)
        self.state_client = self.create_client(GetState, self.state_service)
        self.state_future = None
        self.amcl_active = False
        self.pending_pose = None
        self.timer = self.create_timer(self.poll_period, self._tick)

        self.get_logger().info(
            '[INITIALPOSE-SAFE] READY: operator pose is cached until AMCL ACTIVE; '
            'delivery stamp backdated %.3fs' % self.backdate)

    def _restamp(self, msg: PoseWithCovarianceStamped) -> PoseWithCovarianceStamped:
        out = copy.deepcopy(msg)
        out.header.stamp = (
            self.get_clock().now() - Duration(seconds=self.backdate)).to_msg()
        return out

    def _deliver_pending(self):
        if not self.amcl_active or self.pending_pose is None:
            return
        out = self._restamp(self.pending_pose)
        self.pub.publish(out)
        self.pending_pose = None
        self.get_logger().info(
            '[INITIALPOSE-SAFE] initial pose delivered after AMCL lifecycle ACTIVE')

    def _cb(self, msg: PoseWithCovarianceStamped):
        # Keep only the newest operator estimate. Do not publish while AMCL is
        # inactive: that avoids both a lost pre-configure message and the normal
        # AMCL "received while inactive" warning.
        self.pending_pose = copy.deepcopy(msg)
        # Delivery happens only from the lifecycle polling timer, immediately
        # after a fresh /amcl/get_state result confirms ACTIVE. This avoids a
        # narrow stale-state race if AMCL transitions between timer polls.
        self.get_logger().info(
            '[INITIALPOSE-SAFE] initial pose queued; waiting for AMCL ACTIVE confirmation')

    def _tick(self):
        if self.state_future is not None:
            if not self.state_future.done():
                return
            try:
                response = self.state_future.result()
                self.amcl_active = bool(
                    response and response.current_state.label.strip().lower() == 'active')
            except Exception:
                self.amcl_active = False
            self.state_future = None

        if self.amcl_active:
            self._deliver_pending()

        if self.state_future is None and self.state_client.service_is_ready():
            self.state_future = self.state_client.call_async(GetState.Request())


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseStampRelay()
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
