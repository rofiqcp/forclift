#!/usr/bin/python3
"""Gate PlannerServer on the minimum map-frame prerequisites.

This gate intentionally does not decide whether localization is accurate enough
for motion.  It only proves that the saved map is present, AMCL is ACTIVE, and
the real map->odom->base_footprint TF chain exists for several consecutive
cycles.  The stricter AMCL covariance/stability gate and autonomy health manager
remain fail-closed motion authorities.
"""

from __future__ import annotations

import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class MapTfReadyGate(Node):
    def __init__(self) -> None:
        super().__init__('autonomous_map_tf_ready_gate')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('amcl_state_service', '/amcl/get_state')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('stable_cycles', 3)
        self.declare_parameter('poll_period_sec', 0.20)
        self.declare_parameter('diagnostic_timeout_sec', 30.0)
        self.declare_parameter('fail_on_timeout', False)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.state_service = str(self.get_parameter('amcl_state_service').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.stable_needed = max(1, int(self.get_parameter('stable_cycles').value))
        self.poll_period = max(0.05, float(self.get_parameter('poll_period_sec').value))
        self.timeout = max(5.0, float(self.get_parameter('diagnostic_timeout_sec').value))
        self.fail_on_timeout = bool(self.get_parameter('fail_on_timeout').value)

        self.map_seen = False
        self.amcl_active = False
        self.state_future = None
        self.stable = 0
        self.ready = False
        self.failed = False
        self.started = time.monotonic()
        self.last_status = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.state_client = self.create_client(GetState, self.state_service)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(self.poll_period, self._tick)
        self.get_logger().info(
            f'[MAP-TF-GATE] waiting for {self.map_topic} + AMCL ACTIVE + '
            f'{self.global_frame}->{self.odom_frame}->{self.base_frame}')

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_seen = bool(msg.info.width and msg.info.height and msg.info.resolution > 0.0)

    def _poll_amcl(self) -> None:
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
        if self.state_future is None and self.state_client.service_is_ready():
            self.state_future = self.state_client.call_async(GetState.Request())

    def _has_tf(self, target: str, source: str) -> bool:
        try:
            return self.tf_buffer.can_transform(
                target, source, Time(), timeout=Duration(seconds=0.03))
        except Exception:
            return False

    def _tick(self) -> None:
        if self.ready or self.failed:
            return
        self._poll_amcl()
        map_odom = self._has_tf(self.global_frame, self.odom_frame)
        map_base = self._has_tf(self.global_frame, self.base_frame)
        status = (self.map_seen, self.amcl_active, map_odom, map_base)
        if status != self.last_status:
            self.get_logger().info(
                f'[MAP-TF-GATE] map={status[0]} amcl_active={status[1]} '
                f'map_odom={status[2]} map_base={status[3]}')
            self.last_status = status

        if all(status):
            self.stable += 1
        else:
            self.stable = 0

        if self.stable >= self.stable_needed:
            self.ready = True
            self.timer.cancel()
            self.get_logger().info(
                f'[MAP-TF-GATE] READY after {self.stable} stable cycles; '
                'PlannerServer may activate')
            return

        if time.monotonic() - self.started >= self.timeout:
            detail = (
                f'map={status[0]} amcl_active={status[1]} '
                f'map_odom={status[2]} map_base={status[3]}')
            if self.fail_on_timeout:
                self.failed = True
                self.timer.cancel()
                self.get_logger().error('[MAP-TF-GATE] timeout: ' + detail)
            else:
                # A missing operator Initial Pose is a normal pre-navigation state.
                # Never turn that into a one-shot permanent PlannerServer hold.
                # Keep waiting and let the already queued Goal be planned as soon
                # as AMCL publishes map->odom.
                self.get_logger().warning(
                    '[MAP-TF-GATE] STILL WAITING (non-terminal): ' + detail)
                self.started = time.monotonic()


def main() -> int:
    rclpy.init()
    node = MapTfReadyGate()
    try:
        while rclpy.ok() and not node.ready and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.10)
        return 0 if node.ready else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
