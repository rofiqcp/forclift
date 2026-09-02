#!/usr/bin/env python3
"""Bounded RViz RobotModel readiness gate for autonomous bring-up.

The autonomous RViz fixed frame is ``map`` while the visual URDF lives under
``visual_/``.  Starting RViz before map->odom, the visual base bridge, and the
prefixed robot_state_publisher are all ready makes RobotModel enter ERROR
intermittently.  This gate waits for the actual TF chain and the latched
robot_description before RViz is started.  The gate prefers a clean RobotModel startup, but it is bounded and fail-open for
RViz itself.  RViz must remain available for operator diagnostics even before an
initial pose creates map->odom; the navigation/motion safety gates remain
fail-closed independently.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class RobotModelReadyGate(Node):
    def __init__(self) -> None:
        super().__init__("rviz_robot_model_ready_gate")
        self.timeout_s = float(self.declare_parameter("timeout_s", 20.0).value)
        self.fail_open = bool(self.declare_parameter("fail_open", True).value)
        self.fixed_frame = str(self.declare_parameter("fixed_frame", "map").value)
        self.required_frames = list(self.declare_parameter(
            "required_frames",
            [
                "visual_/base_footprint",
                "visual_/body_link",
                "visual_/fork_link",
                "visual_/lidar_link",
            ],
        ).value)
        self.started = time.monotonic()
        self.description_seen = False
        self.done = False
        self.last_missing = []

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(String, "/robot_description", self._description_cb, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_timer(0.20, self._poll)
        self.get_logger().info(
            f"RobotModel readiness gate: fixed={self.fixed_frame} "
            f"frames={','.join(self.required_frames)} timeout={self.timeout_s:.1f}s "
            f"fail_open={self.fail_open}"
        )

    def _description_cb(self, msg: String) -> None:
        self.description_seen = bool(str(msg.data).strip())

    def _tf_ready(self, frame: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(self.fixed_frame, frame, rclpy.time.Time())
            return True
        except TransformException:
            return False
        except Exception:
            return False

    def _finish(self, ready: bool) -> None:
        if self.done:
            return
        self.done = True
        if ready:
            self.get_logger().info(
                f"READY: robot_description + {self.fixed_frame}->visual RobotModel TF chain "
                "available; RViz may start cleanly"
            )
        else:
            missing = ", ".join(self.last_missing) if self.last_missing else "unknown"
            self.get_logger().warning(
                f"TIMEOUT after {self.timeout_s:.1f}s: RViz will still start for diagnostics; "
                f"missing={missing} description={'ready' if self.description_seen else 'missing'}"
            )
        rclpy.shutdown()

    def _poll(self) -> None:
        if self.done:
            return
        missing = [frame for frame in self.required_frames if not self._tf_ready(frame)]
        self.last_missing = missing
        if self.description_seen and not missing:
            self._finish(True)
            return
        if (time.monotonic() - self.started) >= self.timeout_s:
            if self.fail_open:
                self._finish(False)
            else:
                missing_text = ", ".join(missing) if missing else "robot_description"
                self.get_logger().warning(
                    "STILL WAITING: autonomous RViz is held until RobotModel TF is valid; "
                    f"missing={missing_text} description={'ready' if self.description_seen else 'missing'}"
                )
                self.started = time.monotonic()


def main() -> int:
    rclpy.init()
    node = RobotModelReadyGate()
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
