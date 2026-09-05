#!/usr/bin/python3
"""Continuous fail-closed health aggregator for autonomous motion.

Publishes /system/autonomy_motion_allowed only while localization, TF, Nav2,
sensor safety and ESC feedback remain healthy.  The output uses transient-local
QoS so a late-starting ESC mux immediately receives the latest state, but the
mux must also enforce its own freshness timeout.
"""
from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, TransitionEvent
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def _qos_state() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _qos_reliable(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)


def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class AutonomyHealthManager(Node):
    def __init__(self):
        super().__init__('autonomy_health_manager')
        self._declare()
        self.rate = max(2.0, float(self.get_parameter('publish_rate_hz').value))
        self.recovery = max(0.0, float(self.get_parameter('stable_recovery_sec').value))
        self.startup_grace = max(0.0, float(self.get_parameter('startup_grace_sec').value))
        self.started = time.monotonic()
        self.healthy_since: Optional[float] = None
        self.allowed = False
        self.last_signature = None

        self.last: Dict[str, float] = {}
        self.bool_state: Dict[str, bool] = {'lidar': False, 'guard': False, 'esc': False}
        self.amcl_cov = (math.inf, math.inf, math.inf)
        self.jump_fault_until = 0.0
        self.last_map_base: Optional[Tuple[float, float, float, float]] = None

        state_qos = _qos_state()
        reliable = _qos_reliable()
        self.create_subscription(Bool, str(self.get_parameter('lidar_health_topic').value),
                                 lambda m: self._bool('lidar', m), state_qos)
        self.create_subscription(Bool, str(self.get_parameter('sensor_guard_topic').value),
                                 lambda m: self._bool('guard', m), reliable)
        self.create_subscription(Bool, str(self.get_parameter('esc_ready_topic').value),
                                 lambda m: self._bool('esc', m), state_qos)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value),
                                 self._odom, reliable)
        self.create_subscription(PoseWithCovarianceStamped, str(self.get_parameter('amcl_pose_topic').value),
                                 self._amcl, state_qos)
        map_qos = _qos_reliable(2)
        self.create_subscription(OccupancyGrid, str(self.get_parameter('global_costmap_topic').value),
                                 lambda _m: self._touch('global_costmap'), map_qos)
        self.create_subscription(OccupancyGrid, str(self.get_parameter('local_costmap_topic').value),
                                 lambda _m: self._touch('local_costmap'), map_qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lifecycle_names = [str(x).lstrip('/') for x in self.get_parameter('required_lifecycle_nodes').value]
        self.lifecycle_clients = {
            n: self.create_client(GetState, f'/{n}/get_state') for n in self.lifecycle_names
        }
        self.lifecycle_states: Dict[str, Optional[int]] = {n: None for n in self.lifecycle_names}
        self.lifecycle_seen: Dict[str, float] = {n: 0.0 for n in self.lifecycle_names}
        self.lifecycle_pending: Dict[str, bool] = {n: False for n in self.lifecycle_names}
        self.lifecycle_generation: Dict[str, int] = {n: 0 for n in self.lifecycle_names}
        lifecycle_event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        for lifecycle_name in self.lifecycle_names:
            self.create_subscription(
                TransitionEvent, f'/{lifecycle_name}/transition_event',
                lambda msg, n=lifecycle_name: self._lifecycle_event(n, msg),
                lifecycle_event_qos)
        self.lifecycle_timer = self.create_timer(
            max(0.2, float(self.get_parameter('lifecycle_poll_sec').value)), self._poll_lifecycle)

        self.allowed_pub = self.create_publisher(Bool, '/system/autonomy_motion_allowed', state_qos)
        self.status_pub = self.create_publisher(String, '/system/autonomy_health', state_qos)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self._publish(False, ['startup'])
        self.get_logger().info('[AUTONOMY-HEALTH] fail-closed continuous interlock armed')

    def _declare(self):
        defaults = {
            'publish_rate_hz': 10.0, 'stable_recovery_sec': 1.0, 'startup_grace_sec': 1.0,
            'lidar_health_topic': '/lidar/safety_healthy', 'sensor_guard_topic': '/sensor_guard/healthy',
            'esc_ready_topic': '/esc/ready', 'odom_topic': '/odometry/filtered',
            'amcl_pose_topic': '/amcl_pose', 'amcl_pose_require_seen': True, 'global_costmap_topic': '/global_costmap/costmap',
            'local_costmap_topic': '/local_costmap/costmap', 'lidar_health_timeout_sec': 0.65,
            'sensor_guard_timeout_sec': 0.65, 'esc_ready_timeout_sec': 0.80,
            'odom_timeout_sec': 0.50, 'costmap_timeout_sec': 3.0,
            'global_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
            'tf_max_age_sec': 1.0, 'tf_future_tolerance_sec': 1.25, 'tf_jump_hold_sec': 1.5, 'max_map_base_jump_m': 0.75,
            'max_map_base_jump_rad': 0.70, 'require_amcl_covariance': True,
            'max_amcl_cov_x': 0.50, 'max_amcl_cov_y': 0.50, 'max_amcl_cov_yaw': 0.40,
            'lifecycle_poll_sec': 0.75, 'lifecycle_response_timeout_sec': 2.0,
            'required_lifecycle_nodes': ['amcl', 'planner_server', 'controller_server', 'behavior_server',
                                         'bt_navigator', 'velocity_smoother', 'collision_monitor'],
            # Static launch-time physical-calibration interlock.  This is kept
            # separate from sensor/runtime health so the full stack can run for
            # diagnostics while actuator motion remains fail-closed.
            'geometry_validated': True,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

    @staticmethod
    def _mono() -> float:
        return time.monotonic()

    def _touch(self, key: str):
        self.last[key] = self._mono()

    def _bool(self, key: str, msg: Bool):
        self.bool_state[key] = bool(msg.data)
        self._touch(key)

    def _odom(self, _msg: Odometry):
        self._touch('odom')

    def _amcl(self, msg: PoseWithCovarianceStamped):
        self._touch('amcl')
        c = msg.pose.covariance
        self.amcl_cov = (float(c[0]), float(c[7]), float(c[35]))

    def _lifecycle_event(self, name: str, msg: TransitionEvent):
        try:
            self.lifecycle_states[name] = int(msg.goal_state.id)
            self.lifecycle_seen[name] = self._mono()
            self.lifecycle_pending[name] = False
        except Exception:
            pass

    def _graph_lifecycle_names(self):
        try:
            return {name for name, _ns in self.get_node_names_and_namespaces()}
        except Exception:
            return set()

    def _poll_lifecycle(self):
        # Event-driven after bootstrap. GetState is used only when a live node has
        # no known lifecycle state yet, preventing periodic RPC storms on Jetson.
        now = self._mono()
        live_nodes = self._graph_lifecycle_names()
        response_timeout = max(0.5, float(self.get_parameter('lifecycle_response_timeout_sec').value))
        for name, client in self.lifecycle_clients.items():
            if name not in live_nodes:
                self.lifecycle_states[name] = None
                self.lifecycle_pending[name] = False
                continue
            if self.lifecycle_states.get(name) is not None:
                continue
            if self.lifecycle_pending[name] and now - self.lifecycle_seen[name] <= response_timeout:
                continue
            if self.lifecycle_pending[name] and now - self.lifecycle_seen[name] > response_timeout:
                self.lifecycle_pending[name] = False
            if not client.service_is_ready():
                continue
            self.lifecycle_pending[name] = True
            self.lifecycle_seen[name] = now
            self.lifecycle_generation[name] += 1
            generation = self.lifecycle_generation[name]
            fut = client.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=name, g=generation: self._lifecycle_done(n, g, f))

    def _lifecycle_done(self, name: str, generation: int, future):
        if generation != self.lifecycle_generation[name]:
            return
        self.lifecycle_pending[name] = False
        try:
            self.lifecycle_states[name] = int(future.result().current_state.id)
            self.lifecycle_seen[name] = self._mono()
        except Exception:
            # Keep fail-closed bootstrap state unknown. Future timer iterations
            # may retry once, but known event-derived states are never erased by
            # a late RPC timeout.
            if self.lifecycle_states.get(name) is None:
                self.lifecycle_seen[name] = 0.0

    def _fresh_true(self, key: str, timeout_param: str, reasons) -> bool:
        t = self.last.get(key)
        fresh = t is not None and self._mono() - t <= float(self.get_parameter(timeout_param).value)
        ok = fresh and bool(self.bool_state.get(key, False))
        if not ok:
            reasons.append(f'{key}:false_or_stale')
        return ok

    def _fresh(self, key: str, timeout_param: str, reasons) -> bool:
        t = self.last.get(key)
        ok = t is not None and self._mono() - t <= float(self.get_parameter(timeout_param).value)
        if not ok:
            reasons.append(f'{key}:stale')
        return ok

    def _tf_ok(self, reasons) -> bool:
        now_m = self._mono()
        global_f = str(self.get_parameter('global_frame').value)
        odom_f = str(self.get_parameter('odom_frame').value)
        base_f = str(self.get_parameter('base_frame').value)
        max_age = float(self.get_parameter('tf_max_age_sec').value)
        future_tol = float(self.get_parameter('tf_future_tolerance_sec').value)
        transforms = {}
        for key, target, source in [('map_base', global_f, base_f), ('odom_base', odom_f, base_f)]:
            try:
                tr = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.03))
                stamp = Time.from_msg(tr.header.stamp)
                age = (self.get_clock().now() - stamp).nanoseconds / 1e9
                if not (-future_tol <= age <= max_age):
                    reasons.append(f'tf:{key}:age={age:.3f}')
                    return False
                transforms[key] = tr
            except Exception:
                reasons.append(f'tf:{key}:missing')
                return False
        tr = transforms['map_base'].transform
        pose = (float(tr.translation.x), float(tr.translation.y), _yaw(tr.rotation), now_m)
        if self.last_map_base is not None:
            lx, ly, lyaw, lt = self.last_map_base
            dt = max(1e-3, now_m - lt)
            # Compare only adjacent fresh samples. Long outages are handled by TF age.
            if dt <= 1.0:
                dist = math.hypot(pose[0] - lx, pose[1] - ly)
                dyaw = abs(_wrap(pose[2] - lyaw))
                if dist > float(self.get_parameter('max_map_base_jump_m').value) or \
                   dyaw > float(self.get_parameter('max_map_base_jump_rad').value):
                    self.jump_fault_until = now_m + float(self.get_parameter('tf_jump_hold_sec').value)
        self.last_map_base = pose
        if now_m < self.jump_fault_until:
            reasons.append('tf:map_base_jump_hold')
            return False
        return True

    def _lifecycle_ok(self, reasons) -> bool:
        # Lifecycle state is persistent, not a freshness signal. Node graph
        # liveness is checked by _poll_lifecycle; an absent process resets state
        # to None. This avoids declaring ACTIVE nodes missing just because a
        # GetState response arrived late.
        ok = True
        for name in self.lifecycle_names:
            state = self.lifecycle_states.get(name)
            if state != State.PRIMARY_STATE_ACTIVE:
                reasons.append(f'lifecycle:{name}:{state if state is not None else "missing"}')
                ok = False
        return ok

    def _publish(self, allowed: bool, reasons, extra=None):
        self.allowed_pub.publish(Bool(data=bool(allowed)))
        finite_or_none = lambda v: float(v) if math.isfinite(float(v)) else None
        payload = {
            'allowed': bool(allowed), 'reasons': list(reasons),
            'amcl_cov_x': finite_or_none(self.amcl_cov[0]),
            'amcl_cov_y': finite_or_none(self.amcl_cov[1]),
            'amcl_cov_yaw': finite_or_none(self.amcl_cov[2]),
            'lifecycle': {n: self.lifecycle_states.get(n) for n in self.lifecycle_names},
            'amcl_pose_age_s': (self._mono() - self.last['amcl']) if 'amcl' in self.last else None,
        }
        if extra: payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), allow_nan=False)))

    def _tick(self):
        reasons = []
        if not bool(self.get_parameter('geometry_validated').value):
            reasons.append('geometry:unvalidated')
        self._fresh_true('lidar', 'lidar_health_timeout_sec', reasons)
        self._fresh_true('guard', 'sensor_guard_timeout_sec', reasons)
        self._fresh_true('esc', 'esc_ready_timeout_sec', reasons)
        self._fresh('odom', 'odom_timeout_sec', reasons)
        if bool(self.get_parameter('amcl_pose_require_seen').value) and 'amcl' not in self.last:
            reasons.append('amcl:pose_not_seen')
        self._fresh('global_costmap', 'costmap_timeout_sec', reasons)
        self._fresh('local_costmap', 'costmap_timeout_sec', reasons)

        if bool(self.get_parameter('require_amcl_covariance').value):
            cx, cy, cyaw = self.amcl_cov
            if not (math.isfinite(cx) and 0.0 <= cx <= float(self.get_parameter('max_amcl_cov_x').value)):
                reasons.append('amcl:cov_x')
            if not (math.isfinite(cy) and 0.0 <= cy <= float(self.get_parameter('max_amcl_cov_y').value)):
                reasons.append('amcl:cov_y')
            if not (math.isfinite(cyaw) and 0.0 <= cyaw <= float(self.get_parameter('max_amcl_cov_yaw').value)):
                reasons.append('amcl:cov_yaw')

        self._tf_ok(reasons)
        self._lifecycle_ok(reasons)
        raw = len(reasons) == 0 and self._mono() - self.started >= self.startup_grace
        if raw:
            if self.healthy_since is None:
                self.healthy_since = self._mono()
            allowed = self._mono() - self.healthy_since >= self.recovery
        else:
            self.healthy_since = None
            allowed = False

        signature = (allowed, tuple(reasons))
        if signature != self.last_signature:
            if allowed:
                self.get_logger().info('[AUTONOMY-HEALTH] ALLOWED - all continuous prerequisites healthy')
            elif self.allowed:
                self.get_logger().error('[AUTONOMY-HEALTH] MOTION REVOKED: ' + ', '.join(reasons[:8]))
            else:
                self.get_logger().info('[AUTONOMY-HEALTH] HOLD: ' + ', '.join(reasons[:8]))
            self.last_signature = signature
        self.allowed = allowed
        self._publish(allowed, reasons)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyHealthManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(False, ['shutdown'])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
