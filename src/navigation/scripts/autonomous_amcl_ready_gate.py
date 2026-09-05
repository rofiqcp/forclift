#!/usr/bin/python3
"""Fail-closed AMCL initialization and convergence gate.

Safe PART-5 Stage-1 behavior:
  * OPERATOR (explicit fallback): never invent an initial pose; wait for /initialpose from
    GUI/RViz, then request no-motion updates until covariance + pose stability pass.
  * KNOWN_POSE: send an explicitly configured pose only when its saved-map SHA256
    exactly matches the map selected by autonomous.launch.py.
  * GLOBAL_LOCALIZATION: call AMCL's real global-localization service, then use
    live /scan_nav updates until covariance + pose stability pass. This is the
    automatic fallback for valid older maps that have no map-bound pose sidecar.

Readiness requires map, AMCL ACTIVE, converged AMCL pose, and real TF map->odom
and map->base_footprint. The node creates no TF itself.
"""

from collections import deque
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import SetInitialPose
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class AmclReadyGate(Node):
    def __init__(self):
        super().__init__('autonomous_amcl_ready_gate')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('operator_initial_pose_topic', '/initialpose_safe')
        self.declare_parameter('amcl_state_service', '/amcl/get_state')
        self.declare_parameter('global_localization_service', '/reinitialize_global_localization')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initial_pose_mode', 'OPERATOR')
        self.declare_parameter('known_pose_x', 0.0)
        self.declare_parameter('known_pose_y', 0.0)
        self.declare_parameter('known_pose_yaw', 0.0)
        self.declare_parameter('known_pose_map_sha256', '')
        self.declare_parameter('current_map_sha256', '')
        self.declare_parameter('require_map_hash_for_known_pose', True)
        self.declare_parameter('convergence_min_samples', 2)
        self.declare_parameter('max_cov_x', 0.50)
        self.declare_parameter('max_cov_y', 0.50)
        self.declare_parameter('max_cov_yaw', 0.40)
        self.declare_parameter('stability_translation_m', 0.15)
        self.declare_parameter('stability_yaw_rad', 0.20)
        self.declare_parameter('diagnostic_timeout_sec', 600.0)
        self.declare_parameter('bootstrap_delay_sec', 0.8)
        self.declare_parameter('initial_pose_backdate_sec', 0.15)
        self.declare_parameter('nomotion_update_period_sec', 1.0)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.operator_pose_topic = str(
            self.get_parameter('operator_initial_pose_topic').value)
        self.state_service = str(self.get_parameter('amcl_state_service').value)
        self.global_localization_service = str(
            self.get_parameter('global_localization_service').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.mode = str(self.get_parameter('initial_pose_mode').value).strip().upper()
        if self.mode not in ('OPERATOR', 'KNOWN_POSE', 'GLOBAL_LOCALIZATION'):
            self.get_logger().error(
                f'Invalid initial_pose_mode={self.mode!r}; forcing safe OPERATOR mode')
            self.mode = 'OPERATOR'
        self.known_x = float(self.get_parameter('known_pose_x').value)
        self.known_y = float(self.get_parameter('known_pose_y').value)
        self.known_yaw = float(self.get_parameter('known_pose_yaw').value)
        self.known_hash = str(self.get_parameter('known_pose_map_sha256').value).strip().lower()
        self.current_hash = str(self.get_parameter('current_map_sha256').value).strip().lower()
        self.require_known_hash = bool(self.get_parameter('require_map_hash_for_known_pose').value)
        self.min_samples = max(2, int(self.get_parameter('convergence_min_samples').value))
        self.max_cov_x = max(0.0, float(self.get_parameter('max_cov_x').value))
        self.max_cov_y = max(0.0, float(self.get_parameter('max_cov_y').value))
        self.max_cov_yaw = max(0.0, float(self.get_parameter('max_cov_yaw').value))
        self.stability_m = max(0.001, float(self.get_parameter('stability_translation_m').value))
        self.stability_yaw = max(0.001, float(self.get_parameter('stability_yaw_rad').value))
        self.timeout_sec = max(10.0, float(self.get_parameter('diagnostic_timeout_sec').value))
        self.bootstrap_delay = max(0.2, float(self.get_parameter('bootstrap_delay_sec').value))
        self.initial_pose_backdate = max(0.0, float(self.get_parameter('initial_pose_backdate_sec').value))
        self.nomotion_period = max(0.2, float(self.get_parameter('nomotion_update_period_sec').value))

        self.map_seen = False
        self.pose_seen = False
        self.operator_pose_requested = False
        self.ready = False
        self.start_wall = time.monotonic()
        self.last_timeout_report = 0.0
        self.last_status = ''
        self.last_operator_reminder = 0.0
        self.state_future = None
        self.amcl_active = False
        self.active_since = None
        self.initial_pose_sent = False
        self.initial_pose_future = None
        self.initial_pose_request_started = None
        # KNOWN_POSE bootstrap must not depend on lifecycle-service discovery.
        # Under a heavily loaded Jetson the /amcl/get_state service can be late
        # even though AMCL is already configured/active. Re-publish the
        # map-hash-bound pose until AMCL confirms it by emitting /amcl_pose.
        self.known_pose_last_publish = 0.0
        self.known_pose_publish_period = 1.0
        self.global_localization_requested = False
        self.global_localization_complete = False
        self.global_localization_future = None
        self.nomotion_future = None
        self.nomotion_last = 0.0
        self.samples = deque(maxlen=max(20, self.min_samples))

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self._pose_cb, pose_qos)
        # Arm this subscriber before AMCL lifecycle activation. A manual pose is
        # always authoritative over automatic/global bootstrapping and must not
        # be overwritten by a concurrent global-localization request.
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.operator_pose_topic,
            self._operator_pose_cb,
            10)
        # AMCL accepts initial pose via topic /initialpose (TRANSIENT_LOCAL) or
        # service /amcl/set_initial_pose. With FastDDS UDP-only the service call
        # discovery can fail, so we publish to the topic which AMCL subscribes.
        initialpose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        self.state_client = self.create_client(GetState, self.state_service)
        self.initial_pose_client = self.create_client(SetInitialPose, '/set_initial_pose')
        self.global_localization_client = self.create_client(
            Empty, self.global_localization_service)
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self._check)

        if self.mode == 'OPERATOR':
            print('[AMCL-READY] OPERATOR mode: waiting for GUI/RViz 2D Pose Estimate; origin is NOT assumed.', flush=True)
        elif self.mode == 'KNOWN_POSE':
            print(
                f'[AMCL-READY] KNOWN_POSE mode: pose=({self.known_x:.3f},{self.known_y:.3f},'
                f'{self.known_yaw:.3f}); map_hash_required={self.require_known_hash}', flush=True)
        else:
            print(
                '[AMCL-READY] GLOBAL_LOCALIZATION mode: AMCL will initialize from '
                'the saved /map and live /scan_nav; map origin is NOT assumed.', flush=True)

    def _map_cb(self, _msg):
        self.map_seen = True

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose_seen = True
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        c = msg.pose.covariance
        self.samples.append((
            float(p.x), float(p.y), float(_yaw(q)),
            float(c[0]), float(c[7]), float(c[35]), time.monotonic()))

    def _operator_pose_cb(self, _msg: PoseWithCovarianceStamped):
        self.operator_pose_requested = True
        # Do not allow covariance samples from a previous global/known pose to
        # make the gate READY before AMCL has processed the new manual estimate.
        self.pose_seen = False
        self.samples.clear()
        self.get_logger().info(
            '[AMCL-READY] operator initial pose requested; automatic bootstrap suppressed')

    def _tf(self, target, source):
        try:
            return self.tf_buffer.can_transform(
                target, source, Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return False

    def _poll_state(self):
        if self.state_future is not None:
            if self.state_future.done():
                try:
                    response = self.state_future.result()
                    was_active = self.amcl_active
                    self.amcl_active = bool(response and response.current_state.label.lower() == 'active')
                    if self.amcl_active and not was_active:
                        self.active_since = time.monotonic()
                        print('[AMCL-READY] AMCL lifecycle ACTIVE', flush=True)
                except Exception:
                    self.amcl_active = False
                self.state_future = None
            return
        if self.state_client.service_is_ready():
            self.state_future = self.state_client.call_async(GetState.Request())

    def _known_pose_hash_ok(self) -> bool:
        if self.operator_pose_requested:
            return True
        if self.mode != 'KNOWN_POSE':
            return True
        if not self.require_known_hash:
            return True
        return bool(self.known_hash and self.current_hash and self.known_hash == self.current_hash)

    def _initialize_if_needed(self):
        now = time.monotonic()

        # KNOWN_POSE is already map-hash-bound, so its delivery must not depend
        # on lifecycle-service discovery. During a loaded autonomous startup
        # FastDDS may discover /map and /initialpose before /amcl/get_state.
        # Publish the valid pose repeatedly until AMCL confirms it via /amcl_pose.
        if self.mode == 'KNOWN_POSE' and not self.operator_pose_requested and not self.pose_seen:
            if not self.map_seen or now - self.start_wall < self.bootstrap_delay:
                return
            if not self._tf(self.odom_frame, self.base_frame):
                # Do not inject an initial pose until EKF has published the local TF chain.
                # This prevents AMCL's initialPoseReceived future-extrapolation race.
                return
            if not self._known_pose_hash_ok():
                if now - self.last_operator_reminder >= 5.0:
                    self.last_operator_reminder = now
                    self.get_logger().error(
                        f'[AMCL-READY] KNOWN_POSE blocked: configured map SHA256='
                        f'{self.known_hash or "EMPTY"} current={self.current_hash or "EMPTY"}')
                return
            if now - self.known_pose_last_publish >= self.known_pose_publish_period:
                msg = PoseWithCovarianceStamped()
                # AMCL transforms the initial pose through odom->base. Under Jetson load
                # TF can trail wall time slightly, so use the same conservative backdate as
                # initialpose_stamp_relay instead of stamping a pose just ahead of TF.
                msg.header.stamp = (
                    self.get_clock().now() - Duration(seconds=self.initial_pose_backdate)
                ).to_msg()
                msg.header.frame_id = self.global_frame
                msg.pose.pose.position.x = self.known_x
                msg.pose.pose.position.y = self.known_y
                half = 0.5 * self.known_yaw
                msg.pose.pose.orientation.z = math.sin(half)
                msg.pose.pose.orientation.w = math.cos(half)
                msg.pose.covariance[0] = min(self.max_cov_x, 0.25)
                msg.pose.covariance[7] = min(self.max_cov_y, 0.25)
                msg.pose.covariance[35] = min(
                    self.max_cov_yaw, (math.pi / 12.0) ** 2)
                self.initialpose_pub.publish(msg)
                self.initial_pose_sent = True
                self.known_pose_last_publish = now
                print(
                    '[AMCL-READY] map-bound KNOWN_POSE published to /initialpose '
                    '(service discovery not required)', flush=True)
            return

        # GLOBAL_LOCALIZATION / OPERATOR readiness still requires authoritative
        # AMCL lifecycle state. This preserves the existing fail-closed behavior.
        if not (self.map_seen and self.amcl_active):
            return
        if self.active_since is None or now - self.active_since < self.bootstrap_delay:
            return

        # A GUI/RViz estimate is an explicit operator decision. The paired
        # initialpose relay waits for AMCL ACTIVE before delivery, therefore it
        # is safe to suppress KNOWN_POSE/GLOBAL_LOCALIZATION here and let the
        # operator estimate become the sole bootstrap source.
        if self.operator_pose_requested:
            if not self.pose_seen and now - self.last_operator_reminder >= 5.0:
                self.last_operator_reminder = now
                print(
                    '[AMCL-READY] operator pose queued; waiting for AMCL to publish '
                    'the updated localization estimate', flush=True)
            if self.pose_seen and self.nomotion_client.service_is_ready():
                if self.nomotion_future is None or self.nomotion_future.done():
                    if now - self.nomotion_last >= self.nomotion_period:
                        self.nomotion_last = now
                        self.nomotion_future = self.nomotion_client.call_async(Empty.Request())
            return

        if self.mode == 'KNOWN_POSE':
            # Inspect the async result. A failed early request must clear the
            # state so the valid map-bound pose can be retried once AMCL's
            # service is fully advertised.
            if self.initial_pose_future is not None:
                if not self.initial_pose_future.done():
                    # A request issued before FastDDS has discovered AMCL's
                    # service can remain pending forever in rclpy Humble.  Do
                    # not convert that discovery race into a permanent wait.
                    if (self.initial_pose_request_started is not None and
                            now - self.initial_pose_request_started >= 3.0):
                        try:
                            self.initial_pose_future.cancel()
                        except Exception:
                            pass
                        self.initial_pose_future = None
                        self.initial_pose_request_started = None
                        self.initial_pose_sent = False
                        self.get_logger().warning(
                            '[AMCL-READY] KNOWN_POSE request timed out; will retry')
                    return
                try:
                    self.initial_pose_future.result()
                    print('[AMCL-READY] map-bound KNOWN_POSE accepted by AMCL', flush=True)
                except Exception as exc:
                    self.get_logger().warning(
                        f'[AMCL-READY] KNOWN_POSE request failed; will retry: {exc}')
                    self.initial_pose_sent = False
                finally:
                    self.initial_pose_future = None
                    self.initial_pose_request_started = None
                if self.initial_pose_sent:
                    return

            if not self.initial_pose_sent:
                if not self._known_pose_hash_ok():
                    if now - self.last_operator_reminder >= 5.0:
                        self.last_operator_reminder = now
                        self.get_logger().error(
                            f'[AMCL-READY] KNOWN_POSE blocked: configured map SHA256='
                            f'{self.known_hash or "EMPTY"} current={self.current_hash or "EMPTY"}')
                    return
                # Publish to /initialpose instead of calling the service: AMCL
                # subscribes to this topic and the UDP-only transport reliably
                # delivers it (the service call was timing out under FastDDS UDP).
                msg = PoseWithCovarianceStamped()
                # AMCL transforms the initial pose through odom->base. Under Jetson load
                # TF can trail wall time slightly, so use the same conservative backdate as
                # initialpose_stamp_relay instead of stamping a pose just ahead of TF.
                msg.header.stamp = (
                    self.get_clock().now() - Duration(seconds=self.initial_pose_backdate)
                ).to_msg()
                msg.header.frame_id = self.global_frame
                msg.pose.pose.position.x = self.known_x
                msg.pose.pose.position.y = self.known_y
                half = 0.5 * self.known_yaw
                msg.pose.pose.orientation.z = math.sin(half)
                msg.pose.pose.orientation.w = math.cos(half)
                msg.pose.covariance[0] = min(self.max_cov_x, 0.25)
                msg.pose.covariance[7] = min(self.max_cov_y, 0.25)
                msg.pose.covariance[35] = min(
                    self.max_cov_yaw, (math.pi / 12.0) ** 2)
                self.initialpose_pub.publish(msg)
                self.initial_pose_sent = True
                print('[AMCL-READY] map-bound KNOWN_POSE published to /initialpose', flush=True)
                return

            # Pose request was accepted; wait for AMCL pose/TF convergence below.
        elif self.mode == 'GLOBAL_LOCALIZATION' and not self.global_localization_complete:
            if self.global_localization_future is not None:
                if not self.global_localization_future.done():
                    return
                try:
                    self.global_localization_future.result()
                    self.global_localization_complete = True
                    print(
                        '[AMCL-READY] AMCL global-localization request completed; '
                        'forcing no-motion laser updates until convergence', flush=True)
                except Exception as exc:
                    self.global_localization_requested = False
                    self.get_logger().error(
                        f'[AMCL-READY] global-localization service failed; will retry: {exc}')
                finally:
                    self.global_localization_future = None
            elif self.global_localization_client.service_is_ready():
                self.global_localization_requested = True
                self.global_localization_future = self.global_localization_client.call_async(
                    Empty.Request())
                print(
                    f'[AMCL-READY] requested scan-based global localization via '
                    f'{self.global_localization_service}', flush=True)
        elif self.mode == 'OPERATOR' and not self.pose_seen and now - self.last_operator_reminder >= 10.0:
            self.last_operator_reminder = now
            print('[AMCL-READY] WAITING: set 2D Pose Estimate in GUI/RViz; no default (0,0,0) will be sent.', flush=True)

        # Once AMCL has an initial pose, request laser updates while stationary so
        # convergence can be demonstrated without requiring wheel movement.
        can_force_laser_update = self.pose_seen or (
            self.mode == 'GLOBAL_LOCALIZATION' and self.global_localization_complete)
        if can_force_laser_update and self.nomotion_client.service_is_ready():
            if self.nomotion_future is None or self.nomotion_future.done():
                if now - self.nomotion_last >= self.nomotion_period:
                    self.nomotion_last = now
                    self.nomotion_future = self.nomotion_client.call_async(Empty.Request())

    def _convergence(self):
        if len(self.samples) < self.min_samples:
            return False, f'samples={len(self.samples)}/{self.min_samples}'
        window = list(self.samples)[-self.min_samples:]
        cov_ok = all(
            s[3] <= self.max_cov_x and s[4] <= self.max_cov_y and s[5] <= self.max_cov_yaw
            for s in window)
        ref = window[-1]
        max_dist = max(math.hypot(s[0] - ref[0], s[1] - ref[1]) for s in window)
        max_dyaw = max(abs(_wrap(s[2] - ref[2])) for s in window)
        stable = max_dist <= self.stability_m and max_dyaw <= self.stability_yaw
        detail = (
            f'samples={len(window)} cov=({ref[3]:.3f},{ref[4]:.3f},{ref[5]:.3f}) '
            f'spread={max_dist:.3f}m/{max_dyaw:.3f}rad')
        return cov_ok and stable, detail

    def _check(self):
        self._poll_state()
        self._initialize_if_needed()
        converged, conv_detail = self._convergence()
        tf_map_odom = self._tf(self.global_frame, self.odom_frame)
        tf_map_base = self._tf(self.global_frame, self.base_frame)
        hash_ok = self._known_pose_hash_ok()
        status = (
            f'mode={self.mode} operator_override={int(self.operator_pose_requested)} '
            f'map={int(self.map_seen)} active={int(self.amcl_active)} '
            f'pose={int(self.pose_seen)} hash_ok={int(hash_ok)} converged={int(converged)} '
            f'global_init={int(self.global_localization_complete)} '
            f'tf_map_odom={int(tf_map_odom)} tf_map_base={int(tf_map_base)} {conv_detail}')
        if status != self.last_status:
            print('[AMCL-READY] ' + status, flush=True)
            self.last_status = status

        if self.map_seen and self.amcl_active and self.pose_seen and hash_ok and converged and tf_map_odom and tf_map_base:
            self.ready = True
            print('[AMCL-READY] READY — initial pose + AMCL convergence + global TF verified', flush=True)
            self.get_logger().info('[AMCL-READY] READY')
            return

        now = time.monotonic()
        if now - self.start_wall >= self.timeout_sec and now - self.last_timeout_report >= 10.0:
            self.last_timeout_report = now
            self.get_logger().error(
                f'[AMCL-READY] NOT READY after {now - self.start_wall:.1f}s; '
                f'Nav2 remains blocked. {status}')


def main():
    rclpy.init()
    node = AmclReadyGate()
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
