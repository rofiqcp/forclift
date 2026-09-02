#!/usr/bin/env python3
"""ROS 2 <-> Qt bridge for the Autonomous Vehicle Interface."""
from __future__ import annotations

import math
import threading
import time
import traceback
from collections import defaultdict, deque
from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
try:
    from rclpy.signals import SignalHandlerOptions
except Exception:  # compatibility with older rclpy builds
    SignalHandlerOptions = None
from sensor_msgs.msg import BatteryState, CameraInfo, Image, Imu, LaserScan, Temperature
from std_msgs.msg import Bool, Float32, Float64, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

try:
    from yolo_obstacle_detection_ros2.msg import AlignmentState, ObstacleArray
except Exception:  # package may be unavailable in partial/offline environments
    AlignmentState = ObstacleArray = None


MONITORED_TOPICS = (
    "/scan_nav", "/scan_safety", "/lidar/safety_healthy", "/lidar/safety_health", "/lidar/status",
    "/imu/data", "/imu/gyro", "/imu/accel", "/imu/mag", "/imu/euler", "/imu/status",
    "/lidar/odom", "/scan_match_quality", "/odometry/filtered",
    "/map", "/nav_map", "/amcl_pose",
    "/global_costmap/costmap", "/local_costmap/costmap",
    "/smac_plan", "/navigation/planner_status", "/transformed_global_plan", "/trajectories",
    "/goal_pose", "/initialpose_safe", "/navigate_to_pose/_action/status",
    "/cmd_vel_nav_raw", "/cmd_vel_nav_smoothed", "/cmd_vel_collision_safe", "/cmd_vel", "/cmd_vel/actuator",
    "/sensor_guard/healthy", "/system/autonomy_motion_allowed", "/system/manual_motion_allowed", "/safety/estop",
    "/mapping/map_valid", "/mapping/map_stats",
    "/camera/color/image_raw", "/camera/color/camera_info", "/camera/color/status",
    "/obstacle_detection/obstacles", "/obstacle_detection/visualization", "/obstacle_detection/status",
    "/obstacle_detection/performance",
    "/fork_alignment/state", "/fork_alignment/image",
    "/winch/connected", "/winch/port", "/winch/state", "/winch/top_limit", "/winch/bottom_limit",
    "/winch/pwm_pct", "/winch/direction", "/winch/servo_deg", "/winch/raw",
    "/esc/odom", "/esc/speed", "/esc/ready", "/esc/armed", "/esc/feedback_valid",
    "/esc/status", "/esc/drive/connected", "/esc/steer/connected",
    "/esc/drive_target_mps", "/esc/drive_actual_mps", "/esc/steering_target_rad",
    "/esc/steering_actual_rad", "/esc/yaw_rate_actual_rps", "/esc/battery", "/esc/temperature",
    "/esc/mux/status", "/esc/mux/active_source", "/esc/mux/selected",
)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def qimage_from_ros_image(msg: Image) -> Optional[QImage]:
    """Convert the newest ROS image outside the rclpy callback thread.

    The Astra driver publishes ``bgr8``. Qt 5.15 provides Format_BGR888, which
    lets the GUI consume that buffer without an additional full-frame
    ``rgbSwapped()`` pass. We still take one owned QImage copy because the ROS
    message can be released immediately after this worker returns.
    """
    try:
        enc = msg.encoding.lower()
        raw = bytes(msg.data)
        if enc in ("rgb8", "8uc3"):
            return QImage(raw, msg.width, msg.height, msg.step, QImage.Format_RGB888).copy()
        if enc == "bgr8":
            bgr_format = getattr(QImage, "Format_BGR888", None)
            if bgr_format is not None:
                return QImage(raw, msg.width, msg.height, msg.step, bgr_format).copy()
            # Compatibility fallback for older Qt builds.
            return QImage(raw, msg.width, msg.height, msg.step, QImage.Format_RGB888).rgbSwapped().copy()
        if enc in ("mono8", "8uc1"):
            return QImage(raw, msg.width, msg.height, msg.step, QImage.Format_Grayscale8).copy()
        if enc == "rgba8":
            return QImage(raw, msg.width, msg.height, msg.step, QImage.Format_RGBA8888).copy()
        if enc == "bgra8":
            # QImage::Format_ARGB32 is BGRA byte order on little-endian Linux.
            return QImage(raw, msg.width, msg.height, msg.step, QImage.Format_ARGB32).copy()
    except Exception:
        return None
    return None


class RosBridge(QObject):
    health = pyqtSignal(object)
    telemetry = pyqtSignal(str, object, float)
    pose = pyqtSignal(str, float, float, float)
    scan_points = pyqtSignal(object)
    occupancy_grid = pyqtSignal(str, object)
    path = pyqtSignal(str, object)
    trajectories = pyqtSignal(object)
    image = pyqtSignal(str, object)
    tf_status = pyqtSignal(object)
    log = pyqtSignal(str)
    ros_state = pyqtSignal(bool)
    active_map = pyqtSignal(str)
    system_state = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.node: Optional[BridgeNode] = None
        # Only subscribe to image streams needed by the currently visible GUI page.
        # This is more important than merely skipping QImage conversion: a hidden
        # 1280x720 ROS Image subscription still incurs DDS deserialization/copies in
        # Python and can steal CPU/memory bandwidth from camera + perception.
        self._image_interests: set[str] = set()
        self._image_interest_lock = threading.Lock()

        # Latest-only image handoff. The rclpy callback stores a reference to the
        # newest Image and returns immediately; a dedicated worker performs the
        # expensive QImage conversion at a capped preview rate.
        self._image_pending: Dict[str, Tuple[int, Image]] = {}
        self._image_pending_lock = threading.Lock()
        self._image_event = threading.Event()
        self._image_worker_stop = threading.Event()
        self._image_worker: Optional[threading.Thread] = None
        self._image_enqueue_seq = 0
        self._image_last_emit: Dict[str, float] = defaultdict(float)
        self._image_failures: Dict[str, int] = defaultdict(int)

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._image_worker_stop.clear()
        if not self._image_worker or not self._image_worker.is_alive():
            self._image_worker = threading.Thread(
                target=self._run_image_worker, name="agv-gui-image", daemon=True)
            self._image_worker.start()
        self._thread = threading.Thread(target=self._run, name="agv-gui-ros", daemon=True)
        self._thread.start()

    def _run(self):
        node = None
        try:
            if not rclpy.ok():
                # This bridge is intentionally started from a background thread
                # after the Qt window is visible. Installing Python signal
                # handlers from a non-main thread is unsafe; the ros2 launch
                # process already owns SIGINT/SIGTERM lifecycle handling.
                if SignalHandlerOptions is not None:
                    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
                else:
                    rclpy.init(args=None)
            node = BridgeNode(self)
            self.node = node
            self.ros_state.emit(True)
            self.log.emit("ROS bridge ONLINE: autonomous_vehicle_interface node created")

            # One malformed/old custom message must never take the complete GUI
            # bridge offline. rclpy.spin_once propagates callback exceptions, so
            # contain them here and keep processing the rest of the ROS graph.
            last_error_text = ""
            last_error_time = 0.0
            while rclpy.ok() and not self._stop.is_set():
                try:
                    rclpy.spin_once(node, timeout_sec=0.05)
                except Exception as exc:
                    now = time.monotonic()
                    text = f"{type(exc).__name__}: {exc}"
                    if text != last_error_text or (now - last_error_time) >= 3.0:
                        self.log.emit(
                            "ROS callback error contained; bridge remains ONLINE: " + text +
                            "\n" + traceback.format_exc(limit=5))
                        last_error_text = text
                        last_error_time = now
                    time.sleep(0.02)
        except Exception as exc:
            self.log.emit(
                f"ROS bridge failed during initialization: {type(exc).__name__}: {exc}\n" +
                traceback.format_exc(limit=8))
        finally:
            self.ros_state.emit(False)
            self.node = None
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass

    @property
    def image_interest(self) -> Tuple[str, ...]:
        with self._image_interest_lock:
            return tuple(sorted(self._image_interests))

    def wants_image(self, topic: str) -> bool:
        with self._image_interest_lock:
            return topic in self._image_interests

    def set_image_interest(self, topic):
        if topic is None:
            wanted = set()
        elif isinstance(topic, (tuple, list, set)):
            wanted = {str(value) for value in topic if value}
        else:
            wanted = {str(topic)}
        with self._image_interest_lock:
            self._image_interests = wanted
        # Drop queued full-resolution frames from pages that are no longer
        # visible so they cannot consume memory or be converted after a tab switch.
        with self._image_pending_lock:
            self._image_pending = {
                key: value for key, value in self._image_pending.items() if key in wanted
            }
        self._image_event.set()

    def queue_image(self, topic: str, msg: Image):
        if not self.wants_image(topic):
            return
        with self._image_pending_lock:
            self._image_enqueue_seq += 1
            self._image_pending[topic] = (self._image_enqueue_seq, msg)
        self._image_event.set()

    def _run_image_worker(self):
        last_processed: Dict[str, int] = defaultdict(int)
        while not self._image_worker_stop.is_set():
            self._image_event.wait(timeout=0.05)
            self._image_event.clear()
            if self._image_worker_stop.is_set():
                break

            with self._image_pending_lock:
                pending = list(self._image_pending.items())

            now = time.monotonic()
            for topic, (seq, msg) in pending:
                if seq <= last_processed[topic] or not self.wants_image(topic):
                    continue
                # Preview conversion is capped to 30 FPS. Newer frames replace
                # older queued frames instead of accumulating callback backlog.
                if now - self._image_last_emit[topic] < (1.0 / 30.0):
                    continue

                qimg = qimage_from_ros_image(msg)
                last_processed[topic] = seq
                self._image_last_emit[topic] = now
                if qimg is not None:
                    if self._image_failures[topic] > 0:
                        self.log.emit(
                            f"GUI image conversion recovered for {topic} after "
                            f"{self._image_failures[topic]} failure(s)")
                    self._image_failures[topic] = 0
                    self.image.emit(topic, qimg)
                else:
                    self._image_failures[topic] += 1
                    failures = self._image_failures[topic]
                    if failures in (1, 30, 150):
                        self.log.emit(
                            f"GUI image conversion failed for {topic}: encoding={msg.encoding} "
                            f"size={msg.width}x{msg.height} step={msg.step} failures={failures}")

                with self._image_pending_lock:
                    queued = self._image_pending.get(topic)
                    if queued is not None and queued[0] == seq:
                        self._image_pending.pop(topic, None)

    def shutdown(self):
        self._stop.set()
        self._image_worker_stop.set()
        self._image_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._image_worker and self._image_worker.is_alive():
            self._image_worker.join(timeout=2.0)

    def publish_goal(self, x: float, y: float, yaw: float):
        if self.node is not None:
            self.node.publish_goal(x, y, yaw)

    def publish_initial_pose(self, x: float, y: float, yaw: float, covariance_xy: float = 0.25,
                             covariance_yaw: float = 0.0685):
        if self.node is not None:
            self.node.publish_initial_pose(x, y, yaw, covariance_xy, covariance_yaw)

    def cancel_navigation(self):
        if self.node is not None:
            self.node.cancel_navigation()

    def publish_winch_command(self, command: str):
        if self.node is not None:
            self.node.publish_winch_command(command)


class BridgeNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__("autonomous_vehicle_interface")
        self.bridge = bridge
        self._times: Dict[str, deque] = defaultdict(lambda: deque(maxlen=120))
        self._counts: Dict[str, int] = defaultdict(int)
        self._details: Dict[str, str] = {}
        self._latest: Dict[str, float] = {}
        self._map_odom: Optional[Tuple[float, float, float]] = None
        self._last_health_emit = 0.0
        self._known_topics = tuple(MONITORED_TOPICS)
        self._last_goal_monotonic: Optional[float] = None
        self._last_goal_xy: Optional[Tuple[float, float]] = None
        self._last_cmd_seen: Dict[str, float] = {}
        self._last_amcl_stamp_ns: Optional[int] = None
        self._image_qos = None
        self._image_subs: Dict[str, Any] = {}

        # V32: coalesce high-rate ROS callbacks before crossing into the Qt
        # main thread.  The previous bridge emitted every IMU/odom/cmd_vel/
        # pose/costmap update as a queued Qt signal.  With the autonomous stack
        # running this can create hundreds of GUI events per second and starve
        # the Qt event loop, making the window look frozen until it is killed.
        self._pending_telemetry: Dict[str, Tuple[Dict[str, Any], float]] = {}
        self._pending_pose: Dict[str, Tuple[float, float, float]] = {}
        self._pending_scan: Optional[Sequence[Tuple[float, float]]] = None
        self._pending_grids: Dict[str, Any] = {}
        self._pending_paths: Dict[str, Sequence[Tuple[float, float]]] = {}
        self._pending_trajectories: Optional[Sequence[Sequence[Tuple[float, float]]]] = None
        self._last_grid_metrics: Dict[str, float] = defaultdict(float)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        reliable = QoSProfile(depth=10)
        best_effort = QoSProfile(depth=10)
        best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
        best_effort.durability = DurabilityPolicy.VOLATILE
        best_effort.history = HistoryPolicy.KEEP_LAST
        transient = QoSProfile(depth=1)
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.history = HistoryPolicy.KEEP_LAST
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        image_qos.history = HistoryPolicy.KEEP_LAST
        self._image_qos = image_qos

        # Sensor / localization. /scan_nav is the canonical navigation stream in
        # V50+; /scan_safety is monitored separately so safety health cannot be
        # confused with the map-overlay scan.
        self.create_subscription(LaserScan, "/scan_nav", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_safety", self._scan_safety_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, "/lidar/safety_healthy", self._bool_cb("/lidar/safety_healthy", "lidar_safety"), transient)
        self.create_subscription(String, "/lidar/safety_health",
                                 self._status_string_cb("lidar_safety", "health_text", "/lidar/safety_health"), transient)
        self.create_subscription(String, "/lidar/status", self._status_string_cb("lidar", "status", "/lidar/status"), reliable)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, reliable)
        self.create_subscription(Imu, "/imu/gyro", self._imu_aux_cb("imu_gyro", "/imu/gyro"), reliable)
        self.create_subscription(Imu, "/imu/accel", self._imu_aux_cb("imu_accel", "/imu/accel"), reliable)
        self.create_subscription(Vector3Stamped, "/imu/mag", self._mag_cb, reliable)
        self.create_subscription(Vector3Stamped, "/imu/euler", self._euler_cb, reliable)
        self.create_subscription(String, "/imu/status", self._status_string_cb("imu_status", "status", "/imu/status"), reliable)
        self.create_subscription(Odometry, "/lidar/odom", self._odom_cb("lidar_odom", "/lidar/odom"), reliable)
        self.create_subscription(Float32, "/scan_match_quality", self._float_cb("scan_match_quality", "/scan_match_quality"), reliable)
        self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb("ekf", "/odometry/filtered"), reliable)
        # AMCL on Humble is normally RELIABLE + TRANSIENT_LOCAL.  Subscribe with
        # the canonical transient profile so a GUI opened after localization
        # receives the latest stationary pose immediately.  Keep a volatile
        # BEST_EFFORT fallback for non-standard AMCL builds. _amcl_cb deduplicates
        # the two readers by ROS header stamp.
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, transient)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, best_effort)

        # Maps / planner
        for topic in ("/map", "/nav_map", "/global_costmap/costmap", "/local_costmap/costmap"):
            self.create_subscription(OccupancyGrid, topic, self._grid_cb(topic), transient)
        self.create_subscription(Bool, "/mapping/map_valid", self._bool_cb("/mapping/map_valid", "mapping"), transient)
        self.create_subscription(String, "/mapping/map_stats", self._status_string_cb("mapping", "map_stats", "/mapping/map_stats"), transient)
        self.create_subscription(Path, "/smac_plan", self._path_cb("/smac_plan"), transient)
        self.create_subscription(String, "/navigation/planner_status",
                                 self._status_string_cb("planner", "status", "/navigation/planner_status"), reliable)
        # MPPI debug publishers may be BEST_EFFORT depending on Nav2 build.
        # BEST_EFFORT subscribers also match RELIABLE writers and avoid a silent
        # GUI when the controller visualization QoS differs.
        self.create_subscription(Path, "/transformed_global_plan", self._path_cb("/transformed_global_plan"), best_effort)
        self.create_subscription(MarkerArray, "/trajectories", self._trajectories_cb, best_effort)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_observed_cb, reliable)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._nav_status_cb, reliable)

        # Command pipeline
        for topic in ("/cmd_vel_nav_raw", "/cmd_vel_nav_smoothed", "/cmd_vel_collision_safe", "/cmd_vel", "/cmd_vel/actuator", "/esc/mux/selected"):
            self.create_subscription(Twist, topic, self._twist_cb(topic), reliable)
        self.create_subscription(Bool, "/sensor_guard/healthy", self._bool_cb("/sensor_guard/healthy", "safety"), reliable)
        self.create_subscription(Bool, "/system/autonomy_motion_allowed", self._bool_cb("/system/autonomy_motion_allowed", "safety"), reliable)
        self.create_subscription(Bool, "/system/manual_motion_allowed", self._bool_cb("/system/manual_motion_allowed", "safety"), reliable)
        self.create_subscription(Bool, "/safety/estop", self._bool_cb("/safety/estop", "safety"), transient)

        # ESC telemetry
        for topic in ("/esc/speed", "/esc/drive_target_mps", "/esc/drive_actual_mps",
                      "/esc/steering_target_rad", "/esc/steering_actual_rad", "/esc/yaw_rate_actual_rps"):
            # ESC numeric telemetry is BEST_EFFORT in esc_driver_node.cpp. A
            # reliable subscriber is incompatible and silently loses the data.
            self.create_subscription(Float64, topic, self._float64_cb(topic), best_effort)
        for topic in ("/esc/ready", "/esc/armed", "/esc/feedback_valid", "/esc/drive/connected", "/esc/steer/connected"):
            self.create_subscription(Bool, topic, self._bool_cb(topic, "esc_state"), transient)
        for topic in ("/esc/status", "/esc/mux/status", "/esc/mux/active_source"):
            self.create_subscription(String, topic, self._status_string_cb("esc_state", topic, topic), transient)
        self.create_subscription(Odometry, "/esc/odom", self._odom_cb("esc_odom", "/esc/odom"), reliable)
        self.create_subscription(BatteryState, "/esc/battery", self._battery_cb, reliable)
        self.create_subscription(Temperature, "/esc/temperature", self._temperature_cb, reliable)

        # Electric winch telemetry. The bridge node publishes these as reliable,
        # transient-local values so the GUI receives the last known state when
        # the Winch tab is opened after startup.
        self.create_subscription(Bool, "/winch/connected", self._winch_bool_cb("connected", "/winch/connected"), transient)
        self.create_subscription(String, "/winch/port", self._winch_string_cb("port", "/winch/port"), transient)
        self.create_subscription(String, "/winch/state", self._winch_string_cb("state", "/winch/state"), transient)
        self.create_subscription(Bool, "/winch/top_limit", self._winch_bool_cb("top_limit", "/winch/top_limit"), transient)
        self.create_subscription(Bool, "/winch/bottom_limit", self._winch_bool_cb("bottom_limit", "/winch/bottom_limit"), transient)
        self.create_subscription(Float64, "/winch/pwm_pct", self._winch_float_cb("pwm_pct", "/winch/pwm_pct"), transient)
        self.create_subscription(Int32, "/winch/direction", self._winch_int_cb("direction", "/winch/direction"), transient)
        self.create_subscription(Float64, "/winch/servo_deg", self._winch_float_cb("servo_deg", "/winch/servo_deg"), transient)
        self.create_subscription(String, "/winch/raw", self._winch_string_cb("raw", "/winch/raw"), reliable)

        # Camera and processed views. Image subscriptions are demand-driven and
        # created by _sync_image_subscriptions() only for the visible GUI page.
        # Lightweight CameraInfo/status topics stay subscribed continuously.
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._camera_info_cb, best_effort)
        self.create_subscription(String, "/camera/color/status",
                                 self._status_string_cb("camera_status", "backend", "/camera/color/status"), transient)
        self.create_subscription(String, "/obstacle_detection/status",
                                 self._status_string_cb("yolo_status", "backend", "/obstacle_detection/status"), transient)
        self.create_subscription(String, "/obstacle_detection/performance",
                                 self._status_string_cb("yolo_performance", "performance", "/obstacle_detection/performance"), best_effort)
        if ObstacleArray is not None:
            self.create_subscription(ObstacleArray, "/obstacle_detection/obstacles", self._obstacles_cb, reliable)
        if AlignmentState is not None:
            # Alignment state is latched by the producer. Match transient-local
            # QoS so opening the GUI after startup still receives the latest state.
            self.create_subscription(AlignmentState, "/fork_alignment/state", self._alignment_cb, transient)

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose_safe", 10)
        self.winch_command_pub = self.create_publisher(String, "/winch/command", 10)
        self.cancel_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.map_param_client = self.create_client(GetParameters, "/map_server/get_parameters")
        self._map_param_future = None

        # UI-facing ROS updates are deliberately bounded.  5 Hz is enough for
        # tables/maps/status while preventing Qt queued-signal backlog.
        self.create_timer(0.20, self._flush_ui_updates)
        self.create_timer(0.50, self._poll_tf)
        self.create_timer(0.5, self._emit_health)
        self.create_timer(1.0, self._emit_system_state)
        self.create_timer(2.0, self._query_active_map)
        self.create_timer(0.20, self._sync_image_subscriptions)

    def _queue_telemetry(self, source: str, values: Dict[str, Any], timestamp: Optional[float] = None):
        """Merge latest telemetry by source; emitted to Qt at a bounded rate."""
        stamp = time.time() if timestamp is None else float(timestamp)
        current = self._pending_telemetry.get(source)
        if current is None:
            merged: Dict[str, Any] = dict(values)
        else:
            merged = dict(current[0])
            merged.update(values)
        self._pending_telemetry[source] = (merged, stamp)

    def _queue_pose(self, source: str, x: float, y: float, yaw: float):
        self._pending_pose[source] = (float(x), float(y), float(yaw))

    def _queue_scan(self, points: Sequence[Tuple[float, float]]):
        self._pending_scan = points

    def _queue_grid(self, topic: str, msg: Any):
        self._pending_grids[topic] = msg

    def _queue_path(self, topic: str, points: Sequence[Tuple[float, float]]):
        self._pending_paths[topic] = points

    def _queue_trajectories(self, trajectories: Sequence[Sequence[Tuple[float, float]]]):
        self._pending_trajectories = trajectories

    def _flush_ui_updates(self):
        """Emit only the newest high-rate data to Qt (max 5 Hz)."""
        telemetry = self._pending_telemetry
        poses = self._pending_pose
        scan = self._pending_scan
        grids = self._pending_grids
        paths = self._pending_paths
        trajectories = self._pending_trajectories

        self._pending_telemetry = {}
        self._pending_pose = {}
        self._pending_scan = None
        self._pending_grids = {}
        self._pending_paths = {}
        self._pending_trajectories = None

        for source, (values, stamp) in telemetry.items():
            self.bridge.telemetry.emit(source, values, stamp)
        for source, (x, y, yaw) in poses.items():
            self.bridge.pose.emit(source, x, y, yaw)
        if scan is not None:
            self.bridge.scan_points.emit(scan)
        for topic, msg in grids.items():
            self.bridge.occupancy_grid.emit(topic, msg)
        for topic, points in paths.items():
            self.bridge.path.emit(topic, points)
        if trajectories is not None:
            self.bridge.trajectories.emit(trajectories)

    def _sync_image_subscriptions(self):
        """Subscribe only to image topics required by the visible page.

        DDS still reports the topics through graph discovery, so system status
        remains available without continuously deserializing three 1280x720
        image streams in Python. This keeps GUI load from reducing perception FPS.
        """
        allowed = {
            "/camera/color/image_raw",
            "/obstacle_detection/visualization",
            "/fork_alignment/image",
        }
        wanted = set(self.bridge.image_interest) & allowed

        for topic in list(self._image_subs):
            if topic in wanted:
                continue
            try:
                self.destroy_subscription(self._image_subs[topic])
            except Exception as exc:
                self.bridge.log.emit(f"GUI image unsubscribe notice {topic}: {exc}")
            self._image_subs.pop(topic, None)

        for topic in sorted(wanted):
            if topic in self._image_subs:
                continue
            try:
                self._image_subs[topic] = self.create_subscription(
                    Image, topic, self._image_cb(topic), self._image_qos)
            except Exception as exc:
                self.bridge.log.emit(f"GUI image subscribe notice {topic}: {exc}")

    # ---------------- health ----------------
    def _mark(self, topic: str, detail: str = ""):
        now = time.monotonic()
        self._latest[topic] = now
        self._times[topic].append(now)
        self._counts[topic] += 1
        if detail:
            self._details[topic] = detail

    def _emit_health(self):
        now = time.monotonic()
        payload: Dict[str, Dict[str, Any]] = {}
        all_topics = set(self._known_topics) | set(self._counts) | set(self._latest)
        for topic in sorted(all_topics):
            times = self._times[topic]
            hz = 0.0
            if len(times) >= 2 and times[-1] > times[0]:
                hz = (len(times) - 1) / (times[-1] - times[0])
            seen = topic in self._latest
            payload[topic] = {
                "age": (now - self._latest[topic]) if seen else float("inf"),
                "hz": hz,
                "count": self._counts.get(topic, 0),
                "detail": self._details.get(topic, ""),
                "seen": seen,
            }
        self.bridge.health.emit(payload)

        # V59: continuously export a compact perception summary to the GUI.
        # This is derived only from real received ROS traffic (or explicit zero
        # counts), so the Perception Validation page never stays ambiguous.
        def _m(topic):
            info = payload.get(topic, {})
            age = float(info.get("age", float("inf")))
            return {
                "count": float(info.get("count", 0) or 0),
                "hz": float(info.get("hz", 0.0) or 0.0),
                "age": age if math.isfinite(age) else -1.0,
            }
        cam = _m("/camera/color/image_raw")
        cam_info = _m("/camera/color/camera_info")
        obs = _m("/obstacle_detection/obstacles")
        viz = _m("/obstacle_detection/visualization")
        status = _m("/obstacle_detection/status")
        perf = _m("/obstacle_detection/performance")
        align = _m("/fork_alignment/state")
        align_img = _m("/fork_alignment/image")
        perception_ready = (cam["count"] > 0 and cam["age"] >= 0.0 and cam["age"] < 2.0 and
                            obs["count"] > 0 and obs["age"] >= 0.0 and obs["age"] < 2.0)
        self._queue_telemetry("perception", {
            "perception_ready": 1.0 if perception_ready else 0.0,
            "camera_count": cam["count"], "camera_hz": cam["hz"], "camera_age_s": cam["age"],
            "camera_info_count": cam_info["count"],
            "obstacles_count_msgs": obs["count"], "obstacles_hz": obs["hz"], "obstacles_age_s": obs["age"],
            "visualization_hz": viz["hz"], "visualization_age_s": viz["age"],
            "yolo_status_count": status["count"], "yolo_performance_hz": perf["hz"],
            "alignment_state_hz": align["hz"], "alignment_state_age_s": align["age"],
            "alignment_image_hz": align_img["hz"],
        }, time.time())

    def _emit_system_state(self):
        """Publish a lightweight ROS graph snapshot for process-level status cards.

        Some Nav2 nodes (planner/controller/smoother) may be perfectly healthy
        while idle and therefore publish no data.  Topic-only health incorrectly
        labelled those subsystems UNKNOWN.  The graph snapshot distinguishes
        "node present / waiting for data" from a genuinely missing process.
        """
        try:
            pairs = self.get_node_names_and_namespaces()
            nodes = []
            for name, ns in pairs:
                ns = (ns or "/").rstrip("/")
                nodes.append(f"{ns}/{name}" if ns else f"/{name}")
            topic_names = [name for name, _types in self.get_topic_names_and_types()]
            self.bridge.system_state.emit({
                "nodes": sorted(set(nodes)),
                "topics": sorted(set(topic_names)),
                "timestamp": time.time(),
            })
        except Exception as exc:
            self.bridge.log.emit(f"ROS graph snapshot unavailable: {exc}")

    # ---------------- callbacks ----------------
    def _scan_cb(self, msg: LaserScan):
        self._mark("/scan_nav", msg.header.frame_id)
        points: List[Tuple[float, float]] = []
        try:
            tf = self.tf_buffer.lookup_transform("map", msg.header.frame_id or "lidar_link", rclpy.time.Time())
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = yaw_from_quaternion(tf.transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
            angle = msg.angle_min
            stride = max(1, len(msg.ranges) // 1000)
            for index, radius in enumerate(msg.ranges):
                if index % stride:
                    angle += msg.angle_increment
                    continue
                if math.isfinite(radius) and msg.range_min <= radius <= msg.range_max:
                    lx = radius * math.cos(angle)
                    ly = radius * math.sin(angle)
                    points.append((tx + c * lx - s * ly, ty + s * lx + c * ly))
                angle += msg.angle_increment
        except Exception:
            pass
        if points:
            self._queue_scan(points)
        valid_ranges = [float(r) for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        valid = len(valid_ranges)
        total = len(msg.ranges)
        values = {
            "valid_bins": float(valid),
            "scan_bins": float(total),
            "invalid_bins": float(max(0, total - valid)),
            "valid_ratio_pct": (100.0 * valid / total) if total else 0.0,
            "invalid_ratio_pct": (100.0 * (total - valid) / total) if total else 0.0,
            "range_min_cfg": float(msg.range_min),
            "range_max_cfg": float(msg.range_max),
        }
        if valid_ranges:
            mean = sum(valid_ranges) / len(valid_ranges)
            var = sum((r - mean) ** 2 for r in valid_ranges) / len(valid_ranges)
            values.update({
                "range_min_measured": min(valid_ranges),
                "range_max_measured": max(valid_ranges),
                "range_mean": mean,
                "range_std": math.sqrt(max(0.0, var)),
            })
        self._queue_telemetry("lidar", values, time.time())

    def _scan_safety_cb(self, msg: LaserScan):
        self._mark("/scan_safety", msg.header.frame_id)
        total = len(msg.ranges)
        valid = sum(1 for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max)
        self._queue_telemetry("lidar_safety", {
            "safety_scan_bins": float(total),
            "safety_valid_bins": float(valid),
            "safety_valid_ratio_pct": (100.0 * valid / total) if total else 0.0,
        }, time.time())

    def _imu_cb(self, msg: Imu):
        self._mark("/imu/data", msg.header.frame_id)
        a, g = msg.linear_acceleration, msg.angular_velocity
        self._queue_telemetry("imu", {
            "ax": a.x, "ay": a.y, "az": a.z,
            "gx": g.x, "gy": g.y, "gz": g.z,
            "orientation_cov_x": float(msg.orientation_covariance[0]),
            "orientation_cov_y": float(msg.orientation_covariance[4]),
            "orientation_cov_z": float(msg.orientation_covariance[8]),
        }, time.time())

    def _imu_aux_cb(self, source: str, topic: str):
        def cb(msg: Imu):
            # /imu/data already carries accel + gyro at the source rate. Keep
            # the dedicated aux topics in health monitoring, but do not inject
            # duplicate numeric samples into the same plot buffer.
            self._mark(topic, msg.header.frame_id)
        return cb

    def _mag_cb(self, msg: Vector3Stamped):
        self._mark("/imu/mag", msg.header.frame_id)
        self._queue_telemetry("imu_mag", {"mx": msg.vector.x * 1e6, "my": msg.vector.y * 1e6, "mz": msg.vector.z * 1e6}, time.time())

    def _euler_cb(self, msg: Vector3Stamped):
        self._mark("/imu/euler", msg.header.frame_id)
        self._queue_telemetry("imu_euler", {"roll": msg.vector.x, "pitch": msg.vector.y, "yaw": msg.vector.z}, time.time())

    def _odom_cb(self, source: str, topic: str):
        def cb(msg: Odometry):
            self._mark(topic, f"{msg.header.frame_id}->{msg.child_frame_id}")
            p = msg.pose.pose.position
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            x, y = p.x, p.y
            if msg.header.frame_id == "odom" and self._map_odom is not None:
                x, y, yaw = self._transform_pose(self._map_odom, x, y, yaw)
            self._queue_pose(source, float(x), float(y), float(yaw))
            pcov = msg.pose.covariance
            tcov = msg.twist.covariance
            self._queue_telemetry(source, {
                "x": x, "y": y, "yaw": yaw,
                "vx": msg.twist.twist.linear.x,
                "vy": msg.twist.twist.linear.y,
                "wz": msg.twist.twist.angular.z,
                "pose_cov_x": float(pcov[0]),
                "pose_cov_y": float(pcov[7]),
                "pose_cov_yaw": float(pcov[35]),
                "twist_cov_vx": float(tcov[0]),
                "twist_cov_wz": float(tcov[35]),
            }, time.time())
        return cb

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns > 0 and stamp_ns == self._last_amcl_stamp_ns:
            return
        if stamp_ns > 0:
            self._last_amcl_stamp_ns = stamp_ns
        self._mark("/amcl_pose", msg.header.frame_id)
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._queue_pose("amcl", float(p.x), float(p.y), float(yaw))
        cov = msg.pose.covariance
        self._queue_telemetry("amcl", {"x": p.x, "y": p.y, "yaw": yaw, "cov_x": cov[0], "cov_y": cov[7], "cov_yaw": cov[35]}, time.time())

    def _grid_cb(self, topic: str):
        def cb(msg: OccupancyGrid):
            self._mark(topic, msg.header.frame_id)
            self._queue_grid(topic, msg)
            data = list(msg.data)
            total = len(data)
            occupied = sum(1 for v in data if v >= 65)
            unknown = sum(1 for v in data if v < 0)
            free = max(0, total - occupied - unknown)
            source = "global_costmap" if topic == "/global_costmap/costmap" else (
                "local_costmap" if topic == "/local_costmap/costmap" else "map_live")
            self._queue_telemetry(source, {
                "width_cells": float(msg.info.width),
                "height_cells": float(msg.info.height),
                "resolution": float(msg.info.resolution),
                "width_m": float(msg.info.width * msg.info.resolution),
                "height_m": float(msg.info.height * msg.info.resolution),
                "occupied_pct": (100.0 * occupied / total) if total else 0.0,
                "free_pct": (100.0 * free / total) if total else 0.0,
                "unknown_pct": (100.0 * unknown / total) if total else 0.0,
            }, time.time())
        return cb

    def _path_cb(self, topic: str):
        def cb(msg: Path):
            self._mark(topic, msg.header.frame_id)
            points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self._queue_path(topic, points)
            length = 0.0
            heading_change = 0.0
            max_curvature = 0.0
            reverse_segments = 0
            previous_heading = None
            for idx, (a, b) in enumerate(zip(points, points[1:])):
                ds = math.hypot(b[0] - a[0], b[1] - a[1])
                length += ds
                seg_heading = math.atan2(b[1] - a[1], b[0] - a[0]) if ds > 1e-9 else previous_heading
                if seg_heading is not None and previous_heading is not None:
                    dtheta = math.atan2(math.sin(seg_heading - previous_heading), math.cos(seg_heading - previous_heading))
                    heading_change += abs(dtheta)
                    if ds > 1e-6:
                        max_curvature = max(max_curvature, abs(dtheta) / ds)
                previous_heading = seg_heading
                try:
                    pyaw = yaw_from_quaternion(msg.poses[idx].pose.orientation)
                    if ds > 1e-6 and math.cos(seg_heading - pyaw) < 0.0:
                        reverse_segments += 1
                except Exception:
                    pass
            source = "planner" if topic == "/smac_plan" else "mppi_plan"
            values = {
                "path_length": length,
                "pose_count": float(len(points)),
                "heading_change_deg": math.degrees(heading_change),
                "reverse_segments": float(reverse_segments),
                "max_curvature_1pm": max_curvature,
            }
            if topic == "/smac_plan" and self._last_goal_monotonic is not None:
                values["planning_time_s"] = max(0.0, time.monotonic() - self._last_goal_monotonic)
            self._queue_telemetry(source, values, time.time())
        return cb

    def _trajectories_cb(self, msg: MarkerArray):
        self._mark("/trajectories", "MarkerArray")
        lines: List[List[Tuple[float, float]]] = []
        for marker in msg.markers[:120]:
            pts = [(p.x, p.y) for p in marker.points]
            if pts:
                lines.append(pts)
        self._queue_trajectories(lines)
        point_count = sum(len(line) for line in lines)
        self._queue_telemetry("mppi", {
            "trajectory_count": float(len(lines)),
            "trajectory_point_count": float(point_count),
        }, time.time())

    def _twist_cb(self, topic: str):
        def cb(msg: Twist):
            now_mono = time.monotonic()
            self._mark(topic, "Twist")
            chain = ["/cmd_vel_nav_raw", "/cmd_vel_nav_smoothed", "/cmd_vel_collision_safe", "/cmd_vel", "/cmd_vel/actuator"]
            values = {
                f"{topic}:vx": msg.linear.x,
                f"{topic}:wz": msg.angular.z,
            }
            try:
                idx = chain.index(topic)
                if idx > 0 and chain[idx - 1] in self._last_cmd_seen:
                    values[f"{topic}:stage_latency_ms"] = max(0.0, (now_mono - self._last_cmd_seen[chain[idx - 1]]) * 1000.0)
            except ValueError:
                pass
            self._last_cmd_seen[topic] = now_mono
            self._queue_telemetry("cmd", values, time.time())
        return cb

    def _float_cb(self, name: str, topic: str):
        def cb(msg: Float32):
            self._mark(topic, "Float32")
            self._queue_telemetry(name, {name: float(msg.data)}, time.time())
        return cb

    def _float64_cb(self, topic: str):
        def cb(msg: Float64):
            self._mark(topic, "Float64")
            self._queue_telemetry("esc", {topic: float(msg.data)}, time.time())
        return cb

    def _bool_cb(self, topic: str, source: str = "esc_state"):
        def cb(msg: Bool):
            self._mark(topic, "true" if msg.data else "false")
            self._queue_telemetry(source, {topic: 1.0 if msg.data else 0.0}, time.time())
        return cb

    def _status_string_cb(self, source: str, key: str, topic: str):
        def cb(msg: String):
            text = str(msg.data)
            self._mark(topic, text[:120])
            self._queue_telemetry(source, {key: text}, time.time())
        return cb

    def _string_cb(self, topic: str):
        return self._status_string_cb("text", topic, topic)

    def _battery_cb(self, msg: BatteryState):
        self._mark("/esc/battery", "BatteryState")
        self._queue_telemetry("esc", {"battery_voltage": msg.voltage, "battery_current": msg.current}, time.time())

    def _temperature_cb(self, msg: Temperature):
        self._mark("/esc/temperature", "Temperature")
        self._queue_telemetry("esc", {"temperature": msg.temperature}, time.time())

    def _obstacles_cb(self, msg):
        self._mark("/obstacle_detection/obstacles", "ObstacleArray")
        obstacles = list(getattr(msg, "obstacles", []) or [])
        conf = [float(getattr(o, "confidence", 0.0) or 0.0) for o in obstacles]
        danger = sum(1 for o in obstacles if bool(getattr(o, "in_danger_zone", False)))
        self._queue_telemetry("yolo", {
            "detection_count": float(len(obstacles)),
            "dynamic_count": float(getattr(msg, "dynamic_count", 0) or 0),
            "static_count": float(getattr(msg, "static_count", 0) or 0),
            "floor_count": float(getattr(msg, "floor_count", 0) or 0),
            "pallet_count": float(getattr(msg, "pallet_count", 0) or 0),
            "danger_count": float(danger),
            "mean_confidence": (sum(conf) / len(conf)) if conf else 0.0,
            "warning_active": 1.0 if bool(getattr(msg, "warning_active", False)) else 0.0,
        }, time.time())

    def _alignment_cb(self, msg):
        self._mark("/fork_alignment/state", str(getattr(msg, "state_text", "")))
        self._queue_telemetry("alignment", {
            "state_text": str(getattr(msg, "state_text", "")),
            "data_valid": 1.0 if bool(getattr(msg, "data_valid", False)) else 0.0,
            "pallet_detected": 1.0 if bool(getattr(msg, "pallet_detected", False)) else 0.0,
            "detection_stable": 1.0 if bool(getattr(msg, "detection_stable", False)) else 0.0,
            "confidence": float(getattr(msg, "confidence", 0.0) or 0.0),
            "depth_quality": float(getattr(msg, "depth_quality", 0.0) or 0.0),
            "error_lateral_m": float(getattr(msg, "error_lateral_m", 0.0) or 0.0),
            "error_yaw_deg": float(getattr(msg, "error_yaw_deg", 0.0) or 0.0),
            "estimated_steering_deg": float(getattr(msg, "estimated_steering_deg", 0.0) or 0.0),
            "linear_velocity_cmd": float(getattr(msg, "linear_velocity_cmd", 0.0) or 0.0),
            "angular_velocity_cmd": float(getattr(msg, "angular_velocity_cmd", 0.0) or 0.0),
            "safety_stop_active": 1.0 if bool(getattr(msg, "safety_stop_active", False)) else 0.0,
            "ready_for_insertion": 1.0 if bool(getattr(msg, "ready_for_insertion", False)) else 0.0,
        }, time.time())

    def _camera_info_cb(self, msg: CameraInfo):
        self._mark("/camera/color/camera_info", msg.header.frame_id)
        # sensor_msgs fixed arrays may be backed by numpy.ndarray. Never use
        # them directly in a boolean expression (``if msg.k``), because numpy
        # raises "truth value of an array is ambiguous" and used to interrupt
        # the entire GUI ROS executor repeatedly.
        k = list(msg.k)
        if len(k) < 9:
            k.extend([0.0] * (9 - len(k)))
        self._queue_telemetry("camera_info", {
            "camera_width": float(msg.width),
            "camera_height": float(msg.height),
            "fx": float(k[0]) if len(k) > 0 else 0.0,
            "fy": float(k[4]) if len(k) > 4 else 0.0,
            "cx": float(k[2]) if len(k) > 2 else 0.0,
            "cy": float(k[5]) if len(k) > 5 else 0.0,
            "distortion_coeff_count": float(len(msg.d)),
        }, time.time())

    def _winch_bool_cb(self, key: str, topic: str):
        def cb(msg: Bool):
            self._mark(topic, "1" if msg.data else "0")
            self._queue_telemetry("winch", {key: bool(msg.data)}, time.time())
        return cb

    def _winch_float_cb(self, key: str, topic: str):
        def cb(msg: Float64):
            self._mark(topic, f"{float(msg.data):.3f}")
            self._queue_telemetry("winch", {key: float(msg.data)}, time.time())
        return cb

    def _winch_int_cb(self, key: str, topic: str):
        def cb(msg: Int32):
            self._mark(topic, str(int(msg.data)))
            self._queue_telemetry("winch", {key: int(msg.data)}, time.time())
        return cb

    def _winch_string_cb(self, key: str, topic: str):
        def cb(msg: String):
            self._mark(topic, str(msg.data))
            self._queue_telemetry("winch", {key: str(msg.data)}, time.time())
        return cb

    def _image_cb(self, topic: str):
        def cb(msg: Image):
            self._mark(topic, f"{msg.width}x{msg.height} {msg.encoding}")
            times = self._times[topic]
            fps = 0.0
            if len(times) >= 2 and times[-1] > times[0]:
                fps = (len(times) - 1) / (times[-1] - times[0])
            if topic == "/camera/color/image_raw":
                self._queue_telemetry("camera", {
                    "width": float(msg.width), "height": float(msg.height),
                    "fps": fps, "encoding": msg.encoding, "frame_id": msg.header.frame_id,
                    "image_received": 1.0,
                }, time.time())
            elif topic == "/obstacle_detection/visualization":
                self._queue_telemetry("yolo_view", {"visualization_fps": fps}, time.time())
            elif topic == "/fork_alignment/image":
                self._queue_telemetry("alignment_view", {"alignment_image_fps": fps}, time.time())

            # Do not convert/copy multi-megabyte image buffers in the rclpy
            # callback. Hand only the newest message to the dedicated preview
            # worker; this callback returns quickly so other ROS callbacks keep
            # running even while Qt displays 1280x720 images.
            self.bridge.queue_image(topic, msg)
        return cb

    def _goal_observed_cb(self, msg: PoseStamped):
        self._mark("/goal_pose", msg.header.frame_id or "map")
        self._last_goal_monotonic = time.monotonic()
        self._last_goal_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self._queue_telemetry("navigation", {
            "goal_x": float(msg.pose.position.x),
            "goal_y": float(msg.pose.position.y),
            "goal_yaw": yaw_from_quaternion(msg.pose.orientation),
        }, time.time())

    def _nav_status_cb(self, msg: GoalStatusArray):
        self._mark("/navigate_to_pose/_action/status", "GoalStatusArray")
        statuses = list(msg.status_list)
        code = int(statuses[-1].status) if statuses else 0
        self._queue_telemetry("navigation", {
            "nav_status": float(code),
            "active_goal_count": float(len(statuses)),
        }, time.time())

    def _query_active_map(self):
        if self._map_param_future is not None and not self._map_param_future.done():
            return
        if not self.map_param_client.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ["yaml_filename"]
        future = self.map_param_client.call_async(req)
        self._map_param_future = future
        future.add_done_callback(self._active_map_response)

    def _active_map_response(self, future):
        try:
            response = future.result()
            if response and response.values:
                value = response.values[0].string_value.strip()
                if value:
                    self.bridge.active_map.emit(value)
        except Exception:
            pass

    # ---------------- TF ----------------
    def _poll_tf(self):
        result: Dict[str, Dict[str, float | str | bool]] = {}
        for parent, child in (("map", "odom"), ("odom", "base_footprint"),
                              ("base_footprint", "lidar_link"), ("base_footprint", "imu_link"),
                              ("base_footprint", "camera_color_optical_frame")):
            key = f"{parent}->{child}"
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                yaw = yaw_from_quaternion(tf.transform.rotation)
                result[key] = {
                    "available": True,
                    "x": tf.transform.translation.x,
                    "y": tf.transform.translation.y,
                    "z": tf.transform.translation.z,
                    "yaw": yaw,
                }
                if parent == "map" and child == "odom":
                    self._map_odom = (tf.transform.translation.x, tf.transform.translation.y, yaw)
                if parent == "map" and child == "base_footprint":
                    pass
            except TransformException:
                result[key] = {"available": False}
            except Exception:
                result[key] = {"available": False}
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = yaw_from_quaternion(tf.transform.rotation)
            result["map->base_footprint"] = {"available": True, "x": x, "y": y, "z": tf.transform.translation.z, "yaw": yaw}
            self._queue_pose("tf", float(x), float(y), float(yaw))
            nav_values = {"robot_x": float(x), "robot_y": float(y), "robot_yaw": float(yaw)}
            if self._last_goal_xy is not None:
                gx, gy = self._last_goal_xy
                nav_values["distance_to_goal"] = math.hypot(gx - x, gy - y)
            self._queue_telemetry("navigation", nav_values, time.time())
        except Exception:
            result["map->base_footprint"] = {"available": False}
        self.bridge.tf_status.emit(result)

    @staticmethod
    def _transform_pose(tf_xy_yaw: Tuple[float, float, float], x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        tx, ty, tyaw = tf_xy_yaw
        c, s = math.cos(tyaw), math.sin(tyaw)
        return tx + c * x - s * y, ty + s * x + c * y, tyaw + yaw

    # ---------------- commands ----------------
    def publish_winch_command(self, command: str):
        value = str(command).strip().upper()
        if not value:
            return
        msg = String()
        msg.data = value
        self.winch_command_pub.publish(msg)

    def publish_goal(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation = quaternion_from_yaw(float(yaw))
        self.goal_pub.publish(msg)
        self.bridge.log.emit(f"Goal published: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg")

    def publish_initial_pose(self, x: float, y: float, yaw: float, covariance_xy: float, covariance_yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = quaternion_from_yaw(float(yaw))
        msg.pose.covariance[0] = float(covariance_xy)
        msg.pose.covariance[7] = float(covariance_xy)
        msg.pose.covariance[35] = float(covariance_yaw)
        self.initial_pub.publish(msg)
        self.bridge.log.emit(f"Initial pose published to /initialpose_safe: x={x:.3f}, y={y:.3f}")

    def cancel_navigation(self):
        if not self.cancel_client.wait_for_service(timeout_sec=0.15):
            self.bridge.log.emit("Cancel failed: NavigateToPose cancel service unavailable")
            return
        req = CancelGoal.Request()  # zero UUID + zero stamp => cancel all goals
        self.cancel_client.call_async(req)
        self.bridge.log.emit("Cancel-all request sent to NavigateToPose")
