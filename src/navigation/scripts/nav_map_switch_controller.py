#!/usr/bin/python3
"""Fail-safe runtime Map 1/2/3 selector for the existing autonomous Nav2 stack.

The controller never owns hardware drivers. LiDAR, IMU, EKF, perception and the
BAB 4.2 mapping controller remain untouched. It only coordinates saved-map
loading, AMCL re-localization and the existing Nav2 lifecycle managers.
"""

import hashlib
import json
import math
import os
import re
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.msg import Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ClearEntireCostmap, LoadMap, ManageLifecycleNodes
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener

WORKSPACE = Path(os.environ.get('AGV_WS') or os.environ.get('AGV_ROOT') or '/home/otomasi2/forclift')
MAP_DIR = WORKSPACE / 'src/navigation/maps'
BUILD_MAP_DIR = MAP_DIR / 'build_map'
SELECTED_NAV_YAML = MAP_DIR / 'navigation_selected.yaml'
SELECTED_NAV_PGM = MAP_DIR / 'navigation_selected.pgm'
SELECTED_NAV_NAME = MAP_DIR / 'navigation_selected_name.txt'
PREP_SCRIPT = WORKSPACE / 'install/navigation/lib/navigation/prepare_nav_map.py'
SWITCH_NAVMAP_ROOT = Path('/tmp/navigation_nav_switch')

NAV_MANAGERS = {
    'global': ('lifecycle_manager_navigation_global', ('planner_server',)),
    'local': ('lifecycle_manager_navigation_local', ('controller_server',)),
    'aux': ('lifecycle_manager_navigation_aux', (
        'behavior_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor')),
}


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavMapSwitchController(Node):
    def __init__(self):
        super().__init__('nav_map_switch_controller')
        self._lock = threading.RLock()
        self._busy = False
        self._phase = 'INITIALIZING'
        self._error = ''
        self._message = ''
        self._active_slot = 0
        self._requested_slot = 0
        self._active_map_name = ''
        self._requested_map_name = ''
        self._nav_enabled = False
        self._loaded_yaml = ''
        self._planning_yaml = ''
        self._mapping_active = False
        self._last_amcl_mono = 0.0
        self._last_amcl = None
        self._localization_epoch = 0.0
        self._samples = deque(maxlen=20)
        self._live_map_slot = 0
        self._raw_grid_signature = None
        self._nav_grid_signature = None
        self._raw_grid_mono = 0.0
        self._nav_grid_mono = 0.0
        self._map_signature_cache = {}
        # Runtime health timestamps are additive only: they prevent a transient
        # zero-wait TF cache miss from being misreported as AMCL recovery.
        self._last_scan_mono = 0.0
        self._last_odom_mono = 0.0
        self._localization_bad_since = 0.0
        self._localization_latched = False

        self.declare_parameter('localization_timeout_sec', 120.0)
        # Bounded switch phases: known map-bound poses should converge quickly;
        # global localization gets a longer window but never leaves SWITCHING forever.
        self.declare_parameter('known_pose_localization_timeout_sec', 30.0)
        self.declare_parameter('global_localization_timeout_sec', 180.0)
        self.declare_parameter('post_activation_tf_timeout_sec', 20.0)
        self.declare_parameter('manager_rpc_attempt_timeout_sec', 4.0)
        self.declare_parameter('manager_rpc_retries', 2)
        self.declare_parameter('max_cov_x', 0.50)
        self.declare_parameter('max_cov_y', 0.50)
        self.declare_parameter('max_cov_yaw', 0.40)
        self.declare_parameter('stability_translation_m', 0.15)
        self.declare_parameter('stability_yaw_rad', 0.20)
        self.declare_parameter('min_convergence_samples', 2)
        self.declare_parameter('initial_pose_backdate_sec', 0.15)
        self.declare_parameter('nav_map_filter_max_cells', 3)
        self.declare_parameter('nav_map_filter_unknown_halo_cells', 2)
        # Keep AMCL's map->odom TF fresh while the robot is stationary.  AMCL
        # normally updates only after motion exceeds update_min_d/update_min_a;
        # under a rolling odom TF cache that can leave no temporal overlap for
        # Nav2 costmaps.  A low-rate no-motion request refreshes localization
        # without forcing the laser matcher to run at the full scan rate.
        self.declare_parameter('amcl_nomotion_keepalive_sec', 0.75)
        self.declare_parameter('amcl_pose_stale_sec', 1.5)
        # Status/recovery debounce. A single zero-wait tf2 miss is normal under
        # Jetson load and must not immediately become LOCALIZATION_RECOVERY.
        self.declare_parameter('localization_failure_grace_sec', 2.0)
        # If scan/odometry themselves are stale, report SENSOR_WAIT instead of
        # blaming AMCL and do not spam request_nomotion_update.
        self.declare_parameter('localization_sensor_stale_sec', 1.5)
        self._nomotion_keepalive_last = 0.0
        self._nomotion_keepalive_future = None

        self._status_pub = self.create_publisher(String, '/navigation/map_switch_status', 10)
        readiness_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Single ROS authority for Mission FSM/Nav2 readiness. This mirrors the
        # same map+AMCL validation used in map_switch_status, not GUI state.
        self._nav2_ready_pub = self.create_publisher(
            Bool, '/system/nav2_ready', readiness_qos)
        initialpose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        for slot in (1, 2, 3, 4, 5):
            self.create_service(
                Trigger, f'/navigation/map/start_{slot}',
                lambda req, resp, s=slot: self._start_service(s, req, resp))
        self.create_service(Trigger, '/navigation/map/stop', self._stop_service)

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, pose_qos)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(LaserScan, '/scan_nav', self._scan_health_cb, sensor_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_health_cb, sensor_qos)
        self.create_subscription(String, '/mapping/web_status', self._mapping_status_cb, 10)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map_identity_cb, map_qos)
        self.create_subscription(OccupancyGrid, '/nav_map', self._nav_map_cb, map_qos)

        self._load_raw = self.create_client(LoadMap, '/map_server/load_map')
        self._load_nav = self.create_client(LoadMap, '/nav_map_server/load_map')
        self._clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._global_localization = self.create_client(Empty, '/reinitialize_global_localization')
        # AMCL creates this as a private service under the node name.
        self._nomotion = self.create_client(Empty, '/request_nomotion_update')
        self._cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._state_clients = {}
        self._transition_clients = {}
        self._lifecycle_state_cache = {}
        self._lifecycle_state_cache_at = {}
        self._core_nav_state_snapshot = {
            'planner_server': 'unknown',
            'controller_server': 'unknown',
            'bt_navigator': 'unknown',
        }
        self._core_nav_state_checked_mono = 0.0
        self._manager_clients = {}
        for manager, _nodes in NAV_MANAGERS.values():
            self._manager_clients[manager] = self.create_client(
                ManageLifecycleNodes, f'/{manager}/manage_nodes')
        self._manager_clients['lifecycle_manager_localization'] = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes')
        lifecycle_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        lifecycle_nodes = ('map_server', 'nav_map_server', 'amcl',
                           'planner_server', 'controller_server',
                           'behavior_server', 'bt_navigator',
                           'velocity_smoother', 'collision_monitor')
        for lifecycle_name in lifecycle_nodes:
            self.create_subscription(
                TransitionEvent, f'/{lifecycle_name}/transition_event',
                lambda msg, n=lifecycle_name: self._lifecycle_event_cb(n, msg),
                lifecycle_qos)

        # Keep TF ingestion off this controller's busy executor. On the Jetson,
        # status/service callbacks can briefly starve /tf and cause false
        # LOCALIZATION_RECOVERY even while AMCL is healthy.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener_node = Node('nav_map_switch_tf_listener', use_global_arguments=False)
        self.tf_listener = TransformListener(
            self.tf_buffer, self._tf_listener_node, spin_thread=True)
        self.create_timer(0.5, self._publish_status)
        threading.Thread(target=self._initialize_snapshot, daemon=True).start()
        self.get_logger().info('Nav map switch controller started; hardware ownership unchanged')

    def _mapping_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self._mapping_active = bool(payload.get('active', False))
        except Exception:
            pass

    def _lifecycle_event_cb(self, node_name, msg):
        try:
            label = str(msg.goal_state.label).strip().lower()
            if label:
                self._lifecycle_state_cache[node_name] = label
                self._lifecycle_state_cache_at[node_name] = time.monotonic()
        except Exception:
            pass

    def _graph_has_node(self, node_name):
        try:
            return any(name == node_name for name, _ns in self.get_node_names_and_namespaces())
        except Exception:
            return False

    def _amcl_cb(self, msg):
        c = msg.pose.covariance
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        sample = (
            float(p.x), float(p.y), float(yaw_from_quaternion(q)),
            float(c[0]), float(c[7]), float(c[35]), time.monotonic())
        with self._lock:
            self._last_amcl = sample
            self._last_amcl_mono = sample[-1]
            if sample[-1] >= self._localization_epoch:
                self._samples.append(sample)

    def _scan_health_cb(self, _msg):
        with self._lock:
            self._last_scan_mono = time.monotonic()

    def _odom_health_cb(self, _msg):
        with self._lock:
            self._last_odom_mono = time.monotonic()

    def _sensor_health(self):
        now = time.monotonic()
        stale = max(0.5, float(self.get_parameter('localization_sensor_stale_sec').value))
        with self._lock:
            scan_t = self._last_scan_mono
            odom_t = self._last_odom_mono
        scan_age = float('inf') if scan_t <= 0.0 else max(0.0, now - scan_t)
        odom_age = float('inf') if odom_t <= 0.0 else max(0.0, now - odom_t)
        reasons = []
        if scan_age > stale:
            reasons.append('scan_nav=' + ('never' if not math.isfinite(scan_age) else f'{scan_age:.2f}s'))
        if odom_age > stale:
            reasons.append('odometry_filtered=' + ('never' if not math.isfinite(odom_age) else f'{odom_age:.2f}s'))
        detail = 'OK' if not reasons else ', '.join(reasons)
        return not reasons, detail, scan_age, odom_age

    @staticmethod
    def _pgm_size(path):
        with open(path, 'rb') as fh:
            if fh.readline().strip() not in (b'P5', b'P2'):
                return 0, 0
            tokens = []
            while len(tokens) < 2:
                line = fh.readline()
                if not line:
                    break
                line = line.split(b'#', 1)[0]
                tokens.extend(line.split())
            return (int(tokens[0]), int(tokens[1])) if len(tokens) >= 2 else (0, 0)

    def _slot_label(self, slot):
        slot = int(slot)
        if slot == 4:
            return 'Navigation Map (Default)'
        if slot == 5:
            try:
                name = SELECTED_NAV_NAME.read_text(encoding='utf-8').strip()
            except OSError:
                name = ''
            return name or 'Build Map'
        return f'Map {slot}'

    def _slot_pgm(self, slot):
        slot = int(slot)
        if slot == 4:
            return MAP_DIR / 'map_Navigation.pgm'
        if slot == 5:
            return SELECTED_NAV_PGM
        return MAP_DIR / f'map_{slot}.pgm'

    def _sidecar_path(self, slot):
        slot = int(slot)
        if slot == 5:
            name = self._slot_label(slot)
            safe = re.sub(r'[^A-Za-z0-9_-]+', '_', name).strip('_')[:64] or 'build_map'
            BUILD_MAP_DIR.mkdir(parents=True, exist_ok=True)
            return BUILD_MAP_DIR / f'{safe}.autopose.json'
        return MAP_DIR / f'map_{slot}.autopose.json'

    @staticmethod
    def _read_pgm(path):
        data = Path(path).read_bytes()
        index = 0
        size = len(data)

        def token():
            nonlocal index
            while index < size:
                if data[index] == 35:  # # comment
                    while index < size and data[index] not in (10, 13):
                        index += 1
                elif chr(data[index]).isspace():
                    index += 1
                else:
                    break
            start = index
            while index < size and not chr(data[index]).isspace() and data[index] != 35:
                index += 1
            return data[start:index]

        magic = token()
        width = int(token())
        height = int(token())
        max_value = int(token())
        if width <= 0 or height <= 0 or max_value <= 0 or max_value > 255:
            raise RuntimeError(f'PGM header tidak valid: {path}')
        if magic == b'P5':
            if index < size and data[index] == 13:
                index += 1
                if index < size and data[index] == 10:
                    index += 1
            elif index < size and chr(data[index]).isspace():
                index += 1
            pixels = data[index:index + width * height]
            if len(pixels) != width * height:
                raise RuntimeError(f'PGM payload tidak lengkap: {path}')
            return width, height, max_value, pixels
        if magic == b'P2':
            values = []
            for _ in range(width * height):
                values.append(int(token()))
            return width, height, max_value, bytes(values)
        raise RuntimeError(f'PGM format tidak didukung: {magic!r}')

    def _grid_signature_from_yaml(self, yaml_path):
        yaml_path = Path(yaml_path).expanduser().resolve()
        text = yaml_path.read_text(encoding='utf-8', errors='replace')
        image_match = re.search(r'^\s*image\s*:\s*(.+?)\s*$', text, re.M)
        resolution_match = re.search(r'^\s*resolution\s*:\s*([-+0-9.eE]+)', text, re.M)
        origin_match = re.search(
            r'^\s*origin\s*:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)',
            text, re.M)
        occupied_match = re.search(r'^\s*occupied_thresh\s*:\s*([-+0-9.eE]+)', text, re.M)
        free_match = re.search(r'^\s*free_thresh\s*:\s*([-+0-9.eE]+)', text, re.M)
        negate_match = re.search(r'^\s*negate\s*:\s*(\d+)', text, re.M)
        mode_match = re.search(r'^\s*mode\s*:\s*(\w+)', text, re.M)
        if not all((image_match, resolution_match, origin_match, occupied_match, free_match, negate_match)):
            raise RuntimeError(f'YAML map tidak lengkap untuk verifikasi grid: {yaml_path}')
        mode = mode_match.group(1).strip().lower() if mode_match else 'trinary'
        if mode != 'trinary':
            raise RuntimeError(f'Verifikasi switch hanya mendukung mode trinary, ditemukan {mode}: {yaml_path}')
        image_value = image_match.group(1).strip().strip('\"\'')
        image_path = Path(image_value)
        if not image_path.is_absolute():
            image_path = (yaml_path.parent / image_path).resolve()
        cache_key = (str(yaml_path), yaml_path.stat().st_mtime_ns, yaml_path.stat().st_size,
                     str(image_path), image_path.stat().st_mtime_ns, image_path.stat().st_size)
        cached = self._map_signature_cache.get(cache_key)
        if cached is not None:
            return cached
        width, height, max_value, pixels = self._read_pgm(image_path)
        occupied_thresh = float(occupied_match.group(1))
        free_thresh = float(free_match.group(1))
        negate = int(negate_match.group(1)) != 0
        grid = bytearray(width * height)
        out = 0
        # nav2_map_server flips image rows into OccupancyGrid map coordinates.
        for row in range(height - 1, -1, -1):
            base = row * width
            for column in range(width):
                value = pixels[base + column]
                occupancy = (value / max_value) if negate else (1.0 - value / max_value)
                if occupancy > occupied_thresh:
                    cell = 100
                elif occupancy < free_thresh:
                    cell = 0
                else:
                    cell = 255  # signed OccupancyGrid -1 encoded as one byte
                grid[out] = cell
                out += 1
        signature = (
            width, height, float(resolution_match.group(1)),
            float(origin_match.group(1)), float(origin_match.group(2)), float(origin_match.group(3)),
            hashlib.sha256(grid).hexdigest())
        # Retain one signature per YAML/image pair and drop stale versions of that path.
        for key in list(self._map_signature_cache):
            if key[0] == str(yaml_path) and key != cache_key:
                self._map_signature_cache.pop(key, None)
        self._map_signature_cache[cache_key] = signature
        return signature

    @staticmethod
    def _grid_signature_from_msg(msg):
        info = msg.info
        return (
            int(info.width), int(info.height), float(info.resolution),
            float(info.origin.position.x), float(info.origin.position.y),
            float(yaw_from_quaternion(info.origin.orientation)),
            hashlib.sha256(bytes(int(value) & 0xff for value in msg.data)).hexdigest())

    @staticmethod
    def _signature_matches(actual, expected):
        if actual is None or expected is None:
            return False
        return (actual[0] == expected[0] and actual[1] == expected[1] and
                abs(actual[2] - expected[2]) <= 1e-6 and
                abs(actual[3] - expected[3]) <= 1e-4 and
                abs(actual[4] - expected[4]) <= 1e-4 and
                abs(wrap_angle(actual[5] - expected[5])) <= 1e-6 and
                actual[6] == expected[6])

    def _match_slot_from_signature(self, signature):
        with self._lock:
            preferred = [int(self._requested_slot), int(self._active_slot), int(self._live_map_slot)]
        seen = set()
        for slot in preferred + [4, 1, 2, 3, 5]:
            if slot not in (1, 2, 3, 4, 5) or slot in seen:
                continue
            seen.add(slot)
            try:
                expected = self._grid_signature_from_yaml(self._slot_yaml(slot))
            except Exception:
                continue
            if self._signature_matches(signature, expected):
                return slot
        return 0

    def _map_identity_cb(self, msg):
        try:
            signature = self._grid_signature_from_msg(msg)
            matched = self._match_slot_from_signature(signature)
        except Exception as exc:
            self.get_logger().warning(f'Gagal menghitung identitas /map: {exc}')
            return
        with self._lock:
            self._raw_grid_signature = signature
            self._raw_grid_mono = time.monotonic()
            if matched:
                self._live_map_slot = matched
                if (not self._busy) or matched == int(self._requested_slot):
                    self._active_slot = matched
                    self._active_map_name = self._slot_label(matched)
                    self._loaded_yaml = str(self._slot_yaml(matched).resolve())

    def _nav_map_cb(self, msg):
        try:
            signature = self._grid_signature_from_msg(msg)
        except Exception as exc:
            self.get_logger().warning(f'Gagal menghitung identitas /nav_map: {exc}')
            return
        with self._lock:
            self._nav_grid_signature = signature
            self._nav_grid_mono = time.monotonic()

    def _wait_for_loaded_grid(self, yaml_path, kind, newer_than, timeout=12.0):
        expected = self._grid_signature_from_yaml(yaml_path)
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                if kind == 'raw':
                    actual, observed = self._raw_grid_signature, self._raw_grid_mono
                else:
                    actual, observed = self._nav_grid_signature, self._nav_grid_mono
            if observed >= newer_than and self._signature_matches(actual, expected):
                return True
            time.sleep(0.05)
        topic = '/map' if kind == 'raw' else '/nav_map'
        raise RuntimeError(f'{topic} belum cocok dengan map target {Path(yaml_path).name} setelah LoadMap')

    def _slot_yaml(self, slot):
        slot = int(slot)
        if slot == 4:
            return MAP_DIR / 'map_Navigation.yaml'
        if slot == 5:
            return SELECTED_NAV_YAML
        return MAP_DIR / f'map_{slot}.yaml'

    def _validate_slot(self, slot):
        if slot not in (1, 2, 3, 4, 5):
            raise RuntimeError('target Nav2 tidak valid')
        yaml_path = self._slot_yaml(slot)
        pgm_path = self._slot_pgm(slot)
        pgm_name = pgm_path.name
        label = self._slot_label(slot)
        if not yaml_path.is_file() or yaml_path.stat().st_size <= 0:
            raise RuntimeError(f'{label} YAML belum tersedia: {yaml_path}')
        if not pgm_path.is_file() or pgm_path.stat().st_size <= 0:
            raise RuntimeError(f'{label} PGM belum tersedia: {pgm_path}')
        text = yaml_path.read_text(encoding='utf-8', errors='replace')
        if pgm_name not in text:
            raise RuntimeError(f'{label} YAML tidak menunjuk ke {pgm_name}')
        return yaml_path.resolve()

    def _future_result(self, future, timeout=8.0):
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)
        try:
            future.cancel()
        except Exception:
            pass
        raise TimeoutError('ROS service response timeout')

    def _wait_client(self, client, timeout=4.0):
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.15):
                return True
        return False

    def _node_state(self, node_name, timeout=2.5, force_refresh=False):
        # Transition events remain the fast path, but lifecycle state is not
        # allowed to stay cached indefinitely. A respawned Nav2 node can return
        # to UNCONFIGURED without this controller seeing the earlier event.
        if not self._graph_has_node(node_name):
            self._lifecycle_state_cache.pop(node_name, None)
            self._lifecycle_state_cache_at.pop(node_name, None)
            return 'missing'
        now_mono = time.monotonic()
        cached = self._lifecycle_state_cache.get(node_name)
        cached_at = float(self._lifecycle_state_cache_at.get(node_name, 0.0))
        if cached and not force_refresh and (now_mono - cached_at) < 0.75:
            return cached
        client = self._state_clients.get(node_name)
        if client is None:
            client = self.create_client(GetState, f'/{node_name}/get_state')
            self._state_clients[node_name] = client
        deadline = time.monotonic() + max(0.20, float(timeout))
        saw_service = False
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            if not self._wait_client(client, timeout=min(0.25, remaining)):
                continue
            saw_service = True
            try:
                response = self._future_result(
                    client.call_async(GetState.Request()), timeout=min(0.75, remaining))
                if response:
                    label = str(response.current_state.label).strip().lower()
                    if label:
                        self._lifecycle_state_cache[node_name] = label
                        self._lifecycle_state_cache_at[node_name] = time.monotonic()
                        return label
            except Exception:
                time.sleep(0.03)
        # Forced health checks are fail-closed: stale ACTIVE must never make the
        # GUI or mission readiness report READY when the RPC cannot confirm it.
        if force_refresh:
            return 'unknown' if saw_service else 'missing'
        return cached or ('unknown' if saw_service else 'missing')

    def _change_state(self, node_name, transition_id, timeout=6.0):
        client = self._transition_clients.get(node_name)
        if client is None:
            client = self.create_client(ChangeState, f'/{node_name}/change_state')
            self._transition_clients[node_name] = client
        if not self._wait_client(client, timeout=min(3.0, timeout)):
            raise RuntimeError(f'/{node_name}/change_state tidak tersedia')
        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        result = self._future_result(client.call_async(request), timeout=timeout)
        if result is None or not bool(result.success):
            raise RuntimeError(f'{node_name} transition={int(transition_id)} gagal')
        return True

    def _ensure_node_active(self, node_name, timeout=30.0):
        deadline = time.monotonic() + float(timeout)
        last_state = 'unknown'
        while rclpy.ok() and time.monotonic() < deadline:
            last_state = self._node_state(node_name, timeout=2.0, force_refresh=True)
            if last_state == 'active':
                return True
            try:
                if last_state == 'unconfigured':
                    self._change_state(node_name, Transition.TRANSITION_CONFIGURE, timeout=5.0)
                elif last_state == 'inactive':
                    self._change_state(node_name, Transition.TRANSITION_ACTIVATE, timeout=5.0)
                elif last_state in ('missing', 'unknown'):
                    time.sleep(0.25)
                else:
                    time.sleep(0.20)
            except Exception:
                time.sleep(0.20)
        raise RuntimeError(f'{node_name} tidak mencapai ACTIVE (state={last_state})')

    def _ensure_node_inactive(self, node_name, timeout=30.0):
        deadline = time.monotonic() + float(timeout)
        last_state = 'unknown'
        while rclpy.ok() and time.monotonic() < deadline:
            last_state = self._node_state(node_name, timeout=1.8, force_refresh=True)
            if last_state in ('inactive', 'unconfigured', 'finalized', 'missing'):
                return True
            try:
                if last_state == 'active':
                    self._change_state(node_name, Transition.TRANSITION_DEACTIVATE, timeout=4.0)
                else:
                    time.sleep(0.20)
            except Exception:
                time.sleep(0.20)
        raise RuntimeError(f'{node_name} tidak mencapai INACTIVE (state={last_state})')

    def _ensure_node_unconfigured(self, node_name, timeout=30.0):
        deadline = time.monotonic() + float(timeout)
        last_state = 'unknown'
        while rclpy.ok() and time.monotonic() < deadline:
            last_state = self._node_state(node_name, timeout=2.0, force_refresh=True)
            if last_state == 'unconfigured':
                return True
            try:
                if last_state == 'active':
                    self._change_state(node_name, Transition.TRANSITION_DEACTIVATE, timeout=5.0)
                elif last_state == 'inactive':
                    self._change_state(node_name, Transition.TRANSITION_CLEANUP, timeout=5.0)
                elif last_state in ('missing', 'finalized'):
                    raise RuntimeError(f'{node_name} tidak dapat di-reset dari state={last_state}')
                else:
                    time.sleep(0.20)
            except Exception:
                time.sleep(0.20)
        raise RuntimeError(f'{node_name} tidak mencapai UNCONFIGURED (state={last_state})')

    def _manager_command(self, manager, command, timeout=10.0):
        client = self._manager_clients.get(manager)
        if client is None:
            client = self.create_client(ManageLifecycleNodes, f'/{manager}/manage_nodes')
            self._manager_clients[manager] = client
        retries = max(1, int(self.get_parameter('manager_rpc_retries').value))
        per_try = max(1.0, min(float(timeout), float(self.get_parameter('manager_rpc_attempt_timeout_sec').value)))
        last = 'no response'
        for attempt in range(1, retries + 1):
            if not self._wait_client(client, timeout=min(2.0, per_try)):
                last = 'service unavailable'
            else:
                request = ManageLifecycleNodes.Request()
                request.command = int(command)
                result = self._future_result(client.call_async(request), timeout=per_try)
                if result is not None and bool(result.success):
                    return True
                last = 'timeout/unsuccessful response'
            if attempt < retries:
                time.sleep(0.15)
        raise RuntimeError(f'{manager} command={command} gagal setelah {retries} percobaan ({last})')

    def _manager_needed(self, node_names):
        return any(self._node_state(name, timeout=1.5) == 'active' for name in node_names)

    def _pause_navigation(self, goals_cancelled=False):
        # Map switching only needs the goal-producing global/BT branches stopped.
        # controller_server is deliberately kept ACTIVE so the local costmap stays
        # live while AMCL is reset. Motion remains fail-closed because goals are
        # cancelled and bt_navigator is verified non-ACTIVE.
        warnings = []
        for key in ('aux', 'global'):
            manager, nodes = NAV_MANAGERS[key]
            if not self._manager_needed(nodes):
                continue
            try:
                self._manager_command(manager, ManageLifecycleNodes.Request.PAUSE, timeout=10.0)
            except Exception as exc:
                warnings.append(f'{manager}: {exc}; direct lifecycle fallback')
        # Manager services can transiently fail under Jetson load. Verify the two
        # nodes that can create/execute a navigation goal and force them inactive
        # through their canonical lifecycle services when necessary.
        for node_name in ('bt_navigator', 'planner_server'):
            try:
                self._ensure_node_inactive(node_name, timeout=30.0)
            except Exception as exc:
                raise RuntimeError(f'Nav2 gagal pause {node_name}: {exc}')
        controller_state = self._node_state('controller_server', 2.5)
        if controller_state == 'active':
            if goals_cancelled:
                warnings.append('controller_server tetap ACTIVE untuk local-costmap; semua goal sudah dicancel')
            else:
                # Never assume an unavailable cancel service means "no goal".
                # Safe fallback: pause/deactivate controller_server itself for
                # the short map-switch window, then _activate_navigation restores it.
                manager, _nodes = NAV_MANAGERS['local']
                try:
                    self._manager_command(manager, ManageLifecycleNodes.Request.PAUSE, timeout=10.0)
                except Exception as exc:
                    warnings.append(f'{manager}: {exc}; direct controller pause fallback')
                self._ensure_node_inactive('controller_server', timeout=30.0)
                warnings.append('cancel-all tidak terverifikasi; controller_server dipause fail-safe')
        return warnings

    def _activate_navigation(self):
        # Prefer lifecycle managers, but never send a new batch command while a
        # previous configure/activate/deactivate transition is still in flight.
        warnings = []
        transitional = {'configuring','activating','deactivating','cleaningup','shuttingdown','errorprocessing'}
        for key in ('global', 'local', 'aux'):
            manager, nodes = NAV_MANAGERS[key]
            states = {name: self._node_state(name, 2.0) for name in nodes}
            for name, state in list(states.items()):
                if state not in transitional:
                    continue
                settle = 90.0 if name == 'planner_server' else 35.0
                deadline = time.monotonic() + settle
                while rclpy.ok() and time.monotonic() < deadline and state in transitional:
                    time.sleep(0.30)
                    state = self._node_state(name, 2.0)
                states[name] = state
            if states and all(value == 'active' for value in states.values()):
                continue
            command = (ManageLifecycleNodes.Request.STARTUP
                       if any(value in ('unconfigured', 'finalized') for value in states.values())
                       else ManageLifecycleNodes.Request.RESUME)
            try:
                self._manager_command(manager, command, timeout=12.0)
            except Exception as exc:
                warnings.append(f'{manager}: {exc}; direct lifecycle fallback')

        activation_order = (
            'planner_server', 'controller_server', 'behavior_server',
            'velocity_smoother', 'collision_monitor', 'bt_navigator')
        activation_timeouts = {
            'planner_server': 90.0,
            'controller_server': 45.0,
            'behavior_server': 45.0,
            'velocity_smoother': 30.0,
            'collision_monitor': 30.0,
            'bt_navigator': 45.0,
        }
        errors = []
        for node_name in activation_order:
            try:
                with self._lock:
                    self._phase = 'STARTING_NAV2'
                    self._message = f'Mengaktifkan {node_name}; menunggu lifecycle stabil'
                self._ensure_node_active(node_name, timeout=activation_timeouts[node_name])
            except Exception as exc:
                errors.append(str(exc))
        states = {name: self._node_state(name, 3.0) for name in activation_order}
        not_active = [name for name, state in states.items() if state != 'active']
        if not_active:
            errors.append('belum ACTIVE: ' + ', '.join(f'{n}={states[n]}' for n in not_active))
        if errors:
            raise RuntimeError(' | '.join(errors))
        if warnings:
            self.get_logger().warning('Lifecycle manager fallback used: ' + ' | '.join(warnings))
        return True

    def _cancel_goals(self):
        if not self._wait_client(self._cancel, timeout=1.5):
            return False
        try:
            request = CancelGoal.Request()  # zero UUID + zero stamp = cancel all goals
            self._future_result(self._cancel.call_async(request), timeout=3.0)
            return True
        except Exception:
            return False

    def _prepare_planning_map(self, slot, raw_yaml):
        if not PREP_SCRIPT.is_file():
            raise RuntimeError(f'prepare_nav_map.py tidak ditemukan: {PREP_SCRIPT}')
        out_dir = SWITCH_NAVMAP_ROOT / f'map_{slot}'
        out_dir.mkdir(parents=True, exist_ok=True)
        max_cells = int(self.get_parameter('nav_map_filter_max_cells').value)
        halo = int(self.get_parameter('nav_map_filter_unknown_halo_cells').value)
        result = subprocess.run([
            '/usr/bin/python3', str(PREP_SCRIPT),
            '--map-yaml', str(raw_yaml),
            '--output-dir', str(out_dir),
            '--max-cells', str(max_cells),
            '--unknown-halo-cells', str(halo),
        ], cwd=str(WORKSPACE), text=True, stdout=subprocess.PIPE,
           stderr=subprocess.STDOUT, timeout=15.0, check=False)
        if result.returncode != 0:
            raise RuntimeError('prepare_nav_map gagal: ' + result.stdout.strip()[-500:])
        match = re.search(r'^NAV_MAP=(.+)$', result.stdout, flags=re.MULTILINE)
        if not match:
            raise RuntimeError('prepare_nav_map tidak mengembalikan NAV_MAP')
        path = Path(match.group(1).strip()).expanduser().resolve()
        if not path.is_file():
            raise RuntimeError(f'planning map tidak ditemukan: {path}')
        return path

    def _ensure_map_servers_active(self, timeout=30.0):
        """Recover both map servers before LoadMap or AMCL startup."""
        for node_name in ('map_server', 'nav_map_server'):
            self._ensure_node_active(node_name, timeout=timeout)
        return True

    def _load_map(self, client, service_name, yaml_path):
        if not self._wait_client(client, timeout=4.0):
            raise RuntimeError(f'{service_name} tidak tersedia')
        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        response = self._future_result(client.call_async(request), timeout=10.0)
        if response is None or int(response.result) != int(LoadMap.Response.RESULT_SUCCESS):
            code = 'NO_RESPONSE' if response is None else int(response.result)
            raise RuntimeError(f'{service_name} gagal load {yaml_path} result={code}')
        return True

    def _clear_costmap(self, client, service_name, timeout=6.0):
        if not self._wait_client(client, timeout=min(3.0, timeout)):
            raise RuntimeError(f'{service_name} tidak tersedia')
        response = self._future_result(
            client.call_async(ClearEntireCostmap.Request()), timeout=timeout)
        if response is None:
            raise RuntimeError(f'{service_name} tidak merespons')
        return True

    def _refresh_costmaps_after_map_switch(self):
        # The global static layer is map-dependent and MUST be rebuilt from the
        # newly selected /nav_map. Local costmap is rolling, but clearing it here
        # prevents transient obstacle remnants from the previous localization.
        self._clear_costmap(
            self._clear_global_costmap,
            '/global_costmap/clear_entirely_global_costmap', timeout=8.0)
        warnings = []
        try:
            self._clear_costmap(
                self._clear_local_costmap,
                '/local_costmap/clear_entirely_local_costmap', timeout=5.0)
        except Exception as exc:
            warnings.append(str(exc))
        time.sleep(0.35)
        return warnings

    def _reset_and_start_amcl(self):
        manager = 'lifecycle_manager_localization'
        warnings = []
        state = self._node_state('amcl', timeout=3.0)
        if state != 'unconfigured':
            try:
                self._manager_command(manager, ManageLifecycleNodes.Request.RESET, timeout=10.0)
            except Exception as exc:
                warnings.append(f'{manager} RESET: {exc}; direct AMCL reset fallback')
            self._ensure_node_unconfigured('amcl', timeout=30.0)
        try:
            self._manager_command(manager, ManageLifecycleNodes.Request.STARTUP, timeout=12.0)
        except Exception as exc:
            warnings.append(f'{manager} STARTUP: {exc}; direct AMCL startup fallback')
        self._ensure_node_active('amcl', timeout=20.0)
        if warnings:
            self.get_logger().warning('AMCL lifecycle fallback used: ' + ' | '.join(warnings))
        return True

    def _map_sha256(self, yaml_path):
        yaml_path = Path(yaml_path).resolve()
        text = yaml_path.read_text(encoding='utf-8', errors='replace')
        match = re.search(r'^\s*image\s*:\s*(.+?)\s*$', text, flags=re.MULTILINE)
        if not match:
            raise RuntimeError(f'YAML map tidak memiliki image: {yaml_path}')
        image_value = match.group(1).strip().strip('\"\'')
        image_path = Path(image_value)
        if not image_path.is_absolute():
            image_path = (yaml_path.parent / image_path).resolve()
        if not image_path.is_file():
            raise RuntimeError(f'image map tidak tersedia: {image_path}')
        digest = hashlib.sha256()
        for part in (yaml_path, image_path):
            with part.open('rb') as handle:
                for chunk in iter(lambda: handle.read(1024 * 1024), b''):
                    digest.update(chunk)
        return digest.hexdigest(), image_path

    def _map_bound_pose(self, slot, yaml_path):
        sidecar = self._sidecar_path(slot)
        if not sidecar.is_file():
            return None, f'no sidecar {sidecar.name}'
        try:
            payload = json.loads(sidecar.read_text(encoding='utf-8'))
            current_hash, _image_path = self._map_sha256(yaml_path)
            side_hash = str(payload.get('map_sha256', '')).strip().lower()
            frames_ok = (str(payload.get('frame_id', 'map')) == 'map' and
                         str(payload.get('child_frame_id', 'base_footprint')) == 'base_footprint')
            values = (float(payload.get('x')), float(payload.get('y')), float(payload.get('yaw')))
            numeric_ok = all(math.isfinite(v) for v in values)
            if side_hash != current_hash.lower() or not frames_ok or not numeric_ok:
                return None, (f'sidecar rejected hash_ok={int(side_hash == current_hash.lower())} '
                              f'frames_ok={int(frames_ok)} numeric_ok={int(numeric_ok)}')
            return values, f'map-bound sidecar {sidecar.name}'
        except Exception as exc:
            return None, f'invalid sidecar {sidecar.name}: {exc}'

    def _publish_known_pose(self, pose):
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        backdate = max(0.0, float(self.get_parameter('initial_pose_backdate_sec').value))
        msg.header.stamp = (self.get_clock().now() - Duration(seconds=backdate)).to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        half = 0.5 * float(yaw)
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.pose.covariance[0] = min(float(self.get_parameter('max_cov_x').value), 0.25)
        msg.pose.covariance[7] = min(float(self.get_parameter('max_cov_y').value), 0.25)
        msg.pose.covariance[35] = min(float(self.get_parameter('max_cov_yaw').value), (math.pi / 12.0) ** 2)
        self._initialpose_pub.publish(msg)

    def _persist_converged_pose(self, slot, yaml_path):
        with self._lock:
            sample = self._last_amcl
        if sample is None or sample[-1] < self._localization_epoch:
            return False
        current_hash, image_path = self._map_sha256(yaml_path)
        payload = {
            'schema_version': 1,
            'map_yaml': str(Path(yaml_path).resolve()),
            'map_image': str(image_path.resolve()),
            'map_sha256': current_hash,
            'frame_id': 'map',
            'child_frame_id': 'base_footprint',
            'x': round(float(sample[0]), 6),
            'y': round(float(sample[1]), 6),
            'yaw': round(float(sample[2]), 6),
            'source': 'amcl_converged_nav_map_switch',
        }
        sidecar = self._sidecar_path(slot)
        tmp = sidecar.with_suffix(sidecar.suffix + '.tmp')
        tmp.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
        os.replace(tmp, sidecar)
        return True

    def _request_global_localization(self):
        if not self._wait_client(self._global_localization, timeout=5.0):
            raise RuntimeError('/reinitialize_global_localization tidak tersedia')
        self._future_result(self._global_localization.call_async(Empty.Request()), timeout=5.0)
        return True

    def _request_nomotion(self):
        # Never block the map-switch worker on a DDS service response. One pending
        # no-motion request is enough; AMCL can answer it when executor load permits.
        try:
            if not self._nomotion.service_is_ready():
                return False
            pending = self._nomotion_keepalive_future
            if pending is not None and not pending.done():
                return True
            self._nomotion_keepalive_last = time.monotonic()
            self._nomotion_keepalive_future = self._nomotion.call_async(Empty.Request())
            return True
        except Exception:
            return False

    def _amcl_tf_keepalive(self):
        now = time.monotonic()
        with self._lock:
            nav_enabled = self._nav_enabled
            mapping_active = self._mapping_active
            busy = self._busy
            last_amcl = self._last_amcl_mono
        with self._lock:
            active_slot = int(self._active_slot)
        # Keep AMCL localization warm even while Nav2 is intentionally OFF, as
        # long as a saved map is loaded. This lets a later same-map START resume
        # without resetting AMCL merely because the controller restarted.
        if mapping_active or busy or (not nav_enabled and active_slot not in (1,2,3,4,5)):
            return
        sensor_ok, _sensor_detail, _scan_age, _odom_age = self._sensor_health()
        if not sensor_ok:
            # Sensor transport failure is not an AMCL failure. Keep the existing
            # localization state and wait for scan/odom to recover.
            return
        stale_sec = max(0.5, float(self.get_parameter('amcl_pose_stale_sec').value))
        base_period = max(0.5, float(self.get_parameter('amcl_nomotion_keepalive_sec').value))
        # Keep map->odom refreshed *before* AMCL's transform_tolerance window
        # expires. Waiting until TF is already missing creates a short gap in
        # FollowPath where the SMAC path exists but MPPI cannot run. This is a
        # low-rate no-motion update only; LiDAR/IMU/EKF and MPPI tuning stay
        # untouched. A stale pose or missing TF accelerates recovery to 0.5 s.
        tf_missing = not self._tf_ready(0.0)
        if last_amcl <= 0.0:
            # Fresh controller process: ask AMCL for one stationary update so
            # this observer can establish a proven-good localization baseline.
            pose_stale = True
            period_sec = max(1.0, base_period)
        else:
            pose_stale = now - last_amcl >= stale_sec
            period_sec = 0.5 if (tf_missing or pose_stale) else base_period
        if now - self._nomotion_keepalive_last < period_sec:
            return
        pending = self._nomotion_keepalive_future
        if pending is not None and not pending.done():
            # Do not let one DDS/service stall permanently disable the keepalive.
            if now - self._nomotion_keepalive_last < 5.0:
                return
            try:
                pending.cancel()
            except Exception:
                pass
        if not self._nomotion.service_is_ready():
            return
        self._nomotion_keepalive_last = now
        self._nomotion_keepalive_future = self._nomotion.call_async(Empty.Request())

    def _tf_ready(self, timeout_sec=0.0):
        try:
            # Status publication must never spend 250 ms blocking every 500 ms.
            # Callers performing an actual START validation may request a small
            # bounded wait, while the steady-state watchdog uses a zero-wait
            # cache lookup.  This substantially lowers executor contention on
            # the Jetson without relaxing the localization gate.
            # A zero-wait cache probe is too sensitive under high CPU load.
            # Use a small bounded wait; healthy cached TF returns immediately.
            requested = max(0.0, float(timeout_sec))
            wait_sec = requested if requested > 0.0 else 0.075
            tf_wait = Duration(seconds=min(0.20, wait_sec))
            return (self.tf_buffer.can_transform('map', 'odom', Time(), timeout=tf_wait) and
                    self.tf_buffer.can_transform('map', 'base_footprint', Time(), timeout=tf_wait))
        except Exception:
            return False

    def _localized(self, tf_timeout_sec=0.0):
        with self._lock:
            samples = [s for s in self._samples if s[-1] >= self._localization_epoch]
        minimum = max(2, int(self.get_parameter('min_convergence_samples').value))
        if len(samples) < minimum:
            return False, f'samples={len(samples)}/{minimum}'
        window = samples[-minimum:]
        max_cov_x = float(self.get_parameter('max_cov_x').value)
        max_cov_y = float(self.get_parameter('max_cov_y').value)
        max_cov_yaw = float(self.get_parameter('max_cov_yaw').value)
        cov_ok = all(s[3] <= max_cov_x and s[4] <= max_cov_y and s[5] <= max_cov_yaw for s in window)
        ref = window[-1]
        spread_m = max(math.hypot(s[0] - ref[0], s[1] - ref[1]) for s in window)
        spread_yaw = max(abs(wrap_angle(s[2] - ref[2])) for s in window)
        stable = (spread_m <= float(self.get_parameter('stability_translation_m').value) and
                  spread_yaw <= float(self.get_parameter('stability_yaw_rad').value))
        tf_ok = self._tf_ready(tf_timeout_sec)
        detail = (f'cov=({ref[3]:.3f},{ref[4]:.3f},{ref[5]:.3f}) '
                  f'spread={spread_m:.3f}m/{spread_yaw:.3f}rad tf={int(tf_ok)}')
        return bool(cov_ok and stable and tf_ok), detail

    def _initialize_snapshot(self):
        time.sleep(1.0)
        try:
            self._ensure_map_servers_active(timeout=25.0)
        except Exception as exc:
            self.get_logger().warning(f'Map-server startup recovery pending: {exc}')
        # Identify the live map from OccupancyGrid content, never from yaml_filename.
        # LoadMap does not guarantee that parameter changes after a runtime switch.
        map_deadline = time.monotonic() + 12.0
        slot = 0
        while rclpy.ok() and time.monotonic() < map_deadline:
            with self._lock:
                slot = int(self._live_map_slot)
            if slot:
                break
            time.sleep(0.20)
        current = str(self._slot_yaml(slot).resolve()) if slot else ''
        required = ('planner_server', 'controller_server', 'bt_navigator')
        deadline = time.monotonic() + 30.0
        states = {name: 'unknown' for name in required}
        while rclpy.ok() and time.monotonic() < deadline:
            states = {name: self._node_state(name, timeout=1.5) for name in required}
            if all(value not in ('missing', 'unknown') for value in states.values()):
                break
            time.sleep(0.5)
        nav_enabled = all(value == 'active' for value in states.values())
        # /map may arrive while lifecycle discovery is still retrying. Content identity wins.
        with self._lock:
            final_live_slot = int(self._live_map_slot)
        if final_live_slot:
            slot = final_live_slot
            current = str(self._slot_yaml(final_live_slot).resolve())
        with self._lock:
            self._loaded_yaml = current
            self._active_slot = slot
            self._active_map_name = self._slot_label(slot) if slot else ''
            self._nav_enabled = nav_enabled
            self._phase = 'NAV2_READY' if nav_enabled else 'NAV2_OFF'
            self._message = (f'active map={current or "unknown"}; identity=occupancy_sha256; ' +
                             ', '.join(f'{k}={v}' for k, v in states.items()))
        self._publish_status()

    def _start_service(self, slot, _request, response):
        try:
            yaml_path = self._validate_slot(slot)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        with self._lock:
            if self._mapping_active:
                response.success = False
                response.message = 'Mapping BAB 4.2 sedang aktif; STOP mapping dahulu sebelum START Nav2'
                return response
            if self._busy:
                response.success = False
                response.message = f'Nav map switch sedang sibuk: {self._phase}'
                return response
            label = self._slot_label(slot)
            self._busy = True
            self._requested_slot = slot
            self._requested_map_name = label
            self._phase = 'REQUESTED'
            self._error = ''
            self._message = f'USE {label} diterima'
        threading.Thread(target=self._start_worker, args=(slot, yaml_path), daemon=True).start()
        response.success = True
        response.message = f'USE {label} diterima; controller melakukan switch fail-safe'
        self._publish_status()
        return response

    def _stop_service(self, _request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = f'Nav map switch sedang sibuk: {self._phase}'
                return response
            self._busy = True
            self._phase = 'STOPPING_NAV2'
            self._error = ''
            self._message = 'STOP NAV2 diterima'
        threading.Thread(target=self._stop_worker, daemon=True).start()
        response.success = True
        response.message = 'STOP NAV2 diterima; sensor/localization tetap dipertahankan'
        self._publish_status()
        return response

    def _start_worker(self, slot, yaml_path):
        label = self._slot_label(slot)
        try:
            # map_server's yaml_filename parameter reflects its launch-time map
            # and is not guaranteed to change after LoadMap. After startup, use
            # the controller's last successfully loaded slot/path as truth so a
            # Map 2 -> Map 3 switch can never be mistaken for "same map".
            with self._lock:
                live_slot = int(self._live_map_slot)
                current_slot = int(self._active_slot)
                current_loaded = str(self._loaded_yaml or '')
                current_name = str(self._active_map_name or '')
            try:
                loaded_matches = bool(current_loaded) and Path(current_loaded).resolve() == Path(yaml_path).resolve()
            except Exception:
                loaded_matches = False
            same_map = ((live_slot == slot) if live_slot else (current_slot == slot and loaded_matches))
            if int(slot) == 5:
                same_map = bool(same_map and current_name == label)
            # If STOP was used and localization is still valid on the same map,
            # do not churn map_server/AMCL. Resume Nav2 only.
            localized, detail = self._localized(0.10)
            if same_map and localized:
                with self._lock:
                    self._phase = 'STARTING_NAV2'
                    self._message = f'{label} sudah aktif; localization valid ({detail})'
                self._activate_navigation()
                with self._lock:
                    self._active_slot = slot
                    self._active_map_name = label
                    self._loaded_yaml = str(yaml_path)
                    self._nav_enabled = True
                    self._phase = 'NAV2_READY'
                    self._message = f'NAV2 READY • {label}'
                return

            with self._lock:
                self._phase = 'STOPPING_NAV2'
                self._message = 'Cancel goal + pause planner/controller/BT sebelum ganti map'
            goals_cancelled = self._cancel_goals()
            pause_warnings = self._pause_navigation(goals_cancelled=goals_cancelled)
            with self._lock:
                self._nav_enabled = False
                self._phase = 'PREPARING_MAP'
                self._message = ('Nav2 paused; membuat planning map' +
                                 (f'; warning={" | ".join(pause_warnings)}' if pause_warnings else ''))

            planning_yaml = self._prepare_planning_map(slot, yaml_path)
            with self._lock:
                self._phase = 'ENSURING_MAP_SERVERS'
                self._message = 'Memastikan map_server + nav_map_server ACTIVE sebelum reload map'
            self._ensure_map_servers_active(timeout=30.0)
            with self._lock:
                self._phase = 'LOADING_MAP'
                self._message = f'Load raw {label} ke /map dan planning map ke /nav_map'
            raw_load_started = time.monotonic()
            self._load_map(self._load_raw, '/map_server/load_map', yaml_path)
            self._wait_for_loaded_grid(yaml_path, 'raw', raw_load_started, timeout=12.0)
            nav_load_started = time.monotonic()
            self._load_map(self._load_nav, '/nav_map_server/load_map', planning_yaml)
            self._wait_for_loaded_grid(planning_yaml, 'nav', nav_load_started, timeout=12.0)
            with self._lock:
                self._active_slot = slot
                self._active_map_name = label
                self._loaded_yaml = str(yaml_path)
                self._planning_yaml = str(planning_yaml)
                self._phase = 'RESETTING_LOCALIZATION'
                self._message = f'{label} loaded; reset AMCL agar pose map lama tidak dipakai'

            self._reset_and_start_amcl()
            known_pose, init_detail = self._map_bound_pose(slot, yaml_path)
            with self._lock:
                self._localization_epoch = time.monotonic()
                self._samples.clear()
                self._phase = 'KNOWN_POSE' if known_pose is not None else 'GLOBAL_LOCALIZATION'
                self._message = (f'AMCL ACTIVE pada {label}; {init_detail}')
            if known_pose is not None:
                # Publish more than once to survive DDS discovery races immediately
                # after the lifecycle reset. AMCL convergence/TF gates remain strict.
                for _ in range(3):
                    self._publish_known_pose(known_pose)
                    time.sleep(0.25)
            else:
                self._request_global_localization()

            base_timeout = max(15.0, float(self.get_parameter('localization_timeout_sec').value))
            specific = (float(self.get_parameter('known_pose_localization_timeout_sec').value)
                        if known_pose is not None else
                        float(self.get_parameter('global_localization_timeout_sec').value))
            timeout = max(15.0, min(base_timeout, max(15.0, specific)))
            deadline = time.monotonic() + timeout
            last_nomotion = 0.0
            last_pose_retry = 0.0
            detail = 'waiting'
            stable_ready = 0
            while rclpy.ok() and time.monotonic() < deadline:
                ok, detail = self._localized(0.10)
                if ok:
                    stable_ready += 1
                    # Require several consecutive good samples so one lucky TF
                    # lookup under CPU load cannot prematurely declare NAV2 READY.
                    if stable_ready >= 3:
                        break
                else:
                    stable_ready = 0
                now = time.monotonic()
                if known_pose is not None and now - last_pose_retry >= 2.0 and len(self._samples) < 2:
                    last_pose_retry = now
                    self._publish_known_pose(known_pose)
                if now - last_nomotion >= 1.0:
                    last_nomotion = now
                    self._request_nomotion()
                with self._lock:
                    self._phase = 'WAITING_LOCALIZATION'
                    self._message = f'{label}: {init_detail}; menunggu AMCL converge • {detail}'
                time.sleep(0.35)
            else:
                raise RuntimeError(
                    f'AMCL {label} belum converge dalam {timeout:.0f}s ({detail}); Nav2 tetap OFF')

            self._persist_converged_pose(slot, yaml_path)
            with self._lock:
                self._phase = 'STARTING_NAV2'
                self._message = f'Localization {label} valid ({detail}); pose map-bound diperbarui; activate Nav2'
            self._activate_navigation()
            with self._lock:
                self._phase = 'REFRESHING_COSTMAP'
                self._message = f'Nav2 ACTIVE pada {label}; rebuild global costmap dari /nav_map'
            costmap_warnings = self._refresh_costmaps_after_map_switch()
            if costmap_warnings:
                self.get_logger().warning(
                    'Costmap refresh warning: ' + ' | '.join(costmap_warnings))
            # Lifecycle nodes are now genuinely ACTIVE. Keep that fact visible while
            # AMCL refreshes map->odom instead of converting a transient TF delay to ERROR.
            with self._lock:
                self._nav_enabled = True
                self._phase = 'LOCALIZATION_RECOVERY'
                self._message = f'Nav2 ACTIVE pada {label}; menunggu map->odom stabil'
            post_timeout = max(6.0, float(self.get_parameter('post_activation_tf_timeout_sec').value))
            post_deadline = time.monotonic() + post_timeout
            post_stable = 0
            post_detail = detail
            while rclpy.ok() and time.monotonic() < post_deadline:
                post_ok, post_detail = self._localized(0.10)
                if post_ok:
                    post_stable += 1
                    if post_stable >= 2:
                        break
                else:
                    post_stable = 0
                    self._request_nomotion()
                time.sleep(0.25)
            if post_stable < 2:
                raise RuntimeError(
                    f'Nav2 nodes ACTIVE tetapi AMCL/TF belum stabil dalam {post_timeout:.0f}s ({post_detail})')
            with self._lock:
                self._nav_enabled = True
                self._phase = 'NAV2_READY'
                self._error = ''
                self._message = f'NAV2 READY • {label}'
            self.get_logger().info(self._message)
        except Exception as exc:
            rollback_note = ''
            try:
                # Keep ROS lifecycle state consistent with the GUI after a hard switch failure.
                # Cancel goals first, then pause only navigation nodes; sensors/AMCL/EKF stay alive.
                cancelled = self._cancel_goals()
                self._pause_navigation(goals_cancelled=cancelled)
                rollback_note = '; Nav2 rollback=PAUSED'
            except Exception as rollback_exc:
                rollback_note = f'; rollback warning={rollback_exc}'
            with self._lock:
                self._nav_enabled = False
                self._phase = 'ERROR'
                self._error = str(exc)
                self._message = f'Nav2 switch gagal fail-safe: {exc}{rollback_note}'
            self.get_logger().error(self._message)
        finally:
            with self._lock:
                self._busy = False
                self._requested_slot = 0
                self._requested_map_name = ''
            self._publish_status()

    def _stop_worker(self):
        try:
            goals_cancelled = self._cancel_goals()
            warnings = self._pause_navigation(goals_cancelled=goals_cancelled)
            with self._lock:
                self._nav_enabled = False
                self._phase = 'NAV2_OFF'
                self._error = ''
                self._message = ('NAV2 OFF; map_server + AMCL + LiDAR/IMU/EKF tetap hidup' +
                                 (f'; warning={" | ".join(warnings)}' if warnings else ''))
            self.get_logger().info(self._message)
        except Exception as exc:
            with self._lock:
                self._nav_enabled = False
                self._phase = 'ERROR'
                self._error = str(exc)
                self._message = f'STOP NAV2 bermasalah: {exc}'
            self.get_logger().error(self._message)
        finally:
            with self._lock:
                self._busy = False
            self._publish_status()

    def _publish_status(self):
        # Refresh the three lifecycle states that are mandatory for an actual
        # navigation goal. This is intentionally rate-limited so the 0.5 s GUI
        # status timer does not overload lifecycle RPCs on the Jetson.
        now_core = time.monotonic()
        with self._lock:
            refresh_core = (now_core - self._core_nav_state_checked_mono) >= 1.0
            core_states = dict(self._core_nav_state_snapshot)
        if refresh_core:
            core_states = {
                name: self._node_state(name, timeout=0.20, force_refresh=True)
                for name in ('planner_server', 'controller_server', 'bt_navigator')
            }
            with self._lock:
                self._core_nav_state_snapshot = dict(core_states)
                self._core_nav_state_checked_mono = time.monotonic()
        core_ready = all(state == 'active' for state in core_states.values())

        # Lifecycle activation can finish after the startup snapshot. Reconcile
        # only from freshly confirmed states, never from an indefinitely stale
        # transition-event cache.
        with self._lock:
            if not self._busy and not self._nav_enabled and self._phase == 'NAV2_OFF' and core_ready:
                self._nav_enabled = True
                self._phase = 'NAV2_READY'
                self._message = 'Nav2 lifecycle ACTIVE; late discovery reconciled tanpa reset map/AMCL'
        self._amcl_tf_keepalive()
        # Never hold the controller lock during a TF lookup.  AMCL/lifecycle
        # callbacks need that lock too and were previously delayed by the
        # periodic status timer under high Jetson CPU load.
        localized_raw, localization_detail = self._localized(0.0)
        sensor_ok, sensor_detail, scan_age, odom_age = self._sensor_health()
        now_mono = time.monotonic()
        grace = max(0.5, float(self.get_parameter('localization_failure_grace_sec').value))
        with self._lock:
            if localized_raw:
                self._localization_latched = True
                self._localization_bad_since = 0.0
            elif not self._localization_latched:
                # Never call startup/discovery latency a recovery. Recovery is
                # meaningful only after this controller has once proven AMCL+TF.
                self._localization_bad_since = 0.0
            elif self._localization_bad_since <= 0.0:
                self._localization_bad_since = now_mono
            bad_since = self._localization_bad_since
            latched = self._localization_latched
        failure_age = 0.0 if localized_raw or bad_since <= 0.0 else max(0.0, now_mono - bad_since)
        # Hold the last proven-good localization through a brief tf2 cache miss,
        # but never mask a real scan/odom outage or a persistent TF failure.
        localized = bool(localized_raw or (latched and sensor_ok and failure_age < grace))
        tf_pose = {}
        tf_chain = []
        # Additive telemetry for BAB IV only.  This does not publish/modify TF;
        # it mirrors the current tf2 buffer so the web report can validate the
        # same chain used by Nav2 even when /amcl_pose is intentionally quiet
        # while the robot is stationary.
        for parent, child, is_static in (
                ('map', 'base_footprint', False),
                ('odom', 'base_footprint', False),
                ('base_footprint', 'lidar_link', True),
                ('base_footprint', 'imu_link', True)):
            try:
                # Telemetry only: never block the executor waiting for TF.
                # A cache miss simply skips this row for one status cycle; it
                # must not steal callback time from AMCL/EKF/Nav2.
                tf = self.tf_buffer.lookup_transform(
                    parent, child, Time(), timeout=Duration(seconds=0.0))
                tr = tf.transform.translation
                qr = tf.transform.rotation
                row = {
                    'parent': parent, 'child': child,
                    'x': float(tr.x), 'y': float(tr.y), 'z': float(tr.z),
                    'roll': 0.0, 'pitch': 0.0,
                    'yaw': float(yaw_from_quaternion(qr)),
                    'static': bool(is_static),
                    'stamp_sec': float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9,
                }
                tf_chain.append(row)
                if parent == 'map' and child == 'base_footprint':
                    tf_pose = {'x': row['x'], 'y': row['y'], 'yaw': row['yaw'],
                               'measurement_stamp_sec': row['stamp_sec']}
            except Exception:
                pass
        with self._lock:
            phase = self._phase
            message = self._message
            nav_enabled_requested = self._nav_enabled
            nav_enabled = bool(nav_enabled_requested and core_ready)
            if nav_enabled_requested and not self._busy and not core_ready:
                phase = 'NAV2_LIFECYCLE_WAIT'
                detail = ', '.join(f'{name}={state}' for name, state in core_states.items())
                message = f'Nav2 belum siap untuk goal; menunggu lifecycle ACTIVE ({detail})'
            elif nav_enabled and not self._busy and phase == 'NAV2_READY':
                if not sensor_ok:
                    phase = 'SENSOR_WAIT'
                    message = f'Nav2 nodes ACTIVE; menunggu sensor localization ({sensor_detail})'
                elif not latched and not localized_raw:
                    phase = 'LOCALIZATION_WAIT'
                    message = 'Nav2 nodes ACTIVE; menunggu baseline AMCL/TF pertama (bukan recovery)'
                elif not localized_raw and localized:
                    # Debounce only: keep ACTIVE while a short zero-wait TF miss
                    # is being rechecked. No AMCL reset/restart is performed.
                    message = (f'NAV2 READY • transient TF check {failure_age:.1f}/{grace:.1f}s; '
                               'last localization tetap dipertahankan')
                elif not localized_raw:
                    phase = 'LOCALIZATION_RECOVERY'
                    message = (f'Nav2 nodes ACTIVE; TF localization gagal persisten '
                               f'{failure_age:.1f}s; request no-motion recovery aktif')
            payload = {
                'busy': self._busy,
                'phase': phase,
                'nav_enabled': nav_enabled,
                'nav_ready': bool(nav_enabled and localized),
                'core_lifecycle_ready': bool(core_ready),
                'core_lifecycle_states': dict(core_states),
                'active_slot': self._active_slot,
                'requested_slot': self._requested_slot,
                'active_map_name': self._active_map_name or (self._slot_label(self._active_slot) if self._active_slot else ''),
                'requested_map_name': self._requested_map_name,
                'loaded_yaml': self._loaded_yaml,
                'planning_yaml': self._planning_yaml,
                'mapping_active': self._mapping_active,
                'localized': localized,
                'localized_raw': bool(localized_raw),
                'localization_failure_age_sec': round(float(failure_age), 3),
                'localization_failure_grace_sec': round(float(grace), 3),
                'sensor_localization_ok': bool(sensor_ok),
                'sensor_localization_detail': sensor_detail,
                'scan_age_sec': None if not math.isfinite(scan_age) else round(float(scan_age), 3),
                'odom_age_sec': None if not math.isfinite(odom_age) else round(float(odom_age), 3),
                'localization_detail': localization_detail,
                'tf_pose': tf_pose,
                'tf_chain': tf_chain,
                'error': self._error,
                'message': message,
                'hardware_untouched': True,
            }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._status_pub.publish(msg)
        self._nav2_ready_pub.publish(Bool(data=bool(payload['nav_ready'])))


def main(args=None):
    rclpy.init(args=args)
    node = NavMapSwitchController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        try:
            if hasattr(node.tf_listener, 'executor'):
                node.tf_listener.executor.shutdown(timeout_sec=1.0)
            if hasattr(node.tf_listener, 'dedicated_listener_thread'):
                node.tf_listener.dedicated_listener_thread.join(timeout=1.0)
            node.tf_listener.unregister()
            node._tf_listener_node.destroy_node()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
