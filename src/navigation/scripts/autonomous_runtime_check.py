#!/usr/bin/python3
"""Runtime health check for autonomous.launch.py on the real AGV.

Checks that real messages arrive from IMU, LiDAR, ESC/EKF local odometry, camera,
YOLO visualization and AMCL, verifies ESC readiness, and validates the TF chain required by RViz/Nav2.
/lidar/odom is reported as an OPTIONAL diagnostic and is not required for PASS.
It never publishes control commands and is safe to run while autonomous is up.
"""

import math
import os
import statistics
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
)
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from yolo_obstacle_detection_ros2.msg import ObstacleArray, AlignmentState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from tf2_ros import Buffer, TransformListener


class RuntimeCheck(Node):
    def __init__(self):
        super().__init__('autonomous_runtime_check')
        self.counts = {
            '/imu/data': 0,
            '/scan_nav': 0,
            '/scan_safety': 0,
            '/lidar/odom': 0,
            '/odometry/filtered': 0,
            '/camera/color/image_raw': 0,
            '/obstacle_detection/visualization': 0,
            '/obstacle_detection/status': 0,
            '/obstacle_detection/performance': 0,
            '/obstacle_detection/obstacles': 0,
            '/fork_alignment/state': 0,
            '/amcl_pose': 0,
            '/map': 0,
            '/global_costmap/costmap': 0,
            '/local_costmap/costmap': 0,
            '/smac_plan': 0,
            '/transformed_global_plan': 0,
            '/trajectories': 0,
        }
        self.first_lidar = None
        self.last_lidar = None
        self.first_ekf = None
        self.last_ekf = None
        self.esc_ready_seen = False
        self.autonomy_gate_seen = False
        self.autonomy_gate_allowed = False
        self.lidar_safety_seen = False
        self.lidar_safety_healthy = False
        self.esc_ready = False
        self.scan_valid_beams = []
        self.last_planner_status = ''

        self.create_subscription(Imu, '/imu/data', self._cb('/imu/data'), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_nav', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_safety', self._cb('/scan_safety'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/lidar/odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf_cb, 10)
        state_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/esc/ready', self._esc_ready_cb, state_qos)
        self.create_subscription(Bool, '/lidar/safety_healthy', self._lidar_safety_cb, state_qos)
        self.create_subscription(Bool, '/system/autonomy_motion_allowed', self._autonomy_gate_cb, state_qos)
        self.create_subscription(Image, '/camera/color/image_raw', self._cb('/camera/color/image_raw'), qos_profile_sensor_data)
        self.create_subscription(Image, '/obstacle_detection/visualization', self._cb('/obstacle_detection/visualization'), qos_profile_sensor_data)
        self.create_subscription(String, '/obstacle_detection/status', self._cb('/obstacle_detection/status'), state_qos)
        self.create_subscription(String, '/obstacle_detection/performance', self._cb('/obstacle_detection/performance'), qos_profile_sensor_data)
        self.create_subscription(ObstacleArray, '/obstacle_detection/obstacles', self._cb('/obstacle_detection/obstacles'), 10)
        self.create_subscription(AlignmentState, '/fork_alignment/state', self._cb('/fork_alignment/state'), state_qos)
        self.create_subscription(String, '/navigation/planner_status', self._planner_status_cb, 10)
        # AMCL Humble publishes /amcl_pose as RELIABLE + TRANSIENT_LOCAL.
        # Match it so a stationary robot's last pose is still observed.
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb('/amcl_pose'), amcl_qos)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._cb('/map'), map_qos)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._cb('/global_costmap/costmap'), map_qos)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self._cb('/local_costmap/costmap'), map_qos)
        self.create_subscription(Path, '/smac_plan', self._cb('/smac_plan'), map_qos)
        self.create_subscription(
            Path, '/transformed_global_plan',
            self._cb('/transformed_global_plan'), qos_profile_sensor_data)
        self.create_subscription(
            MarkerArray, '/trajectories', self._cb('/trajectories'), 10)

        self.nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.compute_action = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.lifecycle_nodes = [
            'map_server', 'amcl', 'planner_server', 'controller_server',
            'behavior_server', 'bt_navigator', 'velocity_smoother',
            'collision_monitor',
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _cb(self, topic):
        def callback(_msg):
            self.counts[topic] += 1
        return callback

    def _odom_cb(self, msg):
        self.counts['/lidar/odom'] += 1
        p = msg.pose.pose.position
        sample = (float(p.x), float(p.y))
        if self.first_lidar is None:
            self.first_lidar = sample
        self.last_lidar = sample

    def _scan_cb(self, msg):
        self.counts['/scan_nav'] += 1
        valid = sum(
            1 for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max)
        if len(self.scan_valid_beams) < 500:
            self.scan_valid_beams.append(valid)

    def _lidar_safety_cb(self, msg):
        self.lidar_safety_seen = True
        self.lidar_safety_healthy = bool(msg.data)

    def _esc_ready_cb(self, msg):
        self.esc_ready_seen = True
        self.esc_ready = bool(msg.data)

    def _autonomy_gate_cb(self, msg):
        self.autonomy_gate_seen = True
        self.autonomy_gate_allowed = bool(msg.data)

    def _planner_status_cb(self, msg):
        self.last_planner_status = str(msg.data)

    def _ekf_cb(self, msg):
        self.counts['/odometry/filtered'] += 1
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        sample = (float(p.x), float(p.y), float(yaw))
        if self.first_ekf is None:
            self.first_ekf = sample
        self.last_ekf = sample


def main():
    rclpy.init()
    node = RuntimeCheck()
    duration_s = float(os.environ.get('AUTONOMOUS_CHECK_SECONDS', '10'))
    require_perception = os.environ.get(
        'AUTONOMOUS_CHECK_REQUIRE_PERCEPTION', '1').strip().lower() in {
            '1', 'true', 'yes', 'on'}
    require_mppi_output = os.environ.get(
        'AUTONOMOUS_CHECK_REQUIRE_MPPI_OUTPUT', '0').strip().lower() in {
            '1', 'true', 'yes', 'on'}
    require_rviz = os.environ.get(
        'AUTONOMOUS_CHECK_REQUIRE_RVIZ', '0').strip().lower() in {
            '1', 'true', 'yes', 'on'}
    deadline = time.monotonic() + max(3.0, duration_s)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        optional = {'/lidar/odom'}
        if not require_mppi_output:
            optional.update({'/smac_plan', '/transformed_global_plan', '/trajectories'})
        if not require_perception:
            optional.update({
                '/camera/color/image_raw',
                '/obstacle_detection/visualization',
                '/obstacle_detection/status',
                '/obstacle_detection/performance',
                '/obstacle_detection/obstacles',
                '/fork_alignment/state',
            })
        required = [topic for topic in node.counts if topic not in optional]
        print('\n=== AUTONOMOUS RUNTIME CHECK ===')
        failed = []
        for topic in required:
            count = node.counts[topic]
            status = 'PASS' if count > 0 else 'FAIL'
            print(f'{status:4s}  {topic:<40} messages={count}')
            if count <= 0:
                failed.append(topic)

        # Real transport-rate acceptance, not mere topic-name discovery.
        imu_rate = node.counts['/imu/data'] / max(3.0, duration_s)
        scan_rate = node.counts['/scan_nav'] / max(3.0, duration_s)
        imu_rate_ok = imu_rate >= 20.0
        scan_rate_ok = scan_rate >= 3.0
        print(f'{"PASS" if imu_rate_ok else "FAIL":4s}  /imu/data rate >= 20 Hz            measured={imu_rate:.2f} Hz')
        print(f'{"PASS" if scan_rate_ok else "FAIL":4s}  /scan_nav rate >= 3 Hz             measured={scan_rate:.2f} Hz')
        if not imu_rate_ok:
            failed.append('/imu/data_rate_low')
        if not scan_rate_ok:
            failed.append('/scan_nav_rate_low')

        median_beams = statistics.median(node.scan_valid_beams) if node.scan_valid_beams else 0.0
        scan_density_ok = median_beams >= 30
        print(
            f'{"PASS" if scan_density_ok else "FAIL":4s}  /scan_nav median usable beams >= 30 '
            f'measured={median_beams:.1f}')
        if not scan_density_ok:
            failed.append('/scan_nav_sparse_after_filter')

        lidar_diag_count = node.counts['/lidar/odom']
        print(f'INFO  {"/lidar/odom":<40} messages={lidar_diag_count} (diagnostic-only)')

        if not require_mppi_output:
            print(
                f'INFO  {"MPPI visualization":<40} '
                f'smac_plan={node.counts["/smac_plan"]} '
                f'transformed_plan={node.counts["/transformed_global_plan"]} '
                f'trajectories={node.counts["/trajectories"]} '
                '(set AUTONOMOUS_CHECK_REQUIRE_MPPI_OUTPUT=1 while a goal is ACTIVE)')

        lidar_safety_ok = node.lidar_safety_seen and node.lidar_safety_healthy
        print(f'{"PASS" if lidar_safety_ok else "FAIL":4s}  /lidar/safety_healthy true')
        if not lidar_safety_ok:
            failed.append('/lidar/safety_healthy=false_or_missing')

        esc_ok = node.esc_ready_seen and node.esc_ready
        print(f'{"PASS" if esc_ok else "FAIL":4s}  /esc/ready true')
        if not esc_ok:
            failed.append('/esc/ready=false_or_missing')

        gate_ok = node.autonomy_gate_seen and node.autonomy_gate_allowed
        print(f'{"PASS" if gate_ok else "FAIL":4s}  /system/autonomy_motion_allowed true')
        if not gate_ok:
            failed.append('/system/autonomy_motion_allowed=false_or_missing')

        compute_ok = node.compute_action.server_is_ready()
        print(f'{"PASS" if compute_ok else "FAIL":4s}  ACTION /compute_path_to_pose')
        if not compute_ok:
            failed.append('ACTION /compute_path_to_pose')

        action_ok = node.nav_action.server_is_ready()
        print(f'{"PASS" if action_ok else "FAIL":4s}  ACTION /navigate_to_pose')
        if not action_ok:
            failed.append('ACTION /navigate_to_pose')

        node_names = set(node.get_node_names())
        rviz_ok = 'rviz2_autonomous' in node_names
        rviz_label = 'PASS' if rviz_ok else ('FAIL' if require_rviz else 'INFO')
        print(f'{rviz_label:4s}  RViz node /rviz2_autonomous')
        if require_rviz and not rviz_ok:
            failed.append('RViz:/rviz2_autonomous')

        if node.last_planner_status:
            print('INFO  /navigation/planner_status = ' + node.last_planner_status)
        else:
            print('INFO  /navigation/planner_status = <no sample during check>')

        for lifecycle_name in node.lifecycle_nodes:
            service = f'/{lifecycle_name}/get_state'
            client = node.create_client(GetState, service)
            if not client.wait_for_service(timeout_sec=1.0):
                print(f'FAIL  lifecycle {lifecycle_name:<24} service missing')
                failed.append(f'lifecycle:{lifecycle_name}')
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            response = future.result()
            label = response.current_state.label if response is not None else 'NO_RESPONSE'
            ok = label.lower() == 'active'
            print(f'{"PASS" if ok else "FAIL":4s}  lifecycle {lifecycle_name:<24} state={label}')
            if not ok:
                failed.append(f'lifecycle:{lifecycle_name}={label}')

        tf_results = []
        for parent, child in [
            ('base_footprint', 'lidar_link'),
            ('base_footprint', 'imu_link'),
            ('odom', 'base_footprint'),
            ('map', 'odom'),
            ('map', 'visual_/base_footprint'),
        ]:
            ok = node.tf_buffer.can_transform(
                parent, child, Time(), timeout=Duration(seconds=1.0))
            tf_results.append((parent, child, ok))
            print(f'{"PASS" if ok else "FAIL":4s}  TF {parent} -> {child}')
            if not ok:
                failed.append(f'TF {parent}->{child}')

        if node.first_lidar is not None and node.last_lidar is not None:
            dx = node.last_lidar[0] - node.first_lidar[0]
            dy = node.last_lidar[1] - node.first_lidar[1]
            dist = math.hypot(dx, dy)
            print(f'INFO  /lidar/odom delta during check = {dist:.4f} m '
                  f'(dx={dx:.4f}, dy={dy:.4f})')
            if dist < 0.01:
                print('INFO  Robot appears stationary during this check; move the AGV to verify translation.')

        if require_perception:
            model_candidates = [
                '/home/otomasi2/forclift/models/yolov8n_agv_forklift_opencv.onnx',
                '/home/otomasi2/forclift/models/yolov8n_agv_forklift.onnx',
            ]
            model = next((path for path in model_candidates if os.path.isfile(path)), '')
            if model:
                print(f'PASS  YOLO AGV model file = {model}')
            else:
                print('WARN  YOLO AGV model file not found in /home/otomasi2/forclift/models')
                print('      Camera passthrough should still be visible; detections require the trained AGV model.')
        else:
            print('INFO  Perception checks disabled by AUTONOMOUS_CHECK_REQUIRE_PERCEPTION=0')

        if node.first_ekf is not None and node.last_ekf is not None:
            dx = node.last_ekf[0] - node.first_ekf[0]
            dy = node.last_ekf[1] - node.first_ekf[1]
            dyaw = math.atan2(math.sin(node.last_ekf[2] - node.first_ekf[2]),
                              math.cos(node.last_ekf[2] - node.first_ekf[2]))
            print(f'INFO  /odometry/filtered delta = {math.hypot(dx, dy):.4f} m, '
                  f'yaw={math.degrees(dyaw):.2f} deg')
            if math.hypot(dx, dy) < 0.01 and abs(dyaw) < math.radians(1.0):
                print('INFO  EKF pose did not move during this check. Move/rotate the AGV while the checker runs.')

        if failed:
            print('\nRESULT: FAIL -> ' + ', '.join(failed))
            return 2
        print('\nRESULT: PASS - sensors, localization, Nav2 lifecycle, costmaps, action and TF are alive.')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
