#!/usr/bin/env python3
"""GUI-only hardware/data runtime.

This launch intentionally does NOT start SLAM, AMCL, Nav2, map_server, planners,
or autonomous safety/navigation lifecycle nodes.  Its sole purpose is to make the
real data required by the engineering GUI available independently of whether the
autonomous stack has a valid saved map or reaches localization readiness.

Ownership in GUI mode:
  resolver -> /tmp/agv_devices/{imu,lidar}
  IMU      -> /imu/data
  LiDAR    -> /scan_safety, /scan_nav, compatibility /scan
  ESC      -> /esc/odom + /esc/joint_states
  EKF      -> /odometry/filtered
  alias    -> /odom (copy of /esc/odom for BAB-IV raw-odometry acquisition)
  camera   -> /camera/color/image_raw (+ perception outputs when enabled)
"""
from __future__ import annotations

import os

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from navigation_runtime.lidar_safety_config import ensure_stage2_lidar_runtime
from navigation_runtime.localization_config import ensure_stage5_localization_runtime
from navigation_runtime.vehicle_geometry import sync_derived_configs


def _runtime_config(package_share: str, package_name: str, filename: str) -> str:
    ws = os.environ.get('AGV_WS', '/home/otomasi2/ros')
    root = os.environ.get('AGV_RUNTIME_CONFIG_ROOT', os.path.join(ws, 'config', 'runtime'))
    target = os.path.join(os.path.expanduser(root), package_name, filename)
    source = os.path.join(package_share, 'config', filename)
    return target if os.path.isfile(target) else source


def _success_only_exit(target_action, success_actions, stage_name: str):
    def _handle(event, _context):
        rc = getattr(event, 'returncode', None)
        if rc == 0:
            return list(success_actions)
        return [LogInfo(msg=f'[GUI-SENSOR] {stage_name} belum siap (rc={rc}); GUI tetap hidup dan menunggu run berikutnya.')]
    return RegisterEventHandler(OnProcessExit(target_action=target_action, on_exit=_handle))


def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    esc_share = get_package_share_directory('esc')
    yolo_share = get_package_share_directory('yolo_obstacle_detection_ros2')

    # These are configuration migrations only. They do not start autonomous.
    # They also recover the two legacy malformed runtime YAML forms that used to
    # abort launch before any sensor publisher could be created.
    ensure_stage2_lidar_runtime(nav_share)
    ensure_stage5_localization_runtime(nav_share)
    geometry_state = sync_derived_configs(nav_share, esc_share)
    g = geometry_state['geometry']

    robot_description = xacro.process_file(
        os.path.join(nav_share, 'urdf', 'agv.urdf.xacro'),
        mappings={
            'cad_to_base_yaw_rad': str(g['cad_to_base_yaw_rad']),
            'max_steering_angle_rad': str(g['max_steering_angle_rad']),
        }).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_port', default_value='/tmp/agv_devices/imu'),
        DeclareLaunchArgument('imu_baudrate', default_value='921600'),
        DeclareLaunchArgument('lidar_port', default_value='/tmp/agv_devices/lidar'),
        DeclareLaunchArgument('lidar_baudrate', default_value='230400'),
        DeclareLaunchArgument('esc_port', default_value='/dev/esc'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('use_tensorrt', default_value='true'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('enable_hole_alignment', default_value='true'),
    ]

    robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher_gui_data', output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    esc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(esc_share, 'launch', 'esc.launch.py')),
        launch_arguments={
            'profile': 'ackermann_1_board.yaml',
            'board0_port': LaunchConfiguration('esc_port'),
            'use_sim_time': use_sim_time,
            'enable_keyboard': 'false',
            'enable_joystick': 'false',
            'enable_nav2': 'false',
            'standalone_mode': 'false',
            'require_autonomy_gate': 'false',
            'require_manual_gate': 'false',
            'joint_states_topic': '/esc/joint_states',
            'offline_zero_output': 'true',
            'enable_winch': 'false',
        }.items())

    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_gui_data', output='screen',
        parameters=[
            _runtime_config(nav_share, 'navigation', 'ekf_autonomous.yaml'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[('odometry/filtered', '/odometry/filtered')])

    odom_alias = Node(
        package='navigation', executable='gui_odom_alias.py',
        name='gui_odom_alias', output='screen',
        parameters=[{
            'source_topic': '/esc/odom',
            'output_topic': '/odom',
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'imu.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('imu_port'),
            'baudrate': LaunchConfiguration('imu_baudrate'),
            'frame_id': 'imu_link',
            'use_sim_time': use_sim_time,
        }.items())

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'lidar.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('lidar_port'),
            'baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'lidar_link',
            'intensity_mode': 'true',
            'intensity_bits': '16',
            'strict_checksum': 'true',
            'use_sim_time': use_sim_time,
        }.items())

    # Remove stale role aliases only; unlike autonomous preflight this does not
    # kill/replug hardware owners or change the autonomous startup implementation.
    cleanup_aliases = ExecuteProcess(
        cmd=['bash', '-c',
             "mkdir -p /tmp/agv_devices; rm -f /tmp/agv_devices/imu /tmp/agv_devices/lidar; "
             "echo '[GUI-SENSOR] stale serial aliases cleaned'"],
        output='screen')

    role_resolver = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', os.path.join(nav_share, 'tools', 'resolve_usb_roles.py'),
            '--output-dir', '/tmp/agv_devices',
            '--require-imu', 'true', '--require-lidar', 'true', '--require-camera', 'false',
            '--stable-seconds', '0.25',
        ], output='screen')

    ready_gate_script = os.path.join(
        get_package_prefix('navigation'), 'lib', 'navigation', 'serial_transport_ready_gate.py')
    imu_transport_ready = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', ready_gate_script,
            '--imu', LaunchConfiguration('imu_port'), '--lidar', LaunchConfiguration('lidar_port'),
            '--require-imu', 'true', '--require-lidar', 'false',
            '--stable-cycles', '1', '--poll-sec', '0.10', '--timeout-sec', '0',
        ], output='screen')
    lidar_transport_ready = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', ready_gate_script,
            '--imu', LaunchConfiguration('imu_port'), '--lidar', LaunchConfiguration('lidar_port'),
            '--require-imu', 'false', '--require-lidar', 'true',
            '--stable-cycles', '1', '--poll-sec', '0.10', '--timeout-sec', '0',
        ], output='screen')

    start_resolver = _success_only_exit(cleanup_aliases, [role_resolver], 'alias-cleanup')
    start_transport = _success_only_exit(
        role_resolver, [imu_transport_ready, lidar_transport_ready], 'serial-role-resolver')
    start_imu = _success_only_exit(
        imu_transport_ready, [TimerAction(period=0.05, actions=[imu])], 'imu-transport')
    start_lidar = _success_only_exit(
        lidar_transport_ready, [TimerAction(period=0.05, actions=[lidar])], 'lidar-transport')

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yolo_share, 'launch', 'perception_all.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        launch_arguments={
            'enable_camera': 'true',
            'enable_yolo': LaunchConfiguration('enable_yolo'),
            'enable_hole_alignment': LaunchConfiguration('enable_hole_alignment'),
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
            'use_sim_time': use_sim_time,
        }.items())

    # A diagnostic gate makes startup state visible but never shuts the GUI down.
    topic_gate = Node(
        package='navigation', executable='allsystem_gate',
        name='gui_sensor_topic_gate', output='screen', emulate_tty=True,
        parameters=[{
            'gate_name': 'gui_data',
            'topics': ['/imu/data', '/scan', '/esc/odom', '/odom'],
            'min_messages': 2,
            'timeout_ms': 60000,
            'exit_on_ready': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    return LaunchDescription(args + [
        LogInfo(msg='[GUI-SENSOR] data runtime starting; no SLAM/AMCL/Nav2 ownership'),
        robot_state,
        esc,
        ekf,
        odom_alias,
        TimerAction(period=0.20, actions=[perception]),
        start_resolver,
        start_transport,
        start_imu,
        start_lidar,
        cleanup_aliases,
        TimerAction(period=1.0, actions=[topic_gate]),
    ])
