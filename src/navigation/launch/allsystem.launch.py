#!/usr/bin/env python3
"""AGV all-system launch with USB-hub-safe staged startup.

One command:
  ros2 launch navigation allsystem.launch.py

Staging (now fully decoupled, resolver-safe):
  Stage A: map.launch.py resolves IMU + LiDAR FIRST (camera disabled here).
           role_resolver_serial creates /tmp/agv_devices/{imu,lidar} aliases.
           Serial sensors start, sensor_gate waits for /imu/data + /scan_nav.
  Stage B: allsystem sensor_gate verifies IMU+LiDAR alive.
  Stage C: Camera resolved separately via role_resolver_camera (from map.launch.py).
           Camera node + camera_gate start only after camera resolver succeeds.
  Stage D: YOLO starts only after camera_gate verifies /camera/color/image_raw.

ISSUE A FIX: Sensors start only after resolver succeeds (OnProcessExit gating).
ISSUE B FIX: Camera decoupled from IMU/LiDAR — separate resolver runs first.
ISSUE C FIX: Stale aliases cleaned by cleanup_stale_aliases in map.launch.py.

This prevents the camera from blocking or destabilizing serial sensor resolution.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
                            LogInfo, EmitEvent)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue




def _success_only_exit(target_action, success_actions, stage_name: str, shutdown_on_failure: bool = True):
    def _handle(event, _context):
        rc = getattr(event, "returncode", None)
        if rc == 0:
            return list(success_actions)
        reason = f"ALLSYSTEM gate {stage_name} failed with exit code {rc}"
        actions = [LogInfo(msg="[ALLSYSTEM-GATE] ERROR: " + reason)]
        if shutdown_on_failure:
            actions.append(EmitEvent(event=Shutdown(reason=reason + "; fail-closed shutdown")))
        return actions
    return RegisterEventHandler(OnProcessExit(target_action=target_action, on_exit=_handle))

def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    workspace = os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or str(Path.home() / 'forclift')
    yolo_share = get_package_share_directory('yolo_obstacle_detection_ros2')

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_pixel_format', default_value='MJPG'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument(
            'yolo_model',
            default_value=os.path.join(workspace, 'models', 'yolov8n_agv_forklift.onnx')),
        DeclareLaunchArgument('use_tensorrt', default_value='false'),
        DeclareLaunchArgument('yolo_engine', default_value=''),
    ]

    # Reuse the mapping launch that already contains the working IMU/LiDAR
    # startup. Explicitly keep camera/YOLO/RViz out of this include so only one
    # process owns each device and allsystem controls the downstream stages.
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'map.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_camera': 'false',
            'enable_yolo': 'false',
            'enable_rviz': 'false',
        }.items())

    sensor_gate = Node(
        package='navigation',
        executable='allsystem_gate',
        name='allsystem_gate_sensor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gate_name': 'sensor',
            'topics': ['/imu/data', '/scan_nav'],
            'min_messages': 3,
            'timeout_ms': 60000,
            'exit_on_ready': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_share, 'launch', 'perception_all.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        launch_arguments={
            'enable_camera': 'true',
            'enable_yolo': LaunchConfiguration('enable_yolo'),
            'enable_hole_alignment': 'false',
            'use_composition': 'true',
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'camera_pixel_format': LaunchConfiguration('camera_pixel_format'),
            'use_sim_time': use_sim_time,
        }.items())

    camera_gate = Node(
        package='navigation',
        executable='allsystem_gate',
        name='allsystem_gate_camera',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        parameters=[{
            'gate_name': 'camera',
            'topics': ['/camera/color/image_raw'],
            'min_messages': 3,
            'timeout_ms': 60000,
            'exit_on_ready': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    # YOLO is loaded with the camera in perception_all component container.

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_allsystem',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        arguments=['-d', os.path.join(nav_share, 'rviz', 'allsystem.rviz')],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}])

    # When /imu/data + /scan are proven alive, sensor_gate exits cleanly and
    # only then are the camera process and its frame gate launched.
    start_camera_after_serial = _success_only_exit(
        sensor_gate, [camera, camera_gate], 'serial-sensors-ready')

    return LaunchDescription(args + [
        mapping,
        sensor_gate,
        start_camera_after_serial,
        rviz,
    ])
