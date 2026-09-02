#!/usr/bin/env python3
"""GUI master native C++/Qt + autonomous runtime source utama."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _source_config_dir(share_path: str, package_name: str) -> str:
    """Prefer workspace source config so GUI edits the YAML used by this source."""
    share = Path(share_path).resolve()
    parts = list(share.parts)
    if 'install' in parts:
        index = parts.index('install')
        workspace = Path(*parts[:index]) if index > 0 else Path('/')
        candidate = workspace / 'src' / package_name / 'config'
        if candidate.is_dir():
            return str(candidate.resolve())
    return str(share / 'config')


def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    nav_config_dir = os.environ.get('AGV_CONFIG_DIR', '').strip() or _source_config_dir(
        nav_share, 'navigation')

    try:
        esc_share = get_package_share_directory('esc')
        esc_config_dir = os.environ.get('AGV_ESC_CONFIG_DIR', '').strip() or _source_config_dir(
            esc_share, 'esc')
    except PackageNotFoundError:
        esc_config_dir = ''

    try:
        perception_share = get_package_share_directory('yolo_obstacle_detection_ros2')
        perception_config_dir = (
            os.environ.get('AGV_PERCEPTION_CONFIG_DIR', '').strip()
            or _source_config_dir(perception_share, 'yolo_obstacle_detection_ros2')
        )
    except PackageNotFoundError:
        perception_config_dir = ''

    start_autonomous = LaunchConfiguration('start_autonomous')
    args = [
        DeclareLaunchArgument('start_autonomous', default_value='false'),
        DeclareLaunchArgument('map', default_value='auto'),
        DeclareLaunchArgument('maps_dir', default_value='/home/otomasi2/ros/maps'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('auto_global_localization', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('use_gpu_decode', default_value='true'),
        DeclareLaunchArgument('require_gpu_decode', default_value='false'),
        DeclareLaunchArgument('use_tensorrt', default_value='true'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('enable_hole_alignment', default_value='true'),
    ]

    gui = Node(
        package='navigation',
        executable='agv_gui',
        name='agv_gui_process',
        output='screen',
        emulate_tty=True,
        additional_env={'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb')},
    )

    # GUI acquisition is independent of saved-map/Nav2 readiness.  By default
    # the root GUI command starts only this sensor/data runtime.  Passing
    # start_autonomous:=true preserves the original autonomous include and
    # disables this branch, preventing duplicate hardware owners.
    gui_sensor_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'gui_sensor_runtime.launch.py')),
        condition=UnlessCondition(start_autonomous),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
            'enable_yolo': LaunchConfiguration('enable_yolo'),
            'enable_hole_alignment': LaunchConfiguration('enable_hole_alignment'),
        }.items(),
    )

    autonomous_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'autonomous.launch.py')),
        condition=IfCondition(start_autonomous),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'maps_dir': LaunchConfiguration('maps_dir'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_nav2': LaunchConfiguration('enable_nav2'),
            'auto_global_localization': LaunchConfiguration('auto_global_localization'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'use_gpu_decode': LaunchConfiguration('use_gpu_decode'),
            'require_gpu_decode': LaunchConfiguration('require_gpu_decode'),
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
            'enable_yolo': LaunchConfiguration('enable_yolo'),
            'enable_hole_alignment': LaunchConfiguration('enable_hole_alignment'),
            'enable_rviz': 'false',
        }.items(),
    )

    stop_when_gui_closes = RegisterEventHandler(
        OnProcessExit(
            target_action=gui,
            on_exit=[EmitEvent(event=Shutdown(reason='AGV GUI closed'))],
        )
    )

    environment = [
        SetEnvironmentVariable('AGV_CONFIG_DIR', nav_config_dir),
        SetEnvironmentVariable('AGV_ESC_CONFIG_DIR', esc_config_dir),
        SetEnvironmentVariable('AGV_PERCEPTION_CONFIG_DIR', perception_config_dir),
        SetEnvironmentVariable('QT_QPA_PLATFORM', os.environ.get('QT_QPA_PLATFORM', 'xcb')),
        SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '1'),
        SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '1'),
        SetEnvironmentVariable('QT_SCALE_FACTOR_ROUNDING_POLICY', 'PassThrough'),
    ]

    return LaunchDescription(args + environment + [
        LogInfo(msg='[AGV-GUI] native C++/Qt master interface starting (RViz disabled in GUI mode)'),
        gui,
        stop_when_gui_closes,
        TimerAction(period=0.20, actions=[gui_sensor_include]),
        TimerAction(period=0.8, actions=[autonomous_include]),
    ])
