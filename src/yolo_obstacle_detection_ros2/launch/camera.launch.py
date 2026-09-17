#!/usr/bin/env python3
"""Launch the internal V4L2 Astra RGB camera driver.

This replaces the external astra_camera_ros2 package with a self-contained
Linux V4L2 driver that publishes /camera/color/image_raw + /camera/color/camera_info.

USB hub safe: uses /dev/v4l/by-id/by-path stable aliases instead of /dev/videoX.
Automatic device rediscovery on disconnect.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('yolo_obstacle_detection_ros2')
    ws = (os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or os.path.expanduser('~/forclift'))
    runtime_root = os.environ.get('AGV_RUNTIME_CONFIG_ROOT', os.path.join(ws, 'config', 'runtime'))
    runtime_file = os.path.join(runtime_root, 'yolo_obstacle_detection_ros2', 'camera_v4l2.yaml')
    params_file = runtime_file if os.path.isfile(runtime_file) else os.path.join(share_dir, 'config', 'camera_v4l2.yaml')

    args = [
        DeclareLaunchArgument('device', default_value='auto',
            description='Camera device: "auto" (discover) or /dev/videoX path'),
        DeclareLaunchArgument('width', default_value='640',
            description='Camera resolution width'),
        DeclareLaunchArgument('height', default_value='480',
            description='Camera resolution height'),
        DeclareLaunchArgument('fps', default_value='30',
            description='Camera framerate'),
        DeclareLaunchArgument('pixel_format', default_value='MJPG',
            description='Pixel format: MJPG (preferred) or YUYV'),
        DeclareLaunchArgument('frame_id', default_value='camera_color_optical_frame',
            description='ROS frame ID for camera optical frame'),
        DeclareLaunchArgument('calibration_url', default_value='',
            description='Path to camera_calibration YAML (optional)'),
        DeclareLaunchArgument('reconnect_interval_ms', default_value='1000',
            description='Reconnect interval in milliseconds'),
        DeclareLaunchArgument('frame_timeout_ms', default_value='3000',
            description='Frame timeout in milliseconds'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gpu_decode', default_value='true',
            description='Use NVIDIA nvv4l2decoder hardware decode for MJPG (GPU).'),
        DeclareLaunchArgument('require_gpu_decode', default_value='false',
            description='Prefer GPU but allow CPU V4L2 fallback so camera images remain available.'),
    ]

    camera_node = Node(
        package='yolo_obstacle_detection_ros2',
        executable='astra_rgb_v4l2_node',
        name='astra_rgb_v4l2_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        # Runtime YAML is the tuning authority. Structural camera changes are
        # applied by respawning this node; use_sim_time is the only launch override.
        parameters=[params_file, {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription(args + [camera_node])
