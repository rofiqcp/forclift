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
    params_file = os.path.join(share_dir, 'config', 'camera_v4l2.yaml')

    args = [
        DeclareLaunchArgument('device', default_value='auto',
            description='Camera device: "auto" (discover) or /dev/videoX path'),
        DeclareLaunchArgument('width', default_value='1280',
            description='Camera resolution width'),
        DeclareLaunchArgument('height', default_value='720',
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
        parameters=[params_file, {
            'device': LaunchConfiguration('device'),
            'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('fps'), value_type=int),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'frame_id': LaunchConfiguration('frame_id'),
            'calibration_url': LaunchConfiguration('calibration_url'),
            'reconnect_interval_ms': ParameterValue(
                LaunchConfiguration('reconnect_interval_ms'), value_type=int),
            'frame_timeout_ms': ParameterValue(
                LaunchConfiguration('frame_timeout_ms'), value_type=int),
            'use_gpu_decode': ParameterValue(
                LaunchConfiguration('use_gpu_decode'), value_type=bool),
            'require_gpu_decode': ParameterValue(
                LaunchConfiguration('require_gpu_decode'), value_type=bool),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription(args + [camera_node])
