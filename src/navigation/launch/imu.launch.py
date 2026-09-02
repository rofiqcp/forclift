#!/usr/bin/env python3
"""Launch the B2imu WT901 IMU driver (master reference).

This matches the supplied master behavior: 115200 baud, 50 Hz,
on-board AHRS orientation on /imu/data, using /tmp/agv_devices/imu
from the USB role resolver.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def _runtime_config(package_share: str, package_name: str, filename: str) -> str:
    """Return persistent runtime YAML, seeding it once from package defaults."""
    ws = os.environ.get("AGV_WS", "/home/otomasi2/ros")
    base = os.environ.get("AGV_RUNTIME_CONFIG_ROOT", os.path.join(ws, "config", "runtime"))
    target_dir = os.path.join(os.path.expanduser(base), package_name)
    os.makedirs(target_dir, exist_ok=True)
    target = os.path.join(target_dir, filename)
    source = os.path.join(package_share, "config", filename)
    if not os.path.exists(target) and os.path.isfile(source):
        shutil.copy2(source, target)
    return target if os.path.isfile(target) else source

def generate_launch_description():
    params_file = _runtime_config(get_package_share_directory('navigation'), 'navigation', 'imu.yaml')

    args = [
        DeclareLaunchArgument('port',
            default_value='/tmp/agv_devices/imu',
            description='Stable IMU alias from USB role resolver'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    node = Node(
        package='navigation',
        executable='imu_node',
        name='imu_node',
        output='screen',
        respawn=True,
        respawn_delay=1.5,
        parameters=[params_file, {
            'port': LaunchConfiguration('port'),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])
