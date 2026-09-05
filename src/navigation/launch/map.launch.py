#!/usr/bin/env python3
"""Headless mapping entrypoint. RViz/native mapping GUI are intentionally not started."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('web_bind_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('web_port', default_value='5005'),
        DeclareLaunchArgument('web_read_only', default_value='false'),
    ]
    controller = Node(
        package='navigation', executable='mapping_web_controller.py',
        name='mapping_web_controller', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])
    web = Node(
        package='navigation', executable='agv_web_gui', name='agv_web_gui',
        output='screen', respawn=True, respawn_delay=2.0,
        parameters=[{
            'bind_address': LaunchConfiguration('web_bind_address'),
            'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
            'read_only': ParameterValue(LaunchConfiguration('web_read_only'), value_type=bool),
            'camera_jpeg_fps': 5.0,
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }])
    return LaunchDescription(args + [
        LogInfo(msg='[MAPPING-HEADLESS] localhost control active; RViz/native GUI disabled'),
        controller, web])
