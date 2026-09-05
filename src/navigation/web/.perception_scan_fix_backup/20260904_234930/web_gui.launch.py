#!/usr/bin/env python3
"""Standalone browser HMI for an already-running ADV ROS2 stack."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="5005"),
        DeclareLaunchArgument("read_only", default_value="false"),
        DeclareLaunchArgument("camera_jpeg_fps", default_value="8.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    web = Node(
        package="navigation",
        executable="agv_web_gui",
        name="agv_web_gui",
        output="screen",
        parameters=[{
            "bind_address": LaunchConfiguration("bind_address"),
            "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
            "read_only": ParameterValue(LaunchConfiguration("read_only"), value_type=bool),
            "camera_jpeg_fps": ParameterValue(LaunchConfiguration("camera_jpeg_fps"), value_type=float),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        # Browser Live Perception must show the hole-guided alignment HUD:
        # setpoint box + validated MIDDLE BLOCK, not the raw YOLO all-box view.
        # Remapping is local to agv_web_gui; the original camera/YOLO topics
        # remain unchanged for all other ROS consumers.
        remappings=[
            # Browser JPEG uses the native raw camera stream for maximum FPS.
            # Alignment geometry is drawn client-side from lightweight JSON.
            ("/obstacle_detection/visualization", "/web_gui/disabled_obstacle_visualization"),
            ("/camera/yolop/image_annotated", "/web_gui/disabled_legacy_visualization"),
        ],
    )
    return LaunchDescription(args + [web])
