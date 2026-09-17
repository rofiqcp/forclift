#!/usr/bin/env python3
"""Complete RGB perception launch: camera + YOLO + hole-block alignment."""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    yolo_share = get_package_share_directory('yolo_obstacle_detection_ros2')
    workspace = os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or str(Path.home() / 'forclift')
    runtime_root = os.environ.get('AGV_RUNTIME_CONFIG_ROOT', os.path.join(workspace, 'config', 'runtime'))
    runtime_dir = os.path.join(os.path.expanduser(runtime_root), 'yolo_obstacle_detection_ros2')

    def runtime_or_fallback(filename, fallback):
        candidate = os.path.join(runtime_dir, filename)
        return candidate if os.path.isfile(candidate) else fallback

    default_model = os.path.join(workspace, 'models', 'yolov8n_agv_forklift_opencv.onnx')
    default_engine = os.path.join(workspace, 'models', 'yolov8n_agv_forklift_opencv.engine')
    camera_params = runtime_or_fallback('camera_v4l2.yaml', os.path.join(yolo_share, 'config', 'camera_v4l2.yaml'))
    yolo_params = runtime_or_fallback('yolo_detection.yaml', os.path.join(yolo_share, 'config', 'yolo_detection.yaml'))
    alignment_params = runtime_or_fallback(
        'alignment_realtime.yaml', os.path.join(yolo_share, 'hole_block_alignment', 'alignment_realtime.yaml'))

    args = [
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('engine', default_value=default_engine),
        DeclareLaunchArgument('use_tensorrt', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('enable_hole_alignment', default_value='true'),
        DeclareLaunchArgument('enable_pallet_pose', default_value='false',
                              description='Compatibility argument; RGB-only V4L2 has no depth source'),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_pixel_format', default_value='MJPG'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    camera_node = Node(
        package='yolo_obstacle_detection_ros2',
        executable='astra_rgb_v4l2_node',
        name='astra_rgb_v4l2_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        parameters=[camera_params, {
            'device': LaunchConfiguration('camera_device'),
            'width': ParameterValue(LaunchConfiguration('camera_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('camera_height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=int),
            'pixel_format': LaunchConfiguration('camera_pixel_format'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    obstacle_node = Node(
        package='yolo_obstacle_detection_ros2',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_yolo')),
        parameters=[yolo_params, {
            'model_path': LaunchConfiguration('model'),
            'engine_path': LaunchConfiguration('engine'),
            'use_tensorrt': ParameterValue(LaunchConfiguration('use_tensorrt'), value_type=bool),
            'input_topic': '/camera/color/image_raw',
            'output_topic': '/obstacle_detection/obstacles',
            'visualization_topic': '/obstacle_detection/visualization',
            'max_inference_fps': 30.0,
            'max_visualization_fps': 6.0,
        }],
    )

    alignment_node = Node(
        package='yolo_obstacle_detection_ros2',
        executable='hole_block_alignment_node.py',
        name='hole_block_alignment_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_hole_alignment')),
        parameters=[{
            'config': alignment_params,
            'image_topic': '/camera/color/image_raw',
            'obstacle_topic': '/obstacle_detection/obstacles',
            'state_topic': '/fork_alignment/state',
            'output_image_topic': '/fork_alignment/image',
            'save_csv': False,
            'save_output_video': False,
            'visualization_fps': 8.0,
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription(args + [camera_node, obstacle_node, alignment_node])
