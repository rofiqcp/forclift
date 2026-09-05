"""Astra YOLOP GPU dengan satu RViz mandiri berlatensi rendah."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "perception"


# Fungsi: Menyusun LaunchDescription beserta node, parameter, kondisi, dan remapping yang diperlukan.
def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE_NAME)
    config_file = LaunchConfiguration("config_file")
    engine_path = LaunchConfiguration("engine_path")
    rviz_config = LaunchConfiguration("rviz_config")
    rgb_device = LaunchConfiguration("rgb_device")
    rgb_width = LaunchConfiguration("rgb_width")
    rgb_height = LaunchConfiguration("rgb_height")
    camera_fps = LaunchConfiguration("camera_fps")
    gpu_device = LaunchConfiguration("gpu_device")
    rviz_buffer_count = LaunchConfiguration("rviz_buffer_count")
    allow_fps_resolution_fallback = LaunchConfiguration(
        "allow_fps_resolution_fallback"
    )
    strict_camera_mode = LaunchConfiguration("strict_camera_mode")
    respawn_node = LaunchConfiguration("respawn_node")

    yolop_node = Node(
        package=PACKAGE_NAME,
        executable="perception_node",
        name="perception",
        output="screen",
        emulate_tty=True,
        respawn=PythonExpression(["'", respawn_node, "' == 'true'"]),
        respawn_delay=5.0,
        parameters=[
            config_file,
            {
                "engine_path": engine_path,
                "rgb_device": rgb_device,
                "rgb_width": ParameterValue(rgb_width, value_type=int),
                "rgb_height": ParameterValue(rgb_height, value_type=int),
                "fps": ParameterValue(camera_fps, value_type=int),
                "gpu_device": ParameterValue(gpu_device, value_type=int),
                "v4l2_pixel_format": "YUYV",
                "allow_mjpeg_cpu_fallback": False,
                "use_v4l2_userptr_zero_copy": True,
                "v4l2_buffer_count": 3,
                "allow_resolution_fallback_for_fps": ParameterValue(
                    allow_fps_resolution_fallback, value_type=bool
                ),
                "strict_camera_mode": ParameterValue(
                    strict_camera_mode, value_type=bool
                ),
                "publish_annotated": True,
                "publish_raw_rgb": False,
                "publish_masks": False,
                "publish_class_mask": False,
                "publish_detections": True,
                "publish_only_when_subscribed": True,
                "show_opencv_cuda_window": False,
                "draw_text_labels_cpu": False,
                "async_rviz_publish": True,
                "rviz_publish_buffer_count": ParameterValue(
                    rviz_buffer_count, value_type=int
                ),
                "publisher_reliability": "reliable",
                "annotated_topic": "/camera/yolop/image_annotated",
                "detections_topic": "/yolop/detections",
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_yolop",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(share, "config", "astra_yolop_gpu.yaml"),
            ),
            DeclareLaunchArgument(
                "engine_path",
                default_value="/home/sirobo/ros/models/yolopv2.engine",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(share, "rviz", "yolop_thesis.rviz"),
            ),
            DeclareLaunchArgument("rgb_device", default_value="auto"),
            DeclareLaunchArgument("rgb_width", default_value="1280"),
            DeclareLaunchArgument("rgb_height", default_value="720"),
            DeclareLaunchArgument("camera_fps", default_value="30"),
            DeclareLaunchArgument("gpu_device", default_value="0"),
            DeclareLaunchArgument("rviz_buffer_count", default_value="2"),
            DeclareLaunchArgument(
                "allow_fps_resolution_fallback", default_value="false"
            ),
            DeclareLaunchArgument("strict_camera_mode", default_value="true"),
            DeclareLaunchArgument("respawn_node", default_value="false"),
            DeclareLaunchArgument(
                "enable_rviz", default_value="true",
                description="Matikan bila node ini digabung ke launch yang sudah memiliki RViz.",
            ),
            SetEnvironmentVariable("CUDA_MODULE_LOADING", "LAZY"),
            SetEnvironmentVariable("QT_OPENGL", "desktop"),
            SetEnvironmentVariable("OGRE_RTT_MODE", "FBO"),
            SetEnvironmentVariable("__GL_SYNC_TO_VBLANK", "0"),
            SetEnvironmentVariable("vblank_mode", "0"),
            yolop_node,
            rviz_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rviz_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="RViz ditutup"))],
                )
            ),
        ]
    )
