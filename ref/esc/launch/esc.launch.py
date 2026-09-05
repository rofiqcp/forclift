"""ESC runtime: exactly two nodes, motor_teleop + esc_ackermann."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _active_config_dir() -> str:
    env = os.environ.get("AGV_ESC_CONFIG_DIR", "").strip()
    if env and os.path.isdir(os.path.expanduser(env)):
        return os.path.abspath(os.path.expanduser(env))
    share = Path(get_package_share_directory("esc")).resolve()
    parts = list(share.parts)
    if "install" in parts:
        idx = parts.index("install")
        workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
        candidate = workspace / "src" / "esc" / "config"
        if candidate.is_dir():
            return str(candidate.resolve())
    return str(share / "config")


def generate_launch_description():
    config_dir = _active_config_dir()
    teleop_params = os.path.join(config_dir, "teleop.yaml")
    ackermann_params = os.path.join(config_dir, "ackermann.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_teleop", default_value="true"),
        DeclareLaunchArgument("start_ackermann", default_value="true"),
        DeclareLaunchArgument("serial_device", default_value="auto"),
        DeclareLaunchArgument("serial_enabled", default_value="true"),
        DeclareLaunchArgument("nav2_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("teleop_topic", default_value="/cmd_vel/teleop"),
        DeclareLaunchArgument("active_source_topic", default_value="/esc/mux/active_source"),
        DeclareLaunchArgument("require_autonomy_gate", default_value="true"),
    ]

    teleop = Node(
        package="esc",
        executable="motor_teleop",
        name="motor_teleop",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_teleop")),
        parameters=[
            teleop_params,
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
    )

    ackermann = Node(
        package="esc",
        executable="ackermann_controller_server",
        name="esc_ackermann",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_ackermann")),
        # Runtime-routing keys intentionally live only in this launch override
        # block (not in node-scoped ackermann.yaml), so deployment/commissioning
        # values cannot be shadowed by a more-specific YAML node scope.
        parameters=[
            {
                "serial_device": LaunchConfiguration("serial_device"),
                "serial_enabled": ParameterValue(LaunchConfiguration("serial_enabled"), value_type=bool),
                "nav2_topic": LaunchConfiguration("nav2_topic"),
                "teleop_topic": LaunchConfiguration("teleop_topic"),
                "active_source_topic": LaunchConfiguration("active_source_topic"),
                "require_autonomy_gate": ParameterValue(
                    LaunchConfiguration("require_autonomy_gate"), value_type=bool),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
            teleop_params,  # shared speed/yaw/steering/RPM limits
            ackermann_params,
        ],
    )

    return LaunchDescription(args + [teleop, ackermann])
