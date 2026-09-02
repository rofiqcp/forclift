"""Standalone/embedded ESC launch. ESC package owns mux + keyboard + joystick."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import shutil
import time
from pathlib import Path
import yaml



def _runtime_dir():
    ws=os.environ.get("AGV_WS","/home/otomasi2/ros")
    root=os.environ.get("AGV_RUNTIME_CONFIG_ROOT",os.path.join(ws,"config","runtime"))
    target=os.path.join(os.path.expanduser(root),"esc")
    os.makedirs(target,exist_ok=True)
    source=os.path.join(get_package_share_directory("esc"),"config")
    for name in ("ackermann_1_board.yaml","ackermann_2_board.yaml","differential_1_board.yaml","esc_mux.yaml","keyboard_teleop.yaml","winch.yaml"):
        src=os.path.join(source,name); dst=os.path.join(target,name)
        if not os.path.exists(dst) and os.path.isfile(src): shutil.copy2(src,dst)
    return target



def _ensure_manual_safety(runtime_dir):
    """Migrate persistent ESC runtime YAML to fail-closed manual commissioning defaults."""
    backup_dir=os.path.join(runtime_dir,".stage5_manual_backups")
    os.makedirs(backup_dir,exist_ok=True)
    for name in ("esc_mux.yaml","keyboard_teleop.yaml"):
        path=os.path.join(runtime_dir,name)
        if not os.path.isfile(path):
            continue
        data=yaml.safe_load(open(path,"r",encoding="utf-8")) or {}
        before=yaml.safe_dump(data,sort_keys=False)
        if name=="esc_mux.yaml":
            params=data.setdefault("esc_command_mux",{}).setdefault("ros__parameters",{})
            params["manual_gate_topic"]="/system/manual_motion_allowed"
            params["require_manual_gate"]=True
            params["manual_gate_timeout_sec"]=min(1.0,max(0.10,float(params.get("manual_gate_timeout_sec",0.60))))
            params["manual_max_forward_speed_mps"]=min(0.40,max(0.05,float(params.get("manual_max_forward_speed_mps",0.30))))
            params["manual_max_reverse_speed_mps"]=min(0.25,max(0.05,float(params.get("manual_max_reverse_speed_mps",0.15))))
            params["manual_max_yaw_rate_rps"]=min(0.50,max(0.05,float(params.get("manual_max_yaw_rate_rps",0.35))))
        else:
            params=data.setdefault("esc_keyboard_teleop",{}).setdefault("ros__parameters",{})
            params["require_deadman"]=True
            params["enable_tty_fallback"]=False
        after=yaml.safe_dump(data,sort_keys=False)
        if after!=before:
            shutil.copy2(path,os.path.join(backup_dir,f"{Path(name).stem}_{time.time_ns()}{Path(name).suffix}"))
            tmp=path+".tmp"
            with open(tmp,"w",encoding="utf-8") as f:
                yaml.safe_dump(data,f,sort_keys=False); f.flush(); os.fsync(f.fileno())
            os.replace(tmp,path)
            yaml.safe_load(open(path,"r",encoding="utf-8"))

def _geometry_overrides():
    """Read canonical navigation runtime geometry without creating a package cycle.

    Standalone ESC launch remains usable even if navigation is not running.  If
    canonical geometry exists, it overrides duplicated geometry-only YAML fields.
    """
    ws=os.environ.get("AGV_WS","/home/otomasi2/ros")
    root=os.environ.get("AGV_RUNTIME_CONFIG_ROOT",os.path.join(ws,"config","runtime"))
    path=os.path.join(os.path.expanduser(root),"navigation","vehicle_geometry.yaml")
    if not os.path.isfile(path):
        return {}, {}, {}
    try:
        data=yaml.safe_load(open(path,"r",encoding="utf-8")) or {}
        v=data.get("vehicle",{})
        driver={
            "wheelbase": float(v["wheelbase_m"]),
            "wheel_radius": float(v["wheel_radius_m"]),
            "max_steering_rad": float(v["max_steering_angle_rad"]),
        }
        mux={
            "wheelbase_m": float(v["wheelbase_m"]),
            "max_forward_speed_mps": float(v["max_forward_speed_mps"]),
            "max_reverse_speed_mps": float(v["max_reverse_speed_mps"]),
            "max_steering_angle_rad": float(v["max_steering_angle_rad"]),
            "max_yaw_rate_rps": float(v["max_yaw_rate_rps"]),
        }
        keyboard={
            "wheelbase_m": float(v["wheelbase_m"]),
            "max_forward_speed_mps": float(v["max_forward_speed_mps"]),
            "max_reverse_speed_mps": float(v["max_reverse_speed_mps"]),
            "max_steering_angle_rad": float(v["max_steering_angle_rad"]),
        }
        return driver,mux,keyboard
    except Exception as exc:
        print(f"[ESC-GEOMETRY] canonical geometry ignored because it is invalid: {exc}")
        return {}, {}, {}

def generate_launch_description():
    profile = LaunchConfiguration("profile")
    runtime_dir = _runtime_dir()
    _ensure_manual_safety(runtime_dir)
    driver_geometry, mux_geometry, keyboard_geometry = _geometry_overrides()
    driver_params = PathJoinSubstitution([runtime_dir, profile])
    mux_params = os.path.join(runtime_dir, "esc_mux.yaml")
    keyboard_params = os.path.join(runtime_dir, "keyboard_teleop.yaml")
    winch_params = os.path.join(runtime_dir, "winch.yaml")

    args = [
        DeclareLaunchArgument("profile", default_value="ackermann_1_board.yaml"),
        DeclareLaunchArgument("board0_port", default_value="/dev/esc"),
        DeclareLaunchArgument("board1_port", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("start_mux", default_value="true"),
        DeclareLaunchArgument("enable_keyboard", default_value="true"),
        DeclareLaunchArgument("enable_joystick", default_value="true"),
        DeclareLaunchArgument("enable_nav2", default_value="false"),
        DeclareLaunchArgument("standalone_mode", default_value="true"),
        DeclareLaunchArgument("require_autonomy_gate", default_value="false"),
        DeclareLaunchArgument("require_manual_gate", default_value="false"),
        DeclareLaunchArgument("nav2_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("actuator_topic", default_value="/cmd_vel/actuator"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("offline_zero_output", default_value="true"),
        DeclareLaunchArgument("enable_winch", default_value="true"),
        DeclareLaunchArgument("winch_port", default_value="auto"),
    ]

    driver = Node(
        package="esc", executable="esc_driver", name="esc_driver", output="screen",
        condition=IfCondition(LaunchConfiguration("start_driver")),
        parameters=[driver_params, driver_geometry, {
            "board0_port": LaunchConfiguration("board0_port"),
            "board1_port": LaunchConfiguration("board1_port"),
            "cmd_vel_topic": LaunchConfiguration("actuator_topic"),
            "publish_joint_states": True,
            "offline_zero_output": ParameterValue(LaunchConfiguration("offline_zero_output"), value_type=bool),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[("joint_states", LaunchConfiguration("joint_states_topic"))],
    )
    mux = Node(
        package="esc", executable="esc_command_mux", name="esc_command_mux", output="screen",
        condition=IfCondition(LaunchConfiguration("start_mux")),
        parameters=[mux_params, mux_geometry, {
            "nav2_topic": LaunchConfiguration("nav2_topic"),
            "output_topic": LaunchConfiguration("actuator_topic"),
            "standalone_mode": ParameterValue(LaunchConfiguration("standalone_mode"), value_type=bool),
            "enable_nav2": ParameterValue(LaunchConfiguration("enable_nav2"), value_type=bool),
            "enable_keyboard": ParameterValue(LaunchConfiguration("enable_keyboard"), value_type=bool),
            "enable_joystick": ParameterValue(LaunchConfiguration("enable_joystick"), value_type=bool),
            "require_autonomy_gate": ParameterValue(LaunchConfiguration("require_autonomy_gate"), value_type=bool),
            "require_manual_gate": ParameterValue(LaunchConfiguration("require_manual_gate"), value_type=bool),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    keyboard = Node(
        package="esc", executable="esc_keyboard_teleop", name="esc_keyboard_teleop", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_keyboard")),
        parameters=[keyboard_params, keyboard_geometry, {
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    winch = Node(
        package="esc", executable="winch_serial_node", name="winch_serial_node", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_winch")),
        parameters=[winch_params, {"port": LaunchConfiguration("winch_port")}],
    )
    joy = Node(
        package="joy", executable="joy_node", name="esc_joy_node", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_joystick")),
        parameters=[{"deadzone": 0.08, "autorepeat_rate": 20.0, "coalesce_interval_ms": 1}],
    )

    return LaunchDescription(args + [driver, mux, keyboard, winch, joy])
