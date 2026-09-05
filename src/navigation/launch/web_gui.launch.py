#!/usr/bin/env python3
"""Single operator entrypoint: full autonomous AGV stack + browser HMI.

This launch intentionally does not create a second agv_web_gui node.  It wraps
``autonomous.launch.py`` and lets that master launch own sensors, localization,
Nav2, perception, mapping controls, map switching, and the one web server.
"""

import errno
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_SINGLETON_PID = "/tmp/navigation_web_gui_full_stack.pid"

def _claim_singleton():
    current = os.getpid()
    for _ in range(2):
        try:
            fd = os.open(_SINGLETON_PID, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            with os.fdopen(fd, "w") as handle:
                handle.write(str(current) + "\n")
            return
        except FileExistsError:
            try:
                with open(_SINGLETON_PID, "r", encoding="utf-8") as handle:
                    owner = int(handle.read().strip())
            except (OSError, ValueError):
                owner = -1
            if owner > 0 and owner != current:
                try:
                    os.kill(owner, 0)
                except ProcessLookupError:
                    owner = -1
                except PermissionError:
                    pass
                else:
                    raise RuntimeError(
                        f"web_gui full stack already running as PID {owner}; refusing duplicate sensors/filters/Nav2"
                    )
            try:
                os.unlink(_SINGLETON_PID)
            except FileNotFoundError:
                pass
    raise RuntimeError("unable to claim web_gui full-stack singleton PID file")

def generate_launch_description():
    _claim_singleton()
    nav_share = get_package_share_directory("navigation")
    autonomous = os.path.join(nav_share, "launch", "autonomous.launch.py")

    args = [
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="5005"),
        DeclareLaunchArgument("read_only", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_nav2", default_value="true"),
        # Web is the primary operator UI. RViz remains optional diagnostics and
        # is OFF by default to avoid its previous crash/respawn load on Jetson.
        DeclareLaunchArgument("enable_rviz", default_value="false"),
        DeclareLaunchArgument("map", default_value="auto"),
    ]

    full_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autonomous),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "enable_nav2": LaunchConfiguration("enable_nav2"),
            "enable_rviz": LaunchConfiguration("enable_rviz"),
            "map": LaunchConfiguration("map"),
            "start_web_gui": "true",
            "web_bind_address": LaunchConfiguration("bind_address"),
            "web_port": LaunchConfiguration("port"),
            "web_read_only": LaunchConfiguration("read_only"),
        }.items(),
    )
    return LaunchDescription(args + [full_stack])
