#!/usr/bin/env python3
"""Runtime mapping stack started/stopped by mapping_gui_cpp.

Master-aligned LiDAR mapping + IMU-driven RobotModel visualization.

Mapping path (LiDAR scan mapping with fused local odometry):
  /scan_nav -> hector_slam_node -> /lidar/odom (diagnostic measurement only, no TF)
  /esc/odom + /imu/data -> EKF -> odom->base_footprint
  /scan_nav + odom->base_footprint -> async slam_toolbox -> /map + map->odom

RobotModel visual path (isolated from mapping TF):
  /lidar/odom remains diagnostic/visual comparison only; never fused into EKF
  Yaw  <- /imu/data while IMU rotates; stationary AHRS jitter is held
  /esc/odom is used only as a translation stationary gate for the visual tree.
  imu_visual_tf_node publishes odom -> visual_/base_footprint.

Important:
  * LiDAR remains the SLAM scan source; production local odometry is ESC/wheels + IMU only.
  * RobotModel yaw no longer comes from LiDAR, visual EKF, wheel odometry, or a yaw guard.
  * IMU acceleration is never double-integrated into X/Y, avoiding inertial-position drift.
  * The visual TF tree is prefixed with visual_/, exactly like the reference master.
"""

import os
import shutil
import tempfile

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import (RegisterEventHandler, LogInfo, EmitEvent, ExecuteProcess,
                            DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from navigation_runtime.vehicle_geometry import sync_derived_configs
from navigation_runtime.lidar_safety_config import ensure_stage2_lidar_runtime
from navigation_runtime.localization_config import ensure_stage5_localization_runtime
from navigation_runtime.runtime_schema import ensure_runtime_schema



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



def _success_only_exit(target_action, success_actions, stage_name: str, shutdown_on_failure: bool = True):
    """Never treat a non-zero/crashed readiness process as success."""
    def _handle(event, _context):
        rc = getattr(event, "returncode", None)
        if rc == 0:
            return list(success_actions)
        reason = f"MAPPING gate {stage_name} failed with exit code {rc}"
        actions = [LogInfo(msg="[MAPPING-GATE] ERROR: " + reason)]
        if shutdown_on_failure:
            actions.append(EmitEvent(event=Shutdown(reason=reason + "; fail-closed shutdown")))
        return actions
    return RegisterEventHandler(OnProcessExit(target_action=target_action, on_exit=_handle))

def generate_launch_description():

    # ── ANTI-STACK GUARD (minimal, launch-only) ─────────────────────────────
    # Best-effort cleanup of stale mapping nodes from a previous session.
    # This fires every time map.launch starts (even from the wrapper script).
    #
    # IMPORTANT DESIGN DECISIONS:
    #   1. ROS nodes (hector_slam, slam_toolbox, lidar_node, imu_node, etc.)
    #      are ALWAYS killed — they belong to this launch's session.
    #   2. RViz is NOT killed here. The wrapper script (agv_mapping_control.sh)
    #      is the start/stop entry point and manages the RViz lifecycle there.
    #      Killing RViz from within the launch would kill ALL RViz instances,
    #      including any the user opened manually outside this session.
    #   3. `pkill -f map.launch` matches the Python script path (which works),
    #      NOT the process argv[0] name. This kills the ros2 launch parent.
    #   4. A stale `/tmp/agv_mapping.pid` file from a crash is also cleaned.
    #   5. CP210x ttyUSB force-close guards against kernel-level port hang.
    #
    # For the full graceful stop flow (SIGINT -> SIGTERM -> SIGKILL with PGID),
    # use:  ./agv_mapping_control.sh stop
    # ─────────────────────────────────────────────────────────────────────────
    kill_stale = ExecuteProcess(
        cmd=['bash', '-c',
             # Detect stale by unique node name rviz2_mapping (unique to map.launch).
             'if ros2 node list 2>/dev/null | grep -q "rviz2_mapping"; then '
             '  echo "[map.launch] STALE SESSION DETECTED — cleaning mapping nodes"; '
             '  ros2 node list 2>/dev/null | grep -q "rviz2_mapping" && '
             '  pkill -f "map.launch.py" 2>/dev/null || true; '
             '  pkill -f "hector_slam_node" 2>/dev/null || true; '
             '  pkill -f "async_slam_toolbox_node" 2>/dev/null || true; '
             '  pkill -f "lidar_node" 2>/dev/null || true; '
             '  pkill -f "imu_node" 2>/dev/null || true; '
             '  pkill -f "imu_visual_tf_node" 2>/dev/null || true; '
             '  pkill -f "robot_state_publisher" 2>/dev/null || true; '
             '  pkill -f "joint_state_publisher" 2>/dev/null || true; '
             '  pkill -f "allsystem_gate" 2>/dev/null || true; '
             '  pkill -f "rviz2_mapping" 2>/dev/null || true; '
             '  pkill -f "rviz2" 2>/dev/null || true; '  # only rviz2_mapping reaches here
             '  sleep 2; '
             '  fuser -k -9 /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || true; '
             '  rm -f /tmp/agv_mapping.pid; '
             '  echo "[map.launch] Stale session cleared"; '
             'else '
             '  echo "[map.launch] No stale session — starting fresh"; '
             'fi'],
        output='screen',
        name='kill_stale_launch',
    )

    nav_share = get_package_share_directory('navigation')
    ensure_stage2_lidar_runtime(nav_share)
    ensure_stage5_localization_runtime(nav_share)
    esc_share = get_package_share_directory('esc')
    geometry_state = sync_derived_configs(nav_share, esc_share)
    try:
        yolo_share = get_package_share_directory('yolo_obstacle_detection_ros2')
    except Exception:
        yolo_share = None
    runtime_manifest = ensure_runtime_schema(nav_share, esc_share, yolo_share)
    if not runtime_manifest.get("valid", False):
        raise RuntimeError("runtime configuration manifest is incomplete: " + str(runtime_manifest.get("missing_files", [])))
    g = geometry_state["geometry"]
    geometry_log = LogInfo(msg=(
        f"[MAPPING-GEOMETRY] REP-103 candidate adapter yaw={g['cad_to_base_yaw_rad']:.6f} rad; "
        f"wheelbase={g['wheelbase_m']:.4f} m; validated={geometry_state['validated']}. "
        "Mapping/RViz is allowed for physical frame verification even when validation is incomplete."
    ))

    # V17: mapping and autonomous use the same deterministic serial preflight.
    # It clears stale hardware owners and, when the one-time recovery helper is
    # installed, performs a software CP210x replug before any new sensor node.
    preflight_script = os.path.join(
        get_package_prefix('navigation'), 'lib', 'navigation',
        'autonomous_sensor_preflight.sh')
    sensor_preflight = ExecuteProcess(
        cmd=['bash', preflight_script],
        output='screen',
        # Mapping reopens the same LiDAR transport immediately after preflight.
        # Give lidar_node enough time to send A5 65 and power the motor down
        # cleanly. Other callers keep the preflight's original 1.0 s default.
        additional_env={'AGV_SENSOR_TERM_GRACE_SEC': '2.5'},
    )

    robot_description = xacro.process_file(
        os.path.join(nav_share, 'urdf', 'agv.urdf.xacro'),
        mappings={
            'cad_to_base_yaw_rad': str(g['cad_to_base_yaw_rad']),
            'max_steering_angle_rad': str(g['max_steering_angle_rad']),
        }).toxml()

    joint_state_urdf = os.path.join(
        tempfile.gettempdir(), 'navigation_agv_joint_states.urdf')
    with open(joint_state_urdf, 'w', encoding='utf-8') as urdf_file:
        urdf_file.write(robot_description)

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/tmp/agv_devices/lidar'),
        DeclareLaunchArgument('lidar_baudrate', default_value='230400'),
        DeclareLaunchArgument('lidar_intensity_mode', default_value='true'),
        DeclareLaunchArgument('lidar_intensity_bits', default_value='16'),
        DeclareLaunchArgument('lidar_strict_checksum', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_joint_state_publisher', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_imu_visual', default_value='true'),
        DeclareLaunchArgument('imu_port', default_value='/tmp/agv_devices/imu'),
        DeclareLaunchArgument('imu_baudrate', default_value='115200'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('enable_esc', default_value='true'),
        DeclareLaunchArgument('esc_port', default_value='/dev/esc'),
        DeclareLaunchArgument('enable_keyboard', default_value='true'),
        DeclareLaunchArgument('enable_joystick', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav_share, 'rviz', 'mapping.rviz')),
        DeclareLaunchArgument('enable_camera', default_value='false',
            description='Start the internal V4L2 Astra RGB camera driver'),
        DeclareLaunchArgument('enable_yolo', default_value='false',
            description='Start YOLO obstacle detector (camera must be enabled)'),
        DeclareLaunchArgument('camera_device', default_value='/tmp/agv_devices/camera'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
    ]

    # Real mapping geometry: base_footprint -> base_link -> lidar_link / imu_link / etc.
    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'publish_frequency': 30.0,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    joint_state = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=[joint_state_urdf],
        condition=IfCondition(LaunchConfiguration('enable_joint_state_publisher')),
        parameters=[{
            'source_list': ['/esc/joint_states'],
            'rate': 20,
            'publish_default_positions': True,
            'publish_default_velocities': False,
            'publish_default_efforts': False,
            'use_mimic_tags': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    # Isolated visual tree. This is the same design as the supplied master:
    # RViz RobotModel resolves visual_/base_* while mapping keeps base_* unchanged.
    visual_robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='imu_visual_robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_imu_visual')),
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'frame_prefix': 'visual_/',
            'publish_frequency': 50.0,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[('robot_description', 'robot_description_imu_visual')])

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'imu.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_imu')),
        launch_arguments={
            'port': LaunchConfiguration('imu_port'),
            'baudrate': LaunchConfiguration('imu_baudrate'),
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'publish_raw': 'false',
            'use_sim_time': use_sim_time,
        }.items())

    # Mapping TF is untouched.  V38 deliberately uses a wider stationary band
    # than the sensor noise observed in log(6): quiet IMU/ESC/LiDAR motion holds
    # the visual model exactly, while real chassis motion still exits hysteresis.
    imu_visual_tf = Node(
        package='navigation',
        executable='imu_visual_tf_node',
        name='imu_visual_tf_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_imu_visual')),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'imu_stationary_enter_rad_s': 0.020,
            'imu_stationary_exit_rad_s': 0.060,
            'stationary_confirm_s': 0.20,
            'esc_stationary_linear_mps': 0.030,
            'esc_stationary_angular_rad_s': 0.050,
            'esc_timeout_s': 0.60,
            'lidar_stationary_linear_mps': 0.025,
            'lidar_stationary_angular_rad_s': 0.050,
            'position_filter_alpha': 0.20,
            'yaw_filter_alpha': 0.25,
            'visual_position_deadband_m': 0.0080,
            'visual_yaw_deadband_rad': 0.0040,
        }])

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'lidar.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('lidar_port'),
            'baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'lidar_link',
            'intensity_mode': LaunchConfiguration('lidar_intensity_mode'),
            'intensity_bits': LaunchConfiguration('lidar_intensity_bits'),
            'strict_checksum': LaunchConfiguration('lidar_strict_checksum'),
            'use_sim_time': use_sim_time,
        }.items())

    # PART-1: Hector is scan-synchronous LiDAR odometry only. It publishes
    # /lidar/odom with measurement timestamps but never owns navigation TF.
    lidar_odom = Node(
        package='navigation',
        executable='hector_slam_node',
        name='hector_slam_node',
        output='screen',
        parameters=[
            _runtime_config(nav_share, 'navigation', 'hector.yaml'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[('/odom', '/lidar/odom')])

    # Asynchronous SLAM Toolbox keeps scan ingestion responsive. PART-1 uses a
    # 0.5 s occupancy-grid publish interval and non-zero motion thresholds to
    # reduce duplicate zero-motion work without changing LiDAR scan ingestion.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'slam_async.launch.py')),
        launch_arguments={
            'slam_params_file': _runtime_config(nav_share, 'navigation', 'slam_toolbox.yaml'),
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'use_sim_time': use_sim_time,
        }.items())

    map_monitor = Node(
        package='navigation',
        executable='map_monitor_node',
        name='map_monitor_node',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'min_known_cells': 100,
            'min_occupied_cells': 5,
            'stats_min_interval': 0.5,
            'occupied_drop_warn_ratio': 0.08,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    # PART-1 production local EKF. It is the sole odom->base_footprint owner
    # and fuses only wheel velocity + IMU gyro. LiDAR odometry is diagnostic-only.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            _runtime_config(nav_share, 'navigation', 'ekf_mapping_reference.yaml'),
            {
                'frequency': 20.0,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
        remappings=[('odometry/filtered', '/odometry/filtered')])

    localization_timing_monitor = Node(
        package='navigation',
        executable='localization_timing_monitor.py',
        name='localization_timing_monitor',
        output='screen',
        parameters=[
            _runtime_config(nav_share, 'navigation', 'localization_timing.yaml'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ])

    manual_motion_health = Node(
        package='navigation',
        executable='manual_motion_health.py',
        name='manual_motion_health',
        output='screen',
        parameters=[
            _runtime_config(nav_share, 'navigation', 'manual_motion_health.yaml'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ])

    esc_share = get_package_share_directory('esc')
    esc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(esc_share, 'launch', 'esc.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_esc')),
        launch_arguments={
            'profile': 'ackermann_1_board.yaml',
            'board0_port': LaunchConfiguration('esc_port'),
            'use_sim_time': use_sim_time,
            'enable_keyboard': LaunchConfiguration('enable_keyboard'),
            'enable_joystick': LaunchConfiguration('enable_joystick'),
            'enable_nav2': 'false',
            'standalone_mode': 'false',
            'require_autonomy_gate': 'false',
            'require_manual_gate': 'true',
            'joint_states_topic': '/esc/joint_states',
            'offline_zero_output': 'true',
        }.items())

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    # USB ROLE RESOLVER (single source of truth for the whole hub)
    # ------------------------------------------------------------------
    # No sensor is allowed to auto-probe raw ttyUSBX/videoX paths. The resolver
    # waits until udev aliases are stable, then creates nested aliases under
    # /tmp/agv_devices. These nested aliases point to by-id/by-path paths, not
    # to raw kernel indices, so ttyUSB0<->ttyUSB1 and video0<->video2 swaps are
    # harmless after hub re-enumeration.

    # ISSUE C FIX: Delete stale aliases before resolver runs to prevent stale
    # symlinks from a previous run causing sensors to open wrong device nodes.
    cleanup_stale_aliases = ExecuteProcess(
        cmd=['bash', '-c',
             "rm -f /tmp/agv_devices/imu /tmp/agv_devices/lidar /tmp/agv_devices/camera "
             "&& echo 'Stale aliases cleaned'"],
        output='screen',
    )

    # Resolve both serial roles in ONE process.  Two concurrent resolver
    # processes used to probe/update the same /tmp/agv_devices directory at the
    # same time; after a USB hub re-enumeration that could leave both aliases
    # absent and neither driver would ever start.  One inventory + one protocol
    # pass gives an atomic IMU/LiDAR handoff.
    role_resolver_serial = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', os.path.join(nav_share, 'tools', 'resolve_usb_roles.py'),
            '--output-dir', '/tmp/agv_devices',
            '--require-imu', LaunchConfiguration('enable_imu'),
            '--require-lidar', 'true', '--require-camera', 'false', '--stable-seconds', '0.25',
        ], output='screen')

    ready_gate_script = os.path.join(
        get_package_prefix('navigation'), 'lib', 'navigation', 'serial_transport_ready_gate.py')
    imu_transport_ready = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', ready_gate_script,
            '--imu', LaunchConfiguration('imu_port'), '--lidar', LaunchConfiguration('lidar_port'),
            '--require-imu', LaunchConfiguration('enable_imu'), '--require-lidar', 'false',
            '--stable-cycles', '1', '--poll-sec', '0.10', '--timeout-sec', '0',
        ], output='screen')
    lidar_transport_ready = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', ready_gate_script,
            '--imu', LaunchConfiguration('imu_port'), '--lidar', LaunchConfiguration('lidar_port'),
            '--require-imu', 'false', '--require-lidar', 'true',
            '--stable-cycles', '1', '--poll-sec', '0.10', '--timeout-sec', '0',
        ], output='screen')

    # Camera resolver: runs independently after serial sensors are resolved.
    role_resolver_camera = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', os.path.join(nav_share, 'tools', 'resolve_usb_roles.py'),
            '--output-dir', '/tmp/agv_devices',
            '--require-imu', 'false',
            '--require-lidar', 'false',
            '--require-camera', LaunchConfiguration('enable_camera'),
            '--stable-seconds', '2.0',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    # Drivers are handed off independently. Mapping consumers are started only
    # after the combined topic gate proves both real streams are publishing.
    delayed_imu = TimerAction(period=0.10, actions=[imu])
    delayed_lidar = TimerAction(period=0.10, actions=[lidar])
    delayed_lidar_odom = TimerAction(period=0.05, actions=[lidar_odom])
    delayed_imu_visual_tf = TimerAction(period=0.10, actions=[imu_visual_tf])
    delayed_slam = TimerAction(period=0.10, actions=[slam])
    delayed_monitor = TimerAction(period=0.15, actions=[map_monitor])
    delayed_rviz = TimerAction(period=0.20, actions=[rviz])

    # Do not start V4L2 until both serial sensors have actually published data.
    sensor_gate = Node(
        package='navigation', executable='allsystem_gate',
        name='map_usb_serial_ready_gate', output='screen', emulate_tty=True,
        parameters=[{
            'gate_name': 'map_serial',
            'topics': ['/imu/data', '/scan_nav'],
            'min_messages': 1,
            'timeout_ms': 8000,
            'exit_on_ready': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    # Deterministic serial handoff: resolver must finish creating stable aliases
    # before either transport gate is allowed to inspect them. This matches the
    # GUI sensor runtime that already publishes LiDAR reliably.
    start_sensor_gate = TimerAction(period=0.80, actions=[sensor_gate])
    start_transport_after_resolver = _success_only_exit(
        role_resolver_serial,
        [imu_transport_ready, lidar_transport_ready, start_sensor_gate],
        'serial-role-resolver')
    start_imu_after_transport_ready = _success_only_exit(
        imu_transport_ready, [delayed_imu], 'imu-transport-ready')
    start_lidar_after_transport_ready = _success_only_exit(
        lidar_transport_ready, [delayed_lidar], 'lidar-transport-ready')
    start_mapping_consumers_after_sensor_gate = _success_only_exit(
        sensor_gate, [delayed_lidar_odom, delayed_imu_visual_tf, delayed_slam,
                      delayed_monitor, delayed_rviz], 'map-serial-topic-gate')

    # Camera discovery is a separate root and never waits for serial sensors.
    start_camera_resolver = TimerAction(period=0.20, actions=[role_resolver_camera])

    camera_share = get_package_share_directory('yolo_obstacle_detection_ros2')
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share, 'launch', 'perception_all.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        launch_arguments={
            'enable_camera': 'true',
            'enable_yolo': LaunchConfiguration('enable_yolo'),
            'enable_hole_alignment': 'false',
            'use_composition': 'false',
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'use_sim_time': use_sim_time,
        }.items())

    camera_gate = Node(
        package='navigation', executable='allsystem_gate',
        name='map_camera_ready_gate', output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        parameters=[{
            'gate_name': 'map_camera',
            'topics': ['/camera/color/image_raw'],
            'min_messages': 3,
            'timeout_ms': 60000,
            'exit_on_ready': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    # Camera + YOLO are loaded into one component_container_mt by
    # perception_all.launch.py.  The camera readiness gate remains diagnostic.

    # ISSUE B FIX: Camera starts only after role_resolver_camera succeeds.
    # This decouples camera from IMU/LiDAR — camera issues no longer block
    # or destabilize the serial sensor startup sequence.
    start_camera_after_camera_resolver = _success_only_exit(
        role_resolver_camera, [camera_node, camera_gate], 'camera-role-resolver', shutdown_on_failure=False)

    # Stage the complete mapping foundation after preflight.  This prevents the
    # preflight's stale-owner cleanup from racing newly-created RSP/EKF processes.
    start_foundation_after_preflight = _success_only_exit(
        sensor_preflight,
        [robot_state, joint_state, visual_robot_state, ekf,
         localization_timing_monitor, manual_motion_health, esc,
         role_resolver_serial, start_camera_resolver],
        'sensor-preflight')

    return LaunchDescription(args + [
        geometry_log,
        # Register transitions before the sole direct Stage-0 process.
        start_foundation_after_preflight,
        start_transport_after_resolver,
        start_imu_after_transport_ready,
        start_lidar_after_transport_ready,
        start_mapping_consumers_after_sensor_gate,
        start_camera_after_camera_resolver,
        sensor_preflight,
    ])
