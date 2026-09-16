#!/usr/bin/env python3
"""V6 AMCL/QoS + deterministic costmap bringup.

Full autonomous AGV startup in LOCALIZATION-ONLY mode.

This launch is the single master entrypoint.  Normal operation stays
LOCALIZATION-ONLY against the saved occupancy map.  For BAB 4.2.1-4.2.3 the
localhost mapping controller temporarily pauses map_server/AMCL/Nav2 and starts
SLAM Toolbox only, reusing the already-running LiDAR/IMU/EKF.  map.launch.py and
mapping_runtime.launch.py are not included by autonomous.  EKF remains the sole
owner of odom -> base_footprint.

Startup goals for the real AGV:
  * Canonical /map + map TF activate PlannerServer/global inflation without
    waiting for the stricter motion-confidence threshold.
  * map:=auto loads the exact map committed by the latest successful map.launch.py STOP+SAVE via /home/otomasi2/ros/maps/latest_map.txt; timestamp search is fallback only.
  * One USB-role resolver owns IMU, LiDAR and Astra RGB discovery.
  * Camera starts independently and publishes RAW RGB internally.
  * Wheel/ESC odometry + IMU are the ONLY production EKF inputs; LiDAR odometry is diagnostic-only.
  * Stage 0 preflight removes stale mapping/Nav2/TF owners BEFORE any new child starts.
  * Stage 1 proves IMU, LiDAR safety, ESC odometry, EKF and required local TF first.
  * Only then may Map Server + AMCL lifecycle activate; planning-map global inflation is the first Nav2 milestone.
  * RViz displays only the YOLO-processed image, never the raw camera feed.

The launch argument ``map`` accepts:
  * auto / empty / the old placeholder /path/to/map.yaml -> latest committed map.launch.py result; timestamp search only if the pointer is missing/invalid
  * an explicit existing YAML path -> that map is used
  * a basename such as map_20260818_010000.yaml -> resolved across persistent/legacy/backup map directories
"""

import glob
import hashlib
import json
import math
import os
import shutil
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from datetime import datetime, timezone

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from navigation_runtime.vehicle_geometry import GeometryError, sync_derived_configs
from navigation_runtime.lidar_safety_config import ensure_stage2_lidar_runtime
from navigation_runtime.localization_config import ensure_stage5_localization_runtime
from navigation_runtime.planning_safety_config import ensure_stage5_planning_runtime
from navigation_runtime.runtime_schema import ensure_runtime_schema


WORKSPACE = "/home/otomasi2/ros"
PERSISTENT_MAP_DIR = "/home/otomasi2/ros/maps"
LEGACY_MAP_DIR = "/home/otomasi2/ros/src/navigation/maps"
DEFAULT_MAP_DIR = PERSISTENT_MAP_DIR
LATEST_MAP_POINTER = os.path.join(PERSISTENT_MAP_DIR, "latest_map.txt")
DEFAULT_MAP_YAML = "auto"
DEFAULT_YOLO_MODELS_DIR = "/home/otomasi2/ros/models"
DEFAULT_YOLO_MODEL = "auto"
OLD_PLACEHOLDER = "/path/to/map.yaml"
SERIAL_ROLE_STAGE = "serial-role-resolver"
AUTONOMOUS_SINGLETON_PID = "/tmp/navigation_autonomous_full_stack.pid"


def _claim_autonomous_singleton():
    """Refuse a second full autonomous stack, regardless of entrypoint."""
    current = os.getpid()
    for _ in range(2):
        try:
            fd = os.open(AUTONOMOUS_SINGLETON_PID, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            with os.fdopen(fd, "w") as handle:
                handle.write(str(current) + "\n")
            return
        except FileExistsError:
            try:
                owner = int(Path(AUTONOMOUS_SINGLETON_PID).read_text().strip())
            except (OSError, ValueError):
                owner = -1
            if owner > 0 and owner != current:
                try:
                    os.kill(owner, 0)
                except (ProcessLookupError, PermissionError):
                    owner = -1
                else:
                    raise RuntimeError(
                        f"autonomous full stack already running as PID {owner}; refusing duplicate ROS/Nav2/sensors"
                    )
            try:
                os.unlink(AUTONOMOUS_SINGLETON_PID)
            except FileNotFoundError:
                pass
    raise RuntimeError("unable to claim autonomous full-stack singleton PID file")


def _resolve_rviz_desktop_env():
    """Return a desktop environment for RViz, even from SSH/remote shells."""
    env = {}
    uid = os.getuid()

    display = os.environ.get("DISPLAY", "").strip()
    if not display:
        sockets = []
        for path in glob.glob("/tmp/.X11-unix/X*"):
            name = os.path.basename(path)
            suffix = name[1:]
            if suffix.isdigit():
                sockets.append(int(suffix))
        if sockets:
            # Prefer the local GDM/Xorg display (normally :0 or :1) over
            # high-numbered virtual displays such as NoMachine :1001.
            display = f":{min(sockets)}"
    if display:
        env["DISPLAY"] = display

    xauthority = os.environ.get("XAUTHORITY", "").strip()
    if not xauthority:
        candidate = f"/run/user/{uid}/gdm/Xauthority"
        if os.path.isfile(candidate):
            xauthority = candidate
    if xauthority:
        env["XAUTHORITY"] = xauthority

    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    if not runtime_dir:
        candidate = f"/run/user/{uid}"
        if os.path.isdir(candidate):
            runtime_dir = candidate
    if runtime_dir:
        env["XDG_RUNTIME_DIR"] = runtime_dir

    dbus = os.environ.get("DBUS_SESSION_BUS_ADDRESS", "").strip()
    if not dbus and runtime_dir and os.path.exists(os.path.join(runtime_dir, "bus")):
        dbus = f"unix:path={runtime_dir}/bus"
    if dbus:
        env["DBUS_SESSION_BUS_ADDRESS"] = dbus

    return env
SERIAL_TRANSPORT_STAGE = "serial-transport-ready"


def _success_only_exit(target_action, success_actions, stage_name: str):
    """Run downstream actions only for exit code 0; otherwise stop autonomous bringup.

    ROS 2 launch OnProcessExit fires for every process exit. Safety-critical
    bringup must never interpret a crash/non-zero return as readiness.
    """
    def _handle(event, _context):
        rc = getattr(event, "returncode", None)
        if rc == 0:
            return list(success_actions)
        # A readiness gate can legitimately return non-zero while hardware is
        # reconnecting or localization is not ready. Keep the ROS graph alive
        # and motion fail-closed instead of terminating gui/map/autonomous.
        # This makes startup observable and restartable without converting an
        # expected HOLD state into a launch error.
        reason = f"AUTONOMOUS gate {stage_name} returned {rc}; downstream stage held fail-closed"
        return [LogInfo(msg="[AUTONOMOUS-GATE] HOLD: " + reason)]
    return RegisterEventHandler(
        OnProcessExit(target_action=target_action, on_exit=_handle)
    )


def _resolve_map_yaml(context):
    """Resolve the newest valid saved map without hard-coded map preference.

    Mapping maps are persistent data, not source code.  V21 searches the
    persistent workspace map directory first, then the current/legacy source
    tree and any src_backup* trees left by previous clean installs.  A
    map_YYYYMMDD_HHMMSS filename is ranked by its embedded timestamp so ZIP
    extraction mtimes can never make an older map appear newer.
    """
    requested = LaunchConfiguration("map").perform(context).strip()
    configured_dir = LaunchConfiguration("maps_dir").perform(context).strip() or DEFAULT_MAP_DIR
    configured_dir = os.path.abspath(os.path.expanduser(configured_dir))

    def candidate_dirs():
        dirs = [
            configured_dir,
            PERSISTENT_MAP_DIR,
            LEGACY_MAP_DIR,
        ]
        dirs.extend(sorted(glob.glob(os.path.join(WORKSPACE, "src_backup*", "navigation", "maps"))))
        dirs.extend(sorted(glob.glob(os.path.join(WORKSPACE, "src_*", "navigation", "maps"))))
        # Installed package is last-resort only; persistent/source maps outrank it.
        dirs.append(os.path.join(WORKSPACE, "install", "navigation", "share", "navigation", "maps"))
        out = []
        seen = set()
        for item in dirs:
            item = os.path.abspath(os.path.expanduser(item))
            if item not in seen and os.path.isdir(item):
                seen.add(item)
                out.append(item)
        return out

    search_dirs = candidate_dirs()

    def validate_pair(path):
        if not os.path.isfile(path) or not path.lower().endswith((".yaml", ".yml")):
            return None
        try:
            text = open(path, "r", encoding="utf-8").read()
        except OSError:
            return None
        image_match = re.search(r"^\s*image\s*:\s*(.+?)\s*$", text, flags=re.MULTILINE)
        resolution_match = re.search(r"^\s*resolution\s*:\s*([0-9eE+\-.]+)", text, flags=re.MULTILINE)
        if not image_match or not resolution_match:
            return None
        try:
            if float(resolution_match.group(1)) <= 0.0:
                return None
        except ValueError:
            return None
        image_value = image_match.group(1).strip().strip("\"'")
        image_path = image_value if os.path.isabs(image_value) else os.path.join(os.path.dirname(path), image_value)
        image_path = os.path.abspath(os.path.expanduser(image_path))
        return image_path if os.path.isfile(image_path) else None

    def map_epoch(path):
        name = os.path.basename(path)
        match = re.search(r"map_(\d{8})_(\d{6})", name)
        if match:
            try:
                dt = datetime.strptime(match.group(1) + match.group(2), "%Y%m%d%H%M%S")
                return dt.replace(tzinfo=timezone.utc).timestamp()
            except ValueError:
                pass
        try:
            return os.path.getmtime(path)
        except OSError:
            return 0.0

    def map_rank(path):
        parent = os.path.abspath(os.path.dirname(path))
        persistent_bonus = 2 if parent == os.path.abspath(PERSISTENT_MAP_DIR) else 0
        configured_bonus = 1 if parent == configured_dir else 0
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            mtime = 0.0
        return (map_epoch(path), persistent_bonus, configured_bonus, mtime)

    def all_valid_maps():
        found = []
        for directory in search_dirs:
            for path in glob.glob(os.path.join(directory, "*.yaml")) + glob.glob(os.path.join(directory, "*.yml")):
                if validate_pair(path):
                    found.append(os.path.abspath(path))
        return sorted(set(found), key=map_rank, reverse=True)

    def committed_map():
        # Primary contract with mapping_gui_cpp.  This pointer is written with
        # os.replace() only AFTER a complete YAML+image pair is stable, so an
        # interrupted save can never replace the previously-good autonomous map.
        try:
            value = open(LATEST_MAP_POINTER, "r", encoding="utf-8").read().strip()
        except OSError:
            return None
        if not value:
            return None
        candidate = os.path.abspath(os.path.expanduser(value))
        # Mapping GUI may commit either a persistent runtime map or one of the
        # three SLAM experiment slots under src/navigation/maps.
        parent = os.path.abspath(os.path.dirname(candidate))
        allowed = {os.path.abspath(PERSISTENT_MAP_DIR), os.path.abspath(LEGACY_MAP_DIR)}
        if parent not in allowed:
            return None
        return candidate if validate_pair(candidate) else None

    def newest_persistent_map():
        found = []
        if os.path.isdir(PERSISTENT_MAP_DIR):
            for path in glob.glob(os.path.join(PERSISTENT_MAP_DIR, "*.yaml")) + glob.glob(os.path.join(PERSISTENT_MAP_DIR, "*.yml")):
                if os.path.basename(path).startswith("map_") and validate_pair(path):
                    found.append(os.path.abspath(path))
        return max(found, key=map_rank) if found else None

    def newest_map():
        # Fallback for first V23 run / migration only. Persistent mapping data
        # outranks every source/install copy so an old packaged map cannot win.
        persistent = newest_persistent_map()
        if persistent:
            return persistent
        candidates = all_valid_maps()
        if not candidates:
            raise RuntimeError(
                "[AUTONOMOUS-MAP] No valid saved map pair found. Searched: " +
                ", ".join(search_dirs) +
                ". Save a map first with STOP + SAVE MAP or pass map:=/absolute/path/map.yaml"
            )
        return candidates[0]

    selected = ""
    fallback_reason = ""
    auto_request = requested in ("", "auto", "AUTO", OLD_PLACEHOLDER)

    def seed_latest_pointer(path):
        """Best-effort atomic repair of the mapping -> autonomous pointer."""
        try:
            os.makedirs(PERSISTENT_MAP_DIR, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix=".latest_map.", suffix=".tmp", dir=PERSISTENT_MAP_DIR, text=True)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as handle:
                    handle.write(os.path.abspath(path) + "\n")
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(tmp_path, LATEST_MAP_POINTER)
            except Exception:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
            return True, "repaired"
        except OSError as exc:
            return False, f"repair-failed:{exc}"

    def newest_current_slot_map():
        # map.launch.py / mapping_gui_cpp manages exactly three source-tree slots.
        # Prefer the newest valid slot before searching backup/install trees, so an
        # old packaged map cannot win merely because a build/install touched mtime.
        found = []
        for slot in (1, 2, 3):
            for ext in (".yaml", ".yml"):
                path = os.path.join(LEGACY_MAP_DIR, f"map_{slot}{ext}")
                if validate_pair(path):
                    try:
                        mtime = os.path.getmtime(path)
                    except OSError:
                        mtime = 0.0
                    found.append((mtime, slot, os.path.abspath(path)))
        if not found:
            return None
        return max(found, key=lambda item: (item[0], item[1]))[2]

    if auto_request:
        selected = committed_map()
        if selected:
            fallback_reason = "AUTO exact-map.launch-commit-pointer"
        else:
            # Recovery path for the real field failure where latest_map.txt was
            # missing/invalid even though a valid saved map still existed.
            # Priority: persistent runtime map -> current Map1/2/3 -> legacy/backup/install.
            selected = newest_persistent_map() or newest_current_slot_map()
            if not selected:
                try:
                    selected = newest_map()
                except RuntimeError as exc:
                    raise RuntimeError(
                        "[AUTONOMOUS-MAP] map:=auto could not recover because no valid saved map pair exists. "
                        f"Pointer={LATEST_MAP_POINTER}. {exc}"
                    ) from exc
            repaired, repair_status = seed_latest_pointer(selected)
            fallback_reason = f"AUTO pointer-missing/invalid -> newest-valid ({repair_status})"
    else:
        expanded = os.path.abspath(os.path.expanduser(requested))
        if validate_pair(expanded):
            selected = expanded
            fallback_reason = "explicit"
        else:
            basename = os.path.basename(requested)
            matching = []
            for directory in search_dirs:
                candidate = os.path.join(directory, basename)
                if validate_pair(candidate):
                    matching.append(candidate)
            if matching:
                selected = max(matching, key=map_rank)
                fallback_reason = "resolved-by-basename"
            else:
                selected = newest_map()
                fallback_reason = "requested-map-invalid/missing -> newest-valid"

    image_path = validate_pair(selected)
    if image_path is None:
        raise RuntimeError("[AUTONOMOUS-MAP] Selected map pair became invalid: " + selected)

    # Bind any KNOWN_POSE initialization to the exact YAML+image pair.
    map_hasher = hashlib.sha256()
    for map_part in (selected, image_path):
        with open(map_part, "rb") as fh:
            for chunk in iter(lambda: fh.read(1024 * 1024), b""):
                map_hasher.update(chunk)
    selected_map_sha256 = map_hasher.hexdigest()

    # Mapping GUI writes a map-hash-bound pose sidecar at STOP+SAVE.  Reusing
    # that pose is safe only when the sidecar hash exactly matches the selected
    # YAML+image pair.  When an older map has no sidecar, let AMCL perform real
    # scan-based global localization instead of waiting forever or inventing a
    # (0, 0, 0) pose.
    auto_pose_enabled = LaunchConfiguration("auto_initial_pose_from_mapping").perform(context).strip().lower() \
        not in ("0", "false", "no", "off")
    auto_global_enabled = LaunchConfiguration("auto_global_localization").perform(context).strip().lower() \
        not in ("0", "false", "no", "off")
    resolved_pose_mode = "GLOBAL_LOCALIZATION" if auto_global_enabled else "OPERATOR"
    resolved_pose_x = 0.0
    resolved_pose_y = 0.0
    resolved_pose_yaw = 0.0
    resolved_pose_hash = ""
    pose_log = (
        "[AUTONOMOUS-INITIALPOSE] GLOBAL_LOCALIZATION: no valid map-bound saved pose; "
        "AMCL will initialize from /map + /scan_nav"
        if auto_global_enabled else
        "[AUTONOMOUS-INITIALPOSE] OPERATOR: set Initial Pose in GUI/RViz"
    )
    sidecar = os.path.splitext(selected)[0] + ".autopose.json"
    if auto_pose_enabled and os.path.isfile(sidecar):
        try:
            with open(sidecar, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            sx = float(payload.get("x", 0.0))
            sy = float(payload.get("y", 0.0))
            syaw = float(payload.get("yaw", 0.0))
            side_hash = str(payload.get("map_sha256", "")).strip().lower()
            frames_ok = (
                str(payload.get("frame_id", "map")) == "map" and
                str(payload.get("child_frame_id", "base_footprint")) == "base_footprint"
            )
            numeric_ok = all(math.isfinite(v) for v in (sx, sy, syaw))
            if side_hash == selected_map_sha256.lower() and frames_ok and numeric_ok:
                resolved_pose_mode = "KNOWN_POSE"
                resolved_pose_x = sx
                resolved_pose_y = sy
                resolved_pose_yaw = syaw
                resolved_pose_hash = side_hash
                pose_log = (
                    f"[AUTONOMOUS-INITIALPOSE] KNOWN_POSE map-bound sidecar={sidecar}; "
                    f"pose=({sx:.3f},{sy:.3f},{syaw:.3f}); sha256={side_hash}"
                )
            else:
                resolved_pose_mode = "GLOBAL_LOCALIZATION" if auto_global_enabled else "OPERATOR"
                pose_log = (
                    f"[AUTONOMOUS-INITIALPOSE] {resolved_pose_mode}: sidecar rejected "
                    f"hash_ok={side_hash == selected_map_sha256.lower()} "
                    f"frames_ok={frames_ok} numeric_ok={numeric_ok}; sidecar={sidecar}"
                )
        except Exception as exc:
            resolved_pose_mode = "GLOBAL_LOCALIZATION" if auto_global_enabled else "OPERATOR"
            pose_log = (
                f"[AUTONOMOUS-INITIALPOSE] {resolved_pose_mode}: invalid sidecar "
                f"{sidecar}: {exc}"
            )
    elif auto_pose_enabled:
        pose_log = (
            f"[AUTONOMOUS-INITIALPOSE] GLOBAL_LOCALIZATION: no sidecar {sidecar}; "
            "AMCL will localize from the live LiDAR scan"
            if auto_global_enabled else
            f"[AUTONOMOUS-INITIALPOSE] OPERATOR: no sidecar {sidecar}; set Initial Pose once, or resave this map"
        )
    else:
        pose_log = (
            "[AUTONOMOUS-INITIALPOSE] GLOBAL_LOCALIZATION: map-bound pose disabled; "
            "using scan-based initialization"
            if auto_global_enabled else
            "[AUTONOMOUS-INITIALPOSE] OPERATOR: automatic initialization disabled"
        )

    candidates = all_valid_maps()
    newest_name = os.path.basename(candidates[0]) if candidates else os.path.basename(selected)
    pointer_target = committed_map()
    msg = (
        f"[AUTONOMOUS-MAP] selected={selected}; image={image_path}; "
        f"mode={fallback_reason}; committed={pointer_target or 'NONE'}; "
        f"newest_fallback={newest_name}; searched_dirs={len(search_dirs)}; "
        f"sha256={selected_map_sha256}"
    )

    # Build a SECOND, planning-only map for the GLOBAL costmap.  AMCL keeps the
    # untouched raw saved map on /map.  Tiny isolated occupied speckles inside
    # known free space are removed only from /nav_map so one noisy LiDAR endpoint
    # cannot inflate a large unusable island across the whole planner map.
    # Real/current obstacles are still protected by the LOCAL /scan obstacle layer.
    nav_map_selected = selected
    nav_filter_log = "[AUTONOMOUS-NAV-MAP] filter disabled; using raw map"
    filter_enabled = LaunchConfiguration("enable_nav_map_filter").perform(context).strip().lower()
    if filter_enabled not in ("0", "false", "no", "off"):
        max_cells = int(LaunchConfiguration("nav_map_filter_max_cells").perform(context))
        unknown_halo = int(LaunchConfiguration("nav_map_filter_unknown_halo_cells").perform(context))
        clear_start = LaunchConfiguration("nav_map_clear_start").perform(context).strip().lower()
        clear_start_enabled = clear_start not in ("0", "false", "no", "off")
        initial_x = float(LaunchConfiguration("initial_pose_x").perform(context))
        initial_y = float(LaunchConfiguration("initial_pose_y").perform(context))
        initial_yaw = float(LaunchConfiguration("initial_pose_yaw").perform(context))
        start_margin = float(LaunchConfiguration("nav_map_start_clear_margin").perform(context))
        prep_script = os.path.join(
            get_package_prefix("navigation"), "lib", "navigation", "prepare_nav_map.py")
        out_dir = "/tmp/navigation_autonomous_navmap"
        try:
            prep_cmd = [
                "/usr/bin/python3", prep_script,
                "--map-yaml", selected,
                "--output-dir", out_dir,
                "--max-cells", str(max_cells),
                "--unknown-halo-cells", str(unknown_halo),
            ]
            if clear_start_enabled:
                prep_cmd += [
                    "--clear-start",
                    "--start-x", str(initial_x),
                    "--start-y", str(initial_y),
                    "--start-yaw", str(initial_yaw),
                    "--start-half-length", "0.65",
                    "--start-half-width", "0.40",
                    "--start-margin", str(start_margin),
                ]
            result = subprocess.run(
                prep_cmd,
                check=True,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=10.0,
            )
            nav_match = re.search(r"^NAV_MAP=(.+)$", result.stdout, flags=re.MULTILINE)
            if not nav_match:
                raise RuntimeError("prepare_nav_map.py returned no NAV_MAP path")
            nav_map_selected = nav_match.group(1).strip()
            removed_match = re.search(r"^REMOVED_CELLS=(\d+)$", result.stdout, flags=re.MULTILINE)
            comps_match = re.search(r"^REMOVED_COMPONENTS=(\d+)$", result.stdout, flags=re.MULTILINE)
            removed = removed_match.group(1) if removed_match else "?"
            comps = comps_match.group(1) if comps_match else "?"
            start_match = re.search(r"^START_CLEARED_CELLS=(\d+)$", result.stdout, flags=re.MULTILINE)
            start_cleared = start_match.group(1) if start_match else "?"
            nav_filter_log = (
                f"[AUTONOMOUS-NAV-MAP] raw=/map; planning=/nav_map; "
                f"removed_cells={removed}; removed_components={comps}; "
                f"start_clear={clear_start_enabled}; start_cleared_cells={start_cleared}; "
                f"start_pose=({initial_x:.3f},{initial_y:.3f},{initial_yaw:.3f}); "
                f"start_margin={start_margin:.2f}; max_cells={max_cells}; "
                f"unknown_halo={unknown_halo}; yaml={nav_map_selected}")
        except Exception as exc:
            nav_map_selected = selected
            nav_filter_log = (
                f"[AUTONOMOUS-NAV-MAP] filter FAILED -> raw map fallback: {exc}")

    return [
        SetLaunchConfiguration("resolved_map", selected),
        SetLaunchConfiguration("resolved_map_sha256", selected_map_sha256),
        SetLaunchConfiguration("resolved_nav_map", nav_map_selected),
        SetLaunchConfiguration("resolved_initial_pose_mode", resolved_pose_mode),
        SetLaunchConfiguration("resolved_initial_pose_x", str(resolved_pose_x)),
        SetLaunchConfiguration("resolved_initial_pose_y", str(resolved_pose_y)),
        SetLaunchConfiguration("resolved_initial_pose_yaw", str(resolved_pose_yaw)),
        SetLaunchConfiguration("resolved_initial_pose_map_sha256", resolved_pose_hash),
        LogInfo(msg=msg),
        LogInfo(msg=pose_log),
        LogInfo(msg=nav_filter_log),
    ]



def _resolve_yolo_model(context):
    """Resolve the 5-class AGV YOLO model from /home/otomasi2/ros/models.

    The generic yolov8n.onnx is intentionally NOT selected automatically because
    it is an 80-class COCO network, while this detector expects the trained
    5-class AGV output [1, 9, 8400].
    """
    requested = LaunchConfiguration("yolo_model").perform(context).strip()
    models_dir = LaunchConfiguration("yolo_models_dir").perform(context).strip()
    models_dir = os.path.abspath(os.path.expanduser(models_dir or DEFAULT_YOLO_MODELS_DIR))

    auto_request = requested.lower() in ("", "auto")
    candidates = []
    if not auto_request:
        candidates.append(os.path.abspath(os.path.expanduser(requested)))
    else:
        # Prefer the OpenCV-exported model because autonomous defaults to
        # OpenCV-DNN. Both AGV ONNX files have the expected 5-class output.
        candidates.extend([
            os.path.join(models_dir, "yolov8n_agv_forklift_opencv.onnx"),
            os.path.join(models_dir, "yolov8n_agv_forklift.onnx"),
        ])
        try:
            yolo_share = get_package_share_directory("yolo_obstacle_detection_ros2")
            candidates.extend([
                os.path.join(yolo_share, "models", "yolov8n_agv_forklift_opencv.onnx"),
                os.path.join(yolo_share, "models", "yolov8n_agv_forklift.onnx"),
            ])
        except Exception:
            pass
        candidates.extend([
            "/home/otomasi2/ros/src/yolo_obstacle_detection_ros2/models/yolov8n_agv_forklift_opencv.onnx",
            "/home/otomasi2/ros/src/yolo_obstacle_detection_ros2/models/yolov8n_agv_forklift.onnx",
        ])

    # When TensorRT is requested, prefer an ONNX file that already has a
    # matching serialized engine beside it. The engine builder supplied with
    # this package creates ``yolov8n_agv_forklift.engine`` by default; the old
    # resolver preferred the ``*_opencv.onnx`` file first, which could silently
    # bypass that engine and force the much slower OpenCV-CUDA backend.
    request_trt = LaunchConfiguration("use_tensorrt").perform(context).strip().lower() in (
        "1", "true", "yes", "on"
    )
    if auto_request and request_trt:
        engine_ready = []
        for candidate in candidates:
            if not os.path.isfile(candidate):
                continue
            root, ext = os.path.splitext(candidate)
            engine = root + ".engine" if ext.lower() == ".onnx" else candidate
            if os.path.isfile(engine):
                engine_ready.append(candidate)
        if engine_ready:
            candidates = engine_ready + [c for c in candidates if c not in engine_ready]

    selected = next((candidate for candidate in candidates if os.path.isfile(candidate)), "")
    if not selected:
        selected = candidates[0] if candidates else os.path.join(
            models_dir, "yolov8n_agv_forklift_opencv.onnx")

    selected_root, selected_ext = os.path.splitext(selected)
    selected_engine = selected_root + ".engine" if selected_ext.lower() == ".onnx" else selected
    if os.path.isfile(selected):
        reason = "FOUND + TensorRT engine" if request_trt and os.path.isfile(selected_engine) else "FOUND"
    else:
        reason = "NOT FOUND (camera passthrough mode)"
    return [
        SetLaunchConfiguration("resolved_yolo_model", selected),
        LogInfo(msg=(
            f"[AUTONOMOUS-YOLO] models_dir={models_dir}; "
            f"selected={selected} [{reason}]"
        )),
    ]

def _verify_autonomous_dependencies(context):
    """Fail early with a clear install command when any required Nav2 package is missing."""
    # Verify the COMPLETE autonomous Nav2 stack, not only localization.
    # This makes a missing planner/controller/costmap plugin fail clearly before
    # RViz comes up with a partially working navigation system.
    required = [
        "nav2_map_server",
        "nav2_amcl",
        "nav2_planner",
        "nav2_smac_planner",
        "nav2_controller",
        "nav2_mppi_controller",
        "nav2_behaviors",
        "nav2_bt_navigator",
        "nav2_behavior_tree",
        "nav2_velocity_smoother",
        "nav2_collision_monitor",
        "nav2_lifecycle_manager",
        "nav2_costmap_2d",
        "nav2_msgs",
        "rviz2",
        "robot_state_publisher",
        "joint_state_publisher",
    ]
    missing = []
    for package_name in required:
        try:
            get_package_prefix(package_name)
        except Exception:
            missing.append(package_name)

    if missing:
        raise RuntimeError(
            "[AUTONOMOUS-DEPS] Missing ROS 2 package(s): " + ", ".join(missing) +
            ". Install Nav2 on ROS 2 Humble with: "
            "sudo apt update && sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup"
        )

    return [LogInfo(msg=(
        "[AUTONOMOUS-DEPS] COMPLETE Nav2 stack FOUND: map_server + AMCL + "
        "Smac + controller/MPPI + behaviors + BT + costmaps + smoother + "
        "collision_monitor + lifecycle"
    ))]


def _verify_yolo_runtime(context):
    """Resolve the fastest available GPU backend without expected fallback warnings."""
    enabled = LaunchConfiguration("enable_yolo").perform(context).strip().lower()
    if enabled not in ("1", "true", "yes", "on"):
        return [
            SetLaunchConfiguration("resolved_use_tensorrt", "false"),
            LogInfo(msg="[AUTONOMOUS-YOLO] disabled by enable_yolo:=false"),
        ]

    request_trt = LaunchConfiguration("use_tensorrt").perform(context).strip().lower() in (
        "1", "true", "yes", "on"
    )
    model = os.path.abspath(os.path.expanduser(
        LaunchConfiguration("resolved_yolo_model").perform(context).strip()))
    engine_arg = LaunchConfiguration("yolo_engine").perform(context).strip()
    engine = ""
    if engine_arg:
        engine = os.path.abspath(os.path.expanduser(engine_arg))
    elif model:
        root, ext = os.path.splitext(model)
        engine = root + ".engine" if ext.lower() == ".onnx" else model

    if request_trt and engine and os.path.isfile(engine):
        return [
            SetLaunchConfiguration("resolved_use_tensorrt", "true"),
            LogInfo(msg=f"[AUTONOMOUS-YOLO] GPU backend=TensorRT FP16 engine={engine}"),
        ]

    if os.path.isfile(model):
        reason = "TensorRT engine absent; " if request_trt else ""
        return [
            SetLaunchConfiguration("resolved_use_tensorrt", "false"),
            LogInfo(msg=(
                f"[AUTONOMOUS-YOLO] {reason}GPU backend=OpenCV CUDA FP16 model={model}"
            )),
        ]

    return [
        SetLaunchConfiguration("resolved_use_tensorrt", "false"),
        LogInfo(msg=(
            f"[AUTONOMOUS-YOLO] model not found: {model}. Camera remains available, "
            "but detections require a valid ONNX/TensorRT model."
        )),
    ]


def _verify_camera_gpu_runtime(context):
    """Fail early when strict NVIDIA camera decode is requested but unavailable."""
    enabled = LaunchConfiguration("enable_camera").perform(context).strip().lower()
    strict = LaunchConfiguration("require_gpu_decode").perform(context).strip().lower()
    if enabled not in ("1", "true", "yes", "on") or strict not in ("1", "true", "yes", "on"):
        return [LogInfo(msg="[AUTONOMOUS-CAMERA] strict NVIDIA decode preflight skipped")]

    required_plugins = ("nvv4l2decoder", "nvvidconv")
    missing = []
    for plugin in required_plugins:
        try:
            result = subprocess.run(
                ["gst-inspect-1.0", plugin],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=4.0,
                check=False,
            )
            if result.returncode != 0:
                missing.append(plugin)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            missing.append(plugin)
    if missing:
        raise RuntimeError(
            "[AUTONOMOUS-CAMERA] strict GPU decode requires Jetson GStreamer plugin(s): "
            + ", ".join(sorted(set(missing)))
            + ". CPU camera fallback is intentionally disabled."
        )
    return [LogInfo(msg=(
        "[AUTONOMOUS-CAMERA] NVIDIA hardware decode preflight PASS: "
        "nvv4l2decoder + nvvidconv"
    ))]


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
    _claim_autonomous_singleton()
    nav_share = get_package_share_directory("navigation")
    ensure_stage2_lidar_runtime(nav_share)
    ensure_stage5_localization_runtime(nav_share)
    ensure_stage5_planning_runtime(nav_share)
    esc_share = get_package_share_directory("esc")
    yolo_share = get_package_share_directory("yolo_obstacle_detection_ros2")

    # PART-4 Stage-1: vehicle_geometry.yaml is the single geometry authority.
    # Synchronize only geometry-owned fields before reading Nav2/ESC runtime YAML.
    geometry_state = sync_derived_configs(nav_share, esc_share)
    runtime_manifest = ensure_runtime_schema(nav_share, esc_share, yolo_share)
    if not runtime_manifest.get("valid", False):
        raise RuntimeError("runtime configuration manifest is incomplete: " + str(runtime_manifest.get("missing_files", [])))
    g = geometry_state["geometry"]
    geometry_valid = all(geometry_state["validated"].values())
    allow_unvalidated = os.environ.get("AGV_ALLOW_UNVALIDATED_GEOMETRY", "0").strip().lower() in ("1", "true", "yes", "on")
    # Runtime bring-up must remain observable even before the physical geometry
    # calibration is complete.  Older code raised GeometryError here; when this
    # launch was included by gui.launch.py that exception tore down the entire
    # parent launch and looked like a Qt force-close.  Keep the stack alive for
    # diagnostics, but pass geometry_validated=false into the continuous motion
    # interlock below.  Therefore Nav2/GUI/sensors can be inspected while the
    # actuator path remains fail-closed until the calibration wizard marks every
    # physical item validated.  AGV_ALLOW_UNVALIDATED_GEOMETRY is retained only
    # as an explicit bench override for the motion interlock.
    geometry_motion_valid = bool(geometry_valid or allow_unvalidated)
    missing_geometry = [name for name, ok in geometry_state["validated"].items() if not ok]

    robot_description = xacro.process_file(
        os.path.join(nav_share, "urdf", "agv.urdf.xacro"),
        mappings={
            "cad_to_base_yaw_rad": str(g["cad_to_base_yaw_rad"]),
            "max_steering_angle_rad": str(g["max_steering_angle_rad"]),
        },
    ).toxml()
    geometry_log = LogInfo(msg=(
        f"[AUTONOMOUS-GEOMETRY] REP-103 adapter yaw={g['cad_to_base_yaw_rad']:.6f} rad; "
        f"wheelbase={g['wheelbase_m']:.4f} m; steering={g['max_steering_angle_rad']:.4f} rad; "
        f"Rmin={g['min_turning_radius_m']:.4f} m; validated={geometry_state['validated']}; "
        f"motion_gate={'ENABLED' if geometry_motion_valid else 'HOLD'}"
        + ("" if geometry_motion_valid else f"; missing={missing_geometry}")
    ))

    # ------------------------------------------------------------------
    # Robot visual fallback policy
    # ------------------------------------------------------------------
    # Mapping mode already has a known-good RobotModel pipeline using the
    # visual_/ TF prefix.  Autonomous deliberately reuses that visualization
    # design instead of failing when RViz cannot resolve an installed STL.
    #
    # Resolution order:
    #   1) package://navigation/... in the installed package share
    #   2) the workspace source tree (/home/otomasi2/ros/src/navigation)
    #   3) a primitive URDF fallback that preserves the exact link/joint tree
    #
    # Step 2 is important with --symlink-install / partial installs: map.launch
    # can still have the source meshes while an older install tree misses them.
    mesh_resources = sorted(set(re.findall(
        r'filename=[\"\']package://navigation/([^\"\']+\.(?:STL|stl))[\"\']',
        robot_description,
    )))

    workspace_root = os.path.abspath(os.path.join(nav_share, "../../../.."))
    source_nav_candidates = [
        os.path.join(workspace_root, "src", "navigation"),
        "/home/otomasi2/ros/src/navigation",
    ]
    source_nav = next(
        (p for p in source_nav_candidates if os.path.isdir(os.path.join(p, "meshes"))),
        "",
    )

    installed_missing = [
        rel for rel in mesh_resources if not os.path.isfile(os.path.join(nav_share, rel))
    ]
    source_recovered = []
    unresolved = []

    if installed_missing and source_nav:
        for rel in installed_missing:
            source_mesh = os.path.join(source_nav, rel)
            if os.path.isfile(source_mesh):
                package_uri = "package://navigation/" + rel
                file_uri = "file://" + source_mesh
                robot_description = robot_description.replace(package_uri, file_uri)
                source_recovered.append(rel)
            else:
                unresolved.append(rel)
    else:
        unresolved = list(installed_missing)

    def _primitive_visual(parent, geometry_kind, attrs, xyz="0 0 0", rpy="0 0 0"):
        visual = ET.SubElement(parent, "visual")
        ET.SubElement(visual, "origin", {"xyz": xyz, "rpy": rpy})
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, geometry_kind, attrs)
        material = ET.SubElement(visual, "material", {"name": "fallback_gray"})
        ET.SubElement(material, "color", {"rgba": "0.55 0.58 0.62 1"})

    def _build_primitive_fallback(urdf_xml):
        """Keep the original kinematic tree, replace CAD-only visuals by primitives."""
        root_xml = ET.fromstring(urdf_xml)
        for link in root_xml.findall("link"):
            name = link.attrib.get("name", "")
            for child in list(link):
                if child.tag in ("visual", "collision"):
                    link.remove(child)

            if name == "body_link":
                _primitive_visual(link, "box", {"size": "1.45 0.90 0.55"}, xyz="0 0 0.30")
            elif name == "mast_link":
                _primitive_visual(link, "box", {"size": "0.16 0.82 1.55"}, xyz="0 0 0.78")
            elif name == "top_cross_beam_link":
                _primitive_visual(link, "box", {"size": "0.16 0.88 0.14"})
            elif name == "fork_link":
                _primitive_visual(link, "box", {"size": "1.05 0.72 0.08"}, xyz="0.38 0 -0.02")
            elif name in (
                "front_left_wheel_link", "front_right_wheel_link",
                "rear_left_wheel_link", "rear_right_wheel_link",
            ):
                _primitive_visual(
                    link, "cylinder", {"radius": "0.15", "length": "0.10"},
                    rpy="1.57079632679 0 0")
            elif name in ("steering_link", "steering_control_link"):
                _primitive_visual(link, "box", {"size": "0.28 0.18 0.12"})
            elif name in ("front_left_steering_link", "front_right_steering_link"):
                _primitive_visual(link, "box", {"size": "0.18 0.12 0.12"})
            elif name == "sensor_mount_link":
                _primitive_visual(link, "box", {"size": "0.35 0.30 0.08"})
            elif name == "lidar_link":
                _primitive_visual(link, "cylinder", {"radius": "0.055", "length": "0.07"})
            elif name in ("camera_link", "camera_color_frame"):
                _primitive_visual(link, "box", {"size": "0.09 0.05 0.04"})
            elif name == "imu_link":
                _primitive_visual(link, "box", {"size": "0.05 0.04 0.02"})
            elif name == "gearbox_link":
                _primitive_visual(link, "cylinder", {"radius": "0.09", "length": "0.18"},
                                  rpy="1.57079632679 0 0")
        return ET.tostring(root_xml, encoding="unicode")

    if unresolved or not mesh_resources:
        robot_description = _build_primitive_fallback(robot_description)
        visual_mode = (
            "PRIMITIVE FALLBACK (STL unavailable in install and source; "
            "same map-style visual TF pipeline retained)"
        )
    elif source_recovered:
        visual_mode = (
            f"MAP-STYLE CAD via SOURCE FALLBACK: {len(source_recovered)} STL(s) "
            f"resolved from {source_nav}"
        )
    else:
        visual_mode = (
            f"MAP-STYLE CAD: {len(mesh_resources)} STL(s) resolved from installed package"
        )

    urdf_mesh_log = LogInfo(msg=f"[AUTONOMOUS-URDF] {visual_mode}")

    # joint_state_publisher in Humble accepts the URDF file as its positional
    # source. Feed ESC joint states through it and let it provide zero/default
    # positions for movable joints that have not reported yet.
    joint_state_urdf = os.path.join(
        tempfile.gettempdir(), "navigation_autonomous_joint_states.urdf")
    with open(joint_state_urdf, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_description)

    nav2_params = _runtime_config(nav_share, "navigation", "nav2_ackermann.yaml")
    ekf_params = _runtime_config(nav_share, "navigation", "ekf_autonomous.yaml")
    hector_params = _runtime_config(nav_share, "navigation", "hector_autonomous.yaml")
    collision_params = _runtime_config(nav_share, "navigation", "collision_monitor.yaml")
    autonomy_health_params = _runtime_config(nav_share, "navigation", "autonomy_health.yaml")
    manual_motion_health_params = _runtime_config(nav_share, "navigation", "manual_motion_health.yaml")
    localization_startup_params = _runtime_config(nav_share, "navigation", "localization_startup.yaml")
    localization_timing_params = _runtime_config(nav_share, "navigation", "localization_timing.yaml")
    bt_xml = os.path.join(nav_share, "behavior_trees", "ackermann_navigate_to_pose.xml")
    default_yolo_model = DEFAULT_YOLO_MODEL

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "maps_dir",
            default_value=DEFAULT_MAP_DIR,
            description="Persistent directory containing maps saved by Mapping GUI",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(nav_share, "maps", "map_Navigation.yaml"),
            description=(
                "Single operational Navigation Map used by AMCL and Nav2. "
                "BAB 4.2 Map 1/2/3 remain mapping evidence and are not selected here."
            ),
        ),
        DeclareLaunchArgument(
            "enable_nav_map_filter", default_value="true",
            description=(
                "Create a planning-only /nav_map by removing tiny isolated occupied "
                "speckles while keeping the raw /map untouched for AMCL"),
        ),
        DeclareLaunchArgument(
            "nav_map_filter_max_cells", default_value="3",
            description="Maximum isolated occupied component size removed from /nav_map",
        ),
        DeclareLaunchArgument(
            "nav_map_filter_unknown_halo_cells", default_value="2",
            description="Preserve tiny obstacles that are close to unknown map regions",
        ),
        DeclareLaunchArgument(
            "nav_map_clear_start", default_value="false",
            description=(
                "Optional planning-map start-footprint cleanup. Disabled by default because "
                "PART-5 no longer assumes the robot starts at map origin."),
        ),
        DeclareLaunchArgument(
            "nav_map_start_clear_margin", default_value="0.10",
            description="Extra planning-only free margin [m] around the 1.30 x 0.80 m start footprint",
        ),
        DeclareLaunchArgument("imu_port", default_value="/tmp/agv_devices/imu"),
        DeclareLaunchArgument("lidar_port", default_value="/tmp/agv_devices/lidar"),
        DeclareLaunchArgument("lidar_baudrate", default_value="230400"),
        DeclareLaunchArgument("lidar_intensity", default_value="true"),
        DeclareLaunchArgument("lidar_intensity_bits", default_value="16"),
        DeclareLaunchArgument("lidar_strict_checksum", default_value="true"),
        DeclareLaunchArgument("esc_port", default_value="/dev/esc"),
        DeclareLaunchArgument("enable_winch", default_value="true"),
        DeclareLaunchArgument("winch_port", default_value="auto"),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("camera_device", default_value="auto"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument("use_gpu_decode", default_value="true",
            description="GPU camera decode via NVIDIA nvv4l2decoder (MJPG hardware decode)"),
        DeclareLaunchArgument("require_gpu_decode", default_value="false",
            description="Strict camera mode: do not silently fall back to CPU MJPG decode"),
        DeclareLaunchArgument("enable_yolo", default_value="true"),
        DeclareLaunchArgument(
            "enable_hole_alignment", default_value="true",
            description="Pallet-hole alignment runtime; enabled by default for complete perception telemetry"),
        DeclareLaunchArgument(
            "yolo_models_dir",
            default_value=DEFAULT_YOLO_MODELS_DIR,
            description="Directory containing the trained AGV YOLO model files",
        ),
        DeclareLaunchArgument(
            "yolo_model",
            default_value=default_yolo_model,
            description=(
                "AGV YOLO ONNX model. 'auto' prefers "
                "yolov8n_agv_forklift_opencv.onnx under yolo_models_dir."
            ),
        ),
        DeclareLaunchArgument("use_tensorrt", default_value="true",
            description="Prefer TensorRT FP16 when a compatible engine exists; otherwise use OpenCV CUDA FP16"),
        DeclareLaunchArgument("yolo_engine", default_value=""),
        DeclareLaunchArgument("yolo_require_cuda", default_value="false",
            description=("Prefer CUDA/TensorRT, but keep the perception node alive with a documented "
                         "fallback instead of respawn-looping when a GPU backend/model is temporarily unavailable")),
        DeclareLaunchArgument("yolo_max_inference_fps", default_value="30.0",
            description="YOLO processing ceiling; 30 Hz allows TensorRT to follow the 30 FPS camera while latest-frame dropping prevents backlog"),
        DeclareLaunchArgument("enable_warehouse_person", default_value="true",
            description="Enable low-rate person dynamic semantics; persistent LiDAR corridor obstacles cover static box/object"),
        DeclareLaunchArgument(
            "auto_initial_pose_from_mapping", default_value="true",
            description=(
                "Use mapping_gui STOP+SAVE map-hash-bound .autopose.json when available; "
                "otherwise use scan-based global localization when enabled"),
        ),
        DeclareLaunchArgument(
            "auto_global_localization", default_value="true",
            description=(
                "When the selected map has no valid pose sidecar, call AMCL global "
                "localization and wait for measured convergence instead of assuming map origin"),
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0",
            description="Legacy planning-map start-clear X only; NOT used to initialize AMCL"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0",
            description="Legacy planning-map start-clear Y only; NOT used to initialize AMCL"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0",
            description="Legacy planning-map start-clear yaw only; NOT used to initialize AMCL"),
        DeclareLaunchArgument(
            "enable_nav2", default_value="true",
            description="Start and lifecycle-activate the complete Nav2 autonomous stack",
        ),
        DeclareLaunchArgument("start_web_gui", default_value="true",
            description="Start browser HMI automatically with autonomous.launch.py"),
        DeclareLaunchArgument("web_bind_address", default_value="127.0.0.1",
            description="Loopback-only by default; use 0.0.0.0 only on a trusted LAN"),
        DeclareLaunchArgument("web_port", default_value="5005",
            description="Local browser HMI TCP port"),
        DeclareLaunchArgument("web_read_only", default_value="false",
            description="Disable write/control endpoints when true"),
        DeclareLaunchArgument("enable_rviz", default_value="false",
            description="RViz diagnostics are opt-in; browser HMI is default to avoid headless crash/respawn load"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(nav_share, "rviz", "autonomous.rviz"),
        ),

        # FastDDS UDP-only transport profile. Disables shared-memory (SHM)
        # transport so a force-killed previous run cannot leave orphan DDS
        # participants that collide on /dev/shm/fastrtps_port* and hide the
        # entire ROS graph from `ros2 node list` / lifecycle / TF queries.
        SetEnvironmentVariable(
            name="FASTRTPS_DEFAULT_PROFILES_FILE",
            value=os.path.join(nav_share, "config", "fastdds_udp_only_profile.xml"),
        ),
        SetEnvironmentVariable(name="RMW_FASTRTPS_USE_SHM", value="0"),
    ]

    verify_dependencies = OpaqueFunction(function=_verify_autonomous_dependencies)
    verify_camera_gpu = OpaqueFunction(function=_verify_camera_gpu_runtime)
    resolve_yolo_model = OpaqueFunction(function=_resolve_yolo_model)
    verify_yolo_runtime = OpaqueFunction(function=_verify_yolo_runtime)
    resolve_map = OpaqueFunction(function=_resolve_map_yaml)

    localization_mode_log = LogInfo(
        msg=(
            "[AUTONOMOUS-LOCALIZATION] saved map is READ-ONLY; AMCL uses /scan_nav; "
            "local EKF = /esc/odom + /imu/data only; /lidar/odom is DIAGNOSTIC ONLY; "
            "AMCL uses /scan_nav and exclusively owns map->odom; EKF owns odom->base_footprint; "
            "AMCL never assumes (0,0,0) — map-bound pose, scan-based global localization, "
            "or an explicit operator pose is required"
        )
    )

    # RViz is an independent diagnostics UI. It must open even while AMCL or
    # global costmap gates are fail-closed, otherwise the operator loses the
    # primary visualization needed to diagnose those exact startup conditions.
    rviz_desktop_env = _resolve_rviz_desktop_env()
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_autonomous",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            )
        }],
        additional_env=rviz_desktop_env,
        respawn=True,
        respawn_delay=2.0,
    )

    # Browser HMI starts independently from AMCL/Nav2 readiness so diagnostics
    # remain available while the autonomous stack is still fail-closed.
    web_gui = Node(
        package="navigation",
        executable="agv_web_gui",
        name="agv_web_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_web_gui")),
        parameters=[{
            "bind_address": LaunchConfiguration("web_bind_address"),
            "port": ParameterValue(LaunchConfiguration("web_port"), value_type=int),
            "read_only": ParameterValue(LaunchConfiguration("web_read_only"), value_type=bool),
            "camera_jpeg_fps": 5.0,
            "mag_heading_offset_rad": 0.013525733461806364,
            "mag_heading_sign": 1.0,
            "mag_heading_validation_rmse_deg": 3.0407979290635105,
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        # The browser HMI is the ownership anchor for this full-stack session.
        # Do not respawn it silently: if it exits, the launch-wide exit handler
        # below shuts the full stack down so LiDAR/IMU/camera cannot remain
        # active without the operator Web GUI.
        respawn=False,
    )

    web_gui_exit_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=web_gui,
            on_exit=[
                LogInfo(msg=(
                    "[WEB-GUI-OWNER] Web GUI exited; shutting down the full stack "
                    "so sensor drivers cannot remain active without the HMI."
                )),
                EmitEvent(event=Shutdown(
                    reason="Web GUI exited; stop sensor-owned autonomous stack"
                )),
            ],
        )
    )

    # BAB 4.2.1-4.2.3 mapping control lives inside autonomous.  It reuses the
    # already-running LiDAR/IMU/EKF and starts SLAM-only, so hardware drivers
    # are never duplicated.  Localhost remains the single operator surface.
    mapping_controller = Node(
        package="navigation",
        executable="mapping_web_controller.py",
        name="mapping_web_controller",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_web_gui")),
        parameters=[{
            "shared_sensor_mode": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    # Runtime Map 1/2/3 -> Nav2 selector. This controller never owns hardware;
    # it coordinates only saved-map loading, AMCL re-localization and existing
    # Nav2 lifecycle managers. Mapping BAB 4.2 remains a separate controller.
    nav_map_switch_controller = Node(
        package="navigation",
        executable="nav_map_switch_controller.py",
        name="nav_map_switch_controller",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_web_gui")),
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    # V56: RViz no longer starts on a blind fixed delay.  The autonomous
    # RobotModel uses Fixed Frame=map and TF Prefix=visual_, so an early RViz
    # launch can race map->odom / visual TF and intermittently show RobotModel
    # ERROR even though the model becomes valid moments later.  A bounded gate
    # below starts RViz only after the actual description + TF chain is ready;
    # on timeout it still opens RViz for diagnostics.

    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 30.0,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # Visualization-only tree, intentionally matching map.launch.py.  The real
    # navigation TF remains map -> odom -> base_footprint.  This second RSP only
    # prefixes URDF child frames so RViz resolves the model exactly like mapping.
    visual_robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="autonomous_visual_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "frame_prefix": "visual_/",
            "publish_frequency": 30.0,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
        remappings=[("robot_description", "robot_description_autonomous_visual")],
    )

    # V38 visualization bridge: the isolated visual_/ RobotModel follows the
    # hardware IMU/EKF through stationary hysteresis.  Raw direct-yaw display
    # made sub-degree AHRS noise look like real chassis motion in RViz.
    # Navigation TF ownership is untouched: AMCL still owns map->odom and EKF
    # still owns odom->base_footprint.  Only odom->visual_/base_footprint is
    # published here, so the RViz CAD can track hardware IMU motion without
    # creating a second navigation TF owner.
    visual_base_bridge = Node(
        package="navigation",
        executable="imu_visual_tf_node",
        name="autonomous_imu_visual_tf_node",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "odom_topic": "/odometry/filtered",
            "visual_parent_frame": "odom",
            "visual_child_frame": "visual_/base_footprint",
            "direct_imu_yaw": False,
            "imu_stationary_enter_rad_s": 0.020,
            "imu_stationary_exit_rad_s": 0.060,
            "stationary_confirm_s": 0.20,
            "esc_stationary_linear_mps": 0.030,
            "esc_stationary_angular_rad_s": 0.050,
            "esc_timeout_s": 0.60,
            "lidar_stationary_linear_mps": 0.025,
            "lidar_stationary_angular_rad_s": 0.050,
            "position_filter_alpha": 0.20,
            "yaw_filter_alpha": 0.25,
            "visual_position_deadband_m": 0.0080,
            "visual_yaw_deadband_rad": 0.0040,
            "visual_model_yaw_offset_rad": 3.141592653590,
        }],
    )

    joint_state = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher_autonomous",
        output="screen",
        arguments=[joint_state_urdf],
        parameters=[{
            "source_list": ["/esc/joint_states"],
            "rate": 20,
            "publish_default_positions": True,
            "publish_default_velocities": False,
            "publish_default_efforts": False,
            "use_mimic_tags": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    rviz_model_gate = Node(
        package="navigation",
        executable="rviz_robot_model_ready_gate.py",
        name="rviz_robot_model_ready_gate",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        parameters=[{
            "timeout_s": 6.0,
            "fail_open": True,
            "fixed_frame": "map",
            "required_frames": [
                "visual_/base_footprint",
                "visual_/body_link",
                "visual_/fork_link",
                "visual_/lidar_link",
            ],
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # Gate exit code is always zero after READY or bounded timeout.  Register
    # this transition before the foundation starts so RViz has exactly one
    # launch path and cannot race the prefixed RobotModel TF tree.
    start_rviz_after_robot_model_ready = _success_only_exit(
        rviz_model_gate, [rviz], "rviz-robot-model-ready")

    esc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(esc_share, "launch", "esc.launch.py")),
        launch_arguments={
            "profile": "ackermann_1_board.yaml",
            "board0_port": LaunchConfiguration("esc_port"),
            "enable_winch": LaunchConfiguration("enable_winch"),
            "winch_port": LaunchConfiguration("winch_port"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "enable_keyboard": "false",
            "enable_joystick": "false",
            "enable_nav2": LaunchConfiguration("enable_nav2"),
            "standalone_mode": "false",
            "require_autonomy_gate": "true",
            "require_manual_gate": "true",
            "nav2_topic": "/cmd_vel",
            "actuator_topic": "/cmd_vel/actuator",
            "joint_states_topic": "/esc/joint_states",
        }.items(),
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            )
        }],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

    # READ-ONLY saved-map server. This is localization, not mapping. No node in
    # this launch writes /map or saves a new map; nav2_map_server only serves the
    # occupancy grid loaded from resolved_map. It starts independently from the
    # sensor gate so RViz can show the saved map while sensors are starting.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params,
            {
                "yaml_filename": LaunchConfiguration("resolved_map"),
                "topic_name": "map",
                "frame_id": "map",
            },
        ],
    )

    # Planning-only map server.  The global StaticLayer subscribes to /nav_map;
    # AMCL continues to localize against the untouched /map above.
    nav_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="nav_map_server",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
            "yaml_filename": LaunchConfiguration("resolved_nav_map"),
            "topic_name": "nav_map",
            "frame_id": "map",
        }],
    )

    # PART-5 Stage-2: /nav_map is immutable for the entire autonomous session.
    # Any deterministic speckle cleanup happens once before map_server starts;
    # no runtime node is allowed to erase static cells based on AMCL pose.

    # ── SENSOR STARTUP: mirror the KNOWN-GOOD mapping path ──────────────────
    # Mapping runtime proves this hardware path works reliably:
    #   one serial resolver -> /tmp/agv_devices/imu + lidar -> staggered drivers.
    # Autonomous used to add three concurrent resolvers + alias-promotion event
    # chains.  If any event was missed/stalled, all physical topics could remain
    # silent while RViz/map_server kept running.  Remove that complexity.
    #
    # V17: hardware drivers are gated on deterministic software recovery ->
    # topology resolver -> non-invasive serial handoff.  The preflight can perform
    # a CP210x software replug, and the handoff gate never opens/closes the tty.
    # This removes the probe-close-reopen race seen in the 2026-08-19 EIO logs.
    # Camera remains independent and uses its own V4L2 discovery/reconnect path.

    # Run the installed ownership preflight BEFORE starting any new hardware
    # driver. This is stronger than merely deleting aliases: it terminates stale
    # mapping/autonomous sensor owners first, then recreates clean alias state.
    preflight_script = os.path.join(
        get_package_prefix("navigation"), "lib", "navigation",
        "autonomous_sensor_preflight.sh")
    sensor_preflight = ExecuteProcess(
        cmd=["bash", preflight_script],
        output="screen",
    )

    # Perception has its own short ownership cleanup. Run it as the very first
    # hardware stage so an orphan Astra/YOLO process from an older GUI session
    # cannot keep the shared USB tree busy while CP210x roles are resolved. This
    # action only kills stale perception owners; it does NOT start the camera.
    perception_preflight_script = os.path.join(
        get_package_prefix("navigation"), "lib", "navigation",
        "perception_preflight.sh")
    perception_preflight = ExecuteProcess(
        cmd=["bash", perception_preflight_script],
        output="screen",
    )

    # Resolve IMU + LiDAR atomically in one process.  This avoids two
    # concurrent protocol probes and two writers racing on /tmp/agv_devices
    # during CP210x re-enumeration.
    # Serial resolver intentionally matches mapping_runtime.launch.py.
    # It can publish each role alias as soon as that physical device is known;
    # autonomous does NOT wait for the combined resolver process to exit before
    # allowing a healthy IMU or LiDAR driver to start.
    role_resolver_serial = ExecuteProcess(
        cmd=[
            "/usr/bin/python3", os.path.join(nav_share, "tools", "resolve_usb_roles.py"),
            "--output-dir", "/tmp/agv_devices",
            "--require-imu", "true", "--require-lidar", "true",
            "--require-camera", "false",
            "--stable-seconds", "0.25",
        ], output="screen")

    # Camera discovery is deliberately separated from the CP210x resolver.
    # It runs only after both serial streams have already been proven healthy,
    # and writes a fixed /tmp/agv_devices/camera alias so the V4L2 driver never
    # scans/opens every /dev/video* endpoint on the shared USB tree.
    role_resolver_camera = ExecuteProcess(
        cmd=[
            "/usr/bin/python3", os.path.join(nav_share, "tools", "resolve_usb_roles.py"),
            "--output-dir", "/tmp/agv_devices",
            "--require-imu", "false", "--require-lidar", "false",
            "--require-camera", LaunchConfiguration("enable_camera"),
            "--stable-seconds", "2.0",
            "--recover-missing", "false",
        ], output="screen")

    ready_gate_script = os.path.join(
        get_package_prefix("navigation"), "lib", "navigation",
        "serial_transport_ready_gate.py")

    # Exactly the same per-role handoff strategy as mapping_runtime.launch.py:
    # a healthy role is never held behind the other role. These gates do not
    # open/probe the tty; they only wait for a valid stable resolver alias.
    imu_transport_ready = ExecuteProcess(
        cmd=[
            "/usr/bin/python3", ready_gate_script,
            "--imu", LaunchConfiguration("imu_port"),
            "--lidar", LaunchConfiguration("lidar_port"),
            "--require-imu", "true", "--require-lidar", "false",
            "--stable-cycles", "1",
            "--poll-sec", "0.10",
            "--timeout-sec", "0",
        ], output="screen")
    lidar_transport_ready = ExecuteProcess(
        cmd=[
            "/usr/bin/python3", ready_gate_script,
            "--imu", LaunchConfiguration("imu_port"),
            "--lidar", LaunchConfiguration("lidar_port"),
            "--require-imu", "false", "--require-lidar", "true",
            "--stable-cycles", "1",
            "--poll-sec", "0.10",
            "--timeout-sec", "0",
        ], output="screen")

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "imu.launch.py")),
        launch_arguments={
            "port": LaunchConfiguration("imu_port"),
            "baudrate": "921600",
            "frame_id": "imu_link",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # /scan_nav serves two localization-only consumers: AMCL and local LiDAR odometry.
    # No occupancy map is generated or published by the LiDAR-odometry node.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "lidar.launch.py")),
        launch_arguments={
            "port": LaunchConfiguration("lidar_port"),
            "baudrate": LaunchConfiguration("lidar_baudrate"),
            "frame_id": "lidar_link",
            "intensity_mode": LaunchConfiguration("lidar_intensity"),
            "intensity_bits": LaunchConfiguration("lidar_intensity_bits"),
            "strict_checksum": LaunchConfiguration("lidar_strict_checksum"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Diagnostic LiDAR scan-matching odometry. It is deliberately excluded from
    # the production EKF to avoid double-counting the same LiDAR information used
    # by AMCL. It publishes /lidar/odom only, with NO map/TF ownership.
    lidar_odometry = Node(
        package="navigation",
        executable="hector_slam_node",
        name="hector_slam_node",
        output="screen",
        emulate_tty=True,
        parameters=[hector_params, {
            "publish_map_odom_tf": False,
            "publish_odom_tf": False,
            "publish_map_topic": False,
            "publish_pose_topic": False,
            "publish_odom_topic": True,
            "show_pose": False,
            "show_quality": True,
            "show_scan_count": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
        remappings=[("/odom", "/lidar/odom")],
    )

    # Match mapping_runtime.launch.py: both role gates are already waiting while
    # the resolver creates aliases. Each driver starts independently as soon as
    # its own transport is ready. Sensor publication is therefore not blocked by
    # AMCL, Nav2, camera, costmaps, or readiness of the other serial role.
    delayed_imu = TimerAction(period=0.10, actions=[imu])
    delayed_lidar = TimerAction(period=0.10, actions=[lidar])
    # Fail-open exactly as in the supplied master: drivers start after a short
    # delay independent of the transport gate and own their own reconnect path.
    start_imu_after_transport_ready = TimerAction(period=0.20, actions=[imu])
    start_lidar_after_transport_ready = TimerAction(period=0.25, actions=[lidar])

    # AUTHORITATIVE PRE-AMCL gate. It is started only after the real sensor
    # drivers have been handed off. It still checks actual messages, frame IDs,
    # LiDAR health and odom/TF before AMCL is allowed to activate.
    pre_amcl_gate = Node(
        package="navigation",
        executable="autonomous_pre_amcl_gate.py",
        name="autonomous_pre_amcl_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "imu_topic": "/imu/data",
            "scan_topic": "/scan_nav",
            "esc_odom_topic": "/esc/odom",
            "filtered_odom_topic": "/odometry/filtered",
            "lidar_health_topic": "/lidar/safety_healthy",
            "imu_frame": "imu_link",
            "scan_frame": "lidar_link",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "min_messages": 3,
            "freshness_sec": 2.0,
            "diagnostic_timeout_sec": 120.0,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # Start localization preflight only after both transport handoffs have
    # launched their drivers. The gate itself waits for real messages, so this
    # is not a blind timer-based readiness decision.
    sensor_topic_gate = Node(
        package="navigation",
        executable="allsystem_gate",
        name="autonomous_sensor_topic_ready_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "gate_name": "AUTONOMOUS-SENSORS",
            "topics": ["/imu/data", "/scan_nav"],
            "min_messages": 5,
            "timeout_ms": 60000,
            "exit_on_ready": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )
    # Same as mapping_runtime: the topic gate may wait from the beginning while
    # the sensor drivers come up independently. It gates only downstream
    # localization/perception; it never gates IMU/LiDAR publication.
    start_sensor_topic_gate = TimerAction(period=0.80, actions=[sensor_topic_gate])

    # V58 CRITICAL FIX — localization must never wait for camera/perception.
    # The former graph was:
    #   serial topics -> camera resolver -> camera -> post-camera sensor gate -> PRE-AMCL
    # so one missing/slow V4L2 device kept AMCL, both Nav2 costmaps, Smac and MPPI
    # permanently inactive.  PRE-AMCL already validates the *real* IMU, LiDAR,
    # ESC odom, EKF odom, LiDAR health and TF chain, therefore it is safe and
    # strictly more deterministic to start it directly after the canonical
    # sensor-topic gate.  Camera/perception is now an independent branch.
    start_pre_amcl_after_sensor_topics = _success_only_exit(
        sensor_topic_gate,
        [pre_amcl_gate, TimerAction(period=0.20, actions=[lidar_odometry])],
        "sensor-topics-ready-localization")


    # V59 PERCEPTION MASTER RESTORE — use the proven master camera launch
    # directly.  Do not route camera startup through perception_all composition
    # or persistent runtime YAML shadows; those paths could leave the GUI with
    # no perception topics even though the package built successfully.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_share, "launch", "camera.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        launch_arguments={
            "device": LaunchConfiguration("camera_device"),
            "width": LaunchConfiguration("camera_width"),
            "height": LaunchConfiguration("camera_height"),
            "fps": LaunchConfiguration("camera_fps"),
            "frame_id": "camera_color_optical_frame",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_gpu_decode": LaunchConfiguration("use_gpu_decode"),
            "require_gpu_decode": LaunchConfiguration("require_gpu_decode"),
        }.items(),
    )

    camera_gate = Node(
        package="navigation",
        executable="allsystem_gate",
        name="autonomous_camera_ready_gate",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        parameters=[{
            "gate_name": "autonomous_camera",
            "topics": ["/camera/color/image_raw"],
            "min_messages": 3,
            "timeout_ms": 120000,
            "exit_on_ready": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # ── Localization processes (kept UNCONFIGURED until PRE-AMCL READY) ─────
    # The AMCL process may exist after Stage 0, but its lifecycle manager is not
    # started until real scan/local odometry/static sensor TF are verified.
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[nav2_params, {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
            "global_frame_id": "map",
            "odom_frame_id": "odom",
            "base_frame_id": "base_footprint",
            "scan_topic": "/scan_nav",
            "map_topic": "/map",
            "tf_broadcast": True,
            "transform_tolerance": 1.0,
            # Fail-safe PART-5 initialization: AMCL waits for /initialpose from
            # the operator, unless localization_startup.yaml explicitly selects
            # a map-hash-bound KNOWN_POSE handled by the readiness gate.
            "set_initial_pose": False,
            "always_reset_initial_pose": False,
        }]
    )

    # V16 starts saved-map servers immediately after preflight, independently of
    # hardware readiness. This guarantees RViz can open and show /map even while
    # a temporarily unavailable USB sensor is still being recovered.
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_maps",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    # The derived planning-map publisher is diagnostic/compatibility-only in
    # V35.  Give it an independent lifecycle manager so a bad optional derived
    # file can never roll back or deactivate the canonical /map used by AMCL
    # and the production global costmap.
    nav_map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planning_map",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "autostart": True,
            "node_names": ["nav_map_server"],
        }],
    )

    localization_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "autostart": True,
            # Match the Nav2 planner/controller managers: under temporary Jetson
            # load spikes a 4 s default bond timeout falsely declared healthy
            # AMCL dead and deactivated it, leaving map->odom permanently absent.
            "bond_timeout": 10.0,
            "attempt_respawn_reconnection": True,
            "bond_respawn_max_duration": 20.0,
            "node_names": ["amcl"],
        }],
    )

    # Authoritative AMCL readiness gate. Nav2 will NOT activate from a mere
    # process existence or /amcl_pose alone: the real map->odom and map->base TF
    # must also exist. This directly protects both global and local costmaps.
    localization_gate = Node(
        package="navigation",
        executable="autonomous_amcl_ready_gate.py",
        name="autonomous_amcl_ready_gate",
        output="screen",
        emulate_tty=True,
        parameters=[localization_startup_params, {
            "map_topic": "/map",
            "amcl_pose_topic": "/amcl_pose",
            "amcl_state_service": "/amcl/get_state",
            "operator_initial_pose_topic": "/initialpose_safe",
            "global_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "current_map_sha256": LaunchConfiguration("resolved_map_sha256"),
            "initial_pose_mode": LaunchConfiguration("resolved_initial_pose_mode"),
            "known_pose_x": ParameterValue(LaunchConfiguration("resolved_initial_pose_x"), value_type=float),
            "known_pose_y": ParameterValue(LaunchConfiguration("resolved_initial_pose_y"), value_type=float),
            "known_pose_yaw": ParameterValue(LaunchConfiguration("resolved_initial_pose_yaw"), value_type=float),
            "known_pose_map_sha256": LaunchConfiguration("resolved_initial_pose_map_sha256"),
            "require_map_hash_for_known_pose": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # V35 planning prerequisite.  Global costmap activation needs an ACTIVE
    # AMCL instance and a real map->odom->base transform, but it must not wait
    # for the stricter covariance/stability threshold used to permit motion.
    # Keeping those two decisions separate prevents a localized robot from
    # remaining forever at "global costmap: node up / waiting for topic".
    map_tf_ready_gate = Node(
        package="navigation",
        executable="autonomous_map_tf_ready_gate.py",
        name="autonomous_map_tf_ready_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "map_topic": "/map",
            "amcl_state_service": "/amcl/get_state",
            "global_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "stable_cycles": 3,
            "poll_period_sec": 0.20,
            "diagnostic_timeout_sec": 30.0,
            "fail_on_timeout": False,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # ── COMPLETE Nav2 autonomous stack ────────────────────────────────────────
    # PlannerServer owns the GLOBAL COSTMAP. ControllerServer owns the LOCAL
    # COSTMAP. Local uses live /scan obstacle + inflation. Global uses the
    # lifecycle-proven /map StaticLayer + a deliberately compact inflation band.
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        respawn=True,
        respawn_delay=2.0,
    )
    # Canonical Nav2 ControllerServer.  Do not wrap it in the ESC package:
    # the lifecycle manager must see /controller_server/get_state immediately.
    # Ackermann behavior remains in the MPPI plugin configuration.
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
        respawn=True,
        respawn_delay=2.0,
    )
    behavior = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen", parameters=[nav2_params],
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
    )
    navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[nav2_params, {
            "default_nav_to_pose_bt_xml": bt_xml,
            # Humble still instantiates NavigateThroughPoses internally even
            # when this AGV only exposes NavigateToPose. Point its unused
            # default tree at the same Ackermann-safe XML so configure() does
            # not require a compute_path_through_poses action server.
            "default_nav_through_poses_bt_xml": bt_xml,
        }],
        # Humble BtNavigator also subscribes directly to relative topic
        # "goal_pose". Remap that private convenience input away from RViz so
        # /goal_pose has exactly ONE owner: goal_pose_nav2_bridge. The bridge
        # validates the goal then sends NavigateToPose; BT Navigator plans once.
        remappings=[
            ("cmd_vel", "/cmd_vel_nav_raw"),
            ("goal_pose", "/goal_pose_nav2_internal"),
        ]
    )
    smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        name="velocity_smoother", output="screen", parameters=[nav2_params],
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        remappings=[
            ("cmd_vel", "/cmd_vel_nav_raw"),
            ("cmd_vel_smoothed", "/cmd_vel_nav_smoothed"),
        ]
    )
    collision = Node(
        package="nav2_collision_monitor", executable="collision_monitor",
        name="collision_monitor", output="screen", parameters=[collision_params],
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
    )

    # PART-1 final motion safety gate. Motion requires fresh LiDAR + IMU and
    # /esc/ready=true; any failure immediately forces zero velocity.
    # RViz initial-pose relay prevents AMCL future-extrapolation warnings when
    # odom->base TF trails wall-clock time by a few tens of milliseconds.
    initialpose_relay = Node(
        package="navigation",
        executable="initialpose_stamp_relay.py",
        name="initialpose_stamp_relay",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            "input_topic": "/initialpose_safe",
            "output_topic": "/initialpose",
            "amcl_state_service": "/amcl/get_state",
            "backdate_sec": 0.15,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )

    localization_timing_monitor = Node(
        package="navigation",
        executable="localization_timing_monitor.py",
        name="localization_timing_monitor",
        output="screen",
        parameters=[localization_timing_params, {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )

    autonomy_health_manager = Node(
        package="navigation",
        executable="autonomy_health_manager.py",
        name="autonomy_health_manager",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[autonomy_health_params, {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            # Static calibration interlock.  The node still starts and reports
            # every other prerequisite, but /system/autonomy_motion_allowed can
            # never become true while physical geometry validation is pending.
            "geometry_validated": geometry_motion_valid,
        }],
    )

    sensor_cmd_guard = Node(
        package="navigation",
        executable="autonomous_sensor_cmd_guard.py",
        name="autonomous_sensor_cmd_guard",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=1.0,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "input_cmd_topic": "/cmd_vel_collision_safe",
            "output_cmd_topic": "/cmd_vel",
            "scan_topic": "/scan_safety",
            "lidar_health_topic": "/lidar/safety_healthy",
            "imu_topic": "/imu/data",
            "esc_ready_topic": "/esc/ready",
            "require_esc_ready": True,
            "scan_timeout_sec": 0.40,
            "lidar_health_timeout_sec": 0.55,
            "imu_timeout_sec": 0.40,
            "esc_ready_timeout_sec": 0.50,
            "cmd_timeout_sec": 0.30,
            "stable_recovery_sec": 0.5,
            "publish_rate_hz": 20.0,
        }],
    )

    # Manual teleop interlock is required by the ESC package even during an
    # autonomous session.  Starting it here prevents the GUI from reporting a
    # missing manual-motion health topic and keeps the teleop path fail-closed.
    manual_motion_health = Node(
        package="navigation",
        executable="manual_motion_health.py",
        name="manual_motion_health",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        parameters=[manual_motion_health_params, {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )

    # PART-1 single-authority Goal Pose pipeline:
    #   RViz /goal_pose -> C++ bridge -> ComputePathToPose/Smac Hybrid-A*
    #   -> /smac_plan preview -> NavigateToPose -> BT Navigator -> MPPI.
    # The preview uses the same PlannerServer/GridBased plugin as navigation,
    # so a valid goal always gets a visible path before motion is attempted.
    goal_pose_bridge = Node(
        package="navigation",
        executable="goal_pose_nav2_bridge",
        name="goal_pose_nav2_bridge",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=1.0,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "goal_topic": "/goal_pose",
            "navigate_action_name": "/navigate_to_pose",
            "compute_path_action_name": "/compute_path_to_pose",
            "planner_id": "GridBased",
            "use_explicit_amcl_start": False,
            "amcl_pose_topic": "/amcl_pose",
            "amcl_max_age_sec": 5.0,
            "initial_pose_topic": "/initialpose_safe",
            "initial_pose_max_age_sec": 60.0,
            "snap_goal_to_free": True,
            "snap_radius_m": 0.80,
            "footprint_half_length": 0.65,
            "footprint_half_width": 0.40,
            "max_plan_retries": 20,
            "plan_retry_period_sec": 1.0,
            "plan_request_timeout_sec": 10.0,
            "nav_goal_response_timeout_sec": 5.0,
            "primary_behavior_tree": bt_xml,
            "plan_source_topic": "/plan",
            "plan_topic": "/smac_plan",
            # Validate against the same canonical saved map that feeds the
            # production global costmap.  This removes the previous split-brain
            # dependency on an optional derived /nav_map publisher.
            "nav_map_topic": "/map",
            "default_frame": "map",
            "reject_unknown_goal": True,
            "goal_free_threshold": 20,
        }],
    )

    # V9 GLOBAL-INFLATION-FIRST:
    # The planner lifecycle service is the FIRST Nav2 service gate because
    # PlannerServer owns the global costmap. ControllerServer is deliberately
    # excluded from this first milestone so a controller issue can never block
    # static-map + global-inflation visualization.
    nav2_global_service_gate = Node(
        package="navigation",
        executable="nav2_core_service_gate.py",
        name="nav2_global_service_gate",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "required_services": ["/planner_server/get_state"],
            "poll_period_sec": 0.20,
            "stable_cycles": 3,
        }],
    )

    # Controller/local-costmap service is checked only AFTER global inflation
    # has been proven live and RViz is allowed to open.
    nav2_local_service_gate = Node(
        package="navigation",
        executable="nav2_core_service_gate.py",
        name="nav2_local_service_gate",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "required_services": ["/controller_server/get_state"],
            "poll_period_sec": 0.20,
            "stable_cycles": 3,
        }],
    )

    # V8: Global and local costmap owners are lifecycle-managed independently.
    # A controller issue can no longer prevent PlannerServer / global inflation
    # from activating, and vice versa.
    lifecycle_navigation_global = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation_global",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "autostart": True,
            "bond_timeout": 10.0,
            "attempt_respawn_reconnection": True,
            "bond_respawn_max_duration": 20.0,
            "node_names": ["planner_server"],
        }],
    )

    lifecycle_navigation_local = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation_local",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "autostart": True,
            "bond_timeout": 10.0,
            "attempt_respawn_reconnection": True,
            "bond_respawn_max_duration": 20.0,
            "node_names": ["controller_server"],
        }],
    )

    lifecycle_navigation_aux = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation_aux",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "autostart": True,
            "bond_timeout": 10.0,
            "attempt_respawn_reconnection": True,
            "bond_respawn_max_duration": 20.0,
            "node_names": [
                "behavior_server",
                "bt_navigator",
                "velocity_smoother",
                "collision_monitor",
            ],
        }],
    )

    # PlannerServer process is created directly by the V60 foundation. Its
    # lifecycle activation remains gated only by the non-terminal map-TF gate.
    # This separates process availability from localization readiness.

    # Map-frame milestone. This gate does not care about controller_server.
    global_costmap_ready_gate = Node(
        package="navigation",
        executable="allsystem_gate",
        name="autonomous_global_costmap_ready_gate",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "gate_name": "GLOBAL-INFLATION-FIRST",
            "topics": ["/global_costmap/costmap"],
            "min_messages": 1,
            "timeout_ms": 60000,
            "exit_on_ready": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # Odom-frame milestone: local costmap / inflation owned by ControllerServer.
    local_costmap_ready_gate = Node(
        package="navigation",
        executable="allsystem_gate",
        name="autonomous_local_costmap_ready_gate",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "gate_name": "LOCAL-INFLATION",
            "topics": ["/local_costmap/costmap"],
            "min_messages": 1,
            "timeout_ms": 60000,
            "exit_on_ready": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # Final join gate. OccupancyGrid publishers are transient-local, so starting
    # this after the global milestone deterministically verifies that both
    # independently managed costmaps are available before auxiliary Nav2 nodes
    # are activated.
    both_costmaps_ready_gate = Node(
        package="navigation",
        executable="allsystem_gate",
        name="autonomous_both_costmaps_ready_gate",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{
            "gate_name": "BOTH-COSTMAPS",
            "topics": ["/global_costmap/costmap", "/local_costmap/costmap"],
            "min_messages": 1,
            "timeout_ms": 60000,
            "exit_on_ready": True,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    # PRE-AMCL READY -> activate ControllerServer/local costmap immediately.
    # The local costmap uses odom, so delaying it for map->odom was unnecessary
    # and made the GUI look broken while AMCL was initializing.
    # V60: PRE-AMCL is diagnostic/motion-safety evidence, not an availability
    # dependency for Nav2 lifecycle ownership. ControllerServer is activated by
    # exactly one lifecycle manager from the foundation; these handlers only
    # report readiness and therefore can never permanently suppress the local
    # costmap after one transient sensor-gate timeout.
    start_local_service_gate_after_pre_amcl = _success_only_exit(
        pre_amcl_gate,
        [LogInfo(msg=(
            "[V60-NAV2] PRE-AMCL diagnostics READY; local Nav2 activation is already managed independently."
        ))],
        "pre-amcl-ready-local-costmap")

    start_local_costmap_after_controller_service = _success_only_exit(
        nav2_local_service_gate,
        [LogInfo(msg=(
            "[V60-NAV2] Controller lifecycle service READY; lifecycle manager owns activation/recovery."
        ))],
        "controller-service-ready")

    local_costmap_ready_notice = _success_only_exit(
        local_costmap_ready_gate, [LogInfo(msg=(
            "[LOCAL-INFLATION] READY: /scan_nav + odom local costmap is ACTIVE; "
            "waiting for AMCL only affects the map-frame global costmap."
        ))], "local-costmap-ready")

    # Real map TF READY -> ask for PlannerServer/global-costmap lifecycle
    # service.  Motion confidence remains independently fail-closed in the
    # authoritative AMCL convergence gate and autonomy health manager.
    start_global_service_gate_after_map_tf = _success_only_exit(
        map_tf_ready_gate,
        [TimerAction(period=0.15, actions=[nav2_global_service_gate])],
        "map-tf-ready-global-costmap")

    localization_converged_notice = _success_only_exit(
        localization_gate,
        [LogInfo(msg=(
            "[AMCL-CONVERGENCE] READY: covariance and pose stability passed; "
            "continuous autonomy health checks remain authoritative for motion."
        ))],
        "amcl-localization-converged")

    # Planner service exists -> activate ONLY PlannerServer/global costmap.
    start_global_costmap_after_planner_service = _success_only_exit(
        nav2_global_service_gate,
        [lifecycle_navigation_global, TimerAction(period=0.50, actions=[global_costmap_ready_gate])],
        "planner-service-ready")

    start_both_costmaps_gate_after_global_inflation = _success_only_exit(
        global_costmap_ready_gate, [
                LogInfo(msg=(
                    "[GLOBAL-INFLATION-FIRST] READY: planning /nav_map + global "
                    "inflation are publishing; Smac Hybrid-A* is ACTIVE."
                )),
                both_costmaps_ready_gate,
            ], "global-costmap-ready")

    # Both independent costmap owners have published. Activate the remaining
    # Nav2 servers only after this explicit join, never from timing assumptions.
    start_navigation_aux_after_both_costmaps = _success_only_exit(
        both_costmaps_ready_gate, [
                LogInfo(msg=(
                    "[SMAC-HYBRID-ASTAR] READY: planner=GridBased "
                    "type=nav2_smac_planner/SmacPlannerHybrid topic=/smac_plan"
                )),
                lifecycle_navigation_aux,
                LogInfo(msg=(
                    "[NAV2-AUX] START: both costmaps verified; activating BT Navigator, "
                    "behaviors, velocity smoother and collision monitor exactly once."
                )),
            ], "both-costmaps-ready")

    # V59 PERCEPTION MASTER RESTORE — standalone camera / YOLO / alignment.
    # This mirrors the uploaded master source: each process has independent
    # ownership and respawn, and no component container is required.
    yolo_detection_cfg = _runtime_config(yolo_share, "yolo_obstacle_detection_ros2", "yolo_detection.yaml")
    yolo = Node(
        package="yolo_obstacle_detection_ros2",
        executable="obstacle_detector_node",
        name="obstacle_detector_node",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("enable_yolo")),
        parameters=[yolo_detection_cfg, {
            "model_path": LaunchConfiguration("resolved_yolo_model"),
            "engine_path": LaunchConfiguration("yolo_engine"),
            "require_cuda": ParameterValue(
                LaunchConfiguration("yolo_require_cuda"), value_type=bool
            ),
            "input_topic": "/camera/color/image_raw",
            "output_topic": "/obstacle_detection/obstacles",
            "visualization_topic": "/obstacle_detection/visualization",
        }],
    )

    warehouse_person = Node(
        package="yolo_obstacle_detection_ros2",
        executable="warehouse_person_detector_node",
        name="warehouse_person_detector_node",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("enable_warehouse_person")),
        parameters=[{
            "model_path": "/home/otomasi2/ros/models/yolov8n.onnx",
            "engine_path": "/home/otomasi2/ros/models/yolov8n_fp16_640.engine",
            "use_tensorrt": True,
            "use_cuda": True,
            "require_cuda": True,
            "use_gpu_preprocess": True,
            "allow_passthrough_without_model": False,
            "input_topic": "/camera/color/image_raw",
            "output_topic": "/warehouse_obstacle/persons",
            "visualization_topic": "/warehouse_obstacle/visualization",
            "status_topic": "/warehouse_obstacle/status",
            "performance_topic": "/warehouse_obstacle/performance",
            "confidence_threshold": 0.40,
            "iou_threshold": 0.45,
            "max_inference_fps": 2.0,
            "max_visualization_fps": 1.0,
        }],
    )

    _runtime_root = os.environ.get(
        "AGV_RUNTIME_CONFIG_ROOT", os.path.join(os.environ.get("AGV_WS", "/home/otomasi2/ros"), "config", "runtime"))
    hole_alignment_cfg = os.path.join(
        _runtime_root, "yolo_obstacle_detection_ros2", "alignment_realtime.yaml")
    if not os.path.isfile(hole_alignment_cfg):
        hole_alignment_cfg = os.path.join(
            yolo_share, "hole_block_alignment", "alignment_realtime.yaml")
    hole_alignment = Node(
        package="yolo_obstacle_detection_ros2",
        executable="hole_block_alignment_node.py",
        name="hole_block_alignment_node",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("enable_hole_alignment")),
        parameters=[{
            "config": hole_alignment_cfg,
            "image_topic": "/camera/color/image_raw",
            "obstacle_topic": "/obstacle_detection/obstacles",
            "state_topic": "/fork_alignment/state",
            "output_image_topic": "/fork_alignment/image",
            "save_csv": False,
            "save_output_video": False,
            "visualization_fps": 8.0,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )

    # V60: perception startup is bounded-timer based, not exit-code gated.
    # perception_preflight performs its kill phase before udev settle; starting
    # the master camera branch after 0.80 s avoids stale-owner races while an
    # abnormal cleanup return can no longer suppress camera/YOLO/fork forever.
    start_perception_after_cleanup = TimerAction(period=0.80, actions=[
        LogInfo(msg="[PERCEPTION-V60] independent master topology: camera -> YOLO -> fork alignment"),
        camera,
        TimerAction(period=0.10, actions=[camera_gate]),
        TimerAction(period=0.40, actions=[yolo]),
        TimerAction(period=0.55, actions=[warehouse_person]),
        TimerAction(period=0.70, actions=[hole_alignment]),
    ])

    # Camera readiness is diagnostic only. YOLO and hole alignment are started
    # independently below and safely wait for image data. This prevents a camera
    # readiness-gate timeout/race from leaving /fork_alignment/image permanently absent.
    camera_ready_notice = RegisterEventHandler(
        OnProcessExit(
            target_action=camera_gate,
            on_exit=[LogInfo(msg=(
                "[CAMERA-VISUALIZATION] raw camera READY; perception subscribers are active."
            ))],
        )
    )

    # Stage 0A -> 0B: first remove stale camera/YOLO owners, then clean stale
    # serial/Nav2 ownership. This ordering prevents an orphan UVC process from
    # making the GUI USB resolver PARTIAL during CP210x enumeration.
    start_sensor_preflight_after_perception_cleanup = _success_only_exit(
        perception_preflight,
        [sensor_preflight],
        "perception-stale-owner-cleanup")

    # MASTER SENSOR STARTUP: keep navigation/perception independent and do
    # not gate the navigation foundation behind CP210x preflight.  The supplied
    # master starts the foundation on a bounded timer so IMU/LiDAR drivers can
    # own reconnect/re-enumeration themselves without a destructive pre-open stage.
    foundation_actions = [
                robot_state,
                visual_robot_state,
                visual_base_bridge,
                joint_state,
                # RVIZ-ONLY FIX: start RViz directly and independently.
                # Do not gate the window on RobotModel/TF readiness; RViz must
                # always open during autonomous bring-up. All navigation, AMCL,
                # Smac, MPPI, perception, ESC, and safety behavior is unchanged.
                web_gui,
                mapping_controller,
                nav_map_switch_controller,
                esc,
                ekf,
                localization_timing_monitor,
                # Saved map + RobotModel + RViz are independent of serial sensors.
                # This makes the GUI deterministic even if hardware is still
                # settling; /map is lifecycle-activated immediately below.
                map_server,
                nav_map_server,
                initialpose_relay,
                # Subscribe to /goal_pose immediately. Goals submitted while
                # Nav2 lifecycle nodes are still activating are retained by the
                # bridge and sent as soon as /navigate_to_pose becomes ready.
                goal_pose_bridge,
                # Continuous autonomy health starts fail-closed before Nav2. It
                # publishes FALSE until localization, TF, lifecycle, costmaps,
                # sensor guard and ESC feedback are all continuously healthy.
                autonomy_health_manager,
                # Sensor guards / manual teleop health start immediately and
                # remain fail-closed until their real inputs are healthy.
                sensor_cmd_guard,
                manual_motion_health,
                TimerAction(period=0.25, actions=[map_lifecycle]),
                TimerAction(period=0.35, actions=[nav_map_lifecycle]),
                # RViz has its own bounded RobotModel gate. It opens even if
                # map->odom still awaits Initial Pose, so diagnostics never disappear.
                amcl,
                planner,
                controller,
                behavior,
                navigator,
                smoother,
                collision,
                # AMCL and the local controller may activate independently.
                # Auxiliary Nav2 servers are intentionally released only after
                # both costmaps are publishing; this prevents BtNavigator from
                # configuring before PlannerServer exposes compute_path_to_pose.
                # Motion remains fail-closed through the existing health guards.
                TimerAction(period=0.80, actions=[localization_lifecycle]),
                TimerAction(period=1.00, actions=[lifecycle_navigation_local]),
                # Start long-lived diagnostics independently. map_tf_ready_gate
                # is non-terminal and will eventually release PlannerServer as
                # soon as an Initial Pose makes map->odom valid.
                TimerAction(period=1.35, actions=[nav2_local_service_gate]),
                TimerAction(period=1.50, actions=[localization_gate]),
                TimerAction(period=1.60, actions=[map_tf_ready_gate]),
                TimerAction(period=2.00, actions=[local_costmap_ready_gate]),
                # Perception is intentionally NOT started here. V33 gives it a
                # separate ownership preflight so serial recovery can never block images.
                # Real hardware path: resolver -> transport -> driver.
                # No fixed-delay sensor start is used.
                # Mapping-style serial startup: resolver and both per-role
                # transport gates start together. The real drivers are launched
                # by their own gate success handlers, so neither sensor is held
                # behind the other.
                role_resolver_serial,
                imu_transport_ready,
                lidar_transport_ready,
                start_sensor_topic_gate,
            ]

    start_foundation_after_preflight = TimerAction(
        period=0.30, actions=foundation_actions)

    # V58 intentionally has no camera-resolver dependency here.  Perception is
    # independently started by start_perception_after_cleanup and the V4L2 node
    # owns camera discovery/reconnect.  This keeps AMCL/Nav2 and perception from
    # being able to deadlock each other.

    # STAGE 1 -> STAGE 2. Arm the AMCL-ready subscriber BEFORE activation.
    # AMCL Humble publishes /amcl_pose with TRANSIENT_LOCAL durability and may
    # publish its first pose immediately after activation. Starting this gate
    # first removes the race where a stationary robot produced one pose before
    # the old VOLATILE gate subscribed, blocking Nav2 forever.
    start_localization_after_pre_amcl = _success_only_exit(
        pre_amcl_gate,
        [LogInfo(msg=(
            "[V60-LOCALIZATION] PRE-AMCL diagnostics READY; AMCL lifecycle, convergence "
            "monitor and map-TF gate are already running independently."
        ))],
        "pre-amcl-ready")

    return LaunchDescription(args + [
        # Launch-time validation only; no ROS child process yet.
        verify_dependencies,
        verify_camera_gpu,
        resolve_yolo_model,
        verify_yolo_runtime,
        resolve_map,
        localization_mode_log,
        urdf_mesh_log,
        geometry_log,

        # Register every stage transition BEFORE starting Stage 0.
        # Web GUI is the owner of the Web-GUI-started stack. If it exits, this
        # handler emits launch-wide Shutdown and all sensor children are stopped.
        web_gui_exit_shutdown,
        start_foundation_after_preflight,
        # Register mapping-style per-role serial handoff handlers before Stage 0.
        start_imu_after_transport_ready,
        start_lidar_after_transport_ready,
        start_pre_amcl_after_sensor_topics,
        start_local_service_gate_after_pre_amcl,
        start_local_costmap_after_controller_service,
        local_costmap_ready_notice,
        start_localization_after_pre_amcl,
        start_global_service_gate_after_map_tf,
        localization_converged_notice,
        start_global_costmap_after_planner_service,
        start_both_costmaps_gate_after_global_inflation,
        start_navigation_aux_after_both_costmaps,
        camera_ready_notice,
        # RViz is a diagnostics UI and must be visible even while serial
        # recovery / stale-owner cleanup is still in progress. It has its own
        # desktop environment fallback, so SSH/VSCode/remote shells without
        # DISPLAY cannot suppress the window.
        TimerAction(period=0.50, actions=[rviz]),

        # Perception remains independent and cannot gate localization/planning.
        start_perception_after_cleanup,

        # Match the supplied master: sensor_preflight is intentionally not a
        # direct root.  IMU/LiDAR start independently and recover through their
        # own drivers/resolver. Perception keeps its independent cleanup root.
        perception_preflight,
    ])
