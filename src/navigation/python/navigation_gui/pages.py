#!/usr/bin/env python3
"""Page implementations for the Autonomous Vehicle Interface."""
from __future__ import annotations

import csv
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPixmap
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .experiment_manager import ExperimentManager
from .map_canvas import MapCanvas
from .map_model import MapDocument, MapLoader, MapRepository, MapSource, compare_maps
from .widgets import (
    NoWheelDoubleSpinBox,
    RingPlot,
    ScrollSettings,
    SectionFrame,
    StatusPill,
    TopicTable,
    YamlParameterEditor,
)
from .yaml_store import YamlStore


@dataclass
class PageBundle:
    name: str
    icon_text: str
    settings: QWidget
    content: QWidget
    group: str = ""


class ConnectionPage:
    """System overview driven by both topic freshness and the live ROS graph."""

    DEFAULT_TOPICS = [
        "/scan_nav", "/scan_safety", "/lidar/safety_healthy", "/lidar/safety_health", "/lidar/status",
        "/imu/data", "/imu/gyro", "/imu/accel", "/imu/mag", "/imu/euler", "/imu/status",
        "/lidar/odom", "/scan_match_quality", "/odometry/filtered",
        "/map", "/nav_map", "/amcl_pose",
        "/global_costmap/costmap", "/local_costmap/costmap",
        "/smac_plan", "/navigation/planner_status", "/transformed_global_plan", "/trajectories",
        "/goal_pose", "/initialpose_safe", "/navigate_to_pose/_action/status",
        "/cmd_vel_nav_raw", "/cmd_vel_nav_smoothed", "/cmd_vel_collision_safe", "/cmd_vel", "/cmd_vel/actuator",
        "/sensor_guard/healthy", "/system/autonomy_motion_allowed", "/system/manual_motion_allowed", "/safety/estop",
        "/mapping/map_valid", "/mapping/map_stats",
        "/camera/color/image_raw", "/camera/color/camera_info", "/camera/color/status",
        "/obstacle_detection/obstacles", "/obstacle_detection/visualization", "/obstacle_detection/status",
        "/fork_alignment/state", "/fork_alignment/image",
        "/winch/connected", "/winch/port", "/winch/state", "/winch/top_limit", "/winch/bottom_limit",
        "/winch/pwm_pct", "/winch/direction", "/winch/servo_deg", "/winch/raw",
        "/esc/odom", "/esc/speed", "/esc/ready", "/esc/armed", "/esc/feedback_valid",
        "/esc/drive/connected", "/esc/steer/connected", "/esc/status",
        "/esc/drive_target_mps", "/esc/drive_actual_mps", "/esc/steering_target_rad",
        "/esc/steering_actual_rad", "/esc/yaw_rate_actual_rps", "/esc/battery", "/esc/temperature",
        "/esc/mux/status", "/esc/mux/active_source", "/esc/mux/selected",
    ]

    SYSTEM_DEFS = {
        "ROS 2": {"nodes": ["autonomous_vehicle_interface"]},
        "USB Resolver": {"local": True},
        "LiDAR": {"nodes": ["lidar_node"], "topics": ["/scan_nav", "/lidar/safety_healthy"]},
        "LiDAR Odometry": {"nodes": ["hector_slam_node"], "topics": ["/lidar/odom"]},
        "IMU": {"nodes": ["imu_node"], "topics": ["/imu/data"]},
        "EKF": {"nodes": ["ekf_filter_node"], "topics": ["/odometry/filtered"]},
        "Map Server": {"nodes": ["map_server"], "topics": ["/map"]},
        "AMCL": {"nodes": ["/amcl", "amcl"], "topics": ["/amcl_pose"]},
        "Global Costmap": {"nodes": ["planner_server"], "topics": ["/global_costmap/costmap"]},
        "Smac Planner": {"nodes": ["planner_server"]},
        "Local Costmap": {"nodes": ["controller_server"], "topics": ["/local_costmap/costmap"]},
        "MPPI Controller": {"nodes": ["controller_server"]},
        "Velocity Smoother": {"nodes": ["velocity_smoother"]},
        "Collision Monitor": {"nodes": ["collision_monitor"]},
        "Sensor Guard": {"nodes": ["autonomous_sensor_cmd_guard"], "topics": ["/sensor_guard/healthy"]},
        "Autonomy Interlock": {"nodes": ["autonomy_health_manager"], "topics": ["/system/autonomy_motion_allowed"]},
        "Manual Motion Interlock": {"nodes": ["manual_motion_health"], "topics": ["/system/manual_motion_allowed"]},
        "ESC": {"nodes": ["esc_driver"], "topics": ["/esc/feedback_valid", "/esc/ready"]},
        "Camera": {"nodes": ["astra_rgb_v4l2_node"], "topics": ["/camera/color/image_raw"]},
        "YOLO": {"nodes": ["obstacle_detector_node"], "topics": ["/obstacle_detection/visualization"]},
        "Fork Alignment": {"nodes": ["hole_block_alignment_node"], "topics": ["/fork_alignment/state"]},
        "Winch": {"nodes": ["winch_serial_node"], "topics": ["/winch/connected", "/winch/state"]},
        "TF": {"tf": True},
    }

    def __init__(self, map_repo: MapRepository):
        self.map_repo = map_repo
        self.ros_online = False
        self.latest_health: Dict[str, Dict[str, Any]] = {}
        self.latest_nodes: List[str] = []
        self.tf_payload: Dict[str, Dict[str, Any]] = {}

        self.settings = ScrollSettings()
        section = SectionFrame("Connection")
        self.ros_pill = StatusPill("unknown")
        self.refresh_maps_btn = QPushButton("Refresh Mapping Results")
        section.addWidget(self.ros_pill)
        section.addWidget(self.refresh_maps_btn)
        self.settings.insert_widget(section)

        map_section = SectionFrame("Map Slots")
        self.map_labels: Dict[int, QLabel] = {}
        for i in range(1, 4):
            label = QLabel(f"Map {i}: checking…")
            label.setWordWrap(True)
            self.map_labels[i] = label
            map_section.addWidget(label)
        self.active_map_label = QLabel("Active navigation map: -")
        self.active_map_label.setWordWrap(True)
        map_section.addWidget(self.active_map_label)
        self.settings.insert_widget(map_section)

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("Connection & System Overview")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)

        cards_widget = QWidget()
        cards = QGridLayout(cards_widget)
        cards.setContentsMargins(0, 0, 0, 0)
        cards.setHorizontalSpacing(7)
        cards.setVerticalSpacing(7)
        self.system_cards: Dict[str, Tuple[QLabel, StatusPill]] = {}
        for idx, name in enumerate(self.SYSTEM_DEFS):
            frame = SectionFrame(name)
            pill = StatusPill("unknown")
            value = QLabel("Starting…")
            value.setAlignment(Qt.AlignCenter)
            value.setWordWrap(True)
            frame.addWidget(pill)
            frame.addWidget(value)
            cards.addWidget(frame, idx // 5, idx % 5)
            self.system_cards[name] = (value, pill)
        root.addWidget(cards_widget)

        tabs = QTabWidget()
        self.topic_table = TopicTable(self.DEFAULT_TOPICS)
        tabs.addTab(self.topic_table, "Topics")
        self.tf_table = QTableWidget(0, 6)
        self.tf_table.setHorizontalHeaderLabels(["Transform", "State", "X", "Y", "Z", "Yaw [deg]"])
        self.tf_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.tf_table, "TF")
        root.addWidget(tabs, 1)

        self.refresh_maps_btn.clicked.connect(self.refresh_maps)
        self.refresh_maps()
        self._refresh_cards()

    def set_ros_state(self, online: bool):
        self.ros_online = bool(online)
        self.ros_pill.set_state("healthy" if online else "disabled", "ROS ONLINE" if online else "ROS OFFLINE")
        self._refresh_cards()

    def refresh_maps(self):
        self.map_repo.refresh_sources()
        for source in self.map_repo.sources:
            state = "FOUND" if source.yaml_path.is_file() else "NOT FOUND"
            self.map_labels[source.slot].setText(f"Map {source.slot}: {state}\n{source.yaml_path}")
        active = self.map_repo.active_navigation_yaml()
        slot = self.map_repo.active_slot()
        self.active_map_label.setText(
            f"Active navigation map (pointer fallback): {'Map '+str(slot) if slot else 'OTHER / NONE'}\n{active or '-'}")
        self._refresh_cards()

    def set_active_map(self, path: str, slot: Optional[int]):
        self.active_map_label.setText(
            f"Active navigation map (map_server): {'Map '+str(slot) if slot else 'OTHER / EXTERNAL'}\n{path or '-'}")

    @staticmethod
    def _topic_state(info: Optional[Dict[str, Any]]) -> Tuple[str, str]:
        if not info or int(info.get("count", 0) or 0) <= 0:
            return "unknown", "NO DATA"
        age = float(info.get("age", float("inf")))
        hz = float(info.get("hz", 0.0) or 0.0)
        if age < 1.5:
            return "healthy", f"{hz:.1f} Hz | {age:.2f}s"
        if age < 4.0:
            return "degraded", f"STALE {age:.2f}s"
        return "error", f"STALE {age:.1f}s"

    def update_health(self, payload: Dict[str, Dict[str, Any]]):
        self.latest_health = payload
        self.topic_table.update_health(payload)
        self._refresh_cards()

    def update_system_state(self, payload: Dict[str, Any]):
        self.latest_nodes = [str(v) for v in payload.get("nodes", [])]
        self._refresh_cards()

    def _node_present(self, patterns: Sequence[str]) -> bool:
        if not patterns:
            return False
        low_nodes = [n.lower() for n in self.latest_nodes]
        return any(any(pattern.lower() in node for node in low_nodes) for pattern in patterns)

    @staticmethod
    def _usb_state() -> Tuple[str, str]:
        imu = Path("/tmp/agv_devices/imu")
        lidar = Path("/tmp/agv_devices/lidar")
        imu_ok = imu.exists()
        lidar_ok = lidar.exists()
        if imu_ok and lidar_ok:
            return "healthy", f"IMU {imu.resolve()}\nLiDAR {lidar.resolve()}"
        if imu_ok or lidar_ok:
            return "degraded", f"IMU {'OK' if imu_ok else 'MISSING'} | LiDAR {'OK' if lidar_ok else 'MISSING'}"
        return "error", "IMU/LiDAR aliases missing"

    def _refresh_cards(self):
        for name, definition in self.SYSTEM_DEFS.items():
            value, pill = self.system_cards[name]
            if name == "ROS 2":
                if self.ros_online:
                    pill.set_state("healthy", "ONLINE")
                    value.setText(f"graph nodes: {len(self.latest_nodes)}")
                else:
                    pill.set_state("disabled", "OFFLINE")
                    value.setText("ROS bridge not connected")
                continue
            if definition.get("local"):
                state, text = self._usb_state()
                pill.set_state(state)
                value.setText(text)
                continue
            if definition.get("tf"):
                required = ("map->odom", "odom->base_footprint")
                ok = all(bool(self.tf_payload.get(k, {}).get("available")) for k in required)
                if ok:
                    pill.set_state("healthy", "TF OK")
                    value.setText("map→odom→base")
                elif self.ros_online:
                    pill.set_state("degraded", "WAITING")
                    value.setText("TF chain incomplete")
                else:
                    pill.set_state("disabled", "OFFLINE")
                    value.setText("ROS offline")
                continue
            if not self.ros_online:
                pill.set_state("disabled", "OFFLINE")
                value.setText("ROS offline")
                continue

            node_ok = self._node_present(definition.get("nodes", []))

            # /map is transient-local/static by design; a stationary AMCL may
            # also stop emitting /amcl_pose even though localization remains
            # fully valid. Do not show false yellow/red cards for those cases.
            if name == "Map Server" and node_ok:
                info = self.latest_health.get("/map", {})
                if int(info.get("count", 0) or 0) > 0:
                    pill.set_state("healthy", "MAP READY")
                    value.setText("latched occupancy map available")
                else:
                    pill.set_state("degraded", "LOADING")
                    value.setText("waiting for first map")
                continue
            if name == "AMCL" and node_ok:
                map_odom_ok = bool(self.tf_payload.get("map->odom", {}).get("available"))
                pose_seen = int(self.latest_health.get("/amcl_pose", {}).get("count", 0) or 0) > 0
                if map_odom_ok:
                    pill.set_state("healthy", "LOCALIZED")
                    value.setText("map→odom TF valid")
                elif pose_seen:
                    pill.set_state("healthy", "POSE READY")
                    value.setText("AMCL pose received")
                else:
                    pill.set_state("degraded", "WAITING")
                    value.setText("waiting for initial localization")
                continue
            if name == "LiDAR" and node_ok:
                stream = self.latest_health.get("/scan_nav", {})
                stream_count = int(stream.get("count", 0) or 0)
                stream_age = float(stream.get("age", float("inf")))
                stream_hz = float(stream.get("hz", 0.0) or 0.0)
                status = str(self.latest_health.get("/lidar/status", {}).get("detail", "") or "")
                if stream_count > 0 and stream_age < 1.5:
                    pill.set_state("healthy")
                    value.setText(f"{stream_hz:.1f} Hz | {stream_age:.2f}s")
                elif stream_count > 0:
                    pill.set_state("degraded", "RECOVERING")
                    value.setText(status[:84] if status else f"scan stale {stream_age:.1f}s; watchdog active")
                else:
                    pill.set_state("degraded", "STARTING")
                    value.setText(status[:84] if status else "waiting for first quality scan")
                continue

            if name in ("Camera", "YOLO") and node_ok:
                stream_topic = "/camera/color/image_raw" if name == "Camera" else "/obstacle_detection/visualization"
                status_topic = "/camera/color/status" if name == "Camera" else "/obstacle_detection/status"
                stream = self.latest_health.get(stream_topic, {})
                status = self.latest_health.get(status_topic, {})
                stream_count = int(stream.get("count", 0) or 0)
                stream_age = float(stream.get("age", float("inf")))
                stream_hz = float(stream.get("hz", 0.0) or 0.0)
                backend = str(status.get("detail", "") or "")
                gpu_ok = ("CUDA" in backend.upper()) or ("TENSORRT" in backend.upper())
                passthrough = "PASSTHROUGH" in backend.upper()
                # Image subscriptions in the GUI are demand-driven in V29. A
                # hidden preview therefore has no local image count by design;
                # use the continuously subscribed backend status to avoid a
                # false NODE UP/degraded card while perception is healthy.
                if gpu_ok:
                    pill.set_state("healthy", "GPU READY")
                    if stream_count > 0 and stream_age < 5.0:
                        value.setText(f"{stream_hz:.1f} Hz | {backend[:58]}")
                    else:
                        value.setText(f"{backend[:68]} | preview demand-driven")
                elif passthrough:
                    pill.set_state("degraded", "PASSTHROUGH")
                    value.setText("camera stream available; YOLO model not loaded")
                elif stream_count > 0 and stream_age < 5.0:
                    pill.set_state("degraded", "STREAMING")
                    value.setText(f"{stream_hz:.1f} Hz | waiting for GPU backend status")
                elif backend:
                    pill.set_state("degraded", "WARMING")
                    value.setText(backend[:84])
                else:
                    pill.set_state("degraded", "WAITING DATA")
                    value.setText("waiting for backend status")
                continue

            if name == "Sensor Guard" and node_ok:
                info = self.latest_health.get("/sensor_guard/healthy", {})
                count = int(info.get("count", 0) or 0)
                age = float(info.get("age", float("inf")))
                detail = str(info.get("detail", "") or "").strip().lower()
                if count > 0 and age < 2.0:
                    ok = detail in ("true", "1", "yes", "on")
                    pill.set_state("healthy" if ok else "degraded", "HEALTHY" if ok else "HOLD")
                    value.setText("sensor inputs fresh" if ok else "sensor guard reports HOLD")
                else:
                    pill.set_state("degraded", "STARTING")
                    value.setText("waiting sensor guard state")
                continue

            if name in ("Autonomy Interlock", "Manual Motion Interlock") and node_ok:
                topic = definition.get("topics", [""])[0]
                info = self.latest_health.get(topic, {})
                count = int(info.get("count", 0) or 0)
                age = float(info.get("age", float("inf")))
                detail = str(info.get("detail", "") or "").strip().lower()
                if count > 0 and age < 2.0:
                    allowed = detail in ("true", "1", "yes", "on")
                    if allowed:
                        pill.set_state("healthy", "ALLOWED")
                        value.setText("motion gate open")
                    else:
                        pill.set_state("degraded", "HOLD")
                        value.setText("fail-closed until all checks pass")
                else:
                    pill.set_state("degraded", "WAITING")
                    value.setText("waiting for interlock state")
                continue

            if name == "Smac Planner" and node_ok:
                path = self.latest_health.get("/smac_plan", {})
                path_count = int(path.get("count", 0) or 0)
                path_age = float(path.get("age", float("inf")))
                planner = self.latest_health.get("/navigation/planner_status", {})
                detail = str(planner.get("detail", "") or "").strip()
                if path_count > 0 and path_age < 4.0:
                    pill.set_state("healthy", "PATH READY")
                    value.setText(f"{float(path.get('hz', 0.0) or 0.0):.1f} Hz | {path_age:.2f}s")
                else:
                    pill.set_state("healthy", "IDLE / READY")
                    value.setText(detail[:84] if detail else "planner ready; waiting Goal Pose")
                continue

            if name == "ESC" and node_ok:
                ready = self.latest_health.get("/esc/ready", {})
                count = int(ready.get("count", 0) or 0)
                age = float(ready.get("age", float("inf")))
                ready_text = str(ready.get("detail", "") or "").strip().lower()
                status = str(self.latest_health.get("/esc/status", {}).get("detail", "") or "")
                if count > 0 and age < 3.0:
                    is_ready = ready_text in ("true", "1", "yes", "on")
                    pill.set_state("healthy" if is_ready else "degraded", "READY" if is_ready else "NOT READY")
                    value.setText(status[:84] if status else ("ESC ready telemetry received" if is_ready else "ESC reports ready=false"))
                else:
                    pill.set_state("degraded", "CONNECTING")
                    value.setText("esc_driver active; waiting /esc/ready")
                continue

            if name == "Winch" and node_ok:
                info = self.latest_health.get("/winch/connected", {})
                count = int(info.get("count", 0) or 0)
                detail = str(info.get("detail", "") or "").strip().lower()
                port = str(self.latest_health.get("/winch/port", {}).get("detail", "") or "")
                state_text = str(self.latest_health.get("/winch/state", {}).get("detail", "") or "")
                if count > 0:
                    connected = detail in ("true", "1", "yes", "on")
                    pill.set_state("healthy" if connected else "degraded", "CONNECTED" if connected else "DISCONNECTED")
                    value.setText(f"{port or 'port unknown'} | {state_text or 'state unknown'}")
                else:
                    pill.set_state("degraded", "PROBING")
                    value.setText("winch node active; waiting /winch/connected")
                continue

            if name == "Fork Alignment" and node_ok:
                info = self.latest_health.get("/fork_alignment/state", {})
                count = int(info.get("count", 0) or 0)
                age = float(info.get("age", float("inf")))
                detail = str(info.get("detail", "") or "").strip()
                upper = detail.upper()
                if count > 0 and age < 2.5:
                    if "READY" in upper and ("INSERT" in upper or "FORK" in upper):
                        pill.set_state("healthy", "READY INSERT")
                    elif "ALIGN" in upper:
                        pill.set_state("healthy", "ALIGNING")
                    elif "WAIT" in upper or "NO PALLET" in upper or "SEARCH" in upper:
                        pill.set_state("degraded", "WAITING TARGET")
                    elif "STOP" in upper or "ERROR" in upper or "FAULT" in upper:
                        pill.set_state("error", "HOLD")
                    else:
                        pill.set_state("healthy", "STREAMING")
                    value.setText(detail[:84] if detail else f"state stream {age:.2f}s")
                elif count > 0:
                    pill.set_state("degraded", "STALE")
                    value.setText(f"fork state stale {age:.1f}s")
                else:
                    pill.set_state("degraded", "WAITING TARGET")
                    value.setText("fork node active; waiting alignment state")
                continue

            topic_states = [self._topic_state(self.latest_health.get(t)) for t in definition.get("topics", [])]
            healthy_topics = [v for v in topic_states if v[0] == "healthy"]
            degraded_topics = [v for v in topic_states if v[0] == "degraded"]
            error_topics = [v for v in topic_states if v[0] == "error"]

            if healthy_topics and not error_topics:
                pill.set_state("healthy")
                value.setText(healthy_topics[0][1])
            elif node_ok and (degraded_topics or topic_states):
                pill.set_state("degraded", "WAITING DATA")
                detail = (degraded_topics[0][1] if degraded_topics else "waiting for topic")
                value.setText(detail)
            elif node_ok:
                pill.set_state("healthy", "RUNNING")
                value.setText("node present")
            else:
                if definition.get("optional"):
                    pill.set_state("disabled", "DISABLED")
                    value.setText("optional module not enabled")
                else:
                    pill.set_state("error", "NOT RUNNING")
                    value.setText("node/topic not found")

    def update_tf(self, payload: Dict[str, Dict[str, Any]]):
        self.tf_payload = payload
        self.tf_table.setRowCount(0)
        for key, info in payload.items():
            row = self.tf_table.rowCount()
            self.tf_table.insertRow(row)
            available = bool(info.get("available"))
            values = [
                key,
                "OK" if available else "MISSING",
                f"{float(info.get('x', 0.0)):.3f}" if available else "-",
                f"{float(info.get('y', 0.0)):.3f}" if available else "-",
                f"{float(info.get('z', 0.0)):.3f}" if available else "-",
                f"{math.degrees(float(info.get('yaw', 0.0))):.2f}" if available else "-",
            ]
            for col, text in enumerate(values):
                self.tf_table.setItem(row, col, QTableWidgetItem(text))
        self._refresh_cards()


class MapPage(QObject):
    goal_requested = pyqtSignal(float, float, float)
    initial_requested = pyqtSignal(float, float, float)
    ground_truth_requested = pyqtSignal(float, float)

    def __init__(self, source: MapSource, map_repo: MapRepository):
        super().__init__()
        self.source = source
        self.map_repo = map_repo
        self.document: Optional[MapDocument] = None
        self.active_nav_slot: Optional[int] = None
        # All three snapshots use ROS map coordinates, but commands are gated
        # by the active map_server slot. A goal clicked on Map 1 is a Map-1 GUI
        # marker and is not sent to Nav2 while Map 2/3 is the active nav map.
        self.override_live = True
        self.ui_active = False
        self.last_pose: Dict[str, Tuple[float, float, float]] = {}
        self.last_scan: Sequence[Tuple[float, float]] = []
        self.last_paths: Dict[str, Sequence[Tuple[float, float]]] = {}
        self.last_trajs: Sequence[Sequence[Tuple[float, float]]] = []
        self.last_grids: Dict[str, Any] = {}
        self.ground_truth: Sequence[Dict[str, Any]] = []
        # Goal / Initial markers are GUI-local per map slot.  A goal clicked on
        # Map 1 must never appear on Map 2/3 merely because all snapshots share
        # the ROS ``map`` frame.  Nav2 still receives the published global pose;
        # only the visual marker ownership is isolated here.
        self.local_goal: Optional[Tuple[float, float, float]] = None
        self.local_initial: Optional[Tuple[float, float, float]] = None
        # V32: scan-to-map quality analysis is intentionally slower than the
        # live LiDAR overlay.  Re-running full occupancy-neighbour analysis on
        # every scan can monopolize the Qt thread and make the GUI unresponsive.
        self._last_scan_quality_time = 0.0
        self._scan_quality_period_s = 1.0

        self.settings = ScrollSettings()
        self.meta_section = SectionFrame(f"Map {source.slot} Metadata")
        self.meta_label = QLabel("Not loaded")
        self.meta_label.setWordWrap(True)
        self.reload_btn = QPushButton("Reload Map")
        self.meta_section.addWidget(self.meta_label)
        self.meta_section.addWidget(self.reload_btn)
        self.settings.insert_widget(self.meta_section)
        self.map_yaml_editor = YamlParameterEditor(source.yaml_path, title=f"Map {source.slot} YAML")
        self.map_yaml_editor.saved.connect(lambda _path: self.reload())
        self.settings.insert_widget(self.map_yaml_editor)

        mode_section = SectionFrame("Map Tools")
        row = QHBoxLayout()
        self.pan_btn = QPushButton("Pan")
        self.gt_btn = QPushButton("Add GT")
        self.goal_btn = QPushButton("Set Goal")
        self.initial_btn = QPushButton("Initial Pose")
        for btn in (self.pan_btn, self.gt_btn, self.goal_btn, self.initial_btn):
            row.addWidget(btn)
        mode_section.addLayout(row)
        row2 = QHBoxLayout()
        self.fit_btn = QPushButton("Fit")
        self.center_btn = QPushButton("Center Robot")
        self.shot_btn = QPushButton("Screenshot")
        self.clear_goal_btn = QPushButton("Clear Goal")
        self.clear_initial_btn = QPushButton("Clear Initial")
        for btn in (self.fit_btn, self.center_btn, self.shot_btn, self.clear_goal_btn, self.clear_initial_btn):
            row2.addWidget(btn)
        mode_section.addLayout(row2)
        self.override_box = QCheckBox("Show live overlays on this map snapshot")
        # Default is isolated: only the map currently loaded by map_server gets
        # live navigation overlays.  Operators can explicitly enable comparison
        # overlays on an inactive snapshot when needed.
        self.override_box.setChecked(False)
        self.override_live = False
        self.override_box.setToolTip("Inactive map snapshots stay isolated by default. Goal/Initial commands are published only from the map slot currently loaded by map_server.")
        mode_section.addWidget(self.override_box)
        self.live_warning = QLabel("Live overlay status: checking active map")
        self.live_warning.setWordWrap(True)
        mode_section.addWidget(self.live_warning)
        self.settings.insert_widget(mode_section)

        layer_section = SectionFrame("Layers")
        self.layer_checks: Dict[str, QCheckBox] = {}
        layer_labels = [
            ("scan", "Live LiDAR Scan"), ("lidar_odom", "LiDAR Odometry"), ("ekf", "EKF"),
            ("amcl", "AMCL"), ("tf", "TF Robot Pose"), ("ground_truth", "Ground Truth"),
            ("static_inflation", "Saved-map Costmap + Inflation"),
            ("global_path", "Smac Global Path"), ("local_path", "MPPI Transformed Plan"),
            ("trajectories", "MPPI Trajectories"), ("global_costmap", "Live Global Costmap"),
            ("local_costmap", "Live Local Costmap"), ("goal", "Goal"), ("initial", "Initial Pose"),
        ]
        for key, label in layer_labels:
            cb = QCheckBox(label)
            cb.setChecked(True)
            cb.toggled.connect(lambda checked, k=key: self.canvas.set_layer_visible(k, checked))
            layer_section.addWidget(cb)
            self.layer_checks[key] = cb
        self.settings.insert_widget(layer_section)

        quality_section = SectionFrame("Map Quality")
        self.quality_label = QLabel("Not analyzed")
        self.quality_label.setWordWrap(True)
        self.analyze_btn = QPushButton("Run Map Quality Test")
        self.export_quality_btn = QPushButton("Export Map Quality CSV")
        quality_section.addWidget(self.quality_label)
        quality_section.addWidget(self.analyze_btn)
        quality_section.addWidget(self.export_quality_btn)
        self.settings.insert_widget(quality_section)
        self.quality_metrics: Dict[str, float] = {}
        self.loaded_once = False

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        header = QHBoxLayout()
        self.title = QLabel(f"Map {source.slot} — LiDAR Mapping Result")
        self.title.setObjectName("WorkspaceTitle")
        self.coord = QLabel("Pixel: - | Map: -")
        header.addWidget(self.title)
        header.addStretch(1)
        header.addWidget(self.coord)
        root.addLayout(header)
        self.canvas = MapCanvas()
        root.addWidget(self.canvas, 1)

        self.reload_btn.clicked.connect(self.reload)
        self.pan_btn.clicked.connect(lambda: self.canvas.set_mode("pan"))
        self.gt_btn.clicked.connect(lambda: self.canvas.set_mode("ground_truth"))
        self.goal_btn.clicked.connect(lambda: self.canvas.set_mode("goal"))
        self.initial_btn.clicked.connect(lambda: self.canvas.set_mode("initial"))
        self.fit_btn.clicked.connect(self.canvas.fit_map)
        self.center_btn.clicked.connect(self.canvas.center_robot)
        self.shot_btn.clicked.connect(self.save_screenshot)
        self.clear_goal_btn.clicked.connect(self.clear_goal_marker)
        self.clear_initial_btn.clicked.connect(self.clear_initial_marker)
        self.analyze_btn.clicked.connect(self.analyze_quality)
        self.export_quality_btn.clicked.connect(self.export_quality)
        self.override_box.toggled.connect(self._override_changed)
        self.canvas.coordinate_changed.connect(self._coordinate_changed)
        self.canvas.point_clicked.connect(self._point_clicked)
        self.canvas.pose_clicked.connect(self._pose_clicked)
        self.meta_label.setText("Deferred — select this Map tab to load PGM/YAML")

    def set_active(self, active: bool):
        self.ui_active = bool(active)
        if self.ui_active:
            self.redraw_live()
            self._redraw_local_markers()

    def _override_changed(self, checked: bool):
        self.override_live = bool(checked)
        self._refresh_live_warning()
        self.redraw_live()

    def set_active_nav_slot(self, slot: Optional[int]):
        self.active_nav_slot = slot
        self._refresh_live_warning()
        self.redraw_live()

    def _live_allowed(self) -> bool:
        return self.override_live or (self.active_nav_slot is not None and self.active_nav_slot == self.source.slot)

    def _paint_allowed(self) -> bool:
        # Store full-rate data for all map slots, but only touch QGraphicsScene
        # for the visible map page. This removes hidden LiDAR/costmap rendering.
        return self.ui_active and self._live_allowed()

    def _refresh_live_warning(self):
        if self.active_nav_slot == self.source.slot:
            self.live_warning.setText("Live overlay: ENABLED — this map is the active navigation map")
            self.live_warning.setStyleSheet("color:#4bc17b")
        elif self.override_live:
            self.live_warning.setText(
                f"Live overlay: ENABLED FOR COMPARISON — active navigation map is {self.active_nav_slot or 'OTHER/NONE'}. "
                "Comparison overlays are visual only; Goal/Initial commands remain gated to the active navigation map.")
            self.live_warning.setStyleSheet("color:#4bc17b")
        else:
            self.live_warning.setText(
                f"Live overlay: DISABLED — active navigation map is {self.active_nav_slot or 'OTHER/NONE'}. "
                "Saved-map Costmap + Inflation remains visible for this map slot.")
            self.live_warning.setStyleSheet("color:#d7a642")

    def reload(self):
        self.loaded_once = True
        try:
            if not getattr(self.map_yaml_editor, "loaded_once", False):
                self.map_yaml_editor.reload()
            self.document = MapLoader().load(self.source)
            self.canvas.set_document(self.document)
            stats = self.document.stats()
            m = self.document.metadata
            self.title.setText(f"Map {self.source.slot} — {self.source.actual_name}")
            self.meta_label.setText(
                f"PGM: {self.source.image_path}\nYAML: {self.source.yaml_path}\n"
                f"Resolution: {m.resolution:.6f} m/px\nOrigin: [{m.origin_x:.3f}, {m.origin_y:.3f}, {m.origin_yaw:.5f}]\n"
                f"Size: {self.document.width}×{self.document.height} px | {self.document.width_m:.2f}×{self.document.height_m:.2f} m\n"
                f"Free: {stats['free_pct']:.2f}% | Occupied: {stats['occupied_pct']:.2f}% | Unknown: {stats['unknown_pct']:.2f}%\n"
                f"Build: {datetime.fromtimestamp(self.source.build_timestamp).isoformat(timespec='seconds')}")
            # Every map slot gets its own truthful static inflation preview.
            # Only the active map_server slot may receive live global/local
            # costmaps; copying those grids to Map 1/2 would be geometrically wrong.
            self.canvas.draw_static_inflation(
                self.document.occupancy,
                m.resolution,
                inflation_radius=0.45,
                cost_scaling_factor=12.0,
                inscribed_radius=0.40,
            )
            self.canvas.draw_ground_truth(self.ground_truth)
            self.redraw_live()
            self._redraw_local_markers()
            self.quality_label.setText("Map loaded — press Run Map Quality Test for occupancy-noise analysis")
        except Exception as exc:
            self.document = None
            self.canvas.set_document(None)
            self.meta_label.setText(f"Map {self.source.slot}: NOT AVAILABLE\n{self.source.yaml_path}\n{exc}")

    def _coordinate_changed(self, u: float, v: float, x: float, y: float):
        self.coord.setText(f"Pixel {u:.1f},{v:.1f} | Map {x:.3f},{y:.3f} m")

    def _point_clicked(self, x: float, y: float, _yaw: float):
        if self.canvas.mode == "ground_truth":
            self.ground_truth_requested.emit(x, y)

    def _pose_clicked(self, x: float, y: float, yaw: float):
        if self.canvas.mode == "goal":
            self.local_goal = (float(x), float(y), float(yaw))
            self.canvas.draw_pose_marker("goal", x, y, yaw, QColor("#ff7c59"), f"GOAL M{self.source.slot}")
            self.goal_requested.emit(x, y, yaw)
        elif self.canvas.mode == "initial":
            self.local_initial = (float(x), float(y), float(yaw))
            self.canvas.draw_pose_marker("initial", x, y, yaw, QColor("#b586ff"), f"INITIAL M{self.source.slot}")
            self.initial_requested.emit(x, y, yaw)

    def _redraw_local_markers(self):
        """Render only markers owned by this map slot."""
        if self.document is None:
            return
        self.canvas.clear_layer("goal")
        self.canvas.clear_layer("initial")
        if self.local_goal is not None:
            x, y, yaw = self.local_goal
            self.canvas.draw_pose_marker("goal", x, y, yaw, QColor("#ff7c59"), f"GOAL M{self.source.slot}")
        if self.local_initial is not None:
            x, y, yaw = self.local_initial
            self.canvas.draw_pose_marker("initial", x, y, yaw, QColor("#b586ff"), f"INITIAL M{self.source.slot}")

    def clear_goal_marker(self):
        self.local_goal = None
        self.canvas.clear_layer("goal")

    def clear_initial_marker(self):
        self.local_initial = None
        self.canvas.clear_layer("initial")

    def set_ground_truth(self, points: Sequence[Dict[str, Any]]):
        self.ground_truth = points
        if self.document is not None:
            self.canvas.draw_ground_truth(points)

    def update_pose(self, source: str, x: float, y: float, yaw: float):
        self.last_pose[source] = (x, y, yaw)
        if not self._paint_allowed() or self.document is None:
            return
        colors = {
            "lidar_odom": QColor("#e6c84d"),
            "ekf": QColor("#4dd899"),
            "amcl": QColor("#e96c9f"),
            "tf": QColor("#49b8ff"),
            "esc_odom": QColor("#b9b9b9"),
        }
        layer = source if source in ("lidar_odom", "ekf", "amcl", "tf") else "tf"
        self.canvas.draw_pose_marker(layer, x, y, yaw, colors.get(source, QColor("#ffffff")), source.upper())
        if source == "tf":
            self.canvas.draw_robot(x, y, yaw)

    def update_scan(self, points: Sequence[Tuple[float, float]]):
        self.last_scan = points
        if self._paint_allowed() and self.document is not None:
            self.canvas.draw_points("scan", points, radius_px=1.5, color=QColor("#59d6e8"))
            now = time.monotonic()
            if (now - self._last_scan_quality_time) >= self._scan_quality_period_s:
                self._last_scan_quality_time = now
                self._update_scan_consistency(points)

    def update_path(self, topic: str, points: Sequence[Tuple[float, float]]):
        self.last_paths[topic] = points
        if not self._paint_allowed() or self.document is None:
            return
        if topic == "/smac_plan":
            self.canvas.draw_path("global_path", points, QColor("#ff7a5c"), 2.2)
        elif topic == "/transformed_global_plan":
            self.canvas.draw_path("local_path", points, QColor("#d694ff"), 2.0)

    def update_trajectories(self, trajectories: Sequence[Sequence[Tuple[float, float]]]):
        self.last_trajs = trajectories
        if not self._paint_allowed() or self.document is None:
            return
        self.canvas.clear_layer("trajectories")
        first = True
        for line in trajectories[:80]:
            self.canvas.draw_path("trajectories", line, QColor(130, 180, 255, 130), 1.0, clear=first)
            first = False

    def update_grid(self, topic: str, msg: Any):
        self.last_grids[topic] = msg
        if not self._paint_allowed() or self.document is None:
            return
        if topic == "/global_costmap/costmap":
            self.canvas.draw_occupancy_grid("global_costmap", msg, alpha=80)
        elif topic == "/local_costmap/costmap":
            self.canvas.draw_occupancy_grid("local_costmap", msg, alpha=95)

    def redraw_live(self):
        if self.document is None or not self.ui_active:
            return
        if not self._live_allowed():
            for layer in ("scan", "lidar_odom", "ekf", "amcl", "tf", "robot", "global_path", "local_path", "trajectories", "global_costmap", "local_costmap"):
                self.canvas.clear_layer(layer)
            return
        for source, pose in self.last_pose.items():
            self.update_pose(source, *pose)
        self.update_scan(self.last_scan)
        for topic, points in self.last_paths.items():
            self.update_path(topic, points)
        self.update_trajectories(self.last_trajs)
        for topic, grid in self.last_grids.items():
            self.update_grid(topic, grid)

    def analyze_quality(self):
        if self.document is None:
            self.quality_label.setText("N/A — map not loaded")
            return
        occ = self.document.occupancy
        stats = self.document.stats()
        occupied = occ >= 65
        # Lightweight 8-neighbor count: a practical isolated-noise indicator that
        # does not mutate the production map.
        padded = np.pad(occupied.astype(np.uint8), 1, mode="constant")
        neighbors = np.zeros_like(occupied, dtype=np.uint8)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbors += padded[1 + dy:1 + dy + occupied.shape[0], 1 + dx:1 + dx + occupied.shape[1]]
        isolated = int(np.count_nonzero(occupied & (neighbors == 0)))
        weak_cluster = int(np.count_nonzero(occupied & (neighbors <= 1)))
        self.quality_metrics.update({
            "free_pct": stats["free_pct"],
            "occupied_pct": stats["occupied_pct"],
            "unknown_pct": stats["unknown_pct"],
            "isolated_occupied_cells": float(isolated),
            "weak_cluster_cells": float(weak_cluster),
        })
        text = (
            f"Free {stats['free_pct']:.2f}% | Occupied {stats['occupied_pct']:.2f}% | Unknown {stats['unknown_pct']:.2f}%\n"
            f"Isolated occupied cells: {isolated} | weak-cluster cells: {weak_cluster}"
        )
        if "scan_hit_pct" in self.quality_metrics:
            text += (f"\nScan→occupied: {self.quality_metrics['scan_hit_pct']:.1f}% | "
                     f"near wall: {self.quality_metrics['scan_near_wall_pct']:.1f}% | "
                     f"scan unknown: {self.quality_metrics['scan_unknown_pct']:.1f}%")
        self.quality_label.setText(text)

    def _update_scan_consistency(self, points: Sequence[Tuple[float, float]]):
        if self.document is None or not points:
            return
        occ = self.document.occupancy
        h, w = occ.shape
        direct_occ = 0
        near_occ = 0
        free = 0
        unknown = 0
        inside = 0
        radius = max(1, min(8, int(round(0.20 / self.document.metadata.resolution))))
        for x, y in points[::max(1, len(points)//1200)]:
            u, v = self.document.transform.map_to_pixel(x, y)
            i = int(round(u)); j = int(round(v))
            if i < 0 or i >= w or j < 0 or j >= h:
                continue
            inside += 1
            value = int(occ[j, i])
            if value >= 65:
                direct_occ += 1
                near_occ += 1
            elif value < 0:
                unknown += 1
            else:
                free += 1
                j0, j1 = max(0, j-radius), min(h, j+radius+1)
                i0, i1 = max(0, i-radius), min(w, i+radius+1)
                if np.any(occ[j0:j1, i0:i1] >= 65):
                    near_occ += 1
        if inside:
            self.quality_metrics.update({
                "scan_inside_points": float(inside),
                "scan_hit_pct": 100.0 * direct_occ / inside,
                "scan_near_wall_pct": 100.0 * near_occ / inside,
                "scan_free_pct": 100.0 * free / inside,
                "scan_unknown_pct": 100.0 * unknown / inside,
            })
            self.analyze_quality()

    def export_quality(self):
        if not self.quality_metrics:
            self.analyze_quality()
        if not self.quality_metrics:
            return
        base = Path("/home/otomasi2/forclift/log/agv_gui/manual_exports")
        base.mkdir(parents=True, exist_ok=True)
        default = base / f"map{self.source.slot}_quality_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QFileDialog.getSaveFileName(self.content, "Export Map Quality", str(default), "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["metric", "value"])
            writer.writerow(["map_slot", self.source.slot])
            writer.writerow(["yaml", str(self.source.yaml_path)])
            for key, value in sorted(self.quality_metrics.items()):
                writer.writerow([key, value])

    def save_screenshot(self):
        default = Path("/home/otomasi2/forclift/log/agv_gui") / f"map{self.source.slot}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        path, _ = QFileDialog.getSaveFileName(self.content, "Save Map Screenshot", str(default), "PNG (*.png)")
        if path:
            self.canvas.grab().save(path, "PNG")


class MapComparisonPage:
    def __init__(self, repo: MapRepository, map_pages: Dict[int, MapPage]):
        self.repo = repo
        self.map_pages = map_pages
        self.settings = ScrollSettings()
        sec = SectionFrame("Comparison")
        self.reload_btn = QPushButton("Reload & Compare")
        self.export_btn = QPushButton("Export CSV")
        self.export_png_btn = QPushButton("Export PNG")
        sec.addWidget(self.reload_btn)
        sec.addWidget(self.export_btn)
        sec.addWidget(self.export_png_btn)
        self.settings.insert_widget(sec)
        self.note = QLabel("Comparison is performed in metric map coordinates; raw pixels are never assumed equivalent.")
        self.note.setWordWrap(True)
        self.settings.insert_widget(self.note)

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("Map 1 / Map 2 / Map 3 Comparison")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Metric", "Map 1", "Map 2", "Map 3"])
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table)
        self.pair_table = QTableWidget(0, 4)
        self.pair_table.setHorizontalHeaderLabels(["Pair", "Agreement %", "Known Agreement %", "Samples"])
        self.pair_table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.pair_table, 1)
        self.reload_btn.setText("Load All Maps & Compare")
        self.reload_btn.clicked.connect(self.refresh)
        self.export_btn.clicked.connect(self.export_csv)
        self.export_png_btn.clicked.connect(self.export_png)
        self.table.setRowCount(1)
        self.table.setItem(0, 0, QTableWidgetItem("Status"))
        self.table.setItem(0, 1, QTableWidgetItem("Press Load All Maps & Compare"))

    def refresh(self):
        docs: Dict[int, Optional[MapDocument]] = {}
        for slot, page in self.map_pages.items():
            page.reload()
            docs[slot] = page.document
        rows = [
            ("Actual Name", lambda d: d.source.actual_name),
            ("Resolution [m/px]", lambda d: f"{d.metadata.resolution:.6f}"),
            ("Width [px]", lambda d: str(d.width)),
            ("Height [px]", lambda d: str(d.height)),
            ("Width [m]", lambda d: f"{d.width_m:.3f}"),
            ("Height [m]", lambda d: f"{d.height_m:.3f}"),
            ("Origin X [m]", lambda d: f"{d.metadata.origin_x:.4f}"),
            ("Origin Y [m]", lambda d: f"{d.metadata.origin_y:.4f}"),
            ("Origin Yaw [rad]", lambda d: f"{d.metadata.origin_yaw:.6f}"),
            ("Free [%]", lambda d: f"{d.stats()['free_pct']:.2f}"),
            ("Occupied [%]", lambda d: f"{d.stats()['occupied_pct']:.2f}"),
            ("Unknown [%]", lambda d: f"{d.stats()['unknown_pct']:.2f}"),
        ]
        self.table.setRowCount(len(rows))
        for row, (metric, fn) in enumerate(rows):
            self.table.setItem(row, 0, QTableWidgetItem(metric))
            for slot in range(1, 4):
                doc = docs.get(slot)
                value = fn(doc) if doc is not None else "N/A"
                self.table.setItem(row, slot, QTableWidgetItem(str(value)))
        pair_results = []
        for a, b in ((1, 2), (1, 3), (2, 3)):
            da, db = docs.get(a), docs.get(b)
            if da is None or db is None:
                pair_results.append((f"Map {a} vs Map {b}", None))
            else:
                try:
                    pair_results.append((f"Map {a} vs Map {b}", compare_maps(da, db)))
                except Exception:
                    pair_results.append((f"Map {a} vs Map {b}", None))
        self.pair_table.setRowCount(len(pair_results))
        for row, (name, result) in enumerate(pair_results):
            vals = [name, "N/A", "N/A", "N/A"]
            if result:
                vals[1] = self._fmt(result.get("agreement_pct"))
                vals[2] = self._fmt(result.get("known_agreement_pct"))
                vals[3] = str(int(result.get("overlap_samples", 0)))
            for col, value in enumerate(vals):
                self.pair_table.setItem(row, col, QTableWidgetItem(value))

    @staticmethod
    def _fmt(v):
        try:
            return f"{float(v):.2f}"
        except Exception:
            return "N/A"

    def export_png(self):
        default = Path("/home/otomasi2/forclift/log/agv_gui/map_comparison.png")
        path, _ = QFileDialog.getSaveFileName(self.content, "Export Map Comparison PNG", str(default), "PNG (*.png)")
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.content.grab().save(path, "PNG")

    def export_csv(self):
        default = Path("/home/otomasi2/forclift/log/agv_gui/map_comparison.csv")
        path, _ = QFileDialog.getSaveFileName(self.content, "Export Map Comparison", str(default), "CSV (*.csv)")
        if not path:
            return
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["metric", "map1", "map2", "map3"])
            for row in range(self.table.rowCount()):
                writer.writerow([self.table.item(row, col).text() for col in range(4)])
            writer.writerow([])
            writer.writerow(["pair", "agreement_pct", "known_agreement_pct", "samples"])
            for row in range(self.pair_table.rowCount()):
                writer.writerow([self.pair_table.item(row, col).text() for col in range(4)])


class GroundTruthPage(QObject):
    points_changed = pyqtSignal(object)

    def __init__(self, path: Path):
        super().__init__()
        self.path = path
        self.store = YamlStore(path)
        self.points: List[Dict[str, Any]] = []
        self.live_poses: Dict[str, Tuple[float, float, float]] = {}
        self.measurements: List[Dict[str, Any]] = []
        self.measurement_path = Path("/home/otomasi2/forclift/log/agv_gui/ground_truth_measurements.csv")
        self.settings = ScrollSettings()
        sec = SectionFrame("Ground Truth")
        self.x = NoWheelDoubleSpinBox(); self.x.setRange(-1e5, 1e5); self.x.setDecimals(4)
        self.y = NoWheelDoubleSpinBox(); self.y.setRange(-1e5, 1e5); self.y.setDecimals(4)
        self.yaw = NoWheelDoubleSpinBox(); self.yaw.setRange(-360.0, 360.0); self.yaw.setDecimals(3)
        self.name = QLineEdit()
        form = QFormLayout()
        form.addRow("Name", self.name)
        form.addRow("X [m]", self.x)
        form.addRow("Y [m]", self.y)
        form.addRow("Yaw [deg]", self.yaw)
        sec.addLayout(form)
        row = QHBoxLayout()
        self.add_btn = QPushButton("Add")
        self.delete_btn = QPushButton("Delete Selected")
        row.addWidget(self.add_btn); row.addWidget(self.delete_btn)
        sec.addLayout(row)
        self.capture_btn = QPushButton("Capture Measurement")
        self.export_btn = QPushButton("Export Ground Truth CSV")
        self.export_measure_btn = QPushButton("Export Measurements CSV")
        self.measure_summary = QLabel("Measurements: 0")
        self.measure_summary.setWordWrap(True)
        sec.addWidget(self.capture_btn)
        sec.addWidget(self.export_btn)
        sec.addWidget(self.export_measure_btn)
        sec.addWidget(self.measure_summary)
        self.settings.insert_widget(sec)

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("Ground Truth Database")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(["ID", "Name", "X", "Y", "Yaw [deg]", "Category", "Reference", "Method"])
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table, 1)
        measure_title = QLabel("Ground Truth Measurements")
        measure_title.setObjectName("SectionTitle")
        root.addWidget(measure_title)
        self.measure_table = QTableWidget(0, 7)
        self.measure_table.setHorizontalHeaderLabels(["Time", "GT", "Source", "dx [m]", "dy [m]", "Error [m]", "Yaw Error [deg]"])
        self.measure_table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.measure_table, 1)
        self.add_btn.clicked.connect(self.add_from_form)
        self.delete_btn.clicked.connect(self.delete_selected)
        self.capture_btn.clicked.connect(self.capture_measurement)
        self.export_btn.clicked.connect(self.export_csv)
        self.export_measure_btn.clicked.connect(self.export_measurements)
        self._load_measurements()
        self.load()

    def load(self):
        try:
            data = self.store.load()
            self.points = list(data.get("ground_truth_points", [])) if isinstance(data, dict) else []
        except Exception:
            self.points = []
        self.refresh_table()
        self._refresh_measurements()
        self.points_changed.emit(self.points)

    def save(self):
        self.store.data = {"ground_truth_points": self.points}
        self.store.save(make_backup=True)
        self.points_changed.emit(self.points)

    def add_point(self, x: float, y: float, name: str = ""):
        self.x.setValue(x); self.y.setValue(y)
        if name:
            self.name.setText(name)
        self.add_from_form()

    def add_from_form(self):
        next_id = f"GT-{len(self.points)+1:02d}"
        self.points.append({
            "id": next_id,
            "name": self.name.text().strip() or next_id,
            "x": float(self.x.value()),
            "y": float(self.y.value()),
            "yaw": math.radians(float(self.yaw.value())),
            "description": "",
            "category": "",
            "physical_reference": "",
            "measurement_method": "manual",
        })
        self.name.clear()
        self.save()
        self.refresh_table()

    def delete_selected(self):
        rows = sorted({idx.row() for idx in self.table.selectedIndexes()}, reverse=True)
        for row in rows:
            if 0 <= row < len(self.points):
                self.points.pop(row)
        if rows:
            self.save(); self.refresh_table()

    def refresh_table(self):
        self.table.setRowCount(len(self.points))
        for row, point in enumerate(self.points):
            vals = [
                point.get("id", ""), point.get("name", ""), f"{float(point.get('x',0)):.4f}",
                f"{float(point.get('y',0)):.4f}", f"{math.degrees(float(point.get('yaw',0))):.3f}",
                point.get("category", ""), point.get("physical_reference", ""), point.get("measurement_method", ""),
            ]
            for col, value in enumerate(vals):
                self.table.setItem(row, col, QTableWidgetItem(str(value)))

    def update_pose(self, source: str, x: float, y: float, yaw: float):
        self.live_poses[source] = (float(x), float(y), float(yaw))

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def capture_measurement(self):
        rows = sorted({idx.row() for idx in self.table.selectedIndexes()})
        if not rows:
            QMessageBox.warning(self.content, "Ground Truth", "Select one ground-truth point first.")
            return
        row = rows[0]
        if not (0 <= row < len(self.points)):
            return
        gt = self.points[row]
        gx, gy, gyaw = float(gt.get("x", 0.0)), float(gt.get("y", 0.0)), float(gt.get("yaw", 0.0))
        captured = 0
        for source in ("lidar_odom", "ekf", "amcl", "tf"):
            pose = self.live_poses.get(source)
            if pose is None:
                continue
            x, y, yaw = pose
            dx, dy = x - gx, y - gy
            item = {
                "timestamp": time.time(),
                "gt_id": gt.get("id", ""),
                "source": source,
                "gt_x": gx, "gt_y": gy, "gt_yaw": gyaw,
                "measured_x": x, "measured_y": y, "measured_yaw": yaw,
                "dx": dx, "dy": dy,
                "distance_error": math.hypot(dx, dy),
                "yaw_error": self._wrap_angle(yaw - gyaw),
            }
            self.measurements.append(item)
            captured += 1
        if not captured:
            QMessageBox.warning(self.content, "Ground Truth", "No live LiDAR/EKF/AMCL/TF pose is available yet.")
            return
        self._write_measurements(self.measurement_path)
        self._refresh_measurements()

    def _load_measurements(self):
        self.measurements = []
        if not self.measurement_path.is_file():
            return
        try:
            with self.measurement_path.open("r", newline="", encoding="utf-8") as handle:
                for row in csv.DictReader(handle):
                    item = dict(row)
                    for key in ("timestamp", "gt_x", "gt_y", "gt_yaw", "measured_x", "measured_y", "measured_yaw", "dx", "dy", "distance_error", "yaw_error"):
                        try:
                            item[key] = float(item[key])
                        except Exception:
                            pass
                    self.measurements.append(item)
        except OSError:
            pass

    def _write_measurements(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = ["timestamp", "gt_id", "source", "gt_x", "gt_y", "gt_yaw", "measured_x", "measured_y", "measured_yaw", "dx", "dy", "distance_error", "yaw_error"]
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for row in self.measurements:
                writer.writerow({key: row.get(key, "") for key in fields})

    def _refresh_measurements(self):
        self.measure_table.setRowCount(len(self.measurements))
        for row_idx, row in enumerate(self.measurements):
            vals = [
                datetime.fromtimestamp(float(row.get("timestamp", 0.0))).strftime("%H:%M:%S"),
                row.get("gt_id", ""), row.get("source", ""),
                f"{float(row.get('dx',0)):.4f}", f"{float(row.get('dy',0)):.4f}",
                f"{float(row.get('distance_error',0)):.4f}",
                f"{math.degrees(float(row.get('yaw_error',0))):.3f}",
            ]
            for col, value in enumerate(vals):
                self.measure_table.setItem(row_idx, col, QTableWidgetItem(str(value)))
        parts = [f"Measurements: {len(self.measurements)}"]
        for source in ("lidar_odom", "ekf", "amcl", "tf"):
            errors = [float(r.get("distance_error", 0.0)) for r in self.measurements if r.get("source") == source]
            if errors:
                rmse = math.sqrt(sum(e*e for e in errors) / len(errors))
                parts.append(f"{source} RMSE={rmse:.3f} m")
        self.measure_summary.setText(" | ".join(parts))

    def export_measurements(self):
        default = Path("/home/otomasi2/forclift/log/agv_gui/ground_truth_measurements_export.csv")
        path, _ = QFileDialog.getSaveFileName(self.content, "Export Ground Truth Measurements", str(default), "CSV (*.csv)")
        if path:
            self._write_measurements(Path(path))

    def export_csv(self):
        default = self.path.parent / "ground_truth.csv"
        path, _ = QFileDialog.getSaveFileName(self.content, "Export Ground Truth", str(default), "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as handle:
            fields = ["id", "name", "x", "y", "yaw", "description", "category", "physical_reference", "measurement_method"]
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for point in self.points:
                writer.writerow({key: point.get(key, "") for key in fields})


class TimingPage:
    """Realtime topic timing + TF health page required by the GUI specification."""

    TOPICS = ["/scan_nav", "/imu/data", "/lidar/odom", "/odometry/filtered", "/amcl_pose"]

    def __init__(self):
        self.settings = ScrollSettings()
        sec = SectionFrame("Timing Thresholds")
        note = QLabel(
            "Live rate/age is measured from message arrivals, not copied from YAML. "
            "Age >1.5 s is degraded; >4 s is stale in the overview."
        )
        note.setWordWrap(True)
        sec.addWidget(note)
        self.settings.insert_widget(sec)

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("TF & Timing")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)

        tabs = QTabWidget()
        self.topic_table = TopicTable(self.TOPICS)
        tabs.addTab(self.topic_table, "Topic Timing")

        self.tf_table = QTableWidget(0, 6)
        self.tf_table.setHorizontalHeaderLabels(["Transform", "State", "X", "Y", "Z", "Yaw [deg]"])
        self.tf_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.tf_table, "TF Health")

        self.age_plot = RingPlot(
            "Message Age vs Time",
            ["scan_age", "imu_age", "lidar_odom_age", "ekf_age", "amcl_age"],
            history_seconds=60.0,
        )
        tabs.addTab(self.age_plot, "Age Plot")
        root.addWidget(tabs, 1)

    def update_health(self, payload: Dict[str, Dict[str, Any]]):
        selected = {topic: payload.get(topic, {}) for topic in self.TOPICS}
        self.topic_table.update_health(selected)
        key_map = {
            "/scan_nav": "scan_age",
            "/imu/data": "imu_age",
            "/lidar/odom": "lidar_odom_age",
            "/odometry/filtered": "ekf_age",
            "/amcl_pose": "amcl_age",
        }
        vals: Dict[str, float] = {}
        for topic, key in key_map.items():
            info = payload.get(topic, {})
            age = float(info.get("age", float("inf")))
            if math.isfinite(age):
                vals[key] = age
        if vals:
            self.age_plot.append(vals, time.time())

    def update_tf(self, payload: Dict[str, Dict[str, Any]]):
        self.tf_table.setRowCount(0)
        for key, info in payload.items():
            row = self.tf_table.rowCount()
            self.tf_table.insertRow(row)
            available = bool(info.get("available"))
            values = [
                key,
                "OK" if available else "MISSING",
                f"{float(info.get('x', 0.0)):.3f}" if available else "-",
                f"{float(info.get('y', 0.0)):.3f}" if available else "-",
                f"{float(info.get('z', 0.0)):.3f}" if available else "-",
                f"{math.degrees(float(info.get('yaw', 0.0))):.2f}" if available else "-",
            ]
            for col, text in enumerate(values):
                self.tf_table.setItem(row, col, QTableWidgetItem(text))


class CostmapPage:
    """Global/local costmap metrics plus the production Nav2 parameter editor."""

    def __init__(self, yaml_path: Path):
        self.settings = ScrollSettings()
        self.settings.insert_widget(YamlParameterEditor(yaml_path, "Nav2 Costmap YAML"))
        sec = SectionFrame("Costmap Test")
        self.note = QLabel("Waiting for /global_costmap/costmap and /local_costmap/costmap")
        self.note.setWordWrap(True)
        sec.addWidget(self.note)
        self.settings.insert_widget(sec)

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("Global / Local Costmap")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)

        self.table = QTableWidget(2, 9)
        self.table.setHorizontalHeaderLabels([
            "Source", "Width cells", "Height cells", "Resolution", "Width m", "Height m",
            "Free %", "Occupied %", "Unknown %",
        ])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setItem(0, 0, QTableWidgetItem("Global"))
        self.table.setItem(1, 0, QTableWidgetItem("Local"))
        root.addWidget(self.table)

        self.plot = RingPlot(
            "Costmap Occupancy",
            ["global_occupied_pct", "local_occupied_pct", "global_unknown_pct", "local_unknown_pct"],
            history_seconds=60.0,
        )
        root.addWidget(self.plot, 1)

    def update_telemetry(self, source: str, values: Dict[str, Any], timestamp: float):
        if source not in ("global_costmap", "local_costmap"):
            return
        row = 0 if source == "global_costmap" else 1
        keys = [
            "width_cells", "height_cells", "resolution", "width_m", "height_m",
            "free_pct", "occupied_pct", "unknown_pct",
        ]
        for col, key in enumerate(keys, start=1):
            value = values.get(key, "-")
            text = f"{float(value):.4g}" if isinstance(value, (int, float)) else str(value)
            self.table.setItem(row, col, QTableWidgetItem(text))
        self.note.setText("LIVE costmap data received")
        prefix = "global" if source == "global_costmap" else "local"
        plot_values = {}
        if "occupied_pct" in values:
            plot_values[f"{prefix}_occupied_pct"] = float(values["occupied_pct"])
        if "unknown_pct" in values:
            plot_values[f"{prefix}_unknown_pct"] = float(values["unknown_pct"])
        if plot_values:
            self.plot.append(plot_values, timestamp)


class SubsystemPage:
    """Config-on-left, realtime data/plot/stats-on-right engineering page."""

    def __init__(
        self,
        title: str,
        yaml_path: Optional[Path],
        series: Sequence[str],
        telemetry_sources: Sequence[str],
        tests: Sequence[str] = (),
        health_topics: Sequence[str] = (),
    ):
        self.title = title
        self.series = list(series)
        self.telemetry_sources = set(telemetry_sources)
        self.tests = list(tests)
        self.health_topics = list(health_topics)
        self.settings = ScrollSettings()
        if yaml_path is not None:
            editor = YamlParameterEditor(yaml_path, title=f"{title} YAML")
            self.settings.insert_widget(editor)

        test_sec = SectionFrame("Test & Export")
        self.health_label = QLabel("Live input: waiting")
        self.health_label.setWordWrap(True)
        test_sec.addWidget(self.health_label)
        if self.tests:
            tests_label = QLabel("Tests:\n• " + "\n• ".join(self.tests))
            tests_label.setWordWrap(True)
            test_sec.addWidget(tests_label)
        self.record_box = QCheckBox("Record realtime samples")
        self.stats_btn = QPushButton("Snapshot Statistics")
        self.export_btn = QPushButton("Export CSV + PNG")
        test_sec.addWidget(self.record_box)
        test_sec.addWidget(self.stats_btn)
        test_sec.addWidget(self.export_btn)
        self.settings.insert_widget(test_sec)

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        self.plot = RingPlot(title, self.series)
        root.addWidget(self.plot, 1)

        tabs = QTabWidget()
        self.values_table = QTableWidget(0, 2)
        self.values_table.setHorizontalHeaderLabels(["Signal", "Latest Value"])
        self.values_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.values_table, "Latest")

        self.stats_table = QTableWidget(0, 7)
        self.stats_table.setHorizontalHeaderLabels(["Signal", "N", "Mean", "Std", "Min", "Max", "P-P"])
        self.stats_table.horizontalHeader().setStretchLastSection(True)
        tabs.addTab(self.stats_table, "Statistics")
        root.addWidget(tabs)

        self.latest: Dict[str, Any] = {}
        self._ui_active = False
        self._values_dirty = False
        self._ui_timer = QTimer(self.content)
        self._ui_timer.setInterval(250)
        self._ui_timer.timeout.connect(self._flush_ui)
        self.plot.set_active(False)
        self.export_btn.clicked.connect(self.export)
        self.stats_btn.clicked.connect(self.snapshot_statistics)

    def set_active(self, active: bool):
        self._ui_active = bool(active)
        self.plot.set_active(self._ui_active)
        if self._ui_active:
            if not self._ui_timer.isActive():
                self._ui_timer.start()
            self._flush_ui()
        else:
            self._ui_timer.stop()

    def _flush_ui(self):
        if self._ui_active and self._values_dirty:
            self._refresh_values()
            self._values_dirty = False

    def update_health(self, payload: Dict[str, Dict[str, Any]]):
        if not self.health_topics:
            return
        event_driven = {
            "/smac_plan": "IDLE — waiting for Goal Pose",
            "/transformed_global_plan": "IDLE — available during active navigation",
            "/trajectories": "IDLE — available while MPPI is controlling",
            "/goal_pose": "IDLE — no goal sent yet",
            "/navigate_to_pose/_action/status": "IDLE — no active navigation goal",
        }
        parts = []
        for topic in self.health_topics:
            info = payload.get(topic, {})
            count = int(info.get("count", 0) or 0)
            age = float(info.get("age", float("inf")))
            hz = float(info.get("hz", 0.0) or 0.0)
            if count <= 0:
                if topic in event_driven:
                    state_text = event_driven[topic]
                    parts.append(f"{topic}: {state_text}")
                else:
                    state_text = "NO DATA"
                    parts.append(f"{topic}: NO DATA")
            elif math.isfinite(age):
                if topic in event_driven and age >= 1.5:
                    state_text = "IDLE — last event received"
                else:
                    state_text = f"LIVE {hz:.2f} Hz, age {age:.2f}s"
                parts.append(f"{topic}: {state_text}")
            else:
                state_text = "WAITING"
            self.latest[f"{topic}:state"] = state_text
        self._values_dirty = True
        self.health_label.setText("\n".join(parts) if parts else "Live input: waiting")

    def update_telemetry(self, source: str, values: Dict[str, Any], timestamp: float):
        if source not in self.telemetry_sources:
            return
        numeric: Dict[str, float] = {}
        for key, value in values.items():
            self.latest[key] = value
            try:
                f = float(value)
            except Exception:
                continue
            if key in self.series:
                numeric[key] = f
        if numeric:
            self.plot.append(numeric, timestamp)
        self._values_dirty = True

    def _refresh_values(self):
        items = sorted(self.latest.items(), key=lambda kv: kv[0])
        self.values_table.setRowCount(len(items))
        for row, (key, value) in enumerate(items):
            self.values_table.setItem(row, 0, QTableWidgetItem(str(key)))
            if isinstance(value, float):
                text = f"{value:.6g}"
            else:
                text = str(value)
            self.values_table.setItem(row, 1, QTableWidgetItem(text))

    def snapshot_statistics(self):
        rows = []
        for name in self.series:
            samples = [float(v) for _t, v in self.plot.buffers.get(name, [])]
            if not samples:
                continue
            arr = np.asarray(samples, dtype=np.float64)
            rows.append((
                name,
                len(samples),
                float(np.mean(arr)),
                float(np.std(arr)),
                float(np.min(arr)),
                float(np.max(arr)),
                float(np.ptp(arr)),
            ))
        self.stats_table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for col, value in enumerate(values):
                text = str(value) if col < 2 else f"{float(value):.6g}"
                self.stats_table.setItem(row, col, QTableWidgetItem(text))

    def export(self):
        base = Path("/home/otomasi2/forclift/log/agv_gui/manual_exports")
        base.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe = "".join(ch if ch.isalnum() else "_" for ch in self.title)
        csv_path = base / f"{safe}_{stamp}.csv"
        png_path = base / f"{safe}_{stamp}.png"
        self.plot.export_csv(csv_path)
        self.plot.export_png(png_path)
        self.snapshot_statistics()
        stats_path = base / f"{safe}_{stamp}_statistics.csv"
        with stats_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["signal", "n", "mean", "std", "min", "max", "peak_to_peak"])
            for row in range(self.stats_table.rowCount()):
                writer.writerow([
                    self.stats_table.item(row, col).text() if self.stats_table.item(row, col) else ""
                    for col in range(self.stats_table.columnCount())
                ])
        QMessageBox.information(self.content, "Export", f"Saved:\n{csv_path}\n{png_path}\n{stats_path}")


class WinchPage(QObject):
    """Operator controls and telemetry for the electric winch ROS bridge."""

    command_requested = pyqtSignal(str)

    def __init__(self, yaml_path: Path):
        super().__init__()
        self.latest: Dict[str, Any] = {}
        self.connected = False

        self.settings = ScrollSettings()
        self.settings.insert_widget(YamlParameterEditor(yaml_path, "Winch ROS Bridge YAML"))
        info = SectionFrame("Winch Interface")
        info_text = QLabel(
            "USB CDC firmware @ 115200 baud. Auto discovery prefers /dev/winch, "
            "STM/CDC by-id, then ttyACM*. Every reconnect sends STOP before STATUS.")
        info_text.setWordWrap(True)
        info.addWidget(info_text)
        self.settings.insert_widget(info)

        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("Electric Winch")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)

        cards = QWidget()
        grid = QGridLayout(cards)
        grid.setContentsMargins(0, 0, 0, 0)
        self.value_labels: Dict[str, QLabel] = {}
        specs = [
            ("connection", "Connection"), ("port", "Serial Port"), ("state", "State"),
            ("top_limit", "Top Limit"), ("bottom_limit", "Bottom Limit"),
            ("pwm_pct", "PWM [%]"), ("direction", "Direction"), ("servo_deg", "Servo [deg]"),
        ]
        for idx, (key, label) in enumerate(specs):
            frame = SectionFrame(label)
            value = QLabel("-")
            value.setAlignment(Qt.AlignCenter)
            value.setWordWrap(True)
            frame.addWidget(value)
            grid.addWidget(frame, idx // 4, idx % 4)
            self.value_labels[key] = value
        root.addWidget(cards)

        controls = SectionFrame("Manual Control")
        motion = QHBoxLayout()
        self.up_btn = QPushButton("UP")
        self.down_btn = QPushButton("DOWN")
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("DangerButton")
        for button in (self.up_btn, self.down_btn, self.stop_btn):
            button.setMinimumHeight(44)
            motion.addWidget(button)
        controls.addLayout(motion)

        query = QHBoxLayout()
        self.status_btn = QPushButton("STATUS")
        self.limits_btn = QPushButton("LIMITS")
        self.servo_test_btn = QPushButton("SERVO TEST")
        for button in (self.status_btn, self.limits_btn, self.servo_test_btn):
            query.addWidget(button)
        controls.addLayout(query)

        servo_row = QHBoxLayout()
        servo_row.addWidget(QLabel("Servo target [deg]"))
        self.servo_spin = QSpinBox()
        self.servo_spin.setRange(0, 195)
        self.servo_spin.setValue(0)
        self.servo_spin.setSuffix("°")
        self.servo_set_btn = QPushButton("Set Servo")
        servo_row.addWidget(self.servo_spin)
        servo_row.addWidget(self.servo_set_btn)
        servo_row.addStretch(1)
        controls.addLayout(servo_row)
        root.addWidget(controls)

        raw_section = SectionFrame("Winch Serial Monitor")
        self.raw_log = QTextEdit()
        self.raw_log.setReadOnly(True)
        self.raw_log.document().setMaximumBlockCount(250)
        raw_section.addWidget(self.raw_log)
        root.addWidget(raw_section, 1)

        self.up_btn.clicked.connect(lambda: self.command_requested.emit("UP"))
        self.down_btn.clicked.connect(lambda: self.command_requested.emit("DOWN"))
        self.stop_btn.clicked.connect(lambda: self.command_requested.emit("STOP"))
        self.status_btn.clicked.connect(lambda: self.command_requested.emit("STATUS"))
        self.limits_btn.clicked.connect(lambda: self.command_requested.emit("LIMITS"))
        self.servo_test_btn.clicked.connect(lambda: self.command_requested.emit("SERVOTEST"))
        self.servo_set_btn.clicked.connect(
            lambda: self.command_requested.emit(f"SERVO {self.servo_spin.value()}"))
        self._refresh()

    def set_active(self, active: bool):
        del active

    def update_health(self, payload: Dict[str, Dict[str, Any]]):
        info = payload.get("/winch/connected", {})
        if not info.get("seen", False) or float(info.get("age", float("inf"))) > 3.0:
            self.connected = False
            self.latest["connected"] = False
            self._refresh()

    def update_telemetry(self, source: str, values: Dict[str, Any], timestamp: float):
        del timestamp
        if source != "winch":
            return
        self.latest.update(values)
        if "connected" in values:
            self.connected = bool(values["connected"])
        raw = values.get("raw")
        if raw:
            self.raw_log.append(str(raw))
        self._refresh()

    def _refresh(self):
        connected = bool(self.latest.get("connected", self.connected))
        self.value_labels["connection"].setText("CONNECTED" if connected else "DISCONNECTED")
        self.value_labels["port"].setText(str(self.latest.get("port", "-")))
        self.value_labels["state"].setText(str(self.latest.get("state", "-")))
        for key in ("top_limit", "bottom_limit"):
            value = self.latest.get(key)
            self.value_labels[key].setText("ACTIVE" if bool(value) else ("CLEAR" if value is not None else "-"))
        pwm = self.latest.get("pwm_pct")
        self.value_labels["pwm_pct"].setText(f"{float(pwm):.1f}" if pwm is not None else "-")
        direction = self.latest.get("direction")
        if direction is None:
            direction_text = "-"
        else:
            direction_text = {1: "UP", -1: "DOWN", 0: "STOP"}.get(int(direction), str(direction))
        self.value_labels["direction"].setText(direction_text)
        servo = self.latest.get("servo_deg")
        self.value_labels["servo_deg"].setText(f"{float(servo):.1f}" if servo is not None else "-")



class ImagePage:
    RENDER_INTERVAL_MS = 33   # ~30 FPS active preview; render only genuinely new ROS frames

    def __init__(self, title: str, topic: str, yaml_path: Optional[Path] = None, telemetry_sources: Sequence[str] = (), fallback_topics: Sequence[str] = ()):
        self.title = title
        self.topic = topic
        self.fallback_topics = tuple(fallback_topics)
        self.telemetry_sources = set(telemetry_sources)
        self.latest_metrics: Dict[str, Any] = {}
        self.settings = ScrollSettings()
        if yaml_path is not None:
            self.settings.insert_widget(YamlParameterEditor(yaml_path, f"{title} YAML"))
        sec = SectionFrame("Live Stream")
        self.state = QLabel(f"Waiting for {topic}")
        self.state.setWordWrap(True)
        self.metrics_label = QLabel("Metrics: -")
        self.metrics_label.setWordWrap(True)
        self.shot_btn = QPushButton("Save PNG")
        sec.addWidget(self.state); sec.addWidget(self.metrics_label); sec.addWidget(self.shot_btn)
        self.settings.insert_widget(sec)
        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title_label = QLabel(title)
        title_label.setObjectName("WorkspaceTitle")
        root.addWidget(title_label)
        self.image_label = QLabel("No image")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 360)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setStyleSheet("background:#161a1f; border:1px solid #444;")
        root.addWidget(self.image_label, 1)
        # Image acquisition buffers (written by ROS callbacks)
        self._pending_image: Optional[QImage] = None
        self._pending_stamp = 0.0
        self._pending_topic = ""
        self._last_primary_stamp = 0.0
        # Sequence counters distinguish a genuinely new ROS frame from the
        # same QImage being revisited by the Qt render timer.
        self._pending_seq = 0
        self._rendered_seq = 0
        self._received_count = 0
        self._ui_active = False
        self._metrics_dirty = False
        self._health_text = ""
        # Render timer can display up to ~30 FPS; duplicate frames are never repainted.
        # It is stopped while this tab is hidden.
        self._render_timer = QTimer(self.content)
        self._render_timer.setInterval(self.RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._do_render)
        self.shot_btn.clicked.connect(self.save_png)

    def set_active(self, active: bool):
        self._ui_active = bool(active)
        if self._ui_active:
            if not self._render_timer.isActive():
                self._render_timer.start()
            self._do_render()
        else:
            self._render_timer.stop()

    def update_image(self, topic: str, image: QImage):
        # Prefer the processed stream. A raw-camera fallback is accepted only
        # when the primary stream has been silent for >1 s, so the operator
        # never gets a blank panel while the alignment node is starting.
        now = time.time()
        if topic == self.topic:
            self._last_primary_stamp = now
        elif topic in self.fallback_topics:
            if self._last_primary_stamp > 0.0 and (now - self._last_primary_stamp) <= 1.0:
                return
        else:
            return
        self._pending_image = image
        self._pending_stamp = now
        self._pending_topic = topic
        self._pending_seq += 1
        self._received_count += 1

    def _do_render(self):
        if not self._ui_active:
            return
        if self._metrics_dirty:
            parts = []
            for key, value in sorted(self.latest_metrics.items()):
                if isinstance(value, float):
                    parts.append(f"{key}={value:.3g}")
                else:
                    parts.append(f"{key}={value}")
            self.metrics_label.setText("Metrics: " + " | ".join(parts[:12]))
            self._metrics_dirty = False
        img = self._pending_image
        if img is None:
            if self._health_text:
                self.state.setText(self._health_text)
            return

        # Render only when a new ROS frame arrived. Previously this function
        # incremented the frame counter every timer tick, so a frozen camera
        # still appeared to advance at ~15 FPS.
        if self._pending_seq != self._rendered_seq:
            pix = QPixmap.fromImage(img)
            self.image_label.setPixmap(
                pix.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.FastTransformation))
            self._rendered_seq = self._pending_seq

        age = max(0.0, time.time() - self._pending_stamp) if self._pending_stamp > 0.0 else float("inf")
        if math.isfinite(age) and age > 1.5:
            self.state.setText(
                f"IMAGE STALE — {self._pending_topic or self.topic} — last frame {age:.1f}s ago — "
                f"frames {self._received_count}")
        elif self._pending_topic == self.topic:
            self.state.setText(
                f"LIVE — {img.width()}×{img.height()} — frames {self._received_count} — age {age:.2f}s")
        else:
            self.state.setText(
                f"FALLBACK — {img.width()}×{img.height()} — {self._pending_topic} — "
                f"frames {self._received_count} — waiting for {self.topic}")

    def update_health(self, payload: Dict[str, Dict[str, Any]]):
        """Expose image-topic freshness in the page even before a frame renders."""
        candidates = (self.topic,) + self.fallback_topics
        best_topic = None
        best_info: Dict[str, Any] = {}
        for topic in candidates:
            info = payload.get(topic, {})
            if int(info.get("count", 0) or 0) <= 0:
                continue
            if best_topic is None or float(info.get("age", float("inf"))) < float(best_info.get("age", float("inf"))):
                best_topic, best_info = topic, info
        if best_topic is None:
            self._health_text = f"WAITING IMAGE — {self.topic}"
            return
        hz = float(best_info.get("hz", 0.0) or 0.0)
        age = float(best_info.get("age", float("inf")))
        detail = str(best_info.get("detail", "") or "")
        if math.isfinite(age) and age < 1.5:
            self._health_text = f"STREAM DETECTED — {best_topic} — {hz:.1f} Hz / age {age:.2f}s"
        elif math.isfinite(age):
            self._health_text = f"IMAGE STALE — {best_topic} — age {age:.1f}s"
        else:
            self._health_text = f"WAITING IMAGE — {best_topic}"
        if detail:
            self.latest_metrics["stream_detail"] = detail
        self.latest_metrics["stream_hz"] = hz
        self.latest_metrics["stream_age_s"] = age if math.isfinite(age) else -1.0
        self._metrics_dirty = True

    def update_telemetry(self, source: str, values: Dict[str, Any], timestamp: float):
        if self.telemetry_sources and source not in self.telemetry_sources:
            return
        if not self.telemetry_sources:
            return
        self.latest_metrics.update(values)
        self._metrics_dirty = True

    def save_png(self):
        img = self._pending_image
        if img is None:
            return
        base = Path("/home/otomasi2/forclift/log/agv_gui/manual_exports")
        base.mkdir(parents=True, exist_ok=True)
        path = base / f"{self.title.replace(' ','_')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        img.save(str(path), "PNG")
        QMessageBox.information(self.content, "Screenshot", str(path))


class GoalPage(QObject):
    send_goal = pyqtSignal(float, float, float)
    send_initial = pyqtSignal(float, float, float)
    cancel = pyqtSignal()

    def __init__(self, repo: MapRepository):
        super().__init__()
        self.repo = repo
        self.active_map_text = "Active navigation map: -"
        self.latest_nav: Dict[str, Any] = {}
        self.settings = ScrollSettings()
        sec = SectionFrame("Pose Command")
        self.x = NoWheelDoubleSpinBox(); self.x.setRange(-1e5, 1e5); self.x.setDecimals(4)
        self.y = NoWheelDoubleSpinBox(); self.y.setRange(-1e5, 1e5); self.y.setDecimals(4)
        self.yaw = NoWheelDoubleSpinBox(); self.yaw.setRange(-360, 360); self.yaw.setDecimals(3)
        form = QFormLayout(); form.addRow("X [m]", self.x); form.addRow("Y [m]", self.y); form.addRow("Yaw [deg]", self.yaw)
        sec.addLayout(form)
        self.goal_btn = QPushButton("Send Goal")
        self.initial_btn = QPushButton("Set Initial Pose")
        self.cancel_btn = QPushButton("Cancel Navigation")
        sec.addWidget(self.goal_btn); sec.addWidget(self.initial_btn); sec.addWidget(self.cancel_btn)
        self.settings.insert_widget(sec)
        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("Goal / Initial Pose / End-to-End Control")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)
        self.summary = QTextEdit(); self.summary.setReadOnly(True)
        root.addWidget(self.summary, 1)
        self.goal_btn.clicked.connect(self._goal)
        self.initial_btn.clicked.connect(self._initial)
        self.cancel_btn.clicked.connect(self.cancel.emit)
        self.refresh_map_status()

    def refresh_map_status(self):
        active = self.repo.active_navigation_yaml()
        slot = self.repo.active_slot()
        self.set_active_map(str(active or ""), slot, source="latest_map pointer fallback")

    def set_active_map(self, path: str, slot: Optional[int], source: str = "map_server"):
        self.active_map_text = (
            f"Active navigation map ({source}): {('Map '+str(slot)) if slot else 'OTHER/NONE'}\n{path or '-'}")
        self._render_summary()

    def update_telemetry(self, source: str, values: Dict[str, Any], timestamp: float):
        if source != "navigation":
            return
        self.latest_nav.update(values)
        self._render_summary()

    def _render_summary(self):
        lines = [self.active_map_text, "",
                 "Goal commands use frame 'map'. A Map 1/2/3 click is published to Nav2 only when that same map slot is currently active in map_server; inactive-map clicks remain local GUI markers."]
        if self.latest_nav:
            lines.extend(["", "Live Navigation Metrics:"])
            for key, value in sorted(self.latest_nav.items()):
                if isinstance(value, float):
                    lines.append(f"  {key}: {value:.6g}")
                else:
                    lines.append(f"  {key}: {value}")
        self.summary.setPlainText("\n".join(lines))

    def set_pose_from_map(self, x: float, y: float, yaw: float):
        self.x.setValue(x); self.y.setValue(y); self.yaw.setValue(math.degrees(yaw))

    def _goal(self):
        self.send_goal.emit(self.x.value(), self.y.value(), math.radians(self.yaw.value()))
    def _initial(self):
        self.send_initial.emit(self.x.value(), self.y.value(), math.radians(self.yaw.value()))


class ExperimentsPage:
    def __init__(self, manager: ExperimentManager):
        self.manager = manager
        self.settings = ScrollSettings()
        sec = SectionFrame("Experiment")
        self.name = QLineEdit("Manual AGV Test")
        self.subsystem = QComboBox(); self.subsystem.addItems(["Mapping", "LiDAR", "IMU", "EKF", "AMCL", "Smac", "MPPI", "Perception", "EndToEnd"])
        self.map_id = QComboBox(); self.map_id.addItems(["", "Map 1", "Map 2", "Map 3"])
        form = QFormLayout(); form.addRow("Name", self.name); form.addRow("Subsystem", self.subsystem); form.addRow("Map", self.map_id)
        sec.addLayout(form)
        self.start_btn = QPushButton("Start Recording")
        self.mark_btn = QPushButton("MARK")
        self.stop_btn = QPushButton("Stop & Save")
        sec.addWidget(self.start_btn); sec.addWidget(self.mark_btn); sec.addWidget(self.stop_btn)
        self.state = QLabel("Recording: OFF")
        sec.addWidget(self.state)
        self.settings.insert_widget(sec)
        self.content = QWidget()
        root = QVBoxLayout(self.content)
        title = QLabel("Experiments")
        title.setObjectName("WorkspaceTitle")
        root.addWidget(title)
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Session", "Path"])
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table, 1)
        self.start_btn.clicked.connect(self.start)
        self.mark_btn.clicked.connect(lambda: self.manager.mark("MARK"))
        self.stop_btn.clicked.connect(self.stop)
        self.refresh()

    def start(self):
        if self.manager.current is not None:
            QMessageBox.warning(self.content, "Experiment", "A recording session is already active.")
            return
        session = self.manager.start(self.name.text().strip() or "AGV Test", self.subsystem.currentText(), self.map_id.currentText())
        self.state.setText(f"Recording: {session.test_id}")
        self.refresh()

    def stop(self):
        session = self.manager.stop("COMPLETED")
        self.state.setText("Recording: OFF")
        if session:
            QMessageBox.information(self.content, "Experiment saved", str(session.root))
        self.refresh()

    def append_telemetry(self, source: str, values: Dict[str, Any], timestamp: float):
        if self.manager.current is None:
            return
        row = {"source": source}
        row.update(values)
        self.manager.append(row)

    def refresh(self):
        sessions = self.manager.list_sessions()
        self.table.setRowCount(len(sessions))
        for row, path in enumerate(sessions):
            self.table.setItem(row, 0, QTableWidgetItem(path.name))
            self.table.setItem(row, 1, QTableWidgetItem(str(path)))
