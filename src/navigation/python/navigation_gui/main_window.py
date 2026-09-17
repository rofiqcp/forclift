#!/usr/bin/env python3
"""Main PyQt5 Autonomous Vehicle Interface window."""
from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QDockWidget,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QStackedWidget,
    QStatusBar,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .experiment_manager import ExperimentManager
from .map_model import MapRepository
from .pages import (
    ConnectionPage,
    CostmapPage,
    ExperimentsPage,
    GoalPage,
    GroundTruthPage,
    ImagePage,
    MapComparisonPage,
    MapPage,
    PageBundle,
    SubsystemPage,
    TimingPage,
    WinchPage,
)
from .ros_proxy import RosBridgeProxy
from .widgets import ScrollSettings, SectionFrame, YamlParameterEditor


WORKSPACE = Path(os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or (Path.home() / 'forclift')).expanduser()
NAV_SRC = WORKSPACE / "src" / "navigation"
ESC_SRC = WORKSPACE / "src" / "esc"
YOLO_SRC = WORKSPACE / "src" / "yolo_obstacle_detection_ros2"


class NavigationButton(QPushButton):
    def __init__(self, icon_text: str, full_text: str, parent=None):
        super().__init__(parent)
        self.icon_text = icon_text
        self.full_text = full_text
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(38)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_expanded(self, expanded: bool):
        self.setText(f"{self.icon_text}   {self.full_text}" if expanded else self.icon_text)
        self.setToolTip(self.full_text)


class SimpleInfoPage:
    def __init__(self, title: str, text: str, yaml_path: Optional[Path] = None):
        self.settings = ScrollSettings()
        if yaml_path is not None:
            self.settings.insert_widget(YamlParameterEditor(yaml_path, f"{title} YAML"))
        sec = SectionFrame(title)
        lbl = QLabel(text)
        lbl.setWordWrap(True)
        sec.addWidget(lbl)
        self.settings.insert_widget(sec)
        self.content = QWidget()
        root = QVBoxLayout(self.content)
        head = QLabel(title)
        head.setObjectName("WorkspaceTitle")
        root.addWidget(head)
        body = QTextEdit()
        body.setReadOnly(True)
        body.setPlainText(text)
        root.addWidget(body, 1)


class AutonomousVehicleWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Autonomous Vehicle Interface — Sekolah Vokasi UNDIP")
        screen = QApplication.primaryScreen().availableGeometry()
        width = max(1180, int(screen.width() * 0.96))
        height = max(720, int(screen.height() * 0.92))
        self.resize(min(width, screen.width()), min(height, screen.height()))

        self.map_repo = MapRepository(str(WORKSPACE))
        self.experiments = ExperimentManager(str(WORKSPACE))
        self.ros = RosBridgeProxy(self)
        self.pages: List[PageBundle] = []
        self.page_by_name: Dict[str, PageBundle] = {}
        self.nav_buttons: List[NavigationButton] = []
        self.nav_expanded = True
        self.active_nav_slot: Optional[int] = None
        self.active_nav_path: Optional[Path] = None
        self._map_mtimes: Dict[int, Optional[int]] = {}
        self._activated_pages = set()

        self._build_ui()
        self._build_pages()
        self._wire_ros()
        self._apply_style()
        self._select_page(0)

        self.map_watch_timer = QTimer(self)
        self.map_watch_timer.timeout.connect(self._poll_map_files)
        self.map_watch_timer.start(5000)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status_bar)
        self.status_timer.start(1500)


    def start_runtime(self):
        """Start optional ROS only after the Qt window/event loop is visible."""
        self._log("GUI window is ready. ROS bridge will load asynchronously.")
        self.ros.start()

    # ---------------- layout ----------------
    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self.main_splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(self.main_splitter, 1)

        # LEFT: branding, navigation rail, contextual settings
        self.left_panel = QFrame()
        self.left_panel.setObjectName("LeftPanel")
        left_layout = QVBoxLayout(self.left_panel)
        left_layout.setContentsMargins(10, 10, 10, 10)
        left_layout.setSpacing(8)

        brand = QFrame()
        brand.setObjectName("BrandFrame")
        brand_layout = QHBoxLayout(brand)
        brand_layout.setContentsMargins(10, 8, 10, 8)
        logo = QLabel()
        logo.setAlignment(Qt.AlignCenter)
        logo.setObjectName("LogoBox")
        logo.setFixedSize(72, 72)
        logo.setToolTip("Universitas Diponegoro")
        logo.setAccessibleName("Logo Universitas Diponegoro")
        # V30: resolve the UNDIP logo robustly both from the source tree and
        # from the installed package.  This avoids falling back to the old
        # text-only "UNDIP" box when the GUI is launched from install/.
        logo_candidates = [
            Path(__file__).resolve().parent / "assets" / "logo_undip.png",
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            logo_candidates.append(
                Path(get_package_share_directory("navigation")) / "assets" / "logo_undip.png"
            )
        except Exception:
            pass

        logo_pixmap = QPixmap()
        for logo_path in logo_candidates:
            if logo_path.is_file():
                candidate = QPixmap(str(logo_path))
                if not candidate.isNull():
                    logo_pixmap = candidate
                    break

        if not logo_pixmap.isNull():
            logo.setPixmap(logo_pixmap.scaled(66, 66, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            # Visible diagnostic fallback only if both source/install assets
            # are genuinely missing.
            logo.setText("UNDIP")
            logo.setToolTip("UNDIP logo asset missing")
        brand_text = QVBoxLayout()
        title = QLabel("Autonomous Vehicle Interface")
        title.setObjectName("BrandTitle")
        sub1 = QLabel("Sekolah Vokasi")
        sub2 = QLabel("Universitas Diponegoro")
        sub1.setObjectName("BrandSub")
        sub2.setObjectName("BrandSub")
        brand_text.addWidget(title)
        brand_text.addWidget(sub1)
        brand_text.addWidget(sub2)
        brand_layout.addWidget(logo)
        brand_layout.addLayout(brand_text, 1)
        left_layout.addWidget(brand)

        left_body = QSplitter(Qt.Horizontal)
        left_layout.addWidget(left_body, 1)

        rail_container = QWidget()
        rail_layout = QVBoxLayout(rail_container)
        rail_layout.setContentsMargins(0, 0, 0, 0)
        rail_layout.setSpacing(4)
        self.menu_toggle = QPushButton("☰   Minimize")
        self.menu_toggle.setObjectName("MenuToggle")
        self.menu_toggle.clicked.connect(self._toggle_nav)
        rail_layout.addWidget(self.menu_toggle)
        self.nav_scroll = QScrollArea()
        self.nav_scroll.setWidgetResizable(True)
        self.nav_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.nav_body = QWidget()
        self.nav_layout = QVBoxLayout(self.nav_body)
        self.nav_layout.setContentsMargins(0, 0, 0, 0)
        self.nav_layout.setSpacing(3)
        self.nav_layout.addStretch(1)
        self.nav_scroll.setWidget(self.nav_body)
        rail_layout.addWidget(self.nav_scroll, 1)
        left_body.addWidget(rail_container)

        self.settings_stack = QStackedWidget()
        left_body.addWidget(self.settings_stack)
        left_body.setStretchFactor(0, 0)
        left_body.setStretchFactor(1, 1)
        left_body.setSizes([150, 380])
        self.left_body_splitter = left_body

        # RIGHT: context workspace
        self.workspace_stack = QStackedWidget()
        self.workspace_stack.setObjectName("WorkspaceStack")
        self.main_splitter.addWidget(self.left_panel)
        self.main_splitter.addWidget(self.workspace_stack)
        self.main_splitter.setStretchFactor(0, 2)
        self.main_splitter.setStretchFactor(1, 4)
        self.main_splitter.setSizes([max(420, int(self.width() * 0.34)), max(760, int(self.width() * 0.66))])

        self.setStatusBar(QStatusBar(self))
        self.status_label = QLabel("ROS: STARTING | MAP DISPLAY: - | MAP NAV: - | TF: - | REC: OFF")
        self.statusBar().addPermanentWidget(self.status_label, 1)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        # QTextEdit itself has no setMaximumBlockCount() in PyQt5.
        # The limit belongs to the underlying QTextDocument. Keeping the
        # console bounded prevents an hours-long ROS session from growing
        # memory without crashing GUI startup on Ubuntu 22.04 / PyQt5.
        self.console.setUndoRedoEnabled(False)
        self.console.document().setMaximumBlockCount(500)
        dock = QDockWidget("System Messages", self)
        dock.setWidget(self.console)
        dock.setAllowedAreas(Qt.BottomDockWidgetArea)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        dock.setVisible(False)
        self.console_dock = dock

    def _build_pages(self):
        # Core pages
        self.connection = ConnectionPage(self.map_repo)
        self._add_page("Connection", "CN", self.connection.settings, self.connection.content, "SYSTEM")

        self.map_pages: Dict[int, MapPage] = {}
        for source in self.map_repo.sources:
            page = MapPage(source, self.map_repo)
            self.map_pages[source.slot] = page
            self._add_page(f"Map {source.slot}", f"M{source.slot}", page.settings, page.content, "MAP")

        self.map_compare = MapComparisonPage(self.map_repo, self.map_pages)
        self._add_page("Map Comparison", "MC", self.map_compare.settings, self.map_compare.content, "MAP")

        self.ground_truth = GroundTruthPage(NAV_SRC / "config" / "gui" / "ground_truth.yaml")
        self._add_page("Ground Truth", "GT", self.ground_truth.settings, self.ground_truth.content, "MAP")

        # Localization / motion
        self.lidar_page = SubsystemPage(
            "LiDAR Calibration & Test", NAV_SRC / "config" / "lidar.yaml",
            ["valid_bins", "scan_bins", "invalid_ratio_pct", "range_mean", "range_std",
             "safety_valid_ratio_pct", "scan_match_quality"],
            ["lidar", "lidar_safety", "scan_match_quality"],
            tests=["Scan Rate Test", "Range Distribution Test", "Invalid/Inf Ratio", "Valid Bin Count",
                   "Static Environment Stability", "Starburst/Outlier Test", "Scan Match Quality"],
            health_topics=["/scan_nav", "/scan_safety", "/lidar/safety_healthy", "/scan_match_quality"])
        self._add_page("LiDAR", "LD", self.lidar_page.settings, self.lidar_page.content, "LOCALIZATION")

        self.odom_page = SubsystemPage(
            "LiDAR Odometry", NAV_SRC / "config" / "hector.yaml",
            ["x", "y", "yaw", "vx", "wz", "pose_cov_x", "pose_cov_y", "pose_cov_yaw", "scan_match_quality"],
            ["lidar_odom", "scan_match_quality"],
            tests=["Stationary Drift", "Straight Line", "Rotation", "Square / Closed Loop", "Repeatability", "Return-to-Origin"],
            health_topics=["/lidar/odom", "/scan_match_quality"])
        self._add_page("LiDAR Odometry", "LO", self.odom_page.settings, self.odom_page.content, "LOCALIZATION")

        self.imu_page = SubsystemPage(
            "IMU Calibration & Test", NAV_SRC / "config" / "imu.yaml",
            ["ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw", "mx", "my", "mz"],
            ["imu", "imu_gyro", "imu_accel", "imu_euler", "imu_mag", "imu_status"],
            tests=["Stationary Gyro Bias", "Accelerometer Offset", "Gyro Noise", "Accelerometer Noise",
                   "Yaw Drift", "Sampling Frequency", "Packet Drop"],
            health_topics=["/imu/data", "/imu/gyro", "/imu/accel", "/imu/euler"])
        self._add_page("IMU", "IM", self.imu_page.settings, self.imu_page.content, "LOCALIZATION")

        self.ekf_page = SubsystemPage(
            "EKF Calibration & Test", NAV_SRC / "config" / "ekf_autonomous.yaml",
            ["x", "y", "yaw", "vx", "wz", "pose_cov_x", "pose_cov_y", "pose_cov_yaw"], ["ekf"],
            tests=["Stationary", "Straight", "Turn", "Closed Loop", "Input Dropout", "Repeatability"],
            health_topics=["/odometry/filtered"])
        self._add_page("EKF", "EK", self.ekf_page.settings, self.ekf_page.content, "LOCALIZATION")

        self.amcl_page = SubsystemPage(
            "AMCL / Global Localization", NAV_SRC / "config" / "nav2_ackermann.yaml",
            ["x", "y", "yaw", "cov_x", "cov_y", "cov_yaw"], ["amcl"],
            tests=["Initial Pose Convergence", "Static Localization", "Known Point Accuracy", "Repeatability", "Relocalization", "Closed Loop"],
            health_topics=["/amcl_pose"])
        self._add_page("AMCL", "AM", self.amcl_page.settings, self.amcl_page.content, "LOCALIZATION")

        self.tf_page = TimingPage()
        self._add_page("TF & Timing", "TF", self.tf_page.settings, self.tf_page.content, "LOCALIZATION")

        self.esc_page = SubsystemPage(
            "ESC / Motion Calibration", ESC_SRC / "config" / "ackermann_1_board.yaml",
            ["/esc/speed", "/esc/drive_target_mps", "/esc/drive_actual_mps", "/esc/steering_target_rad",
             "/esc/steering_actual_rad", "/esc/yaw_rate_actual_rps", "battery_voltage", "battery_current", "temperature"],
            ["esc", "esc_state", "esc_odom"],
            tests=["Command vs Actual Speed", "Steering Target vs Actual", "Straight Line", "Minimum Moving Speed",
                   "Acceleration", "Deceleration", "Command Latency", "Deadband", "Repeatability"],
            health_topics=["/esc/feedback_valid", "/esc/ready", "/esc/drive/connected", "/esc/steer/connected"])
        self._add_page("ESC / Motion", "ES", self.esc_page.settings, self.esc_page.content, "LOCALIZATION")

        self.winch_page = WinchPage(ESC_SRC / "config" / "winch.yaml")
        self._add_page("Winch", "WC", self.winch_page.settings, self.winch_page.content, "ACTUATOR")

        # Navigation
        self.costmap_page = CostmapPage(NAV_SRC / "config" / "nav2_ackermann.yaml")
        self._add_page("Global Costmap", "GC", self.costmap_page.settings, self.costmap_page.content, "NAVIGATION")

        self.smac_page = SubsystemPage(
            "Smac Hybrid-A*", NAV_SRC / "config" / "nav2_ackermann.yaml",
            ["planning_time_s", "path_length", "pose_count", "reverse_segments", "heading_change_deg", "max_curvature_1pm"],
            ["planner"],
            tests=["Planning Time", "Path Length", "Reverse Segments", "Heading Change", "Curvature", "Planning Success Rate"],
            health_topics=["/smac_plan"])
        self._add_page("Smac Hybrid-A*", "SM", self.smac_page.settings, self.smac_page.content, "NAVIGATION")

        self.mppi_page = SubsystemPage(
            "MPPI Ackermann", NAV_SRC / "config" / "nav2_ackermann.yaml",
            ["path_length", "pose_count", "trajectory_count", "trajectory_point_count",
             "/cmd_vel_nav_raw:vx", "/cmd_vel_nav_raw:wz"], ["mppi_plan", "mppi", "cmd"],
            tests=["Straight Path Tracking", "Curve Tracking", "Reverse Path", "Narrow Path", "Goal Approach", "Repeatability"],
            health_topics=["/transformed_global_plan", "/trajectories", "/cmd_vel_nav_raw"])
        self._add_page("MPPI", "MP", self.mppi_page.settings, self.mppi_page.content, "NAVIGATION")

        self.command_page = SubsystemPage(
            "Command Pipeline & Safety", NAV_SRC / "config" / "collision_monitor.yaml",
            [
                "/cmd_vel_nav_raw:vx", "/cmd_vel_nav_raw:wz",
                "/cmd_vel_nav_smoothed:vx", "/cmd_vel_nav_smoothed:wz",
                "/cmd_vel_collision_safe:vx", "/cmd_vel_collision_safe:wz",
                "/cmd_vel:vx", "/cmd_vel:wz", "/cmd_vel/actuator:vx", "/cmd_vel/actuator:wz",
            ], ["cmd", "safety"],
            tests=["Pipeline Latency", "Raw vs Smoothed", "Saturation", "Deadband", "Stop Response", "Command Dropout", "Safety Intervention"],
            health_topics=["/cmd_vel_nav_raw", "/cmd_vel_nav_smoothed", "/cmd_vel_collision_safe", "/cmd_vel", "/cmd_vel/actuator", "/sensor_guard/healthy"])
        self._add_page("Command & Safety", "CS", self.command_page.settings, self.command_page.content, "NAVIGATION")

        self.goal_page = GoalPage(self.map_repo)
        self._add_page("Goal / End-to-End", "GO", self.goal_page.settings, self.goal_page.content, "NAVIGATION")

        # Perception
        self.camera_page = ImagePage(
            "Camera — Orbbec Astra RGB", "/camera/color/image_raw",
            YOLO_SRC / "config" / "camera_v4l2.yaml",
            ["camera", "camera_info", "camera_status"],
            fallback_topics=["/obstacle_detection/visualization"])
        self._add_page("Camera", "CA", self.camera_page.settings, self.camera_page.content, "PERCEPTION")

        self.yolo_page = ImagePage(
            "YOLO Detection", "/obstacle_detection/visualization",
            YOLO_SRC / "config" / "yolo_detection.yaml",
            ["yolo", "yolo_view", "yolo_status", "yolo_performance"],
            fallback_topics=["/camera/color/image_raw"])
        self._add_page("YOLO Detection", "YO", self.yolo_page.settings, self.yolo_page.content, "PERCEPTION")

        self.fork_cal_page = ImagePage("Camera / Fork Calibration", "/camera/color/image_raw",
                                       YOLO_SRC / "config" / "alignment_realtime.yaml", ["camera", "camera_info", "camera_status", "alignment"])
        self._add_page("Camera-Fork Calibration", "CF", self.fork_cal_page.settings, self.fork_cal_page.content, "PERCEPTION")

        self.alignment_page = ImagePage(
            "Hole-Block Alignment", "/fork_alignment/image",
            YOLO_SRC / "hole_block_alignment" / "alignment_realtime.yaml",
            ["alignment", "alignment_view"], fallback_topics=["/camera/color/image_raw"])
        self._add_page("Hole-Block Alignment", "HA", self.alignment_page.settings, self.alignment_page.content, "PERCEPTION")

        self.perception_validation = ImagePage(
            "Perception Validation", "/obstacle_detection/visualization", None,
            ["camera", "camera_status", "yolo", "yolo_view", "yolo_status", "yolo_performance", "alignment", "perception"],
            fallback_topics=["/camera/color/image_raw"])
        self._add_page("Perception Validation", "PV", self.perception_validation.settings, self.perception_validation.content, "PERCEPTION")

        # Data / reports
        self.experiment_page = ExperimentsPage(self.experiments)
        self._add_page("Experiments", "EX", self.experiment_page.settings, self.experiment_page.content, "DATA")

        reports = SimpleInfoPage(
            "Reports / Export",
            "Manual plots are exported under /home/otomasi2/forclift/log/agv_gui/manual_exports. "
            "Experiment sessions contain metadata.yaml, configuration snapshots, CSV, plots, and screenshots. "
            "Map Comparison can export its metric table directly to CSV.")
        self._add_page("Reports / Export", "RP", reports.settings, reports.content, "DATA")

        settings_page = SimpleInfoPage(
            "GUI Settings",
            "Interface configuration is stored separately from Nav2 production parameters.",
            NAV_SRC / "config" / "gui" / "interface.yaml")
        self._add_page("Settings", "ST", settings_page.settings, settings_page.content, "DATA")

        # Shared events
        self.ground_truth.points_changed.connect(self._ground_truth_changed)
        self._ground_truth_changed(self.ground_truth.points)
        for slot, page in self.map_pages.items():
            page.goal_requested.connect(lambda x, y, yaw, s=slot: self._map_goal_requested(s, x, y, yaw))
            page.initial_requested.connect(lambda x, y, yaw, s=slot: self._map_initial_requested(s, x, y, yaw))
            page.ground_truth_requested.connect(lambda x, y: self.ground_truth.add_point(x, y))

        self.goal_page.send_goal.connect(self.ros.publish_goal)
        self.goal_page.send_initial.connect(self.ros.publish_initial_pose)
        self.goal_page.cancel.connect(self.ros.cancel_navigation)
        self.winch_page.command_requested.connect(self.ros.publish_winch_command)

    def _add_page(self, name: str, icon_text: str, settings: QWidget, content: QWidget, group: str):
        bundle = PageBundle(name=name, icon_text=icon_text, settings=settings, content=content, group=group)
        self.pages.append(bundle)
        self.page_by_name[name] = bundle
        self.settings_stack.addWidget(settings)
        self.workspace_stack.addWidget(content)

        # Insert group labels before first button of a new group.
        existing_groups = [getattr(btn, "group_name", None) for btn in self.nav_buttons]
        if group not in existing_groups:
            label = QLabel(group)
            label.setObjectName("NavGroup")
            label.setProperty("group_name", group)
            self.nav_layout.insertWidget(self.nav_layout.count() - 1, label)
        button = NavigationButton(icon_text, name)
        button.group_name = group
        button.set_expanded(self.nav_expanded)
        index = len(self.pages) - 1
        button.clicked.connect(lambda _checked=False, i=index: self._select_page(i))
        self.nav_layout.insertWidget(self.nav_layout.count() - 1, button)
        self.nav_buttons.append(button)

    def _select_page(self, index: int):
        if not (0 <= index < len(self.pages)):
            return
        self.settings_stack.setCurrentIndex(index)
        self.workspace_stack.setCurrentIndex(index)
        for i, button in enumerate(self.nav_buttons):
            button.setChecked(i == index)
        self._apply_runtime_visibility(index)
        # Heavy YAML/map I/O is deferred until the selected page is visible.
        QTimer.singleShot(0, lambda i=index: self._activate_page(i))

    def _apply_runtime_visibility(self, index: int):
        """Keep full-rate ROS acquisition, but paint only the visible workspace.

        Hidden maps used to redraw LaserScan/costmaps at sensor rate and every
        hidden plot/image page kept a repaint timer alive.  This method makes UI
        work demand-driven without changing any ROS publisher frequency.
        """
        if not (0 <= index < len(self.pages)):
            return
        active_name = self.pages[index].name

        for slot, page in self.map_pages.items():
            page.set_active(active_name == f"Map {slot}")

        subsystem_pages = {
            "LiDAR": self.lidar_page,
            "LiDAR Odometry": self.odom_page,
            "IMU": self.imu_page,
            "EKF": self.ekf_page,
            "AMCL": self.amcl_page,
            "ESC / Motion": self.esc_page,
            "Winch": self.winch_page,
            "Smac Hybrid-A*": self.smac_page,
            "MPPI": self.mppi_page,
            "Command & Safety": self.command_page,
        }
        for name, page in subsystem_pages.items():
            page.set_active(name == active_name)

        image_pages = {
            "Camera": self.camera_page,
            "YOLO Detection": self.yolo_page,
            "Camera-Fork Calibration": self.fork_cal_page,
            "Hole-Block Alignment": self.alignment_page,
            "Perception Validation": self.perception_validation,
        }
        for name, page in image_pages.items():
            page.set_active(name == active_name)

        image_interest = None
        if active_name in ("Camera", "Camera-Fork Calibration"):
            # One image subscription at a time. Subscribing to both raw RGB and
            # the processed 1280x720 stream doubles DDS deserialization/copy load
            # in Python and can starve the perception callback path on Jetson.
            image_interest = "/camera/color/image_raw"
        elif active_name in ("YOLO Detection", "Perception Validation"):
            image_interest = "/obstacle_detection/visualization"
        elif active_name == "Hole-Block Alignment":
            image_interest = "/fork_alignment/image"
        self.ros.set_image_interest(image_interest)

    def _activate_page(self, index: int):
        if index in self._activated_pages or not (0 <= index < len(self.pages)):
            return
        self._activated_pages.add(index)
        bundle = self.pages[index]
        # Load only YAML editors belonging to the page the operator opened.
        for editor in bundle.settings.findChildren(YamlParameterEditor):
            try:
                if not getattr(editor, "loaded_once", False):
                    editor.reload()
            except Exception as exc:
                self._log(f"YAML deferred load notice ({bundle.name}): {exc}")
        if bundle.name.startswith("Map ") and bundle.name != "Map Comparison":
            try:
                slot = int(bundle.name.split()[-1])
                page = self.map_pages.get(slot)
                if page is not None and not getattr(page, "loaded_once", False):
                    page.reload()
            except Exception as exc:
                self._log(f"Map deferred load notice ({bundle.name}): {exc}")

    def _toggle_nav(self):
        self.nav_expanded = not self.nav_expanded
        self.menu_toggle.setText("☰   Minimize" if self.nav_expanded else "☰")
        for button in self.nav_buttons:
            button.set_expanded(self.nav_expanded)
        self.left_body_splitter.setSizes([150 if self.nav_expanded else 54, 380 if self.nav_expanded else 476])

    # ---------------- ROS wiring ----------------
    def _wire_ros(self):
        self.ros.ros_state.connect(self.connection.set_ros_state)
        self.ros.health.connect(self._health_received)
        self.ros.tf_status.connect(self._tf_received)
        self.ros.pose.connect(self._pose_received)
        self.ros.scan_points.connect(self._scan_received)
        self.ros.path.connect(self._path_received)
        self.ros.trajectories.connect(self._trajectories_received)
        self.ros.occupancy_grid.connect(self._grid_received)
        self.ros.telemetry.connect(self._telemetry_received)
        self.ros.image.connect(self._image_received)
        self.ros.log.connect(self._log)
        self.ros.active_map.connect(self._active_map_received)
        self.ros.system_state.connect(self.connection.update_system_state)

    def _health_received(self, payload):
        self.connection.update_health(payload)
        self.tf_page.update_health(payload)
        for page in (self.lidar_page, self.odom_page, self.imu_page, self.ekf_page,
                     self.amcl_page, self.esc_page, self.winch_page, self.smac_page, self.mppi_page, self.command_page):
            page.update_health(payload)
        for page in (self.camera_page, self.yolo_page, self.fork_cal_page, self.alignment_page, self.perception_validation):
            page.update_health(payload)

    def _tf_received(self, payload):
        self.connection.update_tf(payload)
        self.tf_page.update_tf(payload)

    def _pose_received(self, source: str, x: float, y: float, yaw: float):
        for page in self.map_pages.values():
            page.update_pose(source, x, y, yaw)
        self.ground_truth.update_pose(source, x, y, yaw)

    def _scan_received(self, points):
        for page in self.map_pages.values():
            page.update_scan(points)

    def _path_received(self, topic: str, points):
        for page in self.map_pages.values():
            page.update_path(topic, points)

    def _trajectories_received(self, trajectories):
        for page in self.map_pages.values():
            page.update_trajectories(trajectories)

    def _grid_received(self, topic: str, msg):
        for page in self.map_pages.values():
            page.update_grid(topic, msg)

    def _telemetry_received(self, source: str, values: dict, timestamp: float):
        for page in (self.lidar_page, self.odom_page, self.imu_page, self.ekf_page,
                     self.amcl_page, self.esc_page, self.winch_page, self.smac_page, self.mppi_page, self.command_page):
            page.update_telemetry(source, values, timestamp)
        self.costmap_page.update_telemetry(source, values, timestamp)
        self.goal_page.update_telemetry(source, values, timestamp)
        self.experiment_page.append_telemetry(source, values, timestamp)
        for page in (self.camera_page, self.yolo_page, self.fork_cal_page, self.alignment_page, self.perception_validation):
            page.update_telemetry(source, values, timestamp)

    def _image_received(self, topic: str, image):
        # Only render on the active ImagePage — all others silently discard updates.
        # ImagePage stores the latest frame in _pending_image and renders it
        # at ~15 FPS via its own QTimer, so per-callback rendering is eliminated.
        active = self.pages[self.workspace_stack.currentIndex()].name
        target = None
        if active == "Camera":
            target = self.camera_page
        elif active == "YOLO Detection":
            target = self.yolo_page
        elif active == "Camera-Fork Calibration":
            target = self.fork_cal_page
        elif active == "Hole-Block Alignment":
            target = self.alignment_page
        elif active == "Perception Validation":
            target = self.perception_validation
        if target is not None:
            target.update_image(topic, image)

    def _active_map_received(self, path: str):
        try:
            resolved = Path(path).expanduser().resolve()
        except Exception:
            resolved = Path(path)
        old_slot = self.active_nav_slot
        old_path = self.active_nav_path
        self.active_nav_path = resolved
        self.active_nav_slot = self.map_repo.slot_for_path(resolved)
        if old_slot != self.active_nav_slot or old_path != self.active_nav_path:
            self._log(f"map_server active YAML: {resolved}")
        self.connection.set_active_map(str(resolved), self.active_nav_slot)
        self.goal_page.set_active_map(str(resolved), self.active_nav_slot, source="map_server")
        for page in self.map_pages.values():
            page.set_active_nav_slot(self.active_nav_slot)

    # ---------------- map / ground truth ----------------
    def _poll_map_files(self):
        changed = False
        for source in self.map_repo.sources:
            try:
                mtime = source.yaml_path.stat().st_mtime_ns
            except OSError:
                mtime = None
            if source.slot not in self._map_mtimes:
                self._map_mtimes[source.slot] = mtime
            elif self._map_mtimes[source.slot] != mtime:
                self._map_mtimes[source.slot] = mtime
                changed = True
                self._log(f"New mapping result detected: Map {source.slot}")
        # Before map_server responds, fall back to the V23 latest_map pointer.
        if self.active_nav_path is None:
            old_active = self.active_nav_slot
            self.active_nav_slot = self.map_repo.active_slot()
            if old_active != self.active_nav_slot:
                changed = True
                self._log(f"Active navigation map pointer changed: {self.active_nav_slot or 'OTHER/NONE'}")
            for page in self.map_pages.values():
                page.set_active_nav_slot(self.active_nav_slot)
        if changed:
            self.connection.refresh_maps()
            self.goal_page.refresh_map_status()
            # Do not automatically reload the map document while the operator may
            # be inspecting/annotating it. Instead mark via log; manual reload is explicit.

    def _ground_truth_changed(self, points):
        for page in self.map_pages.values():
            page.set_ground_truth(points)

    def _map_goal_requested(self, slot: int, x: float, y: float, yaw: float):
        """Keep map-tab goals isolated and publish only on the active nav map.

        V56 intentionally treated all saved maps as one shared metric frame. In
        practice that meant a click on Map 1 could immediately become the Nav2
        goal while map_server was actually running Map 3. The planner/path then
        appeared on Map 3, which looked like the goal had leaked across tabs.
        V57 keeps the marker on its source tab and fail-closes command publishing
        when the source slot does not match the active map_server slot.
        """
        self.goal_page.set_pose_from_map(x, y, yaw)
        if self.active_nav_slot != slot:
            active_text = (
                f"Map {self.active_nav_slot}" if self.active_nav_slot is not None
                else "OTHER/NONE"
            )
            self._log(
                f"Goal stored on Map {slot} only: x={x:.3f}, y={y:.3f}, "
                f"yaw={math.degrees(yaw):.1f} deg; active navigation map={active_text}. "
                "Not published to Nav2 to prevent cross-map goal leakage."
            )
            return
        self._log(
            f"Goal Map {slot} -> Nav2: x={x:.3f}, y={y:.3f}, "
            f"yaw={math.degrees(yaw):.1f} deg"
        )
        self.ros.publish_goal(x, y, yaw)

    def _map_initial_requested(self, slot: int, x: float, y: float, yaw: float):
        """Publish initial pose only when the source tab owns the active map."""
        self.goal_page.set_pose_from_map(x, y, yaw)
        if self.active_nav_slot != slot:
            active_text = (
                f"Map {self.active_nav_slot}" if self.active_nav_slot is not None
                else "OTHER/NONE"
            )
            self._log(
                f"Initial pose stored on Map {slot} only: x={x:.3f}, y={y:.3f}, "
                f"yaw={math.degrees(yaw):.1f} deg; active localization map={active_text}. "
                "Not published to prevent cross-map localization."
            )
            return
        self._log(
            f"Initial pose Map {slot} -> localization: x={x:.3f}, y={y:.3f}, "
            f"yaw={math.degrees(yaw):.1f} deg"
        )
        self.ros.publish_initial_pose(x, y, yaw)

    # ---------------- status / logs ----------------
    def _refresh_status_bar(self):
        current = self.pages[self.workspace_stack.currentIndex()].name if self.pages else "-"
        active = self.active_nav_slot if self.active_nav_path is not None else self.map_repo.active_slot()
        recording = "ON" if self.experiments.current else "OFF"
        display = current if current.startswith("Map ") and current[-1:].isdigit() else "-"
        self.status_label.setText(
            f"ROS: {'ONLINE' if self.ros.online else 'OFFLINE'} | "
            f"MAP DISPLAY: {display} | MAP NAV: {('MAP '+str(active)) if active else 'OTHER/NONE'} | "
            f"TF: monitor | REC: {recording}")

    def _log(self, message: str):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        self.console.append(line)
        # agv_gui_launcher.sh tees stdout/stderr into agv_gui_startup.log.
        # Mirror runtime ROS diagnostics there so an OFFLINE card is always
        # explainable from the collected log zip.
        print(f"[AGV-GUI-RUNTIME] {line}", flush=True)

    # ---------------- style ----------------
    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background:#20252b; color:#e8edf2; font-size:12px; }
            QFrame#LeftPanel { background:#1a1f24; border-right:1px solid #39414a; }
            QFrame#BrandFrame { background:#242b32; border:1px solid #39414a; border-radius:10px; }
            QLabel#LogoBox { background:#f5f5f5; color:#5f1c34; border:1px solid #48535e; border-radius:10px; font-size:14px; font-weight:700; padding:3px; }
            QLabel#BrandTitle { font-size:17px; font-weight:700; }
            QLabel#BrandSub { color:#bfc8d1; }
            QLabel#WorkspaceTitle { font-size:19px; font-weight:700; padding:5px 0; }
            QLabel#SectionTitle { font-size:13px; font-weight:700; }
            QLabel#Muted { color:#9da8b3; font-size:10px; }
            QLabel#NavGroup { color:#80909f; font-size:10px; font-weight:700; padding:8px 6px 2px 6px; }
            QFrame#SectionFrame { background:#252c33; border:1px solid #39414a; border-radius:9px; }
            QPushButton { background:#303842; border:1px solid #48535e; border-radius:7px; padding:7px 9px; }
            QPushButton:hover { background:#39444f; }
            QPushButton:checked { background:#5f1c34; border-color:#8d3453; }
            QPushButton#MenuToggle { font-weight:700; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit, QTableWidget {
                background:#191e23; border:1px solid #414b55; border-radius:5px; selection-background-color:#6c2844;
            }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { padding:5px; min-height:22px; }
            QHeaderView::section { background:#2a323a; color:#dfe6ec; padding:6px; border:0; border-right:1px solid #414b55; }
            QScrollArea { border:0; }
            QTabWidget::pane { border:1px solid #3a444d; }
            QTabBar::tab { background:#2b333b; padding:7px 12px; }
            QTabBar::tab:selected { background:#5f1c34; }
            QSplitter::handle { background:#39414a; }
            QStatusBar { background:#15191d; color:#cbd5df; }
        """)

    def closeEvent(self, event):  # noqa: N802
        try:
            if self.experiments.current is not None:
                self.experiments.stop("GUI_CLOSED")
            self.ros.shutdown()
        finally:
            super().closeEvent(event)


def main():
    import sys
    try:
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    except Exception:
        pass
    app = QApplication(sys.argv)
    app.setApplicationName("Autonomous Vehicle Interface")

    # Show a real Qt bootstrap window *before* constructing the engineering
    # pages. This proves the display connection is alive and prevents a heavy
    # map/YAML operation from looking like a dead launch.
    bootstrap = QWidget()
    bootstrap.setWindowTitle("Autonomous Vehicle Interface — Starting")
    layout = QVBoxLayout(bootstrap)
    label = QLabel("Autonomous Vehicle Interface\nLoading interface…")
    label.setAlignment(Qt.AlignCenter)
    label.setStyleSheet("font-size:20px; font-weight:600; padding:30px;")
    layout.addWidget(label)
    bootstrap.resize(520, 180)
    bootstrap.show()
    bootstrap.raise_()
    bootstrap.activateWindow()
    app.processEvents()

    window = AutonomousVehicleWindow()
    window.show()
    window.raise_()
    window.activateWindow()
    bootstrap.close()
    QTimer.singleShot(0, window.start_runtime)
    return app.exec_()
