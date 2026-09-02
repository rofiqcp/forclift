#!/usr/bin/env python3
"""Qt-first lazy ROS bridge loader.

This module deliberately imports *no ROS packages*.  The PyQt interface can
therefore appear even when rclpy/custom interfaces are broken, slow to import,
or not sourced.  The real ROS bridge is imported in a daemon background thread
and attached after the GUI event loop is already alive.
"""
from __future__ import annotations

import importlib
import queue
import threading
import traceback
from typing import Optional

from PyQt5.QtCore import QObject, QTimer, pyqtSignal


class RosBridgeProxy(QObject):
    health = pyqtSignal(object)
    telemetry = pyqtSignal(str, object, float)
    pose = pyqtSignal(str, float, float, float)
    scan_points = pyqtSignal(object)
    occupancy_grid = pyqtSignal(str, object)
    path = pyqtSignal(str, object)
    trajectories = pyqtSignal(object)
    image = pyqtSignal(str, object)
    tf_status = pyqtSignal(object)
    log = pyqtSignal(str)
    ros_state = pyqtSignal(bool)
    active_map = pyqtSignal(str)
    system_state = pyqtSignal(object)

    def __init__(self, parent=None, import_timeout_ms: int = 5000):
        super().__init__(parent)
        self._backend = None
        self._load_queue: "queue.Queue[tuple]" = queue.Queue()
        self._loader: Optional[threading.Thread] = None
        self._timed_out = False
        self._image_interest = None

        self._poll = QTimer(self)
        self._poll.setInterval(100)
        self._poll.timeout.connect(self._poll_loader)

        self._timeout = QTimer(self)
        self._timeout.setSingleShot(True)
        self._timeout.setInterval(max(1000, int(import_timeout_ms)))
        self._timeout.timeout.connect(self._import_slow)

    @property
    def node(self):
        """Expose the real BridgeNode when connected.

        MainWindow historically checked ``self.ros.node``.  The lazy proxy
        originally omitted this compatibility property, which caused a timer
        traceback every 800 ms even though the GUI remained visible.
        """
        backend = self._backend
        return getattr(backend, "node", None) if backend is not None else None

    @property
    def online(self) -> bool:
        return self.node is not None

    def start(self):
        if self._backend is not None:
            return
        if self._loader is not None and self._loader.is_alive():
            return
        self.ros_state.emit(False)
        self.log.emit("GUI ready. Loading ROS bridge asynchronously…")
        self._loader = threading.Thread(target=self._load_worker, name="agv-gui-ros-import", daemon=True)
        self._loader.start()
        self._poll.start()
        self._timeout.start()

    def _load_worker(self):
        try:
            module = importlib.import_module("navigation_gui.ros_bridge")
            bridge_cls = getattr(module, "RosBridge")
            self._load_queue.put(("ok", bridge_cls))
        except BaseException:
            self._load_queue.put(("error", traceback.format_exc()))

    def _import_slow(self):
        if self._backend is not None:
            return
        self._timed_out = True
        self.ros_state.emit(False)
        self.log.emit(
            "ROS bridge import is taking >5 s. GUI remains fully usable in OFFLINE mode; "
            "ROS connection will attach automatically if import later succeeds."
        )

    def _poll_loader(self):
        try:
            state, payload = self._load_queue.get_nowait()
        except queue.Empty:
            if self._loader is not None and not self._loader.is_alive() and self._backend is None:
                self._poll.stop()
            return

        self._timeout.stop()
        self._poll.stop()
        if state != "ok":
            self.ros_state.emit(False)
            self.log.emit("ROS bridge unavailable; GUI stays OFFLINE.\n" + str(payload))
            return

        try:
            backend = payload(None)
            self._attach_backend(backend)
            self._backend = backend
            backend.set_image_interest(self._image_interest)
            backend.start()
            self.log.emit("ROS bridge loaded; connecting to ROS 2 graph…")
        except BaseException:
            self._backend = None
            self.ros_state.emit(False)
            self.log.emit("ROS bridge construction failed; GUI stays OFFLINE.\n" + traceback.format_exc())

    def _attach_backend(self, backend):
        backend.health.connect(self.health.emit)
        backend.telemetry.connect(self.telemetry.emit)
        backend.pose.connect(self.pose.emit)
        backend.scan_points.connect(self.scan_points.emit)
        backend.occupancy_grid.connect(self.occupancy_grid.emit)
        backend.path.connect(self.path.emit)
        backend.trajectories.connect(self.trajectories.emit)
        backend.image.connect(self.image.emit)
        backend.tf_status.connect(self.tf_status.emit)
        backend.log.connect(self.log.emit)
        backend.ros_state.connect(self.ros_state.emit)
        backend.active_map.connect(self.active_map.emit)
        backend.system_state.connect(self.system_state.emit)

    def set_image_interest(self, topic):
        self._image_interest = topic or None
        backend = self._backend
        if backend is not None:
            backend.set_image_interest(self._image_interest)

    def shutdown(self):
        self._poll.stop()
        self._timeout.stop()
        backend = self._backend
        self._backend = None
        if backend is not None:
            try:
                backend.shutdown()
            except Exception as exc:
                self.log.emit(f"ROS bridge shutdown notice: {exc}")

    def publish_goal(self, x: float, y: float, yaw: float):
        if self._backend is None:
            self.log.emit("Goal not sent: ROS bridge is OFFLINE")
            return
        self._backend.publish_goal(x, y, yaw)

    def publish_initial_pose(self, x: float, y: float, yaw: float,
                             covariance_xy: float = 0.25, covariance_yaw: float = 0.0685):
        if self._backend is None:
            self.log.emit("Initial pose not sent: ROS bridge is OFFLINE")
            return
        self._backend.publish_initial_pose(x, y, yaw, covariance_xy, covariance_yaw)

    def cancel_navigation(self):
        if self._backend is None:
            self.log.emit("Cancel not sent: ROS bridge is OFFLINE")
            return
        self._backend.cancel_navigation()

    def publish_winch_command(self, command: str):
        if self._backend is None:
            self.log.emit("Winch command not sent: ROS bridge is OFFLINE")
            return
        self._backend.publish_winch_command(command)
