#!/usr/bin/env python3
"""Reusable Qt widgets for the Autonomous Vehicle Interface."""
from __future__ import annotations

import ast
import csv
import math
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QEvent, QObject, QPointF, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


from .yaml_store import YamlStore, iter_scalar_items


class NoWheelSpinBox(QSpinBox):
    def wheelEvent(self, event):  # noqa: N802
        event.ignore()


class NoWheelDoubleSpinBox(QDoubleSpinBox):
    def wheelEvent(self, event):  # noqa: N802
        event.ignore()


class SectionFrame(QFrame):
    """Compact titled section used in the left settings panel."""

    def __init__(self, title: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("SectionFrame")
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.layout.setSpacing(7)
        title_label = QLabel(title)
        title_label.setObjectName("SectionTitle")
        self.layout.addWidget(title_label)

    def addWidget(self, widget: QWidget, stretch: int = 0):  # noqa: N802
        self.layout.addWidget(widget, stretch)

    def addLayout(self, layout):  # noqa: N802
        self.layout.addLayout(layout)


class StatusPill(QLabel):
    STATES = {
        "healthy": ("HEALTHY", "#1d9b59"),
        "degraded": ("DEGRADED", "#d39b24"),
        "error": ("ERROR", "#c74b4b"),
        "disabled": ("DISABLED", "#777777"),
        "unknown": ("UNKNOWN", "#777777"),
    }

    def __init__(self, state: str = "unknown", parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(78)
        self.set_state(state)

    def set_state(self, state: str, text: Optional[str] = None):
        label, color = self.STATES.get(state, self.STATES["unknown"])
        self.setText(text or label)
        # Keep QSS deliberately simple and syntactically valid on Qt 5.15.
        # The previous string ended with two closing braces and produced
        # "Could not parse stylesheet" warnings on every status update.
        self.setStyleSheet(
            f"background-color: {color}; color: white; border-radius: 9px; "
            "padding: 3px 8px; font-weight: bold;"
        )


class TopicTable(QTableWidget):
    HEADERS = ["Topic", "State", "Hz", "Age [s]", "Count", "Type/Frame"]

    def __init__(self, topics: Sequence[str], parent: Optional[QWidget] = None):
        super().__init__(0, len(self.HEADERS), parent)
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setAlternatingRowColors(True)
        self._rows: Dict[str, int] = {}
        for topic in topics:
            self.ensure_topic(topic)
        self.resizeColumnsToContents()
        self.horizontalHeader().setStretchLastSection(True)

    def ensure_topic(self, topic: str) -> int:
        if topic in self._rows:
            return self._rows[topic]
        row = self.rowCount()
        self.insertRow(row)
        self._rows[topic] = row
        for col, value in enumerate([topic, "UNKNOWN", "-", "-", "0", ""]):
            self.setItem(row, col, QTableWidgetItem(str(value)))
        return row

    def update_health(self, payload: Dict[str, Dict[str, Any]]):
        for topic, info in payload.items():
            row = self.ensure_topic(topic)
            age = info.get("age", float("inf"))
            hz = info.get("hz", 0.0)
            count = int(info.get("count", 0) or 0)
            static_topics = {"/map", "/nav_map", "/camera/color/status", "/obstacle_detection/status"}
            event_topics = {
                "/amcl_pose", "/smac_plan", "/transformed_global_plan",
                "/trajectories", "/goal_pose", "/initialpose_safe",
                "/navigate_to_pose/_action/status",
            }
            if count <= 0:
                state = "IDLE" if topic in event_topics else "NO DATA"
            elif topic in static_topics:
                state = "LATCHED"
            elif topic in event_topics and age >= 1.5:
                state = "IDLE"
            else:
                state = "HEALTHY" if age < 1.5 else ("DEGRADED" if age < 4.0 else "STALE")
            values = [
                topic,
                state,
                f"{hz:.2f}" if hz else "-",
                f"{age:.2f}" if math.isfinite(age) else "-",
                str(count),
                str(info.get("detail", "")),
            ]
            for col, value in enumerate(values):
                item = self.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    self.setItem(row, col, item)
                item.setText(value)


class RingPlot(QWidget):
    """Fast realtime plot with CSV/PNG export.

    Renders with the lightweight built-in QPainter backend (no pyqtgraph
    dependency). Up to 8 series are drawn as colored polylines over a scrolling
    time window. CPU cost is bounded by a fixed paint rate, independent of the
    incoming telemetry rate.
    """

    MAX_POINTS = 7200         # >=60 s at 120 Hz per series; still time-pruned
    PAINT_INTERVAL_MS = 120   # ~8.3 Hz GUI repaint, decoupled from sensor rate

    def __init__(self, title: str, series: Sequence[str], history_seconds: float = 60.0,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.title = title
        self.series = list(series)
        self.history_seconds = float(history_seconds)
        self.buffers: Dict[str, deque] = {name: deque() for name in self.series}
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.header = QLabel(title)
        self.header.setObjectName("WorkspaceTitle")
        self.layout.addWidget(self.header)
        self.canvas = _PlotCanvas(self.series, self.buffers, self.history_seconds, parent=self)
        self.layout.addWidget(self.canvas, 1)
        buttons = QHBoxLayout()
        self.clear_btn = QPushButton("Clear")
        self.png_btn = QPushButton("Export PNG")
        buttons.addWidget(self.clear_btn)
        buttons.addWidget(self.png_btn)
        buttons.addStretch(1)
        self.layout.addLayout(buttons)
        self.clear_btn.clicked.connect(self.clear)
        self.png_btn.clicked.connect(lambda: self.export_png(
            Path("/home/otomasi2/ros/log/agv_gui/manual_exports") /
            f"{self.title.replace(' ', '_')}_{time.strftime('%Y%m%d_%H%M%S')}.png"))

    def append(self, values: Dict[str, float], timestamp: Optional[float] = None):
        now = float(timestamp if timestamp is not None else time.time())
        for name in self.series:
            if name in values:
                buf = self.buffers[name]
                buf.append((now, float(values[name])))
                while len(buf) > self.MAX_POINTS:
                    buf.popleft()
        cutoff = now - self.history_seconds
        for buf in self.buffers.values():
            while buf and buf[0][0] < cutoff:
                buf.popleft()
        # Data acquisition only — painting is driven by the canvas QTimer.

    def set_active(self, active: bool):
        """Suspend hidden plot repaint timers while still buffering samples."""
        if active:
            if not self.canvas._timer.isActive():
                self.canvas._timer.start()
            self.canvas.update()
        else:
            self.canvas._timer.stop()

    def refresh(self):
        self.canvas.update()

    def clear(self):
        for buf in self.buffers.values():
            buf.clear()
        self.refresh()

    def export_csv(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        rows = []
        for name, buf in self.buffers.items():
            rows.extend((t, name, v) for t, v in buf)
        rows.sort(key=lambda r: r[0])
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["timestamp", "series", "value"])
            writer.writerows(rows)

    def export_png(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.canvas.grab().save(str(path), "PNG")


class _PlotCanvas(QWidget):
    """QPainter-based realtime line chart. Repaints on a fixed QTimer, never on
    data arrival, so high-rate sensor callbacks never stall the GUI thread."""

    COLORS = ["#4ea1ff", "#ff5c8a", "#5fd38a", "#ffcc4d", "#c08bff",
              "#4ee0e0", "#ff9d4d", "#b0b0b0"]

    def __init__(self, series, buffers, history_seconds, parent=None):
        super().__init__(parent)
        self.series = series
        self.buffers = buffers
        self.history_seconds = history_seconds
        self._cache: Dict[str, List[Tuple[float, float]]] = {}
        self.setMinimumHeight(160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._timer = QTimer(self)
        self._timer.setInterval(RingPlot.PAINT_INTERVAL_MS)
        self._timer.timeout.connect(self.update)
        self._timer.start()

    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        try:
            painter.fillRect(self.rect(), QColor("#161a1f"))
            w, h = self.width(), self.height()
            if w < 4 or h < 4:
                return

            painter.setPen(QColor("#39414a"))
            painter.drawRect(1, 1, w - 2, h - 2)

            nonempty = [b for b in self.buffers.values() if b]
            if not nonempty:
                painter.setPen(QColor("#6b7681"))
                painter.drawText(self.rect().adjusted(8, 8, -8, -8),
                                 Qt.AlignCenter, "Waiting for data…")
                return

            # During the first history window, stretch the samples that actually
            # exist across the whole canvas. The old fixed (now-history) origin
            # made a fresh plot look half-empty even though data was arriving.
            now = max(b[-1][0] for b in nonempty)
            oldest_available = min(b[0][0] for b in nonempty)
            origin = max(now - self.history_seconds, oldest_available)
            t_span = max(1.0, now - origin)

            # Determine whether mixed units would collapse small signals. IMU
            # combines m/s^2, rad/s, degrees and uT in one compact chart. When
            # their spans differ strongly, render each signal using its own
            # vertical range; the Latest table still exposes the exact values.
            series_limits: Dict[str, Tuple[float, float]] = {}
            spans: List[float] = []
            global_vals: List[float] = []
            for name in self.series:
                vals = [v for t, v in self.buffers[name] if t >= origin and math.isfinite(v)]
                if not vals:
                    continue
                lo, hi = min(vals), max(vals)
                span = hi - lo
                if span < 1e-9:
                    span = max(1.0, abs(lo) * 0.05)
                    lo -= span * 0.5
                    hi += span * 0.5
                series_limits[name] = (lo, hi)
                spans.append(max(1e-9, hi - lo))
                global_vals.extend(vals)

            if not global_vals:
                return
            global_min, global_max = min(global_vals), max(global_vals)
            global_span = max(1e-9, global_max - global_min)
            sorted_spans = sorted(spans)
            median_span = sorted_spans[len(sorted_spans) // 2] if sorted_spans else global_span
            per_signal_scale = bool(spans) and global_span > max(1e-6, median_span) * 25.0

            if not per_signal_scale:
                pad = global_span * 0.10 if global_span > 1e-9 else 1.0
                global_min -= pad
                global_max += pad

            painter.setPen(QColor("#2a323a"))
            for i in range(1, 4):
                y = int(h * i / 4)
                painter.drawLine(2, y, w - 2, y)

            # At most ~2 points per horizontal pixel are useful to QPainter.
            # Downsampling only affects display; every source sample remains in
            # the buffer/CSV and ROS sensor rates are untouched.
            max_draw_points = max(64, (w - 4) * 2)
            for idx, name in enumerate(self.series):
                buf = self.buffers[name]
                if not buf or name not in series_limits:
                    continue
                visible = [(t, v) for t, v in buf if t >= origin and math.isfinite(v)]
                if not visible:
                    continue
                step = max(1, len(visible) // max_draw_points)
                if step > 1:
                    visible = visible[::step]
                    if visible[-1] != buf[-1] and buf[-1][0] >= origin and math.isfinite(buf[-1][1]):
                        visible.append(buf[-1])

                if per_signal_scale:
                    vmin, vmax = series_limits[name]
                    span = max(1e-9, vmax - vmin)
                    pad = span * 0.10
                    vmin -= pad
                    vmax += pad
                else:
                    vmin, vmax = global_min, global_max
                v_span = max(1e-9, vmax - vmin)

                pts = QPolygonF()
                for t, v in visible:
                    x = 2.0 + ((t - origin) / t_span) * (w - 4)
                    x = min(float(w - 2), max(2.0, x))
                    y = h - 2.0 - ((v - vmin) / v_span) * (h - 4)
                    y = min(float(h - 2), max(2.0, y))
                    pts.append(QPointF(x, y))
                color = QColor(self.COLORS[idx % len(self.COLORS)])
                if len(pts) >= 2:
                    painter.setPen(QPen(color, 1.0))
                    painter.drawPolyline(pts)
                # Sparse/event-driven signals (especially /amcl_pose) may only
                # provide one sample while the robot is stationary. Always draw
                # the newest point so the graph never looks empty when data did
                # in fact arrive.
                if len(pts) >= 1:
                    painter.setPen(QPen(color, 1.0))
                    painter.setBrush(QBrush(color))
                    last = pts[-1]
                    painter.drawEllipse(last, 2.8, 2.8)

            # Legend. Mark mixed-unit autoscaling explicitly so the graph does
            # not imply that unlike physical units share one numeric y-axis.
            lx, ly = 8, 8
            for idx, name in enumerate(self.series):
                if not self.buffers[name]:
                    continue
                color = QColor(self.COLORS[idx % len(self.COLORS)])
                painter.setPen(color)
                painter.fillRect(lx, ly + 2, 10, 10, color)
                painter.drawText(lx + 14, ly + 10, name)
                ly += 16
            if per_signal_scale:
                painter.setPen(QColor("#7f8b96"))
                painter.drawText(max(8, w - 150), 18, "per-signal autoscale")
        finally:
            painter.end()

    def grab_pixmap(self):
        return self.grab()


class YamlParameterEditor(QWidget):
    """Dynamic scalar parameter editor with no-wheel numeric controls.

    Changes are written on editingFinished/toggle/selection commit, never on
    mouse-wheel movement.  The editor deliberately exposes the YAML path so the
    operator always knows which production file is being changed.
    """

    changed = pyqtSignal(str)
    saved = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, yaml_path: Path, title: str = "Parameters", parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.yaml_path = Path(yaml_path)
        self.store = YamlStore(self.yaml_path)
        self._widgets: Dict[Tuple[str | int, ...], QWidget] = {}
        self.loaded_once = False
        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.timeout.connect(self._save_now)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        header = QLabel(title)
        header.setObjectName("SectionTitle")
        root.addWidget(header)
        self.path_label = QLabel(str(self.yaml_path))
        self.path_label.setWordWrap(True)
        self.path_label.setObjectName("Muted")
        root.addWidget(self.path_label)
        buttons = QHBoxLayout()
        self.reload_btn = QPushButton("Reload YAML")
        self.save_btn = QPushButton("Save")
        buttons.addWidget(self.reload_btn)
        buttons.addWidget(self.save_btn)
        root.addLayout(buttons)
        self.state_label = QLabel("Not loaded")
        root.addWidget(self.state_label)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.body = QWidget()
        self.form = QFormLayout(self.body)
        self.form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        self.scroll.setWidget(self.body)
        root.addWidget(self.scroll, 1)
        self.reload_btn.clicked.connect(self.reload)
        self.save_btn.clicked.connect(self._save_now)
        self.save_btn.setEnabled(False)
        self.state_label.setText("Deferred — opens when this tab is selected")

    def reload(self):
        self.loaded_once = True
        try:
            if not self.yaml_path.is_file():
                while self.form.rowCount():
                    self.form.removeRow(0)
                self._widgets.clear()
                self.save_btn.setEnabled(False)
                self.state_label.setText("File not found — waiting for source file")
                return
            self.save_btn.setEnabled(True)
            self.store.load()
            while self.form.rowCount():
                self.form.removeRow(0)
            self._widgets.clear()
            leaves = list(iter_scalar_items(self.store.data))
            for path, value in leaves:
                label = "/".join(str(p) for p in path)
                widget = self._editor_for(path, value)
                self.form.addRow(label, widget)
            backend = self.store.backend
            note = "comment-preserving" if backend == "ruamel" else "PyYAML fallback"
            self.state_label.setText(f"Loaded — {len(leaves)} values — {note}")
        except Exception as exc:
            self.state_label.setText(f"Load failed: {exc}")
            self.error.emit(str(exc))

    def _editor_for(self, path: Tuple[str | int, ...], value: Any) -> QWidget:
        if isinstance(value, bool):
            widget = QCheckBox()
            widget.setChecked(value)
            widget.toggled.connect(lambda checked, p=path: self._commit(p, bool(checked)))
        elif isinstance(value, int) and not isinstance(value, bool):
            widget = NoWheelSpinBox()
            widget.setRange(-2_000_000_000, 2_000_000_000)
            widget.setValue(int(value))
            widget.editingFinished.connect(lambda p=path, w=widget: self._commit(p, int(w.value())))
        elif isinstance(value, float):
            widget = NoWheelDoubleSpinBox()
            widget.setDecimals(8)
            widget.setRange(-1.0e9, 1.0e9)
            widget.setSingleStep(max(abs(value) * 0.05, 0.001))
            widget.setValue(float(value))
            widget.editingFinished.connect(lambda p=path, w=widget: self._commit(p, float(w.value())))
        else:
            widget = QLineEdit(self._format_text(value))
            widget.editingFinished.connect(lambda p=path, w=widget, old=value: self._commit(p, self._parse_text(w.text(), old)))
        self._widgets[path] = widget
        return widget

    @staticmethod
    def _format_text(value: Any) -> str:
        if isinstance(value, list):
            return repr(value)
        if value is None:
            return "null"
        return str(value)

    @staticmethod
    def _parse_text(text: str, old: Any) -> Any:
        text = text.strip()
        if isinstance(old, list):
            try:
                parsed = ast.literal_eval(text)
                if isinstance(parsed, (list, tuple)):
                    return list(parsed)
            except Exception:
                return old
        if old is None and text.lower() in ("none", "null", ""):
            return None
        return text

    def _commit(self, path: Tuple[str | int, ...], value: Any):
        self.store.set(path, value)
        self.state_label.setText("Modified — saving…")
        self.changed.emit("/".join(str(p) for p in path))
        self._save_timer.start(250)

    def _save_now(self):
        if not self.yaml_path.is_file():
            self.state_label.setText("SAVE BLOCKED — source YAML does not exist")
            return
        try:
            self.store.save(make_backup=True)
            self.state_label.setText("SAVED — restart may be required")
            self.saved.emit(str(self.yaml_path))
        except Exception as exc:
            self.state_label.setText(f"SAVE FAILED: {exc}")
            self.error.emit(str(exc))


class ScrollSettings(QWidget):
    """Simple wrapper for a vertically scrollable left-settings page."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.body = QWidget()
        self.layout = QVBoxLayout(self.body)
        self.layout.setContentsMargins(8, 8, 8, 8)
        self.layout.setSpacing(9)
        self.layout.addStretch(1)
        self.scroll.setWidget(self.body)
        outer.addWidget(self.scroll)

    def insert_widget(self, widget: QWidget, index: Optional[int] = None):
        index = self.layout.count() - 1 if index is None else index
        self.layout.insertWidget(max(0, index), widget)
