#!/usr/bin/env python3
"""High-performance QGraphicsView occupancy-map canvas."""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QImage, QPainter, QPainterPath, QPen, QPixmap, QPolygonF, QTransform
from PyQt5.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItemGroup,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
)

import numpy as np

from .map_model import MapDocument


class MapCanvas(QGraphicsView):
    coordinate_changed = pyqtSignal(float, float, float, float)  # u, v, x, y
    point_clicked = pyqtSignal(float, float, float)  # x, y, yaw (yaw=nan for point)
    pose_clicked = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setScene(QGraphicsScene(self))
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QBrush(QColor("#1f2329")))
        self.document: Optional[MapDocument] = None
        self._map_item: Optional[QGraphicsPixmapItem] = None
        self.layers: Dict[str, QGraphicsItemGroup] = {}
        self.layer_opacity: Dict[str, float] = {}
        self.mode = "pan"
        self._pose_start: Optional[Tuple[float, float]] = None
        self._latest_robot: Optional[Tuple[float, float, float]] = None
        self._robot_polygon: Optional[QGraphicsPolygonItem] = None
        self._robot_arrow: Optional[QGraphicsLineItem] = None
        self._robot_text: Optional[QGraphicsSimpleTextItem] = None
        self._goal_group: Optional[QGraphicsItemGroup] = None
        self._initial_group: Optional[QGraphicsItemGroup] = None

    def set_document(self, document: Optional[MapDocument]):
        self.scene().clear()
        self.layers.clear()
        self._robot_polygon = self._robot_arrow = self._robot_text = None
        self._goal_group = self._initial_group = None
        self.document = document
        if document is None:
            self._map_item = None
            self.scene().setSceneRect(QRectF(0, 0, 800, 600))
            return
        self._map_item = self.scene().addPixmap(document.pixmap)
        self._map_item.setZValue(0)
        self.scene().setSceneRect(QRectF(0, 0, document.width, document.height))
        self.ensure_layer("scan", 20)
        self.ensure_layer("lidar_odom", 30)
        self.ensure_layer("ekf", 31)
        self.ensure_layer("amcl", 32)
        self.ensure_layer("tf", 33)
        self.ensure_layer("ground_truth", 35)
        self.ensure_layer("global_path", 40)
        self.ensure_layer("local_path", 41)
        self.ensure_layer("trajectories", 42)
        # Saved-map planning preview is map-slot local and is always safe to
        # show on Map 1/2/3.  Live costmaps remain tied to the active map_server.
        self.ensure_layer("static_inflation", 8)
        self.ensure_layer("global_costmap", 10)
        self.ensure_layer("local_costmap", 11)
        self.ensure_layer("robot", 50)
        self.ensure_layer("goal", 55)
        self.ensure_layer("initial", 54)
        self.fit_map()

    def ensure_layer(self, name: str, z: float = 1.0) -> QGraphicsItemGroup:
        if name not in self.layers:
            group = QGraphicsItemGroup()
            group.setZValue(z)
            self.scene().addItem(group)
            self.layers[name] = group
            self.layer_opacity[name] = 1.0
        return self.layers[name]

    def set_layer_visible(self, name: str, visible: bool):
        group = self.layers.get(name)
        if group is not None:
            group.setVisible(bool(visible))

    def set_layer_opacity(self, name: str, opacity: float):
        group = self.layers.get(name)
        if group is not None:
            value = max(0.0, min(1.0, float(opacity)))
            group.setOpacity(value)
            self.layer_opacity[name] = value

    def clear_layer(self, name: str):
        group = self.layers.get(name)
        if group is None:
            return
        for child in list(group.childItems()):
            child.setParentItem(None)
            self.scene().removeItem(child)

    def fit_map(self):
        if self.document is not None:
            self.fitInView(self.scene().sceneRect(), Qt.KeepAspectRatio)

    def reset_view(self):
        self.resetTransform()
        self.fit_map()

    def set_mode(self, mode: str):
        self.mode = mode
        self._pose_start = None
        self.setDragMode(QGraphicsView.ScrollHandDrag if mode == "pan" else QGraphicsView.NoDrag)

    def wheelEvent(self, event):  # noqa: N802
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale(factor, factor)

    def mouseMoveEvent(self, event):  # noqa: N802
        if self.document is not None:
            p = self.mapToScene(event.pos())
            x, y = self.document.transform.pixel_to_map(p.x(), p.y())
            self.coordinate_changed.emit(p.x(), p.y(), x, y)
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event):  # noqa: N802
        if self.document is None or event.button() != Qt.LeftButton:
            super().mousePressEvent(event)
            return
        if self.mode in ("point", "ground_truth", "goal", "initial"):
            p = self.mapToScene(event.pos())
            x, y = self.document.transform.pixel_to_map(p.x(), p.y())
            if self.mode in ("goal", "initial"):
                self._pose_start = (x, y)
            else:
                self.point_clicked.emit(x, y, float("nan"))
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):  # noqa: N802
        if self.document is not None and event.button() == Qt.LeftButton and self._pose_start is not None:
            p = self.mapToScene(event.pos())
            x2, y2 = self.document.transform.pixel_to_map(p.x(), p.y())
            x1, y1 = self._pose_start
            yaw = math.atan2(y2 - y1, x2 - x1) if abs(x2 - x1) + abs(y2 - y1) > 1e-6 else 0.0
            self.pose_clicked.emit(x1, y1, yaw)
            self._pose_start = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _map_point(self, x: float, y: float) -> QPointF:
        if self.document is None:
            return QPointF()
        u, v = self.document.transform.map_to_pixel(x, y)
        return QPointF(u, v)

    def draw_points(self, layer: str, points: Sequence[Tuple[float, float]],
                    radius_px: float = 2.2, color: QColor = QColor("#d8b74a"), clear: bool = True):
        if self.document is None:
            return
        if clear:
            self.clear_layer(layer)
        if not points:
            return
        # One QGraphicsPathItem replaces hundreds/thousands of individual
        # QGraphicsEllipseItem objects. Incoming LaserScan rate is unchanged;
        # only the GUI scene representation is batched.
        path = QPainterPath()
        diameter = radius_px * 2.0
        for x, y in points:
            p = self._map_point(x, y)
            path.addEllipse(p.x() - radius_px, p.y() - radius_px, diameter, diameter)
        item = QGraphicsPathItem(path)
        pen = QPen(color)
        pen.setCosmetic(True)
        item.setPen(pen)
        item.setBrush(QBrush(color))
        item.setParentItem(self.ensure_layer(layer))

    def draw_path(self, layer: str, points: Sequence[Tuple[float, float]],
                  color: QColor, width: float = 2.0, clear: bool = True):
        if self.document is None:
            return
        if clear:
            self.clear_layer(layer)
        if not points:
            return
        path = QPainterPath(self._map_point(*points[0]))
        for point in points[1:]:
            path.lineTo(self._map_point(*point))
        item = QGraphicsPathItem(path)
        pen = QPen(color)
        pen.setWidthF(width)
        pen.setCosmetic(True)
        item.setPen(pen)
        item.setParentItem(self.ensure_layer(layer))

    def draw_robot(self, x: float, y: float, yaw: float,
                   half_length: float = 0.65, half_width: float = 0.40):
        if self.document is None:
            return
        self.clear_layer("robot")
        group = self.ensure_layer("robot", 50)
        corners = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        ]
        c, s = math.cos(yaw), math.sin(yaw)
        polygon = QPolygonF()
        for lx, ly in corners:
            wx = x + c * lx - s * ly
            wy = y + s * lx + c * ly
            polygon.append(self._map_point(wx, wy))
        body = QGraphicsPolygonItem(polygon)
        body.setPen(QPen(QColor("#37b5ff"), 2))
        body.setBrush(QBrush(QColor(55, 181, 255, 55)))
        body.setParentItem(group)
        nose = self._map_point(x + c * half_length, y + s * half_length)
        center = self._map_point(x, y)
        arrow = QGraphicsLineItem(center.x(), center.y(), nose.x(), nose.y())
        pen = QPen(QColor("#37b5ff"), 2)
        pen.setCosmetic(True)
        arrow.setPen(pen)
        arrow.setParentItem(group)
        label = QGraphicsSimpleTextItem(f"{x:.2f}, {y:.2f}\n{math.degrees(yaw):.1f}°")
        label.setBrush(QBrush(QColor("#f1f4f7")))
        label.setPos(center + QPointF(7, -22))
        label.setFlag(QGraphicsSimpleTextItem.ItemIgnoresTransformations, True)
        label.setParentItem(group)
        self._latest_robot = (x, y, yaw)

    def center_robot(self):
        if self._latest_robot and self.document is not None:
            self.centerOn(self._map_point(self._latest_robot[0], self._latest_robot[1]))

    def draw_ground_truth(self, points: Sequence[Dict[str, object]]):
        if self.document is None:
            return
        self.clear_layer("ground_truth")
        group = self.ensure_layer("ground_truth", 35)
        for entry in points:
            try:
                x = float(entry.get("x", 0.0))
                y = float(entry.get("y", 0.0))
            except Exception:
                continue
            p = self._map_point(x, y)
            ellipse = QGraphicsEllipseItem(p.x() - 4, p.y() - 4, 8, 8)
            ellipse.setPen(QPen(QColor("#ffca55"), 2))
            ellipse.setBrush(QBrush(QColor(255, 202, 85, 80)))
            ellipse.setParentItem(group)
            label = QGraphicsSimpleTextItem(str(entry.get("id", "GT")))
            label.setBrush(QBrush(QColor("#ffdf85")))
            label.setPos(p + QPointF(5, -16))
            label.setFlag(QGraphicsSimpleTextItem.ItemIgnoresTransformations, True)
            label.setParentItem(group)


    def draw_static_inflation(self, occupancy: np.ndarray, resolution: float,
                              inflation_radius: float = 0.45,
                              cost_scaling_factor: float = 12.0,
                              inscribed_radius: float = 0.40,
                              alpha: int = 150):
        """Draw a deterministic saved-map cost/inflation preview.

        This is intentionally computed from *this MapPage's own PGM* rather
        than copying the currently active Nav2 OccupancyGrid to another map
        slot.  It therefore gives Map 1/2/3 a truthful static cost/inflation
        view even when only one map can be active in map_server.
        """
        if self.document is None:
            return
        try:
            arr = np.asarray(occupancy, dtype=np.int16)
            if arr.ndim != 2 or arr.size == 0 or resolution <= 0.0:
                return
            h, w = arr.shape
            occupied = arr >= 65
            known = arr >= 0
            radius_cells = max(1, int(math.ceil(float(inflation_radius) / float(resolution))))
            dist = np.full((h, w), np.inf, dtype=np.float32)

            # Exact Euclidean distance within the finite inflation radius. Maps
            # are small, and this runs only on map load/reload (not per frame).
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    d_cells = math.hypot(dx, dy)
                    if d_cells > radius_cells + 1e-9:
                        continue
                    d_m = d_cells * float(resolution)
                    if d_m > float(inflation_radius) + 1e-9:
                        continue
                    sy0 = max(0, -dy)
                    sy1 = min(h, h - dy)
                    sx0 = max(0, -dx)
                    sx1 = min(w, w - dx)
                    if sy0 >= sy1 or sx0 >= sx1:
                        continue
                    src = occupied[sy0:sy1, sx0:sx1]
                    if not np.any(src):
                        continue
                    ty0, ty1 = sy0 + dy, sy1 + dy
                    tx0, tx1 = sx0 + dx, sx1 + dx
                    view = dist[ty0:ty1, tx0:tx1]
                    np.minimum(view, np.where(src, d_m, np.inf).astype(np.float32), out=view)

            rgba = np.zeros((h, w, 4), dtype=np.uint8)
            inflated = known & np.isfinite(dist) & (dist <= float(inflation_radius))
            lethal = occupied
            halo = inflated & ~lethal

            # Nav2-like exponential decay outside the inscribed footprint. The
            # preview's purpose is layer visibility and tuning inspection; live
            # Nav2 remains the authoritative collision-cost computation.
            if np.any(halo):
                d = dist[halo].astype(np.float64)
                decay = np.exp(-max(0.0, float(cost_scaling_factor)) * np.maximum(0.0, d - float(inscribed_radius)))
                strength = np.clip(decay, 0.12, 1.0)
                rgba_halo = rgba[halo]
                rgba_halo[:, 0] = 244
                rgba_halo[:, 1] = (178.0 * strength + 76.0 * (1.0 - strength)).astype(np.uint8)
                rgba_halo[:, 2] = 52
                rgba_halo[:, 3] = np.clip(35.0 + strength * max(20, alpha - 35), 35, 220).astype(np.uint8)
                rgba[halo] = rgba_halo
            rgba[lethal] = (224, 68, 68, min(230, max(120, int(alpha) + 55)))

            qimg = QImage(rgba.data, w, h, int(rgba.strides[0]), QImage.Format_RGBA8888).copy()
            self.clear_layer("static_inflation")
            item = QGraphicsPixmapItem(QPixmap.fromImage(qimg))
            item.setParentItem(self.ensure_layer("static_inflation", 8))
        except Exception:
            # A visualization layer must never be able to take down the GUI.
            self.clear_layer("static_inflation")

    def draw_occupancy_grid(self, layer: str, msg, alpha: int = 95):
        """Overlay a nav_msgs/OccupancyGrid in true metric coordinates."""
        if self.document is None:
            return
        try:
            w = int(msg.info.width)
            h = int(msg.info.height)
            if w <= 0 or h <= 0 or len(msg.data) != w * h:
                return
            arr = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
            # Convert ROS bottom-up rows to raster top-down rows.
            arr = np.flipud(arr)
            rgba = np.zeros((h, w, 4), dtype=np.uint8)
            occupied = arr >= 65
            unknown = arr < 0
            known_free = (arr >= 0) & ~occupied
            rgba[occupied] = (220, 65, 65, alpha)
            rgba[known_free] = (70, 145, 230, max(18, alpha // 4))
            rgba[unknown] = (120, 120, 120, max(10, alpha // 6))
            qimg = QImage(rgba.data, w, h, int(rgba.strides[0]), QImage.Format_RGBA8888).copy()
            self.clear_layer(layer)
            item = QGraphicsPixmapItem(QPixmap.fromImage(qimg))
            item.setParentItem(self.ensure_layer(layer))

            res = float(msg.info.resolution)
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)
            q = msg.info.origin.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            # top-left, top-right, bottom-left world positions
            def world(lx, ly):
                return ox + c * lx - s * ly, oy + s * lx + c * ly
            tl = self._map_point(*world(0.0, h * res))
            tr = self._map_point(*world(w * res, h * res))
            bl = self._map_point(*world(0.0, 0.0))
            m11 = (tr.x() - tl.x()) / float(w)
            m12 = (tr.y() - tl.y()) / float(w)
            m21 = (bl.x() - tl.x()) / float(h)
            m22 = (bl.y() - tl.y()) / float(h)
            item.setTransform(QTransform(m11, m12, m21, m22, tl.x(), tl.y()))
        except Exception:
            return

    def draw_pose_marker(self, layer: str, x: float, y: float, yaw: float,
                         color: QColor, label: str = ""):
        if self.document is None:
            return
        self.clear_layer(layer)
        group = self.ensure_layer(layer)
        p = self._map_point(x, y)
        length_m = 0.55
        end = self._map_point(x + math.cos(yaw) * length_m, y + math.sin(yaw) * length_m)
        line = QGraphicsLineItem(p.x(), p.y(), end.x(), end.y())
        pen = QPen(color, 2)
        pen.setCosmetic(True)
        line.setPen(pen)
        line.setParentItem(group)
        circ = QGraphicsEllipseItem(p.x() - 4, p.y() - 4, 8, 8)
        circ.setPen(pen)
        circ.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 75)))
        circ.setParentItem(group)
        if label:
            text = QGraphicsSimpleTextItem(label)
            text.setBrush(QBrush(color))
            text.setPos(p + QPointF(6, -16))
            text.setFlag(QGraphicsSimpleTextItem.ItemIgnoresTransformations, True)
            text.setParentItem(group)
