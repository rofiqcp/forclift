#!/usr/bin/env python3
"""LiDAR occupancy-map models and coordinate conversion utilities."""
from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import numpy as np
from PyQt5.QtGui import QImage, QPixmap

from .yaml_store import YamlStore


class MapLoadError(RuntimeError):
    pass


@dataclass
class MapMetadata:
    image: str = ""
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    negate: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196
    mode: str = "trinary"


@dataclass
class MapSource:
    slot: int
    display_name: str
    actual_name: str
    yaml_path: Path
    image_path: Optional[Path] = None
    frame_id: str = "map"
    ros_topic: str = "/map"
    build_timestamp: float = 0.0
    available: bool = False
    error: str = ""


@dataclass
class MapDocument:
    source: MapSource
    metadata: MapMetadata
    raw_pixels: np.ndarray
    occupancy: np.ndarray
    pixmap: QPixmap
    transform: "MapCoordinateTransform"

    @property
    def width(self) -> int:
        return int(self.raw_pixels.shape[1])

    @property
    def height(self) -> int:
        return int(self.raw_pixels.shape[0])

    @property
    def width_m(self) -> float:
        return self.width * self.metadata.resolution

    @property
    def height_m(self) -> float:
        return self.height * self.metadata.resolution

    def stats(self) -> Dict[str, float]:
        total = float(self.occupancy.size or 1)
        free = float(np.count_nonzero(self.occupancy == 0))
        occ = float(np.count_nonzero(self.occupancy == 100))
        unknown = float(np.count_nonzero(self.occupancy < 0))
        return {
            "free_pct": 100.0 * free / total,
            "occupied_pct": 100.0 * occ / total,
            "unknown_pct": 100.0 * unknown / total,
            "cells": total,
        }


class MapCoordinateTransform:
    """ROS map <-> raster pixel transform, including non-zero origin yaw."""

    def __init__(self, width: int, height: int, resolution: float,
                 origin_x: float, origin_y: float, origin_yaw: float):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self._c = math.cos(self.origin_yaw)
        self._s = math.sin(self.origin_yaw)

    def pixel_to_map(self, u: float, v: float) -> Tuple[float, float]:
        # Use cell center convention. Raster v=0 is top, ROS grid y=0 is bottom.
        lx = (float(u) + 0.5) * self.resolution
        ly = (self.height - float(v) - 0.5) * self.resolution
        x = self.origin_x + self._c * lx - self._s * ly
        y = self.origin_y + self._s * lx + self._c * ly
        return x, y

    def map_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        dx = float(x) - self.origin_x
        dy = float(y) - self.origin_y
        lx = self._c * dx + self._s * dy
        ly = -self._s * dx + self._c * dy
        u = lx / self.resolution - 0.5
        v = self.height - ly / self.resolution - 0.5
        return u, v

    def world_arrays_to_pixel(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        dx = x - self.origin_x
        dy = y - self.origin_y
        lx = self._c * dx + self._s * dy
        ly = -self._s * dx + self._c * dy
        u = lx / self.resolution - 0.5
        v = self.height - ly / self.resolution - 0.5
        return u, v


class PgmReader:
    @staticmethod
    def read(path: Path) -> Tuple[np.ndarray, int]:
        data = path.read_bytes()
        if not data.startswith((b"P2", b"P5")):
            raise MapLoadError(f"Unsupported PGM magic in {path}")
        magic = data[:2]
        pos = 2

        def token() -> bytes:
            nonlocal pos
            n = len(data)
            while pos < n:
                if data[pos:pos + 1] == b"#":
                    end = data.find(b"\n", pos)
                    pos = n if end < 0 else end + 1
                    continue
                if data[pos:pos + 1].isspace():
                    pos += 1
                    continue
                break
            start = pos
            while pos < n and not data[pos:pos + 1].isspace() and data[pos:pos + 1] != b"#":
                pos += 1
            return data[start:pos]

        try:
            width = int(token())
            height = int(token())
            maxval = int(token())
        except Exception as exc:
            raise MapLoadError(f"Invalid PGM header in {path}: {exc}") from exc
        if width <= 0 or height <= 0 or maxval <= 0 or maxval > 65535:
            raise MapLoadError(f"Invalid PGM dimensions/maxval in {path}")

        if magic == b"P2":
            values = []
            while len(values) < width * height:
                t = token()
                if not t:
                    break
                values.append(int(t))
            if len(values) != width * height:
                raise MapLoadError(f"P2 pixel count mismatch in {path}")
            arr = np.asarray(values, dtype=np.uint16).reshape((height, width))
            return arr, maxval

        # P5 header is terminated by whitespace. Consume the separator only;
        # binary image data may legitimately begin with a whitespace-valued byte.
        if pos >= len(data) or not data[pos:pos + 1].isspace():
            raise MapLoadError(f"Missing P5 header separator in {path}")
        first_sep = data[pos:pos + 1]
        pos += 1
        if first_sep == b"\r" and pos < len(data) and data[pos:pos + 1] == b"\n":
            pos += 1
        count = width * height
        if maxval < 256:
            payload = data[pos:pos + count]
            if len(payload) != count:
                raise MapLoadError(f"P5 pixel count mismatch in {path}")
            arr = np.frombuffer(payload, dtype=np.uint8).astype(np.uint16).reshape((height, width))
        else:
            payload = data[pos:pos + count * 2]
            if len(payload) != count * 2:
                raise MapLoadError(f"16-bit P5 pixel count mismatch in {path}")
            arr = np.frombuffer(payload, dtype=">u2").astype(np.uint16).reshape((height, width))
        return arr, maxval


class MapLoader:
    def load(self, source: MapSource) -> MapDocument:
        if not source.yaml_path.is_file():
            raise MapLoadError(f"YAML not found: {source.yaml_path}")
        store = YamlStore(source.yaml_path)
        data = store.load()
        if not isinstance(data, dict):
            raise MapLoadError(f"Map YAML root is not a mapping: {source.yaml_path}")
        image_value = str(data.get("image", "")).strip()
        if not image_value:
            raise MapLoadError(f"Map YAML has no image: {source.yaml_path}")
        image_path = Path(os.path.expanduser(image_value))
        if not image_path.is_absolute():
            image_path = source.yaml_path.parent / image_path
        image_path = image_path.resolve()
        if not image_path.is_file():
            raise MapLoadError(f"Map image not found: {image_path}")

        origin = data.get("origin", [0.0, 0.0, 0.0])
        if not isinstance(origin, (list, tuple)) or len(origin) < 3:
            raise MapLoadError(f"Invalid map origin in {source.yaml_path}")
        meta = MapMetadata(
            image=image_value,
            resolution=float(data.get("resolution", 0.05)),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            negate=int(data.get("negate", 0)),
            occupied_thresh=float(data.get("occupied_thresh", 0.65)),
            free_thresh=float(data.get("free_thresh", 0.196)),
            mode=str(data.get("mode", "trinary")),
        )
        if meta.resolution <= 0.0:
            raise MapLoadError(f"Map resolution must be > 0 in {source.yaml_path}")

        raw, maxval = PgmReader.read(image_path)
        occupancy = self._classify(raw, maxval, meta)
        visual = self._visual_gray(occupancy)
        h, w = visual.shape
        qimg = QImage(visual.data, w, h, int(visual.strides[0]), QImage.Format_Grayscale8).copy()
        transform = MapCoordinateTransform(w, h, meta.resolution, meta.origin_x, meta.origin_y, meta.origin_yaw)
        source.image_path = image_path
        source.actual_name = source.yaml_path.stem
        source.build_timestamp = max(source.yaml_path.stat().st_mtime, image_path.stat().st_mtime)
        source.available = True
        source.error = ""
        return MapDocument(source, meta, raw, occupancy, QPixmap.fromImage(qimg), transform)

    @staticmethod
    def _classify(raw: np.ndarray, maxval: int, meta: MapMetadata) -> np.ndarray:
        p = raw.astype(np.float32) / float(maxval)
        occ_prob = p if meta.negate else (1.0 - p)
        mode = meta.mode.lower().strip()
        out = np.full(raw.shape, -1, dtype=np.int16)
        if mode == "raw":
            # Nav2 raw mode treats 0..100 as occupancy and 255 as unknown.  For
            # non-100 maxval maps fall back to normalized probability.
            if maxval >= 100:
                scaled = np.rint(raw.astype(np.float32) * (100.0 / maxval)).astype(np.int16)
                out[:] = np.clip(scaled, 0, 100)
                if maxval == 255:
                    out[raw == 255] = -1
                return out
        if mode == "scale":
            occupied = occ_prob >= meta.occupied_thresh
            free = occ_prob <= meta.free_thresh
            middle = ~(occupied | free)
            out[occupied] = 100
            out[free] = 0
            if meta.occupied_thresh > meta.free_thresh:
                scaled = (occ_prob[middle] - meta.free_thresh) / (meta.occupied_thresh - meta.free_thresh)
                out[middle] = np.clip(np.rint(scaled * 100.0), 1, 99).astype(np.int16)
            return out
        out[occ_prob >= meta.occupied_thresh] = 100
        out[occ_prob <= meta.free_thresh] = 0
        return out

    @staticmethod
    def _visual_gray(occupancy: np.ndarray) -> np.ndarray:
        # Match common ROS map display convention: occupied black, free white,
        # unknown mid-gray. Scaled occupancy receives a gray gradient.
        out = np.full(occupancy.shape, 205, dtype=np.uint8)
        known = occupancy >= 0
        out[known] = np.clip(254 - occupancy[known].astype(np.int16) * 254 // 100, 0, 254).astype(np.uint8)
        return out


class MapRepository:
    """Resolve the exact three mapping_gui.py experiment slots."""

    def __init__(self, workspace: str | None = None):
        self.workspace = Path(workspace or os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or (Path.home() / 'forclift'))
        self.map_dir = self.workspace / "src" / "navigation" / "maps"
        self.pointer = self.workspace / "maps" / "latest_map.txt"
        self.sources = [
            MapSource(slot=i, display_name=f"Map {i}", actual_name=f"map_{i}",
                      yaml_path=self.map_dir / f"map_{i}.yaml")
            for i in range(1, 4)
        ]

    def refresh_sources(self) -> Iterable[MapSource]:
        for source in self.sources:
            source.yaml_path = self.map_dir / f"map_{source.slot}.yaml"
            source.available = source.yaml_path.is_file()
            source.error = "" if source.available else "YAML not found"
            if source.available:
                try:
                    source.build_timestamp = source.yaml_path.stat().st_mtime
                except OSError:
                    source.build_timestamp = 0.0
        return self.sources

    def active_navigation_yaml(self) -> Optional[Path]:
        try:
            text = self.pointer.read_text(encoding="utf-8").strip()
        except OSError:
            return None
        if not text:
            return None
        p = Path(text).expanduser().resolve()
        return p if p.is_file() else None

    def slot_for_path(self, path: Optional[Path]) -> Optional[int]:
        if path is None:
            return None
        try:
            active = Path(path).expanduser().resolve()
        except Exception:
            return None
        for source in self.sources:
            try:
                if active == source.yaml_path.resolve():
                    return source.slot
            except OSError:
                pass
        return None

    def active_slot(self) -> Optional[int]:
        return self.slot_for_path(self.active_navigation_yaml())


def compare_maps(reference: MapDocument, other: MapDocument, max_samples: int = 450_000) -> Dict[str, float]:
    """Compare two occupancy maps in metric coordinates, not raw pixels.

    Sampling is downsampled on large maps to keep the Jetson UI responsive.
    Values are calculated only where reference cell centers land inside `other`.
    """
    h, w = reference.occupancy.shape
    total = max(1, h * w)
    step = max(1, int(math.ceil(math.sqrt(total / max(1, max_samples)))))
    rows = np.arange(0, h, step, dtype=np.float32)
    cols = np.arange(0, w, step, dtype=np.float32)
    uu, vv = np.meshgrid(cols, rows)
    # Reference pixels -> world coordinates, vectorized.
    lx = (uu + 0.5) * reference.metadata.resolution
    ly = (h - vv - 0.5) * reference.metadata.resolution
    c = math.cos(reference.metadata.origin_yaw)
    s = math.sin(reference.metadata.origin_yaw)
    wx = reference.metadata.origin_x + c * lx - s * ly
    wy = reference.metadata.origin_y + s * lx + c * ly
    ou, ov = other.transform.world_arrays_to_pixel(wx, wy)
    oi = np.rint(ou).astype(np.int64)
    oj = np.rint(ov).astype(np.int64)
    inside = (oi >= 0) & (oi < other.width) & (oj >= 0) & (oj < other.height)
    if not np.any(inside):
        return {"overlap_samples": 0.0, "agreement_pct": float("nan")}
    ref_vals = reference.occupancy[vv.astype(np.int64)[inside], uu.astype(np.int64)[inside]]
    oth_vals = other.occupancy[oj[inside], oi[inside]]

    def cls(a: np.ndarray) -> np.ndarray:
        out = np.full(a.shape, -1, dtype=np.int8)
        out[a == 0] = 0
        out[a >= 65] = 1
        # scaled cells 1..64 are considered known free-ish for agreement only
        out[(a > 0) & (a < 65)] = 0
        return out

    a = cls(ref_vals)
    b = cls(oth_vals)
    same = a == b
    known_both = (a >= 0) & (b >= 0)
    known_agree = (a == b) & known_both
    return {
        "overlap_samples": float(a.size),
        "agreement_pct": float(np.mean(same) * 100.0),
        "known_agreement_pct": float(np.mean(known_agree[known_both]) * 100.0) if np.any(known_both) else float("nan"),
        "occupied_to_free_pct": float(np.mean((a == 1) & (b == 0)) * 100.0),
        "free_to_occupied_pct": float(np.mean((a == 0) & (b == 1)) * 100.0),
        "known_to_unknown_pct": float(np.mean((a >= 0) & (b < 0)) * 100.0),
        "unknown_to_known_pct": float(np.mean((a < 0) & (b >= 0)) * 100.0),
        "sample_step": float(step),
    }
