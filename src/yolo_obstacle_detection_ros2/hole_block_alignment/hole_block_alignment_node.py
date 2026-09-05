#!/usr/bin/python3
"""
HOLE-GUIDED BLOCK ALIGNMENT — ROS 2 node (Stage 2, live).

Consumes the existing YOLO AGV detector on `/obstacle_detection/obstacles`
(5-class: floor marking / block / front / hole pallet / pallet) plus the raw
RGB image on `/camera/color/image_raw`, letterboxes the image to 1280x720, and
runs the SAME hole-first geometry + block-validation pipeline that the offline
stage 1 used.

Only the detection SOURCE changed (subscribed ObstacleArray instead of calling
ultralytics directly). The geometry, hole-pair selection, block validation,
filtering, lateral/yaw math, HUD and CSV are unchanged from the validated
offline implementation.

Reference axis: (640,700) -> (640,0). Set point: (640,700).
Outputs:
  /fork_alignment/state   (AlignmentState: error_lateral_m, error_yaw_deg, ...)
  /fork_alignment/image    (sensor_msgs/Image annotated HUD, 1280x720)
  CSV log (if save_csv: true) with 1280x720 + letterbox fields.

Keys (when --display 1): S=shot  R=reset  SPACE=pause  Q=quit
"""

import argparse
import csv
import json
import os
import math
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from yolo_obstacle_detection_ros2.msg import (
    ObstacleArray,
    AlignmentState,
)

import yaml


def nan():
    return float("nan")


def is_num(v):
    return v is not None and math.isfinite(v)




def _flatten_scalar_cfg(node, prefix=""):
    out = []
    if isinstance(node, dict):
        for k, v in node.items():
            key = f"{prefix}.{k}" if prefix else str(k)
            out.extend(_flatten_scalar_cfg(v, key))
    elif isinstance(node, (bool, int, float, str)) and not isinstance(node, type(None)):
        out.append((prefix, node))
    return out


def _set_cfg_path(cfg, dotted, value):
    parts = dotted.split('.')
    cur = cfg
    for key in parts[:-1]:
        cur = cur[key]
    cur[parts[-1]] = value

def normalize_angle_deg(a):
    """Wrap angle into (-180, 180]."""
    if not is_num(a):
        return nan()
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


class EMA:
    def __init__(self, alpha):
        self.alpha = float(alpha)
        self.value = None

    def update(self, v):
        if not is_num(v):
            return self.value
        if self.value is None:
            self.value = float(v)
        else:
            self.value = self.alpha * float(v) + (1.0 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None


class EMA2:
    def __init__(self, alpha):
        self.x = EMA(alpha)
        self.y = EMA(alpha)

    def update(self, px, py):
        self.x.update(px)
        self.y.update(py)
        return self.x.value, self.y.value

    def reset(self):
        self.x.reset()
        self.y.reset()


class YawEMA:
    """Wrap-aware EMA for angles in degrees."""

    def __init__(self, alpha):
        self.alpha = float(alpha)
        self.value = None

    def update(self, raw):
        if not is_num(raw):
            return self.value
        if self.value is None:
            self.value = float(raw)
        else:
            diff = normalize_angle_deg(raw - self.value)
            self.value = normalize_angle_deg(self.value + self.alpha * diff)
        return self.value

    def reset(self):
        self.value = None


class Measurement:
    __slots__ = (
        "left_hole_valid", "right_hole_valid", "hole_pair_valid",
        "block_detected", "block_geometry_valid", "measurement_valid",
        "metric_error_valid", "detection_stable", "stable_frame_count",
        "stable_req", "hole_gate_count", "hole_gate_req", "status", "aligned",
        "left_hole_center", "right_hole_center", "left_hole_conf", "right_hole_conf",
        "holes_midpoint", "hole_separation_px", "holes_line_angle_deg",
        "centerline_normal", "hole_unit", "block_offset_along_centerline_px",
        "block_offset_perpendicular_px", "offset_auto",
        "expected_block_center", "detected_block_center", "detected_block_box", "block_confidence",
        "block_residual_x_px", "block_residual_y_px", "block_residual_distance_px",
        "alignment_center", "block_roi", "candidate_count",
        "setpoint_x_px", "setpoint_y_px",
        "setpoint_x_min_px", "setpoint_x_max_px", "setpoint_y_min_px", "setpoint_y_max_px",
        "raw_error_lateral_px", "filtered_error_lateral_px",
        "meter_per_pixel", "raw_error_lateral_m", "filtered_error_lateral_m",
        "raw_error_yaw_deg", "filtered_error_yaw_deg", "setpoint_yaw_offset_deg",
        "yaw_outlier", "lat_outlier",
        "block_angle_deg",
        "resize_scale", "padding_left", "padding_top", "active_w", "active_h",
    )

    def __init__(self):
        self.left_hole_valid = False
        self.right_hole_valid = False
        self.hole_pair_valid = False
        self.block_detected = False
        self.block_geometry_valid = False
        self.measurement_valid = False
        self.metric_error_valid = False
        self.detection_stable = False
        self.stable_frame_count = 0
        self.stable_req = 0
        self.hole_gate_count = 0
        self.hole_gate_req = 0
        self.status = "INVALID"
        self.aligned = False
        self.left_hole_center = (nan(), nan())
        self.right_hole_center = (nan(), nan())
        self.left_hole_conf = nan()
        self.right_hole_conf = nan()
        self.holes_midpoint = (nan(), nan())
        self.hole_separation_px = nan()
        self.holes_line_angle_deg = nan()
        self.centerline_normal = (nan(), nan())
        self.hole_unit = (nan(), nan())
        self.block_offset_along_centerline_px = nan()
        self.block_offset_perpendicular_px = nan()
        self.offset_auto = False
        self.expected_block_center = (nan(), nan())
        self.detected_block_center = (nan(), nan())
        self.detected_block_box = None
        self.block_confidence = nan()
        self.block_residual_x_px = nan()
        self.block_residual_y_px = nan()
        self.block_residual_distance_px = nan()
        self.alignment_center = (nan(), nan())
        self.block_roi = None
        self.candidate_count = 0
        self.setpoint_x_px = 0.0
        self.setpoint_y_px = 0.0
        self.setpoint_x_min_px = 0.0
        self.setpoint_x_max_px = 0.0
        self.setpoint_y_min_px = 0.0
        self.setpoint_y_max_px = 0.0
        self.raw_error_lateral_px = nan()
        self.filtered_error_lateral_px = nan()
        self.meter_per_pixel = nan()
        self.raw_error_lateral_m = nan()
        self.filtered_error_lateral_m = nan()
        self.raw_error_yaw_deg = nan()
        self.filtered_error_yaw_deg = nan()
        self.setpoint_yaw_offset_deg = nan()
        self.yaw_outlier = False
        self.lat_outlier = False
        self.block_angle_deg = nan()
        self.resize_scale = 1.0
        self.padding_left = 0
        self.padding_top = 0
        self.active_w = 0
        self.active_h = 0


class HoleGuidedAlignment:
    def __init__(self, cfg):
        self.cfg = cfg
        self.video_cfg = cfg["video"]
        self.hole_cfg = cfg["hole_detection"]
        self.block_cfg = cfg["block_detection"]
        self.yaw_cfg = cfg["yaw"]
        self.filt_cfg = cfg["filter"]
        self.align_cfg = cfg["alignment"]
        self.cal_cfg = cfg["calibration"]
        self.sp = cfg["setpoint"]

        self.canvas_w = int(self.video_cfg["processing_width"])
        self.canvas_h = int(self.video_cfg["processing_height"])
        self.setpoint_x_min = float(self.sp.get("x_min_px", self.sp.get("x_px", 640)))
        self.setpoint_x_max = float(self.sp.get("x_max_px", self.sp.get("x_px", 640)))
        self.setpoint_y_min = float(self.sp.get("y_min_px", self.sp.get("y_px", 640)))
        self.setpoint_y_max = float(self.sp.get("y_max_px", self.sp.get("y_px", 640)))
        if self.setpoint_x_min > self.setpoint_x_max:
            self.setpoint_x_min, self.setpoint_x_max = self.setpoint_x_max, self.setpoint_x_min
        if self.setpoint_y_min > self.setpoint_y_max:
            self.setpoint_y_min, self.setpoint_y_max = self.setpoint_y_max, self.setpoint_y_min
        self.setpoint_x = float(self.sp.get(
            "x_px", 0.5 * (self.setpoint_x_min + self.setpoint_x_max)))
        self.setpoint_y = float(self.sp.get(
            "y_px", 0.5 * (self.setpoint_y_min + self.setpoint_y_max)))
        self.focal_x_px = max(1.0, float(self.sp.get("focal_length_x_px", self.canvas_w)))

        self.hole_class = str(self.hole_cfg["class_name"])
        self.block_class = str(self.block_cfg["class_name"])
        self.hole_conf = float(self.hole_cfg["minimum_confidence"])
        self.block_conf = float(self.block_cfg["minimum_confidence"])
        self.sep_min = float(self.hole_cfg["minimum_separation_px"])
        self.sep_max = float(self.hole_cfg["maximum_separation_px"])
        self.vert_max = float(self.hole_cfg["maximum_vertical_difference_px"])
        self.size_ratio_max = float(self.hole_cfg["maximum_size_ratio"])
        self.center_jump_max = float(self.filt_cfg.get("maximum_hole_center_jump_px",
                                                       self.hole_cfg.get("maximum_center_jump_px", 30)))
        self.hole_gate_req = max(1, int(self.hole_cfg.get("pair_gate_frames", 2)))

        self.offset_along = self.block_cfg.get("block_offset_along_centerline_px")
        self.offset_perp = float(self.block_cfg.get("block_offset_perpendicular_px", 0.0))
        self.roi_hw = float(self.block_cfg.get("roi_half_width_px", 60))
        self.roi_hh = float(self.block_cfg.get("roi_half_height_px", 50))
        self.max_residual = float(self.block_cfg.get("maximum_block_residual_px", 35))
        self.ar_min = self.block_cfg.get("minimum_aspect_ratio")
        self.ar_max = self.block_cfg.get("maximum_aspect_ratio")

        self.yaw_offset = float(self.yaw_cfg.get("setpoint_yaw_offset_deg", 0.0))
        self.max_yaw_jump = float(self.yaw_cfg.get("maximum_yaw_jump_deg", 8.0))

        self.hole_ema = EMA2(float(self.filt_cfg.get("hole_position_smoothing_alpha", 0.25)))
        self.lat_ema_px = EMA(float(self.filt_cfg.get("lateral_error_smoothing_alpha", 0.25)))
        self.lat_ema_m = EMA(float(self.filt_cfg.get("lateral_error_smoothing_alpha", 0.25)))
        self.yaw_ema = YawEMA(float(self.filt_cfg.get("yaw_error_smoothing_alpha", 0.20)))

        self.max_exp_jump = float(self.filt_cfg.get("maximum_expected_center_jump_px", 30))
        self.max_lat_jump_m = float(self.filt_cfg.get("maximum_lateral_error_jump_m", 0.05))

        self.lat_tol_px = float(self.align_cfg.get("lateral_tolerance_px", 10))
        self.lat_tol_m = float(self.align_cfg.get("lateral_tolerance_m", 0.02))
        self.yaw_tol = float(self.align_cfg.get("yaw_tolerance_deg", 1.0))
        self.stable_req = int(self.align_cfg.get("stable_frame_requirement", 10))

        mpp = self.cal_cfg.get("meter_per_pixel_at_working_distance")
        self.mpp = float(mpp) if mpp is not None and float(mpp) > 0.0 else None
        self.metric_valid = bool(self.cal_cfg.get("metric_calibration_valid", False)) and self.mpp is not None
        ymin = self.cal_cfg.get("calibration_y_min_px")
        ymax = self.cal_cfg.get("calibration_y_max_px")
        self.cal_ymin = float(ymin) if ymin is not None else None
        self.cal_ymax = float(ymax) if ymax is not None else None

        # temporal state
        self.prev_pair = None
        self.prev_expected = None
        self.prev_lat_px_f = None
        self.stable_count = 0
        self.hole_gate_count = 0
        self.offset_samples = []
        # Pallet target is defined strictly by the physical construction
        # hole pallet --- center block --- hole pallet.  Do not auto-learn an
        # offset from arbitrary block detections because that can lock onto a
        # neighbouring block and move the alignment target away from center.
        self.offset_auto_active = False
        self.offset_auto_value = 0.0
        self.last_status = "INVALID"

    # ------------------------------------------------------------------
    # Detection source: subscribed ObstacleArray (already in image pixels
    # of /camera/color/image_raw). We convert to canvas coords via letterbox.
    def detect_from_obstacles(self, obstacles, scale, pad_x, pad_y):
        """Convert ObstacleArray boxes (image space) -> canvas space dicts."""
        holes = []
        blocks = []
        if obstacles is None:
            return holes, blocks
        for obs in obstacles.obstacles:
            cx = obs.center_x
            cy = obs.center_y
            w = obs.x2 - obs.x1
            h = obs.y2 - obs.y1
            # un-letterbox: image = canvas*scale + pad
            ccx = cx * scale + pad_x
            ccy = cy * scale + pad_y
            cw = w * scale
            ch = h * scale
            d = {
                "box": (ccx - cw / 2.0, ccy - ch / 2.0, ccx + cw / 2.0, ccy + ch / 2.0),
                "conf": float(obs.confidence),
                "cx": ccx,
                "cy": ccy,
                "w": cw,
                "h": ch,
            }
            if obs.class_name == self.hole_class:
                holes.append(d)
            elif obs.class_name == self.block_class:
                blocks.append(d)
        return holes, blocks

    def pick_hole_pair(self, holes):
        """All combinations; best (left, right) or None. Left = smaller center_x."""
        if len(holes) < 2:
            return None
        best = None
        best_score = float("inf")
        for i in range(len(holes)):
            for j in range(i + 1, len(holes)):
                a, b = holes[i], holes[j]
                if a["conf"] < self.hole_conf or b["conf"] < self.hole_conf:
                    continue
                if abs(a["cx"] - b["cx"]) < 1e-6:
                    continue
                left, right = (a, b) if a["cx"] < b["cx"] else (b, a)
                sep = right["cx"] - left["cx"]
                if sep < self.sep_min or sep > self.sep_max:
                    continue
                if abs(left["cy"] - right["cy"]) > self.vert_max:
                    continue
                size_l = max(left["w"] * left["h"], 1.0)
                size_r = max(right["w"] * right["h"], 1.0)
                if max(size_l, size_r) / min(size_l, size_r) > self.size_ratio_max:
                    continue
                if self.prev_pair is not None:
                    (plx, ply), (prx, pry) = self.prev_pair
                    jl = math.hypot(left["cx"] - plx, left["cy"] - ply)
                    jr = math.hypot(right["cx"] - prx, right["cy"] - pry)
                    if jl > self.center_jump_max or jr > self.center_jump_max:
                        continue
                score = (abs(left["cy"] - right["cy"])
                         + (1.0 - left["conf"]) * 20 + (1.0 - right["conf"]) * 20)
                if self.prev_pair is not None:
                    (plx, ply), (prx, pry) = self.prev_pair
                    score += math.hypot(left["cx"] - plx, left["cy"] - ply)
                    score += math.hypot(right["cx"] - prx, right["cy"] - pry)
                if score < best_score:
                    best_score = score
                    best = (left, right)
        return best

    def geometry_from_pair(self, left, right):
        """midpoint, hole unit vector, downward normal, separation, line angle."""
        mx = (left["cx"] + right["cx"]) / 2.0
        my = (left["cy"] + right["cy"]) / 2.0
        vx = right["cx"] - left["cx"]
        vy = right["cy"] - left["cy"]
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return None
        ux, uy = vx / length, vy / length
        nx, ny = -uy, ux
        if ny < 0:
            nx, ny = -nx, -ny
        return {
            "midpoint": (mx, my),
            "unit": (ux, uy),
            "normal": (nx, ny),
            "separation": length,
            "angle": math.degrees(math.atan2(vy, vx)),
        }

    def expected_block_center(self, geom):
        """Target center is exactly midway between the two validated pallet holes."""
        mx, my = geom["midpoint"]
        return (mx, my), 0.0

    def find_block_in_roi(self, expected, blocks, geom):
        """Select only the block physically between the two validated pallet holes."""
        ex, ey = expected
        x1 = max(0, int(ex - self.roi_hw))
        y1 = max(0, int(ey - self.roi_hh))
        x2 = min(self.canvas_w, int(ex + self.roi_hw))
        y2 = min(self.canvas_h, int(ey + self.roi_hh))
        roi = (x1, y1, x2, y2)
        if x2 - x1 < 10 or y2 - y1 < 10:
            return None, roi, []

        mx, my = geom["midpoint"]
        ux, uy = geom["unit"]
        nx, ny = geom["normal"]
        half_sep = 0.5 * geom["separation"]
        middle = []
        for b in blocks:
            if b["conf"] < self.block_conf:
                continue
            if not (x1 <= b["cx"] <= x2 and y1 <= b["cy"] <= y2):
                continue
            if self.ar_min is not None or self.ar_max is not None:
                ar = b["w"] / max(b["h"], 1e-6)
                if self.ar_min is not None and ar < self.ar_min:
                    continue
                if self.ar_max is not None and ar > self.ar_max:
                    continue

            dx = b["cx"] - mx
            dy = b["cy"] - my
            along = dx * ux + dy * uy
            perp = dx * nx + dy * ny
            # Center must lie on the segment between both hole centers and
            # remain close to the hole line. Blocks outside this construction
            # are never allowed to become the alignment target.
            if abs(along) > half_sep or abs(perp) > self.roi_hh:
                continue
            middle.append(b)

        if not middle:
            return None, roi, []
        best = min(
            middle,
            key=lambda b: math.hypot(b["cx"] - ex, b["cy"] - ey)
            + (1.0 - b["conf"]) * 20.0)
        return best, roi, middle

    def estimate_block_angle(self, canvas, box):
        """minAreaRect orientation of block ROI (diagnostic only)."""
        x1, y1, x2, y2 = map(int, box)
        bw, bh = x2 - x1, y2 - y1
        m = max(6, int(0.12 * max(bw, bh)))
        x1m, y1m = max(0, x1 - m), max(0, y1 - m)
        x2m, y2m = min(self.canvas_w, x2 + m), min(self.canvas_h, y2 + m)
        if x2m - x1m < 8 or y2m - y1m < 8:
            return nan()
        roi = canvas[y1m:y2m, x1m:x2m]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        if cv2.countNonZero(cv2.bitwise_not(mask)) > cv2.countNonZero(mask):
            mask = cv2.bitwise_not(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return nan()
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 40:
            return nan()
        rect = cv2.minAreaRect(cnt)
        (_, _), (rw, rh), rangle = rect
        if rw < 4 or rh < 4:
            return nan()
        a = float(rangle)
        if rw < rh:
            a += 90.0
        while a >= 90.0:
            a -= 180.0
        while a < -90.0:
            a += 180.0
        return a

    # ------------------------------------------------------------------
    def process_frame(self, canvas, obstacles):
        m = Measurement()
        m.setpoint_x_px = self.setpoint_x
        m.setpoint_y_px = self.setpoint_y
        m.setpoint_x_min_px = self.setpoint_x_min
        m.setpoint_x_max_px = self.setpoint_x_max
        m.setpoint_y_min_px = self.setpoint_y_min
        m.setpoint_y_max_px = self.setpoint_y_max
        m.offset_auto = self.offset_auto_active
        m.stable_req = self.stable_req
        m.hole_gate_req = self.hole_gate_req
        m.setpoint_yaw_offset_deg = self.yaw_offset
        # Publish the configured m/px estimate so lateral error remains visible
        # even before physical metric calibration is certified.  The separate
        # metric_error_valid flag still remains false until calibration is valid.
        m.meter_per_pixel = self.mpp if self.mpp is not None else nan()

        # convert detections to canvas coords (needs scale/pad from letterbox)
        scale = getattr(self, "_last_scale", 1.0)
        pad_x = getattr(self, "_last_pad_x", 0)
        pad_y = getattr(self, "_last_pad_y", 0)
        holes, blocks = self.detect_from_obstacles(obstacles, scale, pad_x, pad_y)

        pair = self.pick_hole_pair(holes)
        no_pair = pair is None
        if no_pair:
            self.hole_ema.reset()
            self.yaw_ema.reset()
            self.lat_ema_px.reset()
            self.lat_ema_m.reset()
            self.prev_pair = None
            self.prev_expected = None
            self.prev_lat_px_f = None
            self.stable_count = 0
            self.hole_gate_count = 0
            m.status = "HOLES LOST" if self.last_status not in ("INVALID", "HOLES LOST") else "INVALID"
            self.last_status = m.status
            return m

        left, right = pair
        m.left_hole_valid = True
        m.right_hole_valid = True
        m.left_hole_center = (left["cx"], left["cy"])
        m.right_hole_center = (right["cx"], right["cy"])
        m.left_hole_conf = left["conf"]
        m.right_hole_conf = right["conf"]

        geom = self.geometry_from_pair(left, right)
        if geom is None:
            m.status = "HOLE PAIR INVALID"
            self.last_status = m.status
            return m
        m.hole_pair_valid = True
        m.hole_separation_px = geom["separation"]
        m.holes_line_angle_deg = geom["angle"]

        lx, ly = self.hole_ema.update(left["cx"], left["cy"])
        s_left = {"cx": lx, "cy": ly}
        s_right = {"cx": lx + (right["cx"] - left["cx"]), "cy": ly + (right["cy"] - left["cy"])}
        sgeom = self.geometry_from_pair(s_left, s_right)
        if sgeom is None:
            sgeom = geom
        m.holes_midpoint = sgeom["midpoint"]
        m.centerline_normal = sgeom["normal"]
        m.hole_unit = sgeom["unit"]

        expected, offset = self.expected_block_center(sgeom)
        if self.offset_auto_active:
            pass
        if self.prev_expected is not None:
            jump = math.hypot(expected[0] - self.prev_expected[0],
                              expected[1] - self.prev_expected[1])
            if jump > self.max_exp_jump:
                m.status = "EXPECTED CENTER JUMP"
                self.last_status = m.status
                return m
        m.expected_block_center = expected
        m.block_offset_along_centerline_px = offset
        m.block_offset_perpendicular_px = self.offset_perp

        # Explicit hole-first temporal gate. A block may already be present in
        # the YOLO array, but it is ignored until the same valid left+right hole
        # pair has been observed for the configured number of consecutive frames.
        self.hole_gate_count = min(self.hole_gate_count + 1, self.hole_gate_req)
        m.hole_gate_count = self.hole_gate_count
        if self.hole_gate_count < self.hole_gate_req:
            m.status = "LOCKING HOLE PAIR"
            self.prev_pair = (m.left_hole_center, m.right_hole_center)
            self.prev_expected = expected
            self.last_status = m.status
            return m

        best, roi, middle_candidates = self.find_block_in_roi(expected, blocks, sgeom)
        # Report only candidates that satisfy hole---block---hole geometry;
        # unrelated block detections are deliberately excluded here.
        m.candidate_count = len(middle_candidates)
        m.block_roi = roi

        if best is not None:
            m.block_detected = True
            m.block_confidence = best["conf"]
            m.detected_block_center = (best["cx"], best["cy"])
            m.detected_block_box = best["box"]
            rx = best["cx"] - expected[0]
            ry = best["cy"] - expected[1]
            m.block_residual_x_px = rx
            m.block_residual_y_px = ry
            m.block_residual_distance_px = math.hypot(rx, ry)
            m.block_angle_deg = self.estimate_block_angle(canvas, best["box"])
            cross = (right["cy"] - left["cy"]) * (best["cx"] - left["cx"]) - \
                    (right["cx"] - left["cx"]) * (best["cy"] - left["cy"])
            below = cross > -5.0
            m.block_geometry_valid = m.block_residual_distance_px <= self.max_residual and below
        else:
            m.block_residual_distance_px = nan()
            m.block_angle_deg = nan()

        if self.offset_auto_active and m.block_detected and m.block_geometry_valid:
            bx, by = best["cx"], best["cy"]
            mx, my = sgeom["midpoint"]
            nx, ny = sgeom["normal"]
            ux, uy = sgeom["unit"]
            along = (bx - mx) * nx + (by - my) * ny
            perp = (bx - mx) * ux + (by - my) * uy
            self.offset_samples.append((along, perp))
            if len(self.offset_samples) >= 30:
                al = statistics.median(s[0] for s in self.offset_samples)
                pe = statistics.median(s[1] for s in self.offset_samples)
                self.offset_auto_value = al
                self.offset_auto_active = False
                print(f"[CALIB] auto block_offset_along_centerline_px = {al:.1f} "
                      f"(perp {pe:.1f}) from {len(self.offset_samples)} frames")

        basic_valid = (m.left_hole_valid and m.right_hole_valid and m.hole_pair_valid
                       and m.block_detected and m.block_geometry_valid)

        # Camera yaw error is generated only from the SELECTED middle block.
        # The two pallet holes remain the gate/reference used to decide which block
        # is physically the middle one; unrelated blocks never produce yaw output.
        # Pinhole bearing: + error means the middle block is to the right of the
        # configured camera/fork setpoint center.
        if basic_valid:
            bx, by = best["cx"], best["cy"]
            dx_px = bx - self.setpoint_x
            raw_yaw = normalize_angle_deg(
                math.degrees(math.atan2(dx_px, self.focal_x_px)) - self.yaw_offset)
            m.raw_error_yaw_deg = raw_yaw
            if is_num(raw_yaw):
                prev = self.yaw_ema.value
                if prev is not None and abs(normalize_angle_deg(raw_yaw - prev)) > self.max_yaw_jump:
                    m.yaw_outlier = True
                else:
                    self.yaw_ema.update(raw_yaw)
            m.filtered_error_yaw_deg = self.yaw_ema.value
        else:
            self.yaw_ema.reset()
            m.raw_error_yaw_deg = nan()
            m.filtered_error_yaw_deg = nan()

        if basic_valid:
            # Lateral output uses the straight-line center-to-center distance from
            # the selected middle block to the rectangular camera setpoint center.
            # Preserve the existing ROS sign convention: negative=left, positive=right.
            bx, by = best["cx"], best["cy"]
            m.alignment_center = (bx, by)
            dx_px = bx - self.setpoint_x
            dy_px = by - self.setpoint_y
            straight_px = math.hypot(dx_px, dy_px)
            raw_px = -straight_px if dx_px < 0.0 else straight_px
            m.raw_error_lateral_px = raw_px
            if self.prev_lat_px_f is not None and \
                    abs(raw_px - self.prev_lat_px_f) > self.max_lat_jump_m / (self.mpp or 1e-9):
                m.lat_outlier = True
            else:
                self.lat_ema_px.update(raw_px)
                if self.mpp is not None:
                    self.lat_ema_m.update(raw_px * self.mpp)
            self.prev_lat_px_f = self.lat_ema_px.value
            m.filtered_error_lateral_px = self.lat_ema_px.value

            # Keep an estimated metric lateral value visible whenever m/px is
            # configured. metric_error_valid remains the independent flag that
            # says whether this conversion has been physically calibrated.
            if self.mpp is not None:
                m.raw_error_lateral_m = raw_px * self.mpp
                m.filtered_error_lateral_m = self.lat_ema_m.value
                in_zone = (self.cal_ymin is None or by >= self.cal_ymin) and \
                          (self.cal_ymax is None or by <= self.cal_ymax)
                m.metric_error_valid = bool(self.metric_valid and in_zone)
                if self.metric_valid and not in_zone:
                    m.raw_error_lateral_m = nan()
                    m.filtered_error_lateral_m = nan()
        else:
            self.lat_ema_px.reset()
            self.lat_ema_m.reset()
            self.prev_lat_px_f = None

        within = False
        if basic_valid and is_num(m.filtered_error_yaw_deg):
            if m.metric_error_valid and is_num(m.filtered_error_lateral_m):
                within = (abs(m.filtered_error_lateral_m) <= self.lat_tol_m
                          and abs(m.filtered_error_yaw_deg) <= self.yaw_tol)
            else:
                within = (abs(m.filtered_error_lateral_px) <= self.lat_tol_px
                          and abs(m.filtered_error_yaw_deg) <= self.yaw_tol)
        self.stable_count = self.stable_count + 1 if within else 0
        m.stable_frame_count = self.stable_count
        m.detection_stable = self.stable_count >= self.stable_req

        m.measurement_valid = basic_valid and m.detection_stable

        m.aligned = (m.measurement_valid and m.metric_error_valid
                     and is_num(m.filtered_error_lateral_m)
                     and abs(m.filtered_error_lateral_m) <= self.lat_tol_m
                     and is_num(m.filtered_error_yaw_deg)
                     and abs(m.filtered_error_yaw_deg) <= self.yaw_tol)

        if not basic_valid:
            if not m.block_detected:
                m.status = "BLOCK NOT IN ROI"
            elif not m.block_geometry_valid:
                m.status = "BLOCK GEOMETRY MISMATCH"
            elif not m.hole_pair_valid:
                m.status = "HOLE PAIR INVALID"
            else:
                m.status = "INVALID"
        elif m.aligned:
            m.status = "ALIGNED"
        elif not m.metric_error_valid:
            m.status = "METRIC NOT CALIBRATED / OUT OF ZONE"
        elif not m.detection_stable:
            m.status = "NOT STABLE"
        else:
            m.status = "ALIGNING"
        self.last_status = m.status

        self.prev_pair = (m.left_hole_center, m.right_hole_center)
        self.prev_expected = expected
        return m


# ----------------------------------------------------------------------------
# Drawing
# ----------------------------------------------------------------------------
def draw_marker(img, pt, color, kind="circle", size=8):
    x, y = int(pt[0]), int(pt[1])
    if kind == "circle":
        cv2.circle(img, (x, y), size, color, 2, cv2.LINE_AA)
    elif kind == "diamond":
        pts = np.array([[x, y - size], [x + size, y], [x, y + size], [x - size, y]], np.int32)
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], True, color, 2, cv2.LINE_AA)
    elif kind == "cross":
        cv2.drawMarker(img, (x, y), color, cv2.MARKER_CROSS, size * 2, 2, cv2.LINE_AA)
    elif kind == "arrow_up":
        pts = np.array([[x, y - size], [x - size + 2, y], [x + size - 2, y]], np.int32)
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], True, color, 2, cv2.LINE_AA)
        cv2.circle(img, (x, y), size - 1, color, 1, cv2.LINE_AA)


def fmt(v, d=1, suf=""):
    if not is_num(v):
        return "N/A"
    return f"{v:+.{d}f}{suf}" if suf else f"{v:.{d}f}"


def draw_hud(canvas, m, max_residual):
    """Minimal geometry overlay; all textual indicators live in the Web GUI."""
    _ = max_residual
    h, w = canvas.shape[:2]
    spx, spy = int(m.setpoint_x_px), int(m.setpoint_y_px)
    sx1 = max(0, min(w - 1, int(m.setpoint_x_min_px)))
    sx2 = max(0, min(w - 1, int(m.setpoint_x_max_px)))
    sy1 = max(0, min(h - 1, int(m.setpoint_y_min_px)))
    sy2 = max(0, min(h - 1, int(m.setpoint_y_max_px)))

    cv2.rectangle(canvas, (sx1, sy1), (sx2, sy2), (0, 0, 255), 2, cv2.LINE_AA)
    draw_marker(canvas, (spx, spy), (0, 0, 255), "cross", 9)

    if m.left_hole_valid and m.right_hole_valid:
        lx, ly = map(int, m.left_hole_center)
        rx, ry = map(int, m.right_hole_center)
        cv2.line(canvas, (lx, ly), (rx, ry), (0, 255, 255), 2, cv2.LINE_AA)
        draw_marker(canvas, (lx, ly), (0, 255, 255), "circle", 7)
        draw_marker(canvas, (rx, ry), (0, 255, 255), "circle", 7)
        mx, my = map(int, m.holes_midpoint)
        draw_marker(canvas, (mx, my), (0, 255, 255), "diamond", 6)

    if m.block_detected:
        bx, by = map(int, m.detected_block_center)
        if m.detected_block_box is not None:
            bx1, by1, bx2, by2 = map(int, m.detected_block_box)
            cv2.rectangle(canvas, (bx1, by1), (bx2, by2), (0, 255, 0), 2, cv2.LINE_AA)
        draw_marker(canvas, (bx, by), (0, 255, 0), "circle", 6)
        cv2.arrowedLine(canvas, (bx, by), (spx, spy), (255, 0, 255), 2,
                        cv2.LINE_AA, tipLength=0.08)


# ----------------------------------------------------------------------------
# Letterbox
# ----------------------------------------------------------------------------
def letterbox(frame, tw, th):
    """Letterbox frame to target size, preserving aspect ratio.
    Returns: canvas, scale, pad_x, pad_y, active_w, active_h"""
    h, w = frame.shape[:2]
    if w == tw and h == th:
        # Keep the source view for computation. The caller copies only when a
        # HUD/video frame is actually needed, avoiding a 1280x720 memcpy on
        # every control frame.
        return frame, 1.0, 0, 0, w, h
    scale = min(tw / w, th / h)
    nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((th, tw, 3), np.uint8)
    px, py = (tw - nw) // 2, (th - nh) // 2
    canvas[py:py + nh, px:px + nw] = resized
    return canvas, scale, px, py, nw, nh


# ----------------------------------------------------------------------------
# ROS 2 node
# ----------------------------------------------------------------------------
class HoleBlockAlignmentNode(Node):
    def __init__(self):
        super().__init__("hole_block_alignment_node")

        self.declare_parameter("config",
            "/home/otomasi2/ros/src/yolo_obstacle_detection_ros2/hole_block_alignment/alignment_realtime.yaml")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("obstacle_topic", "/obstacle_detection/obstacles")
        self.declare_parameter("state_topic", "/fork_alignment/state")
        self.declare_parameter("output_image_topic", "/fork_alignment/image")
        self.declare_parameter("display", 0)
        self.declare_parameter("save_csv", True)
        self.declare_parameter("save_output_video", False)
        # 0 = publish every unique input frame. A positive value is an optional
        # operator-side preview cap; autonomous V25 keeps this at 0 so no ROS
        # topic frequency is reduced by the performance fix.
        self.declare_parameter("visualization_fps", 0.0)

        cfg_path = self.get_parameter("config").value
        display = int(self.get_parameter("display").value)
        save_csv = bool(self.get_parameter("save_csv").value)
        save_video = bool(self.get_parameter("save_output_video").value)
        self.visualization_fps = max(0.0, float(self.get_parameter("visualization_fps").value))

        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        # Mirror every scalar from the custom alignment YAML into ROS parameters.
        # The web tuner restarts this node after a YAML edit and then reads these
        # parameters back; APPLIED therefore means the algorithm loaded the value,
        # not merely that a text file changed.
        for cfg_path_key, cfg_value in _flatten_scalar_cfg(cfg):
            param_name = "cfg." + cfg_path_key
            self.declare_parameter(param_name, cfg_value)
            _set_cfg_path(cfg, cfg_path_key, self.get_parameter(param_name).value)

        self.stage = HoleGuidedAlignment(cfg)
        control_cfg = cfg.get("control", {})
        self.kp_lateral = float(control_cfg.get("kp_lateral", 1.0))
        self.kp_yaw = float(control_cfg.get("kp_yaw", 1.0))
        self.approach_speed_mps = max(0.0, float(control_cfg.get("approach_speed_mps", 0.08)))
        self.max_steering_rad = max(0.01, abs(float(control_cfg.get("max_steering_rad", 0.34))))
        self.control_wheelbase_m = max(0.05, float(control_cfg.get("wheelbase_m", 0.70)))
        self.control_output_enabled = bool(control_cfg.get("output_enabled", False))
        self.control_preview_topic = str(control_cfg.get("preview_topic", "/docking/cmd_vel_preview"))
        self.control_output_topic = str(control_cfg.get("output_topic", "/cmd_vel_docking_raw"))
        self._last_control = {
            "valid": False, "lat_term": 0.0, "yaw_term": 0.0,
            "steering_rad": 0.0, "yaw_rate": 0.0, "speed_mps": 0.0,
            "limited": False, "reason": "WAIT_ALIGNMENT"}
        self.bridge = CvBridge()
        self.display = display

        # output paths
        out_dir = Path(cfg["video"].get("output_dir",
                         "/home/otomasi2/ros/log/a2_alignment_validation")).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = out_dir / f"alignment_{ts}.csv"
        self.video_path = out_dir / f"alignment_{ts}.mp4"

        self.csv_file = None
        self.csv_writer = None
        if save_csv:
            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "frame", "timestamp",
                "proc_w", "proc_h",
                "resize_scale", "padding_left_px", "padding_top_px",
                "active_image_width_px", "active_image_height_px",
                "setpoint_x_px", "setpoint_y_px",
                "left_hole_x", "left_hole_y", "right_hole_x", "right_hole_y",
                "holes_midpoint_x", "holes_midpoint_y", "hole_separation_px", "holes_line_angle_deg",
                "expected_block_x", "expected_block_y",
                "detected_block_x", "detected_block_y", "block_residual_px",
                "raw_error_lateral_px", "filtered_error_lateral_px",
                "raw_error_lateral_m", "filtered_error_lateral_m",
                "raw_error_yaw_deg", "filtered_error_yaw_deg",
                "metric_error_valid", "detection_stable", "aligned", "status",
            ])

        self.video_writer = None
        if save_video:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(
                str(self.video_path), fourcc, 20.0,
                (self.stage.canvas_w, self.stage.canvas_h))

        # subscriptions
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.img_sub = self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self.img_cb, img_qos)
        self.obs_sub = self.create_subscription(
            ObstacleArray, self.get_parameter("obstacle_topic").value,
            self.obs_cb, 10)

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.state_pub = self.create_publisher(
            AlignmentState, self.get_parameter("state_topic").value, state_qos)
        self.img_pub = self.create_publisher(
            Image, self.get_parameter("output_image_topic").value, img_qos)
        self.docking_preview_pub = self.create_publisher(Twist, self.control_preview_topic, 10)
        self.docking_output_pub = self.create_publisher(Twist, self.control_output_topic, 10)

        self.latest_img = None
        self.latest_obs = None
        self.frame_idx = 0
        self._last_t = time.perf_counter()
        self._image_seq = 0
        self._processed_image_seq = -1
        self._last_viz_pub = 0.0
        self._last_processed_monotonic = 0.0
        self._last_waiting_state_pub = 0.0
        # Lightweight browser telemetry lives in tmpfs; no SSD write load.
        self.web_status_path = Path("/dev/shm/agv_alignment_status.json")
        self._last_web_status_write = 0.0
        self._web_status_warned = False

        # Poll faster than the camera so a newly arrived frame is picked up
        # promptly, but tick() below guarantees each image is processed ONCE.
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        # Keep /fork_alignment/state alive while the camera is enumerating or
        # reconnecting. This is an explicit INVALID/WAITING state, not fake
        # alignment data, and prevents an ambiguous persistent "NODE UP" card.
        self.heartbeat_timer = self.create_timer(0.5, self.publish_waiting_state)

        self.get_logger().info(
            f"HoleBlockAlignment ready: canvas={self.stage.canvas_w}x{self.stage.canvas_h} "
            f"setpoint=({self.stage.setpoint_x:.0f},{self.stage.setpoint_y:.0f}) "
            f"metric_valid={self.stage.metric_valid} mpp={self.stage.mpp} "
            f"csv={self.csv_path}")

    def _compute_docking_control(self, m):
        metric_ok = bool(m.metric_error_valid and is_num(m.filtered_error_lateral_m))
        yaw_ok = is_num(m.filtered_error_yaw_deg)
        geometry_ok = bool(m.hole_pair_valid and m.block_detected and m.block_geometry_valid)
        valid = bool(metric_ok and yaw_ok and geometry_ok)
        reason = "VALID" if valid else ("WAIT_METRIC_CALIBRATION" if geometry_ok and yaw_ok and not metric_ok else "WAIT_ALIGNMENT")
        e_lat = float(m.filtered_error_lateral_m) if metric_ok else 0.0
        e_yaw_rad = math.radians(float(m.filtered_error_yaw_deg)) if yaw_ok else 0.0
        lat_term = self.kp_lateral * e_lat if valid else 0.0
        yaw_term = self.kp_yaw * e_yaw_rad if valid else 0.0
        raw_steer = -(lat_term + yaw_term) if valid else 0.0
        steering = max(-self.max_steering_rad, min(self.max_steering_rad, raw_steer))
        limited = valid and abs(raw_steer - steering) > 1e-9
        speed = 0.0 if (not valid or m.aligned) else self.approach_speed_mps
        yaw_rate = (speed / self.control_wheelbase_m) * math.tan(steering) if speed > 0.0 else 0.0
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(yaw_rate)
        self.docking_preview_pub.publish(cmd)
        if self.control_output_enabled:
            # Dedicated topic only; autonomous ESC mux is intentionally NOT wired
            # to this topic by this patch. Invalid data always produces zero.
            self.docking_output_pub.publish(cmd)
        return {"valid": valid, "lat_term": lat_term, "yaw_term": yaw_term,
                "steering_rad": steering, "yaw_rate": yaw_rate, "speed_mps": speed,
                "limited": limited, "reason": reason}

    def img_cb(self, msg):
        self.latest_img = msg
        self._image_seq += 1

    def obs_cb(self, msg):
        self.latest_obs = msg

    def _publish_state(self, state: AlignmentState):
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "base_link"
        state.pallet_header_stamp = state.header.stamp
        self.state_pub.publish(state)

    def _write_web_status(self, m=None, status=None, force=False):
        """Publish GUI-only alignment indicators to tmpfs at <=10 Hz."""
        now = time.perf_counter()
        if not force and (now - self._last_web_status_write) < 0.10:
            return

        def num(v):
            return float(v) if is_num(v) else None

        def point(pt):
            return [num(pt[0]), num(pt[1])] if pt is not None else [None, None]

        if m is None:
            data = {
                "available": False,
                "at_ms": int(time.time() * 1000),
                "status": str(status or "WAITING"),
            }
        else:
            data = {
                "available": True,
                "at_ms": int(time.time() * 1000),
                "status": str(m.status or "--"),
                "left_hole_valid": bool(m.left_hole_valid),
                "right_hole_valid": bool(m.right_hole_valid),
                "hole_pair_valid": bool(m.hole_pair_valid),
                "hole_gate_count": int(m.hole_gate_count),
                "hole_gate_req": int(m.hole_gate_req),
                "block_detected": bool(m.block_detected),
                "candidate_count": int(m.candidate_count),
                "block_geometry_valid": bool(m.block_geometry_valid),
                "block_residual_px": num(m.block_residual_distance_px),
                "stable_frame_count": int(m.stable_frame_count),
                "stable_req": int(m.stable_req),
                "left_hole_center": point(m.left_hole_center),
                "right_hole_center": point(m.right_hole_center),
                "holes_midpoint": point(m.holes_midpoint),
                "hole_separation_px": num(m.hole_separation_px),
                "centerline_normal": point(m.centerline_normal),
                "block_offset_px": num(m.block_offset_along_centerline_px),
                "expected_block_center": point(m.expected_block_center),
                "detected_block_center": point(m.detected_block_center),
                "detected_block_box": [num(v) for v in m.detected_block_box] if m.detected_block_box is not None else None,
                "setpoint_center": [num(m.setpoint_x_px), num(m.setpoint_y_px)],
                "setpoint_box": [num(m.setpoint_x_min_px), num(m.setpoint_x_max_px),
                                 num(m.setpoint_y_min_px), num(m.setpoint_y_max_px)],
                "yaw_offset_deg": num(m.setpoint_yaw_offset_deg),
                "error_lateral_m": num(m.filtered_error_lateral_m),
                "error_lateral_px": num(m.filtered_error_lateral_px),
                "error_yaw_deg": num(m.filtered_error_yaw_deg),
                "metric_error_valid": bool(m.metric_error_valid),
                "meter_per_pixel": num(m.meter_per_pixel),
                "aligned": bool(m.aligned),
                "controller_valid": bool(self._last_control.get("valid", False)),
                "controller_reason": str(self._last_control.get("reason", "WAIT")),
                "kp_lateral": float(self.kp_lateral),
                "kp_yaw": float(self.kp_yaw),
                "pid_lateral_output": float(self._last_control.get("lat_term", 0.0)),
                "pid_yaw_output": float(self._last_control.get("yaw_term", 0.0)),
                "steering_cmd_rad": float(self._last_control.get("steering_rad", 0.0)),
                "steering_cmd_deg": math.degrees(float(self._last_control.get("steering_rad", 0.0))),
                "linear_velocity_cmd": float(self._last_control.get("speed_mps", 0.0)),
                "angular_velocity_cmd": float(self._last_control.get("yaw_rate", 0.0)),
                "steering_limit_active": bool(self._last_control.get("limited", False)),
                "control_output_enabled": bool(self.control_output_enabled),
            }
        try:
            tmp = self.web_status_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(data, separators=(",", ":"), allow_nan=False), encoding="utf-8")
            os.replace(str(tmp), str(self.web_status_path))
            self._last_web_status_write = now
            self._web_status_warned = False
        except Exception as e:
            if not self._web_status_warned:
                self.get_logger().warning(f"web alignment status write failed: {e}")
                self._web_status_warned = True

    def publish_waiting_state(self):
        """Publish an explicit heartbeat only when no camera frame is flowing."""
        now = time.perf_counter()
        if self._last_processed_monotonic > 0.0 and (now - self._last_processed_monotonic) < 1.0:
            return
        if (now - self._last_waiting_state_pub) < 0.45:
            return
        state = AlignmentState()
        state.state = AlignmentState.UNKNOWN
        state.state_text = "WAITING CAMERA"
        state.data_valid = False
        state.pallet_detected = False
        state.detection_stable = False
        state.ready_for_insertion = False
        self._publish_state(state)
        self._write_web_status(None, "WAITING CAMERA")
        self._last_waiting_state_pub = now

    def tick(self):
        if self.latest_img is None or self._image_seq == self._processed_image_seq:
            return
        # Snapshot the message/sequence so a newer DDS callback can safely
        # replace latest_img while this frame is being processed.
        img_msg = self.latest_img
        img_seq = self._image_seq
        self._processed_image_seq = img_seq
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge conversion failed: {e}")
            return
        self._last_processed_monotonic = time.perf_counter()

        canvas, scale, pad_x, pad_y, act_w, act_h = letterbox(
            frame, self.stage.canvas_w, self.stage.canvas_h)
        self.stage._last_scale = scale
        self.stage._last_pad_x = pad_x
        self.stage._last_pad_y = pad_y

        m = self.stage.process_frame(canvas, self.latest_obs)
        m.resize_scale = scale
        m.padding_left = pad_x
        m.padding_top = pad_y
        m.active_w = act_w
        m.active_h = act_h
        self._last_control = self._compute_docking_control(m)
        self._write_web_status(m)

        # publish alignment state
        state = AlignmentState()
        state.pallet_detected = bool(m.block_detected)
        state.detection_stable = bool(m.detection_stable)
        state.confidence = float(m.block_confidence) if is_num(m.block_confidence) else 0.0
        state.depth_quality = 0.0
        state.error_lateral_m = float(m.filtered_error_lateral_m) if is_num(m.filtered_error_lateral_m) else 0.0
        state.error_yaw_deg = float(m.filtered_error_yaw_deg) if is_num(m.filtered_error_yaw_deg) else 0.0
        state.pid_lateral_output = float(self._last_control["lat_term"])
        state.pid_yaw_output = float(self._last_control["yaw_term"])
        state.desired_yaw_rate = float(self._last_control["yaw_rate"])
        state.estimated_steering_deg = float(math.degrees(self._last_control["steering_rad"]))
        state.linear_velocity_cmd = float(self._last_control["speed_mps"])
        state.angular_velocity_cmd = float(self._last_control["yaw_rate"])
        state.steering_limit_active = bool(self._last_control["limited"])
        state.safety_stop_active = not bool(self._last_control["valid"])
        state.data_valid = bool(m.measurement_valid)
        if m.aligned:
            state.state = AlignmentState.ALIGNED
            state.state_text = "ALIGNED"
        elif m.measurement_valid:
            state.state = AlignmentState.FINE_ALIGN
            state.state_text = str(m.status or "FINE ALIGN")
        elif m.block_detected or m.hole_pair_valid:
            state.state = AlignmentState.APPROACH
            state.state_text = str(m.status or "APPROACH")
        else:
            state.state = AlignmentState.UNKNOWN
            state.state_text = str(m.status or "WAITING DETECTION")
        state.lateral_within_tolerance = m.metric_error_valid and is_num(m.filtered_error_lateral_m) \
            and abs(m.filtered_error_lateral_m) <= 0.02
        state.yaw_within_tolerance = is_num(m.filtered_error_yaw_deg) and abs(m.filtered_error_yaw_deg) <= 1.0
        state.steering_centered = True
        state.ready_for_insertion = bool(m.aligned)
        self._publish_state(state)

        # Drawing a full 1280x720 HUD is display work, not control work. Keep
        # the alignment computation/state publication at every unique camera
        # frame, but only annotate when a preview/video/window actually needs it.
        now_perf = time.perf_counter()
        publish_visual = (self.visualization_fps <= 0.0) or (
            now_perf - self._last_viz_pub >= (1.0 / self.visualization_fps))
        viz_canvas = None
        if publish_visual or self.video_writer is not None or self.display:
            # HUD drawing is intentionally isolated from the control canvas. At
            # native 1280x720 this means a full-frame copy happens only at the
            # bounded visualization rate, not at every camera/control frame.
            viz_canvas = canvas.copy()
            draw_hud(viz_canvas, m, self.stage.max_residual)

        # Publish the annotated preview at a bounded UI/visualization rate.
        # This does NOT lower camera, obstacle, or alignment-state frequency.
        if publish_visual and viz_canvas is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(viz_canvas, encoding="bgr8")
                out_msg.header = img_msg.header
                self.img_pub.publish(out_msg)
                self._last_viz_pub = now_perf
            except Exception as e:
                self.get_logger().warning(f"img publish: {e}")

        if self.video_writer is not None and viz_canvas is not None:
            self.video_writer.write(viz_canvas)

        # CSV
        if self.csv_writer is not None:
            self.csv_writer.writerow([
                self.frame_idx,
                f"{time.time():.3f}",
                self.stage.canvas_w, self.stage.canvas_h,
                f"{scale:.4f}", pad_x, pad_y, act_w, act_h,
                int(m.setpoint_x_px), int(m.setpoint_y_px),
                *[f"{v:.1f}" if is_num(v) else "nan" for v in m.left_hole_center],
                *[f"{v:.1f}" if is_num(v) else "nan" for v in m.right_hole_center],
                *[f"{v:.1f}" if is_num(v) else "nan" for v in m.holes_midpoint],
                f"{m.hole_separation_px:.1f}" if is_num(m.hole_separation_px) else "nan",
                f"{m.holes_line_angle_deg:.2f}" if is_num(m.holes_line_angle_deg) else "nan",
                *[f"{v:.1f}" if is_num(v) else "nan" for v in m.expected_block_center],
                *[f"{v:.1f}" if is_num(v) else "nan" for v in m.detected_block_center],
                f"{m.block_residual_distance_px:.1f}" if is_num(m.block_residual_distance_px) else "nan",
                f"{m.raw_error_lateral_px:.1f}" if is_num(m.raw_error_lateral_px) else "nan",
                f"{m.filtered_error_lateral_px:.1f}" if is_num(m.filtered_error_lateral_px) else "nan",
                f"{m.raw_error_lateral_m:.4f}" if is_num(m.raw_error_lateral_m) else "nan",
                f"{m.filtered_error_lateral_m:.4f}" if is_num(m.filtered_error_lateral_m) else "nan",
                f"{m.raw_error_yaw_deg:.2f}" if is_num(m.raw_error_yaw_deg) else "nan",
                f"{m.filtered_error_yaw_deg:.2f}" if is_num(m.filtered_error_yaw_deg) else "nan",
                int(m.metric_error_valid), int(m.detection_stable), int(m.aligned), m.status,
            ])
            if self.frame_idx % 30 == 0:
                self.csv_file.flush()

        if self.display:
            cv2.imshow("hole_guided_alignment", canvas)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                rclpy.shutdown()
            elif k == ord("r"):
                self.stage.hole_ema.reset()
                self.stage.yaw_ema.reset()
                self.stage.lat_ema_px.reset()
                self.stage.lat_ema_m.reset()
                self.stage.stable_count = 0
                self.stage.prev_pair = None
                self.stage.prev_expected = None
            elif k == ord("s"):
                shot_dir = Path(self.csv_path).parent / "screenshots"
                shot_dir.mkdir(parents=True, exist_ok=True)
                p = shot_dir / f"frame_{self.frame_idx:06d}.png"
                cv2.imwrite(str(p), canvas)

        self.frame_idx += 1

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HoleBlockAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
