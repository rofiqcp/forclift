#!/usr/bin/python3
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import struct
import threading
import zlib
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

WORKSPACE = Path('/home/otomasi2/ros')
MAP_DIR = WORKSPACE / 'src/navigation/maps'
POINTER_DIR = WORKSPACE / 'maps'
LATEST_POINTER = POINTER_DIR / 'latest_map.txt'
PID_FILE = Path('/tmp/agv_mapping_runtime.pid')
WEB_STATIC_DIR = WORKSPACE / 'install/navigation/share/navigation/web/static'
# Final/saved map PNG must be visible from the GUI install-space actually in use.
# Live BAB 4.2 rendering does NOT use PNG; this is only for saved-map export.
GUI_STATIC_DIRS = tuple(dict.fromkeys((
    WEB_STATIC_DIR,
    Path('/home/otomasi2/forclift/install/navigation/share/navigation/web/static'),
)))
LIVE_MAP_PNG = WEB_STATIC_DIR / 'mapping_live.png'
SAVED_MAP_CATALOG = WEB_STATIC_DIR / 'saved_maps.json'
# During BAB 4.2 mapping, pause every Nav2 lifecycle group that is not
# required by the mapping sensor path. LiDAR, IMU, Hector /lidar/odom, TF,
# web GUI and the mapping bridge remain alive. This releases CPU for
# slam_toolbox construction/discovery on the Jetson, then all managers are
# resumed in reverse order after Stop/Save or on startup failure.
CRITICAL_MANAGERS = (
    'lifecycle_manager_navigation_aux',
    'lifecycle_manager_navigation_local',
    'lifecycle_manager_navigation_global',
    'lifecycle_manager_localization',
    'lifecycle_manager_planning_map',
    'lifecycle_manager_maps',
)
LIFECYCLE_MANAGERS = CRITICAL_MANAGERS
NAVIGATION_LIFECYCLE_NODES = (
    'collision_monitor',
    'velocity_smoother',
    'bt_navigator',
    'behavior_server',
    'controller_server',
    'planner_server',
    'amcl',
    'nav_map_server',
    'map_server',
)



def _png_chunk(kind, payload):
    return (struct.pack('>I', len(payload)) + kind + payload +
            struct.pack('>I', zlib.crc32(kind + payload) & 0xffffffff))


def occupancy_grid_png(msg):
    width, height = int(msg.info.width), int(msg.info.height)
    rows = []
    for y in range(height - 1, -1, -1):
        row = bytearray(width)
        base = y * width
        for x in range(width):
            occ = int(msg.data[base + x])
            pixel = 205
            if occ >= 65:
                pixel = 0
            elif 0 <= occ <= 25:
                pixel = 254
            row[x] = pixel
        rows.append(b'\x00' + bytes(row))
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    raw = b''.join(rows)
    return (b'\x89PNG\r\n\x1a\n' + _png_chunk(b'IHDR', ihdr) +
            _png_chunk(b'IDAT', zlib.compress(raw, 6)) + _png_chunk(b'IEND', b''))

def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pgm_tokens_and_payload(path):
    data = path.read_bytes()
    pos = 0
    tokens = []
    size = len(data)
    while len(tokens) < 4:
        while pos < size:
            if data[pos] in b' \t\r\n':
                pos += 1
                continue
            if data[pos] == 35:  # '#': PGM comment
                nl = data.find(b'\n', pos)
                pos = size if nl < 0 else nl + 1
                continue
            break
        if pos >= size:
            raise ValueError('Header PGM tidak lengkap')
        start = pos
        while pos < size and data[pos] not in b' \t\r\n#':
            pos += 1
        tokens.append(data[start:pos].decode('ascii'))
    while pos < size and data[pos] in b' \t\r\n':
        pos += 1
    return tokens, data[pos:]


def pgm_preview_png(path):
    tokens, payload = _pgm_tokens_and_payload(path)
    magic, width_s, height_s, maxval_s = tokens
    width, height, maxval = int(width_s), int(height_s), int(maxval_s)
    if width <= 0 or height <= 0 or maxval <= 0 or maxval > 255:
        raise ValueError('PGM dimensions/maxval tidak didukung')
    expected = width * height
    if magic == 'P5':
        if len(payload) < expected:
            raise ValueError('Payload PGM P5 terlalu pendek')
        pixels = payload[:expected]
    elif magic == 'P2':
        values = [int(v) for v in re.findall(rb'\d+', payload)]
        if len(values) < expected:
            raise ValueError('Payload PGM P2 terlalu pendek')
        pixels = bytes(max(0, min(255, round(v * 255 / maxval))) for v in values[:expected])
    else:
        raise ValueError(f'Format PGM tidak didukung: {magic}')
    if maxval != 255 and magic == 'P5':
        pixels = bytes(round(v * 255 / maxval) for v in pixels)
    rows = [b'\x00' + pixels[y * width:(y + 1) * width] for y in range(height)]
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    raw = b''.join(rows)
    png = (b'\x89PNG\r\n\x1a\n' + _png_chunk(b'IHDR', ihdr) +
           _png_chunk(b'IDAT', zlib.compress(raw, 6)) + _png_chunk(b'IEND', b''))
    return png, width, height


def read_map_yaml_metadata(path):
    meta = {'resolution': None, 'origin_x': None, 'origin_y': None, 'origin_yaw': None}
    if not path.is_file():
        return meta
    text = path.read_text(encoding='utf-8', errors='replace')
    m = re.search(r'^\s*resolution\s*:\s*([-+0-9.eE]+)', text, re.MULTILINE)
    if m:
        meta['resolution'] = float(m.group(1))
    m = re.search(r'^\s*origin\s*:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)', text, re.MULTILINE)
    if m:
        meta['origin_x'], meta['origin_y'], meta['origin_yaw'] = map(float, m.groups())
    return meta


class MappingWebController(Node):
    def __init__(self):
        super().__init__('mapping_web_controller')
        self._lock = threading.Lock()
        self._latest_map = None
        self._last_map_monotonic = 0.0
        self._last_map_heavy_monotonic = 0.0
        # BAB 4.2 live GUI grid.  This is an in-memory RLE transport of
        # /mapping/map; PNG remains reserved for saved/final map export.
        self._map_grid_rle = ''
        self._map_grid_seq = 0
        self._proc = None
        self._log_handle = None
        self._active_slot = 0
        self._phase = 'READY'
        self._last_error = ''
        self._saved_path = ''
        self._saved_yaml_path = ''
        self._saved_slot = 0
        self._saved_map_signature = None
        self._mapping_odom_status = {}
        self._nav_map_switch_status = {}
        self._map_stats = {}
        self._structure_metrics = {'valid': False, 'status': 'WAITING_MAP'}
        self._structure_reference = None
        self._structure_reference_candidate = None
        self._structure_reference_hits = 0
        self._structure_last_eval = 0.0
        self._last_cpu_total = None
        self._last_cpu_idle = None
        self._host_stats = {}
        self._session_active = False
        self._pending_start_slot = 0
        self._nav_stop_requested_at = 0.0
        self._engine_ready = False
        self._engine_started_at = 0.0
        self._engine_config_mtime_ns = 0
        self._engine_restart_reason = ''
        self._engine_start_retry_count = 0
        self._prewarm_due = float('inf')
        self._shutting_down = False
        self.declare_parameter('shared_sensor_mode', False)
        self.declare_parameter('slam_params_file', '')
        self.declare_parameter('engine_start_timeout_sec', 75.0)
        self.declare_parameter('engine_start_max_retries', 1)
        self._shared_sensor_mode = bool(self.get_parameter('shared_sensor_mode').value)
        self._paused_managers = []
        self._deactivated_nav_nodes = []

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mapping_topic = '/mapping/map' if self._shared_sensor_mode else '/map'
        self.create_subscription(OccupancyGrid, self._mapping_topic, self._on_map, map_qos)
        self.create_subscription(String, '/mapping/odom_status', self._on_mapping_odom_status, 10)
        self.create_subscription(PoseStamped, '/mapping/pose', self._on_mapping_pose, 10)
        self.create_subscription(String, '/navigation/map_switch_status', self._on_nav_map_switch_status, 10)
        self._status_pub = self.create_publisher(String, '/mapping/web_status', 10)
        session_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._session_pub = self.create_publisher(Bool, '/mapping/session_enabled', session_qos)
        self._publish_session_enabled(False)
        WEB_STATIC_DIR.mkdir(parents=True, exist_ok=True)
        if self._shared_sensor_mode:
            LIVE_MAP_PNG.unlink(missing_ok=True)
        self._refresh_saved_map_catalog(force=True)

        for slot in (1, 2, 3):
            self.create_service(
                Trigger, f'/mapping/start_{slot}',
                lambda req, resp, s=slot: self._start_service(s, req, resp))
            self.create_service(
                Trigger, f'/mapping/delete_{slot}',
                lambda req, resp, s=slot: self._delete_service(s, req, resp))
        self.create_service(Trigger, '/mapping/stop_save', self._stop_save_service)
        self.create_service(Trigger, '/mapping/stop_without_save', self._stop_without_save_service)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(0.5, self._engine_housekeeping)
        self.create_timer(2.0, self._refresh_saved_map_catalog)
        self._remove_stale_pidfile()
        mode = 'AUTONOMOUS_SHARED_SENSORS' if self._shared_sensor_mode else 'STANDALONE_MAPPING_RUNTIME'
        self.get_logger().info(
            f'Headless mapping controller READY mode={mode}: web START/STOP+SAVE owns SLAM mapping')

    @staticmethod
    def _encode_map_grid_rle(data):
        # Compact browser transport: U=unknown, F=free, O=occupied.  Counts
        # are hexadecimal.  The browser reconstructs the OccupancyGrid canvas
        # directly; no live PNG is involved.
        runs = []
        last = None
        count = 0
        for raw in data:
            value = int(raw)
            code = 'U' if value < 0 else ('O' if value >= 65 else 'F')
            if code == last:
                count += 1
                continue
            if last is not None:
                runs.append(f'{last}{count:x}')
            last = code
            count = 1
        if last is not None:
            runs.append(f'{last}{count:x}')
        return ','.join(runs)

    def _on_map(self, msg):
        if msg.info.width == 0 or msg.info.height == 0 or not msg.data:
            return
        now = time.monotonic()
        previous = self._last_map_monotonic
        update_hz = (1.0 / (now - previous)) if previous and now > previous else 0.0

        # Always retain the freshest grid immediately for STOP+SAVE and freshness
        # checks. Heavy Python traversal/PNG generation is throttled separately so
        # it can never starve the 8 Hz LiDAR scan path.
        with self._lock:
            self._latest_map = msg
            self._last_map_monotonic = now

        if self._map_stats:
            self._map_stats['map_update_hz'] = update_hz
        if (now - self._last_map_heavy_monotonic) < 1.0:
            return
        self._last_map_heavy_monotonic = now

        total = len(msg.data)
        unknown = sum(1 for value in msg.data if int(value) < 0)
        occupied = sum(1 for value in msg.data if int(value) >= 65)
        self._map_stats = {
            'resolution_m': float(msg.info.resolution),
            'width_cells': int(msg.info.width),
            'height_cells': int(msg.info.height),
            'width_m': float(msg.info.width) * float(msg.info.resolution),
            'height_m': float(msg.info.height) * float(msg.info.resolution),
            'unknown_pct': 100.0 * unknown / total if total else 0.0,
            'occupied_pct': 100.0 * occupied / total if total else 0.0,
            'map_update_hz': update_hz,
        }
        # Refresh the browser grid at the same throttled rate as the heavy map
        # metrics.  It is sourced directly from the LiDAR-only OccupancyGrid.
        self._map_grid_rle = self._encode_map_grid_rle(msg.data)
        self._map_grid_seq += 1
        self._update_structure_metrics(msg, now)

    def _reset_structure_evaluator(self):
        self._structure_metrics = {'valid': False, 'status': 'WAITING_MAP'}
        self._structure_reference = None
        self._structure_reference_candidate = None
        self._structure_reference_hits = 0
        self._structure_last_eval = 0.0

    @staticmethod
    def _normalize_wall_angle(theta):
        while theta >= math.pi / 2.0:
            theta -= math.pi
        while theta < -math.pi / 2.0:
            theta += math.pi
        return theta

    def _structure_from_grid(self, msg, forced_theta=None, target_center=None, target_width=None):
        res = float(msg.info.resolution)
        w, h = int(msg.info.width), int(msg.info.height)
        if res <= 0.0 or w <= 0 or h <= 0:
            return {'valid': False, 'status': 'INVALID_GRID'}
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        pts = []
        max_pts = 12000
        occupied_idx = [i for i,v in enumerate(msg.data) if int(v) >= 65]
        if len(occupied_idx) < 80:
            return {'valid': False, 'status': 'WAITING_STRUCTURE', 'occupied_points': len(occupied_idx)}
        step = max(1, len(occupied_idx) // max_pts)
        for idx in occupied_idx[::step]:
            yy, xx = divmod(idx, w)
            pts.append((ox + (xx + 0.5) * res, oy + (yy + 0.5) * res))
        n = len(pts)
        mx = sum(x for x,_ in pts) / n
        my = sum(y for _,y in pts) / n
        if forced_theta is None:
            cxx = sum((x-mx)*(x-mx) for x,_ in pts) / n
            cyy = sum((y-my)*(y-my) for _,y in pts) / n
            cxy = sum((x-mx)*(y-my) for x,y in pts) / n
            theta = 0.5 * math.atan2(2.0*cxy, cxx-cyy)
            theta = self._normalize_wall_angle(theta)
        else:
            theta = self._normalize_wall_angle(float(forced_theta))
        ct, st = math.cos(theta), math.sin(theta)
        nx, ny = -st, ct
        bin_w = max(res * 1.5, 0.05)
        hist = {}
        samples = {}
        for x,y in pts:
            along = x*ct + y*st
            normal = x*nx + y*ny
            b = int(round(normal / bin_w))
            hist[b] = hist.get(b, 0) + 1
            q = samples.get(b)
            if q is None:
                samples[b] = [along, along]
            else:
                q[0] = min(q[0], along); q[1] = max(q[1], along)
        min_count = max(5, int(0.003 * n))
        active_bins = sorted(b for b,c in hist.items() if c >= min_count)
        bands = []
        cur = []
        for b in active_bins:
            if cur and b > cur[-1] + 2:
                bands.append(cur); cur=[]
            cur.append(b)
        if cur: bands.append(cur)
        wall_bands=[]
        for band in bands:
            count=sum(hist[b] for b in band)
            if count < min_count*2: continue
            center=sum((b*bin_w)*hist[b] for b in band)/count
            lo=min(samples[b][0] for b in band); hi=max(samples[b][1] for b in band)
            wall_bands.append({'center':center,'count':count,'along_min':lo,'along_max':hi,'span':max(0.0,hi-lo)})
        wall_bands.sort(key=lambda a:a['center'])
        double=None
        for i,a in enumerate(wall_bands):
            for b in wall_bands[i+1:]:
                sep=abs(b['center']-a['center'])
                if sep > 0.38: break
                if sep < max(0.10, 2.0*res): continue
                overlap=min(a['along_max'],b['along_max'])-max(a['along_min'],b['along_min'])
                if overlap >= 0.80:
                    score=overlap/(sep+0.02)
                    if double is None or score>double['score']:
                        double={'score':score,'offset_m':sep,'overlap_m':overlap}
        corridor=None
        for i,a in enumerate(wall_bands):
            for b in wall_bands[i+1:]:
                sep=abs(b['center']-a['center'])
                if sep < 1.00 or sep > 4.00: continue
                overlap=min(a['along_max'],b['along_max'])-max(a['along_min'],b['along_min'])
                if overlap < 0.80: continue
                center=0.5*(a['center']+b['center'])
                score=overlap*math.sqrt(max(1.0,a['count']*b['count']))
                if target_width is not None:
                    width_err=abs(sep-float(target_width))
                    center_err=abs(center-float(target_center)) if target_center is not None else 0.0
                    if width_err > max(0.45, 0.30*float(target_width)) or center_err > 1.00:
                        continue
                    score=score/(1.0+8.0*width_err+3.0*center_err)
                if corridor is None or score>corridor['score']:
                    corridor={'score':score,'center_m':center, 'width_m':sep, 'overlap_m':overlap}
        result={
            'valid': bool(corridor or double),
            'status': 'OK' if (corridor or double) else 'NO_PARALLEL_STRUCTURE',
            'orientation_deg': theta*180.0/math.pi,
            'occupied_points': len(occupied_idx),
            'wall_band_count': len(wall_bands),
            'double_wall_detected': bool(double),
            'double_wall_offset_m': round(double['offset_m'],4) if double else None,
            'double_wall_overlap_m': round(double['overlap_m'],3) if double else None,
            'corridor_center_m': round(corridor['center_m'],4) if corridor else None,
            'corridor_width_m': round(corridor['width_m'],4) if corridor else None,
            'corridor_overlap_m': round(corridor['overlap_m'],3) if corridor else None,
        }
        return result

    def _update_structure_metrics(self, msg, now=None, force=False):
        if not self._session_active:
            return
        now = time.monotonic() if now is None else now
        if not force and (now - self._structure_last_eval) < 1.5:
            return
        self._structure_last_eval = now
        current = self._structure_from_grid(msg)
        if self._structure_reference is None and current.get('corridor_center_m') is not None and current.get('corridor_overlap_m',0.0) >= 2.0 and current.get('occupied_points',0) >= 180:
            candidate = {
                'orientation_rad': math.radians(float(current['orientation_deg'])),
                'corridor_center_m': float(current['corridor_center_m']),
                'corridor_width_m': float(current['corridor_width_m']),
            }
            prev = self._structure_reference_candidate
            stable = False
            if prev is not None:
                da = abs(self._normalize_wall_angle(candidate['orientation_rad']-prev['orientation_rad']))
                stable = (da <= math.radians(6.0) and
                          abs(candidate['corridor_center_m']-prev['corridor_center_m']) <= 0.30 and
                          abs(candidate['corridor_width_m']-prev['corridor_width_m']) <= 0.35)
            if stable:
                self._structure_reference_hits += 1
            else:
                self._structure_reference_candidate = candidate
                self._structure_reference_hits = 1
            if self._structure_reference_hits >= 3:
                self._structure_reference = dict(self._structure_reference_candidate)
        reference = self._structure_reference
        if reference is not None:
            aligned = self._structure_from_grid(msg, reference['orientation_rad'], reference['corridor_center_m'], reference['corridor_width_m'])
            if aligned.get('corridor_center_m') is not None:
                shift = abs(float(aligned['corridor_center_m']) - reference['corridor_center_m'])
                current['corridor_shift_m'] = round(shift,4)
                current['corridor_width_initial_m'] = round(reference['corridor_width_m'],4)
                current['corridor_width_current_m'] = aligned.get('corridor_width_m')
                current['reference_valid'] = True
            else:
                current['corridor_shift_m'] = None
                current['reference_valid'] = True
        else:
            current['corridor_shift_m'] = None
            current['reference_valid'] = False
            current['reference_stability_hits'] = self._structure_reference_hits
        current['method'] = 'occupancy_grid_parallel_wall_v2'
        self._structure_metrics = current

    def _on_mapping_odom_status(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                # Merge gate diagnostics with the LiDAR-SLAM pose. The scan gate
                # intentionally has no odometry/IMU and must not erase map_x/y/yaw.
                self._mapping_odom_status.update(payload)
        except Exception:
            pass

    def _on_mapping_pose(self, msg):
        try:
            q = msg.pose.orientation
            yaw = yaw_from_quaternion(q)
            self._mapping_odom_status.update({
                'ready': True,
                'pose_ready': True,
                'odometry_used': False,
                'imu_used': False,
                'sensor_source': 'lidar_only',
                'translation_source': 'lidar_scan_matching',
                'rotation_source': 'lidar_scan_matching',
                'map_x': round(float(msg.pose.position.x), 5),
                'map_y': round(float(msg.pose.position.y), 5),
                'map_yaw': round(float(yaw), 6),
                'x': round(float(msg.pose.position.x), 5),
                'y': round(float(msg.pose.position.y), 5),
                'yaw': round(float(yaw), 6),
                'pose_source': 'mapping_hector_lidar_only',
            })
        except Exception:
            pass

    def _on_nav_map_switch_status(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self._nav_map_switch_status = payload
        except Exception:
            pass

    def _saved_map_signature_now(self):
        items = []
        for slot in (1, 2, 3):
            for suffix in ('.pgm', '.yaml'):
                path = MAP_DIR / f'map_{slot}{suffix}'
                try:
                    st = path.stat()
                    items.append((slot, suffix, st.st_mtime_ns, st.st_size))
                except OSError:
                    items.append((slot, suffix, None, None))
        try:
            pointer = LATEST_POINTER.read_text(encoding='utf-8').strip()
        except OSError:
            pointer = ''
        return tuple(items), pointer

    def _refresh_saved_map_catalog(self, force=False):
        signature = self._saved_map_signature_now()
        if not force and signature == self._saved_map_signature:
            return
        WEB_STATIC_DIR.mkdir(parents=True, exist_ok=True)
        pointer_text = signature[1]
        try:
            pointer_path = Path(pointer_text).expanduser().resolve() if pointer_text else None
        except Exception:
            pointer_path = None
        latest_slot = 0
        maps = []
        for slot in (1, 2, 3):
            pgm = MAP_DIR / f'map_{slot}.pgm'
            yaml_path = MAP_DIR / f'map_{slot}.yaml'
            preview = WEB_STATIC_DIR / f'saved_map_{slot}.png'
            structure_path = MAP_DIR / f'map_{slot}_structure.json'
            item = {
                'slot': slot, 'name': f'Map {slot}', 'available': False,
                'pgm': str(pgm), 'yaml': str(yaml_path),
                'structure_json': str(structure_path),
                'preview_url': f'/saved_map_{slot}.png',
            }
            try:
                if not pgm.is_file() or not yaml_path.is_file():
                    preview.unlink(missing_ok=True)
                    maps.append(item)
                    continue
                png, width, height = pgm_preview_png(pgm)
                for static_dir in GUI_STATIC_DIRS:
                    static_dir.mkdir(parents=True, exist_ok=True)
                    target = static_dir / f'saved_map_{slot}.png'
                    tmp = Path(str(target) + '.tmp')
                    tmp.write_bytes(png)
                    os.replace(tmp, target)
                meta = read_map_yaml_metadata(yaml_path)
                item.update(meta)
                if structure_path.is_file():
                    try:
                        structure_metrics = json.loads(structure_path.read_text(encoding='utf-8'))
                        if isinstance(structure_metrics, dict):
                            item['structure_metrics'] = structure_metrics
                    except Exception as exc:
                        item['structure_error'] = str(exc)
                # Persist map-content statistics so BAB IV result tables/graphs remain
                # meaningful after the mapping process is stopped. ROS map_server PGM:
                # 0=occupied, 254=free, 205=unknown (intermediate values kept unknown).
                raw_pgm = pgm.read_bytes()
                header_end = 0
                token_count = 0
                pos = 0
                # Parse P5 header robustly, including the CREATOR comment.
                while token_count < 4 and pos < len(raw_pgm):
                    while pos < len(raw_pgm) and chr(raw_pgm[pos]).isspace(): pos += 1
                    if pos < len(raw_pgm) and raw_pgm[pos] == 35:
                        pos = raw_pgm.find(b'\n', pos)
                        if pos < 0: break
                        pos += 1
                        continue
                    start = pos
                    while pos < len(raw_pgm) and not chr(raw_pgm[pos]).isspace(): pos += 1
                    if pos > start: token_count += 1
                while pos < len(raw_pgm) and chr(raw_pgm[pos]).isspace(): pos += 1
                pixels = raw_pgm[pos:pos + width * height]
                total_cells = max(1, len(pixels))
                occupied_cells = sum(1 for v in pixels if v <= 100)
                free_cells = sum(1 for v in pixels if v >= 250)
                unknown_cells = max(0, total_cells - occupied_cells - free_cells)
                item.update({
                    'available': True, 'width': width, 'height': height,
                    'pgm_size_bytes': pgm.stat().st_size,
                    'occupied_pct': 100.0 * occupied_cells / total_cells,
                    'free_pct': 100.0 * free_cells / total_cells,
                    'unknown_pct': 100.0 * unknown_cells / total_cells,
                    'modified_at_ms': int(max(pgm.stat().st_mtime, yaml_path.stat().st_mtime) * 1000),
                })
                try:
                    if pointer_path is not None and yaml_path.resolve() == pointer_path:
                        latest_slot = slot
                except Exception:
                    pass
            except Exception as exc:
                item['error'] = str(exc)
                preview.unlink(missing_ok=True)
                self.get_logger().warning(f'Preview saved Map {slot} gagal: {exc}')
            maps.append(item)
        payload = {
            'generated_at_ms': int(time.time() * 1000),
            'latest_pointer_slot': latest_slot,
            'latest_pointer': pointer_text,
            'maps': maps,
        }
        catalog_json = json.dumps(payload, separators=(',', ':'))
        for static_dir in GUI_STATIC_DIRS:
            static_dir.mkdir(parents=True, exist_ok=True)
            target = static_dir / 'saved_maps.json'
            tmp_catalog = Path(str(target) + '.tmp')
            tmp_catalog.write_text(catalog_json, encoding='utf-8')
            os.replace(tmp_catalog, target)
        self._saved_map_signature = signature

    def _delete_service(self, slot, _request, response):
        if slot not in (1, 2, 3):
            response.success = False
            response.message = 'Slot map harus 1..3'
            return response
        if self._runtime_alive() and self._active_slot == slot:
            response.success = False
            response.message = f'Map {slot} sedang dipakai untuk mapping; STOP mapping terlebih dahulu'
            return response
        nav_slot = int(self._nav_map_switch_status.get('active_slot') or 0)
        nav_enabled = self._nav_map_switch_status.get('nav_enabled') is True
        nav_busy = self._nav_map_switch_status.get('busy') is True
        if nav_slot == slot and (nav_enabled or nav_busy):
            response.success = False
            response.message = f'Map {slot} sedang dipakai Nav2 aktif; STOP Navigation atau pindahkan map terlebih dahulu'
            return response

        pgm = MAP_DIR / f'map_{slot}.pgm'
        yaml_path = MAP_DIR / f'map_{slot}.yaml'
        structure_path = MAP_DIR / f'map_{slot}_structure.json'
        preview = WEB_STATIC_DIR / f'saved_map_{slot}.png'
        existed = pgm.exists() or yaml_path.exists() or structure_path.exists()
        for path in (pgm, yaml_path, structure_path, preview):
            path.unlink(missing_ok=True)

        try:
            current_pointer = LATEST_POINTER.read_text(encoding='utf-8').strip() if LATEST_POINTER.exists() else ''
        except OSError:
            current_pointer = ''
        deleted_yaml = str(yaml_path.resolve())
        if current_pointer == deleted_yaml:
            remaining = []
            for other in (1, 2, 3):
                yp = MAP_DIR / f'map_{other}.yaml'
                pp = MAP_DIR / f'map_{other}.pgm'
                if yp.is_file() and pp.is_file():
                    try:
                        remaining.append((max(yp.stat().st_mtime, pp.stat().st_mtime), yp.resolve()))
                    except OSError:
                        pass
            POINTER_DIR.mkdir(parents=True, exist_ok=True)
            if remaining:
                remaining.sort(reverse=True, key=lambda item: item[0])
                tmp = Path(str(LATEST_POINTER) + '.tmp')
                tmp.write_text(str(remaining[0][1]) + '\n', encoding='utf-8')
                os.replace(tmp, LATEST_POINTER)
            else:
                LATEST_POINTER.unlink(missing_ok=True)

        if self._saved_slot == slot:
            self._saved_slot = 0
            self._saved_path = ''
            self._saved_yaml_path = ''
        self._refresh_saved_map_catalog(force=True)
        self._publish_status()
        response.success = True
        response.message = (f'Map {slot} dihapus: PGM + YAML + data struktur' if existed
                            else f'Map {slot} sudah kosong')
        self.get_logger().info(response.message)
        return response

    def _remove_stale_pidfile(self):
        try:
            if not PID_FILE.exists():
                return
            pid = int(PID_FILE.read_text().strip())
            os.kill(pid, 0)
            self.get_logger().warning(
                f'PID mapping lama {pid} masih hidup; controller tidak mengambil alih paksa')
        except (ValueError, ProcessLookupError, PermissionError, OSError):
            try:
                PID_FILE.unlink(missing_ok=True)
            except Exception:
                pass

    def _service_set(self):
        # Use the controller's own ROS graph instead of spawning `ros2 service list`.
        # The CLI subprocess can transiently time out / use a stale daemon even while
        # the lifecycle services are already present, which falsely rejected START.
        try:
            return {name for name, _types in self.get_service_names_and_types()}
        except Exception as exc:
            self.get_logger().warning(f'ROS service graph read gagal: {exc}')
            return set()

    def _manager_command(self, manager, command, timeout=12.0):
        service = f'/{manager}/manage_nodes'
        cmd = [
            'ros2', 'service', 'call', service,
            'nav2_msgs/srv/ManageLifecycleNodes', f'{{command: {int(command)}}}']
        try:
            result = subprocess.run(
                cmd, cwd=str(WORKSPACE), env=os.environ.copy(),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=float(timeout))
            out = (result.stdout or '').strip()
            ok = result.returncode == 0 and ('success=true' in out.lower() or 'success: true' in out.lower())
            return ok, out[-500:]
        except Exception as exc:
            return False, str(exc)

    def _manager_is_active(self, manager):
        service = f'/{manager}/is_active'
        cmd = ['ros2', 'service', 'call', service, 'std_srvs/srv/Trigger', '{}']
        try:
            result = subprocess.run(
                cmd, cwd=str(WORKSPACE), env=os.environ.copy(),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=4.0)
            out = (result.stdout or '').lower()
            return result.returncode == 0 and ('success=true' in out or 'success: true' in out)
        except Exception:
            return False

    def _lifecycle_state(self, node_name):
        cmd = ['ros2', 'lifecycle', 'get', f'/{node_name}']
        try:
            result = subprocess.run(
                cmd, cwd=str(WORKSPACE), env=os.environ.copy(),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=5.0)
            out = (result.stdout or '').strip().lower()
            for state in ('active', 'inactive', 'unconfigured', 'finalized'):
                if out.startswith(state) or f'{state} [' in out:
                    return state
            return 'missing' if 'not found' in out or 'service' in out and 'available' in out else 'unknown'
        except Exception:
            return 'unknown'

    def _lifecycle_set(self, node_name, transition):
        cmd = ['ros2', 'lifecycle', 'set', f'/{node_name}', transition]
        try:
            result = subprocess.run(
                cmd, cwd=str(WORKSPACE), env=os.environ.copy(),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10.0)
            out = (result.stdout or '').strip()
            low = out.lower()
            ok = result.returncode == 0 and ('successful' in low or 'success' in low)
            return ok, out[-500:]
        except Exception as exc:
            return False, str(exc)

    def _pause_navigation(self):
        """Temporarily suspend CPU-heavy non-mapping processes for BAB 4.2.

        SIGSTOP/SIGCONT preserves each process exactly as-is and avoids Nav2
        lifecycle/respawn churn.  Mapping-critical LiDAR, IMU, scan filters,
        mapping controller/bridge, web GUI, actuator and safety guard remain live.
        """
        patterns = (
            '/nav2_amcl/amcl',
            '/nav2_map_server/map_server',
            '/nav2_planner/planner_server',
            '/nav2_controller/controller_server',
            '/nav2_behaviors/behavior_server',
            '/nav2_bt_navigator/bt_navigator',
            '/nav2_velocity_smoother/velocity_smoother',
            '__node:=hector_slam_node',
            '/robot_localization/ekf_node',
            'localization_timing_monitor.py',
            'nav_map_switch_controller.py',
            'autonomy_health_manager.py',
            'astra_rgb_v4l2_node',
            'hole_block_alignment_node.py',
            'imu_visual_tf_node',
        )
        suspended = []
        own_pid = os.getpid()
        for proc_dir in Path('/proc').iterdir():
            if not proc_dir.name.isdigit():
                continue
            pid = int(proc_dir.name)
            if pid == own_pid:
                continue
            try:
                cmd = (proc_dir / 'cmdline').read_bytes().replace(b'\0', b' ').decode('utf-8', 'ignore')
            except OSError:
                continue
            if not any(pattern in cmd for pattern in patterns):
                continue
            try:
                os.kill(pid, signal.SIGSTOP)
                suspended.append(pid)
            except (ProcessLookupError, PermissionError):
                continue
        self._suspended_pids = suspended
        self._paused_managers = []
        self._deactivated_nav_nodes = []
        self.get_logger().info(
            f'BAB 4.2 MAPPING-ONLY READY; suspended {len(suspended)} non-mapping processes')

    def _enforce_mapping_pause(self):
        if not self._shared_sensor_mode or not self._runtime_alive():
            return
        # During a live BAB 4.2 session, navigation must remain inactive. If an
        # external watchdog or late lifecycle transition reactivates a node,
        # force only that navigation node back to INACTIVE without touching
        # LiDAR/IMU/Hector/mapping TF.
        for node_name in NAVIGATION_LIFECYCLE_NODES:
            if self._lifecycle_state(node_name) != 'active':
                continue
            ok, detail = self._lifecycle_set(node_name, 'deactivate')
            if not ok or self._lifecycle_state(node_name) == 'active':
                self._last_error = f'mapping-only guard gagal deactivate {node_name}: {detail}'
                self._phase = 'ERROR'
                self.get_logger().error(self._last_error)
                self._stop_process_group()
                self._resume_navigation()
                self._active_slot = 0
                return
            if node_name not in self._deactivated_nav_nodes:
                self._deactivated_nav_nodes.append(node_name)

    def _resume_navigation(self):
        errors = []
        for pid in getattr(self, '_suspended_pids', []):
            try:
                os.kill(int(pid), signal.SIGCONT)
            except (ProcessLookupError, PermissionError):
                pass
        self._suspended_pids = []
        # Nodes deactivated directly are restored in reverse order so map/localization
        # return before planner/controller/BT. Managers paused cleanly are resumed
        # afterwards in reverse group order.
        for node_name in reversed(self._deactivated_nav_nodes):
            state = self._lifecycle_state(node_name)
            if state != 'inactive':
                continue
            ok, detail = self._lifecycle_set(node_name, 'activate')
            if not ok:
                errors.append(f'{node_name}: {detail}')
        self._deactivated_nav_nodes = []

        services = self._service_set()
        for manager in reversed(self._paused_managers):
            if f'/{manager}/manage_nodes' not in services:
                continue
            ok, detail = self._manager_command(manager, 2, timeout=15.0)
            if not ok:
                errors.append(f'{manager}: {detail}')
        self._paused_managers = []
        if errors:
            self.get_logger().error('Navigation lifecycle resume partial: ' + ' | '.join(errors))
        else:
            self.get_logger().info('BAB 4.2 selesai; navigation lifecycle RESUMED')
        return errors

    def _publish_session_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self._session_pub.publish(msg)

    def _reset_mapping_pose_status(self, session_enabled=False):
        # Never expose a pose from a completed/failed mapping session as live.
        # The LiDAR-only gate/SLAM publishers repopulate these fields once the
        # next engine is READY and a fresh /mapping/pose is received.
        self._mapping_odom_status = {
            'ready': False,
            'pose_ready': False,
            'session_enabled': bool(session_enabled),
            'odometry_used': False,
            'imu_used': False,
            'sensor_source': 'lidar_only',
            'translation_source': 'lidar_scan_matching',
            'rotation_source': 'lidar_scan_matching',
            'pose_source': 'none',
        }

    def _slam_params_path(self):
        # Kept under the historical method name for GUI/controller compatibility.
        # BAB 4.2 mapping parameters now come from the LiDAR-only Hector config.
        candidates = [
            WORKSPACE / 'config/runtime/navigation/hector_autonomous.yaml',
            WORKSPACE / 'src/navigation/config/hector_autonomous.yaml',
        ]
        params = next((path for path in candidates if path and path.is_file()), None)
        if params is None:
            raise RuntimeError('hector_autonomous.yaml tidak ditemukan')
        return params

    def _slam_params_mtime_ns(self):
        try:
            return self._slam_params_path().stat().st_mtime_ns
        except OSError:
            return 0

    def _cleanup_orphan_slam_nodes(self):
        """Kill only orphaned mapping slam_toolbox children (PPID=1)."""
        for proc_dir in Path('/proc').iterdir():
            if not proc_dir.name.isdigit():
                continue
            try:
                pid = int(proc_dir.name)
                status = (proc_dir / 'status').read_text(errors='ignore')
                ppid_line = next((x for x in status.splitlines() if x.startswith('PPid:')), '')
                ppid = int(ppid_line.split()[1]) if ppid_line else -1
                if ppid != 1:
                    continue
                cmd = (proc_dir / 'cmdline').read_bytes().replace(b'\0', b' ').decode('utf-8', 'ignore')
            except (OSError, ValueError, StopIteration):
                continue
            if 'async_slam_toolbox_node' not in cmd or 'map:=/mapping/map' not in cmd:
                continue
            self.get_logger().warning(f'Cleaning orphan mapping slam_toolbox PID {pid}')
            try:
                os.kill(pid, signal.SIGTERM)
                time.sleep(0.15)
                os.kill(pid, 0)
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    def _cleanup_stale_engine_pidfile(self):
        """Remove an orphan mapping engine left by a previous controller process."""
        self._cleanup_orphan_slam_nodes()
        if not PID_FILE.exists():
            return
        try:
            pid = int(PID_FILE.read_text(encoding='utf-8').strip())
        except (OSError, ValueError):
            PID_FILE.unlink(missing_ok=True)
            return
        if self._proc is not None and self._proc.poll() is None and pid == self._proc.pid:
            return
        try:
            os.kill(pid, 0)
        except (ProcessLookupError, PermissionError):
            PID_FILE.unlink(missing_ok=True)
            return
        try:
            cmdline = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except OSError:
            cmdline = ''
        if 'mapping_shared_slam.launch.py' not in cmdline:
            self.get_logger().warning(f'Stale mapping PID file points to unrelated PID {pid}; removing PID file only')
            PID_FILE.unlink(missing_ok=True)
            return
        self.get_logger().warning(f'Cleaning orphan mapping engine PID {pid} before prewarm')
        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGINT)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.10)
            else:
                os.killpg(pgid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        PID_FILE.unlink(missing_ok=True)

    def _launch_engine(self, reason='prewarm'):
        if not self._shared_sensor_mode or self._shutting_down:
            return False
        if self._proc is not None and self._proc.poll() is None:
            return True
        self._cleanup_stale_engine_pidfile()
        MAP_DIR.mkdir(parents=True, exist_ok=True)
        (WORKSPACE / 'log').mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = WORKSPACE / 'log' / f'mapping_engine_{stamp}.txt'
        self._log_handle = open(log_path, 'ab', buffering=0)
        try:
            cmd = self._shared_slam_command()
            self._proc = subprocess.Popen(
                cmd, cwd=str(WORKSPACE), stdout=self._log_handle,
                stderr=subprocess.STDOUT, preexec_fn=os.setsid, env=os.environ.copy())
            PID_FILE.write_text(f'{self._proc.pid}\n')
            self._engine_ready = False
            self._engine_started_at = time.monotonic()
            self._engine_config_mtime_ns = self._slam_params_mtime_ns()
            self._engine_restart_reason = reason
            if not self._session_active and not self._pending_start_slot:
                self._phase = 'ENGINE_PREPARING'
            self.get_logger().info(
                f'Mapping engine PREWARM PID {self._proc.pid}; reason={reason}; log={log_path}')
            return True
        except Exception:
            if self._log_handle:
                self._log_handle.close()
            self._log_handle = None
            self._proc = None
            raise

    def _engine_services_ready(self):
        # BAB 4.2 now uses the dedicated LiDAR-only Hector engine, not
        # slam_toolbox + odometry bridge.  Engine readiness therefore depends on
        # the mapping Hector node and the LiDAR-only scan gate being alive.
        try:
            nodes = set()
            for name, namespace in self.get_node_names_and_namespaces():
                namespace = namespace.rstrip('/')
                full = f'{namespace}/{name}' if namespace else f'/{name}'
                nodes.add(full.replace('//', '/'))
        except Exception:
            return False
        hector_ready = '/mapping_hector_slam_node' in nodes
        gate_ready = bool(self._mapping_odom_status.get('ready', False))
        return hector_ready and gate_ready

    def _begin_shared_session(self, slot):
        if slot not in (1, 2, 3) or not self._engine_ready:
            return False
        self._pending_start_slot = 0
        self._engine_start_retry_count = 0
        try:
            self._pause_navigation()
        except Exception as exc:
            self._active_slot = 0
            self._session_active = False
            self._phase = 'ERROR'
            self._last_error = f'mapping-only isolation gagal: {exc}'
            self._publish_session_enabled(False)
            self.get_logger().error(self._last_error)
            return False
        self._active_slot = slot
        self._session_active = True
        self._phase = 'STARTING'
        self._last_error = ''
        self._saved_path = ''
        self._saved_yaml_path = ''
        self._saved_slot = 0
        self._reset_structure_evaluator()
        with self._lock:
            self._latest_map = None
            self._last_map_monotonic = 0.0
        LIVE_MAP_PNG.unlink(missing_ok=True)
        self._publish_session_enabled(True)
        self.get_logger().info(
            f'START MAP {slot}: standby engine already READY; scan gate ENABLED')
        return True

    def _engine_housekeeping(self):
        if not self._shared_sensor_mode or self._shutting_down:
            return
        if self._proc is not None and self._proc.poll() is not None:
            rc = self._proc.returncode
            was_session = self._session_active
            PID_FILE.unlink(missing_ok=True)
            if self._log_handle:
                try:
                    self._log_handle.close()
                except Exception:
                    pass
            self._log_handle = None
            self._proc = None
            self._engine_ready = False
            self._session_active = False
            self._publish_session_enabled(False)
            if was_session:
                self._last_error = f'mapping engine berhenti sendiri rc={rc}'
                resume_errors = self._resume_navigation() if (getattr(self, '_suspended_pids', []) or self._paused_managers or self._deactivated_nav_nodes) else []
                if resume_errors:
                    self._last_error += ' | resume: ' + ' | '.join(resume_errors)
                self._phase = 'ERROR'
                self._active_slot = 0
            self._prewarm_due = time.monotonic() + 1.0

        if self._proc is None:
            if self._pending_start_slot:
                nav = dict(self._nav_map_switch_status)
                nav_phase = str(nav.get('phase', '')).upper()
                nav_enabled = nav.get('nav_enabled') is True
                nav_busy = nav.get('busy') is True
                elapsed = (time.monotonic() - self._nav_stop_requested_at
                           if self._nav_stop_requested_at > 0.0 else 0.0)
                nav_off = (not nav_enabled and not nav_busy and nav_phase == 'NAV2_OFF')

                # BAB 4.2 owns the handover after a short STOP grace period.
                # Engine PREWARM is safe while the scan gate/session is still OFF.
                # When the engine becomes READY, _begin_shared_session() calls
                # _pause_navigation(), SIGSTOPing Nav2 before session_enabled=True.
                # This prevents bt_navigator lifecycle latency from blocking SLAM.
                nav_handover = (
                    elapsed >= 3.0 and
                    nav_phase in ('STOPPING_NAV2', 'ERROR', 'NAV2_OFF')
                )
                if not (nav_off or nav_handover):
                    if elapsed >= 12.0:
                        slot = int(self._pending_start_slot)
                        self._pending_start_slot = 0
                        self._phase = 'ERROR'
                        self._last_error = (
                            f'MAP {slot} dibatalkan: handover Nav2 tidak aman setelah {elapsed:.1f}s; '
                            f'phase={nav.get("phase", "UNKNOWN")} nav_enabled={nav.get("nav_enabled")} '
                            f'busy={nav.get("busy")}')
                        self.get_logger().error(self._last_error)
                        self._publish_session_enabled(False)
                        self._reset_mapping_pose_status(False)
                    return
                if nav_handover and not nav_off:
                    self.get_logger().warning(
                        f'BAB 4.2 handover: Nav2 phase={nav_phase} busy={nav_busy} '
                        f'nav_enabled={nav_enabled}; SLAM prewarm dimulai, scan gate tetap OFF sampai '
                        f'proses Nav2 di-SIGSTOP')
                try:
                    self._phase = 'PREPARING'
                    self._engine_restart_reason = 'nav2-off-start'
                    self._launch_engine('nav2-off-start')
                    self.get_logger().info(
                        f'MAP {self._pending_start_slot}: NAV2 OFF confirmed; starting SLAM engine')
                except Exception as exc:
                    self._last_error = f'mapping engine start gagal: {exc}'
                    self._phase = 'ERROR'
                    self._pending_start_slot = 0
                return
            return

        self._engine_ready = self._engine_services_ready()
        if self._engine_ready and not self._session_active and not self._pending_start_slot:
            self._phase = 'READY'

        if self._pending_start_slot and not self._engine_ready and self._engine_started_at > 0.0:
            timeout = max(5.0, float(self.get_parameter('engine_start_timeout_sec').value))
            max_retries = max(0, int(self.get_parameter('engine_start_max_retries').value))
            elapsed = time.monotonic() - self._engine_started_at
            if elapsed >= timeout:
                slot = int(self._pending_start_slot)
                if self._engine_start_retry_count < max_retries:
                    self._engine_start_retry_count += 1
                    self.get_logger().warning(
                        f'MAP {slot} engine startup timeout {elapsed:.1f}s; '
                        f'retry {self._engine_start_retry_count}/{max_retries}')
                    self._stop_process_group()
                    self._cleanup_orphan_slam_nodes()
                    self._prewarm_due = time.monotonic() + 0.5
                    self._engine_restart_reason = 'startup-timeout-retry'
                    self._phase = 'PREPARING'
                    return
                self._last_error = (
                    f'MAP {slot} gagal start: Hector LiDAR-only tidak READY dalam {timeout:.0f}s '
                    f'setelah {max_retries + 1} percobaan')
                self.get_logger().error(self._last_error)
                self._stop_process_group()
                self._cleanup_orphan_slam_nodes()
                self._pending_start_slot = 0
                self._active_slot = 0
                self._session_active = False
                resume_errors = self._resume_navigation() if (getattr(self, '_suspended_pids', []) or self._paused_managers or self._deactivated_nav_nodes) else []
                if resume_errors:
                    self._last_error += ' | resume: ' + ' | '.join(resume_errors)
                self._phase = 'ERROR'
                self._publish_session_enabled(False)
                return

        current_mtime = self._slam_params_mtime_ns()
        if (not self._session_active and self._engine_ready and
                current_mtime and self._engine_config_mtime_ns and current_mtime != self._engine_config_mtime_ns):
            self.get_logger().info('slam_toolbox.yaml berubah saat idle; standby engine direload otomatis')
            self._stop_process_group()
            self._prewarm_due = time.monotonic() + 0.25
            self._engine_restart_reason = 'yaml-reload'
            return

        if self._pending_start_slot and self._engine_ready and not self._session_active:
            self._begin_shared_session(self._pending_start_slot)

    def _shared_slam_command(self):
        params = self._slam_params_path()
        use_sim_time = bool(self.get_parameter('use_sim_time').value)
        return [
            'ros2', 'launch', 'navigation', 'mapping_shared_slam.launch.py',
            f'slam_params_file:={params}',
            f'use_sim_time:={"true" if use_sim_time else "false"}',
            'transform_publish_period:=0.05',
        ]

    def _runtime_alive(self):
        if self._shared_sensor_mode:
            return bool(self._session_active and self._proc is not None and self._proc.poll() is None)
        return self._proc is not None and self._proc.poll() is None

    def _request_navigation_off(self):
        """Request NAV2_OFF asynchronously via the dedicated Nav2 controller."""
        cmd = [
            'ros2', 'service', 'call', '/navigation/map/stop',
            'std_srvs/srv/Trigger', '{}']
        subprocess.Popen(
            cmd, cwd=str(WORKSPACE), env=os.environ.copy(),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self._nav_stop_requested_at = time.monotonic()

    def _start_service(self, slot, _request, response):
        if self._runtime_alive() or self._pending_start_slot:
            response.success = False
            current = self._active_slot or self._pending_start_slot
            response.message = f'Mapping MAP {current} masih aktif/dipersiapkan'
            return response
        if slot not in (1, 2, 3):
            response.success = False
            response.message = 'Slot map harus 1..3'
            return response
        if self._shared_sensor_mode:
            # BAB 4.2 mapping-only: advertise ownership first, then request
            # NAV2_OFF. The callback returns immediately; housekeeping starts
            # SLAM only after /navigation/map_switch_status confirms NAV2_OFF.
            # LiDAR, IMU, Hector /lidar/odom, base TF and web GUI stay alive.
            self._pending_start_slot = slot
            self._engine_start_retry_count = 0
            self._phase = 'WAITING_NAV2_OFF'
            self._last_error = ''
            self._reset_mapping_pose_status(False)
            with self._lock:
                self._latest_map = None
                self._last_map_monotonic = 0.0
            self._map_stats = {}
            self._reset_structure_evaluator()
            LIVE_MAP_PNG.unlink(missing_ok=True)
            self._publish_session_enabled(False)
            self._publish_status()
            nav = dict(self._nav_map_switch_status)
            already_off = (nav.get('nav_enabled') is False and
                           nav.get('busy') is False and
                           str(nav.get('phase', '')) == 'NAV2_OFF')
            if already_off:
                self._nav_stop_requested_at = time.monotonic()
                response.success = True
                response.message = f'START MAP {slot}: Nav2 sudah OFF; menyiapkan SLAM'
                return response
            try:
                self._request_navigation_off()
            except Exception as exc:
                self._pending_start_slot = 0
                self._phase = 'ERROR'
                self._last_error = str(exc)
                response.success = False
                response.message = f'START MAP {slot} gagal meminta Nav2 OFF: {exc}'
                self.get_logger().error(response.message)
                self._publish_status()
                return response
            response.success = True
            response.message = f'START MAP {slot}: request diterima; menunggu NAV2 OFF sebelum SLAM'
            return response

        # Publish PREPARING immediately so localhost never looks idle after START.
        # In autonomous shared-sensor mode SLAM is isolated on /mapping/map and
        # publishes no map->odom TF, so the working map_server/AMCL/Nav2 remain untouched.
        self._active_slot = slot
        self._phase = 'PREPARING'
        self._last_error = ''
        self._saved_path = ''
        self._saved_yaml_path = ''
        self._saved_slot = 0
        self._reset_structure_evaluator()
        with self._lock:
            self._latest_map = None
            self._last_map_monotonic = 0.0
        self._publish_status()
        try:
            MAP_DIR.mkdir(parents=True, exist_ok=True)
            (WORKSPACE / 'log').mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_path = WORKSPACE / 'log' / f'mapping_web_{stamp}.txt'
            self._log_handle = open(log_path, 'ab', buffering=0)
            if self._shared_sensor_mode:
                LIVE_MAP_PNG.unlink(missing_ok=True)
                cmd = self._shared_slam_command()
                runtime_label = 'mapping_shared_slam(lidar odom synced; isolated /mapping/map)'
            else:
                cmd = ['ros2', 'launch', 'navigation', 'mapping_runtime.launch.py', 'enable_rviz:=false']
                runtime_label = 'mapping_runtime'
            self._proc = subprocess.Popen(
                cmd, cwd=str(WORKSPACE), stdout=self._log_handle,
                stderr=subprocess.STDOUT, preexec_fn=os.setsid, env=os.environ.copy())
            self._phase = 'STARTING'
            PID_FILE.write_text(f'{self._proc.pid}\n')
            self._publish_status()
            response.success = True
            response.message = f'START MAP {slot}: {runtime_label} PID {self._proc.pid}; log={log_path}'
            self.get_logger().info(response.message)
        except Exception as exc:
            if self._shared_sensor_mode and (getattr(self, '_suspended_pids', []) or self._paused_managers or self._deactivated_nav_nodes):
                self._resume_navigation()
            if self._log_handle:
                try:
                    self._log_handle.close()
                except Exception:
                    pass
                self._log_handle = None
            self._phase = 'ERROR'
            self._last_error = str(exc)
            self._active_slot = 0
            self._proc = None
            response.success = False
            response.message = f'START mapping gagal: {exc}'
            self.get_logger().error(response.message)
            self._publish_status()
        return response

    def _snapshot_map(self):
        with self._lock:
            msg = self._latest_map
            age = time.monotonic() - self._last_map_monotonic if self._last_map_monotonic else 1e9
        if msg is None:
            raise RuntimeError('Belum ada pesan /map yang bisa disimpan')
        expected = int(msg.info.width) * int(msg.info.height)
        if expected <= 0 or len(msg.data) != expected:
            raise RuntimeError('Pesan /map memiliki ukuran/data tidak valid')
        if age > 3.0:
            raise RuntimeError(f'/map tidak fresh ({age:.1f}s); mapping tetap aktif')
        return msg

    @staticmethod
    def _validate_navigation_map_grid(msg):
        width, height = int(msg.info.width), int(msg.info.height)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        # BAB 4.2.1-4.2.3 and Navigation use one fixed coordinate grid so
        # mapping -> map_server does not change dimensions or origin.
        if width != 800 or height != 800:
            raise RuntimeError(f'Grid mapping harus 800x800 untuk Navigation, aktual {width}x{height}')
        if abs(res - 0.05) > 1e-6:
            raise RuntimeError(f'Resolusi mapping harus 0.05 m/cell, aktual {res:.6f}')
        if abs(ox + 20.0) > 1e-4 or abs(oy + 20.0) > 1e-4:
            raise RuntimeError(f'Origin mapping harus [-20,-20], aktual [{ox:.6f},{oy:.6f}]')

    def _write_map(self, slot):
        msg = self._snapshot_map()
        self._validate_navigation_map_grid(msg)
        MAP_DIR.mkdir(parents=True, exist_ok=True)
        prefix = MAP_DIR / f'map_{slot}'
        pgm_final = prefix.with_suffix('.pgm')
        yaml_final = prefix.with_suffix('.yaml')
        pgm_tmp = Path(str(pgm_final) + '.tmp')
        yaml_tmp = Path(str(yaml_final) + '.tmp')
        for p in (pgm_tmp, yaml_tmp):
            p.unlink(missing_ok=True)

        width, height = int(msg.info.width), int(msg.info.height)
        with open(pgm_tmp, 'wb') as f:
            f.write(f'P5\n# CREATOR: navigation mapping_web_controller\n{width} {height}\n255\n'.encode())
            data = msg.data
            for y in range(height - 1, -1, -1):
                row = bytearray(width)
                base = y * width
                for x in range(width):
                    occ = int(data[base + x])
                    pixel = 205
                    if occ >= 65:
                        pixel = 0
                    elif 0 <= occ <= 25:
                        pixel = 254
                    row[x] = pixel
                f.write(row)
            f.flush()
            os.fsync(f.fileno())

        origin = msg.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        yaml_text = (
            f'image: map_{slot}.pgm\n'
            'mode: trinary\n'
            f'resolution: {msg.info.resolution:.6f}\n'
            f'origin: [{origin.position.x:.6f}, {origin.position.y:.6f}, {yaw:.6f}]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        with open(yaml_tmp, 'w', encoding='utf-8') as f:
            f.write(yaml_text)
            f.flush()
            os.fsync(f.fileno())

        os.replace(pgm_tmp, pgm_final)
        os.replace(yaml_tmp, yaml_final)

        # Saved PNG is generated from the SAME latest OccupancyGrid snapshot as
        # PGM/YAML. It is export-only; live BAB 4.2 canvas uses map_grid RLE.
        saved_png_bytes = occupancy_grid_png(msg)
        for static_dir in GUI_STATIC_DIRS:
            try:
                static_dir.mkdir(parents=True, exist_ok=True)
                preview = static_dir / f'saved_map_{slot}.png'
                preview_tmp = Path(str(preview) + '.tmp')
                preview_tmp.write_bytes(saved_png_bytes)
                os.replace(preview_tmp, preview)
            except Exception as exc:
                self.get_logger().warning(f'PNG final Map {slot} gagal dipublikasikan ke {static_dir}: {exc}')

        structure_final = MAP_DIR / f'map_{slot}_structure.json'
        structure_tmp = Path(str(structure_final) + '.tmp')
        structure_tmp.write_text(json.dumps(dict(self._structure_metrics), separators=(',', ':')) + '\n', encoding='utf-8')
        os.replace(structure_tmp, structure_final)
        POINTER_DIR.mkdir(parents=True, exist_ok=True)
        pointer_tmp = Path(str(LATEST_POINTER) + '.tmp')
        pointer_tmp.write_text(str(yaml_final.resolve()) + '\n', encoding='utf-8')
        os.replace(pointer_tmp, LATEST_POINTER)
        # PGM is the primary BAB 4.2 mapping result. YAML is retained only
        # as ROS map metadata so the same PGM can be loaded by map_server later.
        self._saved_path = str(pgm_final)
        self._saved_yaml_path = str(yaml_final)
        self._saved_slot = slot
        self._refresh_saved_map_catalog(force=True)
        return pgm_final, yaml_final

    def _stop_process_group(self):
        if self._shared_sensor_mode:
            self._publish_session_enabled(False)
            self._engine_ready = False
        proc = self._proc
        if proc is None:
            return
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    try:
                        proc.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        pass
        PID_FILE.unlink(missing_ok=True)
        if self._log_handle:
            try:
                self._log_handle.close()
            except Exception:
                pass
        self._log_handle = None
        self._proc = None
        if self._shared_sensor_mode:
            LIVE_MAP_PNG.unlink(missing_ok=True)
            self._reset_mapping_pose_status(False)

    def _stop_save_service(self, _request, response):
        if not self._runtime_alive() or self._active_slot not in (1, 2, 3):
            response.success = False
            response.message = 'Tidak ada mapping aktif untuk STOP + SAVE'
            return response
        slot = self._active_slot
        self._phase = 'SAVING'
        try:
            with self._lock:
                structure_map = self._latest_map
            if structure_map is not None:
                self._update_structure_metrics(structure_map, force=True)
            pgm_path, yaml_path = self._write_map(slot)
        except Exception as exc:
            self._phase = 'RUNNING'
            self._last_error = str(exc)
            response.success = False
            response.message = f'SAVE MAP {slot} gagal: {exc}. Mapping tetap aktif.'
            self.get_logger().error(response.message)
            return response

        self._phase = 'STOPPING'
        self._session_active = False
        self._publish_session_enabled(False)
        self._stop_process_group()
        resume_errors = self._resume_navigation() if (getattr(self, '_suspended_pids', []) or self._paused_managers or self._deactivated_nav_nodes) else []
        self._active_slot = 0
        if self._shared_sensor_mode:
            self._phase = 'READY' if not resume_errors else 'READY_WITH_WARNING'
            self._prewarm_due = float('inf')
        else:
            self._phase = 'READY' if not resume_errors else 'READY_WITH_WARNING'
        self._last_error = '' if not resume_errors else ' | '.join(resume_errors)
        response.success = True
        response.message = (
            f'MAP {slot} PGM tersimpan: {pgm_path}; metadata: {yaml_path}; SLAM dihentikan; Nav2 tetap OFF sampai START Navigation'
            if self._shared_sensor_mode else
            f'MAP {slot} PGM tersimpan: {pgm_path}; metadata: {yaml_path}; mapping dihentikan')
        if resume_errors:
            response.message += '; resume warning: ' + ' | '.join(resume_errors)
        self.get_logger().info(response.message)
        return response

    def _stop_without_save_service(self, _request, response):
        if not self._runtime_alive():
            if self._pending_start_slot:
                slot = int(self._pending_start_slot)
                self._pending_start_slot = 0
                self._engine_start_retry_count = 0
                self._session_active = False
                self._phase = 'STOPPING'
                self._publish_session_enabled(False)
                self._stop_process_group()
                self._cleanup_orphan_slam_nodes()
                resume_errors = self._resume_navigation() if (getattr(self, '_suspended_pids', []) or self._paused_managers or self._deactivated_nav_nodes) else []
                self._active_slot = 0
                self._prewarm_due = float('inf')
                self._phase = 'READY' if not resume_errors else 'READY_WITH_WARNING'
                self._last_error = '' if not resume_errors else ' | '.join(resume_errors)
                response.success = not resume_errors
                response.message = f'Persiapan Mapping MAP {slot} dibatalkan; Nav2 tetap OFF'
                if resume_errors:
                    response.message += '; resume warning: ' + ' | '.join(resume_errors)
                return response
            response.success = True
            response.message = 'Mapping sudah berhenti'
            return response
        slot = self._active_slot
        self._phase = 'STOPPING'
        self._session_active = False
        self._publish_session_enabled(False)
        self._stop_process_group()
        resume_errors = self._resume_navigation() if (getattr(self, '_suspended_pids', []) or self._paused_managers or self._deactivated_nav_nodes) else []
        self._active_slot = 0
        if self._shared_sensor_mode:
            self._phase = 'READY' if not resume_errors else 'READY_WITH_WARNING'
            self._prewarm_due = float('inf')
        else:
            self._phase = 'READY' if not resume_errors else 'READY_WITH_WARNING'
        self._last_error = '' if not resume_errors else ' | '.join(resume_errors)
        response.success = not resume_errors
        response.message = f'Mapping MAP {slot} dihentikan tanpa save'
        if self._shared_sensor_mode:
            response.message += '; Nav2 tetap OFF'
        if resume_errors:
            response.message += '; resume warning: ' + ' | '.join(resume_errors)
        return response

    def _sample_host_stats(self):
        try:
            fields = Path('/proc/stat').read_text().splitlines()[0].split()[1:]
            values = [int(v) for v in fields]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            cpu = self._host_stats.get('cpu_percent')
            if self._last_cpu_total is not None and total > self._last_cpu_total:
                dt = total - self._last_cpu_total
                di = max(0, idle - self._last_cpu_idle)
                cpu = 100.0 * max(0, dt - di) / dt
            self._last_cpu_total, self._last_cpu_idle = total, idle
            mem = {}
            for line in Path('/proc/meminfo').read_text().splitlines():
                if ':' not in line:
                    continue
                key, rest = line.split(':', 1)
                try: mem[key] = float(rest.strip().split()[0])
                except Exception: pass
            total_kb = mem.get('MemTotal', 0.0)
            avail_kb = mem.get('MemAvailable', 0.0)
            used_gb = max(0.0, total_kb - avail_kb) / 1048576.0
            total_gb = total_kb / 1048576.0
            self._host_stats = {
                'cpu_percent': cpu,
                'ram_used_gb': used_gb,
                'ram_total_gb': total_gb,
            }
        except Exception:
            pass
        return dict(self._host_stats)

    def _publish_status(self):
        if (not self._shared_sensor_mode) and self._proc is not None and self._proc.poll() is not None:
            rc = self._proc.returncode
            was_active = bool(self._active_slot)
            if was_active:
                self._last_error = f'mapping SLAM berhenti sendiri rc={rc}'
                self._phase = 'ERROR'
            PID_FILE.unlink(missing_ok=True)
            if self._log_handle:
                try:
                    self._log_handle.close()
                except Exception:
                    pass
            self._log_handle = None
            self._proc = None
            self._active_slot = 0
            if was_active and self._paused_managers:
                resume_errors = self._resume_navigation()
                if resume_errors:
                    self._last_error += ' | resume: ' + ' | '.join(resume_errors)

        with self._lock:
            latest_map = self._latest_map
            map_age = (time.monotonic() - self._last_map_monotonic) if self._last_map_monotonic else None
        # Treat PREPARING as mapping-active too. This prevents the Nav2
        # keepalive/recovery controller from reactivating AMCL/map_server while
        # BAB 4.2 is deliberately isolating them before SLAM becomes RUNNING.
        active = self._runtime_alive() or bool(self._pending_start_slot)
        display_slot = self._active_slot or self._pending_start_slot
        mapping_meta = {}
        if latest_map is not None:
            origin = latest_map.info.origin.position
            mapping_meta = {
                'width': int(latest_map.info.width),
                'height': int(latest_map.info.height),
                'resolution': float(latest_map.info.resolution),
                'origin_x': float(origin.x),
                'origin_y': float(origin.y),
                'frame_id': str(latest_map.header.frame_id),
            }
        if active and self._phase == 'STARTING' and map_age is not None and map_age < 2.0:
            self._phase = 'RUNNING'
        payload = {
            'active': active,
            'slot': display_slot,
            'phase': self._phase,
            'map_fresh': map_age is not None and map_age < 3.0,
            'map_age_s': None if map_age is None else round(map_age, 3),
            'map_topic': self._mapping_topic,
            'map_meta': mapping_meta,
            'map_grid': ({
                'encoding': 'ufo-rle-hex-v1',
                'seq': int(self._map_grid_seq),
                'width': int(latest_map.info.width),
                'height': int(latest_map.info.height),
                'resolution': float(latest_map.info.resolution),
                'origin_x': float(latest_map.info.origin.position.x),
                'origin_y': float(latest_map.info.origin.position.y),
                'data': self._map_grid_rle,
            } if active and latest_map is not None and self._map_grid_rle else {}),
            'saved_path': self._saved_path,
            'saved_pgm': self._saved_path,
            'saved_yaml': self._saved_yaml_path,
            'saved_slot': self._saved_slot,
            'error': self._last_error,
            'pid': self._proc.pid if self._proc is not None and self._proc.poll() is None else 0,
            'engine_ready': bool(self._engine_ready),
            'engine_state': 'READY' if self._engine_ready else ('PREPARING' if self._proc is not None else 'OFF'),
            'engine_uptime_s': round(time.monotonic() - self._engine_started_at, 2) if self._engine_started_at else 0.0,
            'engine_config_synced': self._engine_config_mtime_ns == self._slam_params_mtime_ns() if self._shared_sensor_mode else True,
            'engine_restart_reason': self._engine_restart_reason,
            'shared_sensor_mode': self._shared_sensor_mode,
            'navigation_paused': bool(active and (getattr(self, '_suspended_pids', []) or self._nav_map_switch_status.get('nav_enabled') is False)),
            'paused_managers': list(self._paused_managers),
            'deactivated_navigation_nodes': list(self._deactivated_nav_nodes),
            'mapping_odom': dict(self._mapping_odom_status),
            'slam': dict(self._map_stats),
            'structure_metrics': dict(self._structure_metrics),
            'host': self._sample_host_stats(),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._status_pub.publish(msg)

    def shutdown(self):
        self._shutting_down = True
        self._pending_start_slot = 0
        self._session_active = False
        if self._shared_sensor_mode:
            try:
                self._publish_session_enabled(False)
            except Exception:
                # During launch shutdown the rcl context may already be invalid.
                # Cleanup of the mapping process must still continue.
                pass
        if self._proc is not None and self._proc.poll() is None:
            self.get_logger().warning('Controller shutdown: mapping engine dihentikan TANPA save')
            self._stop_process_group()
        if self._shared_sensor_mode and (getattr(self, '_suspended_pids', []) or self._paused_managers or self._deactivated_nav_nodes):
            self._resume_navigation()


def main(args=None):
    rclpy.init(args=args)
    node = MappingWebController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
