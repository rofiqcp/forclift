#!/usr/bin/python3
"""Continuous health gate for the minimally filtered LiDAR safety stream.

Freshness alone is not sufficient for collision safety: a serial/parser fault can
keep a LaserScan topic alive while producing almost no usable ranges.  This node
therefore validates scan density, timing, frame/range sanity and the LiDAR
driver's connected/motor state.  It publishes a latched/reliable Boolean used by
the final autonomous command guard, plus a human-readable JSON status string.
"""
from collections import deque
import json
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class LidarSafetyHealth(Node):
    def __init__(self):
        super().__init__('lidar_safety_health')
        self.declare_parameter('scan_topic', '/scan_safety')
        self.declare_parameter('driver_status_topic', '/lidar/status')
        self.declare_parameter('healthy_topic', '/lidar/safety_healthy')
        self.declare_parameter('health_text_topic', '/lidar/safety_health')
        self.declare_parameter('scan_timeout_sec', 0.35)
        self.declare_parameter('status_timeout_sec', 1.5)
        self.declare_parameter('min_valid_beams', 8)
        self.declare_parameter('min_valid_ratio', 0.015)
        self.declare_parameter('min_scan_hz', 5.0)
        self.declare_parameter('max_scan_hz', 20.0)
        self.declare_parameter('recovery_good_scans', 3)
        self.declare_parameter('max_invalid_packets_per_status', 3)
        self.declare_parameter('max_malformed_packets_per_status', 2)
        self.declare_parameter('max_checksum_failures_per_status', 10)
        self.declare_parameter('require_driver_connected', True)
        self.declare_parameter('require_motor_running', True)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.scan_timeout = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.status_timeout = max(0.5, float(self.get_parameter('status_timeout_sec').value))
        self.min_valid_beams = max(1, int(self.get_parameter('min_valid_beams').value))
        self.min_valid_ratio = min(1.0, max(0.0, float(self.get_parameter('min_valid_ratio').value)))
        self.min_scan_hz = max(0.1, float(self.get_parameter('min_scan_hz').value))
        self.max_scan_hz = max(self.min_scan_hz, float(self.get_parameter('max_scan_hz').value))
        self.recovery_good_scans = max(1, int(self.get_parameter('recovery_good_scans').value))
        self.max_invalid_delta = max(0, int(self.get_parameter('max_invalid_packets_per_status').value))
        self.max_malformed_delta = max(0, int(self.get_parameter('max_malformed_packets_per_status').value))
        self.max_checksum_delta = max(0, int(self.get_parameter('max_checksum_failures_per_status').value))
        self.require_connected = bool(self.get_parameter('require_driver_connected').value)
        self.require_motor = bool(self.get_parameter('require_motor_running').value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.health_pub = self.create_publisher(
            Bool, str(self.get_parameter('healthy_topic').value), state_qos)
        self.text_pub = self.create_publisher(
            String, str(self.get_parameter('health_text_topic').value), state_qos)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value), self._scan_cb, sensor_qos)
        self.create_subscription(
            String, str(self.get_parameter('driver_status_topic').value), self._status_cb, status_qos)

        self.last_scan_wall = None
        self.last_status_wall = None
        self.last_valid_beams = 0
        self.last_total_beams = 0
        self.last_valid_ratio = 0.0
        self.last_frame = ''
        self.last_scan_sane = False
        self.driver_connected = False
        self.motor_running = False
        self.parser_quality_ok = False
        self.strict_checksum = True
        self.parser_counters = {}
        self.parser_deltas = {'invalid_packets': 0, 'malformed_packets': 0, 'checksum_failures': 0}
        self.good_scan_streak = 0
        self.intervals = deque(maxlen=20)
        self.healthy = False
        self._last_report = None
        self._ever_healthy = False

        hz = max(2.0, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            '[LIDAR-SAFETY-HEALTH] armed: freshness + usable-beam density + driver/motor state required')

    @staticmethod
    def _now():
        return time.monotonic()

    def _scan_cb(self, msg: LaserScan):
        now = self._now()
        if self.last_scan_wall is not None:
            dt = now - self.last_scan_wall
            if dt > 0.0:
                self.intervals.append(dt)
        self.last_scan_wall = now
        self.last_frame = msg.header.frame_id
        total = len(msg.ranges)
        valid = 0
        range_bounds_sane = (
            math.isfinite(msg.range_min) and math.isfinite(msg.range_max) and
            msg.range_min >= 0.0 and msg.range_max > msg.range_min)
        if range_bounds_sane:
            for value in msg.ranges:
                if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                    valid += 1
        ratio = (valid / total) if total else 0.0
        shape_sane = total >= 180 and bool(msg.header.frame_id) and math.isfinite(msg.angle_increment) and abs(msg.angle_increment) > 1e-6
        data_good = (
            shape_sane and range_bounds_sane and
            valid >= self.min_valid_beams and ratio >= self.min_valid_ratio)
        self.last_valid_beams = valid
        self.last_total_beams = total
        self.last_valid_ratio = ratio
        self.last_scan_sane = data_good
        if data_good:
            self.good_scan_streak += 1
        else:
            # One physically sparse/corrupt safety scan immediately removes the
            # healthy state. Recovery requires multiple consecutive good scans.
            self.good_scan_streak = 0
            self.healthy = False

    @staticmethod
    def _status_counter(text, label):
        token = label + ': '
        try:
            tail = text.split(token, 1)[1]
            return int(tail.split(' | ', 1)[0].strip())
        except (IndexError, ValueError):
            return None

    def _status_cb(self, msg: String):
        self.last_status_wall = self._now()
        text = msg.data or ''
        self.driver_connected = 'Connected: true' in text
        self.motor_running = 'Motor: RUNNING' in text
        # TMini Plus units are known to report checksum mismatches even while
        # producing stable usable scans. The driver explicitly selects whether
        # checksum is enforcement (strict) or diagnostics-only (advisory).
        if 'StrictChecksum: true' in text:
            self.strict_checksum = True
        elif 'StrictChecksum: false' in text:
            self.strict_checksum = False
        current = {
            'valid_packets': self._status_counter(text, 'ValidPackets'),
            'invalid_packets': self._status_counter(text, 'InvalidPackets'),
            'malformed_packets': self._status_counter(text, 'MalformedPackets'),
            'checksum_failures': self._status_counter(text, 'ChecksumFailures'),
        }
        if any(v is None for v in current.values()):
            self.parser_quality_ok = False
            return
        if self.parser_counters:
            # Counters reset when the driver/reader is rebuilt. Treat a reset as
            # a new baseline rather than a huge negative/positive fault.
            deltas = {}
            for key in ('invalid_packets', 'malformed_packets', 'checksum_failures'):
                prev = int(self.parser_counters.get(key, current[key]))
                now_value = int(current[key])
                deltas[key] = 0 if now_value < prev else now_value - prev
            self.parser_deltas = deltas
            checksum_ok = (
                deltas['checksum_failures'] <= self.max_checksum_delta
                if self.strict_checksum else True)
            self.parser_quality_ok = bool(
                deltas['invalid_packets'] <= self.max_invalid_delta and
                deltas['malformed_packets'] <= self.max_malformed_delta and
                checksum_ok)
        else:
            # Need one interval of parser statistics before declaring the
            # safety stream healthy after startup/recovery.
            self.parser_quality_ok = False
        self.parser_counters = current

    def _timing_ok(self):
        if not self.intervals:
            return self.good_scan_streak >= self.recovery_good_scans
        med = statistics.median(self.intervals)
        if med <= 0.0:
            return False
        hz = 1.0 / med
        return self.min_scan_hz <= hz <= self.max_scan_hz

    def _tick(self):
        now = self._now()
        scan_age = float('inf') if self.last_scan_wall is None else now - self.last_scan_wall
        status_age = float('inf') if self.last_status_wall is None else now - self.last_status_wall
        scan_fresh = scan_age <= self.scan_timeout
        status_fresh = status_age <= self.status_timeout
        driver_ok = (not self.require_connected) or (status_fresh and self.driver_connected)
        motor_ok = (not self.require_motor) or (status_fresh and self.motor_running)
        timing_ok = self._timing_ok()
        recovered = self.good_scan_streak >= self.recovery_good_scans
        healthy = bool(scan_fresh and self.last_scan_sane and timing_ok and recovered and driver_ok and motor_ok and self.parser_quality_ok)
        self.healthy = healthy
        self.health_pub.publish(Bool(data=healthy))

        median_hz = 0.0
        if self.intervals:
            med = statistics.median(self.intervals)
            median_hz = (1.0 / med) if med > 0 else 0.0
        payload = {
            'healthy': healthy,
            'scan_age_sec': None if not math.isfinite(scan_age) else round(scan_age, 4),
            'status_age_sec': None if not math.isfinite(status_age) else round(status_age, 4),
            'valid_beams': self.last_valid_beams,
            'total_beams': self.last_total_beams,
            'valid_ratio': round(self.last_valid_ratio, 4),
            'median_scan_hz': round(median_hz, 3),
            'good_scan_streak': self.good_scan_streak,
            'driver_connected': self.driver_connected,
            'motor_running': self.motor_running,
            'parser_quality_ok': self.parser_quality_ok,
            'strict_checksum': self.strict_checksum,
            'checksum_policy': 'enforced' if self.strict_checksum else 'advisory',
            'parser_deltas': dict(self.parser_deltas),
            'frame': self.last_frame,
            'reasons': [],
        }
        if not scan_fresh:
            payload['reasons'].append('scan_stale')
        if not self.last_scan_sane:
            payload['reasons'].append('scan_sparse_or_invalid')
        if not timing_ok:
            payload['reasons'].append('scan_rate_out_of_range')
        if not driver_ok:
            payload['reasons'].append('driver_disconnected_or_status_stale')
        if not motor_ok:
            payload['reasons'].append('motor_not_running_or_status_stale')
        if not self.parser_quality_ok:
            payload['reasons'].append('parser_quality_degraded_or_warming_up')
        text = json.dumps(payload, sort_keys=True)
        self.text_pub.publish(String(data=text))

        state_key = (healthy, tuple(payload['reasons']))
        if state_key != self._last_report:
            if healthy:
                self._ever_healthy = True
                self.get_logger().info(
                    '[LIDAR-SAFETY-HEALTH] HEALTHY beams=%d/%d (%.1f%%) rate=%.2fHz' % (
                        self.last_valid_beams, self.last_total_beams,
                        100.0 * self.last_valid_ratio, median_hz))
            elif self._ever_healthy:
                self.get_logger().warn('[LIDAR-SAFETY-HEALTH] HOLD %s' % ','.join(payload['reasons']))
            else:
                self.get_logger().info('[LIDAR-SAFETY-HEALTH] startup hold %s' % ','.join(payload['reasons']))
            self._last_report = state_key


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
