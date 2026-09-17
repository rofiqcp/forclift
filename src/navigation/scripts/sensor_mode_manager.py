#!/usr/bin/env python3
"""Single runtime authority for Mapping/Navigation sensor modes.

Mapping selection is consumed by the operational Build Map controller only;
BAB 4.2 remains hard-locked LiDAR-only. Navigation keeps one EKF/TF owner and
this node multiplexes only the measurements allowed by the active mode.
"""
import json
import os
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

MODES = ('lidar', 'lidar_imu', 'lidar_imu_odom_vx')
LABELS = {
    'lidar': 'LiDAR Only',
    'lidar_imu': 'LiDAR + IMU',
    'lidar_imu_odom_vx': 'LiDAR + IMU + Odom VX',
}


def default_config_path() -> Path:
    ws = os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or str(Path.home() / 'forclift')
    root = os.environ.get('AGV_RUNTIME_CONFIG_ROOT') or str(Path(ws) / 'config' / 'runtime')
    return Path(root) / 'navigation' / 'sensor_modes.yaml'


class SensorModeManager(Node):
    def __init__(self):
        super().__init__('sensor_mode_manager')
        self.declare_parameter('config_path', str(default_config_path()))
        self.declare_parameter('freshness_sec', 0.8)
        self.config_path = Path(str(self.get_parameter('config_path').value)).expanduser()
        self.freshness_sec = max(0.2, float(self.get_parameter('freshness_sec').value))
        self.mapping_mode = 'lidar'
        self.navigation_mode = 'lidar_imu_odom_vx'
        self.mapping_session_active = False
        self.nav_enabled = False
        self.nav_busy = False
        self.esc_feedback_valid = False
        self.last = {k: 0.0 for k in ('scan', 'imu', 'lidar_odom', 'esc_odom', 'esc_feedback')}
        self._load_config()

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/sensor_mode/status', state_qos)
        self.odom_vx_pub = self.create_publisher(Odometry, '/sensor_mode/odom_vx', qos_profile_sensor_data)
        self.lidar_heading_pub = self.create_publisher(Odometry, '/sensor_mode/lidar_heading', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/sensor_mode/imu', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_nav', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/lidar/odom', self._on_lidar_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/esc/odom', self._on_esc_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/esc/feedback_valid', self._on_esc_feedback, state_qos)
        self.create_subscription(Bool, '/mapping/session_enabled', self._on_mapping_session, state_qos)
        self.create_subscription(String, '/navigation/map_switch_status', self._on_nav_status, 10)

        for domain in ('mapping', 'navigation'):
            for mode in MODES:
                self.create_service(
                    Trigger, f'/sensor_mode/{domain}/{mode}',
                    lambda req, resp, d=domain, m=mode: self._apply(d, m, resp))
        self.create_timer(0.5, self._publish_status)
        self._publish_status()
        self.get_logger().info(
            'Sensor modes READY mapping=%s navigation=%s', self.mapping_mode, self.navigation_mode)

    def _load_config(self):
        try:
            data = yaml.safe_load(self.config_path.read_text()) or {}
        except (OSError, yaml.YAMLError):
            data = {}
        self.mapping_mode = data.get('mapping_active', 'lidar')
        self.navigation_mode = data.get('navigation_active', 'lidar_imu_odom_vx')
        if self.mapping_mode not in MODES:
            self.mapping_mode = 'lidar'
        if self.navigation_mode not in MODES:
            self.navigation_mode = 'lidar_imu_odom_vx'
        self._persist()

    def _persist(self):
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'schema_version': 1,
            'mapping_active': self.mapping_mode,
            'navigation_active': self.navigation_mode,
        }
        tmp = self.config_path.with_suffix('.yaml.tmp')
        with open(tmp, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(payload, handle, sort_keys=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, self.config_path)

    def _touch(self, key):
        self.last[key] = time.monotonic()

    def _fresh(self, key):
        stamp = self.last.get(key, 0.0)
        return stamp > 0.0 and time.monotonic() - stamp <= self.freshness_sec

    def _on_scan(self, _msg):
        self._touch('scan')

    def _on_imu(self, msg):
        self._touch('imu')
        if self.navigation_mode in ('lidar_imu', 'lidar_imu_odom_vx'):
            self.imu_pub.publish(msg)

    def _on_lidar_odom(self, msg):
        self._touch('lidar_odom')
        if self.navigation_mode in ('lidar', 'lidar_imu'):
            self.odom_vx_pub.publish(msg)
        if self.navigation_mode == 'lidar':
            self.lidar_heading_pub.publish(msg)

    def _on_esc_odom(self, msg):
        self._touch('esc_odom')
        if self.navigation_mode == 'lidar_imu_odom_vx' and self.esc_feedback_valid:
            self.odom_vx_pub.publish(msg)

    def _on_esc_feedback(self, msg):
        self.esc_feedback_valid = bool(msg.data)
        self._touch('esc_feedback')

    def _on_mapping_session(self, msg):
        self.mapping_session_active = bool(msg.data)

    def _on_nav_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.nav_enabled = bool(data.get('nav_enabled', False))
        self.nav_busy = bool(data.get('busy', False))

    def _required(self, domain, mode):
        if mode == 'lidar':
            return ('scan', 'lidar_odom') if domain == 'navigation' else ('scan',)
        if mode == 'lidar_imu':
            return ('scan', 'lidar_odom', 'imu') if domain == 'navigation' else ('scan', 'imu')
        return ('scan', 'esc_odom', 'imu', 'esc_feedback')

    def _readiness(self, domain, mode):
        missing = []
        for key in self._required(domain, mode):
            if key == 'esc_feedback':
                ok = self.esc_feedback_valid and self._fresh('esc_feedback')
            else:
                ok = self._fresh(key)
            if not ok:
                missing.append(key)
        return not missing, missing

    def _apply(self, domain, mode, response):
        if mode not in MODES or domain not in ('mapping', 'navigation'):
            response.success = False
            response.message = 'Mode/domain tidak valid'
            return response
        if domain == 'mapping' and self.mapping_session_active:
            response.success = False
            response.message = 'STOP Build Map terlebih dahulu sebelum mengganti sensor mode'
            return response
        if domain == 'navigation' and (self.nav_enabled or self.nav_busy):
            response.success = False
            response.message = 'STOP Nav2 dan tunggu map switch IDLE sebelum mengganti sensor mode'
            return response
        ready, missing = self._readiness(domain, mode)
        if not ready:
            response.success = False
            response.message = f'{LABELS[mode]} belum READY; missing/stale: {", ".join(missing)}'
            return response
        if domain == 'mapping':
            self.mapping_mode = mode
        else:
            self.navigation_mode = mode
        self._persist()
        self._publish_status()
        response.success = True
        response.message = f'{domain} sensor mode ACTIVE: {LABELS[mode]}'
        return response

    def _publish_status(self):
        mapping_ready, mapping_missing = self._readiness('mapping', self.mapping_mode)
        nav_ready, nav_missing = self._readiness('navigation', self.navigation_mode)
        now = time.monotonic()
        ages = {k: (None if v <= 0.0 else round(now - v, 3)) for k, v in self.last.items()}
        payload = {
            'mapping_active': self.mapping_mode,
            'mapping_label': LABELS[self.mapping_mode],
            'mapping_ready': mapping_ready,
            'mapping_missing': mapping_missing,
            'mapping_locked': self.mapping_session_active,
            'navigation_active': self.navigation_mode,
            'navigation_label': LABELS[self.navigation_mode],
            'navigation_ready': nav_ready,
            'navigation_missing': nav_missing,
            'navigation_locked': self.nav_enabled or self.nav_busy,
            'esc_feedback_valid': self.esc_feedback_valid and self._fresh('esc_feedback'),
            'fresh': {k: self._fresh(k) for k in self.last},
            'age_sec': ages,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = SensorModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
