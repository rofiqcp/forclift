#!/usr/bin/python3
"""Read-only V59 perception acceptance check.

Verifies the exact perception chain used by the uploaded master source:
Astra RGB -> YOLO -> fork alignment, including GUI-facing topics.
No control command is ever published.
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from yolo_obstacle_detection_ros2.msg import ObstacleArray, AlignmentState


class PerceptionCheck(Node):
    def __init__(self):
        super().__init__('perception_runtime_check_v59')
        self.counts = {
            '/camera/color/image_raw': 0,
            '/camera/color/camera_info': 0,
            '/camera/color/status': 0,
            '/obstacle_detection/obstacles': 0,
            '/obstacle_detection/visualization': 0,
            '/obstacle_detection/status': 0,
            '/obstacle_detection/performance': 0,
            '/fork_alignment/state': 0,
            '/fork_alignment/image': 0,
        }
        self.details = {}
        self.start = time.monotonic()

        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Image, '/camera/color/image_raw', self._image('/camera/color/image_raw'), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._camera_info, qos_profile_sensor_data)
        self.create_subscription(String, '/camera/color/status', self._string('/camera/color/status'), transient)
        self.create_subscription(ObstacleArray, '/obstacle_detection/obstacles', self._obstacles, reliable)
        self.create_subscription(Image, '/obstacle_detection/visualization', self._image('/obstacle_detection/visualization'), qos_profile_sensor_data)
        self.create_subscription(String, '/obstacle_detection/status', self._string('/obstacle_detection/status'), transient)
        self.create_subscription(String, '/obstacle_detection/performance', self._string('/obstacle_detection/performance'), qos_profile_sensor_data)
        self.create_subscription(AlignmentState, '/fork_alignment/state', self._alignment, transient)
        self.create_subscription(Image, '/fork_alignment/image', self._image('/fork_alignment/image'), qos_profile_sensor_data)

    def _bump(self, topic, detail=''):
        self.counts[topic] += 1
        if detail:
            self.details[topic] = detail

    def _image(self, topic):
        def cb(msg):
            self._bump(topic, f'{msg.width}x{msg.height} {msg.encoding}')
        return cb

    def _camera_info(self, msg):
        self._bump('/camera/color/camera_info', f'{msg.width}x{msg.height} frame={msg.header.frame_id}')

    def _string(self, topic):
        def cb(msg):
            self._bump(topic, str(msg.data)[:180])
        return cb

    def _obstacles(self, msg):
        self._bump('/obstacle_detection/obstacles',
                   f'detections={len(msg.obstacles)} static={msg.static_count} dynamic={msg.dynamic_count} pallet={msg.pallet_count}')

    def _alignment(self, msg):
        self._bump('/fork_alignment/state',
                   f'state={msg.state_text} valid={bool(msg.data_valid)} pallet={bool(msg.pallet_detected)}')


def main():
    duration = max(5.0, float(os.environ.get('PERCEPTION_CHECK_SECONDS', '12')))
    rclpy.init()
    node = PerceptionCheck()
    try:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        names = {name for name, _ns in node.get_node_names_and_namespaces()}
        expected_nodes = ['astra_rgb_v4l2_node', 'obstacle_detector_node', 'hole_block_alignment_node']
        failed = []
        print('\n=== V59 PERCEPTION ACCEPTANCE ===')
        for name in expected_nodes:
            ok = name in names
            print(f'{"PASS" if ok else "FAIL":4s}  node /{name}')
            if not ok:
                failed.append('node:' + name)

        optional = {'/fork_alignment/image'}
        for topic, count in node.counts.items():
            rate = count / duration
            required = topic not in optional
            ok = count > 0 if required else True
            status = 'PASS' if ok else 'FAIL'
            if topic in optional and count == 0:
                status = 'INFO'
            detail = node.details.get(topic, '')
            print(f'{status:4s}  {topic:<42} messages={count:<5d} rate={rate:6.2f} Hz  {detail}')
            if required and not ok:
                failed.append(topic)

        cam_rate = node.counts['/camera/color/image_raw'] / duration
        obs_rate = node.counts['/obstacle_detection/obstacles'] / duration
        viz_rate = node.counts['/obstacle_detection/visualization'] / duration
        # Camera is configured at 30 FPS in autonomous; allow margin for startup.
        for label, value, threshold in [
            ('camera image rate', cam_rate, 3.0),
            ('YOLO obstacle stream rate', obs_rate, 1.0),
            ('YOLO visualization rate', viz_rate, 1.0),
        ]:
            ok = value >= threshold
            print(f'{"PASS" if ok else "FAIL":4s}  {label:<42} measured={value:.2f} Hz min={threshold:.2f}')
            if not ok:
                failed.append(label)

        if failed:
            print('\nRESULT: FAIL -> ' + ', '.join(failed))
            print('The first FAIL in the chain is the root diagnostic target; downstream FAILs may be consequences.')
            return 2
        print('\nRESULT: PASS - camera, YOLO and fork-alignment data are all reaching ROS/GUI-facing topics.')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
