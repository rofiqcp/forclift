#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState


class Nav2CoreServiceGate(Node):
    def __init__(self):
        super().__init__('nav2_core_service_gate')
        self.declare_parameter(
            'required_services',
            ['/planner_server/get_state', '/controller_server/get_state'])
        self.declare_parameter('poll_period_sec', 0.25)
        self.declare_parameter('stable_cycles', 4)

        self.required_service_names = list(self.get_parameter('required_services').value)
        self.poll = float(self.get_parameter('poll_period_sec').value)
        self.stable_needed = int(self.get_parameter('stable_cycles').value)
        self._service_clients = {
            name: self.create_client(GetState, name)
            for name in self.required_service_names
        }
        self.stable = 0
        self.last_status = None
        self.ready = False
        self.timer = self.create_timer(self.poll, self.tick)
        self.get_logger().info(
            '[NAV2-CORE-SERVICE] waiting for canonical lifecycle services: ' +
            ', '.join(self.required_service_names))

    def tick(self):
        status = {
            name: client.service_is_ready()
            for name, client in self._service_clients.items()
        }
        if status != self.last_status:
            detail = ' '.join(
                f'{name}={"READY" if ready else "WAIT"}'
                for name, ready in status.items())
            self.get_logger().info('[NAV2-CORE-SERVICE] ' + detail)
            self.last_status = status

        if all(status.values()):
            self.stable += 1
        else:
            self.stable = 0

        if self.stable >= self.stable_needed and not self.ready:
            self.ready = True
            self.timer.cancel()
            self.get_logger().info(
                '[NAV2-CORE-SERVICE] READY - planner/controller lifecycle services stable; exiting gate')


def main():
    rclpy.init()
    node = Nav2CoreServiceGate()
    rc = 2
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.20)
        rc = 0 if node.ready else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
