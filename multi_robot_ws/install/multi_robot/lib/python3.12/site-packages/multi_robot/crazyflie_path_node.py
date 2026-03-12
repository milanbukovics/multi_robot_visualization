#!/usr/bin/env python3
"""Fly all enabled Crazyflies in a circular trajectory using /<cf>/cmd_position."""

import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Arm, Land, NotifySetpointsStop, Takeoff
from rclpy.duration import Duration
from rclpy.node import Node


class CrazyfliePathNode(Node):
    """Arms, flies a circle path, and lands all enabled Crazyflies."""

    def __init__(self) -> None:
        super().__init__('crazyflie_path_node')

        self.declare_parameter('crazyflies_yaml_file', '')
        self.declare_parameter('takeoff_height', 1.0)
        self.declare_parameter('takeoff_duration', 2.5)
        self.declare_parameter('circle_radius', 0.4)
        self.declare_parameter('circle_period', 8.0)
        self.declare_parameter('circle_duration', 20.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('land_height', 0.04)
        self.declare_parameter('land_duration', 2.5)
        self.declare_parameter('frame_id', 'world')

        self._takeoff_height = float(self.get_parameter('takeoff_height').value)
        self._takeoff_duration = float(self.get_parameter('takeoff_duration').value)
        self._circle_radius = float(self.get_parameter('circle_radius').value)
        self._circle_period = float(self.get_parameter('circle_period').value)
        self._circle_duration = float(self.get_parameter('circle_duration').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._land_height = float(self.get_parameter('land_height').value)
        self._land_duration = float(self.get_parameter('land_duration').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        configured_path = str(self.get_parameter('crazyflies_yaml_file').value).strip()
        if configured_path:
            self._config_path = Path(configured_path)
        else:
            self._config_path = (
                Path(get_package_share_directory('multi_robot')) / 'config' / 'crazyflies.yaml'
            )

    def load_enabled_crazyflies(self):
        with self._config_path.open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)

        robots = config.get('robots', {})
        enabled = []
        initial_positions = {}

        for name, cfg in robots.items():
            if cfg.get('enabled', False):
                enabled.append(name)
                initial_position = cfg.get('initial_position', [0.0, 0.0, 0.0])
                initial_positions[name] = (
                    float(initial_position[0]),
                    float(initial_position[1]),
                    float(initial_position[2]),
                )

        return enabled, initial_positions

    def wait_for_service(self, client, service_name: str) -> None:
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service {service_name} ...')

    def run_mission(self) -> None:
        enabled_crazyflies, initial_positions = self.load_enabled_crazyflies()
        self.get_logger().info(
            f'Found {len(enabled_crazyflies)} enabled Crazyflies: {enabled_crazyflies}'
        )

        if not enabled_crazyflies:
            self.get_logger().error('No enabled Crazyflies found in config.')
            return

        clients = {}
        publishers = {}
        for name in enabled_crazyflies:
            clients[name] = {
                'arm': self.create_client(Arm, f'/{name}/arm'),
                'takeoff': self.create_client(Takeoff, f'/{name}/takeoff'),
                'land': self.create_client(Land, f'/{name}/land'),
                'notify_stop': self.create_client(
                    NotifySetpointsStop,
                    f'/{name}/notify_setpoints_stop',
                ),
            }
            publishers[name] = self.create_publisher(Position, f'/{name}/cmd_position', 10)

        self.get_logger().info('Waiting for Crazyflie services...')
        for name, node_clients in clients.items():
            self.wait_for_service(node_clients['arm'], f'/{name}/arm')
            self.wait_for_service(node_clients['takeoff'], f'/{name}/takeoff')
            self.wait_for_service(node_clients['land'], f'/{name}/land')
            self.wait_for_service(node_clients['notify_stop'], f'/{name}/notify_setpoints_stop')

        self.get_logger().info('Arming all Crazyflies...')
        arm_futures = {}
        for name, node_clients in clients.items():
            request = Arm.Request()
            request.arm = True
            arm_futures[name] = node_clients['arm'].call_async(request)
        for name, future in arm_futures.items():
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f'{name}: arm result = {future.result() is not None}')

        time.sleep(0.5)

        self.get_logger().info('Taking off all Crazyflies...')
        takeoff_futures = {}
        for name, node_clients in clients.items():
            request = Takeoff.Request()
            request.height = self._takeoff_height
            request.duration = Duration(seconds=self._takeoff_duration).to_msg()
            takeoff_futures[name] = node_clients['takeoff'].call_async(request)
        for name, future in takeoff_futures.items():
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f'{name}: takeoff result = {future.result() is not None}')

        time.sleep(self._takeoff_duration)

        omega = 2.0 * math.pi / self._circle_period
        dt = 1.0 / self._publish_rate_hz
        total = len(enabled_crazyflies)
        phases = {
            name: 2.0 * math.pi * idx / total for idx, name in enumerate(enabled_crazyflies)
        }

        self.get_logger().info(
            f'Flying circle path radius={self._circle_radius}m period={self._circle_period}s '
            f'duration={self._circle_duration}s'
        )

        start_time = time.monotonic()
        while rclpy.ok() and (time.monotonic() - start_time) < self._circle_duration:
            elapsed = time.monotonic() - start_time
            stamp = self.get_clock().now().to_msg()

            for name in enabled_crazyflies:
                center_x, center_y, _ = initial_positions.get(name, (0.0, 0.0, 0.0))
                theta = omega * elapsed + phases[name]

                msg = Position()
                msg.header.stamp = stamp
                msg.header.frame_id = self._frame_id
                msg.x = float(center_x + self._circle_radius * math.cos(theta))
                msg.y = float(center_y + self._circle_radius * math.sin(theta))
                msg.z = float(self._takeoff_height)
                msg.yaw = 0.0
                publishers[name].publish(msg)

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        self.get_logger().info('Notifying setpoint stop...')
        notify_futures = {}
        for name, node_clients in clients.items():
            request = NotifySetpointsStop.Request()
            request.group_mask = 0
            request.remain_valid_millisecs = 100
            notify_futures[name] = node_clients['notify_stop'].call_async(request)
        for name, future in notify_futures.items():
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f'{name}: notify_stop result = {future.result() is not None}')

        self.get_logger().info('Landing all Crazyflies...')
        land_futures = {}
        for name, node_clients in clients.items():
            request = Land.Request()
            request.height = self._land_height
            request.duration = Duration(seconds=self._land_duration).to_msg()
            land_futures[name] = node_clients['land'].call_async(request)
        for name, future in land_futures.items():
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f'{name}: land result = {future.result() is not None}')

        time.sleep(self._land_duration)

        self.get_logger().info('Disarming all Crazyflies...')
        disarm_futures = {}
        for name, node_clients in clients.items():
            request = Arm.Request()
            request.arm = False
            disarm_futures[name] = node_clients['arm'].call_async(request)
        for name, future in disarm_futures.items():
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f'{name}: disarm result = {future.result() is not None}')

        self.get_logger().info('Crazyflie circle mission complete.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrazyfliePathNode()
    try:
        node.run_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
