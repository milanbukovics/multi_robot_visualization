#!/usr/bin/env python3
"""Hello-world flight: arm, takeoff, hover, land for all enabled Crazyflies."""

import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from crazyflie_interfaces.srv import Arm, Land, Takeoff
from rclpy.duration import Duration
from rclpy.node import Node


TAKEOFF_HEIGHT = 1.0
TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
LAND_HEIGHT = 0.04
LAND_DURATION = 2.5


def main():
    rclpy.init()
    node = Node('crazyflie_path_node')

    # Resolve config path: launch param overrides default install path.
    node.declare_parameter('crazyflies_yaml_file', '')
    configured_path = str(node.get_parameter('crazyflies_yaml_file').value).strip()
    if configured_path:
        config_path = Path(configured_path)
    else:
        config_path = (
            Path(get_package_share_directory('multi_robot')) / 'config' / 'crazyflies.yaml'
        )

    with config_path.open('r') as f:
        config = yaml.safe_load(f)

    enabled_cfs = [name for name, cfg in config['robots'].items() if cfg.get('enabled', False)]
    node.get_logger().info(f'Found {len(enabled_cfs)} enabled Crazyflies: {enabled_cfs}')

    if not enabled_cfs:
        node.get_logger().error('No enabled Crazyflies found in config!')
        node.destroy_node()
        rclpy.shutdown()
        return

    clients = {}
    for cf_name in enabled_cfs:
        clients[cf_name] = {
            'arm': node.create_client(Arm, f'/{cf_name}/arm'),
            'takeoff': node.create_client(Takeoff, f'/{cf_name}/takeoff'),
            'land': node.create_client(Land, f'/{cf_name}/land'),
        }

    node.get_logger().info('Waiting for services...')
    for cf_name, cf_clients in clients.items():
        cf_clients['arm'].wait_for_service()
        cf_clients['takeoff'].wait_for_service()
        cf_clients['land'].wait_for_service()
    node.get_logger().info('All services available!')

    # ARM
    node.get_logger().info('Arming all motors...')
    arm_futures = {}
    for cf_name, cf_clients in clients.items():
        req = Arm.Request()
        req.arm = True
        arm_futures[cf_name] = cf_clients['arm'].call_async(req)
    for cf_name, future in arm_futures.items():
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(
            f'{cf_name}: {"Armed" if future.result() is not None else "Arm FAILED"}'
        )

    time.sleep(0.5)

    # TAKEOFF
    node.get_logger().info(f'Taking off to {TAKEOFF_HEIGHT} m...')
    takeoff_futures = {}
    for cf_name, cf_clients in clients.items():
        req = Takeoff.Request()
        req.height = TAKEOFF_HEIGHT
        req.duration = Duration(seconds=TAKEOFF_DURATION).to_msg()
        takeoff_futures[cf_name] = cf_clients['takeoff'].call_async(req)
    for cf_name, future in takeoff_futures.items():
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(
            f'{cf_name}: {"Takeoff sent" if future.result() is not None else "Takeoff FAILED"}'
        )

    # HOVER
    node.get_logger().info(f'Hovering for {HOVER_DURATION} s...')
    time.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    # LAND
    node.get_logger().info('Landing...')
    land_futures = {}
    for cf_name, cf_clients in clients.items():
        req = Land.Request()
        req.height = LAND_HEIGHT
        req.duration = Duration(seconds=LAND_DURATION).to_msg()
        land_futures[cf_name] = cf_clients['land'].call_async(req)
    for cf_name, future in land_futures.items():
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(
            f'{cf_name}: {"Landing sent" if future.result() is not None else "Land FAILED"}'
        )

    time.sleep(LAND_DURATION)

    # DISARM
    node.get_logger().info('Disarming all motors...')
    disarm_futures = {}
    for cf_name, cf_clients in clients.items():
        req = Arm.Request()
        req.arm = False
        disarm_futures[cf_name] = cf_clients['arm'].call_async(req)
    for cf_name, future in disarm_futures.items():
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(
            f'{cf_name}: {"Disarmed" if future.result() is not None else "Disarm FAILED"}'
        )

    node.get_logger().info('Flight complete!')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
