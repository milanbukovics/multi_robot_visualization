#!/usr/bin/env python3
"""formation_goal_publisher.py — CLI helper to publish a formation goal.

Usage (after sourcing your workspaces):

    ros2 run nav2_multi_tb formation_goal_publisher \\
        --ros-args -p x:=2.0 -p y:=1.0 -p yaw_deg:=0.0

This publishes a single geometry_msgs/PoseStamped to /formation/goal.
The formation_controller_node picks it up and drives all robots to their
formation slots.

Parameters
----------
x       : float  Goal X in the global frame (default 0.0)
y       : float  Goal Y in the global frame (default 0.0)
yaw_deg : float  Formation heading in degrees (default 0.0 → pointing +X)
frame   : str    Coordinate frame (default 'world')
"""

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.node import Node


class FormationGoalPublisher(Node):
    def __init__(self) -> None:
        super().__init__('formation_goal_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('frame', 'world')

        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        yaw_deg = self.get_parameter('yaw_deg').get_parameter_value().double_value
        frame = self.get_parameter('frame').get_parameter_value().string_value

        yaw = math.radians(yaw_deg)

        pub = self.create_publisher(PoseStamped, '/formation/goal', 10)

        # Give the subscriber time to connect
        self.create_timer(0.5, lambda: self._publish_and_exit(pub, x, y, yaw, frame))

    def _publish_and_exit(self, pub, x, y, yaw, frame):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position = Point(x=x, y=y, z=0.0)
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        msg.pose.orientation = q
        pub.publish(msg)
        self.get_logger().info(
            f'Published formation goal: ({x:.2f}, {y:.2f}), θ={math.degrees(yaw):.1f}°, '
            f'frame={frame}')
        raise SystemExit


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationGoalPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
