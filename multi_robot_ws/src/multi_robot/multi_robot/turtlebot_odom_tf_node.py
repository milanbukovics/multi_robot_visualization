#!/usr/bin/env python3
"""Publish odom->base_link TF from the nav_msgs/Odometry topic.

The Create3 base on TurtleBot4 publishes odometry as a nav_msgs/Odometry
message but does not publish the corresponding TF transform.  This node
fills that gap so that slam_toolbox and the cloud accumulator can find the
robot pose in the global TF tree.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTFNode(Node):
    def __init__(self) -> None:
        super().__init__('turtlebot_odom_tf_node')
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self._cb, 10)

    def _cb(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
