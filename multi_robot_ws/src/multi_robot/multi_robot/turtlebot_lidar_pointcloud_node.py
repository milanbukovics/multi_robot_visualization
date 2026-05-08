#!/usr/bin/env python3
"""Convert a TurtleBot RPLIDAR LaserScan + TF into an accumulating world-frame PointCloud2.

One node = one TurtleBot. Launch with `namespace=<tb>` so:
- the TF listener subscribes to the namespaced /<tb>/tf and /<tb>/tf_static,
- the scan subscriber resolves to /<tb>/scan,
- the cloud publisher resolves to /<tb>/pointcloud.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def _quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    xw, yw, zw = qx * qw, qy * qw, qz * qw
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - zw),     2 * (xz + yw)],
        [2 * (xy + zw),     1 - 2 * (xx + zz), 2 * (yz - xw)],
        [2 * (xz - yw),     2 * (yz + xw),     1 - 2 * (xx + yy)],
    ])


class TurtlebotLidarPointcloudNode(Node):
    def __init__(self) -> None:
        super().__init__('turtlebot_lidar_pointcloud_node')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('max_points', 100000)
        self.declare_parameter('range_min', 0.05)

        self._world_frame = str(self.get_parameter('world_frame').value)
        self._max_points = int(self.get_parameter('max_points').value)
        self._range_min = float(self.get_parameter('range_min').value)

        self._points: list[tuple[float, float, float]] = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self._pub = self.create_publisher(PointCloud2, 'pointcloud', 10)

        self.get_logger().info(
            f'Accumulating LIDAR -> world frame "{self._world_frame}" '
            f'(max_points={self._max_points})')

    def _scan_cb(self, msg: LaserScan) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                msg.header.frame_id,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'TF lookup failed ({self._world_frame} <- {msg.header.frame_id}): {exc}',
                throttle_duration_sec=5.0)
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        rot = _quat_to_rotmat(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z])

        angle = msg.angle_min
        for r in msg.ranges:
            if self._range_min < r < msg.range_max:
                p_robot = np.array([r * math.cos(angle), r * math.sin(angle), 0.0])
                p_world = rot @ p_robot + translation
                self._points.append(tuple(p_world.tolist()))
            angle += msg.angle_increment

        if len(self._points) > self._max_points:
            self._points = self._points[-self._max_points:]

        self._publish()

    def _publish(self) -> None:
        if not self._points:
            return
        arr = np.asarray(self._points, dtype=np.float32)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.height = 1
        msg.width = arr.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * arr.shape[0]
        msg.data = arr.tobytes()
        msg.is_dense = True
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurtlebotLidarPointcloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
