#!/usr/bin/env python3
"""Convert Multi-Ranger LaserScan + TF into an accumulating world-frame PointCloud2.

For each enabled Crazyflie listed in crazyflies.yaml, subscribes to /<cf>/scan,
looks up world->cf TF, computes 3D world-frame points for each valid ray, and
publishes the running accumulated cloud on /<cf>/pointcloud.
"""

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_matrix


class CrazyflieAccumulator:
    """Per-Crazyflie subscriber + accumulator + publisher."""

    def __init__(self, node: Node, cf_name: str, world_frame: str,
                 max_points: int, range_min: float):
        self._node = node
        self._cf_name = cf_name
        self._world_frame = world_frame
        self._max_points = max_points
        self._range_min = range_min
        self._points: list[tuple[float, float, float]] = []

        node.create_subscription(
            LaserScan, f'/{cf_name}/scan', self._scan_cb, 10)
        self._pub = node.create_publisher(
            PointCloud2, f'/{cf_name}/pointcloud', 10)

    def _scan_cb(self, msg: LaserScan) -> None:
        # Look up world -> drone frame at the scan's timestamp.
        try:
            tf = self._node.tf_buffer.lookup_transform(
                self._world_frame,
                msg.header.frame_id,
                msg.header.stamp,
                Duration(seconds=0.1),
            )
        except TransformException:
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        translation = np.array([t.x, t.y, t.z])

        angle = msg.angle_min
        for r in msg.ranges:
            if self._range_min < r < msg.range_max:
                p_drone = np.array([r * math.cos(angle), r * math.sin(angle), 0.0])
                p_world = rot @ p_drone + translation
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
        msg.header.stamp = self._node.get_clock().now().to_msg()
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


class MultiRangerPointcloudNode(Node):
    def __init__(self) -> None:
        super().__init__('multi_ranger_pointcloud_node')

        self.declare_parameter('crazyflies_yaml_file', '')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('max_points', 30000)
        self.declare_parameter('range_min', 0.05)

        world_frame = str(self.get_parameter('world_frame').value)
        max_points = int(self.get_parameter('max_points').value)
        range_min = float(self.get_parameter('range_min').value)

        cfg_path = str(self.get_parameter('crazyflies_yaml_file').value).strip()
        if cfg_path:
            config_file = Path(cfg_path)
        else:
            config_file = (
                Path(get_package_share_directory('multi_robot'))
                / 'config' / 'crazyflies.yaml'
            )

        with config_file.open('r') as f:
            cfg = yaml.safe_load(f)
        enabled = [n for n, c in cfg['robots'].items() if c.get('enabled', False)]
        self.get_logger().info(f'Multi-Ranger pointcloud for: {enabled}')

        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)

        self._accumulators = [
            CrazyflieAccumulator(self, name, world_frame, max_points, range_min)
            for name in enabled
        ]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiRangerPointcloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
