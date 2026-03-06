import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleBotPathNode(Node):
    """Publish velocity commands for multiple TurtleBot4 robots along a timed path."""

    def __init__(self) -> None:
        super().__init__('turtlebot_path_node')

        self.declare_parameter('turtlebot_namespaces', ['tb4_1', 'tb4_2'])
        self.declare_parameter('timer_period', 0.1)

        namespaces = self.get_parameter('turtlebot_namespaces').get_parameter_value().string_array_value
        self._period = self.get_parameter('timer_period').get_parameter_value().double_value

        self._publishers = {
            namespace: self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
            for namespace in namespaces
        }

        self._elapsed_time = 0.0
        self.create_timer(self._period, self._on_timer)
        self.get_logger().info(f'Controlling TurtleBots: {list(self._publishers.keys())}')

    def _on_timer(self) -> None:
        self._elapsed_time += self._period

        # Repeat an infinite rectangle-like motion profile.
        cycle_time = self._elapsed_time % 24.0

        cmd = Twist()
        if cycle_time < 6.0:
            cmd.linear.x = 0.20
            cmd.angular.z = 0.0
        elif cycle_time < 8.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        elif cycle_time < 14.0:
            cmd.linear.x = 0.20
            cmd.angular.z = 0.0
        elif cycle_time < 16.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        elif cycle_time < 22.0:
            cmd.linear.x = 0.20
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

        for publisher in self._publishers.values():
            publisher.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurtleBotPathNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
