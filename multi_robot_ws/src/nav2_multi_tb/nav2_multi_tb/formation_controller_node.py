#!/usr/bin/env python3
"""formation_controller_node.py — Synchronous multi-TurtleBot formation controller.

Design
------
The formation is a **straight line** of N robots, each separated by a constant
distance ``d`` along the line's heading.  The operator sends a single 2-D goal
pose (x, y, θ) via the ``/formation/goal`` topic; the controller computes an
individual goal for every robot so that they arrive at their formation slots
simultaneously and then hold position.

Formation layout (N=3, d=0.8 m, θ=0 → robots spaced along X axis):

    leader (tb0)  ←── d ──→  tb1  ←── d ──→  tb2

Robot ``i`` (0-indexed) gets goal:
    x_i = goal_x - i * d * cos(goal_θ)
    y_i = goal_y - i * d * sin(goal_θ)
    θ_i = goal_θ

The controller uses the ``nav2_msgs/action/NavigateToPose`` action for each
robot, so a full Nav2 stack (SLAM + planner + controller + lifecycle) must be
running per robot.  See ``nav2_multi_tb.launch.py`` which starts all of that.

Parameters (ROS 2 node parameters)
-----------------------------------
namespaces          : list[str]  Active TB namespaces, e.g. ['tb0','tb1','tb2']
formation_spacing   : float      d — distance between consecutive robots (m)
goal_tolerance_xy   : float      Radius at which a sub-goal counts as reached (m)
goal_timeout        : float      Seconds before a navigation action is aborted
publish_rate        : float      Hz for the formation status publisher

Topics
------
Subscribed:
    /formation/goal         geometry_msgs/PoseStamped   Operator sends a formation goal
    /{ns}/odom              nav_msgs/Odometry           Per-robot odometry for status

Published:
    /formation/status       nav2_multi_tb/FormationStatus (std_msgs/String for now)
    /formation/goal_markers visualization_msgs/MarkerArray  RViz visualisation of sub-goals

Actions (client)
----------------
    /{ns}/navigate_to_pose  nav2_msgs/action/NavigateToPose  One client per robot
"""

import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _yaw_to_quat(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def _pose_stamped(x: float, y: float, yaw: float, frame: str, stamp) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position = Point(x=x, y=y, z=0.0)
    ps.pose.orientation = _yaw_to_quat(yaw)
    return ps


def _dist(pose: Pose, x: float, y: float) -> float:
    return math.hypot(pose.position.x - x, pose.position.y - y)


# ---------------------------------------------------------------------------
# Per-robot action wrapper
# ---------------------------------------------------------------------------

class RobotNavigator:
    """Thin wrapper around a single NavigateToPose action client."""

    def __init__(self, node: Node, namespace: str):
        self.ns = namespace
        self._node = node
        self._client = ActionClient(
            node,
            NavigateToPose,
            f'/{namespace}/navigate_to_pose',
        )
        self._goal_handle = None
        self._status = GoalStatus.STATUS_UNKNOWN
        self._lock = threading.Lock()
        self.current_pose: Pose | None = None

    # ------------------------------------------------------------------
    def send_goal(self, pose_stamped: PoseStamped) -> None:
        """Send a navigation goal asynchronously; cancel any existing goal first."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(
                f'[{self.ns}] NavigateToPose action server not available')
            return

        # Cancel the previous goal if one is active
        with self._lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        with self._lock:
            if not handle.accepted:
                self._node.get_logger().warn(f'[{self.ns}] Goal rejected')
                self._status = GoalStatus.STATUS_CANCELED
                return
            self._goal_handle = handle
            self._status = GoalStatus.STATUS_EXECUTING
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        with self._lock:
            self._status = future.result().status
            self._goal_handle = None
        self._node.get_logger().info(
            f'[{self.ns}] Navigation finished, status={self._status}')

    def _feedback_cb(self, feedback_msg):
        # Could expose distance_remaining here if needed
        pass

    # ------------------------------------------------------------------
    @property
    def is_navigating(self) -> bool:
        with self._lock:
            return self._status == GoalStatus.STATUS_EXECUTING

    @property
    def succeeded(self) -> bool:
        with self._lock:
            return self._status == GoalStatus.STATUS_SUCCEEDED

    def cancel(self) -> None:
        with self._lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()


# ---------------------------------------------------------------------------
# Formation controller node
# ---------------------------------------------------------------------------

class FormationControllerNode(Node):
    """Coordinates multiple TurtleBots in a line formation under Nav2."""

    def __init__(self) -> None:
        super().__init__('formation_controller')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('namespaces', ['tb0', 'tb1', 'tb2', 'tb3', 'tb4'])
        self.declare_parameter('formation_spacing', 0.8)     # metres between robots
        self.declare_parameter('goal_tolerance_xy', 0.15)    # metres
        self.declare_parameter('goal_timeout', 120.0)        # seconds
        self.declare_parameter('publish_rate', 2.0)          # Hz
        self.declare_parameter('global_frame', 'world')

        raw_ns = self.get_parameter('namespaces').get_parameter_value().string_array_value
        self._namespaces: list[str] = list(raw_ns)
        self._d: float = self.get_parameter('formation_spacing').get_parameter_value().double_value
        self._tol: float = self.get_parameter('goal_tolerance_xy').get_parameter_value().double_value
        self._timeout: float = self.get_parameter('goal_timeout').get_parameter_value().double_value
        self._rate: float = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._frame: str = self.get_parameter('global_frame').get_parameter_value().string_value

        self.get_logger().info(
            f'Formation controller started: robots={self._namespaces}, '
            f'd={self._d} m, frame={self._frame}')

        # ── Per-robot navigators ─────────────────────────────────────────
        self._navigators: dict[str, RobotNavigator] = {
            ns: RobotNavigator(self, ns) for ns in self._namespaces
        }

        # ── Odometry subscriptions ───────────────────────────────────────
        for ns in self._namespaces:
            self.create_subscription(
                Odometry,
                f'/{ns}/odom',
                lambda msg, n=ns: self._odom_cb(msg, n),
                10,
            )

        # ── Formation goal subscription ──────────────────────────────────
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/formation/goal',
            self._goal_cb,
            10,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/formation/status', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/formation/goal_markers', 10)

        # ── Periodic status timer ────────────────────────────────────────
        self.create_timer(1.0 / self._rate, self._status_timer_cb)

        self._last_goal: PoseStamped | None = None
        self._sub_goals: dict[str, PoseStamped] = {}

    # ── Odometry callback ────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        self._navigators[ns].current_pose = msg.pose.pose

    # ── Formation goal callback ──────────────────────────────────────────
    def _goal_cb(self, msg: PoseStamped) -> None:
        """Decompose the single formation goal into per-robot sub-goals."""
        self._last_goal = msg

        # Extract desired formation heading from the goal's orientation (yaw only)
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Formation goal received: ({gx:.2f}, {gy:.2f}), '
            f'θ={math.degrees(yaw):.1f}°, d={self._d} m')

        # Build sub-goals: robot i is i*d behind the formation leader
        markers = MarkerArray()
        for i, ns in enumerate(self._namespaces):
            xi = gx - i * self._d * math.cos(yaw)
            yi = gy - i * self._d * math.sin(yaw)

            ps = _pose_stamped(xi, yi, yaw, self._frame, stamp)
            self._sub_goals[ns] = ps

            self.get_logger().info(
                f'  [{ns}] sub-goal → ({xi:.2f}, {yi:.2f})')

            # Send to Nav2
            self._navigators[ns].send_goal(ps)

            # Visualisation marker (arrow)
            m = Marker()
            m.header = Header(frame_id=self._frame, stamp=stamp)
            m.ns = 'formation_goals'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position = Point(x=xi, y=yi, z=0.05)
            m.pose.orientation = _yaw_to_quat(yaw)
            m.scale.x = 0.4   # shaft length
            m.scale.y = 0.08  # shaft width
            m.scale.z = 0.08  # head
            # Colour: leader is green, followers are blue
            c = ColorRGBA(a=0.9)
            if i == 0:
                c.g = 1.0
            else:
                c.b = 1.0
            m.color = c
            markers.markers.append(m)

        # Line markers between consecutive sub-goals
        for i in range(len(self._namespaces) - 1):
            ns_a = self._namespaces[i]
            ns_b = self._namespaces[i + 1]
            xa = gx - i * self._d * math.cos(yaw)
            ya = gy - i * self._d * math.sin(yaw)
            xb = gx - (i + 1) * self._d * math.cos(yaw)
            yb = gy - (i + 1) * self._d * math.sin(yaw)

            lm = Marker()
            lm.header = Header(frame_id=self._frame, stamp=stamp)
            lm.ns = 'formation_lines'
            lm.id = i
            lm.type = Marker.LINE_STRIP
            lm.action = Marker.ADD
            lm.scale.x = 0.03
            lm.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)
            lm.points = [Point(x=xa, y=ya, z=0.05),
                         Point(x=xb, y=yb, z=0.05)]
            markers.markers.append(lm)

        self._marker_pub.publish(markers)

    # ── Periodic status callback ─────────────────────────────────────────
    def _status_timer_cb(self) -> None:
        if self._last_goal is None:
            return

        lines = ['Formation status:']
        all_done = True
        for ns, nav in self._navigators.items():
            if nav.is_navigating:
                state = 'NAVIGATING'
                all_done = False
            elif nav.succeeded:
                state = 'AT_GOAL'
            else:
                state = 'IDLE'
                all_done = False

            pose_str = '(unknown)'
            if nav.current_pose is not None:
                p = nav.current_pose
                pose_str = f'({p.position.x:.2f}, {p.position.y:.2f})'

            if ns in self._sub_goals:
                sg = self._sub_goals[ns]
                sx, sy = sg.pose.position.x, sg.pose.position.y
                goal_str = f'→({sx:.2f},{sy:.2f})'
            else:
                goal_str = '→(none)'

            lines.append(f'  {ns}: {state} {pose_str} {goal_str}')

        if all_done and self._last_goal is not None:
            lines.append('  ✓ All robots at formation positions.')

        msg = String()
        msg.data = '\n'.join(lines)
        self._status_pub.publish(msg)
        self.get_logger().info(msg.data)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
