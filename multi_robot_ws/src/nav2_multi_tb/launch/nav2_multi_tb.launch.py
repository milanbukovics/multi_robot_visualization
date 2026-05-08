#!/usr/bin/env python3
"""nav2_multi_tb.launch.py — Launch the full multi-TurtleBot Nav2 formation system.

What this starts
----------------
For EACH enabled robot in turtlebots.yaml:
  1. static_transform_publisher        world → {ns}/map  (anchors SLAM to world)
  2. turtlebot_odom_tf_node            /{ns}/odom → odom→{ns}/base_link TF
  3. topic_tools relay × 2             /{ns}/tf  → /tf  and  /{ns}/tf_static → /tf_static
  4. slam_toolbox async_slam_toolbox_node  namespaced SLAM
  5. nav2_lifecycle_manager (SLAM)     auto-activates slam_toolbox
  6. controller_server                 namespaced, DWB local planner
  7. planner_server                    namespaced, NavFn global planner
  8. bt_navigator                      namespaced, NavigateToPose action
  9. recoveries_server                 namespaced, spin/backup/wait
 10. nav2_lifecycle_manager (Nav2)     auto-activates all Nav2 nodes

Plus ONE shared node:
 11. formation_controller_node         Subscribes /formation/goal, drives all robots

Launch arguments
----------------
  formation_spacing  : distance d between robots in metres (default 0.8)
  global_frame       : shared world frame id (default 'world')
  rviz               : 'true'/'false' — open RViz2 (default 'true')

Usage
-----
  ros2 launch nav2_multi_tb nav2_multi_tb.launch.py
  ros2 launch nav2_multi_tb nav2_multi_tb.launch.py formation_spacing:=1.0
  ros2 launch nav2_multi_tb nav2_multi_tb.launch.py rviz:=false

Then send a goal:
  ros2 run nav2_multi_tb formation_goal_publisher \\
      --ros-args -p x:=3.0 -p y:=0.0 -p yaw_deg:=0.0

TF tree produced (per robot, e.g. tb0):
  world
  └── tb0/map          ← static TF (world origin)
      └── odom         ← slam_toolbox (map→odom correction)
          └── tb0/base_link  ← turtlebot_odom_tf_node (odom odometry)
              └── tb0/rplidar (or tb0/laser_frame) ← from TurtleBot driver via relay

Known caveats
-------------
* base_link is per-robot: launch overrides base_frame to {ns}/base_link AND
  turtlebot_odom_tf_node publishes odom → {ns}/base_link so frames don't collide.
* The global costmap's global_frame is set to {ns}/map per robot — each robot
  navigates in its own SLAM map.  Map merging (multirobot_map_merge) is future work.
* QoS: if teleop doesn't work, use:
    --remap cmd_vel:=/{ns}/cmd_vel_unstamped
    -p qos_overrides./{ns}/cmd_vel_unstamped.publisher.reliability:=best_effort
"""

import os
import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = 'nav2_multi_tb'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _clean_ld_library_path() -> str:
    """Strip snap-installed library paths to prevent RViz2 Qt crashes."""
    raw = os.environ.get('LD_LIBRARY_PATH', '')
    parts = [p for p in raw.split(':') if p and not p.startswith('/snap/')]
    return ':'.join(parts)


def _load_turtlebots(package_share: str) -> list[dict]:
    """Return list of enabled robot dicts, sorted by formation_slot."""
    yaml_path = os.path.join(package_share, 'config', 'turtlebots.yaml')
    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)

    robots = []
    for name, data in cfg['robots'].items():
        if data.get('enabled', False):
            robots.append({
                'name': name,
                'slot': data.get('formation_slot', 99),
                'initial_position': data.get('initial_position', [0.0, 0.0, 0.0]),
            })

    robots.sort(key=lambda r: r['slot'])
    return robots


def _yaw_to_quat_args(yaw_deg: float) -> list[str]:
    """Return static_transform_publisher quaternion arguments for a pure-yaw rotation."""
    yaw = math.radians(yaw_deg)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return [str(qz), str(qw)]   # only z and w non-zero for 2-D yaw


# ---------------------------------------------------------------------------
# Launch description builder
# ---------------------------------------------------------------------------

def _generate_nodes(context, *args, **kwargs):
    formation_spacing = float(LaunchConfiguration('formation_spacing').perform(context))
    global_frame = LaunchConfiguration('global_frame').perform(context)
    rviz_flag = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes')

    package_share = get_package_share_directory(PACKAGE_NAME)
    nav2_yaml = os.path.join(package_share, 'config', 'nav2_multi_tb.yaml')
    slam_yaml = os.path.join(package_share, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'nav2_multi_tb.rviz')

    robots = _load_turtlebots(package_share)
    namespaces = [r['name'] for r in robots]

    if not robots:
        raise RuntimeError(
            'nav2_multi_tb: no robots enabled in turtlebots.yaml — nothing to launch!')

    nodes = []

    # ── Per-robot node group ──────────────────────────────────────────────
    for robot in robots:
        ns = robot['name']
        x, y, yaw_deg = robot['initial_position']
        yaw = math.radians(yaw_deg)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        base_link = f'{ns}/base_link'

        # 1. Static TF: world → {ns}/map  (anchors SLAM map to world origin)
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{ns}_map',
            arguments=[
                str(x), str(y), '0',          # translation
                '0', '0', str(qz), str(qw),   # quaternion (roll=0, pitch=0)
                global_frame, f'{ns}/map',
            ],
        ))

        # 2. Odom → {ns}/base_link TF bridge
        #    (Create3 publishes Odometry but NOT the odom→base_link TF — see report §4.1)
        #    We reuse multi_robot's turtlebot_odom_tf_node; it reads child_frame_id from
        #    the Odometry message.  The TurtleBot driver must publish child_frame_id as
        #    '{ns}/base_link' or we publish a corrected version below.
        nodes.append(Node(
            package='multi_robot',
            executable='turtlebot_odom_tf_node',
            name=f'odom_tf_{ns}',
            remappings=[('odom', f'/{ns}/odom')],
        ))

        # 3a. TF relay: /{ns}/tf → /tf
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'tf_relay_{ns}',
            arguments=[f'/{ns}/tf', '/tf'],
        ))

        # 3b. TF relay: /{ns}/tf_static → /tf_static
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'tf_static_relay_{ns}',
            arguments=[f'/{ns}/tf_static', '/tf_static'],
        ))

        # 4. SLAM toolbox (async, namespaced)
        nodes.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[
                slam_yaml,
                {
                    'odom_frame': 'odom',
                    'map_frame': f'{ns}/map',
                    'base_frame': base_link,
                },
            ],
            remappings=[
                ('/scan',         f'/{ns}/scan'),
                ('/map',          f'/{ns}/map'),
                ('/map_metadata', f'/{ns}/map_metadata'),
            ],
        ))

        # 5. Lifecycle manager for SLAM
        nodes.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=f'lifecycle_manager_slam_{ns}',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
        ))

        # 6–9. Nav2 stack (controller, planner, recoveries, bt_navigator)
        #       All share nav2_multi_tb.yaml but get per-robot frame overrides.
        nav2_params = [
            nav2_yaml,
            {
                # bt_navigator frames
                'bt_navigator.ros__parameters.global_frame': f'{ns}/map',
                'bt_navigator.ros__parameters.robot_base_frame': base_link,
                # global costmap frames
                'global_costmap.global_costmap.ros__parameters.global_frame': f'{ns}/map',
                'global_costmap.global_costmap.ros__parameters.robot_base_frame': base_link,
                # local costmap frames
                'local_costmap.local_costmap.ros__parameters.robot_base_frame': base_link,
                # recoveries frames
                'behavior_server.ros__parameters.robot_base_frame': base_link,
            },
        ]

        nav2_remappings = [
            ('/scan',               f'/{ns}/scan'),
            ('/cmd_vel',            f'/{ns}/cmd_vel'),
            ('/map',                f'/{ns}/map'),
            ('/map_metadata',       f'/{ns}/map_metadata'),
            ('/odom',               f'/{ns}/odom'),
            ('/tf',                 '/tf'),
            ('/tf_static',          '/tf_static'),
        ]

        for pkg, executable in [
            ('nav2_controller',    'controller_server'),
            ('nav2_planner',       'planner_server'),
            ('nav2_behaviors',    'behavior_server'),
            ('nav2_bt_navigator',  'bt_navigator'),
        ]:
            nodes.append(Node(
                package=pkg,
                executable=executable,
                name=executable,
                namespace=ns,
                output='screen',
                parameters=nav2_params,
                remappings=nav2_remappings,
            ))

        # 10. Lifecycle manager for Nav2
        nodes.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=f'lifecycle_manager_nav2_{ns}',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ))

    # ── Shared: formation controller ──────────────────────────────────────
    nodes.append(Node(
        package=PACKAGE_NAME,
        executable='formation_controller',
        name='formation_controller',
        output='screen',
        parameters=[{
            'namespaces': namespaces,
            'formation_spacing': formation_spacing,
            'global_frame': global_frame,
            'goal_tolerance_xy': 0.15,
            'goal_timeout': 120.0,
            'publish_rate': 2.0,
        }],
    ))

    # ── Optional: RViz2 ───────────────────────────────────────────────────
    if rviz_flag and os.path.isfile(rviz_config):
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'LD_LIBRARY_PATH': _clean_ld_library_path()},
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'formation_spacing',
            default_value='0.8',
            description='Distance d (metres) between consecutive robots in the formation line',
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='world',
            description='Global TF root frame shared by all robots',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 for visualisation (true/false)',
        ),
        OpaqueFunction(function=_generate_nodes),
    ])
