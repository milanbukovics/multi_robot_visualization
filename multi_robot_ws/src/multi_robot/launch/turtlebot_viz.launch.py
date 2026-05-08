import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_NAME = 'multi_robot'


def _clean_ld_library_path():
    raw = os.environ.get('LD_LIBRARY_PATH', '')
    parts = [p for p in raw.split(':') if p and not p.startswith('/snap/')]
    return ':'.join(parts)


def _load_enabled_turtlebots(package_share):
    yaml_path = os.path.join(package_share, 'config', 'turtlebots.yaml')
    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return [n for n, c in cfg['robots'].items() if c.get('enabled', False)]


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)

    slam_toolbox_yaml_path = os.path.join(package_share, 'config', 'slam_toolbox.yaml')
    rviz_config_path = os.path.join(package_share, 'rviz', 'turtlebot.rviz')

    enabled = _load_enabled_turtlebots(package_share)

    nodes = []

    for ns in enabled:
        # Static TF: world -> {ns}/map at origin (adjust translations for actual positions)
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{ns}_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', f'{ns}/map'],
        ))

        nodes.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[
                slam_toolbox_yaml_path,
                {
                    'odom_frame': 'odom',
                    'map_frame': f'{ns}/map',
                    'base_frame': 'base_link',
                },
            ],
            remappings=[('/scan', f'/{ns}/scan')],
        ))

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

        # Convert /{ns}/odom nav_msgs/Odometry -> odom->base_link TF.
        # The Create3 base publishes odometry as a topic but not as TF, so this
        # node fills the gap.  TransformBroadcaster uses absolute /tf, so the
        # transform lands in global TF regardless of node namespace.
        nodes.append(Node(
            package=PACKAGE_NAME,
            executable='turtlebot_odom_tf_node',
            name=f'turtlebot_odom_tf_node_{ns}',
            remappings=[('odom', f'/{ns}/odom')],
        ))

        # Bridge TB onboard's namespaced TF graph to global so the cloud node
        # and RViz (which read global /tf, /tf_static) can see the full chain.
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'tf_relay_{ns}',
            arguments=[f'/{ns}/tf', '/tf'],
        ))
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'tf_static_relay_{ns}',
            arguments=[f'/{ns}/tf_static', '/tf_static'],
        ))

        # LIDAR -> world-frame accumulated PointCloud2 (global namespace, explicit remaps)
        nodes.append(Node(
            package=PACKAGE_NAME,
            executable='turtlebot_lidar_pointcloud_node',
            name=f'turtlebot_lidar_pointcloud_node_{ns}',
            output='screen',
            remappings=[
                ('scan', f'/{ns}/scan'),
                ('pointcloud', f'/{ns}/pointcloud'),
            ],
            parameters=[{
                'world_frame': 'world',
                'max_points': 100000,
                'range_min': 0.05,
            }],
        ))

    # RViz
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        additional_env={'LD_LIBRARY_PATH': _clean_ld_library_path()},
    ))

    return LaunchDescription(nodes)
