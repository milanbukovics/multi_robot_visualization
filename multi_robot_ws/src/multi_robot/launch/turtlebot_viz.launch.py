import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_NAME = 'multi_robot'
TURTLEBOT_NAMESPACES = ['tb0', 'tb1', 'tb2', 'tb3', 'tb4']


def _clean_ld_library_path():
    raw = os.environ.get('LD_LIBRARY_PATH', '')
    parts = [p for p in raw.split(':') if p and not p.startswith('/snap/')]
    return ':'.join(parts)


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)

    slam_toolbox_yaml_path = os.path.join(package_share, 'config', 'slam_toolbox.yaml')
    rviz_config_path = os.path.join(package_share, 'rviz', 'turtlebot.rviz')

    nodes = []

    # Velocity command publisher for TurtleBots
    nodes.append(Node(
        package=PACKAGE_NAME,
        executable='turtlebot_path_node',
        name='turtlebot_path_node',
        output='screen',
    ))

    for ns in TURTLEBOT_NAMESPACES:
        # Static TF: world -> {ns}/map at origin (adjust translations for actual positions)
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{ns}_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', f'{ns}/map'],
        ))

        # Async SLAM for each TurtleBot
        nodes.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[
                slam_toolbox_yaml_path,
                {
                    'odom_frame': f'{ns}/odom',
                    'map_frame': f'{ns}/map',
                    'base_frame': f'{ns}/base_footprint',
                    'scan_topic': f'/{ns}/scan',
                },
            ],
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
