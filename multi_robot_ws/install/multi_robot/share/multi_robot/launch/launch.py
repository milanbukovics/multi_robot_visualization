import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


PACKAGE_NAME = 'multi_robot'


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)

    crazyflies_yaml_path = os.path.join(package_share, 'config', 'crazyflies.yaml')
    motion_capture_yaml_path = os.path.join(package_share, 'config', 'motion_capture.yaml')

    crazyflie_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('crazyflie'), 'launch', 'launch.py')]
        ),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml_path,
            'motion_capture_yaml_file': motion_capture_yaml_path,
        }.items(),
    )

    crazyflie_path_node = Node(
        package=PACKAGE_NAME,
        executable='crazyflie_path_node',
        name='crazyflie_path_node',
        output='screen',
        parameters=[{'crazyflies_yaml_file': crazyflies_yaml_path}],
    )

    turtlebot_path_node = Node(
        package=PACKAGE_NAME,
        executable='turtlebot_path_node',
        name='turtlebot_path_node',
        output='screen',
    )

    return LaunchDescription([
        crazyflie_bringup,
        crazyflie_path_node,
        turtlebot_path_node,
    ])
