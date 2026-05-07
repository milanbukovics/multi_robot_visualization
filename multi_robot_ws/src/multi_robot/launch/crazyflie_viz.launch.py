import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


PACKAGE_NAME = 'multi_robot'
CFLIB_VENV_SITE_PACKAGES = '/home/drl/Desktop/Crazyflies/venv/lib/python3.12/site-packages'


def _clean_ld_library_path():
    raw = os.environ.get('LD_LIBRARY_PATH', '')
    parts = [p for p in raw.split(':') if p and not p.startswith('/snap/')]
    return ':'.join(parts)


def _pythonpath_with_venv():
    existing = os.environ.get('PYTHONPATH', '')
    return f'{CFLIB_VENV_SITE_PACKAGES}:{existing}' if existing else CFLIB_VENV_SITE_PACKAGES


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)

    crazyflies_yaml_path = os.path.join(package_share, 'config', 'crazyflies.yaml')
    motion_capture_yaml_path = os.path.join(package_share, 'config', 'motion_capture.yaml')
    rviz_config_path = os.path.join(package_share, 'rviz', 'crazyflie.rviz')

    crazyflie_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('crazyflie'), 'launch', 'launch.py')]
        ),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml_path,
            'motion_capture_yaml_file': motion_capture_yaml_path,
            'backend': 'cflib',
            'mocap': 'False',
            'teleop': 'False',
            'gui': 'False',
            'rviz': 'False',
        }.items(),
    )

    crazyflie_path_node = Node(
        package=PACKAGE_NAME,
        executable='crazyflie_path_node',
        name='crazyflie_path_node',
        output='screen',
        parameters=[{'crazyflies_yaml_file': crazyflies_yaml_path}],
    )

    pointcloud_node = Node(
        package=PACKAGE_NAME,
        executable='multi_ranger_pointcloud_node',
        name='multi_ranger_pointcloud_node',
        output='screen',
        parameters=[{'crazyflies_yaml_file': crazyflies_yaml_path}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        additional_env={'LD_LIBRARY_PATH': _clean_ld_library_path()},
    )

    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', _pythonpath_with_venv()),
        crazyflie_bringup,
        crazyflie_path_node,
        pointcloud_node,
        rviz_node,
    ])
