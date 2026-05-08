from setuptools import find_packages, setup

package_name = 'multi_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/launch.py',
            'launch/unified_multi_robot.launch.py',
            'launch/crazyflie_viz.launch.py',
            'launch/turtlebot_viz.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/crazyflies.yaml',
            'config/motion_capture.yaml',
            'config/slam_toolbox.yaml',
            'config/turtlebots.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/crazyflie.rviz',
            'rviz/turtlebot.rviz',
            'rviz/multi_robot.rviz',
        ]),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='drl',
    maintainer_email='cmilanes@hawaii.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'crazyflie_path_node = multi_robot.crazyflie_path_node:main',
            'turtlebot_path_node = multi_robot.turtlebot_path_node:main',
            'multi_ranger_pointcloud_node = multi_robot.multi_ranger_pointcloud_node:main',
            'turtlebot_lidar_pointcloud_node = multi_robot.turtlebot_lidar_pointcloud_node:main',
            'turtlebot_odom_tf_node = multi_robot.turtlebot_odom_tf_node:main',
        ],
    },
)
