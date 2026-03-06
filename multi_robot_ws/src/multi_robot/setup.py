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
        ('share/' + package_name + '/launch', ['launch/launch.py', 'launch/unified_multi_robot.launch.py']),
        ('share/' + package_name + '/config', ['config/crazyflies.yaml', 'config/motion_capture.yaml']),
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
        ],
    },
)
