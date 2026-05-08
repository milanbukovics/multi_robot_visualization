from setuptools import find_packages, setup
import glob

package_name = 'nav2_multi_tb'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # launch files
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch.py')),
        # config files
        ('share/' + package_name + '/config',
            glob.glob('config/*.yaml')),
        # RViz config (if present)
        ('share/' + package_name + '/rviz',
            glob.glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='drl',
    maintainer_email='cmilanes@hawaii.edu',
    description='Multi-TurtleBot Nav2 formation controller',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # Core formation controller
            'formation_controller = nav2_multi_tb.formation_controller_node:main',
            # CLI helper to publish a goal from the terminal
            'formation_goal_publisher = nav2_multi_tb.formation_goal_publisher:main',
        ],
    },
)
