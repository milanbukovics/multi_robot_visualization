# Crazyfly commands

source /opt/ros/jazzy/setup.bash
source /home/drl/Desktop/Crazyflies/ros2_ws/install/setup.bash
source /home/drl/multi_robot_RViz/multi_robot_visualization/multi_robot_ws/install/setup.bash
ros2 launch multi_robot crazyflie_viz.launch.py

# Turtlebots commands: 

cd /home/drl/multi_robot_RViz/multi_robot_visualization/multi_robot_ws
colcon build --symlink-install --packages-select multi_robot
source install/setup.bash
ros2 launch multi_robot turtlebot_viz.launch.py