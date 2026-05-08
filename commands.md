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

## teleop control 
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  --remap cmd_vel:=/tb2/cmd_vel_unstamped \
  -p qos_overrides./tb2/cmd_vel_unstamped.publisher.reliability:=best_effort


# Both Turtlebots and Crazyflies


source /opt/ros/jazzy/setup.bash
source /home/drl/Desktop/Crazyflies/ros2_ws/install/setup.bash
source /home/drl/multi_robot_RViz/multi_robot_visualization/multi_robot_ws/install/setup.bash
ros2 launch multi_robot unified_multi_robot.launch.py

## Turtlebot control

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/tb2/cmd_vel_unstamped \
  -p qos_overrides./tb2/cmd_vel_unstamped.publisher.reliability:=best_effort

# Kill all previous ROS2 sessions with Ctrl + C

rm -rf /dev/shm/fastrtps_*

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  --remap cmd_vel:=/tb3/cmd_vel_unstamped \
  -p "qos_overrides./tb3/cmd_vel_unstamped.publisher.reliability:=best_effort"
