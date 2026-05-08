# nav2_multi_tb

**ROS 2 Jazzy** package for driving multiple TurtleBot 4s (tb0–tb4) in a
synchronized **line formation** under Nav2.

---

## Formation concept

All N enabled robots form a straight line spaced `d` metres apart along the
formation heading θ.  A single operator goal `(x, y, θ)` is decomposed into
per-robot sub-goals:

```
robot i → x_i = x_goal - i·d·cos(θ)
          y_i = y_goal - i·d·sin(θ)
          θ_i = θ
```

Robot 0 (`tb0` by default, `formation_slot: 0` in `turtlebots.yaml`) is the
**leader** and lands at the goal pose.  Higher-indexed robots slot in behind it.

```
goal → [tb0] ←d→ [tb1] ←d→ [tb2] ←d→ …
```

Each robot runs its **own Nav2 stack** (SLAM + DWB planner + BT navigator)
and navigates to its sub-goal independently.  The `formation_controller_node`
sends all goals concurrently so robots move simultaneously.

---

## Package layout

```
nav2_multi_tb/
├── nav2_multi_tb/
│   ├── formation_controller_node.py   ← core: decomposes goal → per-robot Nav2 actions
│   └── formation_goal_publisher.py    ← CLI helper: publish a formation goal
├── launch/
│   └── nav2_multi_tb.launch.py        ← main entry point
├── config/
│   ├── turtlebots.yaml                ← which robots, positions, formation slots
│   ├── nav2_multi_tb.yaml             ← shared Nav2 params (DWB, costmaps, planners)
│   └── slam_toolbox.yaml              ← async SLAM params
├── rviz/
│   └── nav2_multi_tb.rviz             ← pre-configured RViz2 layout
├── package.xml
└── setup.py
```

---

## Prerequisites

This package lives alongside the `multi_robot` package and reuses its
`turtlebot_odom_tf_node` executable.  Both packages must be in the same
workspace.

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-recoveries \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-slam-toolbox \
  ros-jazzy-tf2-ros \
  ros-jazzy-topic-tools \
  ros-jazzy-dwb-core \
  ros-jazzy-rviz2
```

---

## Build

Place the package at `multi_robot_ws/src/nav2_multi_tb/` alongside `multi_robot/`:

```bash
cd multi_robot_ws
colcon build --symlink-install --packages-select multi_robot nav2_multi_tb
source install/setup.bash
```

---

## Configure

### 1. Enable robots — `config/turtlebots.yaml`

```yaml
robots:
  tb0:
    enabled: true
    initial_position: [0.0, 0.0, 0.0]   # x (m), y (m), yaw (deg) at startup
    formation_slot: 0                     # 0 = leader, 1 = first follower, …

  tb1:
    enabled: true
    initial_position: [-0.8, 0.0, 0.0]
    formation_slot: 1

  tb2:
    enabled: true
    initial_position: [-1.6, 0.0, 0.0]
    formation_slot: 2
```

`initial_position` is used for the `world → {ns}/map` static TF.  Set it to
each robot's physical location at launch time.

### 2. Formation spacing

Either edit the default in the launch argument or pass it at runtime:

```bash
ros2 launch nav2_multi_tb nav2_multi_tb.launch.py formation_spacing:=1.0
```

---

## Run

```bash
# Source all three workspaces
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash          # crazyflie_ros2 if needed
source ~/multi_robot_ws/install/setup.bash

# Launch the formation system
ros2 launch nav2_multi_tb nav2_multi_tb.launch.py

# Or with custom spacing, no RViz
ros2 launch nav2_multi_tb nav2_multi_tb.launch.py formation_spacing:=1.2 rviz:=false
```

### Send a formation goal

In a second terminal:

```bash
# Drive all robots to (3.0, 0.0) facing +X, spaced 0.8 m apart
ros2 run nav2_multi_tb formation_goal_publisher \
    --ros-args -p x:=3.0 -p y:=0.0 -p yaw_deg:=0.0

# Formation facing 90° (pointing +Y)
ros2 run nav2_multi_tb formation_goal_publisher \
    --ros-args -p x:=0.0 -p y:=3.0 -p yaw_deg:=90.0
```

You can also publish directly:

```bash
ros2 topic pub --once /formation/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}'
```

### Monitor formation status

```bash
ros2 topic echo /formation/status
```

### Visualise sub-goal arrows in RViz

Subscribe to `/formation/goal_markers` (MarkerArray).  Green arrow = leader
(tb0), blue arrows = followers.  Yellow lines connect consecutive slots.

---

## TF tree (per robot, e.g. tb0)

```
world
└── tb0/map              ← static TF at initial_position
    └── odom             ← slam_toolbox (map→odom correction, 50 Hz)
        └── tb0/base_link ← turtlebot_odom_tf_node (from /tb0/odom, 62 Hz)
            └── tb0/rplidar (or tb0/laser_frame)  ← via TF relay
```

---

## Key topics

| Topic | Type | Direction |
|---|---|---|
| `/formation/goal` | `geometry_msgs/PoseStamped` | Operator → controller |
| `/formation/status` | `std_msgs/String` | Controller → operator |
| `/formation/goal_markers` | `visualization_msgs/MarkerArray` | Controller → RViz |
| `/{ns}/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Controller → Nav2 |
| `/{ns}/odom` | `nav_msgs/Odometry` | TurtleBot → controller |
| `/{ns}/scan` | `sensor_msgs/LaserScan` | TurtleBot → SLAM / costmap |
| `/{ns}/map` | `nav_msgs/OccupancyGrid` | slam_toolbox → Nav2 / RViz |
| `/{ns}/cmd_vel` | `geometry_msgs/Twist` | Nav2 → TurtleBot |

---

## Teleop (manual override, one robot at a time)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  --remap cmd_vel:=/tb0/cmd_vel_unstamped \
  -p qos_overrides./tb0/cmd_vel_unstamped.publisher.reliability:=best_effort
```

---

## Diagnostics

```bash
# Confirm all robots are in the TF tree
ros2 run tf2_tools view_frames

# Confirm navigate_to_pose action servers are up
ros2 action list | grep navigate_to_pose

# Check Nav2 lifecycle state
ros2 lifecycle list /tb0/bt_navigator

# Inspect formation controller parameters
ros2 param list /formation_controller
```

---

## Future work

- **Map merging**: `multirobot_map_merge` to fuse per-robot SLAM maps into a
  shared global map for a truly shared global costmap.
- **Formation shapes**: Triangle, V, column — change sub-goal geometry in
  `formation_controller_node.py`.
- **Collision avoidance between robots**: Add each robot's footprint as an
  obstacle source in the other robots' costmaps.
- **Dynamic `d`**: Allow `formation_spacing` to change at runtime via a topic
  or service.
