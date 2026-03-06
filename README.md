# multi_robot

ROS 2 workspace for running **coordinated Crazyflie + TurtleBot path scripts** from one launch file.

This repository contains a ROS 2 Python package (`multi_robot`) inside `multi_robot_ws/src/` with:

- `crazyflie_path_node`: arms, takes off, flies enabled Crazyflies in a circle, then lands/disarms.
- `turtlebot_path_node`: publishes repeated timed `cmd_vel` commands to multiple TurtleBot namespaces.
- `unified_multi_robot.launch.py`: starts Crazyflie bringup and both path nodes together.

---

## Repository layout

- `requirements.txt` – pinned Python environment snapshot used in development.
- `multi_robot_ws/src/multi_robot/` – ROS 2 package source.
  - `launch/unified_multi_robot.launch.py` – unified launch entry point.
  - `config/crazyflies.yaml` – Crazyflie robot definitions, enabled flags, URIs, initial positions.
  - `config/motion_capture.yaml` – motion capture settings used by Crazyflie bringup.
  - `multi_robot/crazyflie_path_node.py` – Crazyflie circular mission script.
  - `multi_robot/turtlebot_path_node.py` – TurtleBot velocity path script.

---

## Dependencies

## 1) System dependencies

You need a ROS 2 distribution with `ament_python` support and the standard build tools.

Recommended (based on this repo’s Python/tooling):

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12

Install base tools:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip
```

Initialize rosdep (first machine setup only):

```bash
sudo rosdep init
rosdep update
```

## 2) ROS package dependencies

From `multi_robot/package.xml`, this package depends on:

- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `ament_index_python`
- `crazyflie_interfaces`
- `crazyflie_py`

Install all resolvable ROS dependencies from the workspace root:

```bash
cd /workspace/multi_robot/multi_robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

> Note: `crazyflie_interfaces` and `crazyflie_py` are provided by the Crazyflie ROS 2 stack. If rosdep cannot resolve them automatically in your environment, install/overlay the Crazyflie ROS packages before building.

## 3) Python dependencies

This repo includes a pinned `requirements.txt` snapshot. Create a virtual environment and install it:

```bash
cd /workspace/multi_robot
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

For ROS 2 execution, many environments run best with system ROS Python + optional venv packages layered on top. If you see import/path issues, try building/running without activating the venv and rely on apt-installed ROS dependencies.

---

## Build and setup

```bash
cd /workspace/multi_robot/multi_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you open a new terminal, re-source both ROS and workspace overlays:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/multi_robot/multi_robot_ws/install/setup.bash
```

---

## Configure robots before running

## Crazyflies (`config/crazyflies.yaml`)

For each robot entry under `robots:`:

- Set `enabled: true/false`
- Set the `uri` to match each drone radio address
- Set `initial_position` (used as the center offset for the circle trajectory)

Example entries in this repo include `cfb1`, `cfb3`, `cfb4` enabled and `cfb2` disabled by default.

## Motion capture (`config/motion_capture.yaml`)

Set:

- `type` (e.g., optitrack/vicon/etc.)
- `hostname` for your mocap server
- marker and dynamics configs appropriate to your rigid body setup

---

## Running scripts

## Option A: Run everything from one launch file (recommended)

```bash
cd /workspace/multi_robot/multi_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch multi_robot unified_multi_robot.launch.py
```

This launches:

1. Crazyflie bringup launch from the `crazyflie` package.
2. `crazyflie_path_node` (circle mission).
3. `turtlebot_path_node` (repeating rectangular-like motion profile).

## Option B: Run nodes separately

Terminal 1 (Crazyflie path):

```bash
cd /workspace/multi_robot/multi_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run multi_robot crazyflie_path_node
```

Terminal 2 (TurtleBot path):

```bash
cd /workspace/multi_robot/multi_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run multi_robot turtlebot_path_node
```

---

## Useful runtime parameters

## `crazyflie_path_node`

You can override parameters at runtime, e.g.:

```bash
ros2 run multi_robot crazyflie_path_node --ros-args \
  -p takeoff_height:=1.0 \
  -p takeoff_duration:=2.5 \
  -p circle_radius:=0.4 \
  -p circle_period:=8.0 \
  -p circle_duration:=20.0 \
  -p publish_rate_hz:=50.0 \
  -p land_height:=0.04 \
  -p land_duration:=2.5 \
  -p frame_id:=world
```

## `turtlebot_path_node`

```bash
ros2 run multi_robot turtlebot_path_node --ros-args \
  -p turtlebot_namespaces:="['tb4_1','tb4_2']" \
  -p timer_period:=0.1
```

---

## Quick verification checks

After launching, verify expected topics/services:

```bash
ros2 node list
ros2 topic list | rg cmd_vel
ros2 topic list | rg cmd_position
ros2 service list | rg '/arm|/takeoff|/land'
```

---

## Troubleshooting

- **`Package 'multi_robot' not found`**
  - Rebuild and source the workspace:
    - `colcon build --symlink-install`
    - `source install/setup.bash`

- **`crazyflie` launch package not found**
  - Install/source the Crazyflie ROS 2 stack (`crazyflie`, `crazyflie_interfaces`, `crazyflie_py`) in your environment.

- **No motion from TurtleBots**
  - Ensure namespaces in `turtlebot_namespaces` match actual robot namespaces.
  - Confirm robots subscribe to `/<namespace>/cmd_vel`.

- **Crazyflies do not arm/takeoff**
  - Check radio URIs in `crazyflies.yaml`.
  - Check enabled flags.
  - Confirm required services are available (`/arm`, `/takeoff`, `/land`, `/notify_setpoints_stop`).
  - Confirm mocap setup and frame alignment when external localization is required.

---

## Development notes

- Existing `multi_robot_ws/build` and `multi_robot_ws/install` directories are generated artifacts. If they become stale, remove and rebuild:

```bash
cd /workspace/multi_robot/multi_robot_ws
rm -rf build install log
colcon build --symlink-install
```
