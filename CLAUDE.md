# CLAUDE.md — Multi-Robot Visualization Project

This file gives Claude Code the context it needs to continue work across machines and sessions.

---

## What this project is

A ROS2 (Jazzy) workspace for coordinating and **visualizing** multiple robots simultaneously in RViz2:
- **Crazyflie drones** (brushless) — each carries a **Flow Deck v2** (bottom, for optical-flow velocity + Z height) and a **Multi-Ranger Deck** (top, for 5-direction distance sensing)
- **TurtleBot 4** robots — use their onboard **RPLIDAR A1** (2D LiDAR) and **OAK-D** depth camera

ROS2 distribution: **Jazzy** on **Ubuntu** (lab computer). Development also happens on macOS.

---

## Repository layout

```
multi_robot_visualization/
├── CLAUDE.md                        ← this file
├── README.md
├── requirements.txt
└── multi_robot_ws/
    └── src/
        └── multi_robot/
            ├── config/
            │   ├── crazyflies.yaml          # Robot definitions, URIs, initial positions
            │   └── motion_capture.yaml      # OptiTrack config (currently inactive — tracking: "none")
            ├── launch/
            │   └── unified_multi_robot.launch.py   # Main entry point
            └── multi_robot/
                ├── crazyflie_path_node.py   # Arms, takeoff, circular flight, land
                └── turtlebot_path_node.py   # Publishes cmd_vel rectangular path
```

---

## Active robots (from `crazyflies.yaml`)

| Robot | Type | Status |
|-------|------|--------|
| cfb3 | Crazyflie (cf21) | enabled |
| cfb4 | Crazyflie (cf21) | enabled |
| cfb1, cfb2, cfb5 | Crazyflie | disabled |
| tb0–tb4 | TurtleBot 4 | configured in turtlebot_path_node |

TF root frame: **`world`**

---

## Localization approach

- **Crazyflies**: sensor-only, no OptiTrack. Flow Deck v2 + onboard Kalman filter estimates position. `crazyflie_ros2` publishes TF (`world → cfb3`, etc.) and pose topics.
- **TurtleBots**: onboard odometry + slam_toolbox (running on lab computer). TF chain: `world → tb{N}/map → tb{N}/odom → tb{N}/base_footprint` (connected via static TF at known starting position).

---

## Planned visualization work (in progress)

The next implementation task is adding **RViz2 visualization + SLAM** to the existing launch. This was planned on 2026-05-05. The plan file lives at `~/.claude/plans/i-have-turtlebots-and-kind-moore.md` on the Mac, and is summarized here.

### Files to create / modify

1. **`multi_robot_ws/src/multi_robot/config/crazyflies.yaml`** — Add `custom_topics` under `firmware_logging` to enable Multi-Ranger range data publishing:
   ```yaml
   custom_topics:
     range:
       frequency: 10
       vars: ["range.front", "range.back", "range.left", "range.right", "range.up"]
   ```
   This causes `crazyflie_ros2` to publish `/{cf_name}/scan` as a `sensor_msgs/LaserScan`.

2. **`multi_robot_ws/src/multi_robot/config/rviz_config.rviz`** — New RViz2 config with:
   - Fixed frame: `world`
   - Grid display
   - TF display
   - LaserScan per TurtleBot (`tb0/scan` … `tb4/scan`)
   - LaserScan per Crazyflie (`cfb3/scan`, `cfb4/scan`)
   - Map display for slam_toolbox output
   - RobotModel displays

3. **`multi_robot_ws/src/multi_robot/config/slam_toolbox.yaml`** — Async SLAM params, namespaced frame names, scan topic remapped per TB namespace.

4. **`multi_robot_ws/src/multi_robot/launch/unified_multi_robot.launch.py`** — Add:
   - Static TF publishers: `world → tb{N}/map` at each robot's known starting position
   - One `slam_toolbox async_slam_toolbox_node` per active TurtleBot (namespaced)
   - `rviz2` node pointing to `rviz_config.rviz`

5. **`multi_robot_ws/src/multi_robot/package.xml`** — Add `exec_depend` for: `slam_toolbox`, `rviz2`, `tf2_ros`

### Lab computer apt installs needed
```bash
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-tf2-ros
sudo apt install ros-jazzy-turtlebot4-description   # for TurtleBot URDF
# rviz2 is usually pre-installed with ros-jazzy-desktop
```

### Verification sequence
1. `cd multi_robot_ws && colcon build --symlink-install && source install/setup.bash`
2. `ros2 launch multi_robot unified_multi_robot.launch.py` → RViz2 opens
3. `ros2 topic list` → confirm `/{cf}/scan` and `/{tb}/scan` exist
4. `ros2 run tf2_tools view_frames` → confirm all robots connect to `world`
5. Drive a TurtleBot → laser scan ring moves in RViz
6. Fly a Crazyflie → 3D position marker moves in RViz, Multi-Ranger rays visible

---

## What is NOT in scope (yet)

- **3D SLAM for Crazyflies**: Multi-Ranger has only 5 rays — not enough for map building, only for real-time obstacle display
- **Multi-robot map merging** across TurtleBots (`multirobot_map_merge` is a future step)
- **OAK-D depth camera point cloud** in RViz (future enhancement)
- **Isaac Sim**: decided against — RViz2 is sufficient, no GPU workstation required

---

## Key external packages (not in this repo)

- `crazyflie_ros2` — Bitcraze ROS2 driver, handles Crazyflie radio comms, TF, and sensor topics
- `motion_capture_tracking` — OptiTrack integration (currently unused; tracking set to `"none"`)
- `turtlebot4` — TurtleBot 4 driver stack, runs on the robot's onboard RPi4 and broadcasts topics over DDS
