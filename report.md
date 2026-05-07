# Multi-Robot Visualization — Progress Report

This document tracks what has been built, what's been verified on hardware, and what is pending. The project gives one RViz2 window for multiple Crazyflie drones and TurtleBot 4s, plus simple coordinated path scripts.

---

## Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1 | Crazyflie in RViz (single drone, hover hello-world) | **Partially done** |
| 2 | TurtleBots in RViz with 2D SLAM occupancy maps | Implemented, hardware test pending |
| 3 | Both Crazyflies + TurtleBots in one RViz | Implemented, hardware test pending |
| 4 | 3D map: Crazyflie Multi-Ranger PointCloud2 | **Implemented (Crazyflie side); ready to test** |
| 5 | 3D map: TurtleBot OAK-D depth PointCloud2 | Not started — needs onboard topic info |

---

## Milestone 1 — Crazyflie in RViz (partially done)

### What works
- `rviz/crazyflie.rviz` config: fixed frame `world`, Grid, TF, RobotModel for cfb3/cfb4 (subscribed to `/<cf>/robot_description`), Pose for cfb3/cfb4.
- `launch/crazyflie_viz.launch.py` brings up:
  - Upstream `crazyflie` package launch with `backend:=cflib`, `mocap:=False`, `teleop:=False`, `gui:=False`, `rviz:=False`.
  - Our `crazyflie_path_node` (hello-world: arm → takeoff 1.0 m → hover 5 s → land → disarm).
  - `rviz2` with the Phase 1 config.
- Verified on hardware (cfb4 only): drone took off, hovered, landed cleanly. Drone mesh + TF + pose visible in RViz throughout.

### Why "partially"
- Only `cfb4` is currently enabled in `crazyflies.yaml`. `cfb3` is disabled because that drone is not present today. Once a second drone is on the bench, flip its `enabled: true` and the same launch will fly both.
- The drone drift/crash on first flight was not a code bug — it was the **black floor**. Flow Deck v2 needs visual contrast on the floor (it's an optical-flow sensor, like a mouse). Fixed by laying a textured surface (newspaper / patterned mat) under the takeoff zone.
- The path node does *not* yet send `cmd_position` setpoints (the circle trajectory was removed because the Flow Deck couldn't handle aggressive position commands on the black floor). Re-introduce when there's a textured arena.

### Environment gotchas locked in
- **Run from a non-VSCode terminal.** VSCode is installed as a snap, and processes spawned from its integrated terminal inherit snap-confined env, which makes `rviz2` load the wrong `libpthread`. Use GNOME Terminal / xterm.
- **Backend = `cflib`** (Python). The cpp `crazyflie_server` binary in the Crazyflies workspace is linked against `libmotion_capture_tracking_interfaces*.so` which isn't built there.
- Apt installs needed (one-time): `ros-jazzy-motion-capture-tracking-interfaces`, `ros-jazzy-tf-transformations`.
- Our launch file does `SetEnvironmentVariable('PYTHONPATH', ...)` to expose `cflib` from `/home/drl/Desktop/Crazyflies/venv/lib/python3.12/site-packages` to system Python (the upstream `crazyflie_server.py` uses `#!/usr/bin/env python3`).
- Cfb4 URI fixed: was `radio://0/80/2M/E7E7E7B4` (4 bytes), now `radio://0/80/2M/E7E7E7E7B4` (5 bytes, matching the `cfb1/2/3/5` pattern).

---

## Milestone 2 — TurtleBots in RViz (implemented, not yet tested on hardware)

- `config/slam_toolbox.yaml`: async SLAM params (mapping mode, 0.05 m resolution, namespace-friendly).
- `launch/turtlebot_viz.launch.py`: per TurtleBot (tb0–tb4) starts a `static_transform_publisher` `world → tbN/map` and an `async_slam_toolbox_node` namespaced to `tbN` with frame names overridden to `tbN/odom`, `tbN/map`, `tbN/base_footprint`.
- `rviz/turtlebot.rviz`: per-robot RobotModel (subscribed to `/<ns>/robot_description`), color-coded LaserScan (`/<ns>/scan`), and 2D occupancy Map (`/<ns>/map`).
- Apt prereq: `ros-jazzy-slam-toolbox` (already on the system per the earlier install).
- Pending: actually drive the TurtleBots and confirm scans + maps build in RViz.

---

## Milestone 3 — Combined Crazyflie + TurtleBot RViz (implemented, not yet tested)

- `rviz/multi_robot.rviz`: every display from milestones 1 and 2.
- `launch/unified_multi_robot.launch.py`: includes the Crazyflie bringup (cflib, mocap off), the path nodes, all TurtleBot static TF + SLAM, the new pointcloud node, and the shared RViz.

---

## Milestone 4 — Crazyflie 3D Multi-Ranger PointCloud (just implemented)

### What was added
- `config/crazyflies.yaml`: enabled `firmware_logging.default_topics.scan` (10 Hz). The crazyflie ROS2 server now publishes `/<cf>/scan` as a `sensor_msgs/LaserScan` with the four horizontal Multi-Ranger rays (back / right / front / left).
- `multi_robot/multi_ranger_pointcloud_node.py`: new node that, for each enabled Crazyflie, subscribes to `/<cf>/scan`, looks up `world ← <cf>` TF at the scan timestamp, projects each valid ray to a 3D world-frame point, and publishes a running `sensor_msgs/PointCloud2` on `/<cf>/pointcloud`. Caps at 30 000 points.
- `setup.py`: registered the new `multi_ranger_pointcloud_node` console script.
- `package.xml`: added `sensor_msgs`, `tf2_ros`, `tf_transformations` dependencies.
- `crazyflie_viz.launch.py` and `unified_multi_robot.launch.py`: launch the new node alongside the path node.
- `rviz/crazyflie.rviz` and `rviz/multi_robot.rviz`: added two `PointCloud2` displays (`cfb3 Cloud`, `cfb4 Cloud`) colored by Z-axis using AxisColor / rainbow.

### How to verify
1. Lay newspaper / patterned mat under the takeoff zone (Flow Deck still applies).
2. Power-cycle cfb4. Place flat. Plug in Crazyradio dongle.
3. From a non-VSCode terminal:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/drl/Desktop/Crazyflies/ros2_ws/install/setup.bash
   source /home/drl/multi_robot_RViz/multi_robot_visualization/multi_robot_ws/install/setup.bash
   ros2 launch multi_robot crazyflie_viz.launch.py
   ```
4. Sanity check from a second terminal:
   ```bash
   ros2 topic hz /cfb4/scan          # should be ~10 Hz
   ros2 topic hz /cfb4/pointcloud    # should match scan rate
   ros2 topic echo /cfb4/scan --once # should show ~4 ranges
   ```
5. In RViz the **`cfb4 Cloud`** display under Displays should populate with rainbow-colored points anchored in the `world` frame, growing as the drone moves.

### Limitations / next steps
- Only the four horizontal rays are used. The "up" ranger publishes via a different (non-LaserScan) message and is not yet wired in. Not critical for ground-level mapping.
- Points accumulate without voxelization → cap is 30 000 oldest-out. If we need denser long flights, switch to a `nav2_voxel_layer` or Octomap.

---

## Milestone 5 — TurtleBot 3D PointCloud (not started)

The TurtleBot 4 RPLIDAR is 2D, so its laser scans give only one Z-slice. To get a real 3D map of what each TurtleBot sees, we should ingest the **OAK-D depth camera** that's on the TurtleBots.

**Need from the user before we can wire this up:**
- Is the OAK-D running on each TurtleBot (i.e., is `depthai` enabled in their onboard launch)?
- What topic name does each TurtleBot publish its depth pointcloud on? Likely something like `/<ns>/oakd/points`, `/<ns>/oakd/stereo/points`, or `/<ns>/oakd/rgb/preview/depth/points`. Confirm by running `ros2 topic list | grep -E "tb[0-4].*(point|depth)"` while the TurtleBots are on.
- If OAK-D isn't running, we have two fallbacks: (a) accumulate the 2D LIDAR scans into a PointCloud2 over time as the robot moves (flat slice, but each map row drawn as it drives — cheap approach), or (b) start the OAK-D stack on the TurtleBots.

Once the topic name is confirmed, the change is purely RViz-side: add per-TurtleBot `PointCloud2` displays subscribed to those topics. No new node needed unless we go with the LIDAR-accumulator fallback (in which case I'd add a `tb_lidar_pointcloud_node.py` analogous to the Crazyflie one).

---

## Files of interest

| File | Purpose |
|------|---------|
| `multi_robot_ws/src/multi_robot/launch/crazyflie_viz.launch.py` | Phase 1 launch — single drone + RViz |
| `multi_robot_ws/src/multi_robot/launch/turtlebot_viz.launch.py` | Phase 2 launch — TurtleBots + SLAM + RViz |
| `multi_robot_ws/src/multi_robot/launch/unified_multi_robot.launch.py` | Phase 3 launch — everything in one RViz |
| `multi_robot_ws/src/multi_robot/multi_robot/crazyflie_path_node.py` | Hello-world arm/takeoff/hover/land |
| `multi_robot_ws/src/multi_robot/multi_robot/multi_ranger_pointcloud_node.py` | LaserScan + TF → accumulating 3D PointCloud2 |
| `multi_robot_ws/src/multi_robot/multi_robot/turtlebot_path_node.py` | TurtleBot velocity path |
| `multi_robot_ws/src/multi_robot/config/crazyflies.yaml` | CF URIs, enabled flags, firmware_logging topics |
| `multi_robot_ws/src/multi_robot/config/slam_toolbox.yaml` | Async SLAM defaults |
| `multi_robot_ws/src/multi_robot/rviz/crazyflie.rviz` | Phase 1 RViz config |
| `multi_robot_ws/src/multi_robot/rviz/turtlebot.rviz` | Phase 2 RViz config |
| `multi_robot_ws/src/multi_robot/rviz/multi_robot.rviz` | Phase 3 RViz config |
