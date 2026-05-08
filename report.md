# Multi-Robot Real-Time Visualization System Using ROS 2

**Author**: Milan Bukovics  
**Institution**: University of Hawaiʻi at Mānoa — Dynamic Robotics Lab  
**Date**: May 2026

---

## Abstract

This report documents the design, implementation, and integration challenges of a real-time multi-robot visualization system built on ROS 2 Jazzy. The system combines two classes of physical robots — Bitcraze Crazyflie 2.1 micro-quadrotors and iRobot Create3-based TurtleBot 4 ground vehicles — within a single shared 3D environment visualized in RViz2. Each robot contributes sensor data to a common world frame: Crazyflie units perform autonomous flight sequences and build a 3D point cloud from a five-ray Time-of-Flight deck, while TurtleBot units run Simultaneous Localization and Mapping (SLAM) using a 360° planar LIDAR and accumulate a world-frame 3D point cloud in parallel. The result is a unified, real-time occupancy view of heterogeneous aerial and ground robots operating together in the same physical space. Eight significant integration challenges were encountered and resolved during development; these are documented in detail as the primary technical contribution of this report.

---

## 1. Introduction

Multi-robot systems that mix aerial and ground platforms are increasingly common in research environments, but real-time unified visualization of such systems remains a non-trivial engineering problem. Each robot type brings its own communication protocol, sensor modality, coordinate frame convention, and ROS 2 middleware assumptions. Getting them to share a coherent world model — one that RViz2 can render as a single consistent scene — requires carefully bridging each of those gaps.

This project targets a lab environment where two Crazyflie 2.1 micro-UAVs (equipped with Flow Deck v2 and Multi-Ranger Deck) and one or more TurtleBot 4 ground robots (equipped with RPLIDAR A1) must be visualized together. Neither robot type has access to an external positioning system (no OptiTrack, no Lighthouse), so localization is entirely sensor-driven: the Crazyflies use onboard Kalman filtering over optical flow and height data, while the TurtleBots use wheel odometry corrected by SLAM loop closure.

The goals of the project are:

1. Execute autonomous Crazyflie flight missions (arm → takeoff → hover → land) and visualize the drone's trajectory and environmental scans in RViz2.
2. Build a real-time SLAM occupancy grid map for each active TurtleBot and visualize it alongside the CF data.
3. Accumulate 3D point clouds from both robot types in a shared world frame so an operator can observe the physical environment from a single RViz2 window.

All software is implemented as a custom `multi_robot` ROS 2 Python package. The system runs on Ubuntu 24.04 with ROS 2 Jazzy (the late-2024 LTS release), Python 3.12, and the Bitcraze `cflib` Python library for Crazyflie communication.

---

## 2. System Architecture

### 2.1 Hardware Platform

**Crazyflie 2.1 (cfb4)**  
The Crazyflie 2.1 is a 27 g open-source quadrotor with a 92 mm motor-to-motor span. It communicates with the host computer via a Crazyradio PA USB dongle over a 2.4 GHz radio link (URI: `radio://0/80/2M/E7E7E7E7B4`).

Two expansion decks are mounted:

- **Flow Deck v2**: Combines a PAA3905 optical flow sensor (measures horizontal velocity from surface texture) and a VL53L1x Time-of-Flight sensor (measures height above ground). The onboard Extended Kalman Filter fuses these to maintain a position estimate without external infrastructure.
- **Multi-Ranger Deck**: Five VL53L1x ToF sensors pointing forward, backward, left, right, and up. Each sensor measures range up to approximately 4 m. The `crazyflie_ros2` driver publishes these five readings as a `sensor_msgs/LaserScan` message at 10 Hz to `/{cf}/scan`.

The Crazyflie firmware uses the Mellinger controller (parameter `stabilizer.controller: 2`) and Kalman filter estimator (`stabilizer.estimator: 2`). High-level flight commands (arm, takeoff, land) are executed via `crazyflie_interfaces` ROS 2 services.

**TurtleBot 4 (tb1)**  
The TurtleBot 4 is a differential-drive ground robot built on the iRobot Create3 mobile base, with a Raspberry Pi 4 as the compute platform. The Create3 base publishes wheel odometry to `/{tb}/odom` as `nav_msgs/Odometry` at approximately 62 Hz over a Wi-Fi DDS link.

The RPLIDAR A1 is a 360° planar LIDAR with a 12 m nominal range. It publishes `sensor_msgs/LaserScan` to `/{tb}/scan` at approximately 5.5 Hz. The sensor frame is published under `/{tb}/rplidar` or `/{tb}/laser_frame` depending on firmware version, connected to the robot body via a static TF from the TurtleBot driver stack.

**Visualization Host**  
A lab workstation running Ubuntu 24.04 serves as the ROS 2 master node and RViz2 host. It communicates with Crazyflies via USB Crazyradio and with TurtleBots via Wi-Fi (DDS peer discovery on the local network).

---

### 2.2 Software Architecture

The system is organized as a single ROS 2 Python package (`multi_robot`) containing five custom nodes, three launch files, and configuration YAML files for each robot type. Robot enablement is controlled at launch time by reading the YAML configs — no recompilation is needed to add or remove robots.

**Software stack**:
- ROS 2 Jazzy (Ubuntu 24.04 LTS)
- Python 3.12
- `crazyflie_ros2` — official Bitcraze ROS 2 driver (sourced from a separate workspace)
- `cflib` — Bitcraze Python library (isolated in a virtual environment at `/home/drl/Desktop/Crazyflies/venv/`)
- `slam_toolbox` — asynchronous SLAM for TurtleBot mapping
- `nav2_lifecycle_manager` — lifecycle management for slam_toolbox
- `topic_tools` — lightweight topic relay for TF bridging
- `rviz2` — 3D visualization

---

### 2.3 TF Frame Hierarchy

All robots share a single `world` frame as the global coordinate origin. The full TF tree for one active Crazyflie and one active TurtleBot is:

```
world
├── cfb4                   ← published by crazyflie_ros2 (Kalman-filtered pose)
│   └── cfb4/scan          ← Multi-Ranger sensor frame (from LaserScan header)
└── tb1/map                ← static TF at world origin (anchors SLAM map globally)
    └── tb1/odom           ← published by slam_toolbox (SLAM-corrected drift)
        └── base_link      ← published by turtlebot_odom_tf_node (from odometry)
            └── tb1/rplidar ← static TF from TurtleBot driver (sensor offset)
```

Without every link in this chain being present and publishing at a sufficient rate, the LIDAR accumulator and SLAM algorithms fail silently. Establishing and debugging this chain was the primary integration challenge of the project (see Section 4).

---

### 2.4 Data Flow

**Crazyflie data path**:
```
Crazyflie radio (cfb4)
  → crazyflie_ros2 driver
      → /{cfb4}/scan         (LaserScan, 5 rays, 10 Hz)
      → /{cfb4}/pose         (PoseStamped, 10 Hz)
      → TF: world → cfb4    (Kalman-filtered position, 10 Hz)
  → multi_ranger_pointcloud_node
      → /{cfb4}/pointcloud   (PointCloud2, accumulated 3D, world frame)
  → RViz2
```

**TurtleBot data path**:
```
TurtleBot RPi4 (tb1, via Wi-Fi DDS)
  → /{tb1}/scan              (LaserScan, RPLIDAR A1, 360°, 5.5 Hz)
  → /{tb1}/odom              (Odometry, wheel encoder, 62 Hz)
  → /{tb1}/tf, /{tb1}/tf_static   (namespaced TF from onboard stack)
  → topic_tools relay
      → /tf, /tf_static      (merged into global TF tree)
  → turtlebot_odom_tf_node
      → TF: odom → base_link (re-broadcast to global /tf)
  → slam_toolbox (async, namespaced)
      → TF: tb1/map → odom   (SLAM-corrected pose)
      → /{tb1}/map            (OccupancyGrid, 5 cm resolution)
  → turtlebot_lidar_pointcloud_node
      → /{tb1}/pointcloud     (PointCloud2, accumulated 3D, world frame)
  → RViz2
```

---

## 3. Implementation

### 3.1 Custom ROS 2 Nodes

Five Python nodes were written as part of the `multi_robot` package. They are registered as console script entry points in `setup.py` and installed into the ROS 2 environment by `colcon build`.

---

#### `crazyflie_path_node` — Flight Mission Sequencer

This node executes a timed autonomous flight sequence for all enabled Crazyflies. It reads `crazyflies.yaml` at startup, filters to robots where `enabled: true`, and creates `crazyflie_interfaces` service clients for each:

| Service | Type | Purpose |
|---|---|---|
| `/{cf}/arm` | `Arm` | Spin motors up or down |
| `/{cf}/takeoff` | `Takeoff` | Climb to target height |
| `/{cf}/land` | `Land` | Descend to near-ground height |

The flight sequence is:

```
ARM  →  TAKEOFF (1.0 m, 2.5 s duration)  →  HOVER (10.0 s)  →  LAND (0.04 m, 2.5 s)  →  DISARM
```

All Crazyflies are commanded in parallel using asynchronous service calls (`call_async`), so a multi-CF lab can synchronize all drones to the same phase simultaneously. The node exits cleanly after the disarm step.

---

#### `multi_ranger_pointcloud_node` — CF 3D Point Cloud Accumulator

This node converts the sparse 5-ray Multi-Ranger LaserScan into a growing 3D point cloud in the world frame.

For each scan message:
1. Look up the TF transform `world ← {scan.header.frame_id}` from the global TF tree.
2. For each valid ray with range $r$ in $[r_\text{min},\ r_\text{max}]$:

$$p_\text{sensor} = \begin{bmatrix} r \cos\theta \\ r \sin\theta \\ 0 \end{bmatrix}, \qquad p_\text{world} = R \cdot p_\text{sensor} + t$$

where $R$ and $t$ come from the TF lookup (quaternion converted to a 3×3 rotation matrix).

3. Append $p_\text{world}$ to a rolling accumulator; trim to `max_points = 30,000` by discarding the oldest entries.
4. Publish the accumulator as a `sensor_msgs/PointCloud2` to `/{cf}/pointcloud` with `frame_id = world`.

The Multi-Ranger's 5-ray geometry means the cloud is sparse and grows slowly — it is primarily useful as a qualitative indicator of obstacles near the drone, not as a dense mapping source.

---

#### `turtlebot_odom_tf_node` — Odometry-to-TF Bridge

The iRobot Create3 base publishes wheel odometry as `nav_msgs/Odometry` but does **not** publish the corresponding `odom → base_link` TF transform. This omission is by design in the Create3 firmware — the expectation is that a higher-level localization system (e.g., Nav2 AMCL) will manage that transform. However, slam_toolbox requires `odom → base_link` to be present in the TF tree before it can process scans.

This node fills the gap. On every odometry message received, it immediately re-broadcasts the pose as a `TransformStamped` via `tf2_ros.TransformBroadcaster`:

```python
t.header = msg.header          # frame_id = 'odom', stamp from message
t.child_frame_id = msg.child_frame_id   # = 'base_link'
t.transform.translation = msg.pose.pose.position
t.transform.rotation = msg.pose.pose.orientation
self._broadcaster.sendTransform(t)
```

`TransformBroadcaster` publishes directly to the global `/tf` topic regardless of node namespace, so one instance per robot (launched with remapping `odom → /{tb}/odom`) correctly populates the shared TF tree.

---

#### `turtlebot_lidar_pointcloud_node` — TB 3D Point Cloud Accumulator

Structurally identical to the Crazyflie accumulator, but tuned for the RPLIDAR A1:

- Max accumulator size: **100,000 points** (the 360° LIDAR fills this much faster than the 5-ray Multi-Ranger)
- Topic remapped at launch: `scan → /{tb}/scan`, `pointcloud → /{tb}/pointcloud`
- TF lookup: `world ← {scan.header.frame_id}` (resolves through the full `world → tb/map → odom → base_link → rplidar` chain)

The node projects each 2D scan ray into the world frame and accumulates a permanent floor plan of the environment as the robot drives. Points are never removed from the cloud until the node restarts or `max_points` is exceeded, which gives a persistent occupancy trace even if the robot returns to a previously visited area.

---

#### `turtlebot_path_node` — Autonomous Motion Profile (Built, Not Used in Final Config)

An autonomous motion controller was implemented that publishes a repeating rectangular driving pattern over a 24-second cycle (forward, turn left, repeat). This was superseded in practice by manual teleop keyboard control, which gives better control during CF hover sessions and avoids driving the TurtleBot into walls during initial SLAM map building.

---

### 3.2 Launch Architecture

Three launch files provide progressively combined capability:

| Launch File | Purpose |
|---|---|
| `crazyflie_viz.launch.py` | Crazyflie only: bringup + flight mission + Multi-Ranger cloud + RViz |
| `turtlebot_viz.launch.py` | TurtleBot only: SLAM + odom bridge + TF relays + LIDAR cloud + RViz |
| `unified_multi_robot.launch.py` | Combined: all of the above in one launch |

All three follow the same YAML-driven per-robot loop pattern. For TurtleBots, the loop reads `turtlebots.yaml`, iterates over enabled robots, and instantiates the following node set per robot:

1. `tf2_ros/static_transform_publisher` — anchors `{tb}/map` at world origin
2. `slam_toolbox/async_slam_toolbox_node` (namespaced, with scan + map remappings)
3. `nav2_lifecycle_manager` (autostart, manages slam_toolbox lifecycle)
4. `multi_robot/turtlebot_odom_tf_node` (with odom topic remapping)
5. `topic_tools/relay` × 2 (bridge `/{tb}/tf` and `/{tb}/tf_static` to global)
6. `multi_robot/turtlebot_lidar_pointcloud_node` (with scan + pointcloud remappings)

This pattern scales cleanly to additional robots: adding a new TurtleBot requires only setting `enabled: true` in `turtlebots.yaml` and relaunching — no Python code changes.

The unified launch also injects the `cflib` virtual environment into `PYTHONPATH` at launch time, because the Crazyflie Python library is not installed system-wide.

---

### 3.3 SLAM Configuration

SLAM is performed per TurtleBot using `slam_toolbox`'s asynchronous mode, which processes scans in a background thread to avoid blocking the main ROS 2 executor.

Key parameters in `slam_toolbox.yaml`:

| Parameter | Value | Meaning |
|---|---|---|
| `solver_plugin` | `CeresSolver` | Nonlinear graph optimization backend |
| `linear_solver_type` | `SPARSE_NORMAL_CHOLESKY` | Efficient sparse Cholesky factorization |
| `trust_region_strategy` | `LEVENBERG_MARQUARDT` | Robust optimizer for pose graph refinement |
| `resolution` | `0.05` m | 5 cm occupancy grid cell size |
| `max_laser_range` | `12.0` m | RPLIDAR A1 usable range |
| `map_update_interval` | `5.0` s | Frequency of map visualization update |
| `transform_publish_period` | `0.02` s | TF broadcast at 50 Hz |
| `mode` | `mapping` | Builds a new map (not localization-only) |

Frame IDs are overridden per robot in the launch file: `odom_frame: odom`, `map_frame: {tb}/map`, `base_frame: base_link`.

---

## 4. Challenges and Solutions

This section documents the eight most significant technical obstacles encountered during integration. Each represents a case where the expected behavior of a ROS 2 component diverged from what actually occurred, requiring diagnosis through topic introspection, TF tree inspection, and incremental isolation testing.

### Summary Table

| # | Challenge | Root Cause | Solution |
|---|---|---|---|
| 4.1 | `odom → base_link` TF missing | Create3 publishes odometry as topic, not TF | Custom `turtlebot_odom_tf_node` |
| 4.2 | slam_toolbox completely silent | Lifecycle node starts unconfigured in ROS 2 Jazzy | `nav2_lifecycle_manager` with autostart |
| 4.3 | slam_toolbox on wrong scan topic | `scan_topic` parameter ignored in namespaced launch | DDS-layer topic remappings |
| 4.4 | SLAM map not visible in RViz | slam_toolbox publishes to absolute `/map` | Output remapping: `/map → /{tb}/map` |
| 4.5 | TurtleBot ignores teleop | QoS mismatch: RELIABLE publisher, BEST_EFFORT subscriber | `qos_overrides` parameter on teleop |
| 4.6 | RViz2 crashes on launch | snap-installed VSCode pollutes `LD_LIBRARY_PATH` | Strip `/snap/*` entries in launch file |
| 4.7 | TurtleBot TF invisible to host | Onboard stack publishes to `/{tb}/tf`, not `/tf` | `topic_tools relay` nodes per robot |
| 4.8 | Crazyflie drifts on black floor | Optical flow sensor needs surface texture | Textured mat placed under flight area |

---

### 4.1 Create3 Does Not Publish the `odom → base_link` TF Transform

**Problem**

The iRobot Create3 mobile base publishes wheel odometry to `/{tb}/odom` as `nav_msgs/Odometry` at approximately 62 Hz. However, it does not publish the corresponding `odom → base_link` transform to the TF tree. Without this link, slam_toolbox cannot determine the robot's position when a scan arrives, and the LIDAR accumulator cannot project scan points into the world frame. Both nodes would silently receive scans they could not process.

**Discovery**

Running `ros2 run tf2_tools view_frames` on the visualization host produced a TF tree that stopped at the `odom` frame. The chain `tb1/map → odom → base_link` was broken at the `odom → base_link` step. Separately, `ros2 topic echo /tb1/odom` confirmed the odometry data was arriving with valid position and orientation fields, but nothing was consuming it to produce a TF.

**Solution**

A new ROS 2 node, `turtlebot_odom_tf_node`, was written. It subscribes to the odometry topic and immediately re-broadcasts the contained pose as a `geometry_msgs/TransformStamped` using `tf2_ros.TransformBroadcaster`. The message header (frame ID, timestamp) is copied directly from the odometry message, preserving timestamps and ensuring continuity. Because `TransformBroadcaster` always publishes to the absolute global `/tf` topic — regardless of node namespace — one instance per robot (with topic remapping `odom → /{tb}/odom`) correctly and exclusively populates each robot's portion of the global TF tree.

After this fix, `ros2 run tf2_tools view_frames` showed the complete chain `tb1/map → odom → base_link → tb1/rplidar`, and slam_toolbox began processing scans.

---

### 4.2 slam_toolbox Is a Lifecycle Node and Starts Unconfigured

**Problem**

In ROS 2 Jazzy (released late 2024), `async_slam_toolbox_node` is implemented as a **managed lifecycle node**. Lifecycle nodes in ROS 2 start in the "unconfigured" state and have no active subscriptions, publications, or timers until they are explicitly configured and activated. This means the node appears in `ros2 node list`, consumes CPU, and logs nothing — but does absolutely nothing. No scan is processed, no TF is published, no map is built.

This behavior was a change from older ROS 2 distributions (Humble, Foxy) where slam_toolbox started in an active state automatically. The ROS 2 Jazzy release notes do not prominently document this change, making it easy to miss.

**Discovery**

After the TF chain was confirmed complete (Section 4.1), slam_toolbox still produced no output. Running `ros2 node info /tb1/slam_toolbox` revealed only one subscriber: `/parameter_events`. There was no subscription to `/scan` or any other data topic. The node was alive but inactive.

To test the hypothesis, the lifecycle was manually advanced:

```bash
ros2 lifecycle set /tb1/slam_toolbox configure
ros2 lifecycle set /tb1/slam_toolbox activate
```

Immediately after activation, `ros2 node info /tb1/slam_toolbox` showed a subscription to `/tb1/scan`, and the TF topic `/tf` began receiving `tb1/map → odom` transforms. SLAM was working.

**Solution**

Added a `nav2_lifecycle_manager` Node to the per-robot launch block in both launch files:

```python
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name=f'lifecycle_manager_slam_{ns}',
    namespace=ns,
    output='screen',
    parameters=[{
        'use_sim_time': False,
        'autostart': True,
        'node_names': ['slam_toolbox'],
    }],
)
```

With `autostart: True`, the lifecycle manager monitors slam_toolbox's state and automatically issues configure → activate transitions at launch. No manual `lifecycle set` commands are required. This fix was applied to `turtlebot_viz.launch.py` and `unified_multi_robot.launch.py`.

---

### 4.3 slam_toolbox Ignores the `scan_topic` Parameter in a Namespaced Launch

**Problem**

slam_toolbox has a parameter `scan_topic` intended to override which topic it subscribes to for scan data. The initial approach was to pass `scan_topic: /{tb}/scan` as a launch parameter. In a namespaced node (`namespace=ns`), this was silently ignored: slam_toolbox subscribed to the default topic `/scan` regardless of what was passed.

With no publisher on `/scan` (all scan data goes to the namespaced `/{tb}/scan`), slam_toolbox received zero data and built no map — again silently.

**Discovery**

After activating slam_toolbox via the lifecycle manager (Section 4.2), `ros2 topic info /tb1/scan --verbose` showed that slam_toolbox was not listed as a subscriber. However, `ros2 node info /tb1/slam_toolbox` showed it was subscribed to `/scan` (the unnamespaced absolute topic). The parameter override had no effect.

**Solution**

Replaced the parameter approach with DDS-layer **topic remappings** in the Node declaration:

```python
remappings=[
    ('/scan', f'/{ns}/scan'),
]
```

Remappings are applied by the ROS 2 middleware at the DDS layer, before any parameter resolution. They are unconditional: the node never sees the original topic name `/scan` — all traffic is silently redirected to `/{tb}/scan` at the transport level. This approach works regardless of how the node handles its `scan_topic` parameter internally.

After this change, slam_toolbox correctly subscribed to `/{tb}/scan` and processed scans from the RPLIDAR.

---

### 4.4 SLAM Occupancy Grid Published to Absolute `/map` Instead of Namespaced Topic

**Problem**

Even after SLAM was activated and processing scans, the occupancy grid did not appear in RViz2. The RViz2 `multi_robot.rviz` configuration had Map display panels configured to look at `/{tb}/map` (one per robot). The occupancy grid was being published, but to the wrong topic.

slam_toolbox's C++ implementation hardcodes its occupancy grid publisher to the absolute topic `/map`, regardless of node namespace. Two robots running simultaneously would both publish to `/map`, overwriting each other's maps, and neither would appear in RViz2 under their expected `/{tb}/map` topic.

**Solution**

Extended the remappings block to redirect the map output topics:

```python
remappings=[
    ('/scan',         f'/{ns}/scan'),
    ('/map',          f'/{ns}/map'),
    ('/map_metadata', f'/{ns}/map_metadata'),
]
```

The same DDS-layer remapping mechanism as the scan fix redirects slam_toolbox's internal `/map` publisher to `/{tb}/map`. After this change, `ros2 topic list | grep map` showed `/tb1/map` being published, and the occupancy grid appeared in RViz2.

---

### 4.5 QoS Mismatch Silently Blocks Teleop Commands

**Problem**

The standard `teleop_twist_keyboard` node publishes `geometry_msgs/Twist` to `/cmd_vel` with **RELIABLE** QoS. The Create3 base's `create3_repub` node subscribes to `/{tb}/cmd_vel_unstamped` with **BEST_EFFORT** QoS.

In DDS (the underlying transport layer for ROS 2), a RELIABLE publisher and a BEST_EFFORT subscriber are **QoS-incompatible**: the publisher's offered QoS is "stronger" than what the subscriber requested, but DDS treats this as a policy mismatch and silently drops all messages. No error, warning, or log message is generated by either node.

The result: the TurtleBot does not move at all, and no feedback indicates why.

**Discovery**

Running `ros2 topic info /tb1/cmd_vel_unstamped --verbose` displayed the QoS profiles for all publishers and subscribers on that topic:

```
Publisher count: 1
  Publisher ... RELIABILITY: RELIABLE

Subscription count: 1
  Subscription ... RELIABILITY: BEST_EFFORT
```

The mismatch was explicit once inspected.

**Solution**

Override the teleop publisher's QoS profile at launch time using ROS 2's `qos_overrides` parameter mechanism:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  --remap cmd_vel:=/tb1/cmd_vel_unstamped \
  -p qos_overrides./tb1/cmd_vel_unstamped.publisher.reliability:=best_effort
```

This downgrades the teleop publisher to BEST_EFFORT, matching the Create3 subscriber's profile, without modifying either node's source code. The TurtleBot responds immediately to keyboard commands after this change.

---

### 4.6 RViz2 Crashes Due to snap `LD_LIBRARY_PATH` Pollution

**Problem**

When ROS 2 launch commands were run from the **VSCode integrated terminal** (VSCode is installed via snap on this system), the `LD_LIBRARY_PATH` environment variable contained paths beginning with `/snap/code/...`. RViz2, as a Qt application, linked against the snap-bundled Qt libraries from these paths instead of the system Qt libraries that ROS 2 was built against. The result was an immediate segfault on RViz2 startup.

**Discovery**

Printing `$LD_LIBRARY_PATH` in a VSCode terminal vs. a standard GNOME terminal showed the snap paths present only in VSCode. Running the same launch command from a regular terminal succeeded.

**Solution**

Added a `_clean_ld_library_path()` helper function to all launch files. It filters out any LD_LIBRARY_PATH entry beginning with `/snap/`:

```python
def _clean_ld_library_path():
    raw = os.environ.get('LD_LIBRARY_PATH', '')
    parts = [p for p in raw.split(':') if p and not p.startswith('/snap/')]
    return ':'.join(parts)
```

The cleaned path is passed to RViz2 via the `additional_env` argument of its Node declaration. This ensures RViz2 sees only system library paths regardless of which terminal the launch was invoked from.

**Note**: All ROS 2 launch commands for this system should be run from a standard system terminal, not from the VSCode integrated terminal. The VSCode terminal may introduce other environment variable conflicts.

---

### 4.7 TurtleBot TF Is Published to a Namespaced Topic, Not the Global `/tf`

**Problem**

The TurtleBot 4 onboard software stack runs in a namespaced ROS 2 environment (namespace `/{tb}`). As a result, all TF transforms published by the onboard nodes go to `/{tb}/tf` and `/{tb}/tf_static` rather than the global `/tf` and `/tf_static`. The visualization host reads only the global TF topics. RViz2 and the LIDAR accumulator node could not find any TurtleBot frames — the robot and its sensor frame were completely invisible to the host.

**Discovery**

Running `ros2 topic list | grep tf` showed `/{tb}/tf` and `/{tb}/tf_static` topics with data, but the global `/tf` had no TurtleBot-related transforms. `ros2 run tf2_tools view_frames` on the host showed no TurtleBot frames at all.

**Solution**

Added two `topic_tools relay` nodes per TurtleBot to the launch file. Each relay runs on the visualization host and forwards messages from the namespaced topic to the global topic:

```python
Node(package='topic_tools', executable='relay',
     name=f'tf_relay_{ns}',
     arguments=[f'/{ns}/tf', '/tf'])

Node(package='topic_tools', executable='relay',
     name=f'tf_static_relay_{ns}',
     arguments=[f'/{ns}/tf_static', '/tf_static'])
```

After launching these relays, all TurtleBot TF frames (including `base_link`, `rplidar`, and their parents) appeared in the global TF tree and became available to RViz2 and the LIDAR accumulator.

---

### 4.8 Flow Deck Optical Flow Failure on Black Lab Floor

**Problem**

The Crazyflie Flow Deck v2 uses a PAA3905 optical flow sensor to estimate horizontal velocity by tracking surface features beneath the drone. The lab floor is uniform black tile with no discernible texture. When hovering over a black surface, the optical flow sensor produces near-zero velocity readings regardless of actual lateral drift, because there are no surface features to track.

The Kalman filter, receiving near-zero velocity measurements, concludes the drone is not moving and does not correct its position estimate for drift. The drone appeared stationary in RViz2 while physically drifting across the room.

**Discovery**

This was observed empirically during early flight tests: the drone's RViz2 pose remained nearly fixed while the physical drone visibly translated. Reverting to a hover over a textured surface (a patterned mat) eliminated the drift.

**Solution (Workaround)**

Place a textured mat — such as a foam puzzle mat or a patterned carpet tile — beneath the Crazyflie's takeoff position. The optical flow sensor requires visible surface features with sufficient contrast to compute velocity. On textured surfaces, the position estimate remains stable throughout hover.

A permanent solution would require an external absolute positioning system. The lab's OptiTrack motion capture system (configured in `motion_capture.yaml`, IP: `141.23.110.143`) can provide sub-millimeter ground truth. Enabling it requires setting `tracking: optitrack` in `crazyflies.yaml` and powering on the OptiTrack server — this infrastructure is present but not yet deployed for this project.

---

### 4.9 Key ROS 2 Diagnostic Commands

The following commands were repeatedly useful during debugging. They are listed here as a quick reference for anyone reproducing or extending this system.

**Inspect the TF tree** — generates a PDF showing all active frames and their parent-child relationships. The most useful first step when anything spatial is broken.

```bash
ros2 run tf2_tools view_frames
```

**Check if a specific transform exists** — streams the live transform between two frames. If it hangs, the link is missing.

```bash
ros2 run tf2_ros tf2_echo world tb1/base_link
```

**Inspect a node's subscriptions and publications** — revealed that slam_toolbox was not subscribed to `/scan` after launch (Challenges 4.2 and 4.3).

```bash
ros2 node info /tb1/slam_toolbox
```

**Check QoS profiles on a topic** — revealed the RELIABLE / BEST_EFFORT mismatch blocking teleop (Challenge 4.5).

```bash
ros2 topic info /tb1/cmd_vel_unstamped --verbose
```

**Manually advance a lifecycle node** — used to confirm slam_toolbox would work once activated, before the lifecycle manager was added (Challenge 4.2).

```bash
ros2 lifecycle set /tb1/slam_toolbox configure
ros2 lifecycle set /tb1/slam_toolbox activate
```

**List all active topics filtered by keyword** — useful for verifying that namespaced topics are being published after remappings are applied.

```bash
ros2 topic list | grep map
ros2 topic list | grep scan
```

---

## 5. Results

The following capabilities were demonstrated and confirmed working:

**Crazyflie**
- cfb4 executes the full autonomous flight sequence (arm → 1 m takeoff → 10 s hover → land → disarm) reliably when a textured surface is in place.
- The Multi-Ranger 3D point cloud accumulates in the world frame and is visible in RViz2 as a sparse environmental scan during hover.
- Kalman-filtered pose is published at 10 Hz and displayed as a drone model in RViz2.

**TurtleBot**
- tb1 builds a SLAM occupancy grid map in real time using the RPLIDAR A1 and slam_toolbox.
- The SLAM map auto-activates on launch (no manual `lifecycle set` commands needed).
- The LIDAR 3D accumulated point cloud builds up in the world frame alongside the CF cloud.
- Manual keyboard teleop works correctly with the QoS override.

**Combined Visualization**
- RViz2 displays CF point cloud, TB SLAM occupancy grid, and TB LIDAR point cloud simultaneously in a shared world frame.
- Switching the active TurtleBot requires only editing `turtlebots.yaml` and relaunching — no code changes.

**Limitation**: Without OptiTrack, the Crazyflie and TurtleBot share the `world` frame by convention but are not spatially aligned. Each robot's odometry origin is wherever it was sitting at startup. A flight session and a drive session in the same room will not produce spatially consistent maps unless both robots start from the same physical position.

---

## 6. How to Run

### Environment Setup

Three workspaces must be sourced in this exact order in every terminal used for launch or teleop:

```bash
source /opt/ros/jazzy/setup.bash
source /home/drl/Desktop/Crazyflies/ros2_ws/install/setup.bash
source /home/drl/multi_robot_RViz/multi_robot_visualization/multi_robot_ws/install/setup.bash
```

**Important**: Use a standard GNOME terminal, not the VSCode integrated terminal (see Section 4.6).

### Launch the Combined System

```bash
ros2 launch multi_robot unified_multi_robot.launch.py
```

This starts Crazyflie bringup, the flight mission, Multi-Ranger accumulator, TurtleBot SLAM, odom bridge, TF relays, LIDAR accumulator, and RViz2 in a single command.

### Manual TurtleBot Teleop

Open a second terminal, source the workspaces, then run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  --remap cmd_vel:=/tb1/cmd_vel_unstamped \
  -p qos_overrides./tb1/cmd_vel_unstamped.publisher.reliability:=best_effort
```

Use `I`, `J`, `L`, `,` keys to drive. Press `K` to stop.

### Switching the Active TurtleBot

Edit `multi_robot_visualization/multi_robot_ws/src/multi_robot/config/turtlebots.yaml`:

```yaml
robots:
  tb1:
    enabled: false   # ← disable this one
  tb2:
    enabled: true    # ← enable this one
```

Then rebuild and relaunch:

```bash
cd /home/drl/multi_robot_RViz/multi_robot_visualization
colcon build --symlink-install
ros2 launch multi_robot unified_multi_robot.launch.py
```

A rebuild is required after editing YAML files because the config is installed into the ROS 2 share directory by `setup.py`.

### Standalone Launch Files

```bash
# Crazyflies only
ros2 launch multi_robot crazyflie_viz.launch.py

# TurtleBots only
ros2 launch multi_robot turtlebot_viz.launch.py
```

---

## 7. Future Work

**Multi-TurtleBot Simultaneous Operation**  
Currently, only one TurtleBot can be active at a time. The root cause is that all TurtleBots publish `odom → base_link` with the same child frame ID (`base_link`). If two robots are enabled simultaneously, their transforms overwrite each other in the global TF tree. The fix is to publish `odom → {tb}/base_link` instead, making each robot's body frame uniquely namespaced. This requires changes to `turtlebot_odom_tf_node` and corresponding updates to the slam_toolbox `base_frame` parameter.

**OptiTrack Integration**  
`motion_capture.yaml` is fully configured for the lab's OptiTrack system (server IP: `141.23.110.143`). Enabling tracking in `crazyflies.yaml` (`tracking: optitrack`) would give Crazyflies millimeter-precision absolute position ground truth, eliminating the optical flow dependency and resolving the spatial alignment gap between CF and TB coordinate origins. This is the most impactful near-term improvement.

**Nav2 Autonomous Navigation**  
The SLAM-generated occupancy grid is a valid input to the Nav2 navigation stack. Adding Nav2 would allow TurtleBots to execute waypoint-following missions autonomously, replacing the manual teleop approach used in current demos.

**CF-Side SLAM**  
The Multi-Ranger's five-ray geometry produces too sparse a scan for slam_toolbox to perform reliable loop closure. A future hardware upgrade to a spinning micro-LIDAR or a depth camera on the Crazyflie would enable CF-side SLAM. The software infrastructure (point cloud accumulator, world-frame TF chain) is already in place.

---

## Appendix: Node and Topic Reference

### Custom Nodes

| Node | Executable | Subscribes | Publishes | Notes |
|---|---|---|---|---|
| `crazyflie_path_node` | `crazyflie_path_node` | — | Service calls | Exits after flight complete |
| `multi_ranger_pointcloud_node` | `multi_ranger_pointcloud_node` | `/{cf}/scan` | `/{cf}/pointcloud` | Per enabled CF |
| `turtlebot_odom_tf_node` | `turtlebot_odom_tf_node` | `/{tb}/odom` | TF: `odom → base_link` | Per enabled TB |
| `turtlebot_lidar_pointcloud_node` | `turtlebot_lidar_pointcloud_node` | `/{tb}/scan` | `/{tb}/pointcloud` | Per enabled TB |
| `turtlebot_path_node` | `turtlebot_path_node` | — | `/{tb}/cmd_vel` | Built; not used in final config |

### Key Topics

| Topic | Message Type | Publisher | Subscribers |
|---|---|---|---|
| `/{cf}/scan` | `LaserScan` | `crazyflie_ros2` | `multi_ranger_pointcloud_node` |
| `/{cf}/pointcloud` | `PointCloud2` | `multi_ranger_pointcloud_node` | RViz2 |
| `/{tb}/scan` | `LaserScan` | TurtleBot driver | `slam_toolbox`, `turtlebot_lidar_pointcloud_node` |
| `/{tb}/odom` | `Odometry` | TurtleBot driver | `turtlebot_odom_tf_node` |
| `/{tb}/map` | `OccupancyGrid` | `slam_toolbox` (remapped) | RViz2 |
| `/{tb}/pointcloud` | `PointCloud2` | `turtlebot_lidar_pointcloud_node` | RViz2 |
| `/tf` | `TFMessage` | All broadcasters | RViz2, accumulator nodes |
| `/tf_static` | `TFMessage` | Static publishers, relays | RViz2, accumulator nodes |

### Source Workspaces

| Workspace | Path | Contents |
|---|---|---|
| ROS 2 base | `/opt/ros/jazzy/` | All standard ROS 2 packages |
| Crazyflie | `/home/drl/Desktop/Crazyflies/ros2_ws/` | `crazyflie_ros2`, `crazyflie_interfaces` |
| This project | `/home/drl/multi_robot_RViz/multi_robot_visualization/multi_robot_ws/` | `multi_robot` package |
| cflib venv | `/home/drl/Desktop/Crazyflies/venv/` | Bitcraze Python library (injected via `PYTHONPATH` in launch) |
