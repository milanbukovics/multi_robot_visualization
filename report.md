# Multi-Robot Real-Time Visualization Using ROS 2

**Author**: Milan Bukovics
**Institution**: University of Hawaiʻi at Mānoa — Dynamic Robotics Lab
**Date**: May 2026

---

## 1. Introduction

Heterogeneous multi-robot systems — teams that mix aerial and ground platforms — are increasingly central to applications in search and rescue, environmental monitoring, and cooperative mapping [11]. A practical challenge that arises immediately in deploying such systems is unified situational awareness: an operator needs to observe all robots simultaneously in a shared spatial reference frame, with each robot's sensor data rendered in real time. This is straightforward when all robots share the same hardware and software stack, but becomes significantly more complex when the platforms differ in communication protocol, sensor modality, localization method, and middleware assumptions.

This project addresses that challenge for a specific heterogeneous pair: Bitcraze Crazyflie 2.1 micro-quadrotors and iRobot Create3-based TurtleBot 4 ground vehicles. The Crazyflies carry a Flow Deck v2 (optical flow velocity estimation and Time-of-Flight height sensing) and a Multi-Ranger Deck (five-direction Time-of-Flight ranging at up to 4 m), and localize using an onboard Extended Kalman Filter without any external positioning infrastructure. The TurtleBots carry an RPLIDAR A1 360° planar LIDAR and localize via wheel odometry corrected by Simultaneous Localization and Mapping (SLAM). Neither platform has access to an external motion capture system during development. The two robots communicate through entirely different transports — 2.4 GHz Crazyradio PA for the Crazyflie and Wi-Fi DDS for the TurtleBot — and make incompatible assumptions about the ROS 2 TF coordinate frame tree.

Bridging these two platforms into a single coherent visualization required resolving a series of middleware integration problems that are not covered by either robot's documentation in isolation. The contribution of this project is primarily that integration work: the custom ROS 2 software layer, the YAML-driven launch architecture, and the practical knowledge gained from bringing both systems up on physical hardware simultaneously.

The goals of the project are: (1) execute autonomous Crazyflie flight sequences and visualize the drone's pose and Multi-Ranger 3D point cloud in RViz2; (2) run real-time SLAM for each active TurtleBot and visualize the resulting occupancy grid map; and (3) accumulate 3D point clouds from both robot types in a common world frame so that a single RViz2 window provides a unified environmental view. All software is developed as a custom `multi_robot` ROS 2 Python package running on Ubuntu 24.04 with ROS 2 Jazzy (the late-2024 LTS release).

---

## 2. Related Work

**ROS 2 as multi-robot middleware.** The Robot Operating System 2 (ROS 2) is the dominant middleware framework for academic and industrial robotics [1]. Its use of DDS (Data Distribution Service) for transport enables decentralized, peer-to-peer communication across heterogeneous platforms on a shared network, without the single-master bottleneck present in ROS 1. Macenski et al. survey ROS 2's architecture and document real-world deployments spanning mobile, aerial, and marine platforms [1]. Namespace isolation — the ability to prefix all node names and topics with a robot identifier — is the standard mechanism for running multiple robots on the same DDS domain without topic collision, and is used extensively throughout this project. The TF2 library, which underlies all coordinate frame management, was introduced by Foote as a distributed transform management system designed explicitly for multi-robot environments where different processes publish transforms from different physical locations [2].

**SLAM for ground robots.** Occupancy grid mapping from 2D LIDAR is a well-established problem with a rich literature. slam_toolbox [3], used in this project, implements an efficient pose graph SLAM algorithm backed by the Ceres nonlinear least-squares optimizer [4]. The pose graph is built from scan-match constraints computed at each LIDAR sweep; loop closure is detected by comparing the current scan against a spatial index of prior keyframes. slam_toolbox is the default SLAM solution in the Nav2 autonomous navigation stack [5] and supports asynchronous scan processing, meaning SLAM runs in a background thread that does not block the main ROS 2 executor — an important property when sharing CPU with visualization and point cloud accumulation. Alternative approaches considered include GMapping [6], a Rao-Blackwellized particle filter approach that is simpler but does not scale well to large environments, and Cartographer [7], which offers high accuracy but heavier computational requirements. slam_toolbox's native ROS 2 lifecycle node architecture and active maintenance for the Jazzy distribution made it the appropriate choice.

**Crazyflie as a research platform.** The Crazyflie 2.x quadrotor has been widely adopted for swarm robotics and autonomous flight research due to its open-source firmware, small form factor (27 g), and modular expansion deck system. Preiss et al. demonstrated coordinated flight with a 49-drone Crazyswarm, using the same Crazyflie hardware in a motion-capture arena [8]. This project uses the `crazyflie_ros2` package maintained by Bitcraze and the IMRC Lab, which bridges the Crazyflie radio protocol to standard ROS 2 topics and `crazyflie_interfaces` services [9]. Because no motion capture is available, position estimation relies entirely on the Flow Deck v2's optical flow sensor and the onboard Extended Kalman Filter. EKF-based state estimation for small quadrotors using optical flow and IMU measurements is described in Weiss et al. [10], and represents the same fusion approach implemented in Crazyflie firmware. A practical consequence of this choice is that the optical flow sensor requires visible surface texture beneath the drone — a constraint discussed further in Section 3.

**Multi-robot coordination and visualization.** Surveys on multi-robot systems [11] consistently identify unified situational awareness as a key operational requirement, yet most published aerial-ground systems rely on motion capture or GPS for global localization and treat the visualization layer as a solved problem. This project specifically targets the scenario where no external localization is available, and where the visualization infrastructure itself must compensate for the different frame conventions and middleware behaviors of each platform. The spatial alignment problem that results — each robot treats its own startup location as the world origin — is a known limitation of infrastructure-free multi-robot deployments and motivates the OptiTrack integration described in Section 4.

---

## 3. Current Progress

### 3.1 Hardware Platform

**Crazyflie 2.1 (cfb4).** The Crazyflie 2.1 is a 27 g open-source quadrotor with a 92 mm motor-to-motor diagonal. It communicates with the host computer via a Crazyradio PA USB dongle over a 2.4 GHz radio link. Two expansion decks are mounted: the Flow Deck v2 combines a PAA3905 optical flow sensor (measures surface-relative horizontal velocity) with a VL53L1x Time-of-Flight sensor (measures height above ground); the Multi-Ranger Deck adds five VL53L1x sensors pointing forward, back, left, right, and up, each with approximately 4 m range. The `crazyflie_ros2` driver publishes the five Multi-Ranger readings as a `sensor_msgs/LaserScan` message at 10 Hz to `/{cf}/scan`, and the Kalman-filtered pose as `geometry_msgs/PoseStamped` at 10 Hz to `/{cf}/pose`. The firmware uses the Mellinger controller and a Kalman filter estimator that fuses optical flow, height, and IMU measurements. High-level commands (arm, takeoff, land) are issued via `crazyflie_interfaces` ROS 2 service calls rather than continuous velocity setpoints.

A hardware constraint specific to this lab: the lab floor is uniform black tile. The PAA3905 optical flow sensor requires visible surface texture to estimate velocity; on a featureless black surface it produces near-zero readings regardless of actual motion, causing the Kalman filter to underestimate lateral drift. All flight tests use a textured foam mat placed under the takeoff position to provide the necessary optical contrast.

**TurtleBot 4 (tb2, tb3).** The TurtleBot 4 is a differential-drive ground robot built on the iRobot Create3 mobile base, with a Raspberry Pi 4 as the onboard compute platform. The Create3 base publishes wheel odometry as `nav_msgs/Odometry` to `/{tb}/odom` at approximately 62 Hz over a Wi-Fi DDS link. The RPLIDAR A1 is a 360° planar LIDAR with a 12 m nominal range; it publishes `sensor_msgs/LaserScan` to `/{tb}/scan` at approximately 5.5 Hz. The sensor frame `rplidar_link` is connected to the robot body frame `base_link` via a static TF published by the TurtleBot's onboard driver stack.

### 3.2 System Architecture

The visualization host (Ubuntu 24.04, ROS 2 Jazzy) runs all custom nodes and receives sensor data from both robots over the network. All robots share a single `world` frame as the coordinate root. The TF frame tree for one active Crazyflie and one active TurtleBot is:

```
world
├── cfb4                     ← crazyflie_ros2 (Kalman-filtered pose, 10 Hz)
│   └── cfb4/scan            ← Multi-Ranger sensor frame
└── tb{N}/map                ← static TF at world origin (anchors SLAM globally)
    └── odom                 ← slam_toolbox (SLAM-corrected drift estimate)
        └── base_link        ← turtlebot_odom_tf_node (from wheel odometry)
            └── rplidar_link ← TurtleBot driver static TF (sensor mounting offset)
```

Every link in this chain must be present and publishing at sufficient rate for the point cloud accumulators and SLAM to function. Data flows through two parallel pipelines to RViz2:

- **Crazyflie**: Crazyradio → `crazyflie_ros2` driver → `/{cf}/scan` + TF `world → cfb4` → `multi_ranger_pointcloud_node` → `/{cf}/pointcloud` → RViz2
- **TurtleBot**: Wi-Fi DDS → `/{tb}/scan` + `/{tb}/odom` + `/{tb}/tf` → `slam_toolbox` + `turtlebot_odom_tf_node` + `turtlebot_lidar_pointcloud_node` → `/{tb}/map` + `/{tb}/pointcloud` → RViz2

Because the TurtleBot's onboard software stack publishes TF transforms to the namespaced `/{tb}/tf` and `/{tb}/tf_static` topics rather than the global `/tf` and `/tf_static` topics, a pair of `topic_tools relay` nodes bridges each robot's namespaced TF stream into the global TF tree that RViz2 and the accumulator nodes query.

### 3.3 Software Implementation

Five custom Python nodes were written as part of the `multi_robot` package.

**`crazyflie_path_node`** reads `crazyflies.yaml` at startup, filters to enabled robots, and creates service clients for the `Arm`, `Takeoff`, and `Land` services on each. It then executes the flight sequence — arm all CFs, send takeoff (1.0 m height, 2.5 s duration), block for the hover period (10 s), send land (0.04 m height), disarm — issuing commands to all enabled Crazyflies in parallel via asynchronous service calls so that multi-drone flights are synchronized.

**`multi_ranger_pointcloud_node`** converts the five-ray Multi-Ranger `LaserScan` into a growing world-frame `PointCloud2`. For each incoming scan, it looks up the TF transform `world ← {scan.header.frame_id}` and converts each valid range reading $r$ at angle $\theta$ into a 3D world-frame point:

$$p_\text{world} = R \cdot \begin{bmatrix} r\cos\theta \\ r\sin\theta \\ 0 \end{bmatrix} + t$$

where $R$ and $t$ are the rotation and translation from the TF lookup. Points are appended to a rolling buffer (default cap: 30,000) and the full buffer is published as a `PointCloud2` on each scan. This produces an incrementally growing 3D map of obstacles detected by the drone during flight.

**`turtlebot_odom_tf_node`** bridges the `nav_msgs/Odometry` topic published by the Create3 base into a TF transform — a gap that required a custom node because the Create3 does not publish the `odom → base_link` transform itself (see Challenge 2 in Section 3.4). On every odometry message received, it constructs a `TransformStamped` from the message's position and orientation fields, copying the message timestamp and frame IDs directly, and broadcasts it via `tf2_ros.TransformBroadcaster`. Because `TransformBroadcaster` publishes to the absolute global `/tf` topic regardless of node namespace, one instance per robot launched with topic remapping (`odom → /{tb}/odom`) correctly and exclusively populates each robot's portion of the tree.

**`turtlebot_lidar_pointcloud_node`** applies the same ray-projection algorithm as the Crazyflie accumulator to the TurtleBot's RPLIDAR scan, accumulating up to 100,000 world-frame points. The larger cap reflects the much higher scan density — the RPLIDAR sweeps 360° at several hundred rays per rotation versus the Multi-Ranger's five fixed rays.

Three launch files provide increasing levels of integration: `crazyflie_viz.launch.py` for Crazyflie-only testing, `turtlebot_viz.launch.py` for TurtleBot-only testing, and `unified_multi_robot.launch.py` for the combined system. All three follow a YAML-driven per-robot loop pattern: `crazyflies.yaml` and `turtlebots.yaml` each list all robots with an `enabled` flag. The launch reads these files and instantiates the full node set — SLAM, lifecycle manager, TF relay, odom bridge, and LIDAR accumulator — only for enabled robots. Adding a new robot requires only setting `enabled: true` in the YAML and relaunching; no Python code changes are needed.

The per-TurtleBot node set instantiated by the launch loop is:

1. `tf2_ros/static_transform_publisher` — anchors `{tb}/map` at the world origin with a zero-offset static TF, placing the robot's SLAM map in the global coordinate frame.
2. `slam_toolbox/async_slam_toolbox_node` (namespace `{tb}`) — subscribes to `/{tb}/scan` (via DDS-layer remapping), publishes the occupancy grid to `/{tb}/map` (via remapping), and broadcasts the `{tb}/map → odom` transform to the global TF tree.
3. `nav2_lifecycle_manager` — automatically configures and activates slam_toolbox at launch.
4. `multi_robot/turtlebot_odom_tf_node` — bridges `/{tb}/odom` to the global `odom → base_link` TF.
5. `topic_tools/relay` × 2 — forwards `/{tb}/tf → /tf` and `/{tb}/tf_static → /tf_static` so that the TurtleBot's onboard static sensor offsets are visible to RViz2 and the accumulator.
6. `multi_robot/turtlebot_lidar_pointcloud_node` — accumulates the world-frame LIDAR point cloud.

**SLAM configuration.** The slam_toolbox parameters in `slam_toolbox.yaml` are tuned for the lab environment. The Ceres solver uses `SPARSE_NORMAL_CHOLESKY` linear factorization with `LEVENBERG_MARQUARDT` trust-region optimization for the pose graph. Key parameters: grid resolution 5 cm, maximum LIDAR range 12 m (matching the RPLIDAR A1 specification), scan processing throttle of one scan every 0.5 s to reduce CPU load, and TF broadcast period of 20 ms (50 Hz). The map is redrawn in the visualization every 5 s. Interactive mode is enabled, allowing pose corrections to be injected through RViz2 if needed.

### 3.4 Key Integration Challenges

Three integration problems encountered during development were particularly non-obvious and are documented here both as a record of the project's engineering effort and as a practical reference for others working with the same hardware.

#### Challenge 1: slam_toolbox Starts as a Silent Lifecycle Node in ROS 2 Jazzy

In ROS 2 Jazzy (2024 LTS), `async_slam_toolbox_node` is a **managed lifecycle node**. It starts in the "unconfigured" state with no active subscriptions or publishers — the node appears in `ros2 node list`, consumes CPU, and prints nothing, but does not process scans, build a map, or publish any TF transforms. This behavior is a change from older ROS 2 distributions (Humble, Foxy) where the node started active automatically, and it is not prominently noted in the Jazzy release notes or slam_toolbox documentation.

The failure mode was entirely silent. After the node started, the launch output showed no errors; slam_toolbox appeared to be running normally. The problem was identified by directly inspecting the node's active subscriptions:

```bash
ros2 node info /tb1/slam_toolbox
# Subscribers: /parameter_events  (only — no scan subscription)
```

Manually advancing the lifecycle state confirmed the hypothesis: after `configure` and `activate`, slam_toolbox immediately subscribed to the scan topic and began publishing the `tb{N}/map → odom` TF transform. The permanent fix was adding a `nav2_lifecycle_manager` node to each robot's launch block with `autostart: True` and `node_names: ['slam_toolbox']`. The lifecycle manager monitors slam_toolbox's state on launch and automatically drives it through configure → activate, requiring no manual commands.

A second, related issue with slam_toolbox was that its `scan_topic` parameter — intended to redirect which topic it subscribes to — was silently ignored when the node ran inside a namespace. slam_toolbox subscribed to the absolute `/scan` topic regardless of what was passed. The solution was to replace the parameter override entirely with a DDS-layer topic remapping in the Node declaration (`remappings=[('/scan', f'/{ns}/scan')]`). Remappings operate at the middleware transport layer and are applied before any node-internal parameter resolution, making them robust to namespace-related parameter handling differences across package versions.

#### Challenge 2: The iRobot Create3 Does Not Publish the `odom → base_link` TF Transform

The iRobot Create3 mobile base publishes wheel odometry as `nav_msgs/Odometry` to `/{tb}/odom` but does **not** publish the corresponding `odom → base_link` TF transform. This is by design in the Create3 firmware, which expects a higher-level localization system such as Nav2 AMCL to manage that transform. However, slam_toolbox requires `odom → base_link` to be present before it can process scans, and the LIDAR point cloud accumulator requires it to project scan points into the world frame. Without it, both nodes silently receive data they cannot use.

Diagnosis was through TF tree inspection, which showed the chain terminating at `odom` with no children, while the odometry topic confirmed data was arriving correctly:

```bash
ros2 run tf2_tools view_frames   # TF chain stops at 'odom', no children
ros2 topic echo /{tb}/odom       # odometry arriving at 62 Hz with valid data
```

The fix was writing `turtlebot_odom_tf_node` — a minimal node (~25 lines) that subscribes to the odometry topic and, on each message, rebroadcasts the contained pose as a `TransformStamped` to the global TF tree. The node's simplicity belies its importance: without it, the entire TurtleBot visualization pipeline fails. It has been running reliably in all subsequent tests, broadcasting `odom → base_link` at the full 62 Hz odometry rate.

#### Challenge 3: DDS QoS Mismatch Silently Drops All Teleop Commands

`teleop_twist_keyboard` publishes velocity commands with **RELIABLE** QoS, the default for ROS 2 publishers. The Create3's command velocity subscriber uses **BEST_EFFORT** QoS. In DDS, these two profiles are incompatible: a RELIABLE publisher and BEST_EFFORT subscriber do not match, and messages are dropped silently at the transport layer — no error, warning, or log message is produced by either node. The result is a robot that appears connected (topics are visible, the teleop node is running) but does not respond to any keyboard input.

This class of failure is a known footgun in ROS 2 DDS deployments [1] and is only diagnosable through explicit QoS profile inspection:

```bash
ros2 topic info /{tb}/cmd_vel_unstamped --verbose
# Publisher count: 1  RELIABILITY: RELIABLE
# Subscription count: 1  RELIABILITY: BEST_EFFORT
```

The fix downgrades the teleop publisher to BEST_EFFORT using a ROS 2 QoS override parameter, matching the subscriber's profile without modifying either node's source code:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  --remap cmd_vel:=/{tb}/cmd_vel_unstamped \
  -p "qos_overrides./{tb}/cmd_vel_unstamped.publisher.reliability:=best_effort"
```

### 3.5 Results

The system was demonstrated with Crazyflie cfb4 and TurtleBot tb3 operating simultaneously in the Dynamic Robotics Lab. All primary goals were achieved and confirmed on physical hardware:

- cfb4 executes the full arm → 1 m takeoff → 10 s hover → land → disarm sequence autonomously and repeatably when a textured surface is placed beneath the takeoff position.
- The Multi-Ranger 3D point cloud accumulates in the world frame during flight; walls and obstacles near the drone are visible as sparse point clusters in RViz2.
- tb3 builds a SLAM occupancy grid map in real time as it is driven through the lab; the map auto-activates on launch with no manual lifecycle commands required.
- The LIDAR accumulated point cloud builds a dense world-frame floor plan alongside the CF cloud.
- RViz2 displays the SLAM map, LIDAR cloud, and CF cloud simultaneously from a single `ros2 launch` command.
- Manual keyboard teleop works reliably once the QoS override is applied.

A current limitation is spatial alignment. Without an external positioning system, the Crazyflie and TurtleBot each treat their physical startup position as `(0, 0, 0)` in the world frame. The two origins are independent, so their sensor data occupies the same coordinate space by convention but is not physically co-registered. In RViz2 this manifests as the CF point cloud appearing in a geometrically inconsistent position relative to the TurtleBot's SLAM map, as visible in the screenshots below. Resolving this requires either placing both robots at the same physical location at launch, or using OptiTrack to provide absolute positions in a shared reference frame.

**Lab hardware setup:**

*Photo to be added.*

**Combined view — angled perspective** showing the SLAM occupancy grid, TurtleBot LIDAR point cloud, and Crazyflie Multi-Ranger cloud overlaid in the shared world frame. The grey occupancy grid is the SLAM map; colored point clusters are the LIDAR and Multi-Ranger accumulations:

![RViz combined view — angled perspective](/multi_robot_visualization/SLAM.png)

**Top-down view** showing SLAM map coverage and the dense LIDAR point cloud from above:

![RViz combined view — top down](/multi_robot_visualization/SLAM_top.png)

**Side view** showing the 3D vertical structure of the accumulated point clouds relative to the SLAM map plane:

![RViz combined view — side perspective](/multi_robot_visualization/SLAM_side.png)

---

## 4. Next Steps and Future Work

**Multi-TurtleBot simultaneous operation.** Currently only one TurtleBot can be active at a time. The root cause is a coordinate frame naming conflict: every TurtleBot's `turtlebot_odom_tf_node` publishes the transform `odom → base_link` to the global TF tree using those literal frame IDs. With two robots enabled simultaneously, their transforms overwrite each other. The fix is to publish `{tb}/odom → {tb}/base_link` instead of the shared identifiers, making each robot's body frame uniquely named. This requires a small modification to `turtlebot_odom_tf_node` to accept a namespace parameter, and corresponding updates to the slam_toolbox `odom_frame` and `base_frame` parameters in the launch file.

**OptiTrack integration for spatial alignment.** The lab's OptiTrack motion capture system is fully pre-configured in `motion_capture.yaml`, including the server IP address, rigid body marker layouts for the Crazyflie, and dynamics constraints. Enabling tracking requires only setting `tracking: optitrack` in `crazyflies.yaml`. With OptiTrack active, `crazyflie_ros2` replaces the Kalman filter's drift-prone optical flow estimate with millimeter-precision absolute position measurements from the motion capture system. Critically, this gives the Crazyflie a position in the same physical reference frame as the TurtleBot's SLAM map, resolving the spatial alignment limitation and making the combined visualization physically meaningful.

**Autonomous navigation with Nav2.** The SLAM-generated occupancy grid produced by slam_toolbox is the standard input format for the Nav2 autonomous navigation stack [5]. Adding Nav2 would allow TurtleBots to receive goal poses and autonomously plan and execute paths through the mapped environment, replacing manual keyboard control. The existing SLAM configuration and TF chain are already compatible with Nav2's expected inputs.

**Crazyflie mapping contribution.** The Multi-Ranger Deck's five fixed rays are too geometrically sparse for reliable SLAM loop closure — there is insufficient overlap between successive scans to compute a reliable scan-match constraint. The point cloud is useful as a qualitative obstacle indicator during flight, but does not contribute to a persistent map. A future hardware addition of a miniature spinning LIDAR or a depth camera on the Crazyflie would provide the scan density needed to run slam_toolbox on the aerial platform, making the CF a first-class mapping contributor in the unified system.

### 4.1 Operational Notes

Several practical constraints apply to running the system that are not visible in the code but matter for reproducibility.

**Workspace sourcing order.** Three separate ROS 2 workspaces must be sourced in sequence before any launch or topic inspection command: the ROS 2 Jazzy base installation, the `crazyflie_ros2` workspace (which provides `crazyflie_interfaces`), and the `multi_robot` workspace. Sourcing out of order or omitting one workspace produces cryptic "package not found" or import errors that do not clearly indicate the missing source.

**Stale FastRTPS shared memory files.** The default RMW implementation (FastRTPS) uses shared memory segments in `/dev/shm/` for low-latency intra-host communication. When a ROS 2 process is killed without a clean shutdown, its shared memory lock files remain. After many sessions, hundreds of stale files accumulate, causing DDS to fail to initialize shared memory ports at startup and `ros2 topic list` to hang indefinitely in new terminals. The fix is a one-line cleanup before each launch session: `rm -rf /dev/shm/fastrtps_*`. A related issue: launching from the VSCode integrated terminal (which is installed via snap on this machine) pollutes `LD_LIBRARY_PATH` with snap library paths, causing RViz2 to crash against mismatched Qt libraries. All launches are run from a standard system terminal.

**Active robot selection.** Only one TurtleBot should be enabled at a time in `turtlebots.yaml` due to the TF frame conflict described in Section 4. Multiple Crazyflies can be enabled simultaneously; the launch loop creates independent node sets for each. The YAML files are symlinked through the colcon install tree, so changes take effect on the next launch without rebuilding.

---

## References

[1] T. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Science Robotics*, vol. 7, no. 66, p. eabm6074, 2022.

[2] T. Foote, "tf: The transform library," in *2013 IEEE Conf. on Technologies for Practical Robot Applications (TePRA)*, 2013, pp. 1–6.

[3] S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world," *Journal of Open Source Software*, vol. 6, no. 61, p. 2783, 2021.

[4] S. Agarwal, K. Mierle, and The Ceres Solver Team, "Ceres Solver," 2022. [Online]. Available: http://ceres-solver.org

[5] S. Macenski, F. Martín, R. White, and J. G. Clavero, "The Marathon 2: A navigation system," in *2020 IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)*, 2020, pp. 2718–2725.

[6] G. Grisetti, C. Stachniss, and W. Burgard, "Improved techniques for grid mapping with Rao-Blackwellized particle filters," *IEEE Transactions on Robotics*, vol. 23, no. 1, pp. 34–46, 2007.

[7] W. Hess, D. Kohler, H. Rapp, and D. Andor, "Real-time loop closure in 2D LIDAR SLAM," in *2016 IEEE Int. Conf. on Robotics and Automation (ICRA)*, 2016, pp. 1271–1278.

[8] J. A. Preiss, W. Hönig, G. S. Sukhatme, and N. Ayanian, "Crazyswarm: A large nano-quadcopter swarm," in *2017 IEEE Int. Conf. on Robotics and Automation (ICRA)*, 2017, pp. 3299–3304.

[9] Bitcraze AB, "crazyflie-ros2," GitHub repository, 2024. [Online]. Available: https://github.com/IMRCLab/crazyswarm2

[10] S. Weiss, M. W. Achtelik, S. Lynen, M. Chli, and R. Siegwart, "Real-time onboard visual-inertial state estimation and self-calibration of MAVs in unknown environments," in *2012 IEEE Int. Conf. on Robotics and Automation (ICRA)*, 2012, pp. 957–964.

[11] Z. Yan, N. Jouandeau, and A. A. Cherif, "A survey and analysis of multi-robot coordination," *International Journal of Advanced Robotic Systems*, vol. 10, no. 12, p. 399, 2013.
