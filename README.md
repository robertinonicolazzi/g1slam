# g1_slam

ROS 2 package providing SLAM and Nav2 navigation for the Unitree G1 humanoid robot.

The package handles all quirks specific to the G1:
- Converting `/dog_odom` into a TF `odom → base_link` (the G1 never publishes this natively)
- Compensating the ~61s clock offset between `/dog_odom` and the system clock
- Re-stamping the Livox point cloud to align with the corrected TF timestamps
- Projecting the Livox 3D point cloud into a 2D LaserScan for SLAM
- Publishing the robot pose in the `map` frame as `Odometry` for external planners

---

## Package Structure

```
g1_slam/
├── config/
│   ├── pc_to_laserscan.yaml          # pointcloud_to_laserscan parameters
│   ├── slam_toolbox.yaml             # slam_toolbox — mapping mode (default)
│   ├── slam_toolbox_localize.yaml    # slam_toolbox — localization mode
│   └── nav2_params.yaml              # Nav2 full stack parameters
├── launch/
│   ├── slam.launch.py                # SLAM pipeline — mapping or localization
│   ├── nav.launch.py                 # Nav2 navigation stack
│   └── rviz.launch.py                # RViz visualization
├── g1_slam/
│   ├── odom_to_tf.py                 # /dog_odom → TF odom→base_link
│   ├── restamp_cloud.py              # corrects Livox hardware clock
│   ├── restamp_odom.py               # corrects dog_odom clock offset
│   └── pose_publisher.py             # TF map→base_link → /inorbit/odom_pose
├── Dockerfile
├── build.sh                          # builds the Docker image
└── run.sh                            # runs the container on the robot
```

---

## Docker (recommended)

The package ships with a Docker setup based on `ros:humble` with CycloneDDS,
slam_toolbox, pointcloud_to_laserscan, and the Unitree SDK pre-installed.

### Build the image

```bash
./build.sh
```

This produces the image `g1_slam:humble`.

### Run on the robot

```bash
./run.sh
```

This starts the container with:
- `--net=host` and `--ipc=host` so it shares the robot's ROS network
- `ROS_DOMAIN_ID=0` (edit `run.sh` to match your setup)
- `--restart unless-stopped` so it survives reboots
- Auto-launches `slam.launch.py` on startup

To run in localization mode instead, edit `run.sh` and pass the `slam_config` argument:

```bash
/ros_entrypoint.sh ros2 launch g1_slam slam.launch.py \
  slam_config:=/opt/ws/install/g1_slam/share/g1_slam/config/slam_toolbox_localize.yaml
```

### Open a shell inside the running container

```bash
docker exec -it g1_slam bash
```

---

## Native Setup (without Docker)

### Requirements

- ROS 2 Humble
- `slam_toolbox`
- `pointcloud_to_laserscan`
- `tf2_ros`
- `nav2_bt_navigator`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`
- `nav2_waypoint_follower`, `nav2_velocity_smoother`, `nav2_lifecycle_manager`
- `nav2_navfn_planner`, `dwb_core`
- RViz2

### Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <REPOSITORY_URL>/g1_slam.git
cd ..
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select g1_slam --symlink-install
source install/setup.bash
```

---

## SLAM

### Background — why custom nodes are needed

The G1 has two clock issues that prevent standard SLAM from working out of the box:

1. **`/dog_odom` is ~61s behind the system clock** — `odom_to_tf` measures this offset
   on the first message and applies it to all subsequent TF publishes, keeping
   `odom → base_link` aligned with the rest of the system.

2. **Livox hardware clock (PTP) is ahead of `/dog_odom`** — `restamp_cloud` waits for
   the offset to be measured and stamps each cloud with `ros::now()` to align
   with the corrected TF.

Additionally, the G1 never publishes `odom → base_link` in its TF tree. `odom_to_tf`
provides this missing link using `/dog_odom` as the source.

### TF tree after launch

```
map → odom → base_link → pelvis → ... → torso_link → mid360_link → livox_frame
 ↑              ↑                                           ↑
slam_toolbox  odom_to_tf                         G1 robot_state_publisher
(g1_slam)     (g1_slam)
```

### Mapping mode (default)

Builds a new map from scratch.

```bash
ros2 launch g1_slam slam.launch.py
```

With rosbag replay:

```bash
ros2 bag play <bag> --clock
ros2 launch g1_slam slam.launch.py use_sim_time:=true
```

### Localization mode

Loads a previously saved map and localizes within it without modifying it.

```bash
ros2 launch g1_slam slam.launch.py \
  slam_config:=$(ros2 pkg prefix g1_slam)/share/g1_slam/config/slam_toolbox_localize.yaml
```

### Saving the map

Run these while the SLAM node is active:

```bash
# Nav2 format (.pgm + .yaml)
ros2 run nav2_map_server map_saver_cli \
  -f /home/unitree/maps/map_robotspace \
  --ros-args -p map_subscribe_transient_local:=true

# slam_toolbox format (.posegraph) — required for localization mode
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/unitree/maps/map_robotspace'}"
```

Update `map_file_name` in `slam_toolbox_localize.yaml` if you save the map elsewhere.

---

## Navigation (Nav2)

Navigation requires SLAM to be running first. The Nav2 stack uses:
- **NavFn (Dijkstra)** for global path planning
- **DWB** for local path following and obstacle avoidance
- **`/inorbit/odom_pose`** as the robot pose source
- **`/cmd_vel`** as the velocity output to the G1 locomotion controller

### Run

```bash
# Terminal 1 — SLAM
ros2 launch g1_slam slam.launch.py

# Terminal 2 — Nav2
ros2 launch g1_slam nav.launch.py
```

### Send a goal

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Or use the **Nav2 Goal** button in RViz.

### Navigation works in both SLAM modes

| Feature | mapping | localization |
|---|---|---|
| Navigate to goal | ✅ | ✅ |
| Map grows with movement | ✅ | ❌ |
| Loop closing | ✅ | ❌ |
| Loads saved map on start | ❌ | ✅ |

---

## Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/livox/lidar` | `PointCloud2` | Raw Livox 3D point cloud |
| Input | `/dog_odom` | `Odometry` | G1 kinematic odometry (~61s clock offset) |
| Internal | `/livox/lidar_restamped` | `PointCloud2` | Clock-corrected point cloud |
| Internal | `/dog_odom_restamped` | `Odometry` | Clock-corrected odometry |
| Output | `/scan` | `LaserScan` | 2D scan for slam_toolbox and Nav2 |
| Output | `/map` | `OccupancyGrid` | Occupancy map from slam_toolbox |
| Output | `/inorbit/odom_pose` | `Odometry` | Robot pose in map frame |
| Output | `/cmd_vel` | `Twist` | Velocity commands to G1 locomotion controller |

---

## Launch Arguments

### `slam.launch.py`

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Set `true` for rosbag replay with `--clock` |
| `slam_config` | `config/slam_toolbox.yaml` | Path to slam_toolbox YAML — swap for localize YAML |
| `pc_config` | `config/pc_to_laserscan.yaml` | Path to pointcloud_to_laserscan YAML |

### `nav.launch.py`

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Set `true` for rosbag replay with `--clock` |
| `params_file` | `config/nav2_params.yaml` | Path to Nav2 parameters YAML |

---

## Key Configuration Parameters

### `slam_toolbox.yaml` / `slam_toolbox_localize.yaml`

| Parameter | Value | Description |
|---|---|---|
| `mode` | `mapping` / `localization` | Build new map or localize on saved map |
| `map_file_name` | `/home/unitree/maps/map_robotspace` | Map to load (localization only) |
| `coarse_search_angle_offset` | `0.785` | ~45° search window to correct yaw drift |
| `resolution` | `0.05` | Map cell size [m] |
| `throttle_scans` | `2` | Process 1 out of every N scans |

### `pc_to_laserscan.yaml`

| Parameter | Value | Description |
|---|---|---|
| `target_frame` | `livox_frame` | Project in sensor frame — no extra TF lookup |
| `min_height` / `max_height` | `-1.0` / `1.0` | Vertical slice relative to `livox_frame` (~1.17m above floor) |
| `range_max` | `50.0` | Maximum scan range [m] |

### `nav2_params.yaml`

| Parameter | Value | Description |
|---|---|---|
| `max_vel_x` | `0.30` | Maximum forward speed [m/s] |
| `robot_radius` | `0.35` | G1 footprint radius [m] |
| `inflation_radius` | `0.55` | Obstacle inflation for costmap [m] |
| `xy_goal_tolerance` | `0.35` | Goal acceptance radius [m] |

---

## Verification

```bash
# Check all nodes are running
ros2 node list

# Verify the full TF chain is present
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom livox_frame

# Check topic rates
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic hz /inorbit/odom_pose

# Confirm clock offset is being corrected (look for offset log on startup)
ros2 topic echo /rosout | grep offset
```

---

## Known Limitations

- The `/dog_odom` clock offset (~61s) is measured once at startup. If the robot
  restarts without restarting `g1_slam`, the nodes must be restarted to re-measure.
- Sensor fusion (EKF) is intentionally not part of this package.
- The G1 yaw odometry may drift over long distances. The scan matcher is configured
  with a wide angular search window (~45°) to compensate, but significant drift may
  still affect map quality.
- The Livox Mid360 is mounted upside-down on the G1 (`180° roll` in
  `mid360_link → livox_frame`). This is handled transparently by projecting the
  scan directly in `livox_frame`.

---

## Status

Active development.
