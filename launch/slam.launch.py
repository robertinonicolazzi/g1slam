"""
slam.launch.py
==============
SLAM launch file for the Unitree G1 humanoid robot with Livox Mid360 LiDAR.

Clock offset issue:
  dog_odom publishes with a constant ~61s offset behind the system clock.
  odom_to_tf and restamp_cloud measure and compensate this offset automatically.

TF tree after this launch:
  map -> odom -> base_link -> pelvis -> ... -> mid360_link -> livox_frame

Topics published for the Dijkstra planner:
  /map          (nav_msgs/OccupancyGrid)  <- slam_toolbox
  /inorbit/odom_pose  (nav_msgs/Odometry)       <- pose_publisher (map->base_link)

Dijkstra planner configuration:
  map_topic:  '/map'
  odom_topic: '/inorbit/odom_pose'

Nodes started:
  1. odom_to_tf              - /dog_odom -> TF odom->base_link
  2. restamp_cloud           - /livox/lidar -> /livox/lidar_restamped
  3. pointcloud_to_laserscan - /livox/lidar_restamped -> /scan
  4. slam_toolbox            - /scan -> /map + TF map->odom
  5. pose_publisher          - TF map->base_link -> /inorbit/odom_pose

Arguments:
  use_sim_time  [true|false]  default: false
  slam_config   path to slam_toolbox YAML
  pc_config     path to pointcloud_to_laserscan YAML
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

PKG = "g1_slam"


def _pkg_path(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


def generate_launch_description():

    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="true = rosbag with /clock, false = real robot.",
    )
    arg_slam_config = DeclareLaunchArgument(
        "slam_config",
        default_value=_pkg_path("config", "slam_toolbox.yaml"),
        description="Path to slam_toolbox YAML configuration file.",
    )
    arg_pc_config = DeclareLaunchArgument(
        "pc_config",
        default_value=_pkg_path("config", "pc_to_laserscan.yaml"),
        description="Path to pointcloud_to_laserscan YAML configuration file.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config  = LaunchConfiguration("slam_config")
    pc_config    = LaunchConfiguration("pc_config")

    # ── 1. odom -> base_link TF ──────────────────────────────────────────────
    # Converts /dog_odom into a TF transform with corrected timestamp.
    odom_to_tf_node = Node(
        package="g1_slam",
        executable="odom_to_tf.py",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 2. LiDAR re-stamp ────────────────────────────────────────────────────
    # Aligns Livox hardware clock with the corrected system clock.
    restamp_cloud_node = Node(
        package="g1_slam",
        executable="restamp_cloud.py",
        name="restamp_cloud",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 3. PointCloud2 -> LaserScan ──────────────────────────────────────────
    # Slices the 3D point cloud into a 2D scan for slam_toolbox.
    pc_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pc_to_ls",
        output="screen",
        parameters=[
            pc_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", "/livox/lidar_restamped"),
            ("scan",     "/scan"),
        ],
    )

    # ── 4. slam_toolbox ──────────────────────────────────────────────────────
    # Builds the occupancy map and publishes map->odom TF.
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── 5. Pose publisher ────────────────────────────────────────────────────
    # Converts TF map->base_link into Odometry for the Dijkstra planner.
    pose_publisher_node = Node(
        package="g1_slam",
        executable="pose_publisher.py",
        name="pose_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_slam_config,
        arg_pc_config,
        odom_to_tf_node,
        restamp_cloud_node,
        pc_to_laserscan_node,
        slam_node,
        pose_publisher_node,
    ])