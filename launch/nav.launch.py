"""
nav.launch.py
=============
Launches Nav2 for the Unitree G1 humanoid robot.

Requires g1_slam to be running first:
  ros2 launch g1_slam slam.launch.py

Nav2 stack started:
  - controller_server    (DWB local planner -> /cmd_vel)
  - planner_server       (NavFn / Dijkstra global planner)
  - behavior_server      (spin, backup, wait)
  - bt_navigator         (behavior tree navigation)
  - waypoint_follower    (follows a list of waypoints)
  - velocity_smoother    (smooths cmd_vel output)
  - lifecycle_manager    (manages all Nav2 nodes)

Send a single goal from terminal:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, \
      pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}"

Arguments:
  use_sim_time   [true|false]  default: false
  params_file    path to nav2_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


PKG = "g1_slam"


def _pkg_path(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


def generate_launch_description():

    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    arg_params = DeclareLaunchArgument(
        "params_file",
        default_value=_pkg_path("config", "nav2_params.yaml"),
        description="Path to Nav2 parameters YAML file.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file  = LaunchConfiguration("params_file")

    nav2_params = [params_file, {"use_sim_time": use_sim_time}]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=nav2_params,
        remappings=[("cmd_vel", "/cmd_vel_nav")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=nav2_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        output="screen",
        parameters=nav2_params,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=nav2_params,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=nav2_params,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=nav2_params,
        remappings=[
            ("cmd_vel",          "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel"),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart":    True},
            {"node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ]},
        ],
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_params,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])