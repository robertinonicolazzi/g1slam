"""
rviz.launch.py  –  Open RViz2 pre-configured for humanoid SLAM visualisation.

Usage:
  ros2 launch g1_slam rviz.launch.py
  ros2 launch g1_slam rviz.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("g1_slam")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(pkg, "rviz", "g1_slam.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])