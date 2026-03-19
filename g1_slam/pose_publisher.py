#!/usr/bin/env python3
"""
pose_publisher.py
=================
Publishes the robot pose in the map frame as nav_msgs/Odometry.

The Dijkstra planner requires an Odometry topic with the robot pose
expressed in the map frame. This node reads the TF transform
map -> base_link and republishes it as an Odometry message.

Publications:
  /inorbit/odom_pose  (nav_msgs/Odometry, frame_id: map, child: base_link)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class PosePublisher(Node):

    def __init__(self):
        super().__init__("pose_publisher")

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub   = self.create_publisher(Odometry, "/inorbit/odom_pose", qos)
        self._timer = self.create_timer(0.02, self._publish)  # 50 Hz

        self.get_logger().info(
            "pose_publisher: publishing robot pose on /inorbit/odom_pose (map -> base_link)"
        )

    def _publish(self):
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        msg = Odometry()
        msg.header.stamp    = tf.header.stamp
        msg.header.frame_id = "map"
        msg.child_frame_id  = "base_link"

        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()