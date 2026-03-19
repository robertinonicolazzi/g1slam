#!/usr/bin/env python3
"""
restamp_cloud.py
================
Re-publishes the Livox point cloud with a corrected timestamp.

The Livox publishes with a hardware PTP clock aligned with the system.
The dog_odom publishes ~61s behind the system clock.
The odom_to_tf corrects dog_odom by adding the measured offset.

To keep the scan and the odom->base_link TF in the same time reference,
the scan timestamp is replaced with ros::now(), which matches the
corrected TF timestamp published by odom_to_tf.

Subscriptions:
  /livox/lidar   (sensor_msgs/PointCloud2)
  /dog_odom      (nav_msgs/Odometry)
Publications:
  /livox/lidar_restamped (sensor_msgs/PointCloud2)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time


class RestampCloud(Node):

    def __init__(self):
        super().__init__("restamp_cloud")

        self._offset_ns = None

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pub = self.create_publisher(
            PointCloud2, "/livox/lidar_restamped", qos_rel
        )
        self._sub_cloud = self.create_subscription(
            PointCloud2, "/livox/lidar", self._cloud_cb, qos_be
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/dog_odom", self._odom_cb, qos_be
        )
        self.get_logger().info(
            "restamp_cloud: waiting for dog_odom to measure clock offset..."
        )

    def _stamp_to_ns(self, stamp) -> int:
        return stamp.sec * 10**9 + stamp.nanosec

    def _ns_to_stamp(self, ns: int) -> Time:
        t = Time()
        t.sec = int(ns // 10**9)
        t.nanosec = int(ns % 10**9)
        return t

    def _odom_cb(self, msg: Odometry):
        if self._offset_ns is None:
            now_ns = self.get_clock().now().nanoseconds
            msg_ns = self._stamp_to_ns(msg.header.stamp)
            self._offset_ns = now_ns - msg_ns
            self.get_logger().info(
                f"restamp_cloud: clock offset measured = {self._offset_ns / 1e9:.3f}s"
            )

    def _cloud_cb(self, msg: PointCloud2):
        # Wait until the clock offset has been measured
        if self._offset_ns is None:
            return

        # Use ros::now() which matches the corrected TF timestamp
        now_ns = self.get_clock().now().nanoseconds
        msg.header.stamp = self._ns_to_stamp(now_ns)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RestampCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()