#!/usr/bin/env python3
"""
odom_to_tf.py
=============
Converts /dog_odom (nav_msgs/Odometry) into a TF transform: odom -> base_link.

The Unitree G1 publishes /dog_odom with a constant clock offset of ~61s
behind the system clock. The offset is measured automatically on the first
message and applied to all subsequent messages, keeping temporal consistency
with the rest of the system (TF tree, scan, etc).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time


class OdomToTf(Node):

    def __init__(self):
        super().__init__("odom_to_tf")

        self._br = TransformBroadcaster(self)
        self._offset_ns = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub = self.create_subscription(
            Odometry, "/dog_odom", self._cb, qos
        )
        self.get_logger().info(
            "odom_to_tf: waiting for first message to measure clock offset..."
        )

    def _stamp_to_ns(self, stamp) -> int:
        return stamp.sec * 10**9 + stamp.nanosec

    def _ns_to_stamp(self, ns: int) -> Time:
        t = Time()
        t.sec = ns // 10**9
        t.nanosec = ns % 10**9
        return t

    def _cb(self, msg: Odometry):
        now_ns = self.get_clock().now().nanoseconds
        msg_ns = self._stamp_to_ns(msg.header.stamp)

        # Measure the clock offset once on the first message
        if self._offset_ns is None:
            self._offset_ns = now_ns - msg_ns
            self.get_logger().info(
                f"odom_to_tf: clock offset measured = {self._offset_ns / 1e9:.3f}s"
            )

        # Apply offset while preserving the delta between consecutive messages
        corrected_ns = msg_ns + self._offset_ns

        t = TransformStamped()
        t.header.stamp    = self._ns_to_stamp(corrected_ns)
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation

        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()