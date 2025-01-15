#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time

class StaticTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('static_transform_broadcaster')

        # Create a static broadcaster object
        self._broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Define the static transform
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'
        static_transform.child_frame_id = 'demo_frame'

        # Translation (x, y, z)
        # static_transform.transform.translation.x = -1.0
        # static_transform.transform.translation.y = -2.0
        # static_transform.transform.translation.z = 3.0

        static_transform.transform.translation.x = 1.0
        static_transform.transform.translation.y = 2.0
        static_transform.transform.translation.z = 3.0
        # Rotation (Quaternion) - example: rotate 90 degrees around Z axis
        # Convert from Euler angles (roll, pitch, yaw) to quaternion if needed
        # For demonstration, we'll just set an arbitrary orientation
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Publish the static transform
        self._broadcaster.sendTransform(static_transform)
        self.get_logger().info('Static transform published from "world" to "demo_frame"')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformBroadcaster()
    # Spin once and exit, because static transforms are latched
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
