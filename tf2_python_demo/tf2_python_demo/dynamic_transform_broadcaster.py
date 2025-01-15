#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class DynamicTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_transform_broadcaster')

        # Create a transform broadcaster object
        self._broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for updating the transform
        self.timer = self.create_timer(0.1, self.timer_callback)  # Update every 0.1 seconds

        # Initialize parameters for motion
        self.angle = 0.0  # Initial angle
        self.radius = 5.0  # Radius of circular motion
        self.angular_velocity = 0.1  # Angular velocity (radians per second)

    def timer_callback(self):
        # Update angle for circular motion
        self.angle += self.angular_velocity * 0.1  # Increment angle based on time step (0.1s)
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

        # Calculate position in circular path
        x = self.radius * math.cos(self.angle)
        y = self.radius * math.sin(self.angle)
        z = 0.0  # Keep z constant

        # Create the transform
        dynamic_transform = TransformStamped()
        dynamic_transform.header.stamp = self.get_clock().now().to_msg()
        dynamic_transform.header.frame_id = 'world'
        dynamic_transform.child_frame_id = 'demo_frame'

        # Set translation (position)
        dynamic_transform.transform.translation.x = x
        dynamic_transform.transform.translation.y = y
        dynamic_transform.transform.translation.z = z

        # Set rotation (no rotation in this case, identity quaternion)
        dynamic_transform.transform.rotation.x = 0.0
        dynamic_transform.transform.rotation.y = 0.0
        dynamic_transform.transform.rotation.z = 0.0
        dynamic_transform.transform.rotation.w = 1.0

        # Publish the transform
        self._broadcaster.sendTransform(dynamic_transform)

        # Log the updated position
        self.get_logger().info(
            f"Transform updated: world -> demo_frame | Position (x, y, z) = ({x:.2f}, {y:.2f}, {z:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
