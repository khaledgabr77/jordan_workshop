#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')

        # Buffer will store the transforms
        self.tf_buffer = tf2_ros.Buffer()

        # Transform listener will subscribe to the /tf topic
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # We'll use a timer to periodically lookup the transform
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # lookup_transform(target_frame, source_frame, time)
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'world',
                'demo_frame',
                now
            )

            # Extract translation
            translation = trans.transform.translation
            x, y, z = translation.x, translation.y, translation.z

            # Extract rotation in quaternion form
            rot = trans.transform.rotation
            q = [rot.x, rot.y, rot.z, rot.w]

            # Convert quaternion to roll, pitch, yaw
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)

            self.get_logger().info(
                f"Transform: world -> demo_frame | "
                f"Position (x, y, z) = ({x:.2f}, {y:.2f}, {z:.2f}) | "
                f"Orientation (r, p, y) = ({roll:.2f}, {pitch:.2f}, {yaw:.2f})"
            )

        except tf2_ros.LookupException:
            self.get_logger().info('Transform not available yet...')

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
