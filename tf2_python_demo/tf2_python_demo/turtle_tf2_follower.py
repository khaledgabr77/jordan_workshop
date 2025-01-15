import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleFollowerNode(Node):
    def __init__(self):
        super().__init__('turtle_follower_node')

        # Subscriber for Turtle 1's pose
        self.turtle1_pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)

        # Subscriber for Turtle 2's pose
        self.turtle2_pose_sub = self.create_subscription(
            Pose, '/turtle2/pose', self.turtle2_pose_callback, 10)

        # Publisher for Turtle 2's velocity
        self.turtle2_cmd_vel_pub = self.create_publisher(
            Twist, '/turtle2/cmd_vel', 10)

        # Turtle poses
        self.turtle1_pose = None
        self.turtle2_pose = None

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg

    def turtle2_pose_callback(self, msg):
        self.turtle2_pose = msg

    def control_loop(self):
        if self.turtle1_pose is None or self.turtle2_pose is None:
            return

        # Calculate the distance and angle to Turtle 1
        distance = math.sqrt(
            (self.turtle1_pose.x - self.turtle2_pose.x) ** 2 +
            (self.turtle1_pose.y - self.turtle2_pose.y) ** 2
        )
        angle_to_target = math.atan2(
            self.turtle1_pose.y - self.turtle2_pose.y,
            self.turtle1_pose.x - self.turtle2_pose.x
        )

        # Calculate angle difference
        angle_diff = angle_to_target - self.turtle2_pose.theta

        # Normalize angle difference to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Create velocity command
        cmd_vel = Twist()

        # Proportional controller for linear and angular velocity
        linear_speed = 1.0 * distance  # Gain of 1.0
        angular_speed = 4.0 * angle_diff  # Gain of 4.0

        # Apply limits to the velocities
        cmd_vel.linear.x = min(linear_speed, 2.0)  # Limit linear speed to 2.0
        cmd_vel.angular.z = max(min(angular_speed, 2.0), -2.0)  # Limit angular speed to [-2.0, 2.0]

        # Publish the velocity command
        self.turtle2_cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
