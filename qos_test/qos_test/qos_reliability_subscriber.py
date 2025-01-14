import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class QoSReliabilitySubscriber(Node):
    def __init__(self, reliability_policy):
        super().__init__('qos_reliability_subscriber')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # Required setting
            depth=10,  # Required for KEEP_LAST
            reliability=reliability_policy,
        )
        self.subscription = self.create_subscription(
            String,
            'qos_reliability_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info(f"Using Reliability Policy: {reliability_policy}")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")


def main():
    rclpy.init()

    # Toggle between RELIABLE and BEST_EFFORT
    use_reliable = True
    reliability_policy = ReliabilityPolicy.RELIABLE if use_reliable else ReliabilityPolicy.BEST_EFFORT

    node = QoSReliabilitySubscriber(reliability_policy)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Subscriber stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
