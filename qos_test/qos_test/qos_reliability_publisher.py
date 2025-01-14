import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class QoSReliabilityPublisher(Node):
    def __init__(self):
        super().__init__('qos_reliability_publisher')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # Required setting
            depth=10,  # Required for KEEP_LAST
            reliability=ReliabilityPolicy.RELIABLE,  # RELIABLE or BEST_EFFORT
        )
        self.publisher_ = self.create_publisher(String, 'qos_reliability_topic', qos_profile)
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5 seconds
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Message {self.count}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.count += 1


def main():
    rclpy.init()
    node = QoSReliabilityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Publisher stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
