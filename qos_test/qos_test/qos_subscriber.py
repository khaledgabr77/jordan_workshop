import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy
import time


class QoSSubscriber(Node):
    def __init__(self, history_policy):
        super().__init__('qos_subscriber')
        qos_profile = QoSProfile(
            depth=5,  # Keep only the last 5 messages in KEEP_LAST
            history=history_policy
        )
        self.subscription = self.create_subscription(
            String,
            'qos_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info(f'Using History Policy: {history_policy}')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        # Simulate processing delay
        time.sleep(2)


def main():
    rclpy.init()

    # Toggle between KEEP_LAST and KEEP_ALL to observe behavior
    use_keep_last = True
    history_policy = HistoryPolicy.KEEP_LAST if use_keep_last else HistoryPolicy.KEEP_ALL

    node = QoSSubscriber(history_policy)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
