import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        qos_profile = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )
        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Message {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = QoSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
