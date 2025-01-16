import rclpy
from rclpy.node import Node


class MyParameterNode(Node):

    def __init__(self):
        super().__init__("my_parameter_node")

        self.declare_parameter("my_parameter", 123)
      
        self.tmr = self.create_timer(1, self.onTick)
    
    def onTick(self):
    
        my_parameter = self.get_parameter("my_parameter").value
       
        self.get_logger().info("my_parameter : {0}".format(my_parameter))


def main(args=None):

    rclpy.init(args=args)

    node = MyParameterNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()