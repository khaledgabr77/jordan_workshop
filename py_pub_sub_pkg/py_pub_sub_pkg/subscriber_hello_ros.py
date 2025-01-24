#!/usr/bin/env python3
# This is a shebang line that tells the operating system to use Python 3 to run this script.

import rclpy
# Import the rclpy library, which is the ROS 2 Python client library.

from rclpy.node import Node
# Import the Node class from rclpy, used to create ROS 2 nodes.

from std_msgs.msg import String
# Import the String message type from the std_msgs package, used to publish and subscribe to string messages.

class SubscriberNode(Node):
    # Define a class that inherits from Node, representing a ROS 2 subscriber node.
    def __init__(self):
        # Constructor for the SubscriberNode class.
        super().__init__('subscriber_node')
        # Call the parent class constructor and name this node 'subscriber_node'.
        
        self.subscription = self.create_subscription(
            String,              # Message type for the subscription (std_msgs/String).
            'chatter',           # Topic name to subscribe to ('chatter').
            self.listener_callback, # Callback function to handle received messages.
            10                   # Queue size for messages (10).
        )
        self.subscription
        # Retain the subscription object to prevent it from being garbage collected.

    def listener_callback(self, msg):
        # Callback function that gets executed when a message is received on the 'chatter' topic.
        self.get_logger().info(f"Received: '{msg.data}'")
        # Log the received message to the console.

def main(args=None):
    # Define the main function.
    rclpy.init(args=args)
    # Initialize the rclpy library.
    
    node = SubscriberNode()
    # Create an instance of the SubscriberNode class.
    
    rclpy.spin(node)
    # Keep the node running and listening for messages.
    
    node.destroy_node()
    # Destroy the node when done to clean up resources.
    
    rclpy.shutdown()
    # Shut down the rclpy library to release resources.

if __name__ == '__main__':
    main()