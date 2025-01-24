# Publisher Node
#!/usr/bin/env python3
# Shebang line indicating to use Python 3 to run this script.

import rclpy
# Import the rclpy library.

from rclpy.node import Node
# Import the Node class for creating ROS 2 nodes.

from std_msgs.msg import String
# Import the String message type for publishing messages.

class PublisherNode(Node):
    # Define a class that inherits from Node, representing a ROS 2 publisher node.
    def __init__(self):
        # Constructor for the PublisherNode class.
        super().__init__('publisher_node')
        # Call the parent class constructor and name this node 'publisher_node'.

        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # Create a publisher to send String messages to the 'chatter' topic with a queue size of 10.
        
        self.timer = self.create_timer(1.0, self.publish_message)
        # Create a timer that triggers the publish_message method every 1.0 seconds.
        
        self.counter = 0
        # Initialize a counter to keep track of the number of messages published.

    def publish_message(self):
        # Method to publish messages to the 'chatter' topic.
        msg = String()
        # Create a new String message.
        
        msg.data = f"Hello ROS 2! Counter: {self.counter}"
        # Set the content of the message to include the counter value.
        
        self.publisher_.publish(msg)
        # Publish the message to the 'chatter' topic.
        
        self.get_logger().info(f"Publishing: '{msg.data}'")
        # Log the published message to the console.
        
        self.counter += 1
        # Increment the counter for the next message.

def main(args=None):
    # Define the main function.
    rclpy.init(args=args)
    # Initialize the rclpy library.
    
    node = PublisherNode()
    # Create an instance of the PublisherNode class.
    
    rclpy.spin(node)
    # Keep the node running and publishing messages periodically.
    
    node.destroy_node()
    # Destroy the node when done to clean up resources.
    
    rclpy.shutdown()
    # Shut down the rclpy library to release resources.

if __name__ == '__main__':
    main()
