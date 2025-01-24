// Publisher Node
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("publisher_node"), counter_(0) {
        // Create a publisher to the "chatter" topic
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // Create a timer to publish messages every 1 second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&PublisherNode::publish_message, this));
    }

private:
    void publish_message() {
        // Create and populate the message
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS 2! Counter: " + std::to_string(counter_);

        // Publish the message
        publisher_->publish(msg);

        // Log the message
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());

        // Increment the counter
        counter_++;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv) {
    // Initialize the rclcpp library
    rclcpp::init(argc, argv);

    // Create a shared pointer for the publisher node
    auto node = std::make_shared<PublisherNode>();

    // Spin the node to keep it running and publishing messages
    rclcpp::spin(node);

    // Shutdown and clean up resources
    rclcpp::shutdown();
    return 0;
}