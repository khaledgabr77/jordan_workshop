// Subscriber Node
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node") {
        // Create a subscription to the "chatter" topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10, std::bind(&SubscriberNode::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Log the received message
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    // Initialize the rclcpp library
    rclcpp::init(argc, argv);

    // Create a shared pointer for the subscriber node
    auto node = std::make_shared<SubscriberNode>();

    // Spin the node to keep it running and processing callbacks
    rclcpp::spin(node);

    // Shutdown and clean up resources
    rclcpp::shutdown();
    return 0;
}