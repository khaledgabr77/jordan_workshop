#include <memory>

// TODO: Include the necessary ROS 2 header for the Node class and subscriber
// #include ??? (Hint: "rclcpp/rclcpp.hpp")

// TODO: Include the Temperature message header
// #include ??? (Hint: "sensor_msgs/msg/temperature.hpp")

class TemperatureMonitorNode : public rclcpp::Node
{
public:
  TemperatureMonitorNode()
  : Node("temperature_monitor_node")
  {
    // TODO: Create a subscription to the topic "temperature" with queue size 10
    // subscription_ = this->create_subscription<...>(
    //   "temperature", 10,
    //   std::bind(&TemperatureMonitorNode::topicCallback, this, std::placeholders::_1)
    // );
  }

private:
  // This function is called every time a new message is received
  void topicCallback(const sensor_msgs::msg::Temperature::SharedPtr msg)
  {
    // TODO: Log the temperature and variance
    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "...",
    //   msg->temperature,
    //   msg->variance
    // );
  }

  // Member variable
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Initialize rclcpp
  rclcpp::init(argc, argv);

  // TODO: Spin the node (Hint: use std::make_shared<TemperatureMonitorNode>())
  // rclcpp::spin(...);

  // Shutdown rclcpp
  rclcpp::shutdown();
  return 0;
}
