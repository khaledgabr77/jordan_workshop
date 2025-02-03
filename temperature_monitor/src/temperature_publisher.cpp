#include <chrono>
#include <memory>
#include <random>

// TODO: Include the necessary ROS 2 header for the Node class and publisher


// TODO: Include the Temperature message header


using namespace std::chrono_literals;

class TemperatureSensorNode : public rclcpp::Node
{
public:
  TemperatureSensorNode()
  : Node("temperature_sensor_node"),
    generator_(rd_()),
    distribution_(15.0, 25.0)
  {
    // TODO: Create a publisher for sensor_msgs::msg::Temperature
    

    // Create a timer that calls publishTemperature every 1 second
    timer_ = this->create_wall_timer(
      1000ms,
      std::bind(&TemperatureSensorNode::publishTemperature, this)
    );
  }

private:
  void publishTemperature()
  {
    // Generate a random temperature between 15.0 and 25.0
    double random_temp = distribution_(generator_);

    // TODO: Create a Temperature message
   

    // TODO: Assign the random_temp to the temperature field
    

    // Set the variance (example: 0.2)
   

    // Optional: Set header time stamp to current ROS time
    

    // TODO: Log the temperature you're about to publish (use RCLCPP_INFO)
    

    // TODO: Publish the message
   
  }

  // Member variables
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Random temperature generation
  std::random_device rd_;
  std::mt19937 generator_;
  std::uniform_real_distribution<double> distribution_;
};

int main(int argc, char * argv[])
{
  // Initialize rclcpp
  rclcpp::init(argc, argv);

  // TODO: Spin the node (Hint: use std::make_shared<TemperatureSensorNode>())
  

  // Shutdown rclcpp
  rclcpp::shutdown();
  return 0;
}
