#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/rectangle_dimensions.hpp"

class RectanglePublisher : public rclcpp::Node {
public:
    RectanglePublisher() : Node("rectangle_publisher") {
        publisher_ = this->create_publisher<custom_msgs::msg::RectangleDimensions>("rectangle_dimensions", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RectanglePublisher::publish_rectangle, this)
        );
    }

private:
    void publish_rectangle() {
        auto message = custom_msgs::msg::RectangleDimensions();
        message.length = 5.0;
        message.width = 3.0;

        RCLCPP_INFO(this->get_logger(), "Publishing: Length=%.2f, Width=%.2f", message.length, message.width);
        publisher_->publish(message);
    }

    rclcpp::Publisher<custom_msgs::msg::RectangleDimensions>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RectanglePublisher>());
    rclcpp::shutdown();
    return 0;
}
