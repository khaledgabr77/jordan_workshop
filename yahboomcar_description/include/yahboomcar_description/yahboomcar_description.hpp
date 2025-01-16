#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace yahboomcar_description {



class YahboomcarDescription : public rclcpp::Node {

 public:

  YahboomcarDescription();

 private:

  void setup();

 private:
};


}
