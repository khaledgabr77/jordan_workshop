cmake_minimum_required(VERSION 3.8)
project(rectangle_publisher)

# TODO: Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)

# TODO: Add the publisher executable
add_executable(rectangle_publisher_node src/rectangle_publisher.cpp)
ament_target_dependencies(rectangle_publisher_node rclcpp custom_msgs)

# TODO: Install targets
install(TARGETS
  rectangle_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
