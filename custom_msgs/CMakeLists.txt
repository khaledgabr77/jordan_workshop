cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

# TODO: Find necessary ROS 2 packages for message generation
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# TODO: Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RectangleDimensions.msg"
)

# TODO: Export runtime dependencies
# ament_export_dependencies(rosidl_default_runtime)

ament_package()
