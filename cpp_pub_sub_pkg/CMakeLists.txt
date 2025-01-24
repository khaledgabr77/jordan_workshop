cmake_minimum_required(VERSION 3.8)
project(cpp_pub_sub_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher_hello_ros src/publisher_hello_ros.cpp)
add_executable(subscriber_hello_ros src/subscriber_hello_ros.cpp)

ament_target_dependencies(publisher_hello_ros rclcpp std_msgs)
ament_target_dependencies(subscriber_hello_ros rclcpp std_msgs)

install(TARGETS
  publisher_hello_ros
  subscriber_hello_ros
  DESTINATION lib/${PROJECT_NAME})

  
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)
ament_package()
