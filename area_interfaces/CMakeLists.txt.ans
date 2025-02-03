cmake_minimum_required(VERSION 3.8)
project(area_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalculateArea.srv"
)

# ament_export_dependencies(rosidl_default_runtime)
ament_package()
