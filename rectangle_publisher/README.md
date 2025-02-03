To create a **ROS 2 package for custom messages** using **C++**, follow these steps:

---

## **1️⃣ Create the Message Package**
Run the following command:
```bash
ros2 pkg create --build-type ament_cmake custom_msgs
```
This will create a package called `custom_msgs`.

---

## **2️⃣ Create the `msg` Directory and Message File**
Navigate to the package directory:
```bash
cd custom_msgs
mkdir msg
touch msg/RectangleDimensions.msg
```

Edit `msg/RectangleDimensions.msg` and add:
```plaintext
float64 length
float64 width
```

---

## **3️⃣ Modify `package.xml`**
Edit `package.xml` and **add the required dependencies**:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>custom_msgs</name>
  <version>0.0.1</version>
  <description>Custom message package for ROS 2</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- TODO: Add build tool dependencies for message generation -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <!-- TODO: Add runtime dependencies -->
  <depend>rosidl_default_runtime</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <member_of_group>rosidl_interface_packages</member_of_group>
  </export>
</package>
```

---

## **4️⃣ Modify `CMakeLists.txt`**
Edit `CMakeLists.txt` and **add support for message generation**:

```cmake
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
ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

---

## **5️⃣ Build the Package**
Run:
```bash
colcon build --packages-select custom_msgs
source install/setup.bash
```
Ensure there are no errors.

---

## **6️⃣ Create a ROS 2 C++ Package to Use the Message**
Now, create a **C++ package** to use the custom message:
```bash
ros2 pkg create --build-type ament_cmake rectangle_publisher --dependencies rclcpp custom_msgs
```

---

## **7️⃣ Modify `package.xml` for `rectangle_publisher`**
Edit `package.xml` to **include the `custom_msgs` dependency**:
```xml
<depend>custom_msgs</depend>
```

---

## **8️⃣ Modify `CMakeLists.txt` for `rectangle_publisher`**
Edit `CMakeLists.txt`:
```cmake
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
```

---

## **9️⃣ Create the Publisher Node**
Create the file `src/rectangle_publisher.cpp`:
```cpp
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
```

---

## **🔟 Build and Run the Publisher**
1. **Build the package**:
```bash
colcon build --packages-select rectangle_publisher
source install/setup.bash
```

2. **Run the publisher**:
```bash
ros2 run rectangle_publisher rectangle_publisher_node
```

3. **Verify with `ros2 topic echo`**:
```bash
ros2 topic echo /rectangle_dimensions
```

You should see:
```plaintext
length: 5.0
width: 3.0
```

---

