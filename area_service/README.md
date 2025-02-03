
### **1️⃣ Create an Interface Package for `srv` Definitions**
Run:
```bash
ros2 pkg create --build-type ament_cmake area_interfaces
```

Navigate to the package directory:
```bash
cd area_interfaces
mkdir srv
touch srv/CalculateArea.srv
```

Edit `srv/CalculateArea.srv`:
```plaintext
float64 length
float64 width
---
float64 area
```

### **2️⃣ Update `package.xml` for `area_interfaces`**
Modify `package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>area_interfaces</name>
  <version>0.0.1</version>
  <description>ROS2 interface package for area calculation services</description>

  <maintainer email="khaledgabr77@gmail.com">Khaled</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rosidl_default_runtime</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <member_of_group>rosidl_interface_packages</member_of_group>
  </export>
</package>
```

### **3️⃣ Update `CMakeLists.txt` for `area_interfaces`**
Edit `CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(area_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalculateArea.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

Build the interface package:
```bash
colcon build --packages-select area_interfaces
source install/setup.bash
```

---

## **4️⃣ Create the Main Service Package**
Now, create the actual service package:
```bash
ros2 pkg create --build-type ament_cmake area_service --dependencies rclcpp area_interfaces
```

### **5️⃣ Update `package.xml` for `area_service`**
Modify `package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>area_service</name>
  <version>0.0.1</version>
  <description>ROS2 package providing an area calculation service</description>

  <maintainer email="khaledgabr77@gmail.com">Khaled</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>area_interfaces</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

### **6️⃣ Update `CMakeLists.txt` for `area_service`**
Modify `CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(area_service)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(area_interfaces REQUIRED)

add_executable(area_server src/area_server.cpp)
ament_target_dependencies(area_server rclcpp area_interfaces)

add_executable(area_client src/area_client.cpp)
ament_target_dependencies(area_client rclcpp area_interfaces)

install(TARGETS
  area_server
  area_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

### **7️⃣ Implement the Service Server**
Create `src/area_server.cpp`:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "area_interfaces/srv/calculate_area.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AreaService : public rclcpp::Node {
public:
    AreaService() : Node("area_service_server") {
        service_ = this->create_service<area_interfaces::srv::CalculateArea>(
            "calculate_area", std::bind(&AreaService::compute_area, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service Server Ready: calculate_area");
    }

private:
    void compute_area(const std::shared_ptr<area_interfaces::srv::CalculateArea::Request> request,
                      std::shared_ptr<area_interfaces::srv::CalculateArea::Response> response) {
        response->area = request->length * request->width;
        RCLCPP_INFO(this->get_logger(), "Received request - Length: %.2f, Width: %.2f. Computed Area: %.2f",
                    request->length, request->width, response->area);
    }

    rclcpp::Service<area_interfaces::srv::CalculateArea>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AreaService>());
    rclcpp::shutdown();
    return 0;
}
```

---

### **8️⃣ Implement the Service Client**
Create `src/area_client.cpp`:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "area_interfaces/srv/calculate_area.hpp"

class AreaClient : public rclcpp::Node {
public:
    AreaClient() : Node("area_service_client") {
        client_ = this->create_client<area_interfaces::srv::CalculateArea>("calculate_area");

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service...");
        }

        auto request = std::make_shared<area_interfaces::srv::CalculateArea::Request>();
        request->length = 5.0;
        request->width = 3.0;

        auto future = client_->async_send_request(request, 
            std::bind(&AreaClient::response_callback, this, std::placeholders::_1));
    }

private:
    void response_callback(rclcpp::Client<area_interfaces::srv::CalculateArea>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "Area received: %.2f", future.get()->area);
    }

    rclcpp::Client<area_interfaces::srv::CalculateArea>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AreaClient>());
    rclcpp::shutdown();
    return 0;
}
```

---

### **9️⃣ Build the Service Package**
Now, build everything:
```bash
colcon build --packages-select area_service
source install/setup.bash
```

---

### **🔟 Running the Service**
#### **Start the Service Server**
```bash
ros2 run area_service area_server
```

#### **Call the Service from Client**
```bash
ros2 run area_service area_client
```

### **✅ Expected Output**
```
[INFO] [area_service_server]: Service Server Ready: calculate_area
[INFO] [area_service_server]: Received request - Length: 5.00, Width: 3.00. Computed Area: 15.00
[INFO] [area_service_client]: Area received: 15.00
```

---
