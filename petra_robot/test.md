```bash

khaled@khaled:~/jordan_ws/src/petra_robot/urdf$ gz sdf -p petra_robot.urdf > petra_robot.sdf

khaled@khaled:~/jordan_ws/src/petra_robot$ mkdir models

khaled@khaled:~/jordan_ws/src/petra_robot$ mkdir worlds

khaled@khaled:~/jordan_ws/src/petra_robot/models$ mkdir petra_robot

khaled@khaled:~/jordan_ws/src/petra_robot/models$ cd petra_robot/

khaled@khaled:~/jordan_ws/src/petra_robot/models/petra_robot$ mkdir meshes

khaled@khaled:~/jordan_ws/src/petra_robot/models/petra_robot$ touch model.sdf model.config



khaled@khaled:~/jordan_ws/src$ ros2 pkg create qos_test --build-type ament_python --dependencies rclpy std_msgs

khaled@khaled:~/jordan_ws/src$ cd qos_test
models/
└── petra_robot/
    ├── meshes/
    ├── model.sdf
    └── model.config

<?xml version='1.0' encoding='utf-8'?>
<model>
  <name>yahboomcar</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>khaled gabr</name>
    <email>khaledgabr77@gmail.com</email>
  </author>
  <description>A model of Ackermann R2 car.</description>
</model>

khaled@khaled:~/jordan_ws/src/petra_robot$ mkdir config
khaled@khaled:~/jordan_ws/src/petra_robot/config$ touch petra_robot.yaml 

khaled@khaled:~/jordan_ws$ ros2 launch petra_robot petra_sim.launch.py 

khaled@khaled:~/jordan_ws$ ros2 run qos_test qos_reliability_publisher

khaled@khaled:~/jordan_ws$ ros2 run qos_test qos_reliability_subscriber

khaled@khaled:~/jordan_ws$ ros2 run qos_test qos_reliability_publisher -reliability best_effort

khaled@khaled:~/jordan_ws$ ros2 run qos_test qos_reliability_subscriber
[INFO] [1736852338.243167538] [qos_reliability_subscriber]: Using Reliability Policy: 1
[WARN] [1736852338.794529408] [qos_reliability_subscriber]: New publisher discovered on topic 'qos_reliability_topic', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY



khaled@khaled:~/jordan_ws$ colcon build
khaled@khaled:~/jordan_ws$ source install/setup.bash
khaled@khaled:~/jordan_ws$ ros2 run qos_test qos_publisher
khaled@khaled:~/jordan_ws$ ros2 run qos_test qos_subscriber

dev_ws/                # Name your workspace directory as you like, e.g. "dev_ws"
  ├── src/             # Source directory for all your packages
  ├── install/         # (Automatically created by the build tool) Install artifacts here
  ├── build/           # (Automatically created by the build tool) Build artifacts here
  └── log/             # (Automatically created by the build tool) Log files here

khaled@khaled:~$ mkdir -p ~/dev_ws/src
khaled@khaled:~$ cd ~/dev_ws/src

khaled@khaled:~/jordan_ws/src$ ros2 pkg create my_cpp_pkg --build-type ament_cmake



```css

my_cpp_pkg/
├── CMakeLists.txt
├── package.xml
└── src/
    └── ...



cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


khaled@khaled:~/jordan_ws/src$ ros2 pkg create my_cpp_pkg --build-type ament_cmake
going to create a new package
package name: my_cpp_pkg
destination directory: /home/khaled/jordan_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['khaled <khaledgabr77@gmail.com>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
creating folder ./my_cpp_pkg
creating ./my_cpp_pkg/package.xml
creating source and include folder
creating folder ./my_cpp_pkg/src
creating folder ./my_cpp_pkg/include/my_cpp_pkg
creating ./my_cpp_pkg/CMakeLists.txt

[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identifiers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0


```


```yaml
# Gazebo topic published by the simulator core
- ros_topic_name: "clock"                      # ROS 2 topic for simulation time
  gz_topic_name: "clock"                       # Gazebo topic for simulation time
  ros_type_name: "rosgraph_msgs/msg/Clock"     # ROS 2 message type for clock data
  gz_type_name: "gz.msgs.Clock"                # Gazebo message type for clock data
  direction: GZ_TO_ROS                         # Data flows from Gazebo to ROS 2
  
# Gazebo topic published by DiffDrive plugin
- ros_topic_name: "/odom"                      # ROS 2 topic for odometry data
  gz_topic_name: "/odom"                       # Gazebo topic for odometry data
  ros_type_name: "nav_msgs/msg/Odometry"       # ROS 2 message type for odometry
  gz_type_name: "gz.msgs.Odometry"             # Gazebo message type for odometry
  direction: GZ_TO_ROS                         # Data flows from Gazebo to ROS 2

# Gazebo topic published by DiffDrive plugin
- ros_topic_name: "/tf"                        # ROS 2 topic for transforms
  gz_topic_name: "/tf"                         # Gazebo topic for transforms
  ros_type_name: "tf2_msgs/msg/TFMessage"      # ROS 2 message type for TF data
  gz_type_name: "gz.msgs.Pose_V"               # Gazebo message type for pose data
  direction: GZ_TO_ROS                         # Data flows from Gazebo to ROS 2

# Gazebo topic subscribed to by DiffDrive plugin
- ros_topic_name: "cmd_vel"                    # ROS 2 topic for velocity commands
  gz_topic_name: "cmd_vel"                     # Gazebo topic for velocity commands
  ros_type_name: "geometry_msgs/msg/Twist"     # ROS 2 message type for velocity
  gz_type_name: "gz.msgs.Twist"                # Gazebo message type for velocity
  direction: ROS_TO_GZ                         # Data flows from ROS 2 to Gazebo

- ros_topic_name: "/scan"                     # ROS 2 topic for LIDAR data
  gz_topic_name: "/scan"                      # Corresponding Gazebo topic for LIDAR data
  ros_type_name: "sensor_msgs/msg/LaserScan"  # ROS 2 message type for LIDAR data
  gz_type_name: "gz.msgs.LaserScan"           # Gazebo message type for LIDAR data
  direction: GZ_TO_ROS                        # Data flows from Gazebo to ROS 2



cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


```

```cpp

pkg_project_description = get_package_share_directory('petra_robot')
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


sdf_file = os.path.join(pkg_project_description, 'models', 'petra_robot', 'model.sdf')
with open(sdf_file, 'r') as infp:
    robot_desc = infp.read()


from setuptools import setup

package_name = 'qos_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Example package to demonstrate QoS History type in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qos_publisher = qos_test.qos_publisher:main',
            'qos_subscriber = qos_test.qos_subscriber:main',
        ],
    },
)




import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        qos_profile = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )
        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Message {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = QoSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy
import time


class QoSSubscriber(Node):
    def __init__(self, history_policy):
        super().__init__('qos_subscriber')
        qos_profile = QoSProfile(
            depth=5,  # Keep only the last 5 messages in KEEP_LAST
            history=history_policy
        )
        self.subscription = self.create_subscription(
            String,
            'qos_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info(f'Using History Policy: {history_policy}')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        # Simulate processing delay
        time.sleep(2)


def main():
    rclpy.init()

    # Toggle between KEEP_LAST and KEEP_ALL to observe behavior
    use_keep_last = True
    history_policy = HistoryPolicy.KEEP_LAST if use_keep_last else HistoryPolicy.KEEP_ALL

    node = QoSSubscriber(history_policy)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': PathJoinSubstitution([
        pkg_project_description,
        'worlds',
        'default.sdf'
    ])}.items(),
)


robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[
        {'use_sim_time': True},
        {'robot_description': robot_desc},
    ]
)



joint_state_publisher_node = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[{'use_sim_time': True}]
)


bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{
        'config_file': os.path.join(pkg_project_description, 'config', 'petra_robot.yaml'),
        'qos_overrides./tf_static.publisher.durability': 'transient_local',
    }],
    output='screen'
)


spawn_robot_node = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'petra_robot',
        '-topic', 'robot_description',
    ],
    output='screen'
)


rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    output='screen',
    name='sim_rviz2',
    arguments=['-d' + os.path.join(pkg_project_description, 'rviz', 'rahal.rviz')]
)


return LaunchDescription([
    gz_sim,                           # Start Gazebo simulation
    bridge,                           # Establish Gazebo-ROS bridge
    spawn_robot_node,                 # Spawn the robot in Gazebo
    robot_state_publisher,            # Publish robot states
    joint_state_publisher_node,       # Publish joint states
    rviz_node,                        # Visualize in RViz2
])




khaled@khaled:~/jordan_ws$ ros2 launch petra_robot petra_sim.launch.py 
```


```xml
    <link name="right_wheel">
        <pose relative_to="base_link">0.1 -0.175 0 0 1.5708 1.5708</pose>   
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="right_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient>
          <diffuse>1.0 0.0 0.0 1.0</diffuse>
        </material>
      </visual>
      <collision name="right_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
            <limit>
                <lower>-1.79769e+308</lower>    
                <upper>1.79769e+308</upper>
            </limit>
      </axis>
    </joint>

    <link name="left_wheel">
      <pose relative_to="base_link">0.1 0.175 0 0 1.5708 1.5708</pose>   
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="left_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient>
          <diffuse>1.0 0.0 0.0 1.0</diffuse>
        </material>
      </visual>
      <collision name="left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.79769e+308</lower>    
            <upper>1.79769e+308</upper>
          </limit>
      </axis>
    </joint>

    <link name="caster_wheel_link">
        <pose relative_to="base_link">-0.1 0 -0.06 0 0 0</pose>    

      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="caster_wheel_visual">
        <geometry>
          <sphere>
            <radius>0.02</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient>
          <diffuse>1.0 0.0 0.0 1.0</diffuse>
        </material>
      </visual>
      <collision name="caster_wheel_collision">
        <geometry>
          <sphere>
            <radius>0.02</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <joint name="caster_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>caster_wheel_link</child>
    </joint>


    <link name="lidar_link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="lidar_visual">
        <geometry>
          <mesh>
            <uri>meshes/2d_lidar.STL</uri> 
          </mesh>
        </geometry>
        <material>
          <ambient>0.0 0.0 0.0 1.0</ambient>
          <diffuse>0.0 0.0 0.0 1.0</diffuse>
        </material>
      </visual>
      <collision name="lidar_collision">
        <geometry>
          <mesh>
            <uri>meshes/2d_lidar.STL</uri> 
          </mesh>
        </geometry>
      </collision>
    </link> 
    <joint name="lidar_joint" type="fixed">
      <parent>lidar_base_link</parent>
      <child>lidar_link</child>
      <pose>0 0 0.065 0 0 0</pose>
    </joint>


    <link name="camera_link">
    <pose relative_to="base_link">-0.2 0 0.03 0 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="camera_visual">
        <geometry>
            <box>
                <size>0.05 0.05 0.05 </size>
            </box>
        </geometry>
        <material>
          <ambient>0.0 0.0 0.0 1.0</ambient>
          <diffuse>0.0 0.0 0.0 1.0</diffuse>
        </material>
      </visual>
      <collision name="camera_collision">
        <geometry>
            <box>
                <size>0.1 0.1 0.1</size>
            </box>
        </geometry>
      </collision>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
  </model>


  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <topic>cmd_vel</topic>    
    <left_joint>left_wheel_joint</left_joint>    
    <right_joint>right_wheel_joint</right_joint>    
    <wheel_separation>0.4</wheel_separation>    
    <wheel_radius>0.08</wheel_radius>    
    <odom_publish_frequency>50</odom_publish_frequency>   
    <odom_topic>/odom</odom_topic> 
</plugin>

<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <topic>cmd_vel</topic> <!-- Topic for velocity commands to control the robot -->
    <left_joint>left_wheel_joint</left_joint> <!-- Name of the left wheel joint -->
    <right_joint>right_wheel_joint</right_joint> <!-- Name of the right wheel joint -->
    <wheel_separation>0.4</wheel_separation> <!-- Distance between the left and right wheels -->
    <wheel_radius>0.08</wheel_radius> <!-- Radius of each wheel -->
    <odom_publish_frequency>50</odom_publish_frequency> <!-- Frequency (Hz) to publish odometry data -->
    <odom_topic>/odom</odom_topic> <!-- Topic name for odometry output -->
</plugin>





<sensor name="lidar" type="gpu_lidar">
    <!-- <pose>0.00174 0.00064 0.0959 0 0 0</pose> -->
    <lidar>
        <scan>
            <horizontal>
                <samples>400</samples>               <!-- Number of data points in one full scan -->
                <resolution>1</resolution>           <!-- Angular resolution of the scan -->
                <min_angle>-1.5707</min_angle>       <!-- Minimum scanning angle (radians) -->
                <max_angle>1.5707</max_angle>        <!-- Maximum scanning angle (radians) -->
            </horizontal>
        </scan>
        <range>
            <min>0.15</min>                         <!-- Minimum detectable range (meters) -->
            <max>25</max>                           <!-- Maximum detectable range (meters) -->
            <resolution>0.01</resolution>           <!-- Resolution of range measurements (meters) -->
        </range>
    </lidar>
    <gz_frame_id>lidar_link</gz_frame_id>          <!-- Reference frame for the LIDAR -->
    <topic>scan</topic>                            <!-- Topic name for LIDAR data -->
    <alwaysOn>1</alwaysOn>                         <!-- Keep the sensor always active -->
    <visualize>true</visualize>                    <!-- Enable visualization of the LIDAR rays -->
    <update_rate>10</update_rate>                  <!-- Frequency of sensor updates (Hz) -->
</sensor>

<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_cpp_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="khaledgabr77@gmail.com">khaled</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>




```

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

```python









import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy
import time


class QoSSubscriber(Node):
    def __init__(self, history_policy):
        super().__init__('qos_subscriber')
        qos_profile = QoSProfile(
            depth=5,  # Keep only the last 5 messages in KEEP_LAST
            history=history_policy
        )
        self.subscription = self.create_subscription(
            String,
            'qos_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info(f'Using History Policy: {history_policy}')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        # Simulate processing delay
        time.sleep(2)


def main():
    rclpy.init()

    # Toggle between KEEP_LAST and KEEP_ALL to observe behavior
    use_keep_last = True
    history_policy = HistoryPolicy.KEEP_LAST if use_keep_last else HistoryPolicy.KEEP_ALL

    node = QoSSubscriber(history_policy)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        qos_profile = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )
        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Message {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = QoSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class QoSReliabilityPublisher(Node):
    def __init__(self):
        super().__init__('qos_reliability_publisher')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # Required setting
            depth=10,  # Required for KEEP_LAST
            reliability=ReliabilityPolicy.RELIABLE,  # RELIABLE or BEST_EFFORT
        )
        self.publisher_ = self.create_publisher(String, 'qos_reliability_topic', qos_profile)
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5 seconds
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Message {self.count}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.count += 1


def main():
    rclpy.init()
    node = QoSReliabilityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Publisher stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class QoSReliabilitySubscriber(Node):
    def __init__(self, reliability_policy):
        super().__init__('qos_reliability_subscriber')
        qos_profile = QoSProfile(
            reliability=reliability_policy,
        )
        self.subscription = self.create_subscription(
            String,
            'qos_reliability_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info(f"Using Reliability Policy: {reliability_policy}")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")


def main():
    rclpy.init()

    # Toggle between RELIABLE and BEST_EFFORT
    use_reliable = True
    reliability_policy = ReliabilityPolicy.RELIABLE if use_reliable else ReliabilityPolicy.BEST_EFFORT

    node = QoSReliabilitySubscriber(reliability_policy)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Subscriber stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

```cpp

cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

