# Jordan Workshop

This guide walks you through setting up, installing, and running the packages in this workspace.

## Prerequisites

- **Ubuntu 22.04 or 24.04**
- **ROS 2 Jazzy or Humble**
- **Gazebo Sim Ionic or Harmonic**

### Install ROS 2 Jazzy

```bash
cd jordan_workshop/scripts
./install_ros2_jazzy.sh
```

### Install Gazebo Sim Harmonic

```bash
cd jordan_workshop/scripts
./install_gazebo_sim_harmonic.sh
```

## Automated Workspace Setup

To automatically set up the workspace and install all required dependencies, run:

```bash
cd jordan_workshop/scripts
./setup.sh
```

## Manual Setup Instructions

### 1. Clone the Repository

Clone the repository from GitHub:

```bash
git clone https://github.com/khaledgabr77/jordan_workshop
```

### 2. Build the Workspace

Navigate to the workspace and build the project:

```bash
colcon build
source install/setup.bash
```

### 3. Set Environment Variables

Set the necessary environment variable so that Gazebo (Ignition/GZ) can locate the simulation resources:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/jordan_workshop/src
```

> **Note**: Replace `~/jordan_workshop/src` with the absolute path to the `src` directory of your workspace if it's located elsewhere.

---

## Package Usage

Below are instructions for launching simulations and running various ROS 2 nodes in this workspace.

### 1. Launching the Petra Robot Simulation

There are multiple launch files available, each providing different configurations for the Petra Robot:

1. **SDF Simulation**:
    ```bash
    ros2 launch petra_robot petra_sim.launch.py
    ```

2. **URDF Simulation**:
    ```bash
    ros2 launch petra_robot petra_urdf.launch.py
    ```
    > *Note:* There are several URDF launch files available in the `petra_robot` package, such as `petra_urdf_v1.launch.py`, `petra_urdf_v3.launch.py`, and so on. Each file provides a different robot configuration.

3. **SLAM Simulation**:
    ```bash
    ros2 launch petra_robot mapping.launch.py
    ```

4. **Localization Simulation**:
    ```bash
    ros2 launch petra_robot localization.launch.py
    ```

5. **Navigation Simulation**:
    ```bash
    ros2 launch petra_robot navigation.launch.py
    ```

---

### 2. Running the Publisher and Subscriber Nodes

Use the following commands to run publisher and subscriber nodes from the **C++** and **Python** pub/sub packages:

- **C++** (`cpp_pub_sub_pkg`):
    ```bash
    ros2 run cpp_pub_sub_pkg publisher_hello_ros
    ros2 run cpp_pub_sub_pkg subscriber_hello_ros
    ```

- **Python** (`py_pub_sub_pkg`):
    ```bash
    ros2 run py_pub_sub_pkg publisher_hello_ros
    ros2 run py_pub_sub_pkg subscriber_hello_ros
    ```

---

### 3. Running the Client and Server Services

The `py_service_pkg` package includes a client and server:

```bash
ros2 run py_service_pkg py_service_server
ros2 run py_service_pkg py_service_client
```

> **Note:** The `cpp_interface_pkg` contains the custom service definition (`srv` message) used by the service nodes.

---

### 4. Running Launch Files

Several launch files are provided to simplify running multiple nodes simultaneously or to configure simulations.

1. **C++ Publisher/Subscriber**:
   ```bash
   ros2 launch cpp_pub_sub_pkg cpp_pub_sub.launch.py
   ```

2. **Turtlesim Launch (C++)**:
   ```bash
   ros2 launch cpp_launch_pkg cpp_launch_file.launch.py
   ```

3. **Turtlesim Launch (Python)**:
   ```bash
   ros2 launch py_launch_pkg my_first_launch_file.launch.py
   ```

4. **Turtlesim Remapping**:
   ```bash
   ros2 launch py_launch_pkg remapping_turtle.launch.py
   ```

---

### 5. Running Parameter Nodes

To run Python nodes that use parameters (in the `py_parameters_pkg`):

1. **Parameter Nodes**:
   ```bash
   ros2 run py_parameters_pkg my_parameter
   ros2 run py_parameters_pkg py_param
   ```

2. **Launch File with Parameters**:
   ```bash
   ros2 launch py_parameters_pkg publisher_param.launch.py
   ```

---

### 6. Running TF2 Nodes

To work with TF2 (transforms) using `py_tf2_pkg`:

1. **Static Broadcaster**:
   ```bash
   ros2 run py_tf2_pkg static_broadcaster.py
   ```

2. **Static Listener**:
   ```bash
   ros2 run py_tf2_pkg tf_listener.py
   ```

3. **Dynamic Broadcaster**:
   ```bash
   ros2 run py_tf2_pkg dynamic_transform_broadcaster.py
   ```

4. **Pose Estimation** (turtle2 w.r.t. turtle1):
   ```bash
   ros2 run py_tf2_pkg turtle_tf2_pose.py
   ```

5. **Turtle Follower**:
   ```bash
   ros2 run py_tf2_pkg turtle_tf2_follower.py
   ```

---

### 7. Running the Mogi Trajectory Server

To start the Mogi Trajectory Server:

```bash
ros2 run mogi_trajectory_server mogi_trajectory_server
```

---

### 8. Running the Yahboomcar Simulation

To launch the Yahboomcar simulation in `yahboomcar_description`:

```bash
ros2 launch yahboomcar_description robot_sim.launch.py
```

---

## Additional Information

- Ensure all dependencies are installed correctly before running any simulations.
- For questions, contributions, or further details, visit the [GitHub repository](https://github.com/khaledgabr77/jordan_workshop).

---

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or suggestions, please open a GitHub Issue or reach out via email at **khaledgabr77@gmail.com**.

---

*Happy Coding and Simulating!*