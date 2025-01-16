# YahboomcarR2 Ackermann Steering Vehicle Simulation

## Overview
This project provides a simulation of the YahboomcarR2, an Ackermann steering vehicle, in a ROS 2 environment with Gazebo Sim **Ionic**/**Harmonic**. It supports ROS 2 **Humble** and **Jazzy** distributions and is designed for ease of use and customization for robotics research and development.

---

## Requirements
To use this simulation, ensure you have the following installed:

- **Ubuntu 22.04 or 24.04**
- **ROS 2 Jazzy or Humble**
- **Gazebo Sim Ionic or Harmonic**

---

## Installation

Follow these steps to set up the YahboomcarR2 simulation:

### 1. Clone the Repository
Clone the repository from GitHub:
```bash
https://github.com/khaledgabr77/yahboomcarR2
```

### 2. Build the Project
Navigate to the workspace and build the project:
```bash
colcon build && source install/setup.bash
```

### 3. Set Environment Variables
Set the necessary environment variables to locate the simulation resources:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/your/path/src
```
Replace `/your/path/src` with the absolute path to the `src` directory of your workspace.

---

## Launching the Simulation
Launch the YahboomcarR2 robot in Gazebo Sim with the following command:
```bash
ros2 launch yahboomcar_description robot_sim.launch.py
```

This will initialize the simulation environment and load the YahboomcarR2 model.
### Move the Robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
---

## Additional Information
- The simulation supports both ROS 2 Jazzy and Humble distributions.
- Gazebo Sim Ionic or Harmonic is required for the visualization and control of the YahboomcarR2 robot.
- Ensure all dependencies are properly installed before running the simulation.

For further details or contributions, visit the [GitHub repository](https://github.com/khaledgabr77/yahboomcarR2).

---

## License
This project is licensed under the [MIT License](LICENSE).

---

## Contact
For questions or suggestions, feel free to contact the repository maintainer through GitHub Issues or via email at khaledgabr77@gmail.com.

