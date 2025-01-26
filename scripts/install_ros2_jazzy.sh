#!/bin/bash
################################################################
# Function : Install ROS2 Jazzy AMD version                    
# Desc     : Script to install ROS2 Jazzy version on AMD architecture
# Platform : All Linux-Based Platforms                           
# Version  : 1.2                                                
# Date     : 2025-01-26                                         
# Author   : Khaled                                             
# Contact  : khaledgabr77@gmail.com                             
################################################################

export TEXTDOMAINDIR=/usr/share/locale
export TEXTDOMAIN=commands        
echo "$(gettext "Install ROS2 Jazzy AMD version")"

ros2_distro=jazzy

echo "Starting the installation of ROS2 $ros2_distro..."

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Install prerequisites
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
echo "Adding ROS2 $ros2_distro repository..."
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.aliyun.com/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Jazzy desktop-full
echo "Installing ROS2 $ros2_distro desktop-full version..."
sudo apt update
sudo apt install -y ros-$ros2_distro-desktop-full
sudo apt install -y python3-argcomplete python3-pip 

# Install ROS2 tools
echo "Installing additional ROS2 tools..."
sudo apt install -y ros-$ros2_distro-rqt
sudo apt install -y ros-$ros2_distro-rqt-tf-tree
sudo apt install -y ros-$ros2_distro-turtlesim

# Install ROS2 build tools
sudo apt install -y \
python3-colcon-common-extensions \
python3-vcstool \
python3-rosdep

# Install RMW implementations
sudo apt install -y ros-$ros2_distro-rmw-cyclonedds-cpp
sudo apt install -y ros-$ros2_distro-rmw-fastrtps-cpp

# Install TF2 dependencies
sudo apt install -y \
ros-$ros2_distro-turtle-tf2-py \
ros-$ros2_distro-tf2-tools \
ros-$ros2_distro-tf-transformations

# Install ROS2 bag dependencies
sudo apt install -y \
ros-$ros2_distro-nmea-msgs \
ros-$ros2_distro-rosbag2 \
ros-$ros2_distro-rosbag2-storage \
ros-$ros2_distro-rosbag2-storage-default-plugins \
ros-$ros2_distro-ros2bag \
ros-$ros2_distro-rosbag2-transport 

# Add ROS2 Jazzy to bashrc
echo "Adding ROS2 $ros2_distro to bashrc..."
if ! grep -Fq "/opt/ros/$ros2_distro/setup.bash" ~/.bashrc
then
    echo "source /opt/ros/$ros2_distro/setup.bash" >> ~/.bashrc
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    echo "$ros2_distro workspace has been successfully installed! Added to ~/.bashrc."
else
    echo "ROS2 $ros2_distro has already been initialized in ~/.bashrc. Skipping this step."
fi

if ! grep -Fq "RMW_IMPLEMENTATION" ~/.bashrc
then
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
fi

echo "ROS2 $ros2_distro installed successfully!"
