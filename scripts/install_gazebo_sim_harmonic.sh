#!/bin/bash
################################################################
# Function : Install Gazebo Sim Harmonic (AMD version)        #
# Desc     : Script to install Gazebo Sim Harmonic on AMD     #
# Platform : All Linux-Based Platforms                        #
# Version  : 1.0                                             #
# Date     : 2025-02-03                                      #
# Author   : Khaled                                          #
# Contact  : khaledgabr77@gmail.com                          #
################################################################

export TEXTDOMAINDIR=/usr/share/locale
export TEXTDOMAIN=commands

echo "$(gettext "Install Gazebo Sim Harmonic (AMD version)")"

echo "Starting the installation of Gazebo Sim Harmonic..."

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Install prerequisites
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add Gazebo Sim Harmonic repository
echo "Adding Gazebo Sim Harmonic repository..."
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://packages.osrfoundation.org/gazebo.key | sudo tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \nhttps://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Sim Harmonic
echo "Installing Gazebo Sim Harmonic..."
sudo apt update
sudo apt install -y gazebo-harmonic

# Install additional Gazebo tools
echo "Installing additional Gazebo tools..."
sudo apt install -y \
    gazebo-harmonic-plugin-base \
    gazebo-harmonic-sim \
    gazebo-harmonic-sensors \
    gazebo-harmonic-physics \
    gazebo-harmonic-gui

# Install ROS 2 Gazebo Sim bridge (optional for ROS 2 integration)
echo "Installing ROS 2 Gazebo Sim bridge..."
sudo apt install -y ros-jazzy-gz-ros2-control ros-jazzy-gz-sim ros-jazzy-gz-msgs

# Add Gazebo environment variables to bashrc
echo "Adding Gazebo Sim Harmonic to bashrc..."
if ! grep -Fq "/usr/share/gazebo-harmonic/setup.sh" ~/.bashrc
then
    echo "source /usr/share/gazebo-harmonic/setup.sh" >> ~/.bashrc
    echo "Gazebo Sim Harmonic successfully installed and added to ~/.bashrc."
else
    echo "Gazebo Sim Harmonic has already been initialized in ~/.bashrc. Skipping this step."
fi

# Confirm installation
echo "Gazebo Sim Harmonic installed successfully!"
