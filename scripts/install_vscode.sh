#!/bin/bash
################################################
# Function : Install VSCode 
# Desc     : Script to install Visual Studio Code on Ubuntu 
# Website  : https://www.ncnynl.com/archives/202212/5813.html                              
# Platform : Ubuntu                                
# Version  : 1.1                               
# Date     : 2025-01-26                            
# Author   : Khaled                             
# Contact  : khaledgabr77@gmail.com                              
################################################

# Set localization for text output
export TEXTDOMAINDIR=/usr/share/locale
export TEXTDOMAIN=commands        
echo "$(gettext "Install VSCode")"

echo "Starting the installation of Visual Studio Code..."

# Step 1: Ensure required tools are installed
echo "Installing required tools (wget, gpg)..."
sudo apt-get install wget gpg -y
if [ $? -ne 0 ]; then
    echo "Failed to install wget and gpg. Please check your internet connection and try again."
    exit 1
fi

# Step 2: Add Microsoft's GPG key
echo "Adding Microsoft's GPG key..."
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
if [ $? -ne 0 ]; then
    echo "Failed to download Microsoft's GPG key. Please check your internet connection."
    exit 1
fi

# Step 3: Install the GPG key
echo "Installing the GPG key..."
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
if [ $? -ne 0 ]; then
    echo "Failed to install the GPG key. Please check your permissions and try again."
    rm -f packages.microsoft.gpg
    exit 1
fi

# Remove the temporary GPG key file
rm -f packages.microsoft.gpg

# Step 4: Add VSCode repository to sources.list.d
echo "Adding VSCode repository..."
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
if [ $? -ne 0 ]; then
    echo "Failed to add the VSCode repository. Please check your permissions."
    exit 1
fi

# Step 5: Ensure HTTPS transport is installed
echo "Installing apt-transport-https..."
sudo apt install apt-transport-https -y
if [ $? -ne 0 ]; then
    echo "Failed to install apt-transport-https. Please check your internet connection."
    exit 1
fi

# Step 6: Update package lists
echo "Updating package lists..."
sudo apt update
if [ $? -ne 0 ]; then
    echo "Failed to update package lists. Please check your repository settings."
    exit 1
fi

# Step 7: Install VSCode
echo "Installing Visual Studio Code..."
sudo apt install code -y
if [ $? -ne 0 ]; then
    echo "Failed to install Visual Studio Code. Please check your system and try again."
    exit 1
fi

# Success message
echo "Congratulations! Visual Studio Code has been successfully installed."
