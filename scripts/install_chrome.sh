#!/bin/bash
################################################
# Function : Install Google Chrome (Linux version)
# Desc     : Script to install Google Chrome using the .deb package
# Website  : https://itsfoss.com/install-chrome-ubuntu/                       
# Platform : Ubuntu                                
# Version  : 1.1                               
# Date     : 2025-01-26                         
# Author   : Khaled                             
# Contact  : khaledgabr77@gmail.com                              
################################################

# Set localization for text output
export TEXTDOMAINDIR=/usr/share/locale
export TEXTDOMAIN=commands        
echo "$(gettext "Install Google Chrome (Linux version)")"

echo "Starting installation of Google Chrome from .deb package..."

# Change to the user's home directory
cd ~ || { echo "Failed to change to the home directory. Exiting..."; exit 1; }

# Download the Google Chrome .deb package
DEB_PACKAGE="google-chrome-stable_current_amd64.deb"
DOWNLOAD_URL="https://dl.google.com/linux/direct/$DEB_PACKAGE"

echo "Downloading Google Chrome package from $DOWNLOAD_URL..."
if ! wget "$DOWNLOAD_URL"; then
    echo "Failed to download the Google Chrome package. Please check your internet connection."
    exit 1
fi

# Install the downloaded .deb package
echo "Installing the Google Chrome package..."
if ! sudo dpkg -i "$DEB_PACKAGE"; then
    echo "Failed to install the package. Attempting to fix missing dependencies..."
    sudo apt-get -f install -y
    if [ $? -ne 0 ]; then
        echo "Failed to resolve dependencies. Exiting..."
        exit 1
    fi
fi

# Create a symbolic link for easier access
echo "Creating a symbolic link for Google Chrome..."
if ! sudo ln -sf /usr/bin/google-chrome-stable /usr/bin/google-chrome; then
    echo "Failed to create symbolic link. Please check your permissions."
    exit 1
fi

# Clean up the downloaded package
echo "Cleaning up the downloaded package..."
rm -f "$DEB_PACKAGE"

# Success message
echo "Congratulations, Google Chrome has been successfully installed!"
