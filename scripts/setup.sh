#!/bin/bash
################################################
# Function : Check GitHub Site                #
# Desc     : Checks and clones the repository if needed. If the workspace is corrupted, it resets it.            #
# Platform : Ubuntu                                #
# Version  : 1.1                               #
# Date     : 2025-02-03                           #
# Author   : Khaled H Gabr                            #
# Contact  : khaledgabr77@gmail.com                             #
# URL      : https://github.com/khaledgabr77/jordan_workshop
# Usage    : ./check_github.sh
# Notes    : If your workspace is corrupted or not working, this script resets it by deleting the old directory and cloning a fresh copy.                                                                 #
################################################

WORKSPACE=~/jordan_workshop
REPO_URL="https://github.com/khaledgabr77/jordan_workshop"

echo "Checking GitHub repository..."

cd ~ || { echo "Failed to change to home directory. Exiting..."; exit 1; }

if [ -d "$WORKSPACE" ]; then
    echo "The directory '$WORKSPACE' already exists. Deleting it to reset your workspace..."
    rm -rf "$WORKSPACE"
    if [ $? -ne 0 ]; then
        echo "Failed to delete the directory '$WORKSPACE'. Please check permissions or disk space."
        exit 1
    fi
else
    echo "The directory '$WORKSPACE' does not exist. Preparing to clone the repository..."
fi

echo "Cloning the repository from '$REPO_URL'..."
git clone "$REPO_URL" "$WORKSPACE"

if [ -d "$WORKSPACE" ]; then
    echo "Repository successfully cloned into '$WORKSPACE'. Your workspace is now reset."
    cd "$WORKSPACE"
    
    if [ -d src ]; then
        echo "Building existing workspace..."
    else
        echo "Initializing workspace..."
        mkdir src
        mv * src 2>/dev/null
    fi
    
    colcon build --symlink-install
    source install/setup.bash
    echo "Workspace built and sourced successfully."
else
    echo "Failed to clone the repository. Please check the repository URL and your internet connection."
    exit 1
fi
