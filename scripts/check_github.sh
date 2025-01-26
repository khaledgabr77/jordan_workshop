#!/bin/bash
################################################
# Function : Check GitHub Site                
# Desc     : Checks and clones the repository if needed. If the workspace is corrupted, it resets it.            
# Platform : Ubuntu                                
# Version  : 1.0                               
# Date     : 2025-01-26 10:27:32                            
# Author   : Khaled H Gabr                            
# Contact  : khaledgabr77@gmail.com                             
# URL      : https://github.com/khaledgabr77/jordan_workshop
# Usage    : ./check_github.sh
# Notes    : If your workspace is corrupted or not working, this script resets it by deleting the old directory and cloning a fresh copy.                                                                 
################################################

echo "Checking GitHub repository..."

cd ~ || { echo "Failed to change to home directory. Exiting..."; exit 1; }

if [ -d ~/jordan_workshop ]; then
    echo "The directory '~/jordan_workshop' already exists. Deleting it to reset your workspace..."
    rm -rf ~/jordan_workshop
    if [ $? -ne 0 ]; then
        echo "Failed to delete the directory '~/jordan_workshop'. Please check permissions or disk space."
        exit 1
    fi
else
    echo "The directory '~/jordan_workshop' does not exist. Preparing to clone the repository..."
fi

echo "Cloning the repository from 'https://github.com/khaledgabr77/jordan_workshop'..."
git clone https://github.com/khaledgabr77/jordan_workshop ~/jordan_workshop

if [ -d ~/jordan_workshop ]; then
    echo "Repository successfully cloned into '~/jordan_workshop'. Your workspace is now reset."
    exit 0
else
    echo "Failed to clone the repository. Please check the repository URL and your internet connection."
    exit 1
fi
