#!/bin/bash

# Array of ROS 2 package names to install
ros_packages=(
    "ros-humble-rtabmap"
    "ros-humble-rtabmap-ros"
    "ros-humble-rplidar-ros"
    "ros-humble-navigation2"
    "ros-humble-robot-localization"
    "ros-humble-robot-state-publisher"
    "ros-humble-joint-state-publisher"
)

# Install ROS 2 packages
apt install -y "${ros_packages[@]}"

# Array of ROS packages to remove
ros_remove_packages=(
    "ros-humble-librealsense2"
    "ros-humble-realsense2-camera"
    "ros-humble-realsense2-camera-msgs"
    "ros-humble-realsense2-description"
)

# Remove conflicting ROS packages
for pkg in "${ros_remove_packages[@]}"; do
    dpkg -r --force-depends "$pkg"
done
