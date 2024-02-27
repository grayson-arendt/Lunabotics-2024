#!/bin/bash

ros_packages=(
    "ros-humble-rtabmap"
    "ros-humble-rtabmap-ros"
    "ros-humble-rplidar-ros"
    "ros-humble-navigation2"
)

# Install ROS 2 packages
apt install -y "${ros_packages[@]}"