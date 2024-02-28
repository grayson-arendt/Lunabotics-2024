#!/bin/bash

ros_packages=(
    "ros-humble-joint-state-publisher"
    "ros-humble-xacro"
    "ros-humble-navigation2"
    "ros-humble-nav2-common"
)

# Install ROS 2 packages
apt install -y "${ros_packages[@]}"