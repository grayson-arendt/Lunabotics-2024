#!/bin/bash

ros_packages=(
    "ros-humble-robot-state-publisher"
    "ros-humble-joint-state-publisher"
)

# Install ROS 2 packages
apt install -y "${ros_packages[@]}"