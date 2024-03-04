#!/bin/bash

ros_packages=(
    "ros-humble-joint-state-publisher"
    "ros-humble-xacro"
    "ros-humble-navigation2"
    "ros-humble-nav2-common"
    "ros-humble-rmw-cyclonedds-cpp"
)

# Install ROS 2 packages
apt install -y "${ros_packages[@]}"

# Change to CycloneDDS
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc 