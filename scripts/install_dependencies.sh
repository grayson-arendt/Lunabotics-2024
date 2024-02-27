#!/bin/bash

ros_packages=(
    "ros-humble-rtabmap"
    "ros-humble-rtabmap-ros"
    "ros-humble-rplidar-ros"
    "ros-humble-navigation2"
    "ros-humble-robot-localization"
    "ros-humble-robot-state-publisher"
    "ros-humble-joint-state-publisher"
    "ros-humble-imu-filter-madgwick"
)

# Install ROS 2 packages
apt install -y "${ros_packages[@]}"