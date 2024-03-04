#!/bin/bash

ros_packages=(
    "ros-humble-rtabmap"
    "ros-humble-rtabmap-ros"
    "ros-humble-rplidar-ros"
    "ros-humble-navigation2"
    "ros-humble-laser-filters"
    "ros-humble-rmw-cyclonedds-cpp"
)

remove_packages=(
    "ros-humble-librealsense2"
    "ros-humble-realsense2-camera"
)

# Install ROS 2 packages
apt install -y "${ros_packages[@]}"

# Remove the newer versions to be able to use older version in workspace
dpkg -r --force-depends "${remove_packages[@]}"

# Change to CycloneDDS
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc 