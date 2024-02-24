#!/bin/sh

# Install ROS 2 packages
apt install -y \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-navigation2 \
    ros-humble-robot-localization \
    ros-humble-image-publisher \
    ros-humble-depth-image-proc \
    ros-humble-imu-filter-madgwick \
    ros-humble-imu-complementary-filter \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros