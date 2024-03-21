## Overview

This branch contains a simple bringup package for running RViz2 on host computer.

## Installation

```bash
cd <ros_workspace>/src
git clone -b external-dev https://github.com/grayson-arendt/Lunabotics-2024.git
cd Lunabotics-2024
chmod +x install_dependencies.sh
sudo ./install_dependencies.sh
cd ../..
colcon build
```

## Running RViz2

```bash
ros2 launch lunabot_bringup external_launch.py
```

Once RViz2 loads up, use the "Nav2 Goal" tool to set a location for the robot to navigate to. Wait for the map to load before doing so.
