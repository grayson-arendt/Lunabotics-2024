## Overview

This repository contains code made by the College of DuPage team for the NASA Lunabotics competition. It is made for ROS 2 Humble on Ubuntu 22.04. 

## Hardware

- `Intel NUC 13 Pro`
- `RPLidar S2L`
- `RPLidar A3`
- `Intel Realsense D455 Depth Camera`
- `Intel Realsense T265 Tracking Camera`
- `CTRE Falcon 500 (x2)`
- `RoyPow 12V 18Ah LiFePO4 Battery`
- `Turnigy 14.8V 12000mAh LiPo Battery`
- `AndyMark Power Distribution Panel`
- `MKS CANable Pro`

## Dependencies

- `rtabmap`
- `rtabmap_ros`
- `rplidar_ros`
- `navigation2`
- `robot_localization`
- `robot_state_publisher`
- `joint_state_publisher`

## Installation

Download the source code for [this](https://github.com/IntelRealSense/librealsense/releases/tag/v2.53.1) release.
`Note: this repository contains realsense-ros version 4.51.1 in external directory. This is because support was dropped for the T265 camera in later releases.`

```bash
cd Downloads/
tar -xzvf librealsense-2.53.1.tar.gz
cd librealsense-2.53.1/
mkdir build && cd build
cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
sudo make uninstall && make clean && make -j8 && sudo make install
cd ..
cd scripts
./setup_udev_rules.sh
```

Next, clone and build the repository.

```bash
cd <ros_workspace>/src
git clone -b intel-dev https://github.com/grayson-arendt/Lunabotics-2024.git
cd ..
colcon build
```

Run the install_dependencies script to install the required dependencies.

```bash
cd <ros_workspace>/src/Lunabotics-2024/scripts
chmod +x install_dependencies.sh
sudo ./install_dependencies.sh
```
## Setup Permissions and CTRE Phoenix Library

The rplidar_ros package needs to access /dev/ttyUSB0 and /dev/ttyUSB1 (using both lidars). While you can run `sudo chmod +x 777 /dev/ttyUSB0` for example, it would need to be ran each time on startup. To fix this, run the command below and restart the computer.

```bash
sudo usermod -a -G dialout $USER
```

The computer may not be able to find the shared object files for CTRE Phoenix library. An easy way to fix this is to directly copy them into /usr/lib/.

```bash
cd <ros_workspace>/src/Lunabotics-2024/lunabot/autonomous_pkg/phoenix_lib/x86-64/
sudo cp libCTRE_Phoenix.so libCTRE_PhoenixCCI.so libCTRE_PhoenixTools.so /usr/lib/
```

## Running Robot

If you would like to see RViz2 on your host computer without cloning this branch, use the previous instructions to clone external-dev onto your computer and intel-dev onto the robot's computer. Running the second command in step 3 involves no cloning on host computer, but you will have to manually select all the robot topics for visualization.

Each launch file should be ran in a new terminal window. 

#### 1. Navigate to ROS 2 workspace and install (repeat on each new terminal before launches):
```bash
cd <ros_workspace>
source install/setup.bash
```

#### 2. Initialize SocketCAN communication (note: the canable_start.sh script will only need to be ran once each time the robot computer boots up).
```bash
cd <ros_workspace>/src/Lunabotics-2024/scripts
chmod +x canable_start.sh 
./canable_start.sh
```

#### 3. Visualize with RViz2 (host computer):
```bash
ros2 launch bringup external_launch.py
# or, just run RViz2 and manually add in all the topics:
rviz2
```

#### 4. Startup sensors:

```bash
ros2 launch bringup sensor_launch.py
```
`Note: this is a separate launch file in order to check conditions of sensors before trying to start everything else.`

#### 5. Startup RTAB-Map and other nodes:

```bash
ros2 launch bringup mapping_launch.py
```

#### 6. Startup Navigation 2 stack:

```bash
ros2 launch bringup navigation_launch.py
```

In RViz2 on the host computer, you will now be able to select a "Nav2 Goal" in the GUI and have the robot navigate to that location. 

## Structure

- **external** (Packages from external sources)
  - realsense_ros
  - rf2o_laser_odometry
- **lunabot**  (Contains code written specifically for Lunabotics robot)
  - **autonomous**
      - **include**
      - ctre (CTRE Phoenix C++ API for using Falcon 500 motors)
      - **src**   
        - **motor**
          - motor_controller.cpp (Sends percent output commands to motors based off /cmd_vel topic)
          - motor_test.cpp (Simple node for testing motors)
        - **navigation**
          - navigator_client.cpp (Action client for sending goals to Navigation 2 stack)
        - **utilities**
          - imu_rotator_d455.cpp (Rotates the D455 IMU to be ENU)
          - imu_rotator_t265.cpp (Rotates the T265 IMU to be ENU)
  - **bringup** (Contains launch files for running the robot, mapping with RTAB-Map, and navigating with the Navigation 2 stack)
  - **description** (Contains URDF for robot model)
  - **manual**  (Contains node for driving robot with a controller)
