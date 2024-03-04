## Overview

This repository contains code made by the College of DuPage team for the NASA Lunabotics competition. It is made for ROS 2 Humble on Ubuntu 22.04, kernel 5.15. 

## Hardware

- `Intel NUC 13 Pro`
- `RPLidar S2L`
- `RPLidar A3`
- `Intel RealSense D455 Depth Camera`
- `Intel RealSense T265 Tracking Camera`
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
- `laser_filters`

## Installation

#### 1. Download the source code for [this](https://github.com/IntelRealSense/librealsense/releases/tag/v2.53.1) release and install.
`Note: this repository contains realsense-ros version 4.51.1 in external directory. This is because support was dropped for the T265 camera in later releases.`

```bash
cd Downloads/
tar -xzvf librealsense-2.53.1.tar.gz
cd librealsense-2.53.1/
mkdir build && cd build
cmake ../ -DFORCE_RSUSB_BACKEND=false -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
sudo make uninstall && make clean && make -j8 && sudo make install
cd ..
cd scripts
./setup_udev_rules.sh
```

The flag -DFORCE_RSUSB_BACKEND=false will only work with librealsense's supported kernel versions. If this is ran on a version above 5.15, set it to true. To check kernel version, type the command: `uname -r`. If you are using a newer kernel version and want to downgrade, run `sudo apt install linux-image-generic`, reboot the computer, press shift to go GRUB menu (as soon as Ubuntu logo comes up), select `Advanced options for Ubuntu` then use arrow keys/enter to select kernel 5.15. 

The NUC WiFi did not work when I did this, but I ran `sudo apt purge backport-iwlwifi-dkms` then `sudo apt install backport-iwlwifi-dkms` to re-install the driver. After rebooting, the WiFi worked again.

#### 2. Next, clone and build the repository.

```bash
cd <ros_workspace>/src
git clone https://github.com/grayson-arendt/Lunabotics-2024.git
```

#### 3. Run the install_dependencies script to install the required dependencies and build.

```bash
cd <ros_workspace>/src/Lunabotics-2024/scripts
chmod +x install_dependencies.sh
sudo ./install_dependencies.sh
cd ..
colcon build
sudo apt --fix-broken install
```

The install_dependencies.sh script will also remove librealsense2 and realsense2_camera packages. I have not been able to find another way around this yet, but RTAB-Map depends on librealsense2 and realsense2_camera and will cause broken dependencies. However, when the workspace is built, it will choose to use the newest realsense2_ros and not the one in the external directory, which will cause the T265 camera not to work. The current work around is to remove the packages before building the workspace, then fixing the broken dependencies afterwards.

#### 4. Repeat the last two steps (excluding the last line) with the [external-dev](https://github.com/grayson-arendt/Lunabotics-2024/tree/external-dev?tab=readme-ov-file) branch on the host computer (not robot computer). This branch is for visualizing the robot in RViz2.

## Setup Permissions and CTRE Phoenix Library

The rplidar_ros package needs to access /dev/ttyUSB0 and /dev/ttyUSB1 (using both lidars). While you can run `sudo chmod +x 777 /dev/ttyUSB0` for example, it would need to be ran each time on startup. To fix this, run the command below and restart the computer.

```bash
sudo usermod -a -G dialout $USER
```

If the lidars are not under /dev/ttyUSB0 and /dev/ttyUSB1 (which may happen when disconnected and reconnected), use this command with one lidar connected at a time to check the number at the end. Adjust the parameter in hardware_launch.py based off the number. 

```bash
ls /dev/ttyUSB*
```

The computer may not be able to find the shared object files for CTRE Phoenix library. An easy way to fix this is to directly copy them into /usr/lib/.

```bash
cd <ros_workspace>/src/Lunabotics-2024/lunabot/autonomous/phoenix_lib/x86-64/
sudo cp libCTRE_Phoenix.so libCTRE_PhoenixCCI.so libCTRE_PhoenixTools.so /usr/lib/
```

## Running Robot

Each launch file should be ran in a new terminal window. 

`Note: unplug and replug in the T265 after booting up the NUC, it will not detect it if it is not replugged in again.`

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
```

#### 4. Startup hardware:

```bash
ros2 launch bringup hardware_launch.py
```

#### 5. Startup RTAB-Map:

```bash
ros2 launch bringup mapping_launch.py
```

#### 6. Startup Navigation2:

```bash
ros2 launch bringup navigation_launch.py
```

In RViz2 on the host computer, you will now be able to select a "Nav2 Goal" in the GUI and have the robot navigate to that location. 

#### (Optional) 7. Startup action client:

```bash
ros2 run autonomous navigation_client
```

The action client will send two goals, one for excavation zone and another for construction zone. After the goal has been reached, it will publish to /control topic and enable the specific
motors for the mechanisms for the zone.

![Demo](demo.png)

## Structure

- **lunabot** (Contains code for Lunabotics robot)
  - **autonomous**
    - **include**
      - **ctre** (CTRE Phoenix C++ API for using Falcon 500 motors)
    - **phoenix_lib** (Contains shared object files for CTRE Phoenix C++ API)
    - **src**
      - hardware_monitor.cpp (Monitors liveliness of hardware topics)
      - motor_test.cpp (Simple node for testing motors)
      - navigator_client.cpp (Action client that sends goals and motor control commands)
      - robot_controller.cpp (Controller for both autonomous and manual control of robot)
  - **bringup** 
    - **behavior_trees**
      - navigate_to_pose_w_replanning_goal_patience_and_recovery.xml (Behavior tree for Navigation2)
    - **launch**
      - external_launch.py (Launches RViz2 and robot state/joint publisher nodes)
      - hardware_launch.py (Launches lidars, cameras, and robot_controller)
      - mapping_launch.py (Launches RTAB-Map)
      - navigation_launch.py (Launches Navigation2)
    - **params**
      - default_view.rviz (RViz2 configuration file)
      - nav2_params.yaml (Parameters for Navigation2)
      - range_filter.yaml (Filter for lidar)
  - **description** 
    - **meshes** (Meshes for robot model in RViz2)
      - base_link.stl
      - camera_link.stl
      - ebox_link.stl
      - lidar_link.stl
      - wheel_link.stl
    - **urdf**
      - common_properties.xacro (Defines material colors)
      - test_bot.xacro (Defines links and joints for test bot)
  - **external** (Packages from external sources)
    - realsense_ros (Version 4.51.1)
    - rf2o_laser_odometry

