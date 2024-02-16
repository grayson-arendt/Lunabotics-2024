## Overview

This repository contains code made by the College of DuPage team for the NASA Lunabotics competition. It is made for ROS 2 Humble on Ubuntu 22.04. 

## Dependencies

To use this project, you need to have the following packages installed:
- `rtabmap`
- `rtabmap_ros`
- `navigation2`
- `librealsense2`
- `realsense2_camera`
- `realsense2_camera_dbgsym`
- `realsense2_camera_msgs`
- `realsense2_camera_msgs_dbgsym`
- `realsense2_description`
- `imu_filter_madgwick`
- `foxglove_bridge`
- `robot_state_publisher`
- `joint_state_publisher`
- `tf2_ros`

For each dependency, use `sudo apt install ros-humble-<package_name>`. Make sure the underscores in some of the names are replaced by dashes when using this command.

## Installation
```bash
cd <ros_workspace>/src
git clone -b panasonic-dev https://github.com/grayson-arendt/Lunabotics-2024.git
cd ..
colcon build
```

## Running Robot

If you would like to see RViz2 on your host computer without cloning this branch, use the previous instructions to clone external-dev onto your computer and panasonic-dev onto the robot's computer. Running the second command in step 3 involves no cloning on host computer, but you will have to manually select all the robot topics for visualization.

Each launch file should be ran in a new terminal window. 

#### 1. Navigate to ROS 2 workspace and install (repeat on each new terminal before launches):
```bash
cd <ros_workspace>
source install/setup.bash
```

#### 2. Initialize SocketCAN communication (note: the canableStart.sh script will only need to be ran once each time the robot computer boots up).
```bash
cd <ros_workspace>/src/Lunabotics-2024
chmod +x canableStart.sh # make script executable
./canableStart.sh
```

#### 3. Visualize with RViz2 (host computer):
```bash
ros2 launch bringup external_launch.py
# or, just run RViz2 and manually add in all the topics:
rviz2
```

#### 4. Startup sensors, motors, and RTAB-Map:

```bash
ros2 launch bringup panasonic_launch.py
```

#### 5. Startup Navigation 2 stack:

```bash
ros2 launch bringup navigation_launch.py
```

In RViz2 on the host computer, you will now be able to select a "Nav2 Goal" in the GUI and have the robot navigate to that location. 

## Structure

- **external** (Packages from external sources)
  - rf2o_laser_odometry
  - sllidar_ros2
- **lunabot**  (Contains code written specifically for Lunabotics robot)
  - **autonomous**
    - autonomous_msgs
    - autonomous_pkg
      - **include**
        - autonomous_pkg
          - AprilTag.h (Header for AprilTag detection)
          - CalculateGoal.h (Header for calculating goal based off AprilTag positions)
          - ParticleFilter.h (Header for sensor fusion with a particle filter)
      - ctre (CTRE Phoenix C++ API for using Falcon 500 motors)
      - **src**
        - **camera**
          - camera_synchronization.cpp (Synchronizes RGB, depth, and camera info messages to have same timestamp)
        - **filter**
          - particle_filter.cpp (Fuses lidar odometry, camera odometry, and IMU data to estimate robot pose)     
        - **motor**
          - motor_controller.cpp (Sends percent output commands to motors based off /cmd_vel topic)
          - motor_test.cpp (Simple node for testing motors)
        - **navigation**
          - navigator_client.cpp (Action client for sending goals to Navigation 2 stack)
        - **odometry**
          - odometry_transform.cpp (Odometry to base_link transform broadcaster)
          - wheel_imu_odometry.cpp (Calculates odometry based off both encoders and IMU)
          - wheel_odometry.cpp (Calculates odometry based off only encoders)
        - **utilities**
          - imu_rotator.cpp (Rotates the D455 IMU, since the camera has non-standard orientation)
          - laserscan_to_pointcloud_merger.cpp (Combines two LaserScan messages into one and publishes as a PointCloud2 message)
          - map_merger.cpp (Combines two occupancy grids and publishes the merged occupancy grid)
          - pointcloud_to_laserscan.cpp (Converts PointCloud2 into LaserScan and publishes the message)
  - **bringup** (Contains launch files for running the robot and running Navigation 2 stack)
  - **manual**  (Contains node for driving robot with a controller)
