#### Running RViz2
```bash
ros2 launch bringup external_launch.py
# or, just run RViz2 and manually add in all the topics
rviz2
```

#### Running robot code (when connected via SSH)
```bash
ros2 launch bringup panasonic_launch.py
```

#### Project Structure
- **external** (Packages from external sources)
  - imu_tools
  - navigation2
  - rf2o_laser_odometry
  - slam_toolbox
  - sllidar_ros2
- **lunabot**  (Contains code written specifically for Lunabotics robot)
  - **autonomous**
    - autonomous_msgs
    - autonomous_pkg
      - **include**
        - autonomous_pkg
          - AprilTag.h (Header for AprilTag detection)
          - CalculateGoal.h (Header for calculating goal based off AprilTag positions)
          - ParticleFilter.h (Header for particle filter implementation)
      - ctre (CTRE Phoenix C++ API for using Falcon 500 motors)
      - **src**
        - **camera**
          - camera_calibration.cpp (Camera calibration for OpenCV (for AprilTag pose detection)
          - camera_synchronization.cpp (Synchronizes RGB, depth, and camera info messages to have same timestamp)
        - **filter**
          - particle_filter.cpp (Particle filter to fuse lidar, camera, and IMU data)
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
