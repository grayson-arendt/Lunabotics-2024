This repository will contain our Lunabotics 2023-2024 robot code. Currently, apriltag_ros, apriltag_msgs, and realsense-ros packages are being used for ROS2. CTRE Phoenix library is being used to control our motors, and PCL (Point Cloud Library) is used to manage the camera's point cloud data,

src contains the packages that run our robot, while everything outside of the directory is an external package that we utilize. 

Inside src, autonomous_pkg will contain the necessary code for running the robot autonomously, while manual_control_pkg will contain teleop code for driving the robot manually. lunabot_bringup contains all the launch files for our robot.
