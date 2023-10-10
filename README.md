This repository will contain our Lunabotics 2023-2024 robot code. Currently, ros_phoenix and realsense-ros packages are being used.

/launch currently contains launch files to initialize the motors with ros_phoenix and run other nodes.
/scripts is currently empty, but Python files would go there.
/src contains ROS2 nodes for running our robot.

CMakeLists.txt contains everything needed to compile and run the actual code itself, including finding external libraries like PCL and creating executables.
package.xml lists certain information about bot_pkg (like what its dependencies are).