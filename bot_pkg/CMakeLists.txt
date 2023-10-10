cmake_minimum_required(VERSION 3.5)
project(bot_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros_phoenix REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(fourWheelDrive src/fourWheelDrive.cpp)
add_executable(twoWheelDrive src/twoWheelDrive.cpp)
add_executable(motorTest src/motorTest.cpp)
add_executable(laserSub src/laserSub.cpp)
add_executable(pointCloudPipeline src/pointCloudPipeline.cpp)
ament_target_dependencies(fourWheelDrive rclcpp ros_phoenix sensor_msgs)
ament_target_dependencies(twoWheelDrive rclcpp ros_phoenix sensor_msgs)
ament_target_dependencies(motorTest rclcpp ros_phoenix sensor_msgs)
ament_target_dependencies(laserSub rclcpp ros_phoenix sensor_msgs)
ament_target_dependencies(pointCloudPipeline rclcpp sensor_msgs pcl_conversions visualization_msgs)

target_link_libraries(pointCloudPipeline ${PCL_LIBRARIES})

install(TARGETS
  twoWheelDrive
  fourWheelDrive
  motorTest
  laserSub
  pointCloudPipeline
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  scripts/controller.py
#  DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
