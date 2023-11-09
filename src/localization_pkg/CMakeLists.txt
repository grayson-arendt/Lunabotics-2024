cmake_minimum_required(VERSION 3.5)
project(localization_pkg)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(test_goal_publisher src/test_goal_publisher.cpp)
add_executable(set_initial_pose src/set_initial_pose.cpp)
add_executable(map_transform src/map_transform.cpp)
add_executable(odom_transform src/odom_transform.cpp)

ament_target_dependencies(test_goal_publisher rclcpp geometry_msgs)
ament_target_dependencies(set_initial_pose rclcpp geometry_msgs)
ament_target_dependencies(map_transform rclcpp geometry_msgs tf2 tf2_ros)
ament_target_dependencies(odom_transform rclcpp geometry_msgs tf2 tf2_ros)

install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  test_goal_publisher
  set_initial_pose
  map_transform
  odom_transform
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
