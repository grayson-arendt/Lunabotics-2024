cmake_minimum_required(VERSION 3.5)
project(autonomous_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} /usr/include/opencv4
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

include_directories(include)

# Camera
add_executable(camera_synchronization src/camera/camera_synchronization.cpp)

# Filter
add_executable(particle_filter src/filter/particle_filter.cpp)

# Motor
add_executable(motor_controller src/motor/motor_controller.cpp)
add_executable(motor_test src/motor/motor_test.cpp)

# Navigation
add_executable(navigator_client src/navigation/navigator_client.cpp)

# Odometry
add_executable(wheel_imu_odometry src/odometry/wheel_imu_odometry.cpp)
add_executable(odometry_transform src/odometry/odometry_transform.cpp)

# Utilities
add_executable(imu_rotator src/utilities/imu_rotator.cpp)
add_executable(map_merger src/utilities/map_merger.cpp)
add_executable(pointcloud_to_laserscan src/utilities/pointcloud_to_laserscan.cpp)
add_executable(laserscan_to_pointcloud_merger src/utilities/laserscan_to_pointcloud_merger.cpp)

ament_target_dependencies(camera_synchronization rclcpp sensor_msgs message_filters)
ament_target_dependencies(particle_filter rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(motor_controller rclcpp geometry_msgs autonomous_msgs)
ament_target_dependencies(motor_test rclcpp sensor_msgs)
ament_target_dependencies(navigator_client rclcpp rclcpp_action nav2_msgs geometry_msgs)
ament_target_dependencies(odometry_transform rclcpp nav_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(wheel_imu_odometry rclcpp nav_msgs std_msgs geometry_msgs tf2 tf2_ros sensor_msgs autonomous_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(map_merger rclcpp nav_msgs)
ament_target_dependencies(pointcloud_to_laserscan rclcpp sensor_msgs)
ament_target_dependencies(laserscan_to_pointcloud_merger rclcpp sensor_msgs pcl_conversions)

target_link_libraries(laserscan_to_pointcloud_merger
  ${PCL_LIBRARIES}
)

install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  camera_synchronization
  particle_filter
  motor_controller
  motor_test
  navigator_client
  odometry_transform
  wheel_imu_odometry
  imu_rotator
  map_merger
  pointcloud_to_laserscan
  laserscan_to_pointcloud_merger
  DESTINATION lib/${PROJECT_NAME}
)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/x86-64)
endif()

target_link_libraries(wheel_imu_odometry ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(wheel_imu_odometry ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(wheel_imu_odometry ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

target_link_libraries(motor_controller ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(motor_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(motor_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

ament_package()
