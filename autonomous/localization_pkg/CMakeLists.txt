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
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_pkg REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

include_directories(include)

add_executable(pointcloud_pipeline src/pointcloud_pipeline.cpp)
add_executable(pointcloud2d_publisher src/pointcloud2d_publisher.cpp)
add_executable(imu_rotator src/imu_rotator.cpp)
add_executable(sync_camera src/sync_camera.cpp)
add_executable(wheel_odom_publisher src/imu_wheel_odom_publisher.cpp)
add_executable(odom_transform src/odom_transform.cpp)

ament_target_dependencies(pointcloud_pipeline rclcpp sensor_msgs pcl_conversions visualization_msgs)
ament_target_dependencies(pointcloud2d_publisher rclcpp std_msgs message_filters sensor_msgs pcl_conversions visualization_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(sync_camera rclcpp sensor_msgs message_filters)
ament_target_dependencies(wheel_odom_publisher rclcpp nav_msgs std_msgs geometry_msgs tf2 tf2_ros sensor_msgs navigation_pkg)
ament_target_dependencies(odom_transform rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

target_link_libraries(pointcloud_pipeline ${PCL_LIBRARIES})
target_link_libraries(pointcloud2d_publisher ${PCL_LIBRARIES})

install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  imu_rotator
  sync_camera
  wheel_odom_publisher
  pointcloud_pipeline
  pointcloud2d_publisher
  odom_transform
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/x86-64)
endif()

target_link_libraries(wheel_odom_publisher ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(wheel_odom_publisher ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(wheel_odom_publisher ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

ament_package()
