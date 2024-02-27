cmake_minimum_required(VERSION 3.5)
project(autonomous)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Filter
add_executable(particle_filter src/filter/particle_filter.cpp)

# Motor
add_executable(motor_controller src/motor/motor_controller.cpp)
add_executable(motor_test src/motor/motor_test.cpp)

# Navigation
add_executable(navigator_client src/navigation/navigator_client.cpp)

# Utilities
add_executable(imu_rotator src/utilities/imu_rotator.cpp)
add_executable(odometry_transform src/utilities/odometry_transform.cpp)

ament_target_dependencies(particle_filter rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(motor_controller rclcpp geometry_msgs)
ament_target_dependencies(motor_test rclcpp sensor_msgs)
ament_target_dependencies(navigator_client rclcpp rclcpp_action nav2_msgs geometry_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(odometry_transform rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  particle_filter
  motor_controller
  motor_test
  navigator_client
  imu_rotator
  odometry_transform
  DESTINATION lib/${PROJECT_NAME}
)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/x86-64)
endif()

target_link_libraries(motor_controller ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(motor_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(motor_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

ament_package()
