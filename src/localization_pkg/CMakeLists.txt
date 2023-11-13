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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(wheel_odom_publisher src/wheel_odom_publisher.cpp)

ament_target_dependencies(wheel_odom_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros sensor_msgs)


install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  wheel_odom_publisher
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
