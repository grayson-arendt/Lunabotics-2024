cmake_minimum_required(VERSION 3.5)
project(lunabot_autonomous)

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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
)

include_directories(include)

add_executable(hardware_monitor src/hardware_monitor.cpp)
add_executable(imu_rotator src/imu_rotator.cpp)
add_executable(motor_test src/motor_test.cpp)
add_executable(navigator_client src/navigator_client.cpp)
add_executable(robot_controller src/robot_controller.cpp)


ament_target_dependencies(hardware_monitor rclcpp sensor_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(motor_test rclcpp sensor_msgs)
ament_target_dependencies(navigator_client rclcpp rclcpp_action nav2_msgs geometry_msgs)
ament_target_dependencies(robot_controller rclcpp sensor_msgs geometry_msgs)

rosidl_target_interfaces(robot_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(navigator_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  hardware_monitor
  imu_rotator
  motor_test
  navigator_client
  robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") 
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_lib/x86-64)
endif()

target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(robot_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

ament_package()
