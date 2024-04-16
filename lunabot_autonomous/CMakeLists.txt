cmake_minimum_required(VERSION 3.5)
project(lunabot_autonomous)

add_definitions(-w)

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
find_package(phoenix6 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "msg/MotorOutput.msg"
)

add_executable(hardware_monitor src/system/hardware_monitor.cpp)
add_executable(imu_rotator src/system/imu_rotator.cpp)
add_executable(navigator_client src/system/navigator_client.cpp)

add_executable(motor_test src/control/motor_test.cpp)
add_executable(robot_controller src/control/robot_controller.cpp)
add_executable(kraken_controller src/control/kraken_controller.cpp)
add_executable(talon_srx_controller src/control/talon_srx_controller.cpp)

ament_target_dependencies(hardware_monitor rclcpp sensor_msgs)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(motor_test rclcpp sensor_msgs)
ament_target_dependencies(navigator_client rclcpp rclcpp_action nav2_msgs geometry_msgs)
ament_target_dependencies(robot_controller rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(kraken_controller rclcpp phoenix6)
ament_target_dependencies(talon_srx_controller rclcpp phoenix6)

target_include_directories(kraken_controller PRIVATE /usr/include/phoenix6)
target_include_directories(talon_srx_controller PRIVATE include)
target_include_directories(motor_test PRIVATE include)

rosidl_target_interfaces(robot_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(kraken_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(talon_srx_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(navigator_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  hardware_monitor
  imu_rotator
  motor_test
  navigator_client
  robot_controller
  kraken_controller
  talon_srx_controller
  DESTINATION lib/${PROJECT_NAME}
)

set(PHOENIX_LIBS_DIR /usr/lib/phoenix)

target_link_libraries(talon_srx_controller ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(talon_srx_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(talon_srx_controller ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(motor_test ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

ament_package()
