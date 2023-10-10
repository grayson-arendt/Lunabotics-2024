cmake_minimum_required(VERSION 3.8)
project(manual_control_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(manual_control src/manual_control.cpp)
ament_target_dependencies(manual_control rclcpp sensor_msgs)

install(TARGETS
  manual_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
