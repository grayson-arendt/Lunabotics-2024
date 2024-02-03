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
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autonomous_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(libfreenect CONFIG PATHS "/usr/local/lib64/cmake/libfreenect")
find_package(image_transport REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(camera_info_manager REQUIRED)
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

add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan.cpp)
add_executable(imu_rotator src/imu_rotator.cpp)
add_executable(state_estimation src/state_estimation.cpp)
add_executable(map_merger src/map_merger.cpp)
add_executable(camera_calibration src/camera_calibration.cpp)
add_executable(sync_camera src/sync_camera.cpp)
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
add_executable(pose_publisher src/pose_publisher.cpp)
add_executable(kinect_publisher src/kinect_publisher.cpp)
add_executable(wheel_odom_publisher src/imu_wheel_odom_publisher.cpp)
add_executable(odom_transform src/odom_transform.cpp)

ament_target_dependencies(camera_calibration OpenCV)
ament_target_dependencies(imu_rotator rclcpp sensor_msgs)
ament_target_dependencies(map_merger rclcpp nav_msgs)
ament_target_dependencies(pointcloud_to_laserscan rclcpp sensor_msgs)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs std_msgs sensor_msgs tf2 tf2_ros cv_bridge OpenCV)
ament_target_dependencies(pose_publisher rclcpp nav_msgs geometry_msgs)
ament_target_dependencies(sync_camera rclcpp sensor_msgs message_filters)
ament_target_dependencies(state_estimation rclcpp nav_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(wheel_odom_publisher rclcpp nav_msgs std_msgs geometry_msgs tf2 tf2_ros sensor_msgs autonomous_msgs)
ament_target_dependencies(odom_transform rclcpp nav_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(kinect_publisher rclcpp sensor_msgs std_msgs cv_bridge camera_info_manager rclcpp_components ament_index_cpp image_transport OpenCV)

target_link_libraries(kinect_publisher
  freenect
)


install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  imu_rotator
  sync_camera
  map_merger
  kinect_publisher
  initial_pose_publisher
  pose_publisher
  state_estimation
  wheel_odom_publisher
  pointcloud_to_laserscan
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
