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
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(apriltag_msgs REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} /usr/include/opencv4)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(pointCloudPipeline src/pointCloudPipeline.cpp)
add_executable(apriltagDetection src/apriltagDetection.cpp)

ament_target_dependencies(pointCloudPipeline rclcpp sensor_msgs pcl_conversions visualization_msgs)
ament_target_dependencies(apriltagDetection rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

target_link_libraries(pointCloudPipeline ${PCL_LIBRARIES})
target_link_libraries(apriltagDetection ${OpenCV_LIBRARIES})


install(
  DIRECTORY phoenix_lib/
  DESTINATION phoenix_lib
)

install(TARGETS
  apriltagDetection
  pointCloudPipeline
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
