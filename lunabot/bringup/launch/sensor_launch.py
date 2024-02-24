import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetRemap
from launch.substitutions import Command
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events.process import ProcessStarted
from launch.launch_context import LaunchContext
from launch.event_handlers.on_process_start import OnProcessStart
from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory,
)
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    realsense_launch_path = os.path.join(
        get_package_share_path("realsense2_camera"), "launch", "rs_multi_camera_launch.py"
    )

    lidar1 = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        remappings=[("/scan", "/scan1")],
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB1",
                "serial_baudrate": 256000,
                "frame_id": "lidar1_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }
        ],
        output="screen",
    )

    lidar2 = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        remappings=[("/scan", "/scan2")],
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 1000000,
                "frame_id": "lidar2_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
        output="screen",
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "camera_name1": "d455",
            "camera_namespace1": "d455",
            "device_type1": "d455",
            "enable_gyro1": "true",
            "enable_accel1": "true",
            "unite_imu_method1": "2",
            "depth_module.profile1":"848x480x60",
            "rgb_camera.profile1":"848x480x60",
            "camera_name2":"t265",
            "camera_namespace2":"t265",
            "device_type2": "t265",
            "enable_gyro2": "true",
            "enable_accel2": "true",
            "emable_pose2": "true",
            "unite_imu_method2": "2",
        }.items(),
    )

    return LaunchDescription(
        [
            lidar1,
            lidar2,
            realsense_launch,
        ]
    )