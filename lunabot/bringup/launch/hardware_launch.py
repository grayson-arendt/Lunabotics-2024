import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory,
)
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    realsense_launch_path = os.path.join(
        get_package_share_path("realsense2_camera"),
        "launch",
        "rs_multi_camera_launch.py",
    )

    # RPLidar S2L
    lidar1 = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB1",
                "serial_baudrate": 1000000,
                "frame_id": "lidar1_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",
            }
        ],
        output="screen",
    )

    # RPLidar A3
    lidar2 = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        remappings=[("/scan", "/scan2")],
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "scan_frequency": 20.0,
                "serial_baudrate": 256000,
                "frame_id": "lidar2_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }
        ],
        output="screen",
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "camera_name1": "d455",
            "camera_namespace1": "d455",
            "device_type1": "d455",
            "enable_gyro1": "true",
            "enable_accel1": "true",
            "unite_imu_method1": "2",
            "depth_module.profile1": "848x480x30",
            "rgb_camera.profile1": "848x480x30",
            "json_file_path1": "/home/intel-nuc/high_accuracy.json",
            "camera_name2": "t265",
            "camera_namespace2": "t265",
            "device_type2": "t265",
            "enable_gyro2": "true",
            "enable_accel2": "true",
            "enable_pose2": "true",
            "unite_imu_method2": "2",
            "tracking_module.profile2": "848x800x30",
        }.items(),
    )

    lidar1_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("bringup"),
                    "params",
                    "range_filter.yaml",
                ]
            )
        ],
    )

    lidar2_odom = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan2",
                "odom_topic": "/odom_lidar",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 30.0,
            }
        ],
    )

    pose_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "t265_pose_frame", "base_link"],
        output="screen",
        name="static_transform_publisher",
    )

    robot_controller = Node(package="autonomous", executable="robot_controller")

    hardware_monitor = Node(package="autonomous", executable="hardware_monitor")

    apriltag = Node(
        name="apriltag",
        package="apriltag_ros",
        executable="apriltag_node",
        parameters=[
            os.path.join(
                get_package_share_directory("bringup"),
                "params",
                "tag_params.yaml",
            )
        ],
        remappings=[
            ("/image_rect", "/d455/color/image_raw"),
            ("/camera_info", "/d455/color/camera_info"),
            ("/detections", "/tag_detections"),
        ],
    )

    return LaunchDescription(
        [
            lidar1,
            lidar1_filter,
            lidar2,
            lidar2_odom,
            apriltag,
            realsense,
            pose_to_base_link,
            robot_controller,
            hardware_monitor,
        ]
    )
