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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_path("robot_description"), "urdf", "test_bot.xacro"
    )

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    rtabmap_launch_path = os.path.join(
        get_package_share_path("rtabmap_launch"), "launch", "rtabmap.launch.py"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": False}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        name="static_transform_publisher",
    )

    imu_rotator_d455 = Node(package="autonomous_pkg", executable="imu_rotator_d455")

    imu_rotator_t265 = Node(package="autonomous_pkg", executable="imu_rotator_t265")

    motor_controller_node = Node(
        package="autonomous_pkg", executable="motor_controller"
    )

    lidar_odom = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan1",
                "odom_topic": "/odom_lidar",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 35.0,
            }
        ],
    )

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("bringup"), 'params', 'ekf.yaml')],
    )


    rtabmap_launch = GroupAction(
        actions=[
            SetRemap(src="/rtabmap_d455/map", dst="/map"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_path),
                launch_arguments={
                    "namespace": "rtabmap_d455",
                    "rtabmapviz": "false",
                    "frame_id": "base_link",
                    "args": "-d -Rtabmap/DetectionRate 10 -Optimizer/Robust true -Grid/Sensor 2 -Grid/RangeMin 0.5 -Reg/Force3DoF true -Reg/Strategy 0 -Grid/MaxObstacleHeight 2.0 -Grid/RayTracing true",
                    "rgb_topic": "/d455/color/image_raw",
                    "depth_topic": "/d455/depth/image_rect_raw",
                    "camera_info_topic": "/d455/depth/camera_info",
                    "subscribe_rgb": "true",
                    "subscribe_depth": "true",
                    "subscribe_scan": "true",
                    "scan_topic":"/scan2",
                    "subscribe_odom_info":"false",
                    "approx_sync": "true",
                    "rviz": "false",
                    "publish_tf_odom": "false",
                    "publish_tf_map": "false",
                    "publish_null_when_lost": "false",
                    "queue_size": "300",
                }.items(),
            ),
        ]
    )  


    return LaunchDescription(
        [
            lidar_odom,
            imu_rotator_d455,
            imu_rotator_t265,
            ekf_node,
            motor_controller_node,
            robot_state_publisher_node,
            joint_state_publisher_node,
            map_to_odom_tf,
            rtabmap_launch,
        ]
    )
