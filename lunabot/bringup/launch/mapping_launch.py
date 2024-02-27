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

    rtabmap_launch_path = os.path.join(
        get_package_share_path("rtabmap_launch"), "launch", "rtabmap.launch.py"
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        name="static_transform_publisher",
    )

    pose_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "t265_pose_frame", "base_link"],
        output="screen",
        name="static_transform_publisher",
    )

    imu_complementary_filter = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        name="complementary_filter_gain_node",
        output="screen",
        parameters=[
            {"fixed_frame": "odom"},
            {"do_bias_estimation": True},
            {"do_adaptive_gain": True},
            {"use_mag": False},
            {"gain_acc": 0.01},
            {"publish_tf": False},
        ],
    )

    imu_madgwick = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("bringup"), "params", "imu_filter.yaml"
            )
        ],
    )

    imu_rotator = Node(package="autonomous", executable="imu_rotator")

    motor_controller_node = Node(
        package="autonomous", executable="motor_controller"
    )


    odometry_transform = Node(package="autonomous", executable="odometry_transform")

    particle_filter = Node(
        package="autonomous",
        executable="particle_filter",
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
            SetRemap(src="/rtabmap/map", dst="/map"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_path),
                launch_arguments={
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
            motor_controller_node,
            map_to_odom_tf,
            pose_to_base_link_tf,
            rtabmap_launch,
        ]
    )