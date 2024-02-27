import os
import xacro
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetRemap
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events.process import ProcessStarted
from launch.launch_context import LaunchContext
from launch.event_handlers.on_process_start import OnProcessStart
from ament_index_python.packages import get_package_share_path, get_package_share_directory
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

    imu_rotator = Node(
        package="autonomous", 
        executable="imu_rotator",
    )

    motor_controller_node = Node(
        package="autonomous", executable="motor_controller"
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
                "freq": 20.0,
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
            motor_controller_node,
            map_to_odom_tf,
            pose_to_base_link_tf,
            rtabmap_launch,
        ]
    )