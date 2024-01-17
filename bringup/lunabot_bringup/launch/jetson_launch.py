from launch import LaunchDescription 
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('test_bot_description'),
                             'urdf', 'test_bot.xacro')
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)
    
    ekf_config_path = os.path.join(get_package_share_path('localization_pkg'),
                                   'config', 'ekf_.yaml')
    
    lidar_launch_path = os.path.join(get_package_share_path('sllidar_ros2'),
                                     'launch', 'sllidar_a3_launch.py')

    d455_launch_path = os.path.join(get_package_share_path('realsense2_camera'),
                                    'launch', 'rs_d455_launch.py')

    slam_toolbox_launch_file_path = os.path.join(get_package_share_path('slam_toolbox'),
                                                 'launch', 'online_async_launch.py')

    
    DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

    DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

    DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}, {"use_sim_time": False}]
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}]
    )

    map_to_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
            name='static_transform_publisher',
    )

    map_to_camera_link_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link'],
            output='screen',
            name='static_transform_publisher',
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config_path]
    )
    
    imu_filter = Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                    {'publish_tf': True},
                ],
            )

    imu_rotator = Node(
        package='localization_pkg',
        executable='imu_rotator'
    )

    motor_controller_node = Node(
        package='navigation_pkg',
        executable='motor_controller'
    )

    sync_camera = Node(
        package='localization_pkg',
        executable='sync_camera'
    )

    wheel_odom_publisher = Node(
        package="localization_pkg",
        executable="wheel_odom_publisher"
    )

    filtered_odom_transform = Node(
        package="localization_pkg",
        executable="filtered_odom_transform"
    )

    # removed
    # base_link_to_laser_tf,
    #   base_link_to_odom_tf,

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        motor_controller_node,
        robot_localization_node,
        wheel_odom_publisher,
        filtered_odom_transform,
        map_to_odom_tf,
        sync_camera,
        imu_filter,
        imu_rotator,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(d455_launch_path)),
    ])