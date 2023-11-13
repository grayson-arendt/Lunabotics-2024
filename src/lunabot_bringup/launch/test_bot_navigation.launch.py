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
    
    slam_toolbox_launch_file_path = os.path.join(get_package_share_path('slam_toolbox'),
                                                 'launch', 'online_async_launch.py')
    
    lidar_launch_path = os.path.join(get_package_share_path('sllidar_ros2'),
                                     'launch', 'sllidar_a3_launch.py')
    
    laser_scan_config_path = os.path.join(get_package_share_path('lunabot_bringup'),
                                          'config', 'laser_filter.yaml')
    
    t265_launch_path = os.path.join(get_package_share_path('realsense2_camera'),
                                    'launch', 'rs_t265_launch.py')

    d455_launch_path = os.path.join(get_package_share_path('realsense2_camera'),
                                    'launch', 'rs_d455_launch.py')
    
    bringup_dir = os.path.join(get_package_share_directory('lunabot_bringup'),
                               'launch','nav2_bringup')
    
    imu_filter_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config', 'imu_filter.yaml')

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

    # might not even be necessary tbh
    base_footprint_to_base_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.0889', '0', '0', '0', 'base_footprint', 'base_link'],
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
    
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_scan_config_path]
    )

    '''
    imu_filter = Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[imu_filter_dir]
    )
    '''

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

    motor_controller_node = Node(
        package='navigation_pkg',
        executable='motor_controller'
    )

    wheel_odom_publisher = Node(
        package="localization_pkg",
        executable="wheel_odom_publisher"
    )

    filtered_odom_transform = Node(
        package="localization_pkg",
        executable="filtered_odom_transform"
    )

    rviz_nav2_plugins_launch_file_path = os.path.join(get_package_share_path('nav2_bringup'),'launch','rviz_launch.py')

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        motor_controller_node,
        robot_localization_node,
        wheel_odom_publisher,
        filtered_odom_transform,
        base_footprint_to_base_link,
        map_to_odom_tf,
        laser_filter,
        imu_filter,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(d455_launch_path)),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(t265_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_nav2_plugins_launch_file_path))
    ])