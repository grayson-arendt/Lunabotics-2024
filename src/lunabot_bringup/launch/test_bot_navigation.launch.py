from launch import LaunchDescription 
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('test_bot_description'),
                             'urdf', 'test_bot.xacro')
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)
    
    ekf_config_path = os.path.join(get_package_share_path('localization_pkg'),
                                   'config', 'ekf.yaml')
    
    slam_toolbox_launch_file_path = os.path.join(get_package_share_path('slam_toolbox'),
                                                 'launch', 'online_async_launch.py')
    
    lidar_launch_path = os.path.join(get_package_share_path('sllidar_ros2'),
                                     'launch', 'sllidar_a3_launch.py')
    
    t265_launch_path = os.path.join(get_package_share_path('realsense2_camera'),
                                    'launch', 'rs_t265_launch.py')
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config_path]
    )
    
    motor_controller_node = Node(
        package='navigation_pkg',
        executable='motor_controller'
    )
    
    initial_pose_publisher_node = Node(
        package='localization_pkg',
        executable='set_initial_pose'
    )
    
    map_transform_node = Node(
        package='localization_pkg',
        executable='map_transform',
        arguments = ['map','odom', '0', '0', '0', '0', '0', '0']
    )
    
    odom_transform_node = Node(
        package='localization_pkg',
        executable='odom_transform',
        arguments = ['odom','base_link', '0', '0', '0', '0', '0', '0']
    )
    
    #rviz_nav2_pluggins_launch_file_path = os.path.join(get_package_share_path('lunabot_bringup'),
    #                                                   'launch','nav2_bringup','bringup','launch','rviz_launch.py')
    
    return LaunchDescription([
        robot_state_publisher_node,
        map_transform_node,
        odom_transform_node,
        #robot_localization_node,
        #motor_controller_node,
        #initial_pose_publisher_node,
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(slam_toolbox_launch_file_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(t265_launch_path)),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(rviz_nav2_pluggins_launch_file_path))
    ])