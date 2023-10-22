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

    gazebo_launch_file_path = os.path.join(get_package_share_path('gazebo_ros'),
                                           'launch',
                                           'gazebo.launch.py')
    
    gazebo_world_path = os.path.join(get_package_share_path('lunabot_bringup'),
                                     'gazebo_worlds', 'arena1.world')
    
    ekf_config_path = os.path.join(get_package_share_path('localization_pkg'),
                                   'config', 'ekf.yaml')
    
    slam_toolbox_launch_file_path = os.path.join(get_package_share_path('slam_toolbox'),
                                                 'launch', 'online_async_launch.py')
    
    rviz_nav2_pluggins_launch_file_path = '/home/anthony/lunabot_23-24/src/Lunabotics-2024/src/lunabot_bringup/launch/nav2_bringup/bringup/launch/rviz_launch.py'
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "test_bot", '-x', '1.0', '-y', '1.0', '-z', '0.0']
    )
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file_path),
            launch_arguments={'world': gazebo_world_path}.items()
        ),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        spawn_robot_node,
        robot_localization_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_nav2_pluggins_launch_file_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file_path))    
    ])
