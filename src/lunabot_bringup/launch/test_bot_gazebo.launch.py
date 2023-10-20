from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('test_bot_description'),
                             'urdf', 'test_bot.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('test_bot_description'), 
                                    'rviz_config', 'view_urdf_config.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)

    gazebo_launch_file_path = os.path.join(get_package_share_path('gazebo_ros'),
                                           'launch',
                                           'gazebo.launch.py')
    
    gazebo_world_path = os.path.join(get_package_share_path('lunabot_bringup'),
                                     'gazebo_worlds', 'arena1.world')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]        
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "test_bot", '-x', '1.0', '-y', '1.0', '-z', '0.0']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file_path),
            launch_arguments={'world': gazebo_world_path}.items()
        ),
        robot_state_publisher_node,
        rviz2_node,
        spawn_robot_node
    ])
