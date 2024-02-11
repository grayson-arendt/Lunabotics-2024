from launch import LaunchDescription 
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

    DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

    DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
    

    rviz_nav2_plugins_launch_file_path = os.path.join(get_package_share_path('nav2_bringup'),'launch','rviz_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_nav2_plugins_launch_file_path))
    ])