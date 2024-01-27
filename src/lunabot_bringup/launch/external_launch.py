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
    
    rtabmap_launch_path = os.path.join(get_package_share_path('rtabmap_ros'),
                                    'launch', 'rtabmap.launch.py')

    rviz_nav2_plugins_launch_file_path = os.path.join(get_package_share_path('nav2_bringup'),'launch','rviz_launch.py')

    rtabmap_launch = GroupAction(actions=[
                SetRemap(src='/rtabmap/map', dst='/map'),
                        IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(rtabmap_launch_path),
                        launch_arguments={
                        'rtabmapviz': 'true',
                        'frame_id': 'base_link',
                        'args': '-d -Vis/MinDepth 1.0 -Grid/Sensor 2 -Grid/RangeMin 1.0 -Reg/Force3DoF true -Reg/Strategy 2 -Grid/RayTracing true',
                        'rgb_topic':'/camera/color/image_raw',
                        'depth_topic':'/camera/depth/image_rect_raw',
                        'camera_info_topic':'/camera/depth/camera_info',
                        'subscribe_rgb': 'true',
                        'subscribe_depth': 'true',
                        'subscribe_scan':'true',
                        'approx_sync': 'true',
                        'rviz': 'false',
                        'queue_size': '300', 
                        }.items(),)])

    return LaunchDescription([
        rtabmap_launch,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_nav2_plugins_launch_file_path))
    ])