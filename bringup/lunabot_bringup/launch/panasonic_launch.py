import os
from launch import LaunchDescription 
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetRemap
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events.process import ProcessStarted
from launch.launch_context import LaunchContext
from launch.event_handlers.on_process_start import OnProcessStart
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from nav2_common.launch import RewrittenYaml


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
                                    'launch', 'rs_launch.py')

    slam_toolbox_launch_file_path = os.path.join(get_package_share_path('slam_toolbox'),
                                                 'launch', 'online_async_launch.py')

    rtabmap_launch_path = os.path.join(get_package_share_path('rtabmap_launch'),
                                    'launch', 'rtabmap.launch.py')

    rviz_nav2_plugins_launch_file_path = os.path.join(get_package_share_path('nav2_bringup'),'launch','rviz_launch.py')

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
    
    map_to_base_link_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
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

    wheel_odom_publisher = Node(
        package="localization_pkg",
        executable="wheel_odom_publisher"
    )

    initial_pose_publisher = Node(
        package="localization_pkg",
        executable="initial_pose_publisher"
    )

    pose_publisher = Node(
        package="localization_pkg",
        executable="pose_publisher"
    )

    odom_transform = Node(
        package="localization_pkg",
        executable="odom_transform"
    )

    map_merger = Node(
        package="localization_pkg",
        executable="map_merger"
    )

    sync_camera = Node(
        package="localization_pkg",
        executable="sync_camera"
    )

    #pointcloud_to_laserscan = Node(
    #    package="localization_pkg",
    #    executable="pointcloud_to_laserscan"
    #)

    laser_odom = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],
    )

    kinect = Node(
                package="localization_pkg",
                executable="kinect_publisher",
    )

    slam_toolbox_launch = GroupAction(actions=[
            SetRemap(src='/map',dst='/slam_toolbox/map'),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file_path))])

    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path))

    camera_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(d455_launch_path),
                launch_arguments={'device_type': 'd455', 'enable_pointcloud':'true','enable_gyro': 'true', 'enable_accel': 'true','unite_imu_method': '2',}.items(),)

    rtabmap_launch = GroupAction(actions=[
    SetRemap(src='/rtabmap/map', dst='/map'),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rtabmapviz': 'false',
            'frame_id': 'base_link',
            'args': '-d -Grid/Sensor 2 -Grid/RangeMin 1.0 -Reg/Force3DoF true -Reg/Strategy 2 -Grid/MaxObstacleHeight 2.0 -Grid/RayTracing true',
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

    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_nav2_plugins_launch_file_path))

# removed -Grid/NoiseFilteringRadius 0.1 -Grid/NoiseFilteringMinNeighbors 5
    return LaunchDescription([
        lidar_launch,
        camera_launch,
        sync_camera,
        rtabmap_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        robot_localization_node,
        motor_controller_node,
        wheel_odom_publisher,
        map_to_odom_tf,
        map_to_base_link_tf,
        pose_publisher,
        laser_odom,
        imu_filter,
        imu_rotator,
])