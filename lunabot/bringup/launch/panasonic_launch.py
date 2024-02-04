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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events.process import ProcessStarted
from launch.launch_context import LaunchContext
from launch.event_handlers.on_process_start import OnProcessStart
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('robot_description'), 'urdf', 'test_bot.xacro')
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str
    )

    lidar1_launch_path = os.path.join(get_package_share_path('sllidar_ros2'), 'launch', 'sllidar_a3_launch.py')
    lidar2_launch_path = os.path.join(get_package_share_path('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
    d455_launch_path = os.path.join(get_package_share_path('realsense2_camera'), 'launch', 'rs_launch.py')
    slam_toolbox_launch_file_path = os.path.join(get_package_share_path('slam_toolbox'), 'launch', 'online_async_launch.py')
    rtabmap_launch_path = os.path.join(get_package_share_path('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
    foxglove_launch_path = os.path.join(get_package_share_path('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
 
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
    
    imu_filter = Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'fixed_frame': 'odom'},
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'publish_tf': True},
                ],
    )

    imu_rotator = Node(
        package='autonomous_pkg',
        executable='imu_rotator'
    )

    motor_controller_node = Node(
        package='autonomous_pkg',
        executable='motor_controller'
    )

    wheel_imu_odometry = Node(
        package="autonomous_pkg",
        executable="wheel_imu_odometry"
    )

    odometry_transform = Node(
        package="autonomous_pkg",
        executable="odometry_transform"
    )

    lidar1_odom = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan1',
                    'odom_topic' : '/odom_lidar1',
                    'publish_tf' : False,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],
    )

      
    lidar2_odom = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan2',
                    'odom_topic' : '/odom_lidar2',
                    'publish_tf' : False,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],
    )


    particle_filter = Node(
                package="autonomous_pkg",
                executable="particle_filter",
    )

    slam_toolbox_launch = GroupAction(actions=[
            SetRemap(src='/map',dst='/slam_toolbox/map'),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file_path))])

    lidar1_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar1_launch_path))
    
    lidar2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar2_launch_path))

    foxglove_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(foxglove_launch_path))

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
            'args': '-d -Grid/Sensor 1 -Grid/RangeMin 1.0 -Reg/Force3DoF true -Reg/Strategy 1 -Grid/MaxObstacleHeight 2.0 -Grid/RayTracing true',
            'rgb_topic':'/camera/color/image_raw',
            'depth_topic':'/camera/depth/image_rect_raw',
            'camera_info_topic':'/camera/depth/camera_info',
            'subscribe_rgb': 'true',
            'subscribe_depth': 'true',
            'subscribe_scan':'true',
            'scan_topic':'/scan1',
            'approx_sync': 'true',
            'rviz': 'false',
            'queue_size': '300', 
        }.items(),)])

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        map_to_odom_tf,
        motor_controller_node,
        wheel_imu_odometry,
        lidar1_odom,
        particle_filter,
        lidar1_launch,
        foxglove_launch,
        camera_launch,
        rtabmap_launch,
    ])