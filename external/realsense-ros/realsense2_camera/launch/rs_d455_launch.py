"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'device_type', 'default': 'd455', 'description': 'choose device by type'},
                    {'name': 'enable_pointcloud',            'default': 'false', 'description': 'enable pointcloud'},
                    {'name': 'unite_imu_method',             'default': 'linear_interpolation', 'description': '[copy|linear_interpolation]'},
                    {'name': 'color_width',                  'default': '424', 'description': 'color image width'},
                    {'name': 'color_height',                 'default': '240', 'description': 'color image height'},
                    {'name': 'depth_width',                  'default': '640', 'description': 'color image width'},
                    {'name': 'depth_height',                 'default': '480', 'description': 'color image height'},
                    {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'depth_fps',                    'default': '5.0', 'description': ''},
                    {'name': 'color_fps',                    'default': '5.0', 'description': ''},
                    {'name': 'enable_gyro',                  'default': 'true', 'description': ''},
                    {'name': 'enable_accel',                 'default': 'true', 'description': ''},
                    {'name': 'enable_sync',                  'default': 'false', 'description': ''},
                    {'name': 'initial_reset',                'default': 'true', 'description': ''},
                    {'name': 'align_depth',                'default': 'true', 'description': ''},
                   ]

def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),
    ])
