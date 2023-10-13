from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    remap_joy = ("joy", "controller_input") #changes the joy topic name to controller_input
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        remappings=[
            remap_joy
        ]
    )
    
    manual_control_node = Node(
        package="manual_control_pkg",
        executable="manual_control"
    )
    
    ld.add_action(joy_node)
    ld.add_action(manual_control_node)
    return ld
