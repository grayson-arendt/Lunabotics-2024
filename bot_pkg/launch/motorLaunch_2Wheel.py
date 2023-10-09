import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name="PhoenixContainer",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonFX",
                name="front_left",
                parameters=[{"id": 15, "invert": "true"}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonFX",
                name="front_right",
                parameters=[{"id": 18}]
            ),
        ],
        output="screen",
    )

    drive = Node(
    package='bot_pkg',
    executable='4wd',
    name = "drive")

    joy = Node(
    package='joy',
    executable='joy_node',
    name = "joy")

    return launch.LaunchDescription([container,drive,joy])