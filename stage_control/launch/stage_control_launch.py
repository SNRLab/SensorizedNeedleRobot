from launch import LaunchDescription
from launch_ros.actions import Node

# Launch stage control action servers and hardware controls

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stage_control",
            executable="stage_control_node",
            name="stage_control_node",
            output="screen"
        ),
        Node(
            package="stage_control",
            executable="stage_hardware_node",
            name="stage_hardware_node",
            output="screen"
        ),
    ])