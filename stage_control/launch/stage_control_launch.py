from launch import LaunchDescription, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, LaunchConfiguration

# Launch stage control action servers and hardware controls


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_level",
            default_value="0",
            description="Simulation level: 0 - Emulation only, " +
                "1 - virtual only, 2 - hardware only, 3 - virtual and hardware"
        ),
        Node(
            package="stage_control",
            executable="stage_control_node",
            name="stage_control_node",
            output="screen",
            parameters=[
                {"sim_level": LaunchConfiguration('sim_level')}
            ]
        ),
        Node(
            package="stage_control",
            executable="stage_hardware_node",
            name="stage_hardware_node",
            output="screen",
            condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level'), " == 2 or ", 
               LaunchConfiguration('sim_level'), " == 3"]))
        ),
        Node(
            package="stage_control",
            executable="stage_virtual_node",
            name="stage_virtual_node",
            output="screen",
            condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level'), " == 1 or ", 
               LaunchConfiguration('sim_level'), " == 3"]))
        ),
        Node(
            package="stage_control",
            executable="stage_emulated_node",
            name="stage_emulated_node",
            output="screen",
            condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level'), " == 0"]))
        ),
    ])
