from launch import LaunchDescription, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Launch stage control action servers and hardware controls


def generate_launch_description():

    hardware_config = os.path.join(
        get_package_share_directory('stage_control'),
        'config',
        'hardware_config.yaml'
        )
    virtual_config = os.path.join(
        get_package_share_directory('stage_control'),
        'config',
        'virtual_config.yaml'
        )
    stage_config = os.path.join(
        get_package_share_directory('stage_control'),
        'config',
        'stage_config.yaml'
        )

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
                {"sim_level": LaunchConfiguration('sim_level')},
                stage_config
            ]
        ),
        Node(
            package="stage_control",
            executable="stage_hardware_node",
            name="stage_hardware_node",
            output="screen",
            condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level'), " == 2 or ", 
               LaunchConfiguration('sim_level'), " == 3"])),
            parameters=[hardware_config]
        ),
        Node(
            package="stage_control",
            executable="stage_virtual_node",
            name="stage_virtual_node",
            output="screen",
            condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level'), " == 1 or ", 
               LaunchConfiguration('sim_level'), " == 3"])),
            parameters=[virtual_config]
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
